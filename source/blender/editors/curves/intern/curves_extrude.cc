/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute.hh"
#include "BKE_context.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "ED_curves.hh"

namespace blender::ed::curves {

struct CurvesCopy {
  blender::Array<IndexRange> copy_intervals;
  blender::Array<bool> is_first_selected;
  blender::Array<int> points_in_curve;

  CurvesCopy(const int curve_num)
      : copy_intervals(curve_num), is_first_selected(curve_num), points_in_curve(curve_num)
  {
  }
};

static void init_curve(CurvesCopy &curves_copy,
                       const int curve_index,
                       int &ins,
                       int &prev_i,
                       const int i,
                       const int first_elem,
                       int *const intervals,
                       const int first_interval)
{
  curves_copy.is_first_selected[curve_index] = i == first_elem;
  ins = curves_copy.is_first_selected[curve_index] ? -1 : 0;
  intervals[first_interval] = first_elem;
  prev_i = INT_MIN;
}

static int finish_curve_or_shallow_copy(CurvesCopy &curves_copy,
                                        int &curve_index,
                                        int ins,
                                        const int prev_i,
                                        const Span<int> offsets,
                                        int *const intervals,
                                        int &interval_offset)
{
  const int last = offsets[curve_index + 1] - 1;
  // shallow copy if previous selected point vas not on this curve
  if (prev_i < offsets[curve_index]) {
    const int first = offsets[curve_index];
    curves_copy.copy_intervals[curve_index] = IndexRange(interval_offset, 1);
    curves_copy.is_first_selected[curve_index] = false;
    intervals[interval_offset] = first;
    intervals[interval_offset + 1] = last;
  }
  else {
    if ((intervals[interval_offset + ins] != last) ||
        intervals[interval_offset + ins - 1] != intervals[interval_offset + ins])
    {
      ins++;
      intervals[interval_offset + ins] = last;
    }
    // check for one point extrusion
    if (curves_copy.is_first_selected[curve_index] && ins == 1) {
      ins++;
      intervals[interval_offset + ins] = intervals[interval_offset + ins - 1];
      curves_copy.is_first_selected[curve_index] = false;
    }
    curves_copy.copy_intervals[curve_index] = IndexRange(interval_offset, ins);
  }

  curves_copy.points_in_curve[curve_index] = 0;
  for (const int i : curves_copy.copy_intervals[curve_index]) {
    curves_copy.points_in_curve[curve_index] += intervals[i + 1] - intervals[i] + 1;
  }
  interval_offset += curves_copy.copy_intervals[curve_index].size() + 1;
  curve_index++;
  return curves_copy.points_in_curve[curve_index - 1];
}

static int sel_to_copy_ints(const IndexMask &selection,
                            const Span<int> offsets,
                            blender::Array<int> &all_copy_intervals,
                            CurvesCopy &curves_copy)
{
  int curve_index = 0;
  int *copy_intervals = all_copy_intervals.data();
  int interval_offset = 0;
  int total_points = 0;

  int prev_i;
  int ins;

  init_curve(curves_copy,
             curve_index,
             ins,
             prev_i,
             selection.first(),
             offsets[0],
             copy_intervals,
             interval_offset);

  selection.foreach_index([&](const int i) {
    // if selected point not in current curve
    if (i >= offsets[curve_index + 1]) {
      do {
        total_points += finish_curve_or_shallow_copy(
            curves_copy, curve_index, ins, prev_i, offsets, copy_intervals, interval_offset);
      } while (i >= offsets[curve_index + 1]);
      init_curve(curves_copy,
                 curve_index,
                 ins,
                 prev_i,
                 i,
                 offsets[curve_index],
                 copy_intervals,
                 interval_offset);
    }

    /* new selected region started */
    if (prev_i + 1 != i) {
      ins++;
      copy_intervals[interval_offset + ins] = i;
      ins++;
    }
    else if (ins == 1 && curves_copy.is_first_selected[curve_index]) {
      ins++;
      curves_copy.is_first_selected[curve_index] = false;
    }
    copy_intervals[interval_offset + ins] = i;
    prev_i = i;
  });
  do {
    total_points += finish_curve_or_shallow_copy(
        curves_copy, curve_index, ins, prev_i, offsets, copy_intervals, interval_offset);
  } while (curve_index < offsets.size() - 1);
  return total_points;
}

static int curves_extrude_exec(bContext *C, wmOperator *op)
{
  Object *obedit = CTX_data_edit_object(C);
  Curves *curves_id = static_cast<Curves *>(obedit->data);
  const eAttrDomain selection_domain = eAttrDomain(curves_id->selection_domain);
  if (selection_domain != ATTR_DOMAIN_POINT) {
    return OPERATOR_FINISHED;
  }

  IndexMaskMemory memory;
  const IndexMask extruded_points = retrieve_selected_points(*curves_id, memory);
  if (extruded_points.is_empty()) {
    return OPERATOR_FINISHED;
  }

  bke::CurvesGeometry &curves = curves_id->geometry.wrap();

  const int max = extruded_points.size() * 2 + curves.curves_num() * 2;
  blender::Array<int> all_copy_intervals(max);

  CurvesCopy curves_copy(curves.curves_num());

  const Span<int> old_offsets = curves.offsets();
  const int new_points_count = sel_to_copy_ints(
      extruded_points, old_offsets, all_copy_intervals, curves_copy);

  bke::CurvesGeometry new_curves(new_points_count, curves.curves_num());
  MutableSpan<int> offsets = new_curves.offsets_for_write();

  CustomData_copy(
      &curves.curve_data, &new_curves.curve_data, CD_MASK_ALL, new_curves.curves_num());
  new_curves.update_curve_types();

  int new_offset = old_offsets[0];
  offsets[0] = new_offset;
  for (const int c : curves.curves_range()) {
    new_offset += curves_copy.points_in_curve[c];
    offsets[c + 1] = new_offset;
  }

  int d = 0;
  bke::GSpanAttributeWriter selection = ensure_selection_attribute(
      new_curves, selection_domain, CD_PROP_BOOL);
  for (const int c : curves.curves_range()) {
    bool is_selected = curves_copy.is_first_selected[c];
    for (const int i : curves_copy.copy_intervals[c]) {
      const int first = all_copy_intervals[i];
      const int last = all_copy_intervals[i + 1];
      const int size = last - first + 1;
      GMutableSpan selection_span = selection.span.slice(IndexRange(d, size));
      fill_selection(selection_span, is_selected);

      d += size;
      is_selected = !is_selected;
    }
  }
  selection.finish();

  bke::MutableAttributeAccessor dst_attributes = new_curves.attributes_for_write();
  bke::AttributeAccessor src_attributes = curves.attributes();
  src_attributes.for_all([&](const bke::AttributeIDRef &id,
                             const bke::AttributeMetaData &meta_data) {
    if (meta_data.domain == ATTR_DOMAIN_POINT && std::string(".selection") != id.name()) {
      bke::GSpanAttributeWriter dst_attribute = dst_attributes.lookup_or_add_for_write_only_span(
          id, meta_data.domain, meta_data.data_type);
      GMutableSpan dst = dst_attribute.span;
      const CPPType &type = dst.type();
      const GVArraySpan src = (*src_attributes.lookup(id, meta_data.domain));
      d = 0;
      for (const int c : curves.curves_range()) {
        for (const int i : curves_copy.copy_intervals[c]) {
          const int first = all_copy_intervals[i];
          const int last = all_copy_intervals[i + 1];
          const int size = last - first + 1;

          type.copy_assign_n(src.slice(IndexRange(first, size)).data(),
                             dst.slice(IndexRange(d, size)).data(),
                             size);
          d += size;
        }
      }
      dst_attribute.finish();
    }
    return true;
  });

  curves_id->geometry.wrap() = new_curves;

  return OPERATOR_FINISHED;
}

void CURVES_OT_extrude(wmOperatorType *ot)
{
  ot->name = "Extrude";
  ot->description = "Extrude selected control point(s)";
  ot->idname = __func__;

  ot->exec = curves_extrude_exec;
  ot->poll = editable_curves_in_edit_mode_poll;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  /* properties */
  PropertyRNA *prop;
}

}  // namespace blender::ed::curves
