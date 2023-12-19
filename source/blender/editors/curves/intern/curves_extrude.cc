/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute.hh"
#include "BKE_context.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "ED_curves.hh"

namespace blender::ed::curves {

struct CurveCopy {
  int *intervals;
  int count;
  bool is_first_selected;
  int points_in_curve;
};

static void init_curve(
    CurveCopy &copy, int &ins, int &prev_i, const int i, const int first_elem, int *intervals)
{
  copy.is_first_selected = i == first_elem;
  copy.intervals = intervals;
  copy.intervals[0] = first_elem;
  prev_i = INT_MIN;
  ins = copy.is_first_selected ? -1 : 0;
}

static int finish_curve_or_shallow_copy(CurveCopy &copy,
                                        int ins,
                                        const int prev_i,
                                        const Span<int> &offsets,
                                        const int curve_index,
                                        int *&intervals)
{
  const int last = offsets[curve_index + 1] - 1;
  // shallow copy if previous selected point vas not on this curve
  if (prev_i < offsets[curve_index]) {
    const int first = offsets[curve_index];

    copy.count = 1;
    copy.intervals = intervals;
    intervals[0] = first;
    intervals[1] = last;
    copy.is_first_selected = false;
  }
  else {
    if ((copy.intervals[ins] != last) || copy.intervals[ins - 1] != copy.intervals[ins]) {
      ins++;
      copy.intervals[ins] = last;
    }
    // check for one point extrusion
    if (copy.is_first_selected && ins == 1) {
      ins++;
      copy.intervals[ins] = copy.intervals[ins - 1];
      copy.is_first_selected = false;
    }
    copy.count = ins;
  }

  copy.points_in_curve = 0;
  for (const int i : IndexRange(copy.count)) {
    copy.points_in_curve += copy.intervals[i + 1] - copy.intervals[i] + 1;
  }
  intervals += copy.count + 1;
  return copy.points_in_curve;
}

static int sel_to_copy_ints(const IndexMask selection,
                            const Span<int> &offsets,
                            int *all_copy_intervals,
                            CurveCopy *copies)
{
  int curve_index = 0;
  int *copy_intervals = all_copy_intervals;
  int total_points = 0;

  int prev_i;
  int ins;
  CurveCopy *copy = copies;
  init_curve(*copy, ins, prev_i, selection.first(), offsets[0], copy_intervals);

  selection.foreach_index([&](const int i) {
    // if selected point not in current curve
    if (i >= offsets[curve_index + 1]) {
      do {
        total_points += finish_curve_or_shallow_copy(
            *copy, ins, prev_i, offsets, curve_index, copy_intervals);
        curve_index++;
        copy++;
      } while (i >= offsets[curve_index + 1]);
      init_curve(*copy, ins, prev_i, i, offsets[curve_index], copy_intervals);
    }

    /* new selected region started */
    if (prev_i + 1 != i) {
      ins++;
      copy->intervals[ins] = i;
      ins++;
    }
    else if (ins == 1 && copy->is_first_selected) {
      ins++;
      copy->is_first_selected = false;
    }
    copy->intervals[ins] = i;
    prev_i = i;
  });
  do {
    total_points += finish_curve_or_shallow_copy(
        *copy, ins, prev_i, offsets, curve_index, copy_intervals);
    curve_index++;
    copy++;
  } while (curve_index < offsets.size() - 1);
  return total_points;
}

static void copy_attributes(bke::MutableAttributeAccessor src,
                            bke::MutableAttributeAccessor dst,
                            int src_i,
                            int dst_i,
                            int size)
{
  src.for_all([&](const bke::AttributeIDRef &id, const bke::AttributeMetaData &meta_data) {
    if (meta_data.domain == ATTR_DOMAIN_POINT && std::string(".selection") != id.name()) {
      bke::GSpanAttributeWriter src_attribute = src.lookup_for_write_span(id);
      const CPPType &type = src_attribute.span.type();
      bke::GSpanAttributeWriter dest_attribute = dst.lookup_or_add_for_write_only_span(
          id, meta_data.domain, meta_data.data_type);
      GMutableSpan dst = dest_attribute.span.slice(IndexRange(dst_i, size));

      GMutableSpan src = src_attribute.span.slice(IndexRange(src_i, size));
      type.copy_assign_n(src.data(), dst.data(), dst.size());
      dest_attribute.finish();
    }
    return true;
  });
}

static int curves_extrude_exec(bContext *C, wmOperator *op)
{
  Object *obedit = CTX_data_edit_object(C);
  Curves *curves_id = static_cast<Curves *>(obedit->data);
  const eAttrDomain selection_domain = eAttrDomain(curves_id->selection_domain);
  if (selection_domain == ATTR_DOMAIN_POINT) {

    IndexMaskMemory memory;
    const IndexMask selection = retrieve_selected_points(*curves_id, memory);
    if (!selection.is_empty()) {
      bke::CurvesGeometry &curves = curves_id->geometry.wrap();

      const int max = selection.size() * 2 + curves.curves_num() * 2;
      int *const all_copy_intervals = static_cast<int *>(
          MEM_malloc_arrayN(max + 2, sizeof(int), __func__));

      CurveCopy *const curve_copies = static_cast<CurveCopy *>(
          MEM_malloc_arrayN(curves.curves_num(), sizeof(CurveCopy), __func__));

      const Span<int> old_offsets = curves.offsets();
      const int new_points_count = sel_to_copy_ints(
          selection, old_offsets, all_copy_intervals, curve_copies);

      bke::CurvesGeometry new_curves(new_points_count, curves.curves_num());
      MutableSpan<int> offsets = new_curves.offsets_for_write();
      const MutableSpan<int8_t> new_types = new_curves.curve_types_for_write();
      const VArray<int8_t> old_types = curves.curve_types();

      CustomData_copy(
          &curves.curve_data, &new_curves.curve_data, CD_MASK_ALL, new_curves.curves_num());
      new_curves.update_curve_types();

      int new_offset = old_offsets[0];
      offsets[0] = new_offset;
      for (const int c : curves.curves_range()) {
        new_offset += curve_copies[c].points_in_curve;
        offsets[c + 1] = new_offset;
      }

      bke::MutableAttributeAccessor new_attributes = new_curves.attributes_for_write();
      bke::MutableAttributeAccessor old_attributes = curves.attributes_for_write();

      int d = 0;

      bke::GSpanAttributeWriter selection = ensure_selection_attribute(
          new_curves, selection_domain, CD_PROP_BOOL);
      for (const int c : curves.curves_range()) {
        CurveCopy &copy = curve_copies[c];
        bool is_selected = copy.is_first_selected;
        for (const int i : IndexRange(copy.count)) {
          const int first = copy.intervals[i];
          const int last = copy.intervals[i + 1];
          const int size = last - first + 1;

          copy_attributes(old_attributes, new_attributes, first, d, size);

          GMutableSpan selection_span = selection.span.slice(IndexRange(d, size));
          fill_selection(selection_span, is_selected);

          d += size;
          is_selected = !is_selected;
        }
      }
      selection.finish();

      MEM_freeN(curve_copies);
      MEM_freeN(all_copy_intervals);

      curves_id->geometry.wrap() = new_curves;
    }
  }

  return OPERATOR_FINISHED;
}

void CURVES_OT_extrude(wmOperatorType *ot)
{
  ot->name = "Extrude";
  ot->idname = __func__;

  ot->exec = curves_extrude_exec;
  ot->poll = editable_curves_in_edit_mode_poll;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  /* properties */
  PropertyRNA *prop;
}

}  // namespace blender::ed::curves
