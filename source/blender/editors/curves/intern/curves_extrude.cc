/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute.hh"
#include "BKE_context.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "ED_curves.hh"

namespace blender::ed::curves {

class CurvesExtrusion {
  int curve_index_;
  int interval_offset_;
  blender::Array<int> curve_offsets_;

  bool handle_range(int &ins, IndexRange &range, const Span<int> offsets);

  void calc_curve_offset();

  void finish_curve(int ins, int last_elem);

  void finish_curve_or_shallow_copy(int ins,
                                    const std::optional<IndexRange> prev_range,
                                    const Span<int> offsets);

  CurvesExtrusion(const Span<int> offsets, const IndexMask &selection, const int curve_num);

 public:
  blender::Array<IndexRange> curve_intervals;
  blender::Array<int> intervals;
  blender::Array<bool> is_first_selected;

  CurvesExtrusion(const Span<int> offsets, const IndexMask &selection)
      : CurvesExtrusion(offsets, selection, offsets.size() - 1)
  {
  }

  Span<int> curve_offsets() const
  {
    return {curve_offsets_.data(), curve_offsets_.size()};
  }
};

CurvesExtrusion::CurvesExtrusion(const Span<int> offsets,
                                 const IndexMask &selection,
                                 const int curve_num)
    : curve_intervals(curve_num),
      is_first_selected(curve_num),
      curve_offsets_(curve_num + 1),
      intervals(selection.size() * 2 + curve_num * 2)
{
  std::optional<IndexRange> prev_range;
  int ins = 0;

  curve_index_ = 0;
  interval_offset_ = 0;
  intervals[interval_offset_] = offsets[0];
  curve_offsets_[0] = offsets[0];

  selection.foreach_range([&](const IndexRange range) {
    // beginning of the range outside current curve
    if (range.first() > offsets[curve_index_ + 1] - 1) {
      do {
        finish_curve_or_shallow_copy(ins, prev_range, offsets);
      } while (range.first() > offsets[curve_index_ + 1] - 1);
      ins = 0;
      intervals[interval_offset_] = offsets[curve_index_];
    }

    IndexRange range_to_handle = range;
    while (!handle_range(ins, range_to_handle, offsets)) {
      finish_curve(ins, offsets[curve_index_ + 1] - 1);
      ins = 0;
      intervals[interval_offset_] = offsets[curve_index_];
    }
    prev_range = range;
  });

  do {
    finish_curve_or_shallow_copy(ins, prev_range, offsets);
    prev_range.reset();
  } while (curve_index_ < offsets.size() - 1);
}

bool CurvesExtrusion::handle_range(int &ins, IndexRange &range, const Span<int> offsets)
{
  const int first_elem = offsets[curve_index_];
  const int last_elem = offsets[curve_index_ + 1] - 1;

  if (ins == 0) {
    is_first_selected[curve_index_] = range.first() == first_elem && range.size() == 1;
    if (!is_first_selected[curve_index_]) {
      ins++;
    }
  }
  intervals[interval_offset_ + ins] = range.first();
  ins++;

  bool inside_curve = last_elem >= range.last();
  if (inside_curve) {
    intervals[interval_offset_ + ins] = range.last();
  }
  else {
    intervals[interval_offset_ + ins] = last_elem;
    range = IndexRange(last_elem + 1, range.last() - last_elem);
  }
  ins++;
  return inside_curve;
}

void CurvesExtrusion::calc_curve_offset()
{
  int points_in_curve = 0;
  for (const int i : curve_intervals[curve_index_]) {
    points_in_curve += intervals[i + 1] - intervals[i] + 1;
  }
  interval_offset_ += curve_intervals[curve_index_].size() + 1;
  curve_offsets_[curve_index_ + 1] = curve_offsets_[curve_index_] + points_in_curve;
}

void CurvesExtrusion::finish_curve(int ins, int last_elem)
{
  if (intervals[interval_offset_ + ins - 1] != last_elem ||
      intervals[interval_offset_ + ins - 2] != intervals[interval_offset_ + ins - 1])
  {
    intervals[interval_offset_ + ins] = last_elem;
    ins++;
  }
  // check for extrusion from one point
  else if (is_first_selected[curve_index_] && ins == 2) {
    intervals[interval_offset_ + ins] = intervals[interval_offset_ + ins - 1];
    is_first_selected[curve_index_] = false;
    ins++;
  }
  curve_intervals[curve_index_] = IndexRange(interval_offset_, ins - 1);
  calc_curve_offset();
  curve_index_++;
}

void CurvesExtrusion::finish_curve_or_shallow_copy(int ins,
                                                   const std::optional<IndexRange> prev_range,
                                                   const Span<int> offsets)
{
  const int last = offsets[curve_index_ + 1] - 1;

  if (prev_range.has_value() && prev_range.value().last() >= offsets[curve_index_]) {
    finish_curve(ins, last);
  }
  // shallow copy if previous selected point vas not on this curve
  else {
    const int first = offsets[curve_index_];
    curve_intervals[curve_index_] = IndexRange(interval_offset_, 1);
    is_first_selected[curve_index_] = false;
    intervals[interval_offset_] = first;
    intervals[interval_offset_ + 1] = last;
    calc_curve_offset();
    curve_index_++;
  }
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

  const Span<int> old_offsets = curves.offsets();
  CurvesExtrusion curves_copy(old_offsets, extruded_points);

  bke::CurvesGeometry new_curves(curves_copy.curve_offsets().last(), curves.curves_num());
  MutableSpan<int> offsets = new_curves.offsets_for_write();

  CustomData_copy(
      &curves.curve_data, &new_curves.curve_data, CD_MASK_ALL, new_curves.curves_num());
  new_curves.update_curve_types();

  offsets.copy_from(curves_copy.curve_offsets());

  int d = 0;
  bke::GSpanAttributeWriter selection = ensure_selection_attribute(
      new_curves, selection_domain, CD_PROP_BOOL);
  for (const int c : curves.curves_range()) {
    bool is_selected = curves_copy.is_first_selected[c];
    for (const int i : curves_copy.curve_intervals[c]) {
      const int first = curves_copy.intervals[i];
      const int last = curves_copy.intervals[i + 1];
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
        for (const int i : curves_copy.curve_intervals[c]) {
          const int first = curves_copy.intervals[i];
          const int last = curves_copy.intervals[i + 1];
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
