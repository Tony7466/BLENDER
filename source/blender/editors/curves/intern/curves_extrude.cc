/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute.hh"
#include "BKE_context.hh"
#include "BKE_curves_utils.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "ED_curves.hh"

namespace blender::ed::curves {
/* Stores information need to create new curves by copying data from original ones. */
struct CurvesCopy {
  /* Same semantics as in #CurvesGeometry.curve_offsets. */
  blender::Array<int> curve_offsets;

  /**
   * Buffer for intervals of all curves. Beginning and end of a curve can be determined only by
   * #curve_interval_ranges. For ex. [0, 3, 4, 4, 4] indicates one copy interval for first curve
   * [0, 3] and two for second [4, 4][4, 4]. The first curve will be copied as is without changes,
   * in the second one (consisting only one point - 4) first point will be duplicated (extruded).
   */
  blender::Array<int> curve_intervals;

  /**
   * Points to intervals for each curve in the curve_intervals array.
   * For example above value would be [{0, 1}, {2, 2}]
   */
  blender::Array<IndexRange> curve_interval_ranges;

  /**
   * Per curve boolean indicating if first interval in a curve is selected.
   * Other can be calculated as in a curve two adjacent intervals can have same selection state.
   */
  blender::Array<bool> is_first_selected;

  CurvesCopy(const int curve_num, const int curve_intervals_size)
      : curve_offsets(curve_num + 1),
        curve_intervals(curve_intervals_size),
        curve_interval_ranges(curve_num),
        is_first_selected(curve_num)
  {
  }
};

/**
 * Merges copy intervals at curve endings to minimize number of copy operations.
 * For example above intervals [0, 3, 4, 4, 4] became [0, 4, 4].
 * Leading to only two copy operations.
 */
Span<int> compress_intervals(const blender::Array<IndexRange> &curve_interval_ranges,
                             const Span<int> curve_intervals,
                             blender::Array<int> &compressed)
{
  int *dst = compressed.data();
  const int *src = curve_intervals.data();
  dst[0] = src[0];
  int compact_count = 1;
  for (const int c : IndexRange(curve_interval_ranges.size())) {
    IndexRange cr = curve_interval_ranges[c];
    memmove(dst + compact_count, src + cr.first() + 1, (cr.size() - 1) * sizeof(dst[0]));
    compact_count += cr.size() - 1;
  }
  dst[compact_count] = src[curve_interval_ranges[curve_interval_ranges.size() - 1].last() + 1];
  compact_count++;
  return {dst, compact_count};
}

/**
 * Creates copy intervals for selection #range in the context of #curve_index.
 * If part of the #range is outside given curve, slices it and returns false indicating remaining
 * still needs to be handled. If whole #range was handled returns true.
 */
bool handle_range(const int curve_index,
                  const int interval_offset,
                  int &ins,
                  IndexRange &range,
                  const Span<int> offsets,
                  CurvesCopy &extr)
{
  const int first_elem = offsets[curve_index];
  const int last_elem = offsets[curve_index + 1] - 1;

  if (ins == 0) {
    extr.is_first_selected[curve_index] = range.first() == first_elem && range.size() == 1;
    if (!extr.is_first_selected[curve_index]) {
      ins++;
    }
  }
  extr.curve_intervals[interval_offset + ins] = range.first();
  ins++;

  bool inside_curve = last_elem >= range.last();
  if (inside_curve) {
    extr.curve_intervals[interval_offset + ins] = range.last();
  }
  else {
    extr.curve_intervals[interval_offset + ins] = last_elem;
    range = IndexRange(last_elem + 1, range.last() - last_elem);
  }
  ins++;
  return inside_curve;
}

/**
 * Calculates number of points in resulting curve denoted by #curve_index and sets it's
 * #curve_offsets value.
 */
void calc_curve_offset(const int curve_index,
                       int &interval_offset,
                       const Span<int> offsets,
                       CurvesCopy &extr)
{
  const int points_in_curve = (offsets[curve_index + 1] - offsets[curve_index] +
                               extr.curve_interval_ranges[curve_index].size() - 1);
  extr.curve_offsets[curve_index + 1] = extr.curve_offsets[curve_index] + points_in_curve;
  interval_offset += extr.curve_interval_ranges[curve_index].size() + 1;
}

void finish_curve(int &curve_index,
                  int &interval_offset,
                  int ins,
                  int last_elem,
                  const Span<int> offsets,
                  CurvesCopy &extr)
{
  if (extr.curve_intervals[interval_offset + ins - 1] != last_elem ||
      extr.curve_intervals[interval_offset + ins - 2] !=
          extr.curve_intervals[interval_offset + ins - 1])
  {
    extr.curve_intervals[interval_offset + ins] = last_elem;
    ins++;
  }
  else if (extr.is_first_selected[curve_index] && ins == 2) {
    /* Extrusion from one point. */
    extr.curve_intervals[interval_offset + ins] = extr.curve_intervals[interval_offset + ins - 1];
    extr.is_first_selected[curve_index] = false;
    ins++;
  }
  extr.curve_interval_ranges[curve_index] = IndexRange(interval_offset, ins - 1);
  calc_curve_offset(curve_index, interval_offset, offsets, extr);
  curve_index++;
}

void finish_curve_or_shallow_copy(int &curve_index,
                                  int &interval_offset,
                                  int ins,
                                  const std::optional<IndexRange> prev_range,
                                  const Span<int> offsets,
                                  CurvesCopy &extr)
{
  const int last = offsets[curve_index + 1] - 1;

  if (prev_range.has_value() && prev_range.value().last() >= offsets[curve_index]) {
    finish_curve(curve_index, interval_offset, ins, last, offsets, extr);
  }
  else {
    /* Shallow copy if previous selected point vas not on this curve. */
    const int first = offsets[curve_index];
    extr.curve_interval_ranges[curve_index] = IndexRange(interval_offset, 1);
    extr.is_first_selected[curve_index] = false;
    extr.curve_intervals[interval_offset] = first;
    extr.curve_intervals[interval_offset + 1] = last;
    calc_curve_offset(curve_index, interval_offset, offsets, extr);
    curve_index++;
  }
}

/**
 * Creates and fills #CurvesCopy for extrusion of selected points.
 *
 * \param offsets: offsets of original curve.
 * \param selection: selected points.
 */
const CurvesCopy calc_curves_extrusion(const Span<int> offsets, const IndexMask &selection)
{
  const int curve_num = offsets.size() - 1;
  CurvesCopy extr(curve_num, selection.size() * 2 + curve_num * 2);
  std::optional<IndexRange> prev_range;
  int ins = 0;

  int curve_index = 0;
  int interval_offset = 0;
  extr.curve_intervals[interval_offset] = offsets[0];
  extr.curve_offsets[0] = offsets[0];

  selection.foreach_range([&](const IndexRange range) {
    /* Beginning of the range outside current curve. */
    if (range.first() > offsets[curve_index + 1] - 1) {
      do {
        finish_curve_or_shallow_copy(curve_index, interval_offset, ins, prev_range, offsets, extr);
      } while (range.first() > offsets[curve_index + 1] - 1);
      ins = 0;
      extr.curve_intervals[interval_offset] = offsets[curve_index];
    }

    IndexRange range_to_handle = range;
    while (!handle_range(curve_index, interval_offset, ins, range_to_handle, offsets, extr)) {
      finish_curve(curve_index, interval_offset, ins, offsets[curve_index + 1] - 1, offsets, extr);
      ins = 0;
      extr.curve_intervals[interval_offset] = offsets[curve_index];
    }
    prev_range = range;
  });

  do {
    finish_curve_or_shallow_copy(curve_index, interval_offset, ins, prev_range, offsets, extr);
    prev_range.reset();
  } while (curve_index < offsets.size() - 1);
  return extr;
}

static int curves_extrude_exec(bContext *C, wmOperator * /*op*/)
{
  Object *obedit = CTX_data_edit_object(C);
  Curves *curves_id = static_cast<Curves *>(obedit->data);
  const bke::AttrDomain selection_domain = bke::AttrDomain(curves_id->selection_domain);
  if (selection_domain != bke::AttrDomain::Point) {
    return OPERATOR_FINISHED;
  }

  IndexMaskMemory memory;
  const IndexMask extruded_points = retrieve_selected_points(*curves_id, memory);
  if (extruded_points.is_empty()) {
    return OPERATOR_FINISHED;
  }

  bke::CurvesGeometry &curves = curves_id->geometry.wrap();

  const Span<int> old_offsets = curves.offsets();
  const CurvesCopy curves_copy = calc_curves_extrusion(old_offsets, extruded_points);

  const Span<int> curve_offsets = {curves_copy.curve_offsets.data(),
                                   curves_copy.curve_offsets.size()};

  bke::CurvesGeometry new_curves = bke::curves::copy_only_curve_domain(curves);
  new_curves.resize(curve_offsets.last(), curves.curves_num());

  MutableSpan<int> offsets = new_curves.offsets_for_write();

  offsets.copy_from(curve_offsets);

  const Span<int> curve_intervals = {curves_copy.curve_intervals.data(),
                                     curves_copy.curve_intervals.size()};

  int dest_index = 0;

  bke::GSpanAttributeWriter selection = ensure_selection_attribute(
      new_curves, selection_domain, CD_PROP_BOOL);
  for (const int c : curves.curves_range()) {
    bool is_selected = curves_copy.is_first_selected[c];
    for (const int i : curves_copy.curve_interval_ranges[c]) {
      const int size = curve_intervals[i + 1] - curve_intervals[i] + 1;
      GMutableSpan selection_span = selection.span.slice(IndexRange(dest_index, size));
      fill_selection(selection_span, is_selected);

      dest_index += size;
      is_selected = !is_selected;
    }
  }
  selection.finish();

  blender::Array<int> compact_buffer(curve_intervals.size());
  const Span<int> intervals = compress_intervals(
      curves_copy.curve_interval_ranges, curve_intervals, compact_buffer);

  bke::MutableAttributeAccessor dst_attributes = new_curves.attributes_for_write();
  bke::AttributeAccessor src_attributes = curves.attributes();
  src_attributes.for_all(
      [&](const bke::AttributeIDRef &id, const bke::AttributeMetaData &meta_data) {
        if (meta_data.domain != bke::AttrDomain::Point || std::string(".selection") == id.name()) {
          return true;
        }
        bke::GSpanAttributeWriter dst_attribute = dst_attributes.lookup_or_add_for_write_only_span(
            id, meta_data.domain, meta_data.data_type);
        GMutableSpan dst = dst_attribute.span;
        const CPPType &type = dst.type();
        const GVArraySpan src = (*src_attributes.lookup(id, meta_data.domain));

        int dest_index = 0;
        for (const int i : IndexRange(intervals.size() - 1)) {
          const int first = intervals[i];
          const int size = intervals[i + 1] - first + 1;

          type.copy_assign_n(src.slice(IndexRange(first, size)).data(),
                             dst.slice(IndexRange(dest_index, size)).data(),
                             size);
          dest_index += size;
        }
        dst_attribute.finish();
        return true;
      });
  new_curves.update_curve_types();

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
}

}  // namespace blender::ed::curves
