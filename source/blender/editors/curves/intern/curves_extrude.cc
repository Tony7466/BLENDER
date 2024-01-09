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
      : curve_intervals(curve_intervals_size),
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
static Span<int> compress_intervals(const blender::Array<IndexRange> &curve_interval_ranges,
                                    blender::Array<int> &intervals)
{
  const int *src = intervals.data();
  /* Skip the first curve, as all the data stays in the same place. */
  int *dst = intervals.data() + curve_interval_ranges[0].size();

  for (const int c : IndexRange(1, curve_interval_ranges.size() - 1)) {
    const IndexRange cr = curve_interval_ranges[c];
    const int width = cr.size() - 1;
    std::copy_n(src + cr.first() + 1, width, dst);
    dst += width;
  }
  (*dst) = src[curve_interval_ranges[curve_interval_ranges.size() - 1].last() + 1];
  return {intervals.data(), dst - intervals.data() + 1};
}

/**
 * Creates copy intervals for selection #range in the context of #curve_index.
 * If part of the #range is outside given curve, slices it and returns false indicating remaining
 * still needs to be handled. If whole #range was handled returns true.
 */
static bool handle_range(const int curve_index,
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
static void calc_curve_offset(const int curve_index,
                              int &interval_offset,
                              const Span<int> offsets,
                              MutableSpan<int> new_offsets,
                              CurvesCopy &extr)
{
  const int points_in_curve = (offsets[curve_index + 1] - offsets[curve_index] +
                               extr.curve_interval_ranges[curve_index].size() - 1);
  new_offsets[curve_index + 1] = new_offsets[curve_index] + points_in_curve;
  interval_offset += extr.curve_interval_ranges[curve_index].size() + 1;
}

static void finish_curve(int &curve_index,
                         int &interval_offset,
                         int ins,
                         int last_elem,
                         const Span<int> offsets,
                         MutableSpan<int> new_offsets,
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
  calc_curve_offset(curve_index, interval_offset, offsets, new_offsets, extr);
  curve_index++;
}

static void finish_curve_or_full_copy(int &curve_index,
                                      int &interval_offset,
                                      int ins,
                                      const std::optional<IndexRange> prev_range,
                                      const Span<int> offsets,
                                      MutableSpan<int> new_offsets,
                                      CurvesCopy &extr)
{
  const int last = offsets[curve_index + 1] - 1;

  if (prev_range.has_value() && prev_range.value().last() >= offsets[curve_index]) {
    finish_curve(curve_index, interval_offset, ins, last, offsets, new_offsets, extr);
  }
  else {
    /* Copy full curve if previous selected point vas not on this curve. */
    const int first = offsets[curve_index];
    extr.curve_interval_ranges[curve_index] = IndexRange(interval_offset, 1);
    extr.is_first_selected[curve_index] = false;
    extr.curve_intervals[interval_offset] = first;
    extr.curve_intervals[interval_offset + 1] = last;
    calc_curve_offset(curve_index, interval_offset, offsets, new_offsets, extr);
    curve_index++;
  }
}

/**
 * Creates and fills #CurvesCopy for extrusion of selected points.
 *
 * \param offsets: offsets of original curve.
 * \param selection: selected points.
 */
static const CurvesCopy calc_curves_extrusion(const Span<int> offsets,
                                              const IndexMask &selection,
                                              MutableSpan<int> new_offsets)
{
  const int curve_num = offsets.size() - 1;
  CurvesCopy extr(curve_num, selection.size() * 2 + curve_num * 2);
  std::optional<IndexRange> prev_range;
  int ins = 0;

  int curve_index = 0;
  int interval_offset = 0;
  extr.curve_intervals[interval_offset] = offsets[0];
  new_offsets[0] = offsets[0];

  selection.foreach_range([&](const IndexRange range) {
    /* Beginning of the range outside current curve. */
    if (range.first() > offsets[curve_index + 1] - 1) {
      do {
        finish_curve_or_full_copy(
            curve_index, interval_offset, ins, prev_range, offsets, new_offsets, extr);
      } while (range.first() > offsets[curve_index + 1] - 1);
      ins = 0;
      extr.curve_intervals[interval_offset] = offsets[curve_index];
    }

    IndexRange range_to_handle = range;
    while (!handle_range(curve_index, interval_offset, ins, range_to_handle, offsets, extr)) {
      finish_curve(curve_index,
                   interval_offset,
                   ins,
                   offsets[curve_index + 1] - 1,
                   offsets,
                   new_offsets,
                   extr);
      ins = 0;
      extr.curve_intervals[interval_offset] = offsets[curve_index];
    }
    prev_range = range;
  });

  do {
    finish_curve_or_full_copy(
        curve_index, interval_offset, ins, prev_range, offsets, new_offsets, extr);
    prev_range.reset();
  } while (curve_index < offsets.size() - 1);
  return extr;
}

static void curves_obj_extrude(Curves *curves_id)
{
  const bke::AttrDomain selection_domain = bke::AttrDomain(curves_id->selection_domain);
  if (selection_domain != bke::AttrDomain::Point) {
    return;
  }

  IndexMaskMemory memory;
  const IndexMask extruded_points = retrieve_selected_points(*curves_id, memory);
  if (extruded_points.is_empty()) {
    return;
  }

  const bke::CurvesGeometry &curves = curves_id->geometry.wrap();

  const Span<int> old_offsets = curves.offsets();
  bke::CurvesGeometry new_curves = bke::curves::copy_only_curve_domain(curves);
  new_curves.resize(0, curves.curves_num());
  MutableSpan<int> new_offsets = new_curves.offsets_for_write();

  CurvesCopy curves_copy = calc_curves_extrusion(old_offsets, extruded_points, new_offsets);
  new_curves.resize(new_offsets.last(), curves.curves_num());

  const Span<int> curve_intervals = {curves_copy.curve_intervals.data(),
                                     curves_copy.curve_intervals.size()};

  bke::GSpanAttributeWriter selection = ensure_selection_attribute(
      new_curves, bke::AttrDomain::Point, CD_PROP_BOOL);

  threading::parallel_for(curves.curves_range(), 256, [&](IndexRange curves_range) {
    for (const int curve : curves_range) {
      const int first_index = curves_copy.curve_interval_ranges[curve].start();
      const int first_value = curve_intervals[first_index];
      bool is_selected = curves_copy.is_first_selected[curve];

      for (const int i : curves_copy.curve_interval_ranges[curve]) {
        const int dest_index = new_offsets[curve] + curve_intervals[i] - first_value + i -
                               first_index;
        const int size = curve_intervals[i + 1] - curve_intervals[i] + 1;
        GMutableSpan selection_span = selection.span.slice(IndexRange(dest_index, size));
        fill_selection(selection_span, is_selected);

        is_selected = !is_selected;
      }
    }
  });
  selection.finish();

  const Span<int> intervals = compress_intervals(curves_copy.curve_interval_ranges,
                                                 curves_copy.curve_intervals);

  bke::MutableAttributeAccessor dst_attributes = new_curves.attributes_for_write();
  const bke::AttributeAccessor src_attributes = curves.attributes();
  for (auto &attribute : bke::retrieve_attributes_for_transfer(
           src_attributes, dst_attributes, ATTR_DOMAIN_MASK_POINT, {}, {".selection"}))
  {
    const CPPType &type = attribute.src.type();
    threading::parallel_for(IndexRange(intervals.size() - 1), 512, [&](IndexRange range) {
      for (const int i : range) {
        const int first = intervals[i];
        const int size = intervals[i + 1] - first + 1;
        const int dest_index = intervals[i] + i;
        type.copy_assign_n(attribute.src.slice(IndexRange(first, size)).data(),
                           attribute.dst.span.slice(IndexRange(dest_index, size)).data(),
                           size);
      }
    });
    attribute.dst.finish();
  }
  curves_id->geometry.wrap() = std::move(new_curves);
}

static int curves_extrude_exec(bContext *C, wmOperator * /*op*/)
{
  for (Curves *curves_id : get_unique_editable_curves(*C)) {
    curves_obj_extrude(curves_id);
  }
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
