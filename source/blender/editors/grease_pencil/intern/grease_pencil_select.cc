/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgreasepencil
 */

#include "BKE_grease_pencil.hh"

#include "ED_curves.h"
#include "ED_grease_pencil.h"

namespace blender::ed::greasepencil {

Vector<bool> point_selection_get(const GreasePencilDrawing *drawing)
{
  /* Get point selection in the drawing. */
  bke::CurvesGeometry curves = drawing->geometry.wrap();
  bke::GSpanAttributeWriter selection = ed::curves::ensure_selection_attribute(
      curves, ATTR_DOMAIN_POINT, CD_PROP_BOOL);
  MutableSpan<bool> selection_typed = selection.span.typed<bool>();

  /* Copy selection. */
  Vector<bool> point_selection(selection.span.size());
  for (const int point_i : selection_typed.index_range()) {
    point_selection[point_i] = selection_typed[point_i];
  }

  selection.finish();

  return point_selection;
}

static void expand_changed_point_to_segment(const int segment_stroke_index,
                                            const int segment_point_start,
                                            const IndexRange points,
                                            MutableSpan<bool> &new_selection,
                                            Vector<bool> &stored_selection,
                                            const Vector<Stroke2DSpace> &strokes_2d,
                                            const bool selection_state)
{
  /* Get 2D stroke with segment to expand. */
  const Stroke2DSpace stroke_2d = strokes_2d[segment_stroke_index];
  const int index_offset = stroke_2d.first_index;

  /* Walk forward and backward along the curve from the point where
   * the selection has changed. We expand the changed selection
   * to the entire segment. */
  const int directions[2] = {-1, 1};
  for (const int direction : directions) {
    int point_abs = segment_point_start;
    bool intersected = false;
    while ((direction == -1 && point_abs > points.first()) ||
           (direction == 1 && point_abs < points.last()))
    {
      int point_rel = point_abs - index_offset;

      /* The end of a segment is reached when the vector between the current and
       * next point intersects with an other stroke. */
      intersected = intersect_segment_strokes_2d(stroke_2d.points[point_rel],
                                                 stroke_2d.points[point_rel + direction],
                                                 segment_stroke_index,
                                                 strokes_2d);
      /* Set selection state. */
      new_selection[point_abs] = selection_state;
      stored_selection[point_abs] = selection_state;

      /* Intersection found, so stop expanding the segment. */
      if (intersected) {
        break;
      }

      point_abs += direction;
    }

    /* Handle last point. */
    if (!intersected) {
      new_selection[point_abs] = selection_state;
      stored_selection[point_abs] = selection_state;
    }
  }
}

void expand_changed_selection_to_segments(Vector<bool> &stored_selection,
                                          bke::CurvesGeometry &curves,
                                          const int curve_offset,
                                          const Vector<Stroke2DSpace> &strokes_2d)
{
  /* Compare the new point selection with the stored selection and expand the changed points to
   * segments.
   * Note: we can't use parallel threads here, because the new selection will change during
   * execution.
   */
  const OffsetIndices points_by_curve = curves.points_by_curve();
  bke::GSpanAttributeWriter selection = ed::curves::ensure_selection_attribute(
      curves, ATTR_DOMAIN_POINT, CD_PROP_BOOL);
  MutableSpan<bool> new_selection = selection.span.typed<bool>();

  for (const int curve_i : curves.curves_range()) {
    const IndexRange points = points_by_curve[curve_i];

    for (int point_i = points.first(); point_i <= points.last(); point_i++) {
      /* Expand to segment when selection is changed. */
      if (stored_selection[point_i] != new_selection[point_i]) {
        expand_changed_point_to_segment(curve_i + curve_offset,
                                        point_i,
                                        points,
                                        new_selection,
                                        stored_selection,
                                        strokes_2d,
                                        new_selection[point_i]);
      }
    }
  }

  selection.finish();
}

}  // namespace blender::ed::greasepencil
