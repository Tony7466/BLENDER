/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgreasepencil
 */

#include "BLI_rect.h"
#include "BLI_threads.h"

#include "BKE_grease_pencil.hh"

#include "DEG_depsgraph_query.h"

#include "ED_curves.hh"
#include "ED_grease_pencil.h"
#include "ED_view3d.h"

namespace blender::ed::greasepencil {

Curves2DSpace editable_strokes_in_2d_space_get(ViewContext *vc, Object *ob)
{
  /* Get viewport projection matrix and evaluated GP object. */
  float4x4 projection;
  ED_view3d_ob_project_mat_get(vc->rv3d, vc->obact, projection.ptr());
  const Object *ob_eval = DEG_get_evaluated_object(vc->depsgraph, const_cast<Object *>(ob));
  GreasePencil *grease_pencil = static_cast<GreasePencil *>(ob->data);

  /* Count total number of editable curves and points in grease pencil object. */
  Curves2DSpace cv2d;
  Vector<GreasePencilDrawing *> drawings;
  Vector<int> drawing_indices;
  Vector<int> curve_point_offset;
  int drawing_num = 0, curve_num = 0, point_num = 0;

  grease_pencil->foreach_editable_drawing(vc->scene->r.cfra,
                                          [&](int drawing_index, GreasePencilDrawing &drawing) {
                                            cv2d.drawings.append(&drawing);
                                            cv2d.curve_offset.append(curve_num);
                                            curve_point_offset.append(point_num);
                                            drawings.append(&drawing);
                                            drawing_indices.append(drawing_index);

                                            drawing_num++;
                                            curve_num += drawing.geometry.curve_num;
                                            point_num += drawing.geometry.point_num;
                                          });

  /* Initialize the contiguous arrays for the 2D curve data. */
  cv2d.drawing_index_2d = Array<int>(curve_num);
  cv2d.is_cyclic = Array<bool>(curve_num);
  cv2d.point_offset = Array<int>(curve_num);
  cv2d.point_size = Array<int>(curve_num);
  cv2d.curve_bbox = Array<rctf>(curve_num);
  cv2d.points_2d = Array<float2>(point_num);

  /* Loop all drawings. */
  threading::parallel_for(drawings.index_range(), 1, [&](const IndexRange range) {
    for (const int drawing_i : range) {
      /* Get deformed geomtry. */
      const bke::CurvesGeometry &curves = drawings[drawing_i]->geometry.wrap();
      const OffsetIndices points_by_curve = curves.points_by_curve();
      const VArray<bool> cyclic = curves.cyclic();
      const bke::crazyspace::GeometryDeformation deformation =
          bke::crazyspace::get_evaluated_grease_pencil_drawing_deformation(
              ob_eval, *vc->obedit, drawing_indices[drawing_i]);

      /* Get the initial point index in the 2D point array for curves in this drawing. */
      int point_cont = curve_point_offset[drawing_i];

      /* Loop all curves. */
      for (const int curve_i : curves.curves_range()) {
        const int cv_cont = curve_i + cv2d.curve_offset[drawing_i];
        const IndexRange points = points_by_curve[curve_i];

        BLI_rctf_init_minmax(&cv2d.curve_bbox[cv_cont]);
        cv2d.point_size[cv_cont] = points.size();
        cv2d.point_offset[cv_cont] = point_cont;
        cv2d.is_cyclic[cv_cont] = cyclic[curve_i];
        cv2d.drawing_index_2d[cv_cont] = drawing_i;

        /* Loop all stroke points. */
        for (int point_i = points.first(); point_i <= points.last(); point_i++) {
          float2 pos;

          /* Convert point to 2D. */
          ED_view3d_project_float_v2_m4(
              vc->region, deformation.positions[point_i], pos, projection.ptr());

          /* Store 2D point in contiguous array. */
          cv2d.points_2d[point_cont] = pos;

          /* Update stroke bounding box. */
          BLI_rctf_do_minmax_v(&cv2d.curve_bbox[cv_cont], pos);

          point_cont++;
        }
      }
    }
  });

  return cv2d;
}

static float intersection_distance_get(const float2 &v1,
                                       const float2 &v2,
                                       const float2 &v3,
                                       const float2 &v4)
{
  /* Check for zero length vector. */
  const float vec_length = math::length(v2 - v1);
  if (vec_length == 0.0f) {
    return 0.0f;
  }

  /* Calculate intersection point. */
  const float div = (v2[0] - v1[0]) * (v4[1] - v3[1]) - (v2[1] - v1[1]) * (v4[0] - v3[0]);
  float2 isect;
  isect[0] = ((v1[0] * v2[1] - v1[1] * v2[0]) * (v3[0] - v4[0]) -
              (v1[0] - v2[0]) * (v3[0] * v4[1] - v3[1] * v4[0])) /
             div;
  isect[1] = ((v1[0] * v2[1] - v1[1] * v2[0]) * (v3[1] - v4[1]) -
              (v1[1] - v2[1]) * (v3[0] * v4[1] - v3[1] * v4[0])) /
             div;

  return math::length(isect - v1) / vec_length;
}

Vector<IntersectingSegment2D> intersections_segment_strokes_2d(const float2 segment_start,
                                                               const float2 segment_end,
                                                               const int segment_curve_index,
                                                               const Curves2DSpace *curves_2d)
{
  /* Init result vector. */
  Vector<IntersectingSegment2D> intersections;
  std::mutex mutex;

  /* Create bounding box around the segment. */
  rctf bbox_sel;
  BLI_rctf_init_minmax(&bbox_sel);
  BLI_rctf_do_minmax_v(&bbox_sel, segment_start);
  BLI_rctf_do_minmax_v(&bbox_sel, segment_end);

  /* Loop all strokes, looking for intersecting segments. */
  threading::parallel_for(curves_2d->point_offset.index_range(), 256, [&](const IndexRange range) {
    for (const int curve_i : range) {
      /* Do a quick bounding box check first. When the bounding box of a stroke doesn't
       * intersect with the segment, none of the stroke segments do. */
      if (!BLI_rctf_isect(&bbox_sel, &curves_2d->curve_bbox[curve_i], nullptr)) {
        continue;
      }

      /* Handle cyclic curves. */
      const int point_offset = curves_2d->point_offset[curve_i];
      const int point_size = curves_2d->point_size[curve_i] -
                             (curves_2d->is_cyclic[curve_i] ? 0 : 1);
      const int point_last = point_offset + curves_2d->point_size[curve_i] - 1;

      /* Test for intersecting stroke segments. */
      for (const int point_i : IndexRange(point_offset, point_size)) {
        const int point_i_next = (point_i == point_last ? point_offset : point_i + 1);

        /* Don't self check. */
        if (curve_i == segment_curve_index) {
          if (segment_start == curves_2d->points_2d[point_i] ||
              segment_start == curves_2d->points_2d[point_i_next] ||
              segment_end == curves_2d->points_2d[point_i] ||
              segment_end == curves_2d->points_2d[point_i_next])
          {
            continue;
          }
        }

        auto isect = math::isect_seg_seg(curves_2d->points_2d[point_i],
                                         curves_2d->points_2d[point_i_next],
                                         segment_start,
                                         segment_end);

        if (isect.kind == isect.LINE_LINE_CROSS) {
          IntersectingSegment2D intersection{};
          intersection.curve_index_2d = curve_i;
          intersection.point_start = point_i - point_offset;
          intersection.point_end = point_i_next - point_offset;
          intersection.distance = intersection_distance_get(curves_2d->points_2d[point_i],
                                                            curves_2d->points_2d[point_i_next],
                                                            segment_start,
                                                            segment_end);

          std::lock_guard lock{mutex};
          intersections.append(intersection);
        }
      }
    }
  });

  return intersections;
}

}  // namespace blender::ed::greasepencil
