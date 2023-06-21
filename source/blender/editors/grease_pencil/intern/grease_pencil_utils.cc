/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgreasepencil
 */

#include "BLI_rect.h"

#include "BKE_grease_pencil.hh"

#include "DEG_depsgraph_query.h"

#include "ED_curves.h"
#include "ED_grease_pencil.h"
#include "ED_view3d.h"

namespace blender::ed::greasepencil {

Vector<Stroke2DSpace> editable_strokes_in_2d_space_get(ViewContext *vc,
                                                       GreasePencil *grease_pencil)
{
  /* Count total number of editable strokes in grease pencil object. */
  int stroke_count = 0;
  grease_pencil->foreach_editable_drawing(
      vc->scene->r.cfra, [&](int /*drawing_index*/, GreasePencilDrawing &drawing) {
        stroke_count += drawing.geometry.curve_num;
      });

  /* Create vector for strokes converted to 2D space. */
  Vector<Stroke2DSpace> strokes_2d(stroke_count);

  /* Get viewport projection matrix and evaluated GP object. */
  float4x4 projection;
  ED_view3d_ob_project_mat_get(vc->rv3d, vc->obact, projection.ptr());

  const Object *ob_eval = DEG_get_evaluated_object(vc->depsgraph,
                                                   const_cast<Object *>(vc->obedit));

  /* Loop all drawings. */
  int curve_offset = 0;
  grease_pencil->foreach_editable_drawing(
      vc->scene->r.cfra, [&](int drawing_index, GreasePencilDrawing &drawing) {
        /* Get deformed geomtry. */
        const bke::CurvesGeometry curves = drawing.geometry.wrap();
        const OffsetIndices points_by_curve = curves.points_by_curve();
        const bke::crazyspace::GeometryDeformation deformation =
            bke::crazyspace::get_evaluated_grease_pencil_drawing_deformation(
                ob_eval, *vc->obedit, drawing_index);

        /* Loop all strokes (curves). */
        threading::parallel_for(curves.curves_range(), 256, [&](const IndexRange range) {
          for (const int curve_i : range) {
            const int stroke_i = curve_i + curve_offset;
            const IndexRange points = points_by_curve[curve_i];
            strokes_2d[stroke_i] = Stroke2DSpace();

            BLI_rctf_init_minmax(&strokes_2d[stroke_i].bbox);
            strokes_2d[stroke_i].first_index = points.first();

            /* Loop all stroke points. */
            for (int point_i = points.first(); point_i <= points.last(); point_i++) {
              float2 pos;

              /* Convert point to 2D. */
              ED_view3d_project_float_v2_m4(
                  vc->region, deformation.positions[point_i], pos, projection.ptr());

              /* Store 2D point. */
              strokes_2d[stroke_i].points.append(pos);

              /* Update stroke bounding box. */
              BLI_rctf_do_minmax_v(&strokes_2d[stroke_i].bbox, pos);
            }
          }
        });

        curve_offset += drawing.geometry.curve_num;
      });

  return strokes_2d;
}

bool intersect_segment_strokes_2d(const float2 segment_start,
                                  const float2 segment_end,
                                  const int segment_stroke_index,
                                  const Vector<Stroke2DSpace> &strokes_2d)
{
  /* Create bounding box around the segment. */
  rctf bbox_sel;
  BLI_rctf_init_minmax(&bbox_sel);
  BLI_rctf_do_minmax_v(&bbox_sel, segment_start);
  BLI_rctf_do_minmax_v(&bbox_sel, segment_end);

  /* Loop all strokes, looking for an intersecting segment. */
  std::atomic<bool> intersect = false;
  threading::parallel_for(strokes_2d.index_range(), 256, [&](const IndexRange range) {
    for (const int stroke_i : range) {
      /* Abort when intersection is found. */
      if (intersect) {
        break;
      }

      /* Do a quick bounding box check first. When the bounding box of a stroke doesn't
       * intersect with the segment, non of the stroke segments do. */
      Stroke2DSpace stroke_isect = strokes_2d[stroke_i];
      if (!BLI_rctf_isect(&bbox_sel, &stroke_isect.bbox, nullptr)) {
        continue;
      }

      /* Test for intersecting stroke segments. */
      for (const int point_i : IndexRange(strokes_2d[stroke_i].points.size() - 1)) {
        /* Don't self check. */
        if (stroke_i == segment_stroke_index) {
          if (segment_start == stroke_isect.points[point_i] ||
              segment_start == stroke_isect.points[point_i + 1] ||
              segment_end == stroke_isect.points[point_i] ||
              segment_end == stroke_isect.points[point_i + 1])
          {
            continue;
          }
        }

        auto isect = math::isect_seg_seg(stroke_isect.points[point_i],
                                         stroke_isect.points[point_i + 1],
                                         segment_start,
                                         segment_end);

        if (isect.kind == isect.LINE_LINE_CROSS) {
          intersect = true;
          break;
        }
      }
    }
  });

  return intersect;
}

}  // namespace blender::ed::greasepencil
