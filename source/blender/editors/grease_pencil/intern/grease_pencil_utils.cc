/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgreasepencil
 */

#include "BLI_rect.h"
#include "BLI_threads.h"

#include "BKE_grease_pencil.hh"
#include "BKE_material.h"

#include "DEG_depsgraph_query.h"

#include "DNA_material_types.h"

#include "ED_curves.hh"
#include "ED_grease_pencil.hh"
#include "ED_view3d.hh"

namespace blender::ed::greasepencil {

Curves2DSpace curves_in_2d_space_get(ViewContext *vc,
                                     Object *ob,
                                     Vector<GreasePencilDrawing *> &drawings,
                                     Vector<int> &drawing_indices)
{
  /* Get viewport projection matrix and evaluated GP object. */
  float4x4 projection;
  ED_view3d_ob_project_mat_get(vc->rv3d, ob, projection.ptr());
  const Object *ob_eval = DEG_get_evaluated_object(vc->depsgraph, const_cast<Object *>(ob));

  /* Count total number of editable curves and points in given Grease Pencil drawings. */
  Curves2DSpace cv2d;
  Vector<int> curve_point_offset;
  int curve_num = 0, point_num = 0;

  for (const auto &drawing : drawings) {
    cv2d.drawings.append(drawing);
    cv2d.curve_offset.append(curve_num);
    curve_point_offset.append(point_num);

    curve_num += drawing->geometry.curve_num;
    point_num += drawing->geometry.point_num;
  }

  /* Initialize the contiguous arrays for the 2D curve data. */
  cv2d.drawing_index_2d = Array<int>(curve_num);
  cv2d.is_cyclic = Array<bool>(curve_num);
  cv2d.point_offset = Array<int>(curve_num);
  cv2d.point_size = Array<int>(curve_num);
  cv2d.curve_bbox = Array<rctf>(curve_num);
  cv2d.points_2d = Array<float2>(point_num);

  /* Loop all drawings. */
  threading::parallel_for(cv2d.drawings.index_range(), 1, [&](const IndexRange range) {
    for (const int drawing_i : range) {
      /* Get deformed geomtry. */
      const bke::CurvesGeometry &curves = cv2d.drawings[drawing_i]->geometry.wrap();
      const OffsetIndices points_by_curve = curves.points_by_curve();
      const VArray<bool> cyclic = curves.cyclic();
      const bke::crazyspace::GeometryDeformation deformation =
          bke::crazyspace::get_evaluated_grease_pencil_drawing_deformation(
              ob_eval, *ob, drawing_indices[drawing_i]);

      /* Get the initial point index in the 2D point array for curves in this drawing. */
      int point_cont = curve_point_offset[drawing_i];

      /* Loop all curves. */
      for (const int curve_i : curves.curves_range()) {
        const int cv_cont = curve_i + cv2d.curve_offset[drawing_i];
        const IndexRange points = points_by_curve[curve_i];

        /* Set curve data. */
        BLI_rctf_init_minmax(&cv2d.curve_bbox[cv_cont]);
        cv2d.point_size[cv_cont] = points.size();
        cv2d.point_offset[cv_cont] = point_cont;
        cv2d.is_cyclic[cv_cont] = cyclic[curve_i];
        cv2d.drawing_index_2d[cv_cont] = drawing_i;

        /* Loop all stroke points. */
        for (const int point_i : points) {
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

}  // namespace blender::ed::greasepencil
