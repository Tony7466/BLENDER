/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgreasepencil
 */

#include "BKE_brush.hh"
#include "BKE_context.h"
#include "BKE_grease_pencil.hh"
#include "BKE_material.h"

#include "BLI_math_vector.hh"
#include "BLI_rect.h"
#include "BLI_threads.h"

#include "DNA_brush_types.h"
#include "DNA_material_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "DEG_depsgraph_query.hh"

#include "ED_curves.hh"
#include "ED_grease_pencil.hh"
#include "ED_view3d.hh"

namespace blender::ed::greasepencil {

static float3 drawing_origin(const Scene *scene, const Object *object, char align_flag)
{
  BLI_assert(object != nullptr && object->type == OB_GREASE_PENCIL);
  if (align_flag & GP_PROJECT_VIEWSPACE) {
    if (align_flag & GP_PROJECT_CURSOR) {
      return float3(scene->cursor.location);
    }
    /* Use the object location. */
    return float3(object->object_to_world[3]);
  }
  return float3(scene->cursor.location);
}

static float3 screen_space_to_3d(
    const Scene *scene, const ARegion *region, const View3D *v3d, const Object *object, float2 co)
{
  float3 origin = drawing_origin(scene, object, scene->toolsettings->gpencil_v3d_align);
  float3 r_co;
  ED_view3d_win_to_3d(v3d, region, origin, co, r_co);
  return r_co;
}

float brush_radius_world_space(bContext &C, int x, int y)
{
  ARegion *region = CTX_wm_region(&C);
  View3D *v3d = CTX_wm_view3d(&C);
  Scene *scene = CTX_data_scene(&C);
  Object *object = CTX_data_active_object(&C);
  Brush *brush = scene->toolsettings->gp_paint->paint.brush;

  /* Default radius. */
  float radius = 2.0f;
  if (brush == nullptr || object->type != OB_GREASE_PENCIL) {
    return radius;
  }

  /* Use an (arbitrary) screen space offset in the x direction to measure the size. */
  const int x_offest = 64;
  const float brush_size = float(BKE_brush_size_get(scene, brush));

  /* Get two 3d coordinates to measure the distance from. */
  const float2 screen1(x, y);
  const float2 screen2(x + x_offest, y);
  const float3 pos1 = screen_space_to_3d(scene, region, v3d, object, screen1);
  const float3 pos2 = screen_space_to_3d(scene, region, v3d, object, screen2);

  /* Clip extreme zoom level (and avoid division by zero). */
  const float distance = math::max(math::distance(pos1, pos2), 0.001f);

  /* Calculate the radius of the brush in world space. */
  radius = (1.0f / distance) * (brush_size / 64.0f);

  return radius;
}

Array<int> get_frame_numbers_for_layer(const bke::greasepencil::Layer &layer,
                                       const int current_frame,
                                       const bool use_multi_frame_editing)
{
  Vector<int> frame_numbers({current_frame});
  if (use_multi_frame_editing) {
    for (const auto [frame_number, frame] : layer.frames().items()) {
      if (frame.is_selected() && frame_number != current_frame) {
        frame_numbers.append_unchecked(frame_number);
      }
    }
  }
  return frame_numbers.as_span();
}

Array<MutableDrawingInfo> retrieve_editable_drawings(const Scene &scene,
                                                     GreasePencil &grease_pencil)
{
  using namespace blender::bke::greasepencil;
  const int current_frame = scene.r.cfra;
  const ToolSettings *toolsettings = scene.toolsettings;
  const bool use_multi_frame_editing = (toolsettings->gpencil_flags &
                                        GP_USE_MULTI_FRAME_EDITING) != 0;

  Vector<MutableDrawingInfo> editable_drawings;
  Span<Layer *> layers = grease_pencil.layers_for_write();
  for (const int layer_i : layers.index_range()) {
    Layer *layer = layers[layer_i];
    if (!layer->is_editable()) {
      continue;
    }
    const Array<int> frame_numbers = get_frame_numbers_for_layer(
        *layer, current_frame, use_multi_frame_editing);
    for (const int frame_number : frame_numbers) {
      if (Drawing *drawing = grease_pencil.get_editable_drawing_at(layer, frame_number)) {
        editable_drawings.append({*drawing, layer_i, frame_number});
      }
    }
  }

  return editable_drawings.as_span();
}

Array<DrawingInfo> retrieve_visible_drawings(const Scene &scene, const GreasePencil &grease_pencil)
{
  using namespace blender::bke::greasepencil;
  const int current_frame = scene.r.cfra;
  const ToolSettings *toolsettings = scene.toolsettings;
  const bool use_multi_frame_editing = (toolsettings->gpencil_flags &
                                        GP_USE_MULTI_FRAME_EDITING) != 0;

  Vector<DrawingInfo> visible_drawings;
  Span<const Layer *> layers = grease_pencil.layers();
  for (const int layer_i : layers.index_range()) {
    const Layer *layer = layers[layer_i];
    if (!layer->is_visible()) {
      continue;
    }
    const Array<int> frame_numbers = get_frame_numbers_for_layer(
        *layer, current_frame, use_multi_frame_editing);
    for (const int frame_number : frame_numbers) {
      if (const Drawing *drawing = grease_pencil.get_drawing_at(layer, frame_number)) {
        visible_drawings.append({*drawing, layer_i, frame_number});
      }
    }
  }

  return visible_drawings.as_span();
}

Curves2DSpace curves_in_2d_space_get(ViewContext *vc,
                                     Object *ob,
                                     Vector<const bke::greasepencil::Drawing *> &drawings,
                                     Vector<int> &layer_index,
                                     const int frame_number,
                                     const bool get_stroke_flag)
{
  /* Get viewport projection matrix and evaluated GP object. */
  float4x4 projection;
  ED_view3d_ob_project_mat_get(vc->rv3d, ob, projection.ptr());
  const Object *ob_eval = DEG_get_evaluated_object(vc->depsgraph, ob);

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
  if (get_stroke_flag) {
    cv2d.has_stroke = Array<bool>(curve_num);
  }

  /* Loop all drawings. */
  threading::parallel_for(cv2d.drawings.index_range(), 1, [&](const IndexRange range) {
    for (const int drawing_i : range) {
      /* Get deformed geomtry. */
      const bke::CurvesGeometry &curves = cv2d.drawings[drawing_i]->strokes();
      const OffsetIndices points_by_curve = curves.points_by_curve();
      const VArray<bool> cyclic = curves.cyclic();
      const bke::crazyspace::GeometryDeformation deformation =
          bke::crazyspace::get_evaluated_grease_pencil_drawing_deformation(
              ob_eval, *ob, layer_index[drawing_i], frame_number);

      /* Get the curve materials. */
      VArray<int> materials;
      if (get_stroke_flag) {
        materials = *curves.attributes().lookup_or_default<int>(
            "material_index", ATTR_DOMAIN_CURVE, 0);
      }

      /* Get the initial point index in the contiguous 2D point array for the curves in this
       * drawing. */
      int point_contiguous = curve_point_offset[drawing_i];

      /* Loop all curves. */
      for (const int curve_i : curves.curves_range()) {
        const int curve_contiguous = cv2d.curve_offset[drawing_i] + curve_i;
        const IndexRange points = points_by_curve[curve_i];

        /* Set curve data. */
        BLI_rctf_init_minmax(&cv2d.curve_bbox[curve_contiguous]);
        cv2d.point_size[curve_contiguous] = points.size();
        cv2d.point_offset[curve_contiguous] = point_contiguous;
        cv2d.is_cyclic[curve_contiguous] = cyclic[curve_i];
        cv2d.drawing_index_2d[curve_contiguous] = drawing_i;

        /* Set stroke flag: true when the stroke property is active in the curve material. */
        if (get_stroke_flag) {
          Material *material = BKE_object_material_get(ob, materials[curve_i] + 1);
          cv2d.has_stroke[curve_contiguous] = (material && (material->gp_style->flag &
                                                            GP_MATERIAL_STROKE_SHOW) != 0);
        }

        /* Loop all stroke points. */
        for (const int point_i : points) {
          float2 pos;

          /* Convert point to 2D. */
          ED_view3d_project_float_v2_m4(
              vc->region, deformation.positions[point_i], pos, projection.ptr());

          /* Store 2D point in contiguous array. */
          cv2d.points_2d[point_contiguous] = pos;

          /* Update stroke bounding box. */
          BLI_rctf_do_minmax_v(&cv2d.curve_bbox[curve_contiguous], pos);

          point_contiguous++;
        }
      }
    }
  });

  return cv2d;
}

}  // namespace blender::ed::greasepencil
