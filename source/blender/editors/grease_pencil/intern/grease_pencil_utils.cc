/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgreasepencil
 */

#include "BKE_brush.hh"
#include "BKE_context.h"
#include "BKE_grease_pencil.hh"

#include "BLI_math_vector.hh"

#include "DNA_brush_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"

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

static Array<int> get_frame_numbers_for_layer(const blender::bke::greasepencil::Layer *layer,
                                              const int frame,
                                              const bool use_multi_frame_editing)
{
  if (!use_multi_frame_editing) {
    return Array<int>({frame});
  }
  Vector<int> frame_numbers;
  for (const auto &[frame_number, frame] : layer->frames().items()) {
    if (frame.is_selected()) {
      frame_numbers.append_unchecked(frame_number);
    }
  }
  return frame_numbers.as_span();
}

static void foreach_editable_drawing_ex(
    const GreasePencil &grease_pencil,
    const int frame,
    const bool use_multi_frame_editing,
    blender::FunctionRef<void(const int, const int, const blender::bke::greasepencil::Drawing &)>
        function)
{
  using namespace blender::bke::greasepencil;

  blender::Span<const Layer *> layers = grease_pencil.layers();
  for (const int layer_i : layers.index_range()) {
    const Layer *layer = layers[layer_i];
    if (!layer->is_editable()) {
      continue;
    }

    Array<int> frame_numbers = get_frame_numbers_for_layer(layer, frame, use_multi_frame_editing);
    for (const int frame_number : frame_numbers) {
      if (const Drawing *drawing = grease_pencil.get_drawing_at(layer, frame_number)) {
        function(layer_i, frame_number, *drawing);
      }
    }
  }
}

static void foreach_visible_drawing_ex(
    const GreasePencil &grease_pencil,
    const int frame,
    const bool use_multi_frame_editing,
    blender::FunctionRef<void(const int, const int, const blender::bke::greasepencil::Drawing &)>
        function)
{
  using namespace blender::bke::greasepencil;

  blender::Span<const Layer *> layers = grease_pencil.layers();
  for (const int layer_i : layers.index_range()) {
    const Layer *layer = layers[layer_i];
    if (!layer->is_visible()) {
      continue;
    }

    Array<int> frame_numbers = get_frame_numbers_for_layer(layer, frame, use_multi_frame_editing);
    for (const int frame_number : frame_numbers) {
      if (const Drawing *drawing = grease_pencil.get_drawing_at(layer, frame_number)) {
        function(layer_i, frame_number, *drawing);
      }
    }
  }
}

void foreach_editable_drawing(const Scene *scene,
                              GreasePencil &grease_pencil,
                              FunctionRef<void(int, int, bke::greasepencil::Drawing &)> function)
{
  const int current_frame = scene->r.cfra;
  const ToolSettings *toolsettings = scene->toolsettings;
  const bool use_multi_frame_editing = (toolsettings->gpencil_flags &
                                        GP_USE_MULTI_FRAME_EDITING) != 0;
  foreach_editable_drawing_ex(
      grease_pencil,
      current_frame,
      use_multi_frame_editing,
      [&](const int layer_index,
          const int frame_number,
          const blender::bke::greasepencil::Drawing &drawing) {
        function(
            layer_index, frame_number, const_cast<blender::bke::greasepencil::Drawing &>(drawing));
      });
}

void foreach_visible_drawing(
    const Scene &scene,
    const GreasePencil &grease_pencil,
    FunctionRef<void(const int, const int, const bke::greasepencil::Drawing &)> function)
{
  const int current_frame = scene.r.cfra;
  const ToolSettings *toolsettings = scene.toolsettings;
  const bool use_multi_frame_editing = (toolsettings->gpencil_flags &
                                        GP_USE_MULTI_FRAME_EDITING) != 0;
  foreach_visible_drawing_ex(grease_pencil, current_frame, use_multi_frame_editing, function);
}

}  // namespace blender::ed::greasepencil
