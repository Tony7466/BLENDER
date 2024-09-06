/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "BKE_paint.hh"

#include "overlay_next_private.hh"

namespace blender::draw::overlay {
class Fade {
 private:
  const SelectionType selection_type_;

  PassMain ps_ = {"Fade"};

  bool enabled_ = false;

 public:
  Fade(const SelectionType selection_type_) : selection_type_(selection_type_) {}

  void begin_sync(Resources &res, const State &state)
  {
    enabled_ = state.v3d && !state.xray_enabled &&
               (state.overlay.flag & V3D_OVERLAY_FADE_INACTIVE) &&
               (selection_type_ == SelectionType::DISABLED);

    if (!enabled_) {
      /* Not used. But release the data. */
      ps_.init();
      return;
    }
    ps_.init();
    ps_.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_DEPTH_LESS_EQUAL | DRW_STATE_BLEND_ALPHA,
                  state.clipping_plane_count);
    ps_.shader_set(res.shaders.uniform_color.get());
    float4 color = res.background_color_get(state);
    color[3] = state.overlay.fade_alpha;
    if (state.v3d->shading.background_type == V3D_SHADING_BACKGROUND_THEME) {
      srgb_to_linearrgb_v4(color, color);
    }
    ps_.push_constant("ucolor", color);
  }

  void object_sync(Manager &manager, const ObjectRef &ob_ref, const State &state)
  {
    if (!enabled_) {
      return;
    }
    const Object *ob = ob_ref.object;
    const bool renderable = DRW_object_is_renderable(ob);
    const bool draw_surface = (ob->dt >= OB_WIRE) && (renderable || (ob->dt == OB_WIRE));
    const bool draw_fade = draw_surface && overlay_should_fade_object(ob, state.object_active);
    if (!draw_fade) {
      return;
    }

    const bool use_sculpt_pbvh = BKE_sculptsession_use_pbvh_draw(ob, state.rv3d) &&
                                 !DRW_state_is_image_render();

    if (use_sculpt_pbvh) {
      ResourceHandle handle = manager.resource_handle_for_sculpt(ob_ref);

      for (SculptBatch &batch : sculpt_batches_get(ob_ref.object, SCULPT_BATCH_DEFAULT)) {
        ps_.draw(batch.batch, handle, select::SelectMap::select_invalid_id().get());
      }
    }
    else {
      blender::gpu::Batch *geom = DRW_cache_object_surface_get((Object *)ob);
      if (geom) {
        ResourceHandle res_handle = manager.resource_handle(ob_ref);
        ps_.draw(geom, res_handle);
      }
    }
  }

  void draw(Framebuffer &framebuffer, Manager &manager, View &view)
  {
    GPU_framebuffer_bind(framebuffer);
    manager.submit(ps_, view);
  }

 private:
  static bool overlay_should_fade_object(const Object *ob, const Object *active_object)
  {
    if (!active_object || !ob) {
      return false;
    }

    if (ELEM(active_object->mode, OB_MODE_OBJECT, OB_MODE_POSE)) {
      return false;
    }

    if ((active_object->mode & ob->mode) != 0) {
      return false;
    }

    return true;
  }
};
}  // namespace blender::draw::overlay
