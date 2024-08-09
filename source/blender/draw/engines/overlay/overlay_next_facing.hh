/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "BKE_paint.hh"

#include "overlay_next_private.hh"

namespace blender::draw::overlay {

class Facing {

 private:
  PassMain ps_ = {"Facing"};

 public:
  void begin_sync(Resources &res, const State &state)
  {
    ps_.init();
    ps_.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_DEPTH_LESS_EQUAL | DRW_STATE_WRITE_DEPTH |
                  state.clipping_state);
    ps_.shader_set(res.shaders.facing.get());
    ps_.bind_ubo("globalsBlock", &res.globals_buf);
  }

  void object_sync(Manager &manager, const ObjectRef &ob_ref, const State &state)
  {
    if (state.xray_enabled) {
      return;
    }
    const bool use_sculpt_pbvh = BKE_sculptsession_use_pbvh_draw(ob_ref.object, state.rv3d) &&
                                 !DRW_state_is_image_render();

    if (use_sculpt_pbvh) {
      /* TODO: Add sculpt mode. */
      // DRW_shgroup_call_sculpt(pd->facing_grp[is_xray], ob, false, false, false, false, false);
    }
    else {
      blender::gpu::Batch *geom = DRW_cache_object_surface_get(ob_ref.object);
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
};

}  // namespace blender::draw::overlay
