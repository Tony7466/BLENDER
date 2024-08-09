/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "overlay_next_private.hh"

namespace blender::draw::overlay {

class Outline {
 private:
  /* Simple render pass that renders an object ID pass. */
  PassMain outline_prepass_ps_ = {"Prepass"};
  /* Detect edges inside the ID pass and output color for each of them. */
  PassSimple outline_resolve_ps_ = {"Resolve"};

  TextureFromPool object_id_tx_ = {"outline_ob_id_tx"};
  TextureFromPool tmp_depth_tx_ = {"outline_depth_tx"};

  Framebuffer prepass_fb_ = {"outline.prepass_fb"};

  bool enabled = false;

 public:
  void begin_sync(Resources &res, const State &state)
  {
    enabled = (state.v3d_flag & V3D_SELECT_OUTLINE);
    if (!enabled) {
      return;
    }

    const float outline_width = UI_GetThemeValuef(TH_OUTLINE_WIDTH);
    const bool do_expand = (U.pixelsize > 1.0) || (outline_width > 2.0f);
    const bool is_transform = (G.moving & G_TRANSFORM_OBJ) != 0;

    {
      auto &pass = outline_prepass_ps_;
      pass.init();
      pass.framebuffer_set(&prepass_fb_);
      pass.clear_color_depth_stencil(float4(0.0f), 1.0f, 0x0);
      pass.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS_EQUAL |
                     state.clipping_state);
      pass.shader_set(state.xray_enabled_and_not_wire ? res.shaders.outline_prepass_wire.get() :
                                                        res.shaders.outline_prepass_mesh.get());
      pass.push_constant("isTransform", is_transform);
      pass.bind_ubo("globalsBlock", &res.globals_buf);
    }
    {
      auto &pass = outline_resolve_ps_;
      pass.init();
      pass.framebuffer_set(&res.overlay_line_only_fb);
      pass.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_BLEND_ALPHA_PREMUL);
      pass.shader_set(res.shaders.outline_detect.get());
      /* Don't occlude the outline if in xray mode as it causes too much flickering. */
      pass.push_constant("alphaOcclu", state.xray_enabled ? 1.0f : 0.35f);
      pass.push_constant("doThickOutlines", do_expand);
      pass.push_constant("doAntiAliasing", state.xray_enabled_and_not_wire);
      pass.bind_texture("outlineId", &object_id_tx_);
      pass.bind_texture("sceneDepth", &res.depth_tx);
      pass.bind_texture("outlineDepth", &tmp_depth_tx_);
      pass.bind_ubo("globalsBlock", &res.globals_buf);
      pass.draw_procedural(GPU_PRIM_TRIS, 1, 3);
    }
  }

  void object_sync(Manager &manager, const ObjectRef &ob_ref, const State & /*state*/)
  {
    if (!enabled) {
      return;
    }

    /* Outlines of bounding boxes are not drawn. */
    if (ob_ref.object->dt == OB_BOUNDBOX) {
      return;
    }

    ResourceHandle res_handle = manager.resource_handle(ob_ref);

    gpu::Batch *geom = DRW_cache_object_surface_get(ob_ref.object);
    if (geom != nullptr) {
      outline_prepass_ps_.draw(geom, res_handle);
    }
  }

  void draw(Resources &res, Manager &manager, View &view)
  {
    if (!enabled) {
      return;
    }

    GPU_debug_group_begin("Outline");

    int2 render_size = int2(res.depth_tx.size());

    eGPUTextureUsage usage = GPU_TEXTURE_USAGE_SHADER_READ | GPU_TEXTURE_USAGE_ATTACHMENT;
    tmp_depth_tx_.acquire(render_size, GPU_DEPTH24_STENCIL8, usage);
    object_id_tx_.acquire(render_size, GPU_R16UI, usage);

    prepass_fb_.ensure(GPU_ATTACHMENT_TEXTURE(tmp_depth_tx_),
                       GPU_ATTACHMENT_TEXTURE(object_id_tx_));

    manager.submit(outline_prepass_ps_, view);
    manager.submit(outline_resolve_ps_, view);

    tmp_depth_tx_.release();
    object_id_tx_.release();

    GPU_debug_group_end();
  }
};

}  // namespace blender::draw::overlay
