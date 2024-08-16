/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "draw_common.hh"

namespace blender::draw::overlay {

class Wireframe {
 private:
  const SelectionType selection_type_;

  PassMain wireframe_ps_ = {"Wireframe"};
  struct ColoringPass {
    /* TODO(fclem): Not yet implemented. */
    // PassMain::Sub *curves_ps_ = nullptr;
    PassMain::Sub *gpencil_ps_ = nullptr;
    PassMain::Sub *mesh_ps_ = nullptr;
    /* Variant for meshes that force drawing all edges. */
    PassMain::Sub *mesh_all_edges_ps_ = nullptr;
  } colored, non_colored;
  /* Some objects types renders as points in wireframe draw mode. */
  PassMain point_ps_ = {"Loose Points"};

  bool enabled = false;

 public:
  Wireframe(const SelectionType selection_type) : selection_type_(selection_type){};

  void begin_sync(Resources &res, const State &state)
  {
    enabled = state.is_wireframe_mode || (state.overlay.flag & V3D_OVERLAY_WIREFRAMES);
    if (!enabled) {
      return;
    }

    const bool is_transform = (G.moving & G_TRANSFORM_OBJ) != 0;
    const float wire_threshold = wire_discard_threshold_get(state.overlay.wireframe_threshold);

    GPUTexture **depth_tex = (state.xray_enabled) ? &res.depth_tx : &res.dummy_depth_tx;

    {
      auto &pass = wireframe_ps_;
      pass.init();
      pass.state_set(DRW_STATE_FIRST_VERTEX_CONVENTION | DRW_STATE_WRITE_COLOR |
                     DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS_EQUAL | state.clipping_state);
      res.select_bind(pass);

      auto shader_pass =
          [&](GPUShader *shader, const char *name, bool use_coloring, float wire_threshold) {
            auto &sub = pass.sub(name);
            sub.shader_set(shader);
            sub.bind_ubo("globalsBlock", &res.globals_buf);
            sub.bind_texture("depthTex", depth_tex);
            sub.push_constant("wireOpacity", state.overlay.wireframe_opacity);
            sub.push_constant("isTransform", is_transform);
            sub.push_constant("colorType", state.v3d->shading.wire_color_type);
            sub.push_constant("useColoring", use_coloring);
            sub.push_constant("wireStepParam", wire_threshold);
            sub.push_constant("isHair", false);
            return &sub;
          };

      auto coloring_pass = [&](ColoringPass &ps, bool use_color) {
        overlay::ShaderModule &sh = res.shaders;
        ps.mesh_ps_ = shader_pass(sh.wireframe_mesh.get(), "Mesh", use_color, wire_threshold);
        ps.mesh_all_edges_ps_ = shader_pass(sh.wireframe_mesh.get(), "Wire", use_color, 1.0f);
      };

      coloring_pass(non_colored, false);
      coloring_pass(colored, true);
    }
    {
      auto &pass = point_ps_;
      pass.init();
      pass.state_set(DRW_STATE_FIRST_VERTEX_CONVENTION | DRW_STATE_WRITE_COLOR |
                     DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS_EQUAL | state.clipping_state);
      pass.shader_set(res.shaders.wireframe_points.get());
      pass.bind_ubo("globalsBlock", &res.globals_buf);
      res.select_bind(pass);
    }
  }

  void object_sync(Manager &manager, const ObjectRef &ob_ref, Resources &res)
  {
    if (!enabled) {
      return;
    }

    const bool all_edges = (ob_ref.object->dtx & OB_DRAW_ALL_EDGES) != 0;
    const bool use_coloring =
        true;  // !is_edit_mode && !instance_parent_in_edit_mode && !is_sculpt_mode;

    /* TODO(fclem): Non-mandatory handle creation and reuse with other overlays. */
    ResourceHandle res_handle = manager.resource_handle(ob_ref);

    ColoringPass &coloring = use_coloring ? non_colored : colored;
    gpu::Batch *geom;
    switch (ob_ref.object->type) {
      case OB_CURVES:
        /* TODO(fclem): Not yet implemented. */
        break;
      case OB_GREASE_PENCIL:
        break;
      case OB_MESH:
        geom = DRW_cache_mesh_face_wireframe_get(ob_ref.object);
        (all_edges ? coloring.mesh_all_edges_ps_ : coloring.mesh_ps_)
            ->draw(geom, res_handle, res.select_id(ob_ref).get());
#if 0 /* TODO */
        if (!is_edit_mode || has_edit_mesh_cage) {
          /* Draw loose geometry. */
          if (is_mesh_verts_only) {
            /* Draw loose verts. */
          }
          else {
            /* Draw loose edges. */
          }
        }
#endif
        break;
      case OB_POINTCLOUD:
        geom = DRW_pointcloud_batch_cache_get_dots(ob_ref.object);
        point_ps_.draw(geom, res_handle, res.select_id(ob_ref).get());
        break;
      case OB_VOLUME:
        break;
      default:
        break;
    }
  }

  void draw(Framebuffer &framebuffer, Manager &manager, View &view)
  {
    if (!enabled) {
      return;
    }

    GPU_framebuffer_bind(framebuffer);
    manager.submit(wireframe_ps_, view);
  }

 private:
  float wire_discard_threshold_get(float threshold)
  {
    /* Use `sqrt` since the value stored in the edge is a variation of the cosine, so its square
     * becomes more proportional with a variation of angle. */
    threshold = sqrt(abs(threshold));
    /* The maximum value (255 in the VBO) is used to force hide the edge. */
    return math::interpolate(0.0f, 1.0f - (1.0f / 255.0f), threshold);
  }
};

}  // namespace blender::draw::overlay
