/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "BKE_global.hh"
#include "BKE_subdiv_modifier.hh"

#include "draw_cache_impl.hh"

#include "overlay_next_private.hh"

namespace blender::draw::overlay {

class Meshes {
 private:
  PassSimple edit_mesh_normals_ps_ = {"Normals"};
  PassSimple::Sub *face_normals_ = nullptr;
  PassSimple::Sub *face_normals_subdiv_ = nullptr;
  PassSimple::Sub *loop_normals_ = nullptr;
  PassSimple::Sub *loop_normals_subdiv_ = nullptr;
  PassSimple::Sub *vert_normals_ = nullptr;

  PassSimple edit_mesh_analysis_ps_ = {"Mesh Analysis"};

  bool show_retopology = false;
  bool show_mesh_analysis = false;
  bool show_face = false;

 public:
  void begin_sync(Resources &res, const State &state)
  {
    int edit_flag = state.v3d->overlay.edit_flag;
    show_retopology = (edit_flag & V3D_OVERLAY_EDIT_RETOPOLOGY);
    show_mesh_analysis = (edit_flag & V3D_OVERLAY_EDIT_STATVIS);
    show_face = ((edit_flag & V3D_OVERLAY_EDIT_FACES));
    const bool show_face_nor = (edit_flag & V3D_OVERLAY_EDIT_FACE_NORMALS);
    const bool show_loop_nor = (edit_flag & V3D_OVERLAY_EDIT_LOOP_NORMALS);
    const bool show_vert_nor = (edit_flag & V3D_OVERLAY_EDIT_VERT_NORMALS);

    if (state.xray_enabled) {
      /* We should not render the mesh opaque. */
      show_mesh_analysis = false;
    }

    float backwire_opacity = (state.xray_enabled) ? 0.5f : 1.0f;
    float face_alpha = (show_face) ? 1.0f : 0.0f;
    float retopology_offset = RETOPOLOGY_OFFSET(state.v3d);

    GPUTexture **depth_tex = (state.xray_enabled) ? &res.depth_tx : &res.dummy_depth_tx;

    {
      /* Normals */
      const bool use_screen_size = (edit_flag & V3D_OVERLAY_EDIT_CONSTANT_SCREEN_SIZE_NORMALS);
      const bool use_hq_normals = state.scene->r.perf_flag & SCE_PERF_HQ_NORMALS;

      DRWState pass_state = DRW_STATE_WRITE_DEPTH | DRW_STATE_WRITE_COLOR |
                            DRW_STATE_DEPTH_LESS_EQUAL | state.clipping_state;
      if (state.xray_enabled) {
        pass_state |= DRW_STATE_BLEND_ALPHA;
      }

      auto &pass = edit_mesh_normals_ps_;
      pass.init();
      pass.state_set(pass_state);

      auto shader_pass = [&](GPUShader *shader, const char *name) {
        auto &sub = pass.sub(name);
        sub.shader_set(shader);
        sub.bind_ubo("globalsBlock", &res.globals_buf);
        sub.bind_texture("depthTex", depth_tex);
        sub.push_constant("alpha", backwire_opacity);
        sub.push_constant("isConstantScreenSizeNormals", use_screen_size);
        sub.push_constant("normalSize", state.overlay.normals_length);
        sub.push_constant("normalScreenSize", state.overlay.normals_constant_screen_size);
        sub.push_constant("retopologyOffset", retopology_offset);
        sub.push_constant("hq_normals", use_hq_normals);
        return &sub;
      };

      face_normals_ = loop_normals_ = vert_normals_ = vert_normals_ = nullptr;

      if (show_face_nor) {
        face_normals_subdiv_ = shader_pass(res.shaders.mesh_face_normal_subdiv.get(), "SubdFNor");
        face_normals_ = shader_pass(res.shaders.mesh_face_normal.get(), "FaceNor");
      }
      if (show_loop_nor) {
        loop_normals_subdiv_ = shader_pass(res.shaders.mesh_loop_normal_subdiv.get(), "SubdLNor");
        loop_normals_ = shader_pass(res.shaders.mesh_loop_normal.get(), "LoopNor");
      }
      if (show_vert_nor) {
        vert_normals_ = shader_pass(res.shaders.mesh_vert_normal.get(), "VertexNor");
      }
    }
    {
      auto &pass = edit_mesh_analysis_ps_;
      pass.init();
      pass.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_DEPTH_LESS_EQUAL | DRW_STATE_BLEND_ALPHA |
                     state.clipping_state);
      pass.shader_set(res.shaders.mesh_analysis.get());
      pass.bind_texture("weightTex", res.weight_ramp_tx);
    }
  }

  void edit_object_sync(Manager &manager, const ObjectRef &ob_ref, Resources & /*res*/)
  {
    ResourceHandle res_handle = manager.resource_handle(ob_ref);

    Object *ob = ob_ref.object;
    Mesh &mesh = *static_cast<Mesh *>(ob->data);
    /* WORKAROUND: GPU subdiv uses a different normal format. Remove this once GPU subdiv is
     * refactored. */
    const bool use_gpu_subdiv = BKE_subsurf_modifier_has_gpu_subdiv(static_cast<Mesh *>(ob->data));

    bool draw_as_solid = (ob->dt > OB_WIRE);

    if (show_mesh_analysis) {
      gpu::Batch *geom = DRW_cache_mesh_surface_mesh_analysis_get(ob);
      edit_mesh_analysis_ps_.draw(geom, res_handle);
    }
    if (face_normals_) {
      gpu::Batch *geom = DRW_mesh_batch_cache_get_edit_facedots(mesh);
      (use_gpu_subdiv ? face_normals_subdiv_ : face_normals_)
          ->draw_expand(geom, GPU_PRIM_LINES, 1, 1, res_handle);
    }
    if (loop_normals_) {
      gpu::Batch *geom = DRW_mesh_batch_cache_get_edit_loop_normals(mesh);
      (use_gpu_subdiv ? loop_normals_subdiv_ : loop_normals_)
          ->draw_expand(geom, GPU_PRIM_LINES, 1, 1, res_handle);
    }
    if (vert_normals_) {
      gpu::Batch *geom = DRW_mesh_batch_cache_get_edit_vert_normals(mesh);
      vert_normals_->draw_expand(geom, GPU_PRIM_LINES, 1, 1, res_handle);
    }
  }

  void draw(Framebuffer &framebuffer, Manager &manager, View &view)
  {
    GPU_framebuffer_bind(framebuffer);
    manager.submit(edit_mesh_analysis_ps_, view);
    manager.submit(edit_mesh_normals_ps_, view);
  }
};
}  // namespace blender::draw::overlay
