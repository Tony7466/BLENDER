/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "draw_cache_impl.hh"
#include "draw_common_c.hh"
#include "overlay_next_private.hh"

#include "ED_lattice.hh"

namespace blender::draw::overlay {

class Meshes {
 private:
  PassSimple edit_mesh_analysis_ps_ = {"edit_mesh_analysis"};

  bool show_retopology = false;
  bool show_mesh_analysis = false;
  bool show_face_nor = false;
  bool show_vert_nor = false;
  bool show_loop_nor = false;

 public:
  void begin_sync(Resources &res, const State &state)
  {
    int edit_flag = state.v3d->overlay.edit_flag;
    bool show_retopology = (state.xray_enabled == false) &&
                           (edit_flag & V3D_OVERLAY_EDIT_RETOPOLOGY) != 0;
    bool show_mesh_analysis = (edit_flag & V3D_OVERLAY_EDIT_STATVIS) != 0;
    bool show_face_nor = (edit_flag & V3D_OVERLAY_EDIT_FACE_NORMALS) != 0;
    bool show_vert_nor = (edit_flag & V3D_OVERLAY_EDIT_VERT_NORMALS) != 0;
    bool show_loop_nor = (edit_flag & V3D_OVERLAY_EDIT_LOOP_NORMALS) != 0;

    const DRWState pass_state = DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH |
                                DRW_STATE_DEPTH_LESS_EQUAL | state.clipping_state;

    {
      auto &pass = edit_mesh_analysis_ps_;
      pass.init();
      pass.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_DEPTH_LESS_EQUAL | DRW_STATE_BLEND_ALPHA);
      pass.shader_set(res.shaders.mesh_analysis.get());
      pass.bind_texture("weightTex", res.weight_ramp_tx);
    }
  }

  void edit_object_sync(Manager &manager, const ObjectRef &ob_ref, Resources &res)
  {
    ResourceHandle res_handle = manager.resource_handle(ob_ref);

    Object *ob = ob_ref.object;
    bool draw_as_solid = (ob->dt > OB_WIRE);

    if (show_mesh_analysis) {
      gpu::Batch *geom = DRW_cache_mesh_surface_mesh_analysis_get(ob);
      if (geom) {
        edit_mesh_analysis_ps_.draw(geom, res_handle);
      }
    }
  }

  void draw(Framebuffer &framebuffer, Manager &manager, View &view)
  {
    GPU_framebuffer_bind(framebuffer);
    manager.submit(edit_mesh_analysis_ps_, view);
  }
};
}  // namespace blender::draw::overlay
