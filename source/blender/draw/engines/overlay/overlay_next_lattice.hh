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

class Lattices {
 private:
  PassMain lattice_ps_ = {"Lattice"};
  PassMain lattice_in_front_ps_ = {"Lattice_in_front"};

  PassMain edit_lattice_wire_ps_ = {"Edit_lattice_wire"};
  PassMain edit_lattice_wire_in_front_ps_ = {"Edit_lattice_wire_in_front"};

  PassMain edit_lattice_point_ps_ = {"Edit_lattice_point"};
  PassMain edit_lattice_point_in_front_ps_ = {"Edit_lattice_point_in_front"};

 public:
  void begin_sync(Resources &res, const State &state)
  {
    auto init_pass = [&](PassMain &pass, GPUShader *shader) {
      pass.init();
      pass.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS_EQUAL |
                     state.clipping_state);
      pass.shader_set(shader);
      pass.bind_ubo("globalsBlock", &res.globals_buf);
      /* TODO Add "weightTex" for edit_lattice_wire. */
      res.select_bind(pass);
    };

    init_pass(lattice_ps_, res.shaders.extra_wire_object.get());
    init_pass(lattice_in_front_ps_, res.shaders.extra_wire_object.get());

    init_pass(edit_lattice_point_ps_, res.shaders.lattice_points.get());
    init_pass(edit_lattice_point_in_front_ps_, res.shaders.lattice_points.get());

    init_pass(edit_lattice_wire_ps_, res.shaders.lattice_wire.get());
    init_pass(edit_lattice_wire_in_front_ps_, res.shaders.lattice_wire.get());
  }

  void edit_object_sync(Manager &manager, const ObjectRef &ob_ref, Resources &res)
  {
    {
      PassMain &pass = (ob_ref.object->dtx & OB_DRAW_IN_FRONT) != 0 ?
                           edit_lattice_wire_in_front_ps_ :
                           edit_lattice_wire_ps_;
      gpu::Batch *geom = DRW_cache_lattice_wire_get(ob_ref.object, false);
      if (geom) {
        ResourceHandle res_handle = manager.resource_handle(ob_ref);
        pass.draw(geom, res_handle, res.select_id(ob_ref).get());
      }
    }
    {
      PassMain &pass = (ob_ref.object->dtx & OB_DRAW_IN_FRONT) != 0 ?
                           edit_lattice_point_in_front_ps_ :
                           edit_lattice_point_ps_;
      gpu::Batch *geom = DRW_cache_lattice_vert_overlay_get(ob_ref.object);
      if (geom) {
        ResourceHandle res_handle = manager.resource_handle(ob_ref);
        pass.draw(geom, res_handle, res.select_id(ob_ref).get());
      }
    }
  }

  void object_sync(Manager &manager, const ObjectRef &ob_ref, Resources &res, const State &state)
  {
    PassMain &pass = (ob_ref.object->dtx & OB_DRAW_IN_FRONT) != 0 ? lattice_in_front_ps_ :
                                                                    lattice_ps_;
    gpu::Batch *geom = DRW_cache_lattice_wire_get(ob_ref.object, false);
    if (geom) {
      const float4 &color = res.object_wire_color(ob_ref, state);
      float4x4 draw_mat(ob_ref.object->object_to_world().ptr());
      for (int i : IndexRange(3)) {
        draw_mat[i][3] = color[i];
      }
      draw_mat[3][3] = 0.0f /* No stipples. */;
      ResourceHandle res_handle = manager.resource_handle(ob_ref, &draw_mat, nullptr, nullptr);
      pass.draw(geom, res_handle, res.select_id(ob_ref).get());
    }
  }

  void draw(Resources &res, Manager &manager, View &view)
  {
    GPU_framebuffer_bind(res.overlay_line_fb);
    manager.submit(lattice_ps_, view);
    manager.submit(edit_lattice_wire_ps_, view);
    manager.submit(edit_lattice_point_ps_, view);
  }

  void draw_in_front(Resources &res, Manager &manager, View &view)
  {
    GPU_framebuffer_bind(res.overlay_line_in_front_fb);
    manager.submit(lattice_in_front_ps_, view);
    manager.submit(edit_lattice_wire_in_front_ps_, view);
    manager.submit(edit_lattice_point_in_front_ps_, view);
  }
};
}  // namespace blender::draw::overlay
