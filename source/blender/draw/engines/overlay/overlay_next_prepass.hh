/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 *
 * A depth pass that write surface depth when it is needed.
 * It is also used for selecting non overlay-only objects.
 */

#pragma once

#include "overlay_next_private.hh"

namespace blender::draw::overlay {

class Prepass {
 private:
  const SelectionType selection_type_;

  PassMain ps_ = {"prepass"};
  PassMain::Sub *mesh_ps_ = nullptr;
  PassMain::Sub *hair_ps_ = nullptr;
  PassMain::Sub *curves_ps_ = nullptr;
  PassMain::Sub *point_cloud_ps_ = nullptr;

  bool enabled = false;

 public:
  Prepass(const SelectionType selection_type) : selection_type_(selection_type){};

  void begin_sync(Resources &res, const State &state)
  {
    enabled = !state.xray_enabled || (selection_type_ != SelectionType::DISABLED);
    enabled &= state.space_type == SPACE_VIEW3D;

    if (!enabled) {
      /* Not used. But release the data. */
      ps_.init();
      mesh_ps_ = nullptr;
      curves_ps_ = nullptr;
      point_cloud_ps_ = nullptr;
      return;
    }

    const View3DShading &shading = state.v3d->shading;
    bool use_cull = ((shading.type == OB_SOLID) && (shading.flag & V3D_SHADING_BACKFACE_CULLING));
    DRWState backface_cull_state = use_cull ? DRW_STATE_CULL_BACK : DRWState(0);

    ps_.init();
    ps_.state_set(DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS_EQUAL | backface_cull_state,
                  state.clipping_plane_count);
    res.select_bind(ps_);
    {
      auto &sub = ps_.sub("Mesh");
      sub.shader_set(res.shaders.depth_mesh.get());
      sub.bind_ubo("globalsBlock", &res.globals_buf);
      mesh_ps_ = &sub;
    }
    {
      auto &sub = ps_.sub("Hair");
      sub.shader_set(res.shaders.depth_mesh.get());
      sub.bind_ubo("globalsBlock", &res.globals_buf);
      hair_ps_ = &sub;
    }
    {
      auto &sub = ps_.sub("Curves");
      sub.shader_set(res.shaders.depth_curves.get());
      sub.bind_ubo("globalsBlock", &res.globals_buf);
      curves_ps_ = &sub;
    }
    {
      auto &sub = ps_.sub("PointCloud");
      sub.shader_set(res.shaders.depth_point_cloud.get());
      sub.bind_ubo("globalsBlock", &res.globals_buf);
      point_cloud_ps_ = &sub;
    }
  }

  void particle_sync(Manager &manager, const ObjectRef &ob_ref, Resources &res, const State &state)
  {
    Object *ob = ob_ref.object;

    ResourceHandle handle = {0};

    LISTBASE_FOREACH (ParticleSystem *, psys, &ob->particlesystem) {
      if (!DRW_object_is_visible_psys_in_active_context(ob, psys)) {
        continue;
      }

      const ParticleSettings *part = psys->part;
      const int draw_as = (part->draw_as == PART_DRAW_REND) ? part->ren_as : part->draw_as;
      switch (draw_as) {
        case PART_DRAW_PATH:
          if ((state.is_wireframe_mode == false) && (part->draw_as == PART_DRAW_REND)) {
            /* Case where the render engine should have rendered it, but we need to draw it for
             * selection purpose. */
            if (handle.raw == 0u) {
              handle = manager.resource_handle_for_psys(ob_ref,
                                                        Particles::dupli_matrix_get(ob_ref));
            }
            gpu::Batch *geom = DRW_cache_particles_get_hair(ob, psys, nullptr);
            mesh_ps_->draw(geom, handle, res.select_id(ob_ref).get());
            break;
          }
          break;
        default:
          /* Other draw modes should be handled by the particle overlay. */
          break;
      }
    }
  }

  void object_sync(Manager &manager, const ObjectRef &ob_ref, Resources &res, const State &state)
  {
    if (!enabled) {
      return;
    }

    if (ob_ref.object->dt < OB_SOLID) {
      return;
    }

    particle_sync(manager, ob_ref, res, state);

    /* TODO(fclem) This function should contain what `basic_cache_populate` contained. */

    gpu::Batch *geom = nullptr;
    PassMain::Sub *pass = nullptr;
    switch (ob_ref.object->type) {
      case OB_MESH:
        geom = DRW_cache_mesh_surface_get(ob_ref.object);
        pass = mesh_ps_;
        break;
      case OB_VOLUME:
        if (selection_type_ == SelectionType::DISABLED) {
          /* Disable during display, only enable for selection. */
          /* TODO(fclem): Would be nice to have even when not selecting to occlude overlays. */
          return;
        }
        geom = DRW_cache_volume_selection_surface_get(ob_ref.object);
        pass = mesh_ps_;
        break;
      case OB_POINTCLOUD:
        geom = point_cloud_sub_pass_setup(*point_cloud_ps_, ob_ref.object);
        pass = point_cloud_ps_;
        break;
      case OB_CURVES:
        geom = curves_sub_pass_setup(*curves_ps_, state.scene, ob_ref.object);
        pass = curves_ps_;
        break;
      default:
        break;
    }

    if (geom) {
      ResourceHandle res_handle = manager.resource_handle(ob_ref);
      pass->draw(geom, res_handle, res.select_id(ob_ref).get());
    }
  }

  void draw(Framebuffer &framebuffer, Manager &manager, View &view)
  {
    if (!enabled) {
      return;
    }
    /* Should be fine to use the line buffer since the prepass only writes to the depth buffer. */
    GPU_framebuffer_bind(framebuffer);
    manager.submit(ps_, view);
  }
};

}  // namespace blender::draw::overlay
