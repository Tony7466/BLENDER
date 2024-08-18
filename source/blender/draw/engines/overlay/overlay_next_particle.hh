/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "DNA_particle_types.h"

#include "BKE_pointcache.h"

#include "overlay_next_private.hh"

namespace blender::draw::overlay {

class Particles {
 private:
  PassSimple particle_ps_ = {"particle_ps_"};
  PassSimple::Sub *dot_ps_ = nullptr;
  PassSimple::Sub *shape_ps_ = nullptr;

  PassSimple edit_particle_ps_ = {"edit_particle_ps_"};

 public:
  void begin_sync(Resources &res, const State &state)
  {
    auto &pass = particle_ps_;
    pass.init();
    pass.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS_EQUAL |
                   state.clipping_state);
    res.select_bind(pass);
    {
      auto &sub = pass.sub("Dots");
      sub.shader_set(res.shaders.particle_dot.get());
      sub.bind_ubo("globalsBlock", &res.globals_buf);
      sub.bind_texture("weightTex", res.weight_ramp_tx);
      dot_ps_ = &sub;
    }
    {
      auto &sub = pass.sub("Shapes");
      sub.shader_set(res.shaders.particle_shape.get());
      sub.bind_ubo("globalsBlock", &res.globals_buf);
      sub.bind_texture("weightTex", res.weight_ramp_tx);
      shape_ps_ = &sub;
    }
  }

  void object_sync(const ObjectRef &ob_ref, Resources &res, const State & /*state*/)
  {
    Object *ob = ob_ref.object;

    LISTBASE_FOREACH (ParticleSystem *, psys, &ob->particlesystem) {
      if (!DRW_object_is_visible_psys_in_active_context(ob, psys)) {
        continue;
      }

      const ParticleSettings *part = psys->part;
      const int draw_as = (part->draw_as == PART_DRAW_REND) ? part->ren_as : part->draw_as;

      if (part->type == PART_HAIR) {
        /* Hairs should have been rendered by the render engine. */
        continue;
      }

      auto set_color = [&](PassSimple::Sub &sub) {
        /* NOTE(fclem): Is color even useful in our modern context? */
        Material *ma = BKE_object_material_get_eval(ob, part->omat);
        sub.push_constant("ucolor", float4(ma ? float3(&ma->r) : float3(0.6f), part->draw_size));
      };
      auto resource_handle_get = [&]() {
        /* TODO(fclem): Create a handle per object with unit transform matrix. */
        return ResourceHandle(0);
      };

      blender::gpu::Batch *geom = nullptr;
      switch (draw_as) {
        case PART_DRAW_NOT:
          /* Nothing to draw. */
          break;
        case PART_DRAW_OB:
        case PART_DRAW_GR:
          /* Instances are realized by Depsgraph and rendered as a regular object instance. */
          break;
        default:
          /* Eventually, would be good to assert. But there are many other draw type that could be
           * set and they need to revert to PART_DRAW_DOT. */
          // BLI_assert_unreachable();
        case PART_DRAW_DOT:
          geom = DRW_cache_particles_get_dots(ob, psys);
          set_color(*dot_ps_);
          dot_ps_->draw(geom, resource_handle_get(), res.select_id(ob_ref).get());
          break;
        case PART_DRAW_AXIS:
          geom = DRW_cache_particles_get_dots(ob, psys);
          set_color(*shape_ps_);
          shape_ps_->push_constant("shape_type", draw_as);
          shape_ps_->draw_expand(
              geom, GPU_PRIM_LINES, 3, 1, resource_handle_get(), res.select_id(ob_ref).get());
          break;
        case PART_DRAW_CIRC:
          geom = DRW_cache_particles_get_dots(ob, psys);
          set_color(*shape_ps_);
          shape_ps_->push_constant("shape_type", draw_as);
          shape_ps_->draw_expand(
              geom, GPU_PRIM_LINES, 16, 1, resource_handle_get(), res.select_id(ob_ref).get());
          break;
        case PART_DRAW_CROSS:
          geom = DRW_cache_particles_get_dots(ob, psys);
          set_color(*shape_ps_);
          shape_ps_->push_constant("shape_type", draw_as);
          shape_ps_->draw_expand(
              geom, GPU_PRIM_LINES, 3, 1, resource_handle_get(), res.select_id(ob_ref).get());
          break;
      }
    }
  }

  void draw(Framebuffer &framebuffer, Manager &manager, View &view)
  {
    GPU_framebuffer_bind(framebuffer);
    manager.submit(particle_ps_, view);
  }
};
}  // namespace blender::draw::overlay
