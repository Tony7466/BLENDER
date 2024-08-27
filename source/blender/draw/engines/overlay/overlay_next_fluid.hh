/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "DNA_fluid_types.h"
#include "DNA_modifier_types.h"

#include "BKE_modifier.hh"

#include "overlay_next_private.hh"

namespace blender::draw::overlay {

class Fluids {
 private:
  const SelectionType selection_type_;

  PassSimple fluid_ps_ = {"fluid_ps_"};

  ShapeInstanceBuf<ExtraInstanceData> cube_buf_ = {selection_type_, "cube_buf_"};

  int dominant_axis = -1;

 public:
  Fluids(const SelectionType selection_type) : selection_type_(selection_type){};

  void begin_sync(Resources &res, const State &state)
  {
    // dominant_axis = /* TODO */

    {
      auto &pass = fluid_ps_;
      pass.init();
      pass.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS_EQUAL |
                     state.clipping_state);
      res.select_bind(pass);
    }

    cube_buf_.clear();
  }

  void object_sync(Manager &manager, const ObjectRef &ob_ref, Resources &res, const State &state)
  {
    Object *ob = ob_ref.object;

    /* Do not show for dupli objects as the fluid is baked for the original object. */
    if (ob->base_flag & (BASE_FROM_SET | BASE_FROM_DUPLI)) {
      return;
    }

    /* NOTE: There can only be one fluid modifier per object. */
    ModifierData *md = BKE_modifiers_findby_type(ob, eModifierType_Fluid);

    if (md == nullptr) {
      return;
    }

    FluidModifierData *fmd = (FluidModifierData *)md;
    FluidDomainSettings *fds = fmd->domain;

    if (fds == nullptr) {
      return;
    }

    const bool is_active_frame_after_cache_start = state.scene->r.cfra >= fds->cache_frame_start;
    const bool is_active_frame_before_cache_end = state.scene->r.cfra >= fds->cache_frame_start;
    const bool is_active_frame_in_cache_range = is_active_frame_after_cache_start &&
                                                is_active_frame_before_cache_end;
    if (!is_active_frame_in_cache_range) {
      return;
    }

    /* Small cube showing voxel size. */
    {
      float3 min = float3(fds->p0) + float3(fds->cell_size) * float3(int3(fds->res_min));
      float4x4 voxel_cube_mat = math::from_loc_scale<float4x4>(min, float3(fds->cell_size) / 2.0f);
      /* Move small cube into the domain, otherwise its centered on corner of domain object. */
      voxel_cube_mat = math::translate(voxel_cube_mat, float3(1.0f));
      voxel_cube_mat = ob->object_to_world() * voxel_cube_mat;

      const float4 &color = res.object_wire_color(ob_ref, state);

      cube_buf_.append({voxel_cube_mat, color, 1.0f}, res.select_id(ob_ref));
    }

    /* Don't show smoke before simulation starts, this could be made an option in the future. */
    const bool draw_velocity = (fds->draw_velocity && fds->fluid &&
                                is_active_frame_after_cache_start);

#if 0
    /* Show gridlines only for slices with no interpolation. */
    const bool show_gridlines = (fds->show_gridlines && fds->fluid &&
                                 fds->axis_slice_method == AXIS_SLICE_SINGLE &&
                                 (fds->interp_method == FLUID_DISPLAY_INTERP_CLOSEST ||
                                  fds->coba_field == FLUID_DOMAIN_FIELD_FLAGS));

    const bool color_with_flags = (fds->gridlines_color_field == FLUID_GRIDLINE_COLOR_TYPE_FLAGS);

    const bool color_range = (fds->gridlines_color_field == FLUID_GRIDLINE_COLOR_TYPE_RANGE &&
                              fds->use_coba && fds->coba_field != FLUID_DOMAIN_FIELD_FLAGS);


    int slice_axis = slide_axis_get(*fsd);

    if (draw_velocity) {
      const bool use_needle = (fds->vector_draw_type == VECTOR_DRAW_NEEDLE);
      const bool use_mac = (fds->vector_draw_type == VECTOR_DRAW_MAC);
      const bool draw_mac_x = (fds->vector_draw_mac_components & VECTOR_DRAW_MAC_X);
      const bool draw_mac_y = (fds->vector_draw_mac_components & VECTOR_DRAW_MAC_Y);
      const bool draw_mac_z = (fds->vector_draw_mac_components & VECTOR_DRAW_MAC_Z);
      const bool cell_centered = (fds->vector_field == FLUID_DOMAIN_VECTOR_FIELD_FORCE);
      int line_count = 1;
      if (use_needle) {
        line_count = 6;
      }
      else if (use_mac) {
        line_count = 3;
      }

      line_count *= fds->res[0] * fds->res[1] * fds->res[2];
      if (slice_axis != -1) {
        /* Remove the sliced dimension. */
        line_count /= fds->res[slice_axis];
      }

      DRW_smoke_ensure_velocity(fmd);

      GPUShader *sh = OVERLAY_shader_volume_velocity(use_needle, use_mac);
      DRWShadingGroup *grp = DRW_shgroup_create(sh, data->psl->extra_ps[0]);
      DRW_shgroup_uniform_texture(grp, "velocityX", fds->tex_velocity_x);
      DRW_shgroup_uniform_texture(grp, "velocityY", fds->tex_velocity_y);
      DRW_shgroup_uniform_texture(grp, "velocityZ", fds->tex_velocity_z);
      sub.push_constant("displaySize", fds->vector_scale);                       /* float_copy */
      sub.push_constant("slicePosition", fds->slice_depth);                      /* float_copy */
      sub.push_constant("cellSize", float3(fds->cell_size));                     /* vec3_copy */
      sub.push_constant("domainOriginOffset", float3(fds->p0));                  /* vec3_copy */
      sub.push_constant("adaptiveCellOffset", int3(fds->res_min));               /* ivec3_copy */
      sub.push_constant("sliceAxis", slice_axis);                                /* int_copy */
      sub.push_constant("scaleWithMagnitude", fds->vector_scale_with_magnitude); /* bool_copy */
      sub.push_constant("isCellCentered", cell_centered);                        /* bool_copy */

      if (use_mac) {
        DRW_shgroup_uniform_bool_copy(grp, "drawMACX", draw_mac_x);
        DRW_shgroup_uniform_bool_copy(grp, "drawMACY", draw_mac_y);
        DRW_shgroup_uniform_bool_copy(grp, "drawMACZ", draw_mac_z);
      }

      DRW_shgroup_call_procedural_lines(grp, ob, line_count);
    }

    if (show_gridlines) {
      GPUShader *sh = OVERLAY_shader_volume_gridlines(color_with_flags, color_range);
      DRWShadingGroup *grp = DRW_shgroup_create(sh, data->psl->extra_ps[0]);
      DRW_shgroup_uniform_ivec3_copy(grp, "volumeSize", fds->res);
      DRW_shgroup_uniform_float_copy(grp, "slicePosition", fds->slice_depth);
      DRW_shgroup_uniform_vec3_copy(grp, "cellSize", fds->cell_size);
      DRW_shgroup_uniform_vec3_copy(grp, "domainOriginOffset", fds->p0);
      DRW_shgroup_uniform_ivec3_copy(grp, "adaptiveCellOffset", fds->res_min);
      DRW_shgroup_uniform_int_copy(grp, "sliceAxis", slice_axis);

      if (color_with_flags || color_range) {
        DRW_fluid_ensure_flags(fmd);
        DRW_shgroup_uniform_texture(grp, "flagTexture", fds->tex_flags);
      }

      if (color_range) {
        DRW_fluid_ensure_range_field(fmd);
        DRW_shgroup_uniform_texture(grp, "fieldTexture", fds->tex_range_field);
        DRW_shgroup_uniform_float_copy(grp, "lowerBound", fds->gridlines_lower_bound);
        DRW_shgroup_uniform_float_copy(grp, "upperBound", fds->gridlines_upper_bound);
        DRW_shgroup_uniform_vec4_copy(grp, "rangeColor", fds->gridlines_range_color);
        DRW_shgroup_uniform_int_copy(grp, "cellFilter", fds->gridlines_cell_filter);
      }

      const int line_count = 4 * fds->res[0] * fds->res[1] * fds->res[2] / fds->res[slice_axis];
      DRW_shgroup_call_procedural_lines(grp, ob, line_count);
    }
#endif
  }

  void end_sync(Resources &res, ShapeCache &shapes, const State &state)
  {
    cube_buf_.end_sync(fluid_ps_, shapes.quad_wire.get());
  }

  void draw(Framebuffer &framebuffer, Manager &manager, View &view)
  {
    GPU_framebuffer_bind(framebuffer);
    manager.submit(fluid_ps_, view);
  }

 private:
  /* Return axis index or -1 if no slice. */
  int slide_axis_get(FluidDomainSettings &fluid_domain_settings) const
  {
    if (fluid_domain_settings.axis_slice_method != AXIS_SLICE_SINGLE) {
      return -1;
    }
    if (fluid_domain_settings.slice_axis == SLICE_AXIS_AUTO) {
      return dominant_axis;
    }
    return fluid_domain_settings.slice_axis - 1;
  }
};

}  // namespace blender::draw::overlay
