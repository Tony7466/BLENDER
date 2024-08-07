/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "DNA_object_force_types.h"

#include "overlay_next_private.hh"

namespace blender::draw::overlay {

class ForceFields {
  using ForceFieldsInstanceBuf = ShapeInstanceBuf<ExtraInstanceData>;

 private:
  PassSimple ps_ = {"ForceFields"};

  struct CallBuffers {
    const SelectionType selection_type_;

    ForceFieldsInstanceBuf field_force_buf = {selection_type_, "field_force_buf"};
    ForceFieldsInstanceBuf field_wind_buf = {selection_type_, "field_wind_buf"};
    ForceFieldsInstanceBuf field_vortex_buf = {selection_type_, "field_vortex_buf"};
  } call_buffers_;

 public:
  ForceFields(const SelectionType selection_type) : call_buffers_{selection_type} {}

  void begin_sync()
  {
    call_buffers_.field_force_buf.clear();
    call_buffers_.field_wind_buf.clear();
    call_buffers_.field_vortex_buf.clear();
  }

  void object_sync(const ObjectRef &ob_ref, Resources &res, const State &state)
  {
    const select::ID select_id = res.select_id(ob_ref);
    const Object *ob = ob_ref.object;
    PartDeflect *pd = ob->pd;
    Curve *cu = (ob->type == OB_CURVES_LEGACY) ? static_cast<Curve *>(ob->data) : nullptr;

    ExtraInstanceData data(
        ob->object_to_world(), res.object_background_blend_color(ob_ref, state), 1.0f);
    float4x4 &matrix = data.object_to_world_;
    float &size_x = matrix[0][3];
    float &size_y = matrix[1][3];
    float &size_z = matrix[2][3];
    float3 &pos = matrix.location();

    size_x = size_y = size_z = ob->empty_drawsize;

    switch (pd->forcefield) {
      case PFIELD_FORCE:
        call_buffers_.field_force_buf.append(data, select_id);
        break;
      case PFIELD_WIND:
        size_z = pd->f_strength;
        call_buffers_.field_wind_buf.append(data, select_id);
        break;
      case PFIELD_VORTEX:
        size_y = (pd->f_strength < 0.0f) ? -size_y : size_y;
        call_buffers_.field_vortex_buf.append(data, select_id);
        break;
        // case PFIELD_GUIDE:
        //   if (cu && (cu->flag & CU_PATH) && ob->runtime->curve_cache->anim_path_accum_length) {
        //     instdata.size_x = instdata.size_y = instdata.size_z = pd->f_strength;
        //     float pos[4];
        //     BKE_where_on_path(ob, 0.0f, pos, nullptr, nullptr, nullptr, nullptr);
        //     copy_v3_v3(instdata.pos, ob->object_to_world().location());
        //     translate_m4(instdata.mat, pos[0], pos[1], pos[2]);
        //     DRW_buffer_add_entry(cb->field_curve, color, &instdata);

        //     BKE_where_on_path(ob, 1.0f, pos, nullptr, nullptr, nullptr, nullptr);
        //     copy_v3_v3(instdata.pos, ob->object_to_world().location());
        //     translate_m4(instdata.mat, pos[0], pos[1], pos[2]);
        //     DRW_buffer_add_entry(cb->field_sphere_limit, color, &instdata);
        //     /* Restore */
        //     copy_v3_v3(instdata.pos, ob->object_to_world().location());
        //   }
        //   break;
    }

    // if (pd->falloff == PFIELD_FALL_TUBE) {
    //   if (pd->flag & (PFIELD_USEMAX | PFIELD_USEMAXR)) {
    //     instdata.size_z = (pd->flag & PFIELD_USEMAX) ? pd->maxdist : 0.0f;
    //     instdata.size_x = (pd->flag & PFIELD_USEMAXR) ? pd->maxrad : 1.0f;
    //     instdata.size_y = instdata.size_x;
    //     DRW_buffer_add_entry(cb->field_tube_limit, color, &instdata);
    //   }
    //   if (pd->flag & (PFIELD_USEMIN | PFIELD_USEMINR)) {
    //     instdata.size_z = (pd->flag & PFIELD_USEMIN) ? pd->mindist : 0.0f;
    //     instdata.size_x = (pd->flag & PFIELD_USEMINR) ? pd->minrad : 1.0f;
    //     instdata.size_y = instdata.size_x;
    //     DRW_buffer_add_entry(cb->field_tube_limit, color, &instdata);
    //   }
    // }
    // else if (pd->falloff == PFIELD_FALL_CONE) {
    //   if (pd->flag & (PFIELD_USEMAX | PFIELD_USEMAXR)) {
    //     float radius = DEG2RADF((pd->flag & PFIELD_USEMAXR) ? pd->maxrad : 1.0f);
    //     float distance = (pd->flag & PFIELD_USEMAX) ? pd->maxdist : 0.0f;
    //     instdata.size_x = distance * sinf(radius);
    //     instdata.size_z = distance * cosf(radius);
    //     instdata.size_y = instdata.size_x;
    //     DRW_buffer_add_entry(cb->field_cone_limit, color, &instdata);
    //   }
    //   if (pd->flag & (PFIELD_USEMIN | PFIELD_USEMINR)) {
    //     float radius = DEG2RADF((pd->flag & PFIELD_USEMINR) ? pd->minrad : 1.0f);
    //     float distance = (pd->flag & PFIELD_USEMIN) ? pd->mindist : 0.0f;
    //     instdata.size_x = distance * sinf(radius);
    //     instdata.size_z = distance * cosf(radius);
    //     instdata.size_y = instdata.size_x;
    //     DRW_buffer_add_entry(cb->field_cone_limit, color, &instdata);
    //   }
    // }
    // else if (pd->falloff == PFIELD_FALL_SPHERE) {
    //   if (pd->flag & PFIELD_USEMAX) {
    //     instdata.size_x = instdata.size_y = instdata.size_z = pd->maxdist;
    //     DRW_buffer_add_entry(cb->field_sphere_limit, color, &instdata);
    //   }
    //   if (pd->flag & PFIELD_USEMIN) {
    //     instdata.size_x = instdata.size_y = instdata.size_z = pd->mindist;
    //     DRW_buffer_add_entry(cb->field_sphere_limit, color, &instdata);
    //   }
    // }
  }

  void end_sync(Resources &res, ShapeCache &shapes, const State &state)
  {
    ps_.init();
    res.select_bind(ps_);
    ps_.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS_EQUAL |
                  state.clipping_state);
    ps_.shader_set(res.shaders.extra_shape.get());
    ps_.bind_ubo("globalsBlock", &res.globals_buf);

    call_buffers_.field_force_buf.end_sync(ps_, shapes.field_force.get());
    call_buffers_.field_wind_buf.end_sync(ps_, shapes.field_wind.get());
    call_buffers_.field_vortex_buf.end_sync(ps_, shapes.field_vortex.get());
  }

  void draw(Framebuffer &framebuffer, Manager &manager, View &view)
  {
    GPU_framebuffer_bind(framebuffer);
    manager.submit(ps_, view);
  }
};

}  // namespace blender::draw::overlay
