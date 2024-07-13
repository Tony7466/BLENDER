/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "BKE_mball.hh"

#include "BLI_bounds_types.hh"
#include "BLI_utildefines.h"

#include "overlay_next_private.hh"

namespace blender::draw::overlay {
class Bounds {
  using BoundsInstanceBuf = ShapeInstanceBuf<ExtraInstanceData>;

 private:
  const SelectionType selection_type_;

  PassSimple ps_ = {"Bounds"};
  PassSimple in_front_ps_ = {"Bounds_in_front"};

  struct CallBuffers {
    const SelectionType selection_type_;

    BoundsInstanceBuf box = {selection_type_, "bound_box"};
    BoundsInstanceBuf sphere = {selection_type_, "bound_sphere"};
    BoundsInstanceBuf cylinder = {selection_type_, "bound_cylinder"};
    BoundsInstanceBuf cone = {selection_type_, "bound_cone"};
    BoundsInstanceBuf capsule_body = {selection_type_, "bound_capsule_body"};
    BoundsInstanceBuf capsule_cap = {selection_type_, "bound_capsule_cap"};
  };
  std::array<CallBuffers, 2> call_buffers_{CallBuffers{selection_type_},
                                           CallBuffers{selection_type_}};

 public:
  Bounds(const SelectionType selection_type) : selection_type_(selection_type) {}

  void begin_sync()
  {
    for (CallBuffers &call_buffers : call_buffers_) {
      call_buffers.box.clear();
      call_buffers.sphere.clear();
      call_buffers.cylinder.clear();
      call_buffers.cone.clear();
      call_buffers.capsule_body.clear();
      call_buffers.capsule_cap.clear();
    }
  }

  void object_sync(const ObjectRef &ob_ref, Resources &res, const State &state)
  {
    const bool around_origin = false;
    const Object *ob = ob_ref.object;
    const bool from_dupli = (ob->base_flag & (BASE_FROM_SET | BASE_FROM_DUPLI)) != 0;
    const bool has_bounds = !ELEM(
        ob->type, OB_LAMP, OB_CAMERA, OB_EMPTY, OB_SPEAKER, OB_LIGHTPROBE);
    const bool draw_bounds = has_bounds && ((ob->dt == OB_BOUNDBOX) ||
                                            ((ob->dtx & OB_DRAWBOUNDOX) && !from_dupli));
    if (!draw_bounds || (ob_ref.object->type == OB_MBALL && !BKE_mball_is_basis(ob))) {
      return;
    }
    const float4x4 object_mat{ob->object_to_world().ptr()};
    const blender::Bounds<float3> bounds = BKE_object_boundbox_get(ob).value_or(
        blender::Bounds(float3(-1.0f), float3(1.0f)));

    const float4 color = res.object_wire_color(ob_ref, state);
    const float3 size = (bounds.max - bounds.min) * 0.5f;
    const float3 center = around_origin ? float3(0) : math::midpoint(bounds.min, bounds.max);
    const select::ID select_id = res.select_id(ob_ref);

    CallBuffers &call_bufs = call_buffers_[int((ob_ref.object->dtx & OB_DRAW_IN_FRONT) != 0)];

    switch (ob->boundtype) {
      case OB_BOUND_BOX: {
        float4x4 scale = math::from_scale<float4x4>(size);
        scale.location() = center;
        ExtraInstanceData data(object_mat * scale, color, 1.0f);
        call_bufs.box.append(data, select_id);
        break;
      }
      case OB_BOUND_SPHERE: {
        float4x4 scale = math::from_scale<float4x4>(float3{math::reduce_max(size)});
        scale.location() = center;
        ExtraInstanceData data(object_mat * scale, color, 1.0f);
        call_bufs.sphere.append(data, select_id);
        break;
      }
      case OB_BOUND_CYLINDER: {
        float4x4 scale = math::from_scale<float4x4>(
            float3{float2{math::max(size.x, size.y)}, size.z});
        scale.location() = center;
        ExtraInstanceData data(object_mat * scale, color, 1.0f);
        call_bufs.cylinder.append(data, select_id);
        break;
      }
      case OB_BOUND_CONE: {
        float4x4 mat = math::from_scale<float4x4>(
            float3{float2{math::max(size.x, size.y)}, size.z});
        mat.location() = center;
        /* Cone batch has base at 0 and is pointing towards +Y. */
        std::swap(mat[1], mat[2]);
        mat.location().z -= size.z;
        ExtraInstanceData data(object_mat * mat, color, 1.0f);
        call_bufs.cone.append(data, select_id);
        break;
      }
      case OB_BOUND_CAPSULE: {
        float4x4 mat = math::from_scale<float4x4>(float3{math::max(size.x, size.y)});
        mat.location() = center;
        mat.location().z = center.z + std::max(0.0f, size.z - size.x);
        ExtraInstanceData data(object_mat * mat, color, 1.0f);
        call_bufs.capsule_cap.append(data, select_id);
        mat.z_axis() *= -1;
        mat.location().z = center.z - std::max(0.0f, size.z - size.x);
        data.object_to_world_ = object_mat * mat;
        call_bufs.capsule_cap.append(data, select_id);
        mat.z_axis().z = std::max(0.0f, size.z * 2.0f - size.x * 2.0f);
        data.object_to_world_ = object_mat * mat;
        call_bufs.capsule_body.append(data, select_id);
        break;
      }
    }
  }

  void end_sync(Resources &res, ShapeCache &shapes, const State &state)
  {
    auto init_pass = [&](PassSimple &pass, CallBuffers &call_bufs) {
      pass.init();
      pass.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS_EQUAL |
                     state.clipping_state);
      pass.shader_set(res.shaders.extra_shape.get());
      pass.bind_ubo("globalsBlock", &res.globals_buf);
      res.select_bind(pass);

      call_bufs.box.end_sync(pass, shapes.cube.get());
      call_bufs.sphere.end_sync(pass, shapes.empty_sphere.get());
      call_bufs.cylinder.end_sync(pass, shapes.cylinder.get());
      call_bufs.cone.end_sync(pass, shapes.empty_cone.get());
      call_bufs.capsule_body.end_sync(pass, shapes.capsule_body.get());
      call_bufs.capsule_cap.end_sync(pass, shapes.capsule_cap.get());
    };

    init_pass(ps_, call_buffers_[0]);
    init_pass(in_front_ps_, call_buffers_[1]);
  }

  void draw(Resources &res, Manager &manager, View &view)
  {
    GPU_framebuffer_bind(res.overlay_line_fb);
    manager.submit(ps_, view);
  }

  void draw_in_front(Resources &res, Manager &manager, View &view)
  {
    GPU_framebuffer_bind(res.overlay_line_in_front_fb);
    manager.submit(in_front_ps_, view);
  }
};
}  // namespace blender::draw::overlay
