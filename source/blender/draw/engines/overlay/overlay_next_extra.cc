/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "overlay_next_private.hh"

namespace blender::draw::overlay {

void Extras::begin_sync()
{
  for (int i = 0; i < 2; i++) {
    call_buffers_[i].plain_axes.clear();
    call_buffers_[i].single_arrow.clear();
    call_buffers_[i].cube.clear();
    call_buffers_[i].circle.clear();
    call_buffers_[i].sphere.clear();
    call_buffers_[i].cone.clear();
    call_buffers_[i].arrows.clear();
    call_buffers_[i].image.clear();
    call_buffers_[i].speaker.clear();
    call_buffers_[i].probe_cube.clear();
    call_buffers_[i].probe_grid.clear();
    call_buffers_[i].probe_planar.clear();
  }
}

void Extras::object_sync(const ObjectRef &ob_ref, Resources &res, const State &state)
{
  CallBuffers &call_bufs = call_buffers_[int((ob_ref.object->dtx & OB_DRAW_IN_FRONT) != 0)];

  float4 color = res.object_wire_color(ob_ref, state);
  float size = ob_ref.object->type == OB_EMPTY ? ob_ref.object->empty_drawsize : 1.0f;
  ExtraInstanceData data(float4x4(ob_ref.object->object_to_world), color, size);

  const select::ID select_id = res.select_id(ob_ref);

  switch (ob_ref.object->type) {
    case OB_EMPTY:
      empty_sync(call_bufs, ob_ref, data, select_id);
      break;
    case OB_LIGHTPROBE:
      break;
    case OB_SPEAKER:
      call_bufs.speaker.append(data, select_id);
      break;
  }
}

void Extras::empty_sync(CallBuffers &call_bufs,
                        const ObjectRef &ob_ref,
                        const ExtraInstanceData data,
                        const select::ID select_id)
{
  switch (ob_ref.object->empty_drawtype) {
    case OB_PLAINAXES:
      call_bufs.plain_axes.append(data, select_id);
      break;
    case OB_SINGLE_ARROW:
      call_bufs.single_arrow.append(data, select_id);
      break;
    case OB_CUBE:
      call_bufs.cube.append(data, select_id);
      break;
    case OB_CIRCLE:
      call_bufs.circle.append(data, select_id);
      break;
    case OB_EMPTY_SPHERE:
      call_bufs.sphere.append(data, select_id);
      break;
    case OB_EMPTY_CONE:
      call_bufs.cone.append(data, select_id);
      break;
    case OB_ARROWS:
      call_bufs.arrows.append(data, select_id);
      break;
    case OB_EMPTY_IMAGE:
      /* This only show the frame. See OVERLAY_image_empty_cache_populate() for the image. */
      call_bufs.image.append(data, select_id);
      break;
  }
}

void Extras::probe_sync() {}

void Extras::end_sync(Resources &res, ShapeCache &shapes, const State &state)
{
  auto init_pass = [&](PassSimple &pass, CallBuffers &call_bufs) {
    pass.init();
    pass.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS_EQUAL |
                   state.clipping_state);
    pass.shader_set(res.shaders.extra_shape.get());
    pass.bind_ubo("globalsBlock", &res.globals_buf);
    res.select_bind(pass);

    call_bufs.plain_axes.end_sync(pass, shapes.plain_axes.get());
    call_bufs.single_arrow.end_sync(pass, shapes.single_arrow.get());
    call_bufs.cube.end_sync(pass, shapes.cube.get());
    call_bufs.circle.end_sync(pass, shapes.circle.get());
    call_bufs.sphere.end_sync(pass, shapes.empty_sphere.get());
    call_bufs.cone.end_sync(pass, shapes.empty_cone.get());
    call_bufs.arrows.end_sync(pass, shapes.arrows.get());
    call_bufs.image.end_sync(pass, shapes.quad_wire.get());
    call_bufs.speaker.end_sync(pass, shapes.speaker.get());
  };
  init_pass(empty_ps_, call_buffers_[0]);
  init_pass(empty_in_front_ps_, call_buffers_[1]);
}

void Extras::draw(Resources &res, Manager &manager, View &view)
{
  GPU_framebuffer_bind(res.overlay_line_fb);
  manager.submit(empty_ps_, view);
}

void Extras::draw_in_front(Resources &res, Manager &manager, View &view)
{
  GPU_framebuffer_bind(res.overlay_line_in_front_fb);
  manager.submit(empty_in_front_ps_, view);
}

}  // namespace blender::draw::overlay
