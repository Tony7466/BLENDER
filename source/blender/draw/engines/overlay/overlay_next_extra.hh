/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "overlay_next_private.hh"

#include "DNA_lightprobe_types.h"

namespace blender::draw::overlay {

class Extras {
  using InstanceBuf = ShapeInstanceBuf<ExtraInstanceData>;

 private:
  const SelectionType selection_type_;

  PassSimple empty_ps_ = {"Extras"};
  PassSimple empty_in_front_ps_ = {"Extras_In_front"};

  struct InstanceBuffers {
    const SelectionType selection_type_;
    InstanceBuf plain_axes = {selection_type_, "plain_axes_buf"};
    InstanceBuf single_arrow = {selection_type_, "single_arrow_buf"};
    InstanceBuf arrows = {selection_type_, "arrows_buf"};
    InstanceBuf image = {selection_type_, "image_buf"};
    InstanceBuf cube = {selection_type_, "cube_buf"};
    InstanceBuf circle = {selection_type_, "circle_buf"};
    InstanceBuf sphere = {selection_type_, "sphere_buf"};
    InstanceBuf cone = {selection_type_, "cone_buf"};
    InstanceBuf speaker = {selection_type_, "speaker_buf"};
    InstanceBuf probe_cube = {selection_type_, "probe_cube_buf"};
    InstanceBuf probe_grid = {selection_type_, "probe_grid_buf"};
    InstanceBuf probe_planar = {selection_type_, "probe_planar_buf"};
  } buffers_[2] = {{selection_type_}, {selection_type_}};

 public:
  Extras(const SelectionType selection_type) : selection_type_(selection_type){};

  void begin_sync()
  {
    for (InstanceBuffers &bufs : buffers_) {
      bufs.plain_axes.clear();
      bufs.single_arrow.clear();
      bufs.arrows.clear();
      bufs.image.clear();
      bufs.cube.clear();
      bufs.circle.clear();
      bufs.sphere.clear();
      bufs.cone.clear();
      bufs.speaker.clear();
      bufs.probe_cube.clear();
      bufs.probe_grid.clear();
      bufs.probe_planar.clear();
    }
  }

  void object_sync(const ObjectRef &ob_ref, Resources &res, const State &state)
  {
    InstanceBuffers &bufs = buffers_[int((ob_ref.object->dtx & OB_DRAW_IN_FRONT) != 0)];

    float4 color = res.object_wire_color(ob_ref, state);
    float size = ob_ref.object->type == OB_EMPTY ? ob_ref.object->empty_drawsize : 1.0f;
    ExtraInstanceData data(float4x4(ob_ref.object->object_to_world), color, size);

    const select::ID select_id = res.select_id(ob_ref);

    switch (ob_ref.object->type) {
      case OB_EMPTY:
        empty_sync(bufs, ob_ref, data, select_id);
        break;
      case OB_LIGHTPROBE:
        probe_sync(bufs, ob_ref, data, select_id);
        break;
      case OB_SPEAKER:
        bufs.speaker.append(data, select_id);
        break;
    }
  }

  void end_sync(Resources &res, ShapeCache &shapes, const State &state)
  {
    auto init_pass = [&](PassSimple &pass, InstanceBuffers &bufs) {
      pass.init();
      pass.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS_EQUAL |
                     state.clipping_state);
      pass.shader_set(res.shaders.extra_shape.get());
      pass.bind_ubo("globalsBlock", &res.globals_buf);
      res.select_bind(pass);

      bufs.plain_axes.end_sync(pass, shapes.plain_axes.get());
      bufs.single_arrow.end_sync(pass, shapes.single_arrow.get());
      bufs.arrows.end_sync(pass, shapes.arrows.get());
      bufs.image.end_sync(pass, shapes.quad_wire.get());
      bufs.circle.end_sync(pass, shapes.circle.get());
      bufs.cube.end_sync(pass, shapes.empty_cube.get());
      bufs.sphere.end_sync(pass, shapes.empty_sphere.get());
      bufs.cone.end_sync(pass, shapes.empty_cone.get());
      bufs.speaker.end_sync(pass, shapes.speaker.get());
      bufs.probe_cube.end_sync(pass, shapes.probe_cube.get());
      bufs.probe_grid.end_sync(pass, shapes.probe_grid.get());
      bufs.probe_planar.end_sync(pass, shapes.probe_planar.get());
    };
    init_pass(empty_ps_, buffers_[0]);
    init_pass(empty_in_front_ps_, buffers_[1]);
  }

  void draw(Resources &res, Manager &manager, View &view)
  {
    GPU_framebuffer_bind(res.overlay_line_fb);
    manager.submit(empty_ps_, view);
  }

  void draw_in_front(Resources &res, Manager &manager, View &view)
  {
    GPU_framebuffer_bind(res.overlay_line_in_front_fb);
    manager.submit(empty_in_front_ps_, view);
  }

 private:
  void empty_sync(InstanceBuffers &bufs,
                  const ObjectRef &ob_ref,
                  const ExtraInstanceData data,
                  const select::ID select_id)
  {
    switch (ob_ref.object->empty_drawtype) {
      case OB_PLAINAXES:
        bufs.plain_axes.append(data, select_id);
        break;
      case OB_SINGLE_ARROW:
        bufs.single_arrow.append(data, select_id);
        break;
      case OB_CUBE:
        bufs.cube.append(data, select_id);
        break;
      case OB_CIRCLE:
        bufs.circle.append(data, select_id);
        break;
      case OB_EMPTY_SPHERE:
        bufs.sphere.append(data, select_id);
        break;
      case OB_EMPTY_CONE:
        bufs.cone.append(data, select_id);
        break;
      case OB_ARROWS:
        bufs.arrows.append(data, select_id);
        break;
      case OB_EMPTY_IMAGE:
        /* This only show the frame. See OVERLAY_image_empty_cache_populate() for the image. */
        bufs.image.append(data, select_id);
        break;
    }
  }

  void probe_sync(InstanceBuffers &bufs,
                  const ObjectRef &ob_ref,
                  const ExtraInstanceData data,
                  const select::ID select_id)
  {
    const LightProbe *probe = (LightProbe *)ob_ref.object->data;
    const bool show_clipping = (probe->flag & LIGHTPROBE_FLAG_SHOW_CLIP_DIST) != 0;
    const bool show_parallax = (probe->flag & LIGHTPROBE_FLAG_SHOW_PARALLAX) != 0;
    const bool show_influence = (probe->flag & LIGHTPROBE_FLAG_SHOW_INFLUENCE) != 0;
    const bool show_data = (ob_ref.object->base_flag & BASE_SELECTED) ||
                           selection_type_ != SelectionType::DISABLED;

    if (probe->type == LIGHTPROBE_TYPE_CUBE) {
      ExtraInstanceData _data = data;
      _data.object_to_world_[2][3] = show_clipping ? probe->clipsta : -1.0;
      _data.object_to_world_[3][3] = show_clipping ? probe->clipend : -1.0;

      buffers_->probe_cube.append(_data, select_id);

      /* TODO(Miguel Pozo) */
      // DRW_buffer_add_entry(cb->groundline, data.object_to_world_.location());

      if (show_influence) {
        float influence_start = probe->distinf * (1.0f - probe->falloff);
        float influence_end = probe->distinf;

        InstanceBuf &buf = (probe->attenuation_type == LIGHTPROBE_SHAPE_BOX) ? bufs.cube :
                                                                               bufs.sphere;

        buf.append(ExtraInstanceData(data.object_to_world_, data.color_, influence_start),
                   select_id);
        buf.append(ExtraInstanceData(data.object_to_world_, data.color_, influence_end),
                   select_id);
      }

      if (show_parallax) {
        float radius = (probe->flag & LIGHTPROBE_FLAG_CUSTOM_PARALLAX) ? probe->distpar :
                                                                         probe->distinf;
        InstanceBuf &buf = (probe->parallax_type == LIGHTPROBE_SHAPE_BOX) ? bufs.cube :
                                                                            bufs.sphere;
        buf.append(ExtraInstanceData(data.object_to_world_, data.color_, radius), select_id);
      }
    }
    else if (probe->type == LIGHTPROBE_TYPE_GRID) {
      ExtraInstanceData _data = data;
      _data.object_to_world_[2][3] = show_clipping ? probe->clipsta : -1.0;
      _data.object_to_world_[3][3] = show_clipping ? probe->clipend : -1.0;
      buffers_->probe_grid.append(_data, select_id);

      if (show_influence) {
        float influence_start = 1.0f + probe->distinf * (1.0f - probe->falloff);
        float influence_end = 1.0f + probe->distinf;

        bufs.cube.append(ExtraInstanceData(data.object_to_world_, data.color_, influence_start),
                         select_id);
        bufs.cube.append(ExtraInstanceData(data.object_to_world_, data.color_, influence_end),
                         select_id);
      }

      /* TODO(Miguel Pozo) */
#if 0
    /* Data dots */
    if (show_data) {
      _data.object_to_world_[0][3] = probe->grid_resolution_x;
      _data.object_to_world_[1][3] = probe->grid_resolution_y;
      _data.object_to_world_[2][3] = probe->grid_resolution_z;
      /* Put theme id in matrix. */
      if (theme_id == TH_ACTIVE) {
        _data.object_to_world_[3][3] = 1.0;
      }
      else /* TH_SELECT */ {
        _data.object_to_world_[3][3] = 2.0;
      }

      uint cell_count = probe->grid_resolution_x * probe->grid_resolution_y *
                        probe->grid_resolution_z;
      DRWShadingGroup *grp = DRW_shgroup_create_sub(vedata->stl->pd->extra_grid_grp);
      DRW_shgroup_uniform_mat4_copy(grp, "gridModelMatrix", instdata.mat);
      DRW_shgroup_call_procedural_points(grp, nullptr, cell_count);
    }
#endif
    }
    else if (probe->type == LIGHTPROBE_TYPE_PLANAR) {
      buffers_->probe_planar.append(data, select_id);

      if (selection_type_ != SelectionType::DISABLED && (probe->flag & LIGHTPROBE_FLAG_SHOW_DATA))
      {
        /* TODO(Miguel Pozo) */
        // bufs.solid_quad.append(data, select_id);
      }

      ExtraInstanceData _data = data;
      float3 &z = _data.object_to_world_.z_axis();

      if (show_influence) {
        z = math::normalize(z) * probe->distinf;
        bufs.cube.append(_data, select_id);
        z *= 1.0f - probe->falloff;
        bufs.cube.append(_data, select_id);
      }

      z = float3(0);
      bufs.cube.append(_data, select_id);

      _data = data;
      _data.object_to_world_ = math::normalize(_data.object_to_world_);
      _data.object_to_world_[3] = data.object_to_world_[3];
      bufs.single_arrow.append(
          ExtraInstanceData(_data.object_to_world_, data.color_, ob_ref.object->empty_drawsize),
          select_id);
    }
  }
};

}  // namespace blender::draw::overlay
