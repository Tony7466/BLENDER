/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "overlay_next_private.hh"

namespace blender::draw::overlay {

template<typename T> struct BatchInstanceBuf : public ShapeInstanceBuf<T> {
  GPUBatch *batch;

  BatchInstanceBuf(const char *name, GPUBatch *batch, const SelectionType selection_type)
      : ShapeInstanceBuf<T>(selection_type, name), batch(batch){};

  BatchInstanceBuf(const char *name,
                   GPUBatch *batch,
                   const SelectionType selection_type,
                   Vector<BatchInstanceBuf<T> *> &pass_vector)
      : BatchInstanceBuf(name, batch, selection_type)
  {
    pass_vector.append(this);
  };

  void end_sync(PassSimple::Sub &pass)
  {
    ShapeInstanceBuf<T>::end_sync(pass, batch);
  }
};

using ExtraInstanceBuf = BatchInstanceBuf<ExtraInstanceData>;

using GroundLineInstanceBuf = BatchInstanceBuf<float4>;

struct PointInstanceBuf : public ShapeInstanceBuf<float4> {
  float4 color;

  PointInstanceBuf(const char *name, float4 color, const SelectionType selection_type)
      : ShapeInstanceBuf<float4>(selection_type, name), color(color){};

  PointInstanceBuf(const char *name,
                   float4 color,
                   const SelectionType selection_type,
                   Vector<PointInstanceBuf *> &pass_vector)
      : PointInstanceBuf(name, color, selection_type)
  {
    pass_vector.append(this);
  };

  void end_sync(PassSimple::Sub &pass)
  {
    if (data_buf.size() == 0) {
      return;
    }
    this->select_bind(pass);
    data_buf.push_update();
    pass.bind_ssbo("data_buf", &data_buf);
    pass.push_constant("ucolor", color);
    pass.draw_procedural(GPU_PRIM_POINTS, data_buf.size(), 1);
  }
};

struct LineInstanceBuf : public ShapeInstanceBuf<LineInstanceData> {
  float4 color;

  LineInstanceBuf(const char *name, float4 color, const SelectionType selection_type)
      : ShapeInstanceBuf<LineInstanceData>(selection_type, name), color(color){};

  LineInstanceBuf(const char *name,
                  float4 color,
                  const SelectionType selection_type,
                  Vector<LineInstanceBuf *> &pass_vector)
      : LineInstanceBuf(name, color, selection_type)
  {
    pass_vector.append(this);
  };

  void end_sync(PassSimple::Sub &pass)
  {
    if (data_buf.size() == 0) {
      return;
    }
    this->select_bind(pass);
    data_buf.push_update();
    pass.bind_ssbo("data_buf", &data_buf);
    pass.push_constant("ucolor", color);
    pass.draw_procedural(GPU_PRIM_LINES, data_buf.size(), 2);
  }
};

class ExtraInstancePass {
  const SelectionType selection_type;
  const ShapeCache &shapes;
  const GlobalsUboStorage &theme_colors;

  PassSimple ps_;

  enum ExtraType { DEFAULT, DEFAULT_ALWAYS, BLEND_CULL_FRONT, BLEND_CULL_BACK, MAX };
  Vector<ExtraInstanceBuf *> extra_buffers_[ExtraType::MAX] = {{}};
  ExtraInstanceBuf extra_buf(const char *name,
                             const BatchPtr &shape_ptr,
                             ExtraType pass_type = DEFAULT)
  {
    Vector<ExtraInstanceBuf *> &vector = extra_buffers_[pass_type];
    return {name, shape_ptr.get(), selection_type, vector};
  };

  Vector<PointInstanceBuf *> point_buffers_ = {};
  PointInstanceBuf point_buf(const char *name, float4 color)
  {
    return {name, color, selection_type, point_buffers_};
  }

  Vector<LineInstanceBuf *> line_buffers_ = {};
  LineInstanceBuf line_buf(const char *name, float4 color)
  {
    return {name, color, selection_type, line_buffers_};
  }

 public:
  ExtraInstancePass(SelectionType selection_type,
                    const ShapeCache &shapes,
                    const GlobalsUboStorage &theme_colors,
                    const char *name)
      : selection_type(selection_type), shapes(shapes), theme_colors(theme_colors), ps_(name){};

  ExtraInstanceBuf plain_axes = extra_buf("plain_axes", shapes.plain_axes);
  ExtraInstanceBuf single_arrow = extra_buf("single_arrow", shapes.single_arrow);
  ExtraInstanceBuf arrows = extra_buf("arrows", shapes.arrows);
  ExtraInstanceBuf image = extra_buf("image", shapes.quad_wire);
  ExtraInstanceBuf circle = extra_buf("circle", shapes.circle);
  ExtraInstanceBuf cube = extra_buf("cube", shapes.empty_cube);
  ExtraInstanceBuf sphere = extra_buf("sphere", shapes.empty_sphere);
  ExtraInstanceBuf cone = extra_buf("cone", shapes.empty_cone);
  ExtraInstanceBuf cylinder = extra_buf("cylinder", shapes.empty_cylinder);
  ExtraInstanceBuf capsule_body = extra_buf("capsule_body", shapes.empty_capsule_body);
  ExtraInstanceBuf capsule_cap = extra_buf("capsule_cap", shapes.empty_capsule_cap);

  ExtraInstanceBuf sphere_solid = extra_buf("quad", shapes.sphere_solid);
  ExtraInstanceBuf quad = extra_buf("quad", shapes.quad);

  ExtraInstanceBuf speaker = extra_buf("speaker", shapes.speaker);

  GroundLineInstanceBuf groundline = {"groundline", shapes.groundline.get(), selection_type};

  ExtraInstanceBuf light_icon_inner = extra_buf("light_icon_inner", shapes.light_icon_inner);
  ExtraInstanceBuf light_icon_outer = extra_buf("light_icon_outer", shapes.light_icon_outer);
  ExtraInstanceBuf light_icon_sun_rays = extra_buf("light_icon_sun_rays",
                                                   shapes.light_icon_sun_rays);
  ExtraInstanceBuf light_point = extra_buf("light_point", shapes.light_point);
  ExtraInstanceBuf light_sun = extra_buf("light_sun", shapes.light_sun);
  ExtraInstanceBuf light_spot = extra_buf("light_spot", shapes.light_spot);
  ExtraInstanceBuf light_spot_cone_back = extra_buf(
      "light_spot_cone_back", shapes.light_spot_cone, BLEND_CULL_BACK);
  ExtraInstanceBuf light_spot_cone_front = extra_buf(
      "light_spot_cone_front", shapes.light_spot_cone, BLEND_CULL_FRONT);
  ExtraInstanceBuf light_area_disk = extra_buf("light_area_disk", shapes.light_area_disk);
  ExtraInstanceBuf light_area_square = extra_buf("light_area_square", shapes.light_area_square);

  ExtraInstanceBuf probe_cube = extra_buf("probe_cube", shapes.probe_cube);
  ExtraInstanceBuf probe_grid = extra_buf("probe_grid", shapes.probe_grid);
  ExtraInstanceBuf probe_planar = extra_buf("probe_planar", shapes.probe_planar);

  ExtraInstanceBuf camera_volume = extra_buf(
      "camera_volume", shapes.camera_volume, BLEND_CULL_BACK);
  ExtraInstanceBuf camera_volume_wire = extra_buf(
      "camera_volume_wire", shapes.camera_volume_wire, BLEND_CULL_BACK);
  ExtraInstanceBuf camera_frame = extra_buf("camera_frame", shapes.camera_frame);
  ExtraInstanceBuf camera_distances = extra_buf("camera_distances", shapes.camera_distances);
  ExtraInstanceBuf camera_tria_wire = extra_buf("camera_tria_wire", shapes.camera_tria_wire);
  ExtraInstanceBuf camera_tria = extra_buf("camera_tria", shapes.camera_tria);

  ExtraInstanceBuf field_wind = extra_buf("field_wind", shapes.field_wind);
  ExtraInstanceBuf field_force = extra_buf("field_force", shapes.field_force);
  ExtraInstanceBuf field_vortex = extra_buf("field_vortex", shapes.field_vortex);
  ExtraInstanceBuf field_curve = extra_buf("field_curve", shapes.field_curve);
  ExtraInstanceBuf field_tube_limit = extra_buf("field_tube_limit", shapes.field_tube_limit);
  ExtraInstanceBuf field_cone_limit = extra_buf("field_cone_limit", shapes.field_cone_limit);
  ExtraInstanceBuf field_sphere_limit = extra_buf("field_sphere_limit", shapes.field_sphere_limit);

  ExtraInstanceBuf origin_xform = extra_buf("origin_xform", shapes.plain_axes, DEFAULT_ALWAYS);

  PointInstanceBuf center_active = point_buf("center_active", theme_colors.color_active);
  PointInstanceBuf center_selected = point_buf("center_selected", theme_colors.color_select);
  PointInstanceBuf center_deselected = point_buf("center_deselected", theme_colors.color_deselect);
  PointInstanceBuf center_selected_lib = point_buf("center_selected_lib",
                                                   theme_colors.color_library_select);
  PointInstanceBuf center_deselected_lib = point_buf("center_deselected_lib",
                                                     theme_colors.color_library);

  PointInstanceBuf relation_point = point_buf("relation_point", theme_colors.color_wire);
  LineInstanceBuf relation_line = line_buf("relation_line", theme_colors.color_wire);

  LineInstanceBuf constraint_line = line_buf("constraint_line", theme_colors.color_grid_axis_z);

  void begin_sync()
  {
    for (Vector<ExtraInstanceBuf *> &vector : extra_buffers_) {
      for (ExtraInstanceBuf *buf : vector) {
        buf->clear();
      }
    }

    groundline.clear();

    for (PointInstanceBuf *buf : point_buffers_) {
      buf->clear();
    }

    for (LineInstanceBuf *buf : line_buffers_) {
      buf->clear();
    }
  }

  void end_sync(Resources &res, const State &state)
  {
    ps_.init();
    res.select_bind(ps_);

    auto sub_pass =
        [&](const char *name, ShaderPtr &shader, DRWState drw_state) -> PassSimple::Sub * {
      PassSimple::Sub &ps = ps_.sub(name);
      ps.state_set(drw_state);
      ps.shader_set(shader.get());
      /* TODO: Fixed index. */
      ps.bind_ubo("globalsBlock", &res.globals_buf);
      return &ps;
    };

    auto sub_pass_iter = [&](const char *name, auto iter, ShaderPtr &shader, DRWState drw_state) {
      PassSimple::Sub *ps = sub_pass(name, shader, drw_state);
      for (auto *buf : iter) {
        buf->end_sync(*ps);
      }
    };

    DRWState state_base = DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH | state.clipping_state;
    DRWState state_default = state_base | DRW_STATE_DEPTH_LESS_EQUAL;
    DRWState state_blend = state_default | DRW_STATE_BLEND_ALPHA;
    DRWState state_point = state_blend | DRW_STATE_PROGRAM_POINT_SIZE;

    sub_pass_iter("Default", extra_buffers_[DEFAULT], res.shaders.extra_shape, state_default);
    sub_pass_iter("Default Always",
                  extra_buffers_[DEFAULT_ALWAYS],
                  res.shaders.extra_shape,
                  state_base | DRW_STATE_DEPTH_ALWAYS);
    sub_pass_iter("Blend Cull Back",
                  extra_buffers_[BLEND_CULL_BACK],
                  res.shaders.extra_shape,
                  state_blend | DRW_STATE_CULL_BACK);
    sub_pass_iter("Blend Cull Front",
                  extra_buffers_[BLEND_CULL_FRONT],
                  res.shaders.extra_shape,
                  state_blend | DRW_STATE_CULL_FRONT);

    groundline.end_sync(*sub_pass("GroundLine", res.shaders.extra_groundline, state_blend));

    sub_pass_iter("Point", point_buffers_, res.shaders.extra_point, state_point);

    sub_pass_iter("Line", line_buffers_, res.shaders.extra_line, state_blend);
  }

  void draw(Manager &manager, View &view, Framebuffer &fb)
  {
    fb.bind();
    manager.submit(ps_, view);
  }

  ExtraInstanceBuf &empty_buf(int empty_drawtype)
  {
    switch (empty_drawtype) {
      case OB_PLAINAXES:
        return plain_axes;
      case OB_SINGLE_ARROW:
        return single_arrow;
      case OB_CUBE:
        return cube;
      case OB_CIRCLE:
        return circle;
      case OB_EMPTY_SPHERE:
        return sphere;
      case OB_EMPTY_CONE:
        return cone;
      case OB_ARROWS:
        return arrows;
      case OB_EMPTY_IMAGE:
        /* This only show the frame. See OVERLAY_image_empty_cache_populate() for the image. */
        return image;
      default:
        BLI_assert_unreachable();
        return plain_axes;
    }
  }
};

}  // namespace blender::draw::overlay
