/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "overlay_next_private.hh"

namespace blender::draw::overlay {

struct ExtraInstanceBuf : public ShapeInstanceBuf<ExtraInstanceData> {
  GPUBatch *shape;

  ExtraInstanceBuf(const char *name,
                   GPUBatch *shape,
                   const SelectionType selection_type,
                   Vector<ExtraInstanceBuf *> &pass_vector)
      : ShapeInstanceBuf<ExtraInstanceData>(selection_type, name), shape(shape)
  {
    pass_vector.append(this);
  };
};

class ExtraInstancePassesBase {
 protected:
  enum PassType {
    DEFAULT = 0,
    DEFAULT_ALWAYS,
    BLEND_CULL_FRONT,
    BLEND_CULL_BACK,
    GROUNDLINE,
    WIRE,
    WIRE_OB,
    POINT,
    LOOSE_POINT,
    CENTER,
    MAX
  };

  const SelectionType selection_type;
  const ShapeCache &shapes;

  PassSimple pass;
  Vector<ExtraInstanceBuf *> pass_buffers[PassType::MAX] = {{}};

 public:
  ExtraInstancePassesBase(SelectionType selection_type, const ShapeCache &shapes, const char *name)
      : selection_type(selection_type), shapes(shapes), pass(name){};

  void begin_sync()
  {
    for (Vector<ExtraInstanceBuf *> &vector : pass_buffers) {
      for (ExtraInstanceBuf *buf : vector) {
        buf->clear();
      }
    }
  }

  void end_sync(Resources &res, const State &state)
  {
    pass.init();
    res.select_bind(pass);

    auto sub_pass = [&](const char *name, PassType type, ShaderPtr &shader, DRWState drw_state) {
      if (shader == nullptr) {
        /*TODO*/
        return;
      }
      PassSimple::Sub &ps = pass.sub(name);
      ps.state_set(drw_state);
      ps.shader_set(shader.get());
      /* TODO: Fixed index. */
      ps.bind_ubo("globalsBlock", &res.globals_buf);
      for (ExtraInstanceBuf *buf : pass_buffers[type]) {
        buf->end_sync(ps, buf->shape);
      }
    };

    DRWState state_base = DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH | state.clipping_state;
    DRWState state_default = state_base | DRW_STATE_DEPTH_LESS_EQUAL;
    DRWState state_blend = state_default | DRW_STATE_BLEND_ALPHA;

    sub_pass("Default", DEFAULT, res.shaders.extra_shape, state_default);
    sub_pass("Default Always",
             DEFAULT_ALWAYS,
             res.shaders.extra_shape,
             state_base | DRW_STATE_DEPTH_ALWAYS);
    sub_pass("Blend Cull Back",
             BLEND_CULL_BACK,
             res.shaders.extra_shape,
             state_blend | DRW_STATE_CULL_BACK);
    sub_pass("Blend Cull Front",
             BLEND_CULL_FRONT,
             res.shaders.extra_shape,
             state_blend | DRW_STATE_CULL_FRONT);
    sub_pass("GroundLine", GROUNDLINE, res.shaders.extra_groundline, state_blend);
    sub_pass("Wire", WIRE, res.shaders.extra_wire, state_default);
    sub_pass("Wire Object", WIRE_OB, res.shaders.extra_wire_object, state_default);
    sub_pass("Point", POINT, res.shaders.extra_point, state_default);
    sub_pass("Loose Point", LOOSE_POINT, res.shaders.extra_loose_point, state_default);
    sub_pass("Center", CENTER, res.shaders.extra_point, state_blend);
  }

  void draw(Manager &manager, View &view, Framebuffer &fb)
  {
    fb.bind();
    manager.submit(pass, view);
  }
};

class ExtraInstancePasses : public ExtraInstancePassesBase {

  ExtraInstanceBuf make_buf(const char *name,
                            const BatchPtr &shape_ptr,
                            PassType pass_type = DEFAULT)
  {
    Vector<ExtraInstanceBuf *> &vector = pass_buffers[pass_type];
    return {name, shape_ptr.get(), selection_type, vector};
  };

 public:
  ExtraInstancePasses(SelectionType selection_type, const ShapeCache &shapes, const char *name)
      : ExtraInstancePassesBase(selection_type, shapes, name){};

  ExtraInstanceBuf plain_axes = make_buf("plain_axes", shapes.plain_axes);
  ExtraInstanceBuf single_arrow = make_buf("single_arrow", shapes.single_arrow);
  ExtraInstanceBuf arrows = make_buf("arrows", shapes.arrows);
  ExtraInstanceBuf image = make_buf("image", shapes.quad_wire);
  ExtraInstanceBuf circle = make_buf("circle", shapes.circle);
  ExtraInstanceBuf cube = make_buf("cube", shapes.empty_cube);
  ExtraInstanceBuf sphere = make_buf("sphere", shapes.empty_sphere);
  ExtraInstanceBuf cone = make_buf("cone", shapes.empty_cone);
  ExtraInstanceBuf cylinder = make_buf("cylinder", shapes.empty_cylinder);
  ExtraInstanceBuf capsule_body = make_buf("capsule_body", shapes.empty_capsule_body);
  ExtraInstanceBuf capsule_cap = make_buf("capsule_cap", shapes.empty_capsule_cap);

  ExtraInstanceBuf sphere_solid = make_buf("quad", shapes.sphere_solid);
  ExtraInstanceBuf quad = make_buf("quad", shapes.quad);

  ExtraInstanceBuf speaker = make_buf("speaker", shapes.speaker);

  ExtraInstanceBuf groundline = make_buf("groundline", shapes.groundline, GROUNDLINE);
  ExtraInstanceBuf light_icon_inner = make_buf("light_icon_inner", shapes.light_icon_inner);
  ExtraInstanceBuf light_icon_outer = make_buf("light_icon_outer", shapes.light_icon_outer);
  ExtraInstanceBuf light_icon_sun_rays = make_buf("light_icon_sun_rays",
                                                  shapes.light_icon_sun_rays);
  ExtraInstanceBuf light_point = make_buf("light_point", shapes.light_point);
  ExtraInstanceBuf light_sun = make_buf("light_sun", shapes.light_sun);
  ExtraInstanceBuf light_spot = make_buf("light_spot", shapes.light_spot);
  ExtraInstanceBuf light_spot_cone_back = make_buf(
      "light_spot_cone_back", shapes.light_spot_cone, BLEND_CULL_BACK);
  ExtraInstanceBuf light_spot_cone_front = make_buf(
      "light_spot_cone_front", shapes.light_spot_cone, BLEND_CULL_FRONT);
  ExtraInstanceBuf light_area_disk = make_buf("light_area_disk", shapes.light_area_disk);
  ExtraInstanceBuf light_area_square = make_buf("light_area_square", shapes.light_area_square);

  ExtraInstanceBuf probe_cube = make_buf("probe_cube", shapes.probe_cube);
  ExtraInstanceBuf probe_grid = make_buf("probe_grid", shapes.probe_grid);
  ExtraInstanceBuf probe_planar = make_buf("probe_planar", shapes.probe_planar);

  ExtraInstanceBuf camera_volume = make_buf(
      "camera_volume", shapes.camera_volume, BLEND_CULL_BACK);
  ExtraInstanceBuf camera_volume_wire = make_buf(
      "camera_volume_wire", shapes.camera_volume_wire, BLEND_CULL_BACK);
  ExtraInstanceBuf camera_frame = make_buf("camera_frame", shapes.camera_frame);
  ExtraInstanceBuf camera_distances = make_buf("camera_distances", shapes.camera_distances);
  ExtraInstanceBuf camera_tria_wire = make_buf("camera_tria_wire", shapes.camera_tria_wire);
  ExtraInstanceBuf camera_tria = make_buf("camera_tria", shapes.camera_tria);

  ExtraInstanceBuf field_wind = make_buf("field_wind", shapes.field_wind);
  ExtraInstanceBuf field_force = make_buf("field_force", shapes.field_force);
  ExtraInstanceBuf field_vortex = make_buf("field_vortex", shapes.field_vortex);
  ExtraInstanceBuf field_curve = make_buf("field_curve", shapes.field_curve);
  ExtraInstanceBuf field_tube_limit = make_buf("field_tube_limit", shapes.field_tube_limit);
  ExtraInstanceBuf field_cone_limit = make_buf("field_cone_limit", shapes.field_cone_limit);
  ExtraInstanceBuf field_sphere_limit = make_buf("field_sphere_limit", shapes.field_sphere_limit);

  ExtraInstanceBuf origin_xform = make_buf("origin_xform", shapes.plain_axes, DEFAULT_ALWAYS);

  /* TODO
  ExtraInstanceBuf dashed_lines = make_buf("dashed_lines", nullptr, WIRE);
  ExtraInstanceBuf lines = make_buf("lines", nullptr, WIRE);

  ExtraInstanceBuf wires = make_buf("wires", nullptr, WIRE_OB);

  ExtraInstanceBuf loose_points = make_buf("loose_points", nullptr, LOOSE_POINTS);

  ExtraInstanceBuf points = make_buf("points", nullptr, POINTS);

  ExtraInstanceBuf center_active = make_buf("center_active", nullptr, CENTER);
  ExtraInstanceBuf center_selected = make_buf("center_selected", nullptr, CENTER);
  ExtraInstanceBuf center_deselected = make_buf("center_deselected", nullptr, CENTER);
  ExtraInstanceBuf center_selected_lib = make_buf("center_selected_lib", nullptr, CENTER);
  ExtraInstanceBuf center_deselected_lib = make_buf("center_deselected_lib", nullptr, CENTER);
  */

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
