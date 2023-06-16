/* SPDX-FileCopyrightText: 2019 Blender Foundation.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "DRW_gpu_wrapper.hh"
#include "DRW_render.h"
#include "UI_resources.h"
#include "draw_manager.hh"
#include "draw_pass.hh"
#include "gpu_shader_create_info.hh"

#include "../select/select_instance.hh"
#include "overlay_shader_shared.h"

/* Needed for BoneInstanceData. */
#include "overlay_private.hh"

namespace blender::draw::overlay {

using SelectionType = select::SelectionType;

using blender::draw::Framebuffer;
using blender::draw::StorageVectorBuffer;
using blender::draw::Texture;
using blender::draw::TextureFromPool;
using blender::draw::TextureRef;

struct State {
  Depsgraph *depsgraph;
  Manager *manager;
  const ViewLayer *view_layer;
  const Scene *scene;
  const View3D *v3d;
  const RegionView3D *rv3d;
  const Base *active_base;
  View3DOverlay overlay;
  float pixelsize;
  SelectionType selection_type;
  enum eSpace_Type space_type;
  enum eContextObjectMode ctx_mode;
  enum eObjectMode object_mode;
  bool clear_in_front;
  bool use_in_front;
  bool is_wireframe_mode;
  bool hide_overlays;
  bool xray_enabled;
  bool xray_enabled_and_not_wire;
  float xray_opacity;
  short v3d_flag;     /* TODO: move to #View3DOverlay. */
  short v3d_gridflag; /* TODO: move to #View3DOverlay. */
  int cfra;
  DRWState clipping_state;
};

struct BatchDeleter {
  void operator()(GPUBatch *shader)
  {
    GPU_BATCH_DISCARD_SAFE(shader);
  }
};
using BatchPtr = std::unique_ptr<GPUBatch, BatchDeleter>;

/**
 * Contains all overlay generic geometry batches.
 */
class ShapeCache {
 public:
  // GPUBatch *drw_normal_arrow; /* Single Arrow? */
  BatchPtr plain_axes;
  BatchPtr single_arrow;
  BatchPtr arrows;
  BatchPtr quad_wire;
  BatchPtr circle;
  BatchPtr empty_cube;
  BatchPtr empty_sphere;
  BatchPtr empty_cone;
  BatchPtr empty_cylinder;
  BatchPtr empty_capsule_body;
  BatchPtr empty_capsule_cap;

  BatchPtr sphere_solid;

  BatchPtr quad;
  BatchPtr grid;

  BatchPtr metaball_wire_circle;

  BatchPtr speaker;

  BatchPtr groundline;
  BatchPtr light_icon_inner;
  BatchPtr light_icon_outer;
  BatchPtr light_icon_sun_rays;
  BatchPtr light_point;
  BatchPtr light_sun;
  BatchPtr light_spot;
  BatchPtr light_spot_cone;
  BatchPtr light_area_disk;
  BatchPtr light_area_square;

  BatchPtr probe_cube;
  BatchPtr probe_grid;
  BatchPtr probe_planar;

  BatchPtr camera_volume;
  BatchPtr camera_volume_wire;
  BatchPtr camera_frame;
  BatchPtr camera_tria_wire;
  BatchPtr camera_tria;
  BatchPtr camera_distances;

  BatchPtr field_wind;
  BatchPtr field_force;
  BatchPtr field_vortex;
  BatchPtr field_curve;
  BatchPtr field_tube_limit;
  BatchPtr field_cone_limit;
  BatchPtr field_sphere_limit;

  ShapeCache();
};

struct ShaderDeleter {
  void operator()(GPUShader *shader)
  {
    DRW_SHADER_FREE_SAFE(shader);
  }
};

using ShaderPtr = std::unique_ptr<GPUShader, ShaderDeleter>;

/**
 * Shader module. Shared between instances.
 */
class ShaderModule {
 private:
  /** Shared shader module across all engine instances. */
  static ShaderModule *g_shader_modules[2 /*Selection Instance*/][2 /*Clipping Enabled*/];

  const SelectionType selection_type_;
  /** TODO: Support clipping. This global state should be set by the overlay::Instance and switch
   * to the shader variations that use clipping. */
  const bool clipping_enabled_;

 public:
  /** Shaders */
  ShaderPtr grid = shader("overlay_grid");
  ShaderPtr background_fill = shader("overlay_background");
  ShaderPtr background_clip_bound = shader("overlay_clipbound");

  /** Selectable Shaders */
  ShaderPtr armature_sphere_outline;
  ShaderPtr depth_mesh;
  ShaderPtr extra_shape;

  ShaderPtr extra_groundline;
  ShaderPtr extra_point;
  ShaderPtr extra_line;
  ShaderPtr extra_grid;

  /* TODO */
  ShaderPtr extra_wire;
  ShaderPtr extra_wire_object;
  ShaderPtr extra_loose_point;

  ShaderModule(const SelectionType selection_type, const bool clipping_enabled);

  /** Module */
  /** Only to be used by Instance constructor. */
  static ShaderModule &module_get(SelectionType selection_type, bool clipping_enabled);
  static void module_free();

 private:
  ShaderPtr shader(const char *create_info_name)
  {
    return ShaderPtr(GPU_shader_create_from_info_name(create_info_name));
  }
  ShaderPtr selectable_shader(const char *create_info_name);
  ShaderPtr selectable_shader(const char *create_info_name,
                              std::function<void(gpu::shader::ShaderCreateInfo &info)> patch);
};

struct Resources : public select::SelectMap {
  ShaderModule &shaders;

  Framebuffer overlay_fb = {"overlay_fb"};
  Framebuffer overlay_in_front_fb = {"overlay_in_front_fb"};
  Framebuffer overlay_color_only_fb = {"overlay_color_only_fb"};
  Framebuffer overlay_line_fb = {"overlay_line_fb"};
  Framebuffer overlay_line_in_front_fb = {"overlay_line_in_front_fb"};

  TextureFromPool line_tx = {"line_tx"};
  TextureFromPool depth_in_front_alloc_tx = {"overlay_depth_in_front_tx"};
  TextureFromPool color_overlay_alloc_tx = {"overlay_color_overlay_alloc_tx"};
  TextureFromPool color_render_alloc_tx = {"overlay_color_render_alloc_tx"};

  /** TODO(fclem): Copy of G_data.block that should become theme colors only and managed by the
   * engine. */
  GlobalsUboStorage theme_settings;
  /* References, not owned. */
  GPUUniformBuf *globals_buf;
  TextureRef depth_tx;
  TextureRef depth_in_front_tx;
  TextureRef color_overlay_tx;
  TextureRef color_render_tx;

  Resources(const SelectionType selection_type_, ShaderModule &shader_module)
      : select::SelectMap(selection_type_), shaders(shader_module){};

  ThemeColorID object_wire_theme_id(const ObjectRef &ob_ref, const State &state) const
  {
    const bool is_edit = (state.object_mode & OB_MODE_EDIT) &&
                         (ob_ref.object->mode & OB_MODE_EDIT);
    const bool active = (state.active_base != nullptr) &&
                        ((ob_ref.dupli_parent != nullptr) ?
                             (state.active_base->object == ob_ref.dupli_parent) :
                             (state.active_base->object == ob_ref.object));
    const bool is_selected = ((ob_ref.object->base_flag & BASE_SELECTED) != 0);

    /* Object in edit mode. */
    if (is_edit) {
      return TH_WIRE_EDIT;
    }
    /* Transformed object during operators. */
    if (((G.moving & G_TRANSFORM_OBJ) != 0) && is_selected) {
      return TH_TRANSFORM;
    }
    /* Sets the 'theme_id' or fallback to wire */
    if ((ob_ref.object->base_flag & BASE_SELECTED) != 0) {
      return (active) ? TH_ACTIVE : TH_SELECT;
    }

    switch (ob_ref.object->type) {
      case OB_LAMP:
        return TH_LIGHT;
      case OB_SPEAKER:
        return TH_SPEAKER;
      case OB_CAMERA:
        return TH_CAMERA;
      case OB_LIGHTPROBE:
        /* TODO: add light-probe color. Use empty color for now. */
      case OB_EMPTY:
        return TH_EMPTY;
      default:
        return (is_edit) ? TH_WIRE_EDIT : TH_WIRE;
    }
  }

  const float4 &object_wire_color(const ObjectRef &ob_ref, ThemeColorID theme_id) const
  {
    if (UNLIKELY(ob_ref.object->base_flag & BASE_FROM_SET)) {
      return theme_settings.color_wire;
    }
    switch (theme_id) {
      case TH_WIRE_EDIT:
        return theme_settings.color_wire_edit;
      case TH_ACTIVE:
        return theme_settings.color_active;
      case TH_SELECT:
        return theme_settings.color_select;
      case TH_TRANSFORM:
        return theme_settings.color_transform;
      case TH_SPEAKER:
        return theme_settings.color_speaker;
      case TH_CAMERA:
        return theme_settings.color_camera;
      case TH_EMPTY:
        return theme_settings.color_empty;
      case TH_LIGHT:
        return theme_settings.color_light;
      default:
        return theme_settings.color_wire;
    }
  }

  const float4 &object_wire_color(const ObjectRef &ob_ref, const State &state) const
  {
    ThemeColorID theme_id = object_wire_theme_id(ob_ref, state);
    return object_wire_color(ob_ref, theme_id);
  }

  float4 background_blend_color(ThemeColorID theme_id) const
  {
    float4 color;
    UI_GetThemeColorBlendShade4fv(theme_id, TH_BACK, 0.5, 0, color);
    return color;
  }

  float4 object_background_blend_color(const ObjectRef &ob_ref, const State &state) const
  {
    ThemeColorID theme_id = object_wire_theme_id(ob_ref, state);
    return background_blend_color(theme_id);
  }
};

/**
 * Buffer containing instances of a certain shape.
 */
template<typename InstanceDataT> struct ShapeInstanceBuf : protected select::SelectBuf {

  StorageVectorBuffer<InstanceDataT> data_buf;

  ShapeInstanceBuf(const SelectionType selection_type, const char *name = nullptr)
      : select::SelectBuf(selection_type), data_buf(name){};

  void clear()
  {
    this->select_clear();
    data_buf.clear();
  }

  void append(const InstanceDataT &data, select::ID select_id)
  {
    this->select_append(select_id);
    data_buf.append(data);
  }

  void end_sync(PassSimple::Sub &pass, GPUBatch *shape)
  {
    if (data_buf.size() == 0) {
      return;
    }
    this->select_bind(pass);
    data_buf.push_update();
    pass.bind_ssbo("data_buf", &data_buf);
    pass.draw(shape, data_buf.size());
  }
};

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

class OverlayPasses {
 protected:
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
  OverlayPasses(const char *name,
                SelectionType selection_type,
                const ShapeCache &shapes,
                const GlobalsUboStorage &theme_colors,
                bool in_front)
      : selection_type(selection_type),
        shapes(shapes),
        theme_colors(theme_colors),
        ps_(in_front ? (name + std::string(" In Front")).c_str() : name){};

  virtual void begin_sync(Resources &res, const State & /*state*/)
  {
    ps_.init();
    res.select_bind(ps_);

    for (Vector<ExtraInstanceBuf *> &vector : extra_buffers_) {
      for (ExtraInstanceBuf *buf : vector) {
        buf->clear();
      }
    }
    for (PointInstanceBuf *buf : point_buffers_) {
      buf->clear();
    }
    for (LineInstanceBuf *buf : line_buffers_) {
      buf->clear();
    }
  };

  virtual void object_sync(const ObjectRef &ob_ref,
                           const select::ID select_id,
                           Resources & /*res*/,
                           const State &state) = 0;

  virtual void end_sync(Resources &res, const State &state)
  {
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
      if (iter.is_empty()) {
        return;
      }
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
    sub_pass_iter("Point", point_buffers_, res.shaders.extra_point, state_point);
    sub_pass_iter("Line", line_buffers_, res.shaders.extra_line, state_blend);
  };

  virtual void draw(Manager &manager, View &view, Framebuffer &fb)
  {
    fb.bind();
    manager.submit(ps_, view);
  }
};

template<typename T> class OverlayType {
  T passes_;
  T passes_in_front_;

 public:
  OverlayType(const SelectionType selection_type,
              const ShapeCache &shapes,
              const GlobalsUboStorage &theme_colors)
      : passes_(selection_type, shapes, theme_colors, false),
        passes_in_front_(selection_type, shapes, theme_colors, true){};

  void begin_sync(Resources &res, const State &state)
  {
    passes_.begin_sync(res, state);
    passes_in_front_.begin_sync(res, state);
  }

  void object_sync(const ObjectRef &ob_ref,
                   const select::ID select_id,
                   Resources &res,
                   const State &state)
  {
    T &passes = ob_ref.object->dtx & OB_DRAW_IN_FRONT ? passes_in_front_ : passes_;
    passes.object_sync(ob_ref, select_id, res, state);
  }

  void end_sync(Resources &res, const State &state)
  {
    passes_.end_sync(res, state);
    passes_in_front_.end_sync(res, state);
  }

  void draw(Resources &res, Manager &manager, View &view)
  {
    passes_.draw(manager, view, res.overlay_line_fb);
  }

  void draw_in_front(Resources &res, Manager &manager, View &view)
  {
    passes_in_front_.draw(manager, view, res.overlay_line_in_front_fb);
  }
};

}  // namespace blender::draw::overlay
