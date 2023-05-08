/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2019 Blender Foundation. */

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

using eSelectionType = select::eSelectionType;

using blender::draw::Framebuffer;
using blender::draw::StorageVectorBuffer;
using blender::draw::Texture;
using blender::draw::TextureFromPool;
using blender::draw::TextureRef;

struct State {
  Depsgraph *depsgraph;
  const ViewLayer *view_layer;
  const Scene *scene;
  const View3D *v3d;
  const RegionView3D *rv3d;
  const Base *active_base;
  View3DOverlay overlay;
  float pixelsize;
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

/**
 * Shader module. Shared between instances.
 */
class ShaderModule {
 private:
  struct ShaderDeleter {
    void operator()(GPUShader *shader)
    {
      DRW_SHADER_FREE_SAFE(shader);
    }
  };
  using ShaderPtr = std::unique_ptr<GPUShader, ShaderDeleter>;

  /** Shared shader module across all engine instances. */
  static ShaderModule *g_shader_modules[2 /*Selection Instance*/][2 /*Clipping Enabled*/];

  const eSelectionType selection_type_;
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

  ShaderModule(const eSelectionType selection_type, const bool clipping_enabled);

  /** Module */
  /** Only to be used by Instance constructor. */
  static ShaderModule &module_get(eSelectionType selection_type, bool clipping_enabled);
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

  Resources(const eSelectionType selection_type_, ShaderModule &shader_module)
      : select::SelectMap(selection_type_), shaders(shader_module){};

  ThemeColorID object_wire_theme_id(const ObjectRef &ob_ref, const State &state) const;
  const float4 &object_wire_color(const ObjectRef &ob_ref, ThemeColorID theme_id) const;
  const float4 &object_wire_color(const ObjectRef &ob_ref, const State &state) const;
};

/**
 * Buffer containing instances of a certain shape.
 */
template<typename InstanceDataT> struct ShapeInstanceBuf : private select::SelectBuf {

  StorageVectorBuffer<InstanceDataT> data_buf;

  ShapeInstanceBuf(const eSelectionType selection_type, const char *name = nullptr)
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

  void end_sync(PassSimple &pass, GPUBatch *shape)
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

/**
 * Contains all overlay generic geometry batches.
 */
class ShapeCache {
 private:
  struct BatchDeleter {
    void operator()(GPUBatch *shader)
    {
      GPU_BATCH_DISCARD_SAFE(shader);
    }
  };
  using BatchPtr = std::unique_ptr<GPUBatch, BatchDeleter>;

 public:
  BatchPtr quad_wire;
  BatchPtr plain_axes;
  BatchPtr single_arrow;
  BatchPtr cube;
  BatchPtr circle;
  BatchPtr empty_sphere;
  BatchPtr empty_cone;
  BatchPtr arrows;
  BatchPtr metaball_wire_circle;

  ShapeCache();
};

class Prepass {
 private:
  const eSelectionType selection_type_;

  PassMain prepass_ps_ = {"prepass"};
  PassMain prepass_in_front_ps_ = {"prepass_in_front"};

 public:
  Prepass(const eSelectionType selection_type) : selection_type_(selection_type){};

  void begin_sync(Resources &res, const State &state);
  void object_sync(Manager &manager, const ObjectRef &ob_ref, Resources &res);
  void draw(Resources &res, Manager &manager, View &view);
  void draw_in_front(Resources &res, Manager &manager, View &view);
};

class Background {
 private:
  const eSelectionType selection_type_;

  PassSimple bg_ps_ = {"Background"};

 public:
  Background(const eSelectionType selection_type) : selection_type_(selection_type){};

  void begin_sync(Resources &res, const State &state);
  void draw(Resources &res, Manager &manager);
};

class Grid {
 private:
  const eSelectionType selection_type_;

  UniformBuffer<OVERLAY_GridData> data_;

  PassSimple grid_ps_ = {"grid_ps_"};

  float3 grid_axes_ = float3(0.0f);
  float3 zplane_axes_ = float3(0.0f);
  OVERLAY_GridBits grid_flag_ = OVERLAY_GridBits(0);
  OVERLAY_GridBits zneg_flag_ = OVERLAY_GridBits(0);
  OVERLAY_GridBits zpos_flag_ = OVERLAY_GridBits(0);

  bool enabled_ = false;

 public:
  Grid(const eSelectionType selection_type) : selection_type_(selection_type){};

  void begin_sync(Resources &res, const State &state, const View &view);
  void draw(Resources &res, Manager &manager, View &view);

 private:
  void update_ubo(const State &state, const View &view);
};

class Empties {
  using EmptyInstanceBuf = ShapeInstanceBuf<ExtraInstanceData>;

 private:
  const eSelectionType selection_type_;

  PassSimple empty_ps_ = {"Empties"};
  PassSimple empty_in_front_ps_ = {"Empties_In_front"};

  struct CallBuffers {
    const eSelectionType selection_type_;
    EmptyInstanceBuf plain_axes_buf = {selection_type_, "plain_axes_buf"};
    EmptyInstanceBuf single_arrow_buf = {selection_type_, "single_arrow_buf"};
    EmptyInstanceBuf cube_buf = {selection_type_, "cube_buf"};
    EmptyInstanceBuf circle_buf = {selection_type_, "circle_buf"};
    EmptyInstanceBuf sphere_buf = {selection_type_, "sphere_buf"};
    EmptyInstanceBuf cone_buf = {selection_type_, "cone_buf"};
    EmptyInstanceBuf arrows_buf = {selection_type_, "arrows_buf"};
    EmptyInstanceBuf image_buf = {selection_type_, "image_buf"};
  } call_buffers_[2] = {{selection_type_}, {selection_type_}};

 public:
  Empties(const eSelectionType selection_type) : selection_type_(selection_type){};

  void begin_sync();
  void object_sync(const ObjectRef &ob_ref, Resources &res, const State &state);
  void end_sync(Resources &res, ShapeCache &shapes, const State &state);
  void draw(Resources &res, Manager &manager, View &view);
  void draw_in_front(Resources &res, Manager &manager, View &view);
};

class Metaballs {
  using SphereOutlineInstanceBuf = ShapeInstanceBuf<BoneInstanceData>;

 private:
  const eSelectionType selection_type_;

  PassSimple metaball_ps_ = {"MetaBalls"};
  PassSimple metaball_in_front_ps_ = {"MetaBalls_In_front"};

  SphereOutlineInstanceBuf circle_buf_ = {selection_type_, "metaball_data_buf"};
  SphereOutlineInstanceBuf circle_in_front_buf_ = {selection_type_, "metaball_data_buf"};

 public:
  Metaballs(const eSelectionType selection_type) : selection_type_(selection_type){};

  void begin_sync();
  void edit_object_sync(const ObjectRef &ob_ref, Resources &res);
  void object_sync(const ObjectRef &ob_ref, Resources &res, const State &state);
  void end_sync(Resources &res, ShapeCache &shapes, const State &state);
  void draw(Resources &res, Manager &manager, View &view);
  void draw_in_front(Resources &res, Manager &manager, View &view);
};

}  // namespace blender::draw::overlay
