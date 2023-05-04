/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2019 Blender Foundation. */

/** \file
 * \ingroup DNA
 */

#pragma once

#include "BKE_global.h"

#include "DRW_gpu_wrapper.hh"
#include "DRW_render.h"

#include "UI_resources.h"

#include "draw_handle.hh"

#include "overlay_next_shader.hh"
#include "overlay_shader_shared.h"

#ifdef __APPLE__
#  define USE_GEOM_SHADER_WORKAROUND 1
#else
#  define USE_GEOM_SHADER_WORKAROUND 0
#endif

/* Needed for eSpaceImage_UVDT_Stretch and eMaskOverlayMode */
#include "DNA_mask_types.h"
#include "DNA_space_types.h"
/* Forward declarations */
struct ImBuf;

namespace blender::draw::overlay {

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

using blender::draw::Framebuffer;
using blender::draw::StorageVectorBuffer;
using blender::draw::Texture;
using blender::draw::TextureFromPool;
using blender::draw::TextureRef;

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

  [[nodiscard]] ThemeColorID object_wire_theme_id(const ObjectRef &ob_ref,
                                                  const State &state) const
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

  [[nodiscard]] const float4 &object_wire_color(const ObjectRef &ob_ref,
                                                ThemeColorID theme_id) const
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

  [[nodiscard]] const float4 &object_wire_color(const ObjectRef &ob_ref, const State &state) const
  {
    ThemeColorID theme_id = object_wire_theme_id(ob_ref, state);
    return object_wire_color(ob_ref, theme_id);
  }
};

}  // namespace blender::draw::overlay
