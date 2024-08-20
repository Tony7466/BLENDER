/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "ED_view3d.hh"

#include "overlay_next_private.hh"
#include "overlay_shader_shared.h"

namespace blender::draw::overlay {
using namespace blender;

enum eArmatureDrawMode {
  ARM_DRAW_MODE_OBJECT,
  ARM_DRAW_MODE_POSE,
  ARM_DRAW_MODE_EDIT,
};

class Armatures {
  using BoneInstanceBuf = ShapeInstanceBuf<BoneInstanceData>;

 private:
  const SelectionType selection_type_;

  PassSimple armature_ps_ = {"Armature"};

  /* Force transparent drawing in Xray mode. */
  bool draw_transparent = false;
  /* Force disable drawing relation is relations are off in viewport. */
  bool show_relations = false;
  /* Show selection state. */
  bool show_outline = false;

  bool do_pose_xray = false;
  bool do_pose_fade_geom = false;

  struct BoneBuffers {
    const SelectionType selection_type_;

    /* Bone end points (joints). */
    PassSimple::Sub *point_fill = nullptr;
    PassSimple::Sub *point_outline = nullptr;
    /* Bone shapes. */
    PassSimple::Sub *shape_fill = nullptr;
    PassSimple::Sub *shape_outline = nullptr;
    /* Custom bone wire-frame. */
    PassSimple::Sub *shape_wire = nullptr;

    BoneInstanceBuf octahedral_buf = {selection_type_, "octahedral_buf"};

    Map<Object *, PassSimple::Sub *> custom_shapes;

    BoneBuffers(const SelectionType selection_type) : selection_type_(selection_type){};
  };

  BoneBuffers opaque = {selection_type_};
  BoneBuffers transparent = {selection_type_};

 public:
  Armatures(const SelectionType selection_type) : selection_type_(selection_type){};

  void begin_sync(Resources &res, const State &state)
  {
    const bool is_select_mode = (selection_type_ != SelectionType::DISABLED);

    draw_transparent = (state.v3d->shading.type == OB_WIRE) || XRAY_FLAG_ENABLED(state.v3d);
    show_relations = !((state.v3d->flag & V3D_HIDE_HELPLINES) || is_select_mode);
    show_outline = (state.v3d->flag & V3D_SELECT_OUTLINE);
    do_pose_xray = (state.overlay.flag & V3D_OVERLAY_BONE_SELECT);
    // do_pose_fade_geom = do_pose_xray && !(state.object_mode & OB_MODE_WEIGHT_PAINT) &&
    //                     draw_ctx->object_pose != nullptr;

    const bool do_smooth_wire = U.gpu_flag & USER_GPU_FLAG_OVERLAY_SMOOTH_WIRE;
    const float wire_alpha = state.overlay.bone_wire_alpha;
    /* Draw bone outlines and custom shape wire with a specific alpha. */
    const bool use_wire_alpha = (wire_alpha < 1.0f);

    DRWState default_state = DRW_STATE_WRITE_COLOR | DRW_STATE_DEPTH_LESS_EQUAL |
                             DRW_STATE_WRITE_DEPTH | state.clipping_state;

    GPUTexture **depth_tex = (state.xray_enabled) ? &res.depth_tx : &res.dummy_depth_tx;

    armature_ps_.init();
    res.select_bind(armature_ps_);

    /* Transparent draws needs to be issued first. */

    /* "Opaque" draws. */

    {
      {
        auto &sub = armature_ps_.sub("opaque.point_fill");
        sub.state_set(default_state);
        sub.shader_set(res.shaders.armature_sphere_fill.get());
        sub.push_constant("alpha", 1.0f);
        opaque.point_fill = &sub;
      }
      {
        auto &sub = armature_ps_.sub("transparent.point_fill");
        sub.state_set((default_state & ~DRW_STATE_WRITE_DEPTH) | DRW_STATE_BLEND_ALPHA);
        sub.shader_set(res.shaders.armature_sphere_fill.get());
        sub.push_constant("alpha", wire_alpha * 0.4f);
        transparent.point_fill = &sub;
      }

      {
        auto &sub = armature_ps_.sub("opaque.shape_fill");
        sub.state_set(default_state);
        sub.shader_set(res.shaders.armature_shape_fill.get());
        sub.push_constant("alpha", 1.0f);
        opaque.shape_fill = &sub;
      }
      {
        auto &sub = armature_ps_.sub("transparent.shape_fill");
        sub.state_set((default_state & ~DRW_STATE_WRITE_DEPTH) | DRW_STATE_BLEND_ALPHA);
        sub.shader_set(res.shaders.armature_shape_fill.get());
        sub.push_constant("alpha", wire_alpha * 0.6f);
        transparent.shape_fill = &sub;
      }

      {
        auto &sub = armature_ps_.sub("opaque.point_outline");
        sub.state_set(default_state);
        sub.shader_set(res.shaders.armature_sphere_outline.get());
        sub.bind_ubo("globalsBlock", &res.globals_buf);
        sub.push_constant("alpha", 1.0f);
        opaque.point_outline = &sub;
      }
      if (use_wire_alpha) {
        auto &sub = armature_ps_.sub("transparent.point_outline");
        sub.state_set(default_state | DRW_STATE_BLEND_ALPHA);
        sub.shader_set(res.shaders.armature_sphere_outline.get());
        sub.bind_ubo("globalsBlock", &res.globals_buf);
        sub.push_constant("alpha", wire_alpha);
        transparent.point_outline = &sub;
      }
      else {
        transparent.point_outline = opaque.point_outline;
      }

      {
        auto &sub = armature_ps_.sub("opaque.shape_outline");
        sub.state_set(default_state);
        sub.shader_set(res.shaders.armature_shape_outline.get());
        sub.bind_ubo("globalsBlock", &res.globals_buf);
        sub.push_constant("alpha", 1.0f);
        opaque.shape_outline = &sub;
      }
      if (use_wire_alpha) {
        auto &sub = armature_ps_.sub("transparent.shape_outline");
        sub.state_set(default_state | DRW_STATE_BLEND_ALPHA);
        sub.shader_set(res.shaders.armature_shape_outline.get());
        sub.bind_ubo("globalsBlock", &res.globals_buf);
        sub.bind_texture("depthTex", depth_tex);
        sub.push_constant("alpha", wire_alpha * 0.6f);
        sub.push_constant("do_smooth_wire", do_smooth_wire);
        transparent.shape_outline = &sub;
      }
      else {
        transparent.shape_outline = opaque.shape_outline;
      }

      {
        auto &sub = armature_ps_.sub("opaque.shape_wire");
        sub.state_set(default_state);
        sub.shader_set(res.shaders.armature_shape_wire.get());
        sub.bind_ubo("globalsBlock", &res.globals_buf);
        sub.push_constant("alpha", 1.0f);
        opaque.shape_wire = &sub;
      }
      if (use_wire_alpha) {
        auto &sub = armature_ps_.sub("transparent.shape_wire");
        sub.state_set(default_state | DRW_STATE_BLEND_ALPHA);
        sub.shader_set(res.shaders.armature_shape_wire.get());
        sub.bind_ubo("globalsBlock", &res.globals_buf);
        sub.bind_texture("depthTex", depth_tex);
        sub.push_constant("alpha", wire_alpha * 0.6f);
        sub.push_constant("do_smooth_wire", do_smooth_wire);
        transparent.shape_wire = &sub;
      }
      else {
        transparent.shape_wire = opaque.shape_wire;
      }
    }
    {
      /* Degrees-of-Freedom. */
    }
    {
      /* Stick Bones. */
    }
    {
      /* Envelopes. */
    }
    {
      /* Wires. */
    }

    auto clear_buffers = [](BoneBuffers &bb) {
      bb.octahedral_buf.clear();
      bb.custom_shapes.clear();
    };

    clear_buffers(transparent);
    clear_buffers(opaque);
  }

  struct DrawContext {
    /* Current armature object */
    Object *ob = nullptr;

    eArmatureDrawMode draw_mode;
    eArmature_Drawtype drawtype;

    Armatures::BoneBuffers *bone_buf = nullptr;

    /* TODO: Legacy structures to be removed after overlay next is shipped. */
    DRWCallBuffer *outline = nullptr;
    DRWCallBuffer *solid = nullptr;
    DRWCallBuffer *wire = nullptr;
    DRWCallBuffer *envelope_outline = nullptr;
    DRWCallBuffer *envelope_solid = nullptr;
    DRWCallBuffer *envelope_distance = nullptr;
    DRWCallBuffer *stick = nullptr;
    DRWCallBuffer *dof_lines = nullptr;
    DRWCallBuffer *dof_sphere = nullptr;
    DRWCallBuffer *point_solid = nullptr;
    DRWCallBuffer *point_outline = nullptr;
    DRWShadingGroup *custom_solid = nullptr;
    DRWShadingGroup *custom_outline = nullptr;
    DRWShadingGroup *custom_wire = nullptr;
    GHash *custom_shapes_ghash = nullptr;
    OVERLAY_ExtraCallBuffers *extras = nullptr;

    /* Not a theme, this is an override. */
    const float *const_color = nullptr;
    /* Wire thickness. */
    float const_wire = 0.0f;

    bool do_relations = false;
    bool transparent = false;
    bool show_relations = false;
    bool draw_relation_from_head = false;
    /* Draw the inner part of the bones, otherwise render just outlines. */
    bool is_filled = false;

    const ThemeWireColor *bcolor = nullptr; /* pchan color */

    DrawContext() = default;
  };

  DrawContext create_draw_context(const ObjectRef &ob_ref,
                                  const Resources &res,
                                  const State &state,
                                  eArmatureDrawMode draw_mode)
  {
    bArmature *arm = static_cast<bArmature *>(ob_ref.object->data);

    DrawContext ctx;
    ctx.ob = ob_ref.object;
    ctx.draw_mode = draw_mode;
    ctx.drawtype = eArmature_Drawtype(arm->drawtype);

    const bool is_edit_or_pose_mode = draw_mode != ARM_DRAW_MODE_OBJECT;
    const bool draw_as_wire = (ctx.ob->dt < OB_SOLID);
    const bool is_transparent = draw_transparent || (draw_as_wire && is_edit_or_pose_mode);

    ctx.bone_buf = is_transparent ? &transparent : &opaque;

    ctx.is_filled = (!draw_transparent && !draw_as_wire) || is_edit_or_pose_mode;
    ctx.show_relations = show_relations;
    ctx.do_relations = show_relations && is_edit_or_pose_mode;
    ctx.draw_relation_from_head = (arm->flag & ARM_DRAW_RELATION_FROM_HEAD);
    ctx.const_color = is_edit_or_pose_mode ? nullptr : &res.object_wire_color(ob_ref, state)[0];
    ctx.const_wire = ((ctx.ob->base_flag & BASE_SELECTED) && show_outline ?
                          1.5f :
                          ((!ctx.is_filled || is_transparent) ? 1.0f : 0.0f));
    return ctx;
  }

  void edit_object_sync(const ObjectRef &ob_ref, Resources &res)
  {
    const select::ID radius_id = res.select_id(ob_ref);
    opaque.octahedral_buf.append({ob_ref.object->object_to_world()}, radius_id);
  }

  void object_sync(const ObjectRef &ob_ref, Resources &res, const State &state)
  {
    if (ob_ref.object->dt == OB_BOUNDBOX) {
      return;
    }

    DrawContext ctx = create_draw_context(ob_ref, res, state, ARM_DRAW_MODE_OBJECT);
    draw_armature_pose(&ctx);

    // const select::ID radius_id = res.select_id(ob_ref);
    // opaque.octahedral_buf.append({ob_ref.object->object_to_world()}, radius_id);
  }

  void end_sync(Resources & /*res*/, ShapeCache &shapes, const State & /*state*/)
  {
    auto clear_buffers = [&](BoneBuffers &bb) {
      bb.octahedral_buf.end_sync(*bb.shape_fill, shapes.bone_octahedron.get());
    };

    clear_buffers(transparent);
    clear_buffers(opaque);
  }

  void draw(Framebuffer &framebuffer, Manager &manager, View &view)
  {
    GPU_framebuffer_bind(framebuffer);
    manager.submit(armature_ps_, view);
  }

  /* Public for the time of the Overlay Next port to avoid duplicated logic. */
 public:
  static void draw_armature_pose(Armatures::DrawContext *ctx);
  static void draw_armature_edit(Armatures::DrawContext *ctx);
};

}  // namespace blender::draw::overlay
