/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "overlay_next_private.hh"
#include "overlay_shader_shared.h"

namespace blender::draw::overlay {

class Armatures {
  using BoneInstanceBuf = ShapeInstanceBuf<BoneInstanceData>;

 private:
  const SelectionType selection_type_;

  PassSimple armature_ps_ = {"Armature"};

  BoneInstanceBuf octahedral_buf_ = {selection_type_, "octahedral_buf"};

  bool draw_transparent = false;
  bool show_relations = false;
  bool do_pose_xray = false;
  bool do_pose_fade_geom = false;

 public:
  Armatures(const SelectionType selection_type) : selection_type_(selection_type){};

  void begin_sync(const State &state)
  {
    const bool is_select_mode = (selection_type_ != SelectionType::DISABLED);

    draw_transparent = (state.v3d->shading.type == OB_WIRE) || XRAY_FLAG_ENABLED(state.v3d);
    show_relations = !((state.v3d->flag & V3D_HIDE_HELPLINES) || is_select_mode);
    do_pose_xray = (state.overlay.flag & V3D_OVERLAY_BONE_SELECT);
    // do_pose_fade_geom = do_pose_xray && !(state.object_mode & OB_MODE_WEIGHT_PAINT) &&
    //                     draw_ctx->object_pose != nullptr;

    octahedral_buf_.clear();
  }

  void edit_object_sync(Manager &manager, const ObjectRef &ob_ref, Resources &res)
  {
    // const select::ID radius_id = res.select_id(ob_ref, elem_num);
    // octahedral_buf_.append({ob_ref.object, &ml->x, ml->rad, color}, radius_id);
  }

  void object_sync(Manager &manager, const ObjectRef &ob_ref, Resources &res)
  {
    // const select::ID radius_id = res.select_id(ob_ref, elem_num);
    // octahedral_buf_.append({ob_ref.object, &ml->x, ml->rad, color}, radius_id);
  }

  void end_sync(Resources &res, ShapeCache &shapes, const State &state)
  {
    const float wire_alpha = state.overlay.bone_wire_alpha;
    const bool use_wire_alpha = (wire_alpha < 1.0f);

    armature_ps_.init();
    armature_ps_.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH |
                           DRW_STATE_DEPTH_LESS_EQUAL | state.clipping_state);
    res.select_bind(armature_ps_);

    // armature_ps_.shader_set(res.shaders.armature_shape_solid.get());
    armature_ps_.bind_ubo("globalsBlock", &res.globals_buf);
    // octahedral_buf_.end_sync(armature_ps_, shapes.bone_octahedral.get());
  }

  void draw(Framebuffer &framebuffer, Manager &manager, View &view)
  {
    GPU_framebuffer_bind(framebuffer);
    manager.submit(armature_ps_, view);
  }
};

}  // namespace blender::draw::overlay
