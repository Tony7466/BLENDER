/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "overlay_next_private.hh"

namespace blender::draw::overlay {

class ObjectCenterPass {
  const SelectionType selection_type_;
  PassSimple ps_;

  PointInstanceBuf active_ = {selection_type_, "active"};
  PointInstanceBuf selected_ = {selection_type_, "selected"};
  PointInstanceBuf deselected_ = {selection_type_, "deselected"};
  PointInstanceBuf selected_lib_ = {selection_type_, "selected_lib"};
  PointInstanceBuf deselected_lib_ = {selection_type_, "deselected_lib"};

 public:
  ObjectCenterPass(const SelectionType selection_type, const char *name)
      : selection_type_(selection_type), ps_(name){};

  void begin_sync()
  {
    active_.clear();
    selected_.clear();
    deselected_.clear();
    selected_lib_.clear();
    deselected_lib_.clear();
  }

  void object_sync(const ObjectRef &ob_ref,
                   const select::ID select_id,
                   Resources & /*res*/,
                   const State &state)
  {
    Object *ob = ob_ref.object;

    const bool is_paint_mode = state.object_mode & (OB_MODE_ALL_PAINT | OB_MODE_ALL_PAINT_GPENCIL |
                                                    OB_MODE_SCULPT_CURVES);
    const bool from_dupli = ob->base_flag & (BASE_FROM_SET | BASE_FROM_DUPLI);
    const bool hide_centers = state.overlay.flag & V3D_OVERLAY_HIDE_OBJECT_ORIGINS;

    if (is_paint_mode || from_dupli || hide_centers) {
      return;
    }

    float4 center = float4(ob->object_to_world[3]);

    const bool is_library = ID_REAL_USERS(&ob->id) > 1 || ID_IS_LINKED(ob);
    /* TODO(Miguel Pozo): Handle const. */
    BKE_view_layer_synced_ensure(state.scene, (ViewLayer *)state.view_layer);
    if (ob == BKE_view_layer_active_object_get(state.view_layer)) {
      active_.append(center, select_id);
    }
    else if (ob->base_flag & BASE_SELECTED) {
      (is_library ? selected_lib_ : selected_).append(center, select_id);
    }
    else if (state.v3d_flag & V3D_DRAW_CENTERS) {
      (is_library ? deselected_lib_ : deselected_).append(center, select_id);
    }
  }

  void end_sync(Resources &res, const State &state)
  {
    ps_.init();
    res.select_bind(ps_);
    ps_.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_BLEND_ALPHA | DRW_STATE_WRITE_DEPTH |
                  DRW_STATE_PROGRAM_POINT_SIZE | state.clipping_state);
    ps_.shader_set(res.shaders.extra_point.get());
    /* TODO: Fixed index. */
    ps_.bind_ubo("globalsBlock", &res.globals_buf);

    active_.end_sync(ps_, res.theme_settings.color_active);
    selected_.end_sync(ps_, res.theme_settings.color_select);
    deselected_.end_sync(ps_, res.theme_settings.color_deselect);
    selected_lib_.end_sync(ps_, res.theme_settings.color_library_select);
    deselected_lib_.end_sync(ps_, res.theme_settings.color_library);
  }

  void draw(Manager &manager, View &view, Framebuffer &fb)
  {
    fb.bind();
    manager.submit(ps_, view);
  }
};

class ObjectCenter {
  ObjectCenterPass pass_;
  ObjectCenterPass pass_in_front_;

 public:
  ObjectCenter(const SelectionType selection_type)
      : pass_(selection_type, "Object Center"),
        pass_in_front_(selection_type, "Object Center In Front"){};

  void begin_sync()
  {
    pass_.begin_sync();
    pass_in_front_.begin_sync();
  }

  void object_sync(const ObjectRef &ob_ref,
                   const select::ID select_id,
                   Resources &res,
                   const State &state)
  {
    ObjectCenterPass &pass = ob_ref.object->dtx & OB_DRAW_IN_FRONT ? pass_in_front_ : pass_;
    pass.object_sync(ob_ref, select_id, res, state);
  }

  void end_sync(Resources &res, const State &state)
  {
    pass_.end_sync(res, state);
    pass_in_front_.end_sync(res, state);
  }

  void draw(Resources &res, Manager &manager, View &view)
  {
    pass_.draw(manager, view, res.overlay_line_fb);
  }

  void draw_in_front(Resources &res, Manager &manager, View &view)
  {
    pass_in_front_.draw(manager, view, res.overlay_line_in_front_fb);
  }
};

}  // namespace blender::draw::overlay
