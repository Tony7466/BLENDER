/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "overlay_next_private.hh"

namespace blender::draw::overlay {

class ObjectCenterPasses : public OverlayPasses {

  ExtraInstanceBuf origin_xform = extra_buf("origin_xform", shapes.plain_axes, DEFAULT_ALWAYS);

  ExtraInstanceBuf arrows = extra_buf("arrows", shapes.arrows);

  PointInstanceBuf active = point_buf("active", theme_colors.color_active);
  PointInstanceBuf selected = point_buf("selected", theme_colors.color_select);
  PointInstanceBuf deselected = point_buf("deselected", theme_colors.color_deselect);
  PointInstanceBuf selected_lib = point_buf("selected_lib", theme_colors.color_library_select);
  PointInstanceBuf deselected_lib = point_buf("deselected_lib", theme_colors.color_library);

 public:
  ObjectCenterPasses(SelectionType selection_type,
                     const ShapeCache &shapes,
                     const GlobalsUboStorage &theme_colors,
                     bool in_front)
      : OverlayPasses("Centers", selection_type, shapes, theme_colors, in_front){};

  virtual void object_sync(const ObjectRef &ob_ref,
                           const select::ID select_id,
                           Resources &res,
                           const State &state) override
  {
    Object *ob = ob_ref.object;
    ExtraInstanceData data(ob_ref.object, res.object_wire_color(ob_ref, state));

    const bool is_select_mode = state.selection_type != SelectionType::DISABLED;
    const bool draw_xform = state.object_mode == OB_MODE_OBJECT &&
                            (state.scene->toolsettings->transform_flag & SCE_XFORM_DATA_ORIGIN) &&
                            (ob->base_flag & BASE_SELECTED) && !is_select_mode;

    if (draw_xform) {
      /* TODO(Miguel Pozo): What's this? */
      const float4 color_xform = {0.15f, 0.15f, 0.15f, 0.7f};
      origin_xform.append(data.with_color(color_xform), select_id);
    }

    const bool from_dupli = ob->base_flag & (BASE_FROM_SET | BASE_FROM_DUPLI);

    if (!from_dupli && ob->dtx & OB_AXIS) {
      arrows.append(data, select_id);
    }

    const bool is_paint_mode = state.object_mode & (OB_MODE_ALL_PAINT | OB_MODE_ALL_PAINT_GPENCIL |
                                                    OB_MODE_SCULPT_CURVES);
    const bool hide_obcenters = state.overlay.flag & V3D_OVERLAY_HIDE_OBJECT_ORIGINS;

    if (is_paint_mode || from_dupli || hide_obcenters) {
      return;
    }

    float4 center = float4(ob->object_to_world[3]);

    const bool is_library = ID_REAL_USERS(&ob->id) > 1 || ID_IS_LINKED(ob);
    /* TODO(Miguel Pozo): Handle const. */
    BKE_view_layer_synced_ensure(state.scene, (ViewLayer *)state.view_layer);
    if (ob == BKE_view_layer_active_object_get(state.view_layer)) {
      active.append(center, select_id);
    }
    else if (ob->base_flag & BASE_SELECTED) {
      (is_library ? selected_lib : selected).append(center, select_id);
    }
    else if (state.v3d_flag & V3D_DRAW_CENTERS) {
      (is_library ? deselected_lib : deselected).append(center, select_id);
    }
  }
};

using ObjectCenter = OverlayType<ObjectCenterPasses>;

}  // namespace blender::draw::overlay
