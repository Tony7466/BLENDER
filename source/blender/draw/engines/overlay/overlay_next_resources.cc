/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "overlay_next_private.hh"

namespace blender::draw::overlay {

ThemeColorID Resources::object_wire_theme_id(const ObjectRef &ob_ref, const State &state) const
{
  const bool is_edit = (state.object_mode & OB_MODE_EDIT) && (ob_ref.object->mode & OB_MODE_EDIT);
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

const float4 &Resources::object_wire_color(const ObjectRef &ob_ref, ThemeColorID theme_id) const
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

const float4 &Resources::object_wire_color(const ObjectRef &ob_ref, const State &state) const
{
  ThemeColorID theme_id = object_wire_theme_id(ob_ref, state);
  return object_wire_color(ob_ref, theme_id);
}

}  // namespace blender::draw::overlay
