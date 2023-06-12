/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "overlay_next_extra_pass.hh"

namespace blender::draw::overlay {

static void object_center_sync(const ObjectRef &ob_ref,
                               const select::ID select_id,
                               Resources & /*res*/,
                               const State &state,
                               ExtraInstancePass &pass)
{
  Object *ob = ob_ref.object;

  float4 center = float4(ob->object_to_world[3]);

  const bool is_library = ID_REAL_USERS(&ob->id) > 1 || ID_IS_LINKED(ob);
  /* TODO(Miguel Pozo): Handle const. */
  BKE_view_layer_synced_ensure(state.scene, (ViewLayer *)state.view_layer);
  if (ob == BKE_view_layer_active_object_get(state.view_layer)) {
    pass.center_active.append(center, select_id);
  }
  else if (ob->base_flag & BASE_SELECTED) {
    (is_library ? pass.center_selected_lib : pass.center_selected).append(center, select_id);
  }
  else if (state.v3d_flag & V3D_DRAW_CENTERS) {
    (is_library ? pass.center_deselected_lib : pass.center_deselected).append(center, select_id);
  }
}

}  // namespace blender::draw::overlay
