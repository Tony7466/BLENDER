/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */
#include "usd_modifier_disabler.h"

#include <cstdio>

#include "BLI_listbase.h"

#include "DEG_depsgraph.h"
#include "DEG_depsgraph_query.h"

#include "DNA_layer_types.h"
#include "DNA_mesh_types.h"
#include "DNA_modifier_types.h"
#include "DNA_object_types.h"

#include "BKE_layer.h"
#include "BKE_modifier.h"

namespace blender::io::usd {

ModifierDisabler::ModifierDisabler(Depsgraph *depsgraph, const USDExportParams &export_params)
    : depsgraph_(depsgraph), export_params_(export_params)
{
}

ModifierDisabler::~ModifierDisabler()
{
  for (ModifierData *modifier : disabled_modifiers_) {
    modifier->mode &= ~eModifierMode_DisableTemporary;
  }

  for (Object *object : modified_objects_) {
    DEG_id_tag_update(&object->id, ID_RECALC_GEOMETRY);
  }
}

void ModifierDisabler::disable_modifiers()
{
  if (!export_params_.export_armatures) {
    return;
  }

  const ModifierMode mode = export_params_.evaluation_mode == DAG_EVAL_VIEWPORT ?
                                eModifierMode_Realtime :
                                eModifierMode_Render;

  Scene *scene = DEG_get_input_scene(depsgraph_);
  ViewLayer *view_layer = DEG_get_input_view_layer(depsgraph_);

  BKE_view_layer_synced_ensure(scene, view_layer);
  LISTBASE_FOREACH (Base *, base, BKE_view_layer_object_bases_get(view_layer)) {
    Object *object = base->object;

    if (object->type != OB_MESH) {
      continue;
    }

    ModifierData *mod = BKE_modifiers_findby_type(object, eModifierType_Armature);
    if (mod == nullptr || !BKE_modifier_is_enabled(scene, mod, mode)) {
      /* We ignore modifiers that are already disabled. */
      continue;
    }

    /* This disables more modifiers than necessary, as it doesn't take restrictions like
     * "export selected objects only" into account. */
    disable_modifier(mod);
    modified_objects_.insert(object);
    DEG_id_tag_update(&object->id, ID_RECALC_GEOMETRY);
  }
}

void ModifierDisabler::disable_modifier(ModifierData *mod)
{
  mod->mode |= eModifierMode_DisableTemporary;
  disabled_modifiers_.insert(mod);
}

}  // namespace blender::io::usd
