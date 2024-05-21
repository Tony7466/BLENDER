/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup animrig
 */

#include "ANIM_action.hh"
#include "ANIM_fcurve.hh"
#include "BKE_action.h"
#include "BKE_fcurve.hh"
#include "BLI_listbase.h"
#include "BLI_math_vector.h"
#include "BLI_string.h"
#include "DEG_depsgraph_build.hh"
#include "DNA_anim_types.h"

#include "RNA_access.hh"
#include "RNA_path.hh"
#include "RNA_prototypes.h"

namespace blender::animrig {

FCurve *action_fcurve_find(bAction *act, const char rna_path[], const int array_index)
{
  if (ELEM(nullptr, act, rna_path)) {
    return nullptr;
  }
  return BKE_fcurve_find(&act->curves, rna_path, array_index);
}

FCurve *action_fcurve_ensure(Main *bmain,
                             bAction *act,
                             const char group[],
                             PointerRNA *ptr,
                             const char rna_path[],
                             const int array_index)
{
  if (ELEM(nullptr, act, rna_path)) {
    return nullptr;
  }

  /* Try to find f-curve matching for this setting.
   * - add if not found and allowed to add one
   *   TODO: add auto-grouping support? how this works will need to be resolved
   */
  FCurve *fcu = BKE_fcurve_find(&act->curves, rna_path, array_index);

  if (fcu != nullptr) {
    return fcu;
  }

  fcu = create_fcurve_for_channel(rna_path, array_index);

  if (BLI_listbase_is_empty(&act->curves)) {
    fcu->flag |= FCURVE_ACTIVE;
  }

  if (U.keying_flag & KEYING_FLAG_XYZ2RGB && ptr != nullptr) {
    /* For Loc/Rot/Scale and also Color F-Curves, the color of the F-Curve in the Graph Editor,
     * is determined by the array index for the F-Curve.
     */
    PropertyRNA *resolved_prop;
    PointerRNA resolved_ptr;
    PointerRNA id_ptr = RNA_id_pointer_create(ptr->owner_id);
    const bool resolved = RNA_path_resolve_property(
        &id_ptr, rna_path, &resolved_ptr, &resolved_prop);
    if (resolved) {
      PropertySubType prop_subtype = RNA_property_subtype(resolved_prop);
      if (ELEM(prop_subtype, PROP_TRANSLATION, PROP_XYZ, PROP_EULER, PROP_COLOR, PROP_COORDS)) {
        fcu->color_mode = FCURVE_COLOR_AUTO_RGB;
      }
      else if (ELEM(prop_subtype, PROP_QUATERNION)) {
        fcu->color_mode = FCURVE_COLOR_AUTO_YRGB;
      }
    }
  }

  if (group) {
    bActionGroup *agrp = BKE_action_group_find_name(act, group);

    if (agrp == nullptr) {
      agrp = action_groups_add_new(act, group);

      /* Sync bone group colors if applicable. */
      if (ptr && (ptr->type == &RNA_PoseBone) && ptr->data) {
        const bPoseChannel *pchan = static_cast<const bPoseChannel *>(ptr->data);
        action_group_colors_set_from_posebone(agrp, pchan);
      }
    }

    action_groups_add_channel(act, agrp, fcu);
  }
  else {
    BLI_addtail(&act->curves, fcu);
  }

  /* New f-curve was added, meaning it's possible that it affects
   * dependency graph component which wasn't previously animated.
   */
  DEG_relations_tag_update(bmain);

  return fcu;
}

Action *convert_to_layered_action(Main &bmain, const Action &legacy_action)
{
  if (legacy_action.is_empty() || legacy_action.is_action_layered()) {
    return nullptr;
  }

  char layered_action_name[MAX_ID_NAME - 2];
  SNPRINTF(layered_action_name, "%s_layered", legacy_action.id.name);
  bAction *baction = BKE_action_add(&bmain, layered_action_name);

  Action &converted_action = baction->wrap();
  Binding &binding = converted_action.binding_add();
  Layer &layer = converted_action.layer_add(legacy_action.id.name);
  Strip &strip = layer.strip_add(Strip::Type::Keyframe);
  KeyframeStrip &key_strip = strip.as<KeyframeStrip>();

  LISTBASE_FOREACH (FCurve *, fcu, &legacy_action.curves) {
    FCurve &new_fcu = key_strip.fcurve_find_or_create(binding, fcu->rna_path, fcu->array_index);
    new_fcu.bezt = static_cast<BezTriple *>(
        MEM_callocN(fcu->totvert * sizeof(BezTriple), "beztriple"));
    memcpy(new_fcu.bezt, fcu->bezt, sizeof(BezTriple) * fcu->totvert);
    new_fcu.totvert = fcu->totvert;
    new_fcu.color_mode = fcu->color_mode;
    copy_v3_v3(new_fcu.color, fcu->color);
    new_fcu.active_keyframe_index = fcu->active_keyframe_index;
    new_fcu.flag = fcu->flag;
    new_fcu.extend = fcu->extend;
    new_fcu.auto_smoothing = fcu->auto_smoothing;
    copy_fmodifiers(&new_fcu.modifiers, &fcu->modifiers);
  }

  return &converted_action;
}

Action *bake_to_legacy_action(Main &bmain, const Action &layered_action, const Binding &binding)
{
  if (layered_action.is_empty() || layered_action.is_action_legacy()) {
    return nullptr;
  }
  if (!layered_action.is_binding_animated(binding.handle)) {
    return nullptr;
  }

  char legacy_action_name[MAX_ID_NAME - 2];
  SNPRINTF(legacy_action_name, "%s_legacy", layered_action.id.name);
  bAction *baction = BKE_action_add(&bmain, legacy_action_name);

  Action &converted_action = baction->wrap();

  for (const Layer *layer : layered_action.layers()) {
  }

  return &converted_action;
}

}  // namespace blender::animrig
