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

static void clear_selection_legacy_action(Action &action)
{
  BLI_assert(action.is_action_legacy());
  LISTBASE_FOREACH (FCurve *, fcu, &action.curves) {
    if (!fcu->bezt) {
      continue;
    }
    for (int i = 0; i < fcu->totvert; i++) {
      BEZT_DESEL_ALL(&fcu->bezt[i]);
    }
  }
}

static void clear_selection_layered_action(Action &action)
{
  BLI_assert(action.is_action_layered());
  for (Layer *layer : action.layers()) {
    if (!layer) {
      continue;
    }
    for (Strip *strip : layer->strips()) {
      if (!strip || !strip->is<KeyframeStrip>()) {
        continue;
      }
      KeyframeStrip &key_strip = strip->as<KeyframeStrip>();
      for (ChannelBag *ch_bag : key_strip.channelbags()) {
        if (!ch_bag) {
          continue;
        }
        for (int i = 0; i < ch_bag->fcurve_array_num; i++) {
          FCurve *fcu = ch_bag->fcurve_array[i];
          for (int j = 0; j < fcu->totvert; j++) {
            BEZT_DESEL_ALL(&fcu->bezt[j]);
          }
        }
      }
    }
  }
}

void Action::deselect_keys()
{
  if (this->is_empty()) {
    return;
  }

  if (this->is_action_layered()) {
    clear_selection_layered_action(*this);
  }
  else {
    BLI_assert(this->is_action_legacy());
    clear_selection_legacy_action(*this);
  }
}
}  // namespace blender::animrig
