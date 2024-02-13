/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgreasepencil
 */

#include "BLI_math_vector.hh"
#include "BLI_set.hh"

#include "BKE_action.h"
#include "BKE_deform.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_modifier.hh"
#include "BKE_object.hh"
#include "BKE_object_deform.h"

#include "DNA_meshdata_types.h"

#include "ED_grease_pencil.hh"

namespace blender::ed::greasepencil {

int create_vertex_group_in_object(Object *ob)
{
  int def_nr = 0;
  bool named_by_bone = false;

  /* Look for an active bone in armature to name the vertex group after. */
  Object *ob_armature = BKE_modifiers_is_deformed_by_armature(ob);
  if (ob_armature != nullptr) {
    Bone *actbone = ((bArmature *)ob_armature->data)->act_bone;
    if (actbone != nullptr) {
      bPoseChannel *pchan = BKE_pose_channel_find_name(ob_armature->pose, actbone->name);
      if (pchan != nullptr) {
        const int channel_def_nr = BKE_object_defgroup_name_index(ob, pchan->name);
        if (channel_def_nr == -1) {
          BKE_object_defgroup_add_name(ob, pchan->name);
          def_nr = BKE_object_defgroup_active_index_get(ob) - 1;
        }
        else {
          def_nr = channel_def_nr;
        }
        named_by_bone = true;
      }
    }
  }

  /* Create a vertex group with a general name. */
  if (!named_by_bone) {
    BKE_object_defgroup_add(ob);
  }

  return def_nr;
}

Set<std::string> get_bone_deformed_vertex_groups(Object &object)
{
  Set<std::string> bone_deformed_vgroups;

  /* Get all vertex group names in the object. */
  const ListBase *defbase = BKE_object_defgroup_list(&object);
  Set<std::string> defgroups;
  LISTBASE_FOREACH (bDeformGroup *, dg, defbase) {
    defgroups.add(dg->name);
  }

  /* Lambda function for finding deforming bones with a name matching a vertex group. */
  const auto find_pose_channels = [&](ModifierData *md) {
    for (; md; md->next) {
      if (!(md->mode & (eModifierMode_Realtime | eModifierMode_Virtual)) ||
          md->type != eModifierType_Armature)
      {
        continue;
      }
      ArmatureModifierData *amd = reinterpret_cast<ArmatureModifierData *>(md);
      if (!amd->object || !amd->object->pose) {
        continue;
      }

      bPose *pose = amd->object->pose;
      LISTBASE_FOREACH (bPoseChannel *, channel, &pose->chanbase) {
        if (channel->bone->flag & BONE_NO_DEFORM) {
          continue;
        }
        if (defgroups.contains(channel->name)) {
          bone_deformed_vgroups.add(channel->name);
        }
      }
    }
  };

  /* Inspect all armature modifiers in the object. */
  VirtualModifierData virtual_modifier_data;
  ModifierData *md = static_cast<ModifierData *>(object.modifiers.first);
  find_pose_channels(md);
  find_pose_channels(BKE_modifiers_get_virtual_modifierlist(&object, &virtual_modifier_data));

  return bone_deformed_vgroups;
}

/* Normalize the weights of vertex groups deformed by bones so that the sum is 1.0f.
 * Returns false when the normalization failed due to too many locked vertex groups. In that case a
 * second pass can be done with the active vertex group unlocked.
 */
static bool normalize_vertex_weights_try(const MDeformVert &dvert,
                                         const int vertex_groups_num,
                                         const int locked_active_vertex_group,
                                         const Vector<bool> &vertex_group_is_locked,
                                         const Vector<bool> &vertex_group_is_bone_deformed)
{
  /* Nothing to normalize when there are less than two vertex group weights. */
  if (dvert.totweight <= 1) {
    return true;
  }

  /* Get the sum of weights of bone-deformed vertex groups. */
  float sum_weights_total = 0.0f, sum_weights_locked = 0.0f, sum_weights_unlocked = 0.0f;
  int locked_num = 0, unlocked_num = 0;
  for (int i = 0; i < dvert.totweight; i++) {
    MDeformWeight &dw = dvert.dw[i];

    /* Auto-normalize is only applied on bone-deformed vertex groups that have weight already. */
    if (dw.def_nr >= vertex_groups_num || !vertex_group_is_bone_deformed[dw.def_nr] ||
        dw.weight <= FLT_EPSILON)
    {
      continue;
    }

    sum_weights_total += dw.weight;

    if (vertex_group_is_locked[dw.def_nr] || dw.def_nr == locked_active_vertex_group) {
      locked_num++;
      sum_weights_locked += dw.weight;
    }
    else {
      unlocked_num++;
      sum_weights_unlocked += dw.weight;
    }
  }

  /* Already normalized? */
  if (sum_weights_total == 1.0f) {
    return true;
  }

  /* Any unlocked vertex group to normalize? */
  if (unlocked_num == 0) {
    /* We don't need a second pass when there is only one locked group (the active group). */
    return (locked_num == 0);
  }

  /* Locked groups can make it impossible to fully normalize. */
  if (sum_weights_locked >= 1.0f - VERTEX_WEIGHT_LOCK_EPSILON) {
    /* Zero out the weights we are allowed to touch and return false, indicating a second pass is
     * needed. */
    for (int i = 0; i < dvert.totweight; i++) {
      MDeformWeight &dw = dvert.dw[i];
      if (dw.def_nr < vertex_groups_num && vertex_group_is_bone_deformed[dw.def_nr] &&
          !vertex_group_is_locked[dw.def_nr] && dw.def_nr != locked_active_vertex_group)
      {
        dw.weight = 0.0f;
      }
    }

    return (sum_weights_locked == 1.0f);
  }

  /* When the sum of the unlocked weights isn't zero, we can use a multiplier to normalize them
   * to 1.0f. */
  if (sum_weights_unlocked != 0.0f) {
    const float normalize_factor = (1.0f - sum_weights_locked) / sum_weights_unlocked;

    for (int i = 0; i < dvert.totweight; i++) {
      MDeformWeight &dw = dvert.dw[i];
      if (dw.def_nr < vertex_groups_num && vertex_group_is_bone_deformed[dw.def_nr] &&
          dw.weight > FLT_EPSILON && !vertex_group_is_locked[dw.def_nr] &&
          dw.def_nr != locked_active_vertex_group)
      {
        dw.weight = math::clamp(dw.weight * normalize_factor, 0.0f, 1.0f);
      }
    }
  }
  /* Spread out the remainder of the locked weights over the unlocked weights. */
  else {
    const float weight_remainder = math::clamp(
        (1.0f - sum_weights_locked) / unlocked_num, 0.0f, 1.0f);

    for (int i = 0; i < dvert.totweight; i++) {
      MDeformWeight &dw = dvert.dw[i];
      if (dw.def_nr < vertex_groups_num && vertex_group_is_bone_deformed[dw.def_nr] &&
          dw.weight > FLT_EPSILON && !vertex_group_is_locked[dw.def_nr] &&
          dw.def_nr != locked_active_vertex_group)
      {
        dw.weight = weight_remainder;
      }
    }
  }

  return true;
}

void normalize_vertex_weights(const MDeformVert &dvert,
                              const int active_vertex_group,
                              const Vector<bool> &vertex_group_is_locked,
                              const Vector<bool> &vertex_group_is_bone_deformed)
{
  /* Try to normalize the weights with both active and explicitly locked vertex groups restricted
   * from change. */
  const bool success = normalize_vertex_weights_try(dvert,
                                                    vertex_group_is_locked.size(),
                                                    active_vertex_group,
                                                    vertex_group_is_locked,
                                                    vertex_group_is_bone_deformed);

  if (!success) {
    /* Do a second pass with the active vertex group unlocked. */
    normalize_vertex_weights_try(dvert,
                                 vertex_group_is_locked.size(),
                                 -1,
                                 vertex_group_is_locked,
                                 vertex_group_is_bone_deformed);
  }
}

}  // namespace blender::ed::greasepencil
