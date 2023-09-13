/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "usd_armature_utils.h"

#include "BKE_armature.h"
#include "BKE_modifier.h"
#include "DEG_depsgraph.h"
#include "DEG_depsgraph_query.h"
#include "DNA_armature_types.h"
#include "ED_armature.hh"

#include "WM_api.hh"

using namespace blender::io::usd;

/* Recursively invoke the 'visitor' function on the given bone and its children. */
static void visit_bones(const Bone *bone, std::function<void(const Bone *)> visitor)
{
  if (!(bone && visitor)) {
    return;
  }

  visitor(bone);

  for (Bone *child = (Bone *)bone->childbase.first; child; child = child->next) {
    visit_bones(child, visitor);
  }
}

/* Return the armature modifier on the given object.  Return null if no
 * armature modifier can be found. */
static ArmatureModifierData *get_armature_modifier(const Object *obj)
{
  BLI_assert(obj);
  ArmatureModifierData *mod = reinterpret_cast<ArmatureModifierData *>(
      BKE_modifiers_findby_type(obj, eModifierType_Armature));
  return mod;
}

/**
 * Return in #ModifierQueryResult the first enabled modifier of the given type
 * on the given object (or null if the modifier isn't found) and the total number
 * of enabled modifiers on the object.
 */
static ModifierQueryResult get_enabled_modifier(const Object *obj,
                                                const Depsgraph *depsgraph,
                                                const ModifierType type)
{
  BLI_assert(obj);
  BLI_assert(depsgraph);

  ModifierData *ret_md = nullptr;

  Scene *scene = DEG_get_input_scene(depsgraph);
  eEvaluationMode mode = DEG_get_mode(depsgraph);

  int num_enabled = 0;
  LISTBASE_FOREACH (ModifierData *, md, &obj->modifiers) {

    if (!BKE_modifier_is_enabled(scene, md, mode)) {
      continue;
    }

    ++num_enabled;

    if (!ret_md && md->type == type) {
      ret_md = md;
    }
  }

  return std::make_pair(ret_md, num_enabled);
}


namespace blender::io::usd {

void visit_bones(const Object *ob_arm, std::function<void(const Bone *)> visitor)
{
  if (!(ob_arm && ob_arm->type == OB_ARMATURE && ob_arm->data)) {
    return;
  }

  bArmature *armature = (bArmature *)ob_arm->data;

  for (Bone *bone = (Bone *)armature->bonebase.first; bone; bone = bone->next) {
    visit_bones(bone, visitor);
  }
}

void get_armature_bone_names(const Object *ob_arm, std::vector<std::string> &r_names)
{
  auto visitor = [&r_names](const Bone *bone) {
    if (bone) {
      r_names.push_back(bone->name);
    }
  };

  visit_bones(ob_arm, visitor);
}

pxr::TfToken build_usd_joint_path(const Bone *bone)
{
  std::string path(pxr::TfMakeValidIdentifier(bone->name));

  const Bone *parent = bone->parent;
  while (parent) {
    path = pxr::TfMakeValidIdentifier(parent->name) + std::string("/") + path;
    parent = parent->parent;
  }

  return pxr::TfToken(path);
}

void create_pose_joints(pxr::UsdSkelAnimation &skel_anim, const Object *obj)
{
  if (!(skel_anim && obj && obj->pose)) {
    return;
  }

  pxr::VtTokenArray joints;

  const bPose *pose = obj->pose;

  LISTBASE_FOREACH (const bPoseChannel *, pchan, &pose->chanbase) {
    if (pchan->bone) {
      joints.push_back(build_usd_joint_path(pchan->bone));
    }
  }

  skel_anim.GetJointsAttr().Set(joints);
}

bool has_enabled_armature_modifier(const Object *obj, const Depsgraph *depsgraph)
{
  return get_enabled_modifier(obj, depsgraph, eModifierType_Armature).first != nullptr;
}

ModifierQueryResult get_enabled_armature_modifier(const Object *obj,
                                                  const Depsgraph *depsgraph)
{
  return get_enabled_modifier(obj, depsgraph, eModifierType_Armature);
}

const Object *get_armature_modifier_obj(const Object *obj)
{
  const ArmatureModifierData *mod = get_armature_modifier(obj);
  return mod ? mod->object : nullptr;
}

bool is_armature_modifier_bone_name(const Object *obj, const char *name)
{
  if (!obj || !name) {
    return false;
  }
  const ArmatureModifierData *arm_mod = get_armature_modifier(obj);

  if (!arm_mod || !arm_mod->object || !arm_mod->object->data) {
    return false;
  }

  bArmature *arm = (bArmature *)arm_mod->object->data;

  return BKE_armature_find_bone_name(arm, name);
}

bool can_export_skinned_mesh(const Object *obj, const Depsgraph *depsgraph)
{
  ModifierQueryResult result = get_enabled_armature_modifier(obj, depsgraph);

  /* We can export a skinned mesh if the object has an enabled
   * armature modifier and no other enabled modifiers. */
  return result.first != nullptr && result.second == 1;
}

}  // namespace blender::io::usd
