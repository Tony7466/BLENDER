/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "usd_armature_utils.h"

#include "BKE_armature.h"
#include "DNA_armature_types.h"

#include "ED_armature.hh"

#include "WM_api.hh"

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

}  // namespace blender::io::usd
