/* SPDX-FileCopyrightText: 2021 NVIDIA Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "usd_reader_skeleton.h"
#include "usd_skel_convert.h"

#include "BKE_idprop.h"
#include "BKE_armature.h"
#include "BKE_object.h"

#include "BLI_math.h"

#include "DNA_armature_types.h"
#include "DNA_object_types.h"

#include "ED_armature.h"

#include "MEM_guardedalloc.h"

#include "WM_api.h"

#include <pxr/pxr.h>
#include <pxr/usd/usdSkel/cache.h>
#include <pxr/usd/usdSkel/skeletonQuery.h>

#include <iostream>

namespace blender::io::usd {

bool USDSkeletonReader::valid() const
{
  return skel_ && USDXformReader::valid();
}

void USDSkeletonReader::create_object(Main *bmain, const double /* motionSampleTime */)
{
  object_ = BKE_object_add_only_object(bmain, OB_ARMATURE, name_.c_str());

  bArmature *arm = BKE_armature_add(bmain, name_.c_str());
  object_->data = arm;
}

void USDSkeletonReader::read_object_data(Main *bmain, const double motionSampleTime)
{
  if (!object_ || !object_->data || !skel_) {
    return;
  }

  pxr::UsdSkelCache skel_cache;
  pxr::UsdSkelSkeletonQuery skel_query = skel_cache.GetSkelQuery(skel_);

  if (!skel_query.IsValid()) {
    std::cout << "WARNING: couldn't query skeleton " << skel_.GetPath() << std::endl;
    return;
  }

  const pxr::UsdSkelTopology &skel_topology = skel_query.GetTopology();

  pxr::VtTokenArray joint_order = skel_query.GetJointOrder();

  if (joint_order.size() != skel_topology.size()) {
    std::cout << "WARNING: skel topology and joint order size mismatch\n";
    return;
  }

  bArmature *arm = static_cast<bArmature *>(object_->data);

  ED_armature_to_edit(arm);

  /* The bones we create, stored in the skeleton's joint order. */
  std::vector<EditBone *> edit_bones;

  size_t num_joints = skel_topology.GetNumJoints();

  /* Keep track of the bones we create for each joint. */
  std::map<pxr::TfToken, std::string> joint_to_bone_map;

  /* Create the bones. */
  for (const pxr::TfToken &joint : joint_order) {
    std::string name = pxr::SdfPath(joint).GetName();
    EditBone * bone = ED_armature_ebone_add(arm, name.c_str());
    if (!bone) {
      std::cout << "WARNING: couldn't add bone for joint " << joint << std::endl;
      edit_bones.push_back(nullptr);
      continue;
    }
    joint_to_bone_map.insert(std::make_pair(joint, bone->name));
    edit_bones.push_back(bone);
  }

  /* Sanity check: we should have created a bone for each joint. */

  if (edit_bones.size() != num_joints) {
    std::cout << "WARNING: mismatch in bone and joint counts for skeleton " << skel_.GetPath() << std::endl;
    return;
  }

  /* Record the child bone indices per parent bone. */
  std::vector<std::vector<int>> child_bones(num_joints);

  /* Set bone parenting. */
  for (size_t i = 0; i < num_joints; ++i) {
    int parent_idx = skel_topology.GetParent(i);
    if (parent_idx < 0) {
      continue;
    }
    if (parent_idx >= edit_bones.size()) {
      std::cout << "WARNING: out of bounds parent index for bone " << pxr::SdfPath(joint_order[i])
                << " for skeleton " << skel_.GetPath() << std::endl;
      continue;
    }

    child_bones[parent_idx].push_back(i);
    if (edit_bones[i] && edit_bones[parent_idx]) {
      edit_bones[i]->parent = edit_bones[parent_idx];
    }
  }

  /* Joint bind transforms. */
  pxr::VtMatrix4dArray bind_xforms;
  if (!skel_query.GetJointWorldBindTransforms(&bind_xforms)) {
    std::cout << "WARNING: couldn't get world bind transforms for skeleton "
              << skel_query.GetSkeleton().GetPrim().GetPath() << std::endl;
    return;
  }

  if (bind_xforms.size() != num_joints) {
    std::cout << "WARNING: mismatch in local space rest xforms and joint counts for skeleton " << skel_.GetPath() << std::endl;
    return;
  }

  /* Check if any bone natrices have negative determinants,
   * indicating negative scales, possibly due to mirroring
   * operations.  Such matrices can't be propery converted
   * to Blender's axis/roll bone representation (see
   * https://developer.blender.org/T82930).  If we detect
   * such matrices, we will flag an error and won't try
   * to import the animation, since the rotations would
   * be incorrect in such cases.  Unfortunately, the Pixar
   * UsdSkel examples of the "HumanFemale" suffer from
   * this issue. */
  bool negative_determinant = false;

  /* Set bone rest transforms. */
  for (size_t i = 0; i < num_joints; ++i) {
    EditBone *ebone = edit_bones[i];

    if (!ebone) {
      continue;
    }

    pxr::GfMatrix4f mat(bind_xforms[i]);

    float mat4[4][4];
    mat.Get(mat4);

    pxr::GfVec3f head(0.0f, 0.0f, 0.0f);
    pxr::GfVec3f tail(0.0f, 1.0f, 0.0f);

    copy_v3_v3(ebone->head, head.data());
    copy_v3_v3(ebone->tail, tail.data());

    ED_armature_ebone_from_mat4(ebone, mat4);

    if (mat.GetDeterminant() < 0.0) {
      negative_determinant = true;
    }
  }

  bool valid_skeleton = true;
  if (negative_determinant) {
    valid_skeleton = false;
    WM_reportf(RPT_WARNING,
               "USD Skeleton Import: bone matrices with negative determinants detected in prim %s."
               "Such matrices may indicate negative scales, possibly due to mirroring operations, "
               "and can't currently be converted to Blender's bone representation.  "
               "The skeletal animation won't be imported", prim_.GetPath().GetAsString().c_str());
  }

  /* Scale bones to account for separation between parents and
   * children, so that the bone size is in proportion with the
   * overall skeleton hierarchy.  USD skeletons are composed of
   * joints which we imperfectly represent as bones. */

  float avg_len_scale = 0;
  for (size_t i = 0; i < num_joints; ++i) {

    /* If the bone has any children, scale its length
     * by the distance between this bone's head
     * and the average head location of its children. */

    if (child_bones[i].empty()) {
      continue;
    }

    EditBone *parent = edit_bones[i];
    if (!parent) {
      continue;
    }

    pxr::GfVec3f avg_child_head(0);
    for (int j : child_bones[i]) {
      EditBone *child = edit_bones[j];
      if (!child) {
        continue;
      }
      pxr::GfVec3f child_head(child->head);
      avg_child_head += child_head;
    }

    avg_child_head /= child_bones[i].size();

    pxr::GfVec3f parent_head(parent->head);
    pxr::GfVec3f parent_tail(parent->tail);

    float new_len = (avg_child_head - parent_head).GetLength();

    /* Be sure not to scale by zero. */
    if (new_len > .00001) {
      parent_tail = parent_head + (parent_tail - parent_head).GetNormalized() * new_len;
      copy_v3_v3(parent->tail, parent_tail.data());
      avg_len_scale += new_len;
    }
  }

  /* Scale terminal bones by the average length scale. */
  avg_len_scale /= num_joints;

  if (avg_len_scale > .00001) {
    for (size_t i = 0; i < num_joints; ++i) {
      if (!child_bones[i].empty()) {
        continue;
      }
      EditBone *bone = edit_bones[i];
      if (!bone) {
        continue;
      }
      pxr::GfVec3f head(bone->head);
      pxr::GfVec3f tail(bone->tail);
      tail = head + (tail - head).GetNormalized() * avg_len_scale;
      copy_v3_v3(bone->tail, tail.data());
    }
  }

  ED_armature_from_edit(bmain, arm);
  ED_armature_edit_free(arm);

  if (valid_skeleton) {
    create_skeleton_curves(bmain, object_, skel_query, joint_to_bone_map);
  }

  USDXformReader::read_object_data(bmain, motionSampleTime);
}

}  // namespace blender::io::usd
