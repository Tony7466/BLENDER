/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <pxr/base/tf/token.h>
#include <pxr/usd/usdSkel/animation.h>

#include <functional>

struct Bone;
struct Object;

namespace blender::io::usd {

/**
 * Recursively invoke the given function on the given armature object's bones.
 * This function is a no-op if the object isn't an armature.
 *
 * \param ob_arm: The armature object
 * \param visitor: The function to invoke on each bone
 */
void visit_bones(const Object *ob_arm, std::function<void(const Bone *)> visitor);

/**
 * Return in 'r_names' the names of the given armature object's bones.
 *
 * \param ob_arm: The armature object
 * \param r_names: The returned list of bone names
 */
void get_armature_bone_names(const Object *ob_arm, std::vector<std::string> &r_names);

/**
 * Return the USD joint path corresponding to the given bone. For example, for the bone
 * "Hand", this function might return the full path "Shoulder/Elbow/Hand" of the joint
 * in the hierachy.
 *
 * \param bone: The bone whose path will be queried.
 * \return: The path to the joint
 */
pxr::TfToken build_usd_joint_path(const Bone *bone);

/**
 * Sets the USD joint paths as an attribute on the given USD animation,
 * where the paths correspond to the bones of the given armature.
 *
 * \param skel_anim: The animation whose joints attribute will be set
 * \param ob_arm: The armature object
 */
void create_pose_joints(pxr::UsdSkelAnimation &skel_anim, const Object *obj);

}  // namespace blender::io::usd
