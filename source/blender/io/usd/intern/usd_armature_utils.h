/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <pxr/base/tf/token.h>
#include <pxr/usd/usdSkel/animation.h>

#include <functional>

struct Bone;
struct Depsgraph;
struct ModifierData;
struct Object;
struct Scene;
struct USDExportParams;

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

/**
 * Check if the given object has an armature modifier enabled for the
 * given dependency graph's evaluation mode (viewport or render).
 *
 * \param obj: Object to query for the modifier
 * \param depsgraph: The dependency graph in which the object was evaluated
 * \return: True if the object has an enabled armature modifier, false otherwise
 */
bool has_enabled_armature_modifier(const Object *obj, const Depsgraph *depsgraph);

/* The result of querying an object for an enabled modifier of a given type.
 *
 * #ModifierQueryResult::first is the pointer to the modifier, or null if the modifier
 * couldn't be found.
 *
 * #ModifierQueryResult::second is the total number of enabled modifiers of any type
 * found on the object. */
using ModifierQueryResult = std::pair<ModifierData *, int>;

ModifierQueryResult get_enabled_armature_modifier(const Object *obj,
                                                  const Depsgraph *depsgraph);

/**
 * If the given object has an armature modifier, return the
 * armature object bound to the modifier.
 *
 * \param params: Current export parameters
 * \param: Object to check for the modifier
 * \return: The armature object
 */
const Object *get_armature_modifier_obj(const Object *obj);

/**
 * If the given object has an armature modifier, query whether the given
 * name matches the name of a bone on the armature referenced by the modifier.
 *
 * \param obj: Object to query for the modifier
 * \return: True if the name matches a bone name.  Return false if no matching
 *          bone name is found or if the object does not have an armature modifier
 */
bool is_armature_modifier_bone_name(const Object *obj, const char *name);

bool can_export_skinned_mesh(const Object *obj, const Depsgraph *depsgraph);

}  // namespace blender::io::usd
