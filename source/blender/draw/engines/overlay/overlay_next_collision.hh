/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "overlay_next_bounds.hh"

#include "DNA_rigidbody_types.h"

namespace blender::draw::overlay {

static void collision_sync(const ObjectRef &ob_ref,
                           const select::ID select_id,
                           Resources & /*res*/,
                           const State & /*state*/,
                           ExtraInstancePasses &passes,
                           ExtraInstanceData data)
{
  Object *ob = ob_ref.object;

  switch (ob->rigidbody_object->shape) {
    case RB_SHAPE_BOX:
      bounds_sync_base(ob_ref, select_id, passes, data, OB_BOUND_BOX, true);
      break;
    case RB_SHAPE_SPHERE:
      bounds_sync_base(ob_ref, select_id, passes, data, OB_BOUND_SPHERE, true);
      break;
    case RB_SHAPE_CONE:
      bounds_sync_base(ob_ref, select_id, passes, data, OB_BOUND_CONE, true);
      break;
    case RB_SHAPE_CYLINDER:
      bounds_sync_base(ob_ref, select_id, passes, data, OB_BOUND_CYLINDER, true);
      break;
    case RB_SHAPE_CAPSULE:
      bounds_sync_base(ob_ref, select_id, passes, data, OB_BOUND_CAPSULE, true);
      break;
  }
}

}  // namespace blender::draw::overlay
