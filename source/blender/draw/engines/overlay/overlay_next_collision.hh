/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "overlay_next_bounds.hh"

#include "DNA_rigidbody_types.h"

namespace blender::draw::overlay {

class CollisionPasses : public BoundPassesBase {

 public:
  CollisionPasses(SelectionType selection_type,
                  const ShapeCache &shapes,
                  const GlobalsUboStorage &theme_colors,
                  bool in_front)
      : BoundPassesBase("Collisions", selection_type, shapes, theme_colors, in_front){};

  virtual void object_sync(const ObjectRef &ob_ref,
                           const select::ID select_id,
                           Resources &res,
                           const State &state) final override
  {
    Object *ob = ob_ref.object;
    const bool from_dupli = ob->base_flag & (BASE_FROM_SET | BASE_FROM_DUPLI);
    if (from_dupli || ob->rigidbody_object == nullptr) {
      return;
    }

    int bound_type;
    switch (ob->rigidbody_object->shape) {
      case RB_SHAPE_BOX:
        bound_type = OB_BOUND_BOX;
        break;
      case RB_SHAPE_SPHERE:
        bound_type = OB_BOUND_SPHERE;
        break;
      case RB_SHAPE_CONE:
        bound_type = OB_BOUND_CONE;
        break;
      case RB_SHAPE_CYLINDER:
        bound_type = OB_BOUND_CYLINDER;
        break;
      case RB_SHAPE_CAPSULE:
        bound_type = OB_BOUND_CAPSULE;
        break;
      default:
        BLI_assert_unreachable();
    }

    bounds_sync(ob_ref, select_id, res, state, bound_type, true);
  }
};

using Collision = OverlayType<CollisionPasses>;

}  // namespace blender::draw::overlay
