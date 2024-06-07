/* SPDX-FileCopyrightText: Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sim
 */

#pragma once

#include "BKE_physics_geometry.hh"

#include "BLI_array.hh"
#include "BLI_implicit_sharing.h"
#include "BLI_map.hh"

#include <shared_mutex>

class btDiscreteDynamicsWorld;
class btCollisionConfiguration;
class btCollisionDispatcher;
class btBroadphaseInterface;
class btConstraintSolver;
class btOverlapFilterCallback;
class btRigidBody;
class btMotionState;
class btCollisionShape;
class btTypedConstraint;

namespace blender::bke {

/**
 * Physics data can be in one of three states:
 * - Mutable: Only one user and has physics data.
 * - Read-Only: Has physics data, but more than one user.
 * - Cached: No physics data (user count irrelevant, always read-only).
 *
 * State can change between Mutable and Read-Only when adding or removing users.
 * Once physics data has been moved the component becomes Cached, at which point going back to
 * mutable or read-only state is impossible (data cannot be added back).
 */

struct PhysicsGeometryImpl : public ImplicitSharingMixin {
  /* If true then the data is read-only and the attribute cache is used instead of direct access to
   * physics data. No Bullet instances are held by this component when cached. */
  std::atomic<bool> is_cached = false;

  /* Protects shared read/write access to physics data. */
  mutable std::shared_mutex data_mutex;

  btDiscreteDynamicsWorld *world = nullptr;
  btCollisionConfiguration *config = nullptr;
  btCollisionDispatcher *dispatcher = nullptr;
  btBroadphaseInterface *broadphase = nullptr;
  btConstraintSolver *constraint_solver = nullptr;
  btOverlapFilterCallback *overlap_filter = nullptr;

  Array<btRigidBody *> rigid_bodies;
  Array<btMotionState *> motion_states;
  Array<btTypedConstraint *> constraints;
  Array<btCollisionShape *> shapes;

  /* Physics data can be moved while other components still have write access. The physics data is
   * cached for read access, so that data can be moved without requiring locks. */
  struct AttributeCache {
    Array<float3> body_positions;
    Array<math::Quaternion> body_rotations;
    Array<float3> body_velocities;
    Array<float3> body_angular_velocities;
  };
  AttributeCache attribute_cache;

  PhysicsGeometryImpl();
  ~PhysicsGeometryImpl();

  void delete_self() override;
};

void move_physics_impl_data(const PhysicsGeometryImpl &from,
                            PhysicsGeometryImpl &to,
                            bool use_world,
                            int rigid_bodies_offset,
                            int constraints_offset,
                            int shapes_offset);

struct CollisionShapeImpl {
  btCollisionShape &as_bullet_shape()
  {
    return *reinterpret_cast<btCollisionShape *>(this);
  }

  const btCollisionShape &as_bullet_shape() const
  {
    return *reinterpret_cast<const btCollisionShape *>(this);
  }

  operator btCollisionShape *()
  {
    return reinterpret_cast<btCollisionShape *>(this);
  }

  operator const btCollisionShape *() const
  {
    return reinterpret_cast<const btCollisionShape *>(this);
  }

  static CollisionShapeImpl *wrap(btCollisionShape *shape)
  {
    return reinterpret_cast<CollisionShapeImpl *>(shape);
  }

  static const CollisionShapeImpl *wrap(const btCollisionShape *shape)
  {
    return reinterpret_cast<const CollisionShapeImpl *>(shape);
  }
};

}  // namespace blender::bke
