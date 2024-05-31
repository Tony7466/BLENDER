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

class btDiscreteDynamicsWorld;
class btCollisionConfiguration;
class btCollisionDispatcher;
class btBroadphaseInterface;
class btConstraintSolver;
class btOverlapFilterCallback;
class btRigidBody;
class btMotionState;
class btCollisionShape;

namespace blender::bke {

struct PhysicsGeometryImpl : public ImplicitSharingMixin {
  PhysicsWorldImpl *world_impl = nullptr;
  Array<btRigidBody *> rigid_bodies;
  Array<btMotionState *> motion_states;

  PhysicsGeometryImpl();
  ~PhysicsGeometryImpl();

  void delete_self() override;

  PhysicsGeometryImpl *copy() const;
};

struct PhysicsWorldImpl {
  btDiscreteDynamicsWorld *world = nullptr;
  btCollisionConfiguration *config = nullptr;
  btCollisionDispatcher *dispatcher = nullptr;
  btBroadphaseInterface *broadphase = nullptr;
  btConstraintSolver *constraint_solver = nullptr;
  btOverlapFilterCallback *overlap_filter = nullptr;

  PhysicsWorldImpl();
  ~PhysicsWorldImpl();

  PhysicsWorldImpl *copy() const;
};

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
