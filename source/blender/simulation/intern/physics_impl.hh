/* SPDX-FileCopyrightText: Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sim
 */

#pragma once

#include "BLI_array.hh"

class btDiscreteDynamicsWorld;
class btCollisionConfiguration;
class btCollisionDispatcher;
class btBroadphaseInterface;
class btConstraintSolver;
class btOverlapFilterCallback;
class btRigidBody;
class btMotionState;
class btCollisionShape;

namespace blender::simulation {

struct PhysicsImpl {
  btDiscreteDynamicsWorld *world;
  btCollisionConfiguration *config;
  btCollisionDispatcher *dispatcher;
  btBroadphaseInterface *broadphase;
  btConstraintSolver *constraint_solver;
  btOverlapFilterCallback *overlap_filter;

  Array<btRigidBody *> rigid_bodies;
  Array<btMotionState *> motion_states;
};

struct CollisionShapeImpl {
  btCollisionShape *shape;
};

}  // namespace blender::simulation
