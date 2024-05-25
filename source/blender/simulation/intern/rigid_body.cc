/* SPDX-FileCopyrightText: 2004-2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sim
 */

#include "SIM_rigid_body.hh"

#ifdef WITH_BULLET
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "btBulletDynamicsCommon.h"
#endif

namespace blender::simulation {

#ifdef WITH_BULLET

inline float3 to_blender(const btVector3 &v) {
  return float3(v);
}

inline math::Quaternion to_blender(const btQuaternion &q)
{
  return math::Quaternion(q.w(), q.x(), q.y(), q.z());
}

inline btVector3 to_bullet(const float3 &v)
{
  return btVector3(v.x, v.y, v.z);
}

inline btQuaternion to_bullet(const math::Quaternion &q)
{
  return btQuaternion(q.x, q.y, q.z, q.w);
}

struct RigidBodyWorldImpl {
  btCollisionConfiguration *config;
  btCollisionDispatcher *dispatcher; 
  btDynamicsWorld *world;
  btBroadphaseInterface *broadphase;
  btConstraintSolver *constraint_solver;
  btOverlapFilterCallback *overlap_filter;

  RigidBodyWorldImpl()
  {
    this->config = new btDefaultCollisionConfiguration();
    this->dispatcher = new btCollisionDispatcher(this->config);
    btGImpactCollisionAlgorithm::registerAlgorithm((btCollisionDispatcher *)this->dispatcher);

    this->broadphase = new btDbvtBroadphase();
    this->overlap_filter = nullptr;

    this->constraint_solver = new btSequentialImpulseConstraintSolver();

    this->world = new btDiscreteDynamicsWorld(
        this->dispatcher, this->broadphase, this->constraint_solver, this->config);
  }
};

struct DefaultOverlapFilter : public btOverlapFilterCallback {
  virtual bool needBroadphaseCollision(btBroadphaseProxy *proxy0, btBroadphaseProxy *proxy1) const
  {
    return (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) &&
           (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);
  }
};

struct OverlapFilterWrapper : public btOverlapFilterCallback {
  using OverlapFilterFn = RigidBodyWorld::OverlapFilterFn;

  OverlapFilterFn fn;

  OverlapFilterWrapper(OverlapFilterFn fn) : fn(fn) {}

  virtual bool needBroadphaseCollision(btBroadphaseProxy *proxy0, btBroadphaseProxy *proxy1) const
  {
    const int64_t body0 =  int64_t(proxy0->m_clientObject);
    const int64_t body1 = int64_t(proxy1->m_clientObject);
    return (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) &&
           (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask) && fn(body0, body1);
  }
};

RigidBodyWorld::RigidBodyWorld() {
  impl_ = new RigidBodyWorldImpl();
}

RigidBodyWorld::RigidBodyWorld(const RigidBodyWorld &other)
{
  // TODO copy bodies, constraints, shapes from other?
  impl_ = new RigidBodyWorldImpl();
  UNUSED_VARS(other);
}

RigidBodyWorld::~RigidBodyWorld()
{
  delete impl_->world;
  delete impl_->constraint_solver;
  delete impl_->broadphase;
  delete impl_->dispatcher;
  delete impl_->config;
  delete impl_->overlap_filter;
  delete impl_;
}

int RigidBodyWorld::bodies_num() const
{
  return 0;
}

int RigidBodyWorld::constraints_num() const
{
  return 0;
}

int RigidBodyWorld::shapes_num() const
{
  return 0;
}

void RigidBodyWorld::set_overlap_filter(OverlapFilterFn fn)
{
  impl_->overlap_filter = new OverlapFilterWrapper(std::move(fn));
  impl_->broadphase->getOverlappingPairCache()->setOverlapFilterCallback(impl_->overlap_filter);
}

void RigidBodyWorld::clear_overlap_filter() {
  impl_->broadphase->getOverlappingPairCache()->setOverlapFilterCallback(nullptr);
  if (impl_->overlap_filter) {
    delete impl_->overlap_filter;
    impl_->overlap_filter = nullptr;
  }
}

float3 RigidBodyWorld::gravity() const
{
  return to_blender(impl_->world->getGravity());
}

void RigidBodyWorld::set_gravity(const float3 &gravity)
{
  impl_->world->setGravity(to_bullet(gravity));
}

void RigidBodyWorld::set_solver_iterations(const int num_solver_iterations)
{
  btContactSolverInfo &info = impl_->world->getSolverInfo();
  info.m_numIterations = num_solver_iterations;
}

void RigidBodyWorld::set_split_impulse(const bool split_impulse)
{
  btContactSolverInfo &info = impl_->world->getSolverInfo();
  /* Note: Bullet stores this as int, but it's used as a bool. */
  info.m_splitImpulse = int(split_impulse);
}

#else

RigidBodyWorld::RigidBodyWorld() {}

RigidBodyWorld::~RigidBodyWorld() {}

int RigidBodyWorld::bodies_num() const
{
  return 0;
}

int RigidBodyWorld::constraints_num() const
{
  return 0;
}

int RigidBodyWorld::shapes_num() const
{
  return 0;
}

void RigidBodyWorld::set_overlap_filter(OverlapFilterFn /*fn*/) {}

void RigidBodyWorld::clear_overlap_filter() {}

#endif

}  // namespace blender::simulation
