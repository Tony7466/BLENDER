/* SPDX-FileCopyrightText: 2004-2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sim
 */

#include "MEM_guardedalloc.h"

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
    const int64_t body0 = int64_t(proxy0->m_clientObject);
    const int64_t body1 = int64_t(proxy1->m_clientObject);
    return (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) &&
           (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask) && fn(body0, body1);
  }
};

struct RigidBodyWorldImpl {
  btDiscreteDynamicsWorld *world;
  btCollisionConfiguration *config;
  btCollisionDispatcher *dispatcher;
  btBroadphaseInterface *broadphase;
  btConstraintSolver *constraint_solver;
  btOverlapFilterCallback *overlap_filter;

  RigidBodyWorldImpl()
  {
    this->config = MEM_new<btDefaultCollisionConfiguration>(__func__);
    this->dispatcher = MEM_new<btCollisionDispatcher>(__func__, this->config);
    btGImpactCollisionAlgorithm::registerAlgorithm((btCollisionDispatcher *)this->dispatcher);

    this->broadphase = MEM_new<btDbvtBroadphase>(__func__);
    this->overlap_filter = MEM_new<DefaultOverlapFilter>(__func__);
    this->broadphase->getOverlappingPairCache()->setOverlapFilterCallback(this->overlap_filter);

    this->constraint_solver = MEM_new<btSequentialImpulseConstraintSolver>(__func__);

    this->world = MEM_new<btDiscreteDynamicsWorld>(
        __func__, this->dispatcher, this->broadphase, this->constraint_solver, this->config);
  }

  ~RigidBodyWorldImpl() {
    /* XXX This leaks memory, but enabling it somehow causes memory corruption and crash.
     * Possibly caused by alignment? */
    // MEM_delete(this->world);
    MEM_delete(this->constraint_solver);
    MEM_delete(this->broadphase);
    MEM_delete(this->dispatcher);
    MEM_delete(this->config);
    MEM_delete(this->overlap_filter);
  }
};

RigidBodyWorld::RigidBodyWorld() {
  impl_ = MEM_new<RigidBodyWorldImpl>(__func__);
}

RigidBodyWorld::RigidBodyWorld(const RigidBodyWorld &other)
{
  // TODO copy bodies, constraints, shapes from other?
  impl_ = MEM_new<RigidBodyWorldImpl>(__func__);
  UNUSED_VARS(other);
}

RigidBodyWorld::~RigidBodyWorld()
{
  MEM_delete(impl_);
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
  if (impl_->overlap_filter) {
    delete impl_->overlap_filter;
    impl_->overlap_filter = new DefaultOverlapFilter();
  }
  impl_->broadphase->getOverlappingPairCache()->setOverlapFilterCallback(impl_->overlap_filter);
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
