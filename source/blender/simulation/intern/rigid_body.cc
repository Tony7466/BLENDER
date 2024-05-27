/* SPDX-FileCopyrightText: 2004-2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sim
 */

#include "BLI_array_utils.hh"
#include "BLI_mempool.h"
#include "BLI_virtual_array.hh"

#include "MEM_guardedalloc.h"

#include "SIM_rigid_body.hh"
#include <LinearMath/btDefaultMotionState.h>

#ifdef WITH_BULLET
#  include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#  include "BulletCollision/Gimpact/btGImpactShape.h"
#  include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#  include "BulletDynamics/Dynamics/btRigidBody.h"
#  include "btBulletDynamicsCommon.h"
#endif

namespace blender::simulation {

#ifdef WITH_BULLET

inline float to_blender(const btScalar v)
{
  return float(v);
}

inline float3 to_blender(const btVector3 &v)
{
  return float3(v.x(), v.y(), v.z());
}

inline math::Quaternion to_blender(const btQuaternion &q)
{
  return math::Quaternion(q.w(), q.x(), q.y(), q.z());
}

inline btScalar to_bullet(const float v)
{
  return btScalar(v);
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
    this->config = new btDefaultCollisionConfiguration();
    this->dispatcher = new btCollisionDispatcher(this->config);
    btGImpactCollisionAlgorithm::registerAlgorithm((btCollisionDispatcher *)this->dispatcher);

    this->broadphase = new btDbvtBroadphase();
    this->overlap_filter = new DefaultOverlapFilter();
    this->broadphase->getOverlappingPairCache()->setOverlapFilterCallback(this->overlap_filter);

    this->constraint_solver = new btSequentialImpulseConstraintSolver();

    this->world = new btDiscreteDynamicsWorld(
        this->dispatcher, this->broadphase, this->constraint_solver, this->config);
  }

  ~RigidBodyWorldImpl()
  {
    /* XXX This leaks memory, but enabling it somehow causes memory corruption and crash.
     * Possibly caused by alignment? */
    // delete this->world;
    delete this->constraint_solver;
    delete this->broadphase;
    delete this->dispatcher;
    delete this->config;
    delete this->overlap_filter;
  }
};

struct RigidBodyImpl {
  btRigidBody *body;
  btMotionState *motion_state;

  RigidBodyImpl(btCollisionShape *collision_shape, float mass, const btVector3 &local_inertia)
  {
    this->motion_state = new btDefaultMotionState();
    this->body = new btRigidBody(mass, this->motion_state, collision_shape, local_inertia);
  }

  ~RigidBodyImpl()
  {
    delete this->body;
  }
};

struct CollisionShapeImpl {
  btCollisionShape *shape;

  ~CollisionShapeImpl()
  {
    delete shape;
  }
};

RigidBodyWorld::RigidBodyWorld()
{
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
  delete impl_;
}

int RigidBodyWorld::bodies_num() const
{
  return impl_->world->getNumCollisionObjects();
}

int RigidBodyWorld::constraints_num() const
{
  return impl_->world->getNumConstraints();
}

void RigidBodyWorld::set_overlap_filter(OverlapFilterFn fn)
{
  impl_->overlap_filter = new OverlapFilterWrapper(std::move(fn));
  impl_->broadphase->getOverlappingPairCache()->setOverlapFilterCallback(impl_->overlap_filter);
}

void RigidBodyWorld::clear_overlap_filter()
{
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

void RigidBodyWorld::add_rigid_body(RigidBody *body)
{
  impl_->world->addRigidBody(body->impl_->body);
}

void RigidBodyWorld::remove_rigid_body(RigidBody *body)
{
  impl_->world->removeRigidBody(body->impl_->body);
}

// float RigidBodyWorld::body_mass(RigidBodyHandle handle) const
//{
//   const btRigidBody *body = reinterpret_cast<const btRigidBody *>(handle);
//   return body->getMass();
// }
//
// float3 RigidBodyWorld::body_inertia(RigidBodyHandle handle) const
//{
//   const btRigidBody *body = reinterpret_cast<const btRigidBody *>(handle);
//   return to_blender(body->getLocalInertia());
// }
//
// void RigidBodyWorld::set_body_mass(RigidBodyHandle handle, const float mass, const float3
// &inertia)
//{
//   btRigidBody *body = reinterpret_cast<btRigidBody *>(handle);
//   body->setMassProps(mass, to_bullet(inertia));
// }
//
// float RigidBodyWorld::body_friction(RigidBodyHandle handle) const {
//   const btRigidBody *body = reinterpret_cast<const btRigidBody *>(handle);
//   return body->getFriction();
// }
//
// void RigidBodyWorld::set_body_friction(RigidBodyHandle handle, float value)
//{
//   btRigidBody *body = reinterpret_cast<btRigidBody *>(handle);
//   body->setFriction(value);
// }
//
// float RigidBodyWorld::body_restitution(RigidBodyHandle handle) const
//{
//   const btRigidBody *body = reinterpret_cast<const btRigidBody *>(handle);
//   return body->getRestitution();
// }
//
// void RigidBodyWorld::body_set_restitution(RigidBodyHandle handle, float value)
//{
//   btRigidBody *body = reinterpret_cast<btRigidBody *>(handle);
//   body->setRestitution(value);
// }
//
// float RigidBodyWorld::body_linear_damping(RigidBodyHandle handle) const
//{
//   const btRigidBody *body = reinterpret_cast<const btRigidBody *>(handle);
//   return body->getLinearDamping();
// }
//
// void RigidBodyWorld::body_set_linear_damping(RigidBodyHandle handle, float value)
//{
//   btRigidBody *body = reinterpret_cast<btRigidBody *>(handle);
//   body->setDamping(value, body->getAngularDamping());
// }
//
// float RigidBodyWorld::body_angular_damping(RigidBodyHandle handle) const
//{
//   const btRigidBody *body = reinterpret_cast<const btRigidBody *>(handle);
//   return body->getAngularDamping();
// }
//
// void RigidBodyWorld::body_set_angular_damping(RigidBodyHandle handle, float value)
//{
//   btRigidBody *body = reinterpret_cast<btRigidBody *>(handle);
//   body->setDamping(body->getLinearDamping(), value);
// }

RigidBody::RigidBody(const CollisionShape *shape, float mass, const float3 &local_inertia)
{
  impl_ = new RigidBodyImpl(shape->impl_->shape, mass, to_bullet(local_inertia));
}

RigidBody::~RigidBody()
{
  delete impl_;
}

CollisionShape::CollisionShape(CollisionShapeImpl *impl) : impl_(impl) {}

CollisionShape::~CollisionShape()
{
  delete impl_;
}

CollisionShape::ShapeType CollisionShape::type() const
{
  const auto bt_shape_type = BroadphaseNativeTypes(impl_->shape->getShapeType());
  switch (bt_shape_type) {
    case BOX_SHAPE_PROXYTYPE:
      return ShapeType::Box;
    case SPHERE_SHAPE_PROXYTYPE:
      return ShapeType::Sphere;
    case EMPTY_SHAPE_PROXYTYPE:
    case INVALID_SHAPE_PROXYTYPE:
    default:
      return ShapeType::Unknown;
  }
}

BoxCollisionShape::BoxCollisionShape(const float3 &half_extent)
    : CollisionShape(new CollisionShapeImpl{new btBoxShape(to_bullet(half_extent))})
{
}

float3 BoxCollisionShape::half_extent() const
{
  return to_blender(static_cast<btBoxShape *>(impl_->shape)->getHalfExtentsWithoutMargin());
}

SphereCollisionShape::SphereCollisionShape(const float radius)
    : CollisionShape(new CollisionShapeImpl{new btSphereShape(radius)})
{
}

float SphereCollisionShape::radius() const
{
  return static_cast<btSphereShape *>(impl_->shape)->getRadius();
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
