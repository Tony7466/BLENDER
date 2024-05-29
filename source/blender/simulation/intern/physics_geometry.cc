/* SPDX-FileCopyrightText: 2004-2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sim
 */

#include "BLI_array_utils.hh"
#include "BLI_mempool.h"
#include "BLI_virtual_array.hh"

#include "SIM_collision_shape.hh"
#include "SIM_physics_geometry.hh"

#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <LinearMath/btMotionState.h>
#include <LinearMath/btTransform.h>
#include <functional>

#include "physics_impl.hh"

#ifdef WITH_BULLET
#  include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
#  include <BulletCollision/Gimpact/btGImpactShape.h>
#  include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#  include <BulletDynamics/Dynamics/btRigidBody.h>
#  include <LinearMath/btDefaultMotionState.h>
#  include <btBulletDynamicsCommon.h>
#endif

namespace blender::simulation {

template<typename T, int chunk_size, bool allow_iterator> class MemPool {
 private:
  BLI_mempool *pool_;

 public:
  MemPool(const int size = 0)
  {
    const int flag = allow_iterator ? BLI_MEMPOOL_ALLOW_ITER : BLI_MEMPOOL_NOP;
    pool_ = BLI_mempool_create(sizeof(T), size, chunk_size, flag);
  }

  ~MemPool()
  {
    BLI_mempool_destroy(pool_);
  }

  template<typename... Args> T *alloc(Args &&...args)
  {
    return new (BLI_mempool_alloc(pool_)) T(std::forward<Args>(args)...);
  }

  void free(T *ptr)
  {
    BLI_mempool_free(pool_, ptr);
  }

  int size() const
  {
    return BLI_mempool_len(pool_);
  }
};

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
  using OverlapFilterFn = std::function<bool(const RigidBodyID a, const RigidBodyID b)>;

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

PhysicsGeometry::PhysicsGeometry()
{
  impl_ = new PhysicsImpl{};
}

PhysicsGeometry::PhysicsGeometry(const PhysicsGeometry &other)
{
  impl_ = new PhysicsImpl{};

  if (other.has_world()) {
    this->set_world(true);
  }

  // TODO is this possible and/or necessary?

  // impl_->rigid_bodies.reinitialize(other.impl_->rigid_bodies.size());
  // impl_->motion_states.reinitialize(other.impl_->motion_states.size());
  // for (const int i : other.impl_->rigid_bodies.index_range()) {
  //   const btRigidBody &src_body = *other.impl_->rigid_bodies[i];
  //   const btMotionState &src_motion_state = *other.impl_->motion_states[i];
  //   btTransform start_transform, center_of_mass;
  //   src_motion_state.getWorldTransform(start_transform);
  //   btMotionState *dst_motion_state = new btDefaultMotionState(start_transform);
  //   const float3 local_inertia = float3(0.0f);

  //   const CollisionShapeID shape_id = src_body.getCollisionShape()->getUserIndex();
  //   std::shared_ptr<CollisionShape> shape = impl_->collision_shapes.lookup(shape_id);
  //   btCollisionShape *bt_shape = shape ? &shape->impl_->as_bullet_shape() : nullptr;

  //   btRigidBody::btRigidBodyConstructionInfo constructionInfo(
  //       src_body.getMass(), dst_motion_state, bt_shape, to_bullet(local_inertia));

  //   impl_->rigid_bodies[i] = other.impl_->rigid_bodies[i];
  // }
}

PhysicsGeometry::~PhysicsGeometry()
{
  clear_rigid_bodies();
  set_world(false);
}

bool PhysicsGeometry::has_world() const
{
  return impl_->world != nullptr;
}

void PhysicsGeometry::set_world(bool enable)
{
  if (enable) {
    if (impl_->world == nullptr) {
      impl_->config = new btDefaultCollisionConfiguration();
      impl_->dispatcher = new btCollisionDispatcher(impl_->config);
      btGImpactCollisionAlgorithm::registerAlgorithm((btCollisionDispatcher *)impl_->dispatcher);

      impl_->broadphase = new btDbvtBroadphase();
      impl_->overlap_filter = new DefaultOverlapFilter();
      impl_->broadphase->getOverlappingPairCache()->setOverlapFilterCallback(
          impl_->overlap_filter);

      impl_->constraint_solver = new btSequentialImpulseConstraintSolver();

      impl_->world = new btDiscreteDynamicsWorld(
          impl_->dispatcher, impl_->broadphase, impl_->constraint_solver, impl_->config);
    }
  }
  else {
    if (impl_->world != nullptr) {
      delete impl_->world;
      delete impl_->constraint_solver;
      delete impl_->broadphase;
      delete impl_->dispatcher;
      delete impl_->config;
      delete impl_->overlap_filter;
    }
  }
}

void PhysicsGeometry::set_overlap_filter(OverlapFilterFn fn)
{
  if (impl_->world == nullptr) {
    return;
  }
  BLI_assert(impl_->broadphase);

  impl_->overlap_filter = new OverlapFilterWrapper(std::move(fn));
  impl_->broadphase->getOverlappingPairCache()->setOverlapFilterCallback(impl_->overlap_filter);
}

void PhysicsGeometry::clear_overlap_filter()
{
  if (impl_->world == nullptr) {
    return;
  }
  BLI_assert(impl_->broadphase);

  if (impl_->overlap_filter) {
    delete impl_->overlap_filter;
    impl_->overlap_filter = new DefaultOverlapFilter();
  }
  impl_->broadphase->getOverlappingPairCache()->setOverlapFilterCallback(impl_->overlap_filter);
}

float3 PhysicsGeometry::gravity() const
{
  if (impl_->world) {
    return to_blender(impl_->world->getGravity());
  }
  return float3(0.0f);
}

void PhysicsGeometry::set_gravity(const float3 &gravity)
{
  if (impl_->world) {
    impl_->world->setGravity(to_bullet(gravity));
  }
}

void PhysicsGeometry::set_solver_iterations(const int num_solver_iterations)
{
  if (impl_->world) {
    btContactSolverInfo &info = impl_->world->getSolverInfo();
    info.m_numIterations = num_solver_iterations;
  }
}

void PhysicsGeometry::set_split_impulse(const bool split_impulse)
{
  if (impl_->world) {
    btContactSolverInfo &info = impl_->world->getSolverInfo();
    /* Note: Bullet stores this as int, but it's used as a bool. */
    info.m_splitImpulse = int(split_impulse);
  }
}

int PhysicsGeometry::rigid_bodies_num() const
{
  return impl_->rigid_bodies.size();
}

int PhysicsGeometry::constraints_num() const
{
  return 0;
}

int PhysicsGeometry::shapes_num() const
{
  return 0;
}

template<typename T, typename GetFn>
static VArray<T> varray_for_rigid_bodies(const PhysicsImpl &impl, GetFn get_fn)
{
  return VArray<T>::ForFunc(impl.rigid_bodies.size(), [&](const int64_t index) {
    return get_fn(*impl.rigid_bodies[index]);
  });
}

IndexRange PhysicsGeometry::add_rigid_bodies(const Span<const CollisionShape *> shapes,
                                             const VArray<int> &shape_indices,
                                             const VArray<float> &masses,
                                             const VArray<float3> &inertiae,
                                             const VArray<bool> &simulated)
{
  const int num_add = masses.size();
  BLI_assert(inertiae.is_empty() || inertiae.size() == num_add);
  BLI_assert(shape_indices.size() == num_add);

  const IndexRange old_bodies_range = impl_->rigid_bodies.index_range();
  const IndexRange new_bodies_range = old_bodies_range.after(num_add);
  if (new_bodies_range.is_empty()) {
    return new_bodies_range;
  }
  Array<btRigidBody *> new_rigid_bodies(impl_->rigid_bodies.size() + num_add);
  Array<btMotionState *> new_motion_states(impl_->motion_states.size() + num_add);
  array_utils::copy(impl_->rigid_bodies.as_span(),
                    new_rigid_bodies.as_mutable_span().slice(old_bodies_range));
  array_utils::copy(impl_->motion_states.as_span(),
                    new_motion_states.as_mutable_span().slice(old_bodies_range));
  for (const int i : new_bodies_range) {
    BLI_assert(shapes.index_range().contains(shape_indices[i]));
    const CollisionShape *shape = shapes[shape_indices[i]];
    const float3 inertia = inertiae ? inertiae[i] : float3(0.0f);
    const bool simulate = simulated ? simulated[i] : true;

    new_motion_states[i] = new btDefaultMotionState();
    new_rigid_bodies[i] = new btRigidBody(masses[i],
                                          new_motion_states[i],
                                          shape ? &shape->impl_->as_bullet_shape() : nullptr,
                                          to_bullet(inertia));
    if (simulate && impl_->world) {
      impl_->world->addRigidBody(new_rigid_bodies[i]);
    }
  }
  impl_->motion_states = std::move(new_motion_states);
  impl_->rigid_bodies = std::move(new_rigid_bodies);
  return new_bodies_range;
}

void PhysicsGeometry::remove_rigid_bodies(const IndexMask &mask)
{
  mask.foreach_index([&](const int index) {
    btRigidBody *body = impl_->rigid_bodies[index];
    btMotionState *motion_state = impl_->motion_states[index];
    BLI_assert(body != nullptr);
    BLI_assert(motion_state != nullptr);

    if (impl_->world) {
      impl_->world->removeRigidBody(body);
    }

    delete impl_->rigid_bodies[index];
    delete impl_->motion_states[index];
  });

  /* Remove entries. */
  IndexMaskMemory keep_mask_memory;
  IndexMask keep_mask = mask.complement(impl_->rigid_bodies.index_range(), keep_mask_memory);
  Array<btRigidBody *> new_rigid_bodies(impl_->rigid_bodies.size() - mask.size());
  Array<btMotionState *> new_motion_states(impl_->motion_states.size() - mask.size());
  array_utils::copy(impl_->rigid_bodies.as_span(), keep_mask, new_rigid_bodies.as_mutable_span());
  array_utils::copy(
      impl_->motion_states.as_span(), keep_mask, new_motion_states.as_mutable_span());
  impl_->rigid_bodies = std::move(new_rigid_bodies);
  impl_->motion_states = std::move(new_motion_states);
}

void PhysicsGeometry::clear_rigid_bodies()
{
  for (const int index : impl_->rigid_bodies.index_range()) {
    btRigidBody *body = impl_->rigid_bodies[index];
    btMotionState *motion_state = impl_->motion_states[index];
    BLI_assert(body != nullptr);
    BLI_assert(motion_state != nullptr);

    if (impl_->world) {
      impl_->world->removeRigidBody(body);
    }

    delete body;
    delete motion_state;
  }

  impl_->rigid_bodies = {};
  impl_->motion_states = {};
}

VArray<RigidBodyID> PhysicsGeometry::body_ids() const
{
  return varray_for_rigid_bodies<RigidBodyID>(
      *impl_, [&](const btRigidBody &body) { return body.getUserIndex(); });
}

#else

// TODO add stub functions for when Bullet is disabled

#endif

}  // namespace blender::simulation
