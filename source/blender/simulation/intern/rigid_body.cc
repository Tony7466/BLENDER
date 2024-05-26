/* SPDX-FileCopyrightText: 2004-2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sim
 */

#include "BLI_mempool.h"
#include "BLI_virtual_array.hh"

#include "MEM_guardedalloc.h"

#include "SIM_rigid_body.hh"

#ifdef WITH_BULLET
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
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

  MemPool<btRigidBody, 512, true> rigid_body_pool;
  MemPool<btDefaultMotionState, 512, true> motion_state_pool;

  Vector<btRigidBody *> rigid_bodies;
  Set<btCollisionShape *> collision_shapes;

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

  ~RigidBodyWorldImpl()
  {
    BLI_assert(this->rigid_bodies.is_empty());

    /* XXX This leaks memory, but enabling it somehow causes memory corruption and crash.
     * Possibly caused by alignment? */
    // MEM_delete(this->world);
    MEM_delete(this->constraint_solver);
    MEM_delete(this->broadphase);
    MEM_delete(this->dispatcher);
    MEM_delete(this->config);
    MEM_delete(this->overlap_filter);
  }

  template<typename T, typename GetFn> VArray<T> varray_for_rigid_bodies(GetFn get_fn)
  {
    return VArray<T>::ForFunc(this->rigid_bodies.size(), [&](const int64_t index) {
      return get_fn(*this->rigid_bodies[index]);
    });
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
  this->clear_rigid_bodies();
  MEM_delete(impl_);
}

int RigidBodyWorld::bodies_num() const
{
  return impl_->rigid_bodies.size();
}

int RigidBodyWorld::constraints_num() const
{
  return 0;
}

int RigidBodyWorld::shapes_num() const
{
  return impl_->collision_shapes.size();
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

VArray<RigidBodyID> RigidBodyWorld::body_ids() const {
  return impl_->varray_for_rigid_bodies<RigidBodyID>(
      [&](const btRigidBody &body) { return body.getUserIndex(); });
}

IndexRange RigidBodyWorld::add_rigid_bodies(const Span<const CollisionShape *> shapes,
                                            const VArray<int> &shape_indices,
                                            const VArray<float> &masses,
                                            const VArray<float3> &inertiae)
{
  const int num_add = masses.size();
  BLI_assert(inertiae.is_empty() || inertiae.size() == num_add);
  BLI_assert(shape_indices.size() == num_add);

  impl_->rigid_bodies.append_n_times(nullptr, num_add);
  const IndexRange new_bodies = impl_->rigid_bodies.index_range().take_back(num_add);
  for (const int i : new_bodies) {
    btMotionState *motion_state = impl_->motion_state_pool.alloc();
    BLI_assert(shapes.index_range().contains(shape_indices[i]));
    btCollisionShape *shape = shapes[shape_indices[i]]->impl_->shape;
    const float mass = masses[i];
    const float3 inertia = inertiae.is_empty() ? float3(0.0f) : inertiae[i];

    const btRigidBody::btRigidBodyConstructionInfo construction_info{
        masses[i], motion_state, shape, to_bullet(inertia)};
    btRigidBody *body = impl_->rigid_body_pool.alloc(construction_info);
    impl_->rigid_bodies[i] = body;
    impl_->world->addRigidBody(body);
  }
  return new_bodies;
}

void RigidBodyWorld::remove_rigid_bodies(const IndexMask &mask) {
  mask.foreach_index([&](const int index) {
    btRigidBody *body = impl_->rigid_bodies[index];

    impl_->world->removeRigidBody(body);

    impl_->motion_state_pool.free(static_cast<btDefaultMotionState *>(body->getMotionState()));
    impl_->rigid_body_pool.free(body);
  });

  /* Remove entries. */
  IndexMaskMemory keep_mask_memory;
  IndexMask keep_mask = mask.complement(impl_->rigid_bodies.index_range(), keep_mask_memory);
  Vector<btRigidBody *> new_rigid_bodies(keep_mask.size());
  keep_mask.foreach_index(
      [&](const int index, const int pos) { new_rigid_bodies[pos] = impl_->rigid_bodies[index]; });
  impl_->rigid_bodies = std::move(new_rigid_bodies);
}

void RigidBodyWorld::clear_rigid_bodies()
{
  for (const int index : impl_->rigid_bodies.index_range()){
    btRigidBody *body = impl_->rigid_bodies[index];

    impl_->world->removeRigidBody(body);

    impl_->motion_state_pool.free(static_cast<btDefaultMotionState *>(body->getMotionState()));
    impl_->rigid_body_pool.free(body);
  }

  BLI_assert(impl_->rigid_body_pool.size() == 0);
  impl_->rigid_bodies.clear();
}

//RigidBodyHandle RigidBodyWorld::add_rigid_body(const float mass, const float3 &inertia)
//{
//  btMotionState *motion_state = impl_->motion_state_pool.alloc();
//  btCollisionShape *collision_shape = nullptr;
//  const btRigidBody::btRigidBodyConstructionInfo construction_info{
//      mass, motion_state, collision_shape, to_bullet(inertia)};
//  btRigidBody *body = impl_->rigid_body_pool.alloc(construction_info);
//
//  impl_->world->addRigidBody(body);
//
//  return (RigidBodyHandle)body;
//}
//
//void RigidBodyWorld::remove_rigid_body(RigidBodyHandle handle)
//{
//  btRigidBody *body = reinterpret_cast<btRigidBody *>(handle);
//  impl_->world->removeRigidBody(body);
//}
//
//float RigidBodyWorld::body_mass(RigidBodyHandle handle) const
//{
//  const btRigidBody *body = reinterpret_cast<const btRigidBody *>(handle);
//  return body->getMass();
//}
//
//float3 RigidBodyWorld::body_inertia(RigidBodyHandle handle) const
//{
//  const btRigidBody *body = reinterpret_cast<const btRigidBody *>(handle);
//  return to_blender(body->getLocalInertia());
//}
//
//void RigidBodyWorld::set_body_mass(RigidBodyHandle handle, const float mass, const float3 &inertia)
//{
//  btRigidBody *body = reinterpret_cast<btRigidBody *>(handle);
//  body->setMassProps(mass, to_bullet(inertia));
//}
//
//float RigidBodyWorld::body_friction(RigidBodyHandle handle) const {
//  const btRigidBody *body = reinterpret_cast<const btRigidBody *>(handle);
//  return body->getFriction();
//}
//
//void RigidBodyWorld::set_body_friction(RigidBodyHandle handle, float value)
//{
//  btRigidBody *body = reinterpret_cast<btRigidBody *>(handle);
//  body->setFriction(value);
//}
//
//float RigidBodyWorld::body_restitution(RigidBodyHandle handle) const
//{
//  const btRigidBody *body = reinterpret_cast<const btRigidBody *>(handle);
//  return body->getRestitution();
//}
//
//void RigidBodyWorld::body_set_restitution(RigidBodyHandle handle, float value)
//{
//  btRigidBody *body = reinterpret_cast<btRigidBody *>(handle);
//  body->setRestitution(value);
//}
//
//float RigidBodyWorld::body_linear_damping(RigidBodyHandle handle) const
//{
//  const btRigidBody *body = reinterpret_cast<const btRigidBody *>(handle);
//  return body->getLinearDamping();
//}
//
//void RigidBodyWorld::body_set_linear_damping(RigidBodyHandle handle, float value)
//{
//  btRigidBody *body = reinterpret_cast<btRigidBody *>(handle);
//  body->setDamping(value, body->getAngularDamping());
//}
//
//float RigidBodyWorld::body_angular_damping(RigidBodyHandle handle) const
//{
//  const btRigidBody *body = reinterpret_cast<const btRigidBody *>(handle);
//  return body->getAngularDamping();
//}
//
//void RigidBodyWorld::body_set_angular_damping(RigidBodyHandle handle, float value)
//{
//  btRigidBody *body = reinterpret_cast<btRigidBody *>(handle);
//  body->setDamping(body->getLinearDamping(), value);
//}

CollisionShape::CollisionShape() {}

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
{
  impl_ = new CollisionShapeImpl{new btBoxShape(to_bullet(half_extent))};
}

float3 BoxCollisionShape::half_extent() const {
  return to_blender(static_cast<btBoxShape *>(impl_->shape)->getHalfExtentsWithoutMargin());
}

SphereCollisionShape::SphereCollisionShape(const float radius)
{
  impl_ = new CollisionShapeImpl{new btSphereShape(radius)};
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
