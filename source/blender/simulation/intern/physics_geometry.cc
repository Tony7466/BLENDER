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

#include "SIM_physics_geometry.hh"
#include "SIM_rigid_body.hh"

#ifdef WITH_BULLET
#  include "BulletDynamics/Dynamics/btRigidBody.h"
#  include "btBulletDynamicsCommon.h"
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

bool PhysicsGeometry::has_world() const
{
  return world_ != nullptr;
}

RigidBodyWorld *PhysicsGeometry::world() const
{
  return world_;
}

RigidBodyWorld *PhysicsGeometry::ensure_world()
{
  if (world_ == nullptr) {
    world_ = new RigidBodyWorld();
  }
  return world_;
}

Span<const RigidBody *> PhysicsGeometry::rigid_bodies() const
{
  return rigid_bodies_;
}

Span<RigidBody *> PhysicsGeometry::rigid_bodies_for_write()
{
  return rigid_bodies_;
}

template<typename T, typename GetFn>
static VArray<T> varray_for_rigid_bodies(const PhysicsGeometry &physics, GetFn get_fn)
{
  return VArray<T>::ForFunc(physics.rigid_bodies().size(), [&](const int64_t index) {
    return get_fn(*physics.rigid_bodies()[index]);
  });
}

VArray<RigidBodyID> PhysicsGeometry::body_ids() const
{
  return varray_for_rigid_bodies<RigidBodyID>(
      *this, [&](const btRigidBody &body) { return body.getUserIndex(); });
}

IndexRange PhysicsGeometry::add_rigid_bodies(const Span<const CollisionShape *> shapes,
                                            const VArray<int> &shape_indices,
                                            const VArray<float> &masses,
                                            const VArray<float3> &inertiae)
{
  const int num_add = masses.size();
  BLI_assert(inertiae.is_empty() || inertiae.size() == num_add);
  BLI_assert(shape_indices.size() == num_add);

  //rigid_bodies_.append_n_times(nullptr, num_add);
  Array<RigidBody *> new_rigid_bodies(rigid_bodies_.size() + num_add);
  array_utils::copy(rigid_bodies_.as_span(), new_rigid_bodies.as_mutable_span());
  const IndexRange new_bodies_range = new_rigid_bodies.index_range().take_back(num_add);
  for (const int i : new_bodies_range) {
    btMotionState *motion_state = new btDefaultMotionState();
    BLI_assert(shapes.index_range().contains(shape_indices[i]));
    btCollisionShape *shape = shapes[shape_indices[i]]->impl_->shape;
    const float mass = masses[i];
    const float3 inertia = inertiae.is_empty() ? float3(0.0f) : inertiae[i];

    const btRigidBody::btRigidBodyConstructionInfo construction_info{
        masses[i], motion_state, shape, to_bullet(inertia)};
    btRigidBody *body = new btRigidBody(construction_info);
    rigid_bodies_[i] = body;
    impl_->world->addRigidBody(body);
  }
  return new_bodies_range;
}

void PhysicsGeometry::remove_rigid_bodies(const IndexMask &mask)
{
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

void PhysicsGeometry::clear_rigid_bodies()
{
  for (const int index : impl_->rigid_bodies.index_range()) {
    btRigidBody *body = impl_->rigid_bodies[index];

    impl_->world->removeRigidBody(body);

    impl_->motion_state_pool.free(static_cast<btDefaultMotionState *>(body->getMotionState()));
    impl_->rigid_body_pool.free(body);
  }

  BLI_assert(impl_->rigid_body_pool.size() == 0);
  impl_->rigid_bodies.clear();
}

#else

#endif

}  // namespace blender::simulation
