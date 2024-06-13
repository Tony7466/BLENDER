/* SPDX-FileCopyrightText: Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sim
 */

#pragma once

#include "BKE_physics_geometry.hh"

#include "BLI_array.hh"

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

  /* Cache for indices in bodies to ensure mapping from body pointers to indices. */
  mutable CacheMutex body_index_cache_;

  btDiscreteDynamicsWorld *world = nullptr;
  btCollisionConfiguration *config = nullptr;
  btCollisionDispatcher *dispatcher = nullptr;
  btBroadphaseInterface *broadphase = nullptr;
  btConstraintSolver *constraint_solver = nullptr;
  btOverlapFilterCallback *overlap_filter = nullptr;

  Array<btRigidBody *> rigid_bodies;
  Array<btMotionState *> motion_states;
  Array<btTypedConstraint *> constraints;

  /* Physics data can be moved while other components still have write access. The physics data is
   * cached for read access, so that data can be moved without requiring locks. */
  struct AttributeCache {
    Array<float3> body_positions;
    Array<math::Quaternion> body_rotations;
    Array<float3> body_velocities;
    Array<float3> body_angular_velocities;
    Array<int> constraint_bodies_1;
    Array<int> constraint_bodies_2;
  };
  AttributeCache attribute_cache;

  PhysicsGeometryImpl();
  ~PhysicsGeometryImpl();

  void delete_self() override;

  void tag_body_topology_changed();

  void ensure_body_indices() const;
};

void move_physics_impl_data(const PhysicsGeometryImpl &from,
                            PhysicsGeometryImpl &to,
                            bool use_world,
                            int rigid_bodies_offset,
                            int constraints_offset);

struct CollisionShapeImpl {
  ~CollisionShapeImpl() = delete;

  void destroy();

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

int activation_state_to_bullet(bke::PhysicsGeometry::BodyActivationState state);
bke::PhysicsGeometry::BodyActivationState activation_state_to_blender(int bt_state);

/* -------------------------------------------------------------------- */
/** \name Virtual Arrays for Physics Geometry
 * \{ */

template<typename T> using RigidBodyGetFn = T (*)(const btRigidBody &body);
template<typename T> using RigidBodySetFn = void (*)(btRigidBody &body, T value);
template<typename T> using ConstraintGetFn = T (*)(const btTypedConstraint &constraint);
template<typename T> using ConstraintSetFn = void (*)(btTypedConstraint &constraint, T value);

template<typename ElemT, RigidBodyGetFn<ElemT> GetFn, RigidBodySetFn<ElemT> SetFn>
class VArrayImpl_For_PhysicsBodies final : public VArrayImpl<ElemT> {
 private:
  const PhysicsGeometryImpl *impl_;
  // XXX causes mystery crashes, investigate ...
  // std::shared_lock<std::shared_mutex> lock_;

 public:
  VArrayImpl_For_PhysicsBodies(const PhysicsGeometryImpl &impl)
      : VArrayImpl<ElemT>(impl.rigid_bodies.size()), impl_(&impl) /*, lock_(impl_->data_mutex)*/
  {
    // lock_.lock();
  }

  ~VArrayImpl_For_PhysicsBodies()
  {
    // lock_.unlock();
  }

  template<typename OtherElemT,
           RigidBodyGetFn<OtherElemT> OtherGetFn,
           RigidBodySetFn<OtherElemT> OtherSetFn>
  friend class VArrayImpl_For_PhysicsBodies;

 private:
  ElemT get(const int64_t index) const override
  {
    return GetFn(*impl_->rigid_bodies[index]);
  }

  void materialize(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t i) { dst[i] = GetFn(*impl_->rigid_bodies[i]); });
  }

  void materialize_to_uninitialized(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t i) { new (dst + i) ElemT(GetFn(*impl_->rigid_bodies[i])); });
  }

  void materialize_compressed(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t i, const int64_t pos) { dst[pos] = GetFn(*impl_->rigid_bodies[i]); });
  }

  void materialize_compressed_to_uninitialized(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>([&](const int64_t i, const int64_t pos) {
      new (dst + pos) ElemT(GetFn(*impl_->rigid_bodies[i]));
    });
  }
};

/* Placeholder that ignores get/set. */
template<typename ElemT, RigidBodyGetFn<ElemT> GetFn, RigidBodySetFn<ElemT> SetFn>
class VMutableArrayImpl_For_PhysicsBodies final : public VMutableArrayImpl<ElemT> {
 private:
  const PhysicsGeometryImpl *impl_;
  // XXX causes mystery crashes, investigate ...
  // std::unique_lock<std::shared_mutex> lock_;

 public:
  VMutableArrayImpl_For_PhysicsBodies(const PhysicsGeometryImpl &impl)
      : VMutableArrayImpl<ElemT>(impl.rigid_bodies.size()),
        impl_(&impl) /*, lock_(impl_->data_mutex)*/
  {
    // lock_.lock();
  }

  ~VMutableArrayImpl_For_PhysicsBodies()
  {
    // lock_.unlock();
  }

  template<typename OtherElemT,
           RigidBodyGetFn<OtherElemT> OtherGetFn,
           RigidBodySetFn<OtherElemT> OtherSetFn>
  friend class VMutableArrayImpl_For_PhysicsBodies;

 private:
  ElemT get(const int64_t index) const override
  {
    return GetFn(*impl_->rigid_bodies[index]);
  }

  void set(const int64_t index, ElemT value) override
  {
    SetFn(*impl_->rigid_bodies[index], value);
  }

  void materialize(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t i) { dst[i] = GetFn(*impl_->rigid_bodies[i]); });
  }

  void materialize_to_uninitialized(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t i) { new (dst + i) ElemT(GetFn(*impl_->rigid_bodies[i])); });
  }

  void materialize_compressed(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t i, const int64_t pos) { dst[pos] = GetFn(*impl_->rigid_bodies[i]); });
  }

  void materialize_compressed_to_uninitialized(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>([&](const int64_t i, const int64_t pos) {
      new (dst + pos) ElemT(GetFn(*impl_->rigid_bodies[i]));
    });
  }
};
template<typename ElemT, ConstraintGetFn<ElemT> GetFn, ConstraintSetFn<ElemT> SetFn>
class VArrayImpl_For_PhysicsConstraints final : public VArrayImpl<ElemT> {
 private:
  const PhysicsGeometryImpl *impl_;
  // XXX causes mystery crashes, investigate ...
  // std::shared_lock<std::shared_mutex> lock_;

 public:
  VArrayImpl_For_PhysicsConstraints(const PhysicsGeometryImpl &impl)
      : VArrayImpl<ElemT>(impl.constraints.size()), impl_(&impl) /*, lock_(impl_->data_mutex)*/
  {
    // lock_.lock();
  }

  ~VArrayImpl_For_PhysicsConstraints()
  {
    // lock_.unlock();
  }

  template<typename OtherElemT,
           ConstraintGetFn<OtherElemT> OtherGetFn,
           ConstraintSetFn<OtherElemT> OtherSetFn>
  friend class VArrayImpl_For_PhysicsConstraints;

 private:
  ElemT get(const int64_t index) const override
  {
    return GetFn(*impl_->constraints[index]);
  }

  void materialize(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t i) { dst[i] = GetFn(*impl_->constraints[i]); });
  }

  void materialize_to_uninitialized(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t i) { new (dst + i) ElemT(GetFn(*impl_->constraints[i])); });
  }

  void materialize_compressed(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t i, const int64_t pos) { dst[pos] = GetFn(*impl_->constraints[i]); });
  }

  void materialize_compressed_to_uninitialized(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>([&](const int64_t i, const int64_t pos) {
      new (dst + pos) ElemT(GetFn(*impl_->constraints[i]));
    });
  }
};

/* Placeholder that ignores get/set. */
template<typename ElemT, ConstraintGetFn<ElemT> GetFn, ConstraintSetFn<ElemT> SetFn>
class VMutableArrayImpl_For_PhysicsConstraints final : public VMutableArrayImpl<ElemT> {
 private:
  const PhysicsGeometryImpl *impl_;
  // XXX causes mystery crashes, investigate ...
  // std::unique_lock<std::shared_mutex> lock_;

 public:
  VMutableArrayImpl_For_PhysicsConstraints(const PhysicsGeometryImpl &impl)
      : VMutableArrayImpl<ElemT>(impl.constraints.size()),
        impl_(&impl) /*, lock_(impl_->data_mutex)*/
  {
    // lock_.lock();
  }

  ~VMutableArrayImpl_For_PhysicsConstraints()
  {
    // lock_.unlock();
  }

  template<typename OtherElemT,
           ConstraintGetFn<OtherElemT> OtherGetFn,
           ConstraintSetFn<OtherElemT> OtherSetFn>
  friend class VMutableArrayImpl_For_PhysicsConstraints;

 private:
  ElemT get(const int64_t index) const override
  {
    return GetFn(*impl_->constraints[index]);
  }

  void set(const int64_t index, ElemT value) override
  {
    SetFn(*impl_->constraints[index], value);
  }

  void materialize(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t i) { dst[i] = GetFn(*impl_->constraints[i]); });
  }

  void materialize_to_uninitialized(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t i) { new (dst + i) ElemT(GetFn(*impl_->constraints[i])); });
  }

  void materialize_compressed(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t i, const int64_t pos) { dst[pos] = GetFn(*impl_->constraints[i]); });
  }

  void materialize_compressed_to_uninitialized(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>([&](const int64_t i, const int64_t pos) {
      new (dst + pos) ElemT(GetFn(*impl_->constraints[i]));
    });
  }
};

template<typename ElemT> class VArrayImpl_For_PhysicsStub final : public VMutableArrayImpl<ElemT> {
 private:
  ElemT value_;

 public:
  VArrayImpl_For_PhysicsStub(const ElemT value, const size_t size)
      : VMutableArrayImpl<ElemT>(size), value_(value)
  {
  }

  template<typename OtherElemT> friend class VArrayImpl_For_PhysicsStub;

 private:
  ElemT get(const int64_t /*index*/) const override
  {
    return value_;
  }

  void set(const int64_t /*index*/, ElemT /*value*/) override {}

  void materialize(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>([&](const int64_t i) { dst[i] = value_; });
  }

  void materialize_to_uninitialized(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>([&](const int64_t i) { new (dst + i) ElemT(value_); });
  }

  void materialize_compressed(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t /*i*/, const int64_t pos) { dst[pos] = value_; });
  }

  void materialize_compressed_to_uninitialized(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t /*i*/, const int64_t pos) { new (dst + pos) ElemT(value_); });
  }
};

template<typename T, RigidBodyGetFn<T> GetFn>
VArray<T> VArray_For_PhysicsBodies(const PhysicsGeometry *physics, const Span<T> cache)
{
  if (physics->impl().is_cached) {
    return VArray<T>::ForSpan(cache);
  }
  return VArray<T>::template For<VArrayImpl_For_PhysicsBodies<T, GetFn, nullptr>>(physics->impl());
}

template<typename T, RigidBodyGetFn<T> GetFn>
VArray<T> VArray_For_PhysicsBodies(const PhysicsGeometry *physics, const T value)
{
  if (physics->impl().is_cached) {
    const int bodies_num = physics->impl().attribute_cache.body_positions.size();
    return VArray<T>::ForSingle(value, bodies_num);
  }
  return VArray<T>::template For<VArrayImpl_For_PhysicsBodies<T, GetFn, nullptr>>(physics->impl());
}

template<typename T, RigidBodyGetFn<T> GetFn, RigidBodySetFn<T> SetFn>
VMutableArray<T> VMutableArray_For_PhysicsBodies(const PhysicsGeometry *physics,
                                                 const MutableSpan<T> cache)
{
  if (physics->impl().is_cached) {
    return VMutableArray<T>::ForSpan(cache);
  }
  return VMutableArray<T>::template For<VMutableArrayImpl_For_PhysicsBodies<T, GetFn, SetFn>>(
      physics->impl());
}

template<typename T, RigidBodyGetFn<T> GetFn, RigidBodySetFn<T> SetFn>
VMutableArray<T> VMutableArray_For_PhysicsBodies(const PhysicsGeometry *physics, const T value)
{
  if (physics->impl().is_cached) {
    const int bodies_num = physics->impl().attribute_cache.body_positions.size();
    return VMutableArray<T>::template For<VArrayImpl_For_PhysicsStub<T>>(value, bodies_num);
  }
  return VMutableArray<T>::template For<VMutableArrayImpl_For_PhysicsBodies<T, GetFn, SetFn>>(
      physics->impl());
}

template<typename T, ConstraintGetFn<T> GetFn>
VArray<T> VArray_For_PhysicsConstraints(const PhysicsGeometry *physics, const Span<T> cache)
{
  if (physics->impl().is_cached) {
    return VArray<T>::ForSpan(cache);
  }
  return VArray<T>::template For<VArrayImpl_For_PhysicsConstraints<T, GetFn, nullptr>>(
      physics->impl());
}

template<typename T, ConstraintGetFn<T> GetFn>
VArray<T> VArray_For_PhysicsConstraints(const PhysicsGeometry *physics, const T value)
{
  if (physics->impl().is_cached) {
    const int constraints_num = physics->impl().attribute_cache.constraint_bodies_1.size();
    return VArray<T>::ForSingle(value, constraints_num);
  }
  return VArray<T>::template For<VArrayImpl_For_PhysicsConstraints<T, GetFn, nullptr>>(
      physics->impl());
}

template<typename T, ConstraintGetFn<T> GetFn, ConstraintSetFn<T> SetFn>
VMutableArray<T> VMutableArray_For_PhysicsConstraints(const PhysicsGeometry *physics,
                                                      const MutableSpan<T> cache)
{
  if (physics->impl().is_cached) {
    return VMutableArray<T>::ForSpan(cache);
  }
  return VMutableArray<T>::template For<VMutableArrayImpl_For_PhysicsConstraints<T, GetFn, SetFn>>(
      physics->impl());
}

template<typename T, ConstraintGetFn<T> GetFn, ConstraintSetFn<T> SetFn>
VMutableArray<T> VMutableArray_For_PhysicsConstraints(const PhysicsGeometry *physics,
                                                      const T value)
{
  if (physics->impl().is_cached) {
    const int constraints_num = physics->impl().attribute_cache.constraint_bodies_1.size();
    return VMutableArray<T>::template For<VArrayImpl_For_PhysicsStub<T>>(value, constraints_num);
  }
  return VMutableArray<T>::template For<VMutableArrayImpl_For_PhysicsConstraints<T, GetFn, SetFn>>(
      physics->impl());
}

/** \} */

}  // namespace blender::bke
