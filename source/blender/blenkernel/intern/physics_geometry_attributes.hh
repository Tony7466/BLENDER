/* SPDX-FileCopyrightText: Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sim
 */

#pragma once

#include "BLI_virtual_array.hh"

#include "BKE_attribute.hh"

#include "attribute_access_intern.hh"
#include "physics_geometry_impl.hh"

#include <shared_mutex>

class btRigidBody;
class btMotionState;
class btCollisionShape;
class btTypedConstraint;

namespace blender::bke {

/* -------------------------------------------------------------------- */
/** \name Virtual Arrays for Physics Geometry
 * \{ */

template<typename T> using RigidBodyGetFn = T (*)(const btRigidBody &body);
template<typename T> using RigidBodySetFn = void (*)(btRigidBody &body, T value);
template<typename T> using ConstraintGetFn = T (*)(const btTypedConstraint *constraint);
template<typename T> using ConstraintSetFn = void (*)(btTypedConstraint *constraint, T value);

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
    return GetFn(impl_->constraints[index]);
  }

  void materialize(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t i) { dst[i] = GetFn(impl_->constraints[i]); });
  }

  void materialize_to_uninitialized(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t i) { new (dst + i) ElemT(GetFn(impl_->constraints[i])); });
  }

  void materialize_compressed(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t i, const int64_t pos) { dst[pos] = GetFn(impl_->constraints[i]); });
  }

  void materialize_compressed_to_uninitialized(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>([&](const int64_t i, const int64_t pos) {
      new (dst + pos) ElemT(GetFn(impl_->constraints[i]));
    });
  }
};

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
    return GetFn(impl_->constraints[index]);
  }

  void set(const int64_t index, ElemT value) override
  {
    SetFn(impl_->constraints[index], value);
  }

  void materialize(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t i) { dst[i] = GetFn(impl_->constraints[i]); });
  }

  void materialize_to_uninitialized(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t i) { new (dst + i) ElemT(GetFn(impl_->constraints[i])); });
  }

  void materialize_compressed(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t i, const int64_t pos) { dst[pos] = GetFn(impl_->constraints[i]); });
  }

  void materialize_compressed_to_uninitialized(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>([&](const int64_t i, const int64_t pos) {
      new (dst + pos) ElemT(GetFn(impl_->constraints[i]));
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

/* -------------------------------------------------------------------- */
/** \name Attribute Accessors for Physics Geometry
 * \{ */

/**
 * Utility to group together multiple functions that are used to access custom data on geometry
 * components in a generic way.
 */
struct PhysicsAccessInfo {
  using PhysicsGetter = PhysicsGeometry *(*)(void *owner);
  using ConstPhysicsGetter = const PhysicsGeometry *(*)(const void *owner);

  PhysicsGetter get_physics;
  ConstPhysicsGetter get_const_physics;
};

template<typename T> using RigidBodyGetCacheFn = Span<T> (*)(const PhysicsGeometryImpl &impl);
template<typename T>
using RigidBodyGetMutableCacheFn = MutableSpan<T> (*)(PhysicsGeometryImpl &impl);
template<typename T> using ConstraintGetCacheFn = Span<T> (*)(const PhysicsGeometryImpl &impl);
template<typename T>
using ConstraintGetMutableCacheFn = MutableSpan<T> (*)(PhysicsGeometryImpl &impl);

/**
 * Provider for builtin rigid body attributes.
 */
template<typename T,
         RigidBodyGetFn<T> GetFn,
         RigidBodySetFn<T> SetFn = nullptr,
         RigidBodyGetCacheFn<T> GetCacheFn = nullptr,
         RigidBodyGetMutableCacheFn<T> GetMutableCacheFn = nullptr>
class BuiltinRigidBodyAttributeProvider final : public bke::BuiltinAttributeProvider {
  using UpdateOnChange = void (*)(void *owner);
  const PhysicsAccessInfo physics_access_;
  const UpdateOnChange update_on_change_;

 public:
  BuiltinRigidBodyAttributeProvider(std::string attribute_name,
                                    const AttrDomain domain,
                                    const DeletableEnum deletable,
                                    const PhysicsAccessInfo physics_access,
                                    const UpdateOnChange update_on_change,
                                    const AttributeValidator validator = {})
      : BuiltinAttributeProvider(std::move(attribute_name),
                                 domain,
                                 cpp_type_to_custom_data_type(CPPType::get<T>()),
                                 deletable,
                                 validator),
        physics_access_(physics_access),
        update_on_change_(update_on_change)
  {
  }

  GAttributeReader try_get_for_read(const void *owner) const final
  {
    const PhysicsGeometry *physics = physics_access_.get_const_physics(owner);
    if (physics == nullptr) {
      return {};
    }

    GVArray varray;
    if constexpr (GetCacheFn == nullptr) {
      varray = VArray_For_PhysicsBodies<T, GetFn>(physics, T());
    }
    else {
      varray = VArray_For_PhysicsBodies<T, GetFn>(physics, GetCacheFn(physics->impl()));
    }

    return {std::move(varray), domain_, nullptr};
  }

  GAttributeWriter try_get_for_write(void *owner) const final
  {
    if constexpr (SetFn == nullptr) {
      return {};
    }
    PhysicsGeometry *physics = physics_access_.get_physics(owner);
    if (physics == nullptr) {
      return {};
    }

    // GVMutableArray varray = VMutableArray_For_PhysicsBodies<T, GetFn, SetFn>(physics);
    PhysicsGeometryImpl &impl = physics->impl_for_write();

    GVMutableArray varray;
    if constexpr (GetMutableCacheFn == nullptr) {
      varray = VMutableArray_For_PhysicsBodies<T, GetFn, SetFn>(physics, T());
    }
    else {
      varray = VMutableArray_For_PhysicsBodies<T, GetFn, SetFn>(physics, GetMutableCacheFn(impl));
    }

    std::function<void()> tag_modified_fn;
    if (update_on_change_ != nullptr) {
      tag_modified_fn = [owner, update = update_on_change_]() { update(owner); };
    }

    return {std::move(varray), domain_, std::move(tag_modified_fn)};
  }

  bool try_delete(void * /*owner*/) const final
  {
    return false;
  }

  bool try_create(void * /*owner*/, const AttributeInit & /*initializer*/) const final
  {
    return false;
  }

  bool exists(const void * /*owner*/) const final
  {
    return true;
  }
};

/**
 * Provider for builtin constraint attributes.
 */
template<typename T,
         ConstraintGetFn<T> GetFn,
         ConstraintSetFn<T> SetFn = nullptr,
         ConstraintGetCacheFn<T> GetCacheFn = nullptr,
         ConstraintGetMutableCacheFn<T> GetMutableCacheFn = nullptr>
class BuiltinConstraintAttributeProvider final : public bke::BuiltinAttributeProvider {
  using UpdateOnChange = void (*)(void *owner);
  using EnsureOnAccess = void (*)(const void *owner);
  const PhysicsAccessInfo physics_access_;
  const UpdateOnChange update_on_change_;
  const EnsureOnAccess ensure_on_access_;

 public:
  BuiltinConstraintAttributeProvider(std::string attribute_name,
                                     const AttrDomain domain,
                                     const DeletableEnum deletable,
                                     const PhysicsAccessInfo physics_access,
                                     const UpdateOnChange update_on_change,
                                     const AttributeValidator validator = {},
                                     const EnsureOnAccess ensure_on_access = nullptr)
      : BuiltinAttributeProvider(std::move(attribute_name),
                                 domain,
                                 cpp_type_to_custom_data_type(CPPType::get<T>()),
                                 deletable,
                                 validator),
        physics_access_(physics_access),
        update_on_change_(update_on_change),
        ensure_on_access_(ensure_on_access)
  {
  }

  GAttributeReader try_get_for_read(const void *owner) const final
  {
    const PhysicsGeometry *physics = physics_access_.get_const_physics(owner);
    if (physics == nullptr) {
      return {};
    }

    if (ensure_on_access_) {
      ensure_on_access_(owner);
    }

    GVArray varray;
    if constexpr (GetCacheFn == nullptr) {
      varray = VArray_For_PhysicsConstraints<T, GetFn>(physics, T());
    }
    else {
      varray = VArray_For_PhysicsConstraints<T, GetFn>(physics, GetCacheFn(physics->impl()));
    }

    return {std::move(varray), domain_, nullptr};
  }

  GAttributeWriter try_get_for_write(void *owner) const final
  {
    if constexpr (SetFn == nullptr) {
      return {};
    }
    PhysicsGeometry *physics = physics_access_.get_physics(owner);
    if (physics == nullptr) {
      return {};
    }

    if (ensure_on_access_) {
      ensure_on_access_(owner);
    }

    // GVMutableArray varray = VMutableArray_For_PhysicsBodies<T, GetFn, SetFn>(physics);
    PhysicsGeometryImpl &impl = physics->impl_for_write();

    GVMutableArray varray;
    if constexpr (GetMutableCacheFn == nullptr) {
      varray = VMutableArray_For_PhysicsConstraints<T, GetFn, SetFn>(physics, T());
    }
    else {
      varray = VMutableArray_For_PhysicsConstraints<T, GetFn, SetFn>(physics,
                                                                     GetMutableCacheFn(impl));
    }

    std::function<void()> tag_modified_fn;
    if (update_on_change_ != nullptr) {
      tag_modified_fn = [owner, update = update_on_change_]() { update(owner); };
    }

    return {std::move(varray), domain_, std::move(tag_modified_fn)};
  }

  bool try_delete(void * /*owner*/) const final
  {
    return false;
  }

  bool try_create(void * /*owner*/, const AttributeInit & /*initializer*/) const final
  {
    return false;
  }

  bool exists(const void * /*owner*/) const final
  {
    return true;
  }
};

/** \} */

}  // namespace blender::bke
