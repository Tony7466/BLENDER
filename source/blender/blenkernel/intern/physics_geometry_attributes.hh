/* SPDX-FileCopyrightText: Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sim
 */

#pragma once

#include "BLI_assert.h"
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
template<typename T> using RigidBodyGetCacheFn = Span<T> (*)(const PhysicsGeometryImpl &impl);
template<typename T> using RigidBodySetFn = void (*)(btRigidBody &body, T value);

template<typename T> using ConstraintGetFn = T (*)(const btTypedConstraint *constraint);
template<typename T> using ConstraintGetCacheFn = Span<T> (*)(const PhysicsGeometryImpl &impl);
template<typename T> using ConstraintSetFn = void (*)(btTypedConstraint *constraint, T value);

template<typename ElemT, RigidBodyGetFn<ElemT> GetFn>
class VArrayImpl_For_PhysicsBodies final : public VArrayImpl<ElemT> {
 private:
  const PhysicsGeometryImpl *impl_;
  Span<ElemT> cache_;
  // XXX causes mystery crashes, investigate ...
  // std::shared_lock<std::shared_mutex> lock_;

 public:
  VArrayImpl_For_PhysicsBodies(const PhysicsGeometryImpl &impl, const Span<ElemT> cache = {})
      : VArrayImpl<ElemT>(impl.rigid_bodies.size()),
        impl_(&impl),
        cache_(cache) /*, lock_(impl_->data_mutex)*/
  {
    // lock_.lock();
  }

  ~VArrayImpl_For_PhysicsBodies()
  {
    // lock_.unlock();
  }

  template<typename OtherElemT, RigidBodyGetFn<OtherElemT> OtherGetFn>
  friend class VArrayImpl_For_PhysicsBodies;

 private:
  ElemT get(const int64_t index) const override
  {
    if constexpr (GetFn) {
      return GetFn(*impl_->rigid_bodies[index]);
    }
    return cache_[index];
  }

  void materialize(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>([&](const int64_t i) {
      if constexpr (GetFn) {
        dst[i] = GetFn(*impl_->rigid_bodies[i]);
      }
      else {
        dst[i] = cache_[i];
      }
    });
  }

  void materialize_to_uninitialized(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>([&](const int64_t i) {
      if constexpr (GetFn) {
        new (dst + i) ElemT(GetFn(*impl_->rigid_bodies[i]));
      }
      else {
        new (dst + i) ElemT(cache_[i]);
      }
    });
  }

  void materialize_compressed(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>([&](const int64_t i, const int64_t pos) {
      if constexpr (GetFn) {
        dst[pos] = GetFn(*impl_->rigid_bodies[i]);
      }
      else {
        dst[pos] = cache_[i];
      }
    });
  }

  void materialize_compressed_to_uninitialized(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>([&](const int64_t i, const int64_t pos) {
      if constexpr (GetFn) {
        new (dst + pos) ElemT(GetFn(*impl_->rigid_bodies[i]));
      }
      else {
        new (dst + pos) ElemT(cache_[i]);
      }
    });
  }
};

template<typename ElemT, RigidBodyGetFn<ElemT> GetFn, RigidBodySetFn<ElemT> SetFn>
class VMutableArrayImpl_For_PhysicsBodies final : public VMutableArrayImpl<ElemT> {
 private:
  const PhysicsGeometryImpl *impl_;
  Span<ElemT> cache_;
  // XXX causes mystery crashes, investigate ...
  // std::unique_lock<std::shared_mutex> lock_;

 public:
  VMutableArrayImpl_For_PhysicsBodies(const PhysicsGeometryImpl &impl,
                                      const Span<ElemT> cache = {})
      : VMutableArrayImpl<ElemT>(impl.rigid_bodies.size()),
        impl_(&impl),
        cache_(cache) /*, lock_(impl_->data_mutex)*/
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
    if constexpr (GetFn) {
      return GetFn(*impl_->rigid_bodies[index]);
    }
    return cache_[index];
  }

  void set(const int64_t index, ElemT value) override
  {
    SetFn(*impl_->rigid_bodies[index], value);
  }

  void materialize(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>([&](const int64_t i) {
      if constexpr (GetFn) {
        dst[i] = GetFn(*impl_->rigid_bodies[i]);
      }
      else {
        dst[i] = cache_[i];
      }
    });
  }

  void materialize_to_uninitialized(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>([&](const int64_t i) {
      if constexpr (GetFn) {
        new (dst + i) ElemT(GetFn(*impl_->rigid_bodies[i]));
      }
      else {
        new (dst + i) ElemT(cache_[i]);
      }
    });
  }

  void materialize_compressed(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>([&](const int64_t i, const int64_t pos) {
      if constexpr (GetFn) {
        dst[pos] = GetFn(*impl_->rigid_bodies[i]);
      }
      else {
        dst[pos] = cache_[i];
      }
    });
  }

  void materialize_compressed_to_uninitialized(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>([&](const int64_t i, const int64_t pos) {
      if constexpr (GetFn) {
        new (dst + pos) ElemT(GetFn(*impl_->rigid_bodies[i]));
      }
      else {
        new (dst + pos) ElemT(cache_[i]);
      }
    });
  }
};

template<typename ElemT, ConstraintGetFn<ElemT> GetFn>
class VArrayImpl_For_PhysicsConstraints final : public VArrayImpl<ElemT> {
 private:
  const PhysicsGeometryImpl *impl_;
  Span<ElemT> cache_;
  // XXX causes mystery crashes, investigate ...
  // std::shared_lock<std::shared_mutex> lock_;

 public:
  VArrayImpl_For_PhysicsConstraints(const PhysicsGeometryImpl &impl, const Span<ElemT> cache)
      : VArrayImpl<ElemT>(impl.constraints.size()),
        impl_(&impl),
        cache_(cache) /*, lock_(impl_->data_mutex)*/
  {
    // lock_.lock();
  }

  ~VArrayImpl_For_PhysicsConstraints()
  {
    // lock_.unlock();
  }

  template<typename OtherElemT, ConstraintGetFn<OtherElemT> OtherGetFn>
  friend class VArrayImpl_For_PhysicsConstraints;

 private:
  ElemT get(const int64_t index) const override
  {
    if constexpr (GetFn) {
      return GetFn(impl_->constraints[index]);
    }
    return cache_[index];
  }

  void materialize(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>([&](const int64_t i) {
      if constexpr (GetFn) {
        dst[i] = GetFn(impl_->constraints[i]);
      }
      else {
        dst[i] = cache_[i];
      }
    });
  }

  void materialize_to_uninitialized(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>([&](const int64_t i) {
      if constexpr (GetFn) {
        new (dst + i) ElemT(GetFn(impl_->constraints[i]));
      }
      else {
        new (dst + i) ElemT(cache_[i]);
      }
    });
  }

  void materialize_compressed(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>([&](const int64_t i, const int64_t pos) {
      if constexpr (GetFn) {
        dst[pos] = GetFn(impl_->constraints[i]);
      }
      else {
        dst[pos] = cache_[i];
      }
    });
  }

  void materialize_compressed_to_uninitialized(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>([&](const int64_t i, const int64_t pos) {
      if constexpr (GetFn) {
        new (dst + pos) ElemT(GetFn(impl_->constraints[i]));
      }
      else {
        new (dst + pos) ElemT(cache_[i]);
      }
    });
  }
};

template<typename ElemT, ConstraintGetFn<ElemT> GetFn, ConstraintSetFn<ElemT> SetFn>
class VMutableArrayImpl_For_PhysicsConstraints final : public VMutableArrayImpl<ElemT> {
 private:
  const PhysicsGeometryImpl *impl_;
  const Span<ElemT> cache_;
  // XXX causes mystery crashes, investigate ...
  // std::unique_lock<std::shared_mutex> lock_;

 public:
  VMutableArrayImpl_For_PhysicsConstraints(const PhysicsGeometryImpl &impl,
                                           const Span<ElemT> cache)
      : VMutableArrayImpl<ElemT>(impl.constraints.size()),
        impl_(&impl),
        cache_(cache) /*, lock_(impl_->data_mutex)*/
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
    if constexpr (GetFn) {
      return GetFn(impl_->constraints[index]);
    }
    return cache_[index];
  }

  void set(const int64_t index, ElemT value) override
  {
    SetFn(impl_->constraints[index], value);
  }

  void materialize(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>([&](const int64_t i) {
      if constexpr (GetFn) {
        dst[i] = GetFn(impl_->constraints[i]);
      }
      else {
        dst[i] = cache_[i];
      }
    });
  }

  void materialize_to_uninitialized(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>([&](const int64_t i) {
      if constexpr (GetFn) {
        new (dst + i) ElemT(GetFn(impl_->constraints[i]));
      }
      else {
        new (dst + i) ElemT(cache_[i]);
      }
    });
  }

  void materialize_compressed(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>([&](const int64_t i, const int64_t pos) {
      if constexpr (GetFn) {
        dst[pos] = GetFn(impl_->constraints[i]);
      }
      else {
        dst[pos] = cache_[i];
      }
    });
  }

  void materialize_compressed_to_uninitialized(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>([&](const int64_t i, const int64_t pos) {
      if constexpr (GetFn) {
        new (dst + pos) ElemT(GetFn(impl_->constraints[i]));
      }
      else {
        new (dst + pos) ElemT(cache_[i]);
      }
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

/**
 * Provider for builtin rigid body attributes.
 */
template<typename T,
         RigidBodyGetFn<T> GetFn,
         RigidBodySetFn<T> SetFn = nullptr,
         RigidBodyGetCacheFn<T> GetCacheFn = nullptr>
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

    BLI_STATIC_ASSERT(GetFn != nullptr || GetCacheFn != nullptr,
                      "A getter function must be defined");
    Span<T> cache = {};
    if constexpr (GetCacheFn) {
      cache = GetCacheFn(physics->impl());
    }

    GVArray varray = VArray<T>::template For<VArrayImpl_For_PhysicsBodies<T, GetFn>>(
        physics->impl(), cache);

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

    Span<T> cache = {};
    if constexpr (GetCacheFn) {
      cache = GetCacheFn(physics->impl());
    }

    GVMutableArray varray =
        VMutableArray<T>::template For<VMutableArrayImpl_For_PhysicsBodies<T, GetFn, SetFn>>(
            physics->impl(), cache);

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
         ConstraintGetCacheFn<T> GetCacheFn = nullptr>
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

    Span<T> cache = {};
    if constexpr (GetCacheFn) {
      cache = GetCacheFn(physics->impl());
    }

    GVArray varray = VArray<T>::template For<VArrayImpl_For_PhysicsConstraints<T, GetFn>>(
        physics->impl(), cache);

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

    Span<T> cache = {};
    if constexpr (GetCacheFn) {
      cache = GetCacheFn(physics->impl());
    }

    GVMutableArray varray =
        VMutableArray<T>::template For<VMutableArrayImpl_For_PhysicsConstraints<T, GetFn, SetFn>>(
            physics->impl(), cache);

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
