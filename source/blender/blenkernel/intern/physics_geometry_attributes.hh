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
#include "BKE_customdata.hh"

#include "DNA_customdata_types.h"
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

static Span<PhysicsGeometry::BodyAttribute> all_body_attributes()
{
  using BodyAttribute = PhysicsGeometry::BodyAttribute;
  static Array<BodyAttribute> attributes = {
      BodyAttribute::id,
      BodyAttribute::collision_shape,
      BodyAttribute::is_static,
      BodyAttribute::is_kinematic,
      BodyAttribute::mass,
      BodyAttribute::inertia,
      BodyAttribute::center_of_mass,
      BodyAttribute::position,
      BodyAttribute::rotation,
      BodyAttribute::velocity,
      BodyAttribute::angular_velocity,
      BodyAttribute::activation_state,
      BodyAttribute::friction,
      BodyAttribute::rolling_friction,
      BodyAttribute::spinning_friction,
      BodyAttribute::restitution,
      BodyAttribute::linear_damping,
      BodyAttribute::angular_damping,
      BodyAttribute::linear_sleeping_threshold,
      BodyAttribute::angular_sleeping_threshold,
      BodyAttribute::total_force,
      BodyAttribute::total_torque,
  };
  return attributes;
}

static Span<PhysicsGeometry::ConstraintAttribute> all_constraint_attributes()
{
  using ConstraintAttribute = PhysicsGeometry::ConstraintAttribute;
  static Array<ConstraintAttribute> attributes = {
      ConstraintAttribute::constraint_type,
      ConstraintAttribute::constraint_body1,
      ConstraintAttribute::constraint_body2,
      ConstraintAttribute::constraint_enabled,
      ConstraintAttribute::constraint_frame1,
      ConstraintAttribute::constraint_frame2,
      ConstraintAttribute::applied_impulse,
      ConstraintAttribute::applied_force1,
      ConstraintAttribute::applied_force2,
      ConstraintAttribute::applied_torque1,
      ConstraintAttribute::applied_torque2,
      ConstraintAttribute::breaking_impulse_threshold,
      ConstraintAttribute::disable_collision,
  };
  return attributes;
}

static Span<std::string> all_body_attribute_names()
{
  static Array<std::string> all_names = []() {
    const Span<PhysicsGeometry::BodyAttribute> all_attributes = all_body_attributes();
    Array<std::string> all_names(all_attributes.size());
    for (const int i : all_names.index_range()) {
      all_names[i] = PhysicsGeometry::body_attribute_name(all_attributes[i]);
    }
    return all_names;
  }();
  return all_names;
}

static Span<std::string> all_constraint_attribute_names()
{
  static Array<std::string> all_names = []() {
    const Span<PhysicsGeometry::ConstraintAttribute> all_attributes = all_constraint_attributes();
    Array<std::string> all_names(all_attributes.size());
    for (const int i : all_names.index_range()) {
      all_names[i] = PhysicsGeometry::constraint_attribute_name(all_attributes[i]);
    }
    return all_names;
  }();
  return all_names;
}

static StringRef physics_attribute_name(PhysicsGeometry::BodyAttribute attribute)
{
  using BodyAttribute = PhysicsGeometry::BodyAttribute;

  switch (attribute) {
    case BodyAttribute::id:
      return "id";
    case BodyAttribute::collision_shape:
      return "collision_shape";
    case BodyAttribute::is_static:
      return "is_static";
    case BodyAttribute::is_kinematic:
      return "is_kinematic";
    case BodyAttribute::mass:
      return "mass";
    case BodyAttribute::inertia:
      return "inertia";
    case BodyAttribute::center_of_mass:
      return "center_of_mass";
    case BodyAttribute::position:
      return "position";
    case BodyAttribute::rotation:
      return "rotation";
    case BodyAttribute::velocity:
      return "velocity";
    case BodyAttribute::angular_velocity:
      return "angular_velocity";
    case BodyAttribute::activation_state:
      return "activation_state";
    case BodyAttribute::friction:
      return "friction";
    case BodyAttribute::rolling_friction:
      return "rolling_friction";
    case BodyAttribute::spinning_friction:
      return "spinning_friction";
    case BodyAttribute::restitution:
      return "restitution";
    case BodyAttribute::linear_damping:
      return "linear_damping";
    case BodyAttribute::angular_damping:
      return "angular_damping";
    case BodyAttribute::linear_sleeping_threshold:
      return "linear_sleeping_threshold";
    case BodyAttribute::angular_sleeping_threshold:
      return "angular_sleeping_threshold";
    case BodyAttribute::total_force:
      return "total_force";
    case BodyAttribute::total_torque:
      return "total_torque";
  }
  BLI_assert_unreachable();
  return "";
}

static StringRef physics_attribute_name(PhysicsGeometry::ConstraintAttribute attribute)
{
  using ConstraintAttribute = PhysicsGeometry::ConstraintAttribute;

  switch (attribute) {
    case ConstraintAttribute::constraint_type:
      return "constraint_type";
    case ConstraintAttribute::constraint_body1:
      return "constraint_body1";
    case ConstraintAttribute::constraint_body2:
      return "constraint_body2";
    case ConstraintAttribute::constraint_enabled:
      return "constraint_enabled";
    case ConstraintAttribute::constraint_frame1:
      return "constraint_frame1";
    case ConstraintAttribute::constraint_frame2:
      return "constraint_frame2";
    case ConstraintAttribute::applied_impulse:
      return "applied_impulse";
    case ConstraintAttribute::applied_force1:
      return "applied_force1";
    case ConstraintAttribute::applied_force2:
      return "applied_force2";
    case ConstraintAttribute::applied_torque1:
      return "applied_torque1";
    case ConstraintAttribute::applied_torque2:
      return "applied_torque2";
    case ConstraintAttribute::breaking_impulse_threshold:
      return "breaking_impulse_threshold";
    case ConstraintAttribute::disable_collision:
      return "disable_collision";
  }
  BLI_assert_unreachable();
  return "";
}

static const CPPType &physics_attribute_type(PhysicsGeometry::BodyAttribute attribute)
{
  using BodyAttribute = PhysicsGeometry::BodyAttribute;

  switch (attribute) {
    case BodyAttribute::id:
      return CPPType::get<int>();
    case BodyAttribute::collision_shape:
      return CPPType::get<int>();
    case BodyAttribute::is_static:
      return CPPType::get<bool>();
    case BodyAttribute::is_kinematic:
      return CPPType::get<bool>();
    case BodyAttribute::mass:
      return CPPType::get<float>();
    case BodyAttribute::inertia:
      return CPPType::get<float3>();
    case BodyAttribute::center_of_mass:
      return CPPType::get<float4x4>();
    case BodyAttribute::position:
      return CPPType::get<float3>();
    case BodyAttribute::rotation:
      return CPPType::get<math::Quaternion>();
    case BodyAttribute::velocity:
      return CPPType::get<float3>();
    case BodyAttribute::angular_velocity:
      return CPPType::get<float3>();
    case BodyAttribute::activation_state:
      return CPPType::get<int>();
    case BodyAttribute::friction:
      return CPPType::get<float>();
    case BodyAttribute::rolling_friction:
      return CPPType::get<float>();
    case BodyAttribute::spinning_friction:
      return CPPType::get<float>();
    case BodyAttribute::restitution:
      return CPPType::get<float>();
    case BodyAttribute::linear_damping:
      return CPPType::get<float>();
    case BodyAttribute::angular_damping:
      return CPPType::get<float>();
    case BodyAttribute::linear_sleeping_threshold:
      return CPPType::get<float>();
    case BodyAttribute::angular_sleeping_threshold:
      return CPPType::get<float>();
    case BodyAttribute::total_force:
      return CPPType::get<float3>();
    case BodyAttribute::total_torque:
      return CPPType::get<float3>();
  }
  BLI_assert_unreachable();
  return CPPType::get<int>();
}

static const CPPType &physics_attribute_type(PhysicsGeometry::ConstraintAttribute attribute)
{
  using ConstraintAttribute = PhysicsGeometry::ConstraintAttribute;

  switch (attribute) {
    case ConstraintAttribute::constraint_type:
      return CPPType::get<int>();
    case ConstraintAttribute::constraint_body1:
      return CPPType::get<int>();
    case ConstraintAttribute::constraint_body2:
      return CPPType::get<int>();
    case ConstraintAttribute::constraint_enabled:
      return CPPType::get<bool>();
    case ConstraintAttribute::constraint_frame1:
      return CPPType::get<float4x4>();
    case ConstraintAttribute::constraint_frame2:
      return CPPType::get<float4x4>();
    case ConstraintAttribute::applied_impulse:
      return CPPType::get<float>();
    case ConstraintAttribute::applied_force1:
      return CPPType::get<float3>();
    case ConstraintAttribute::applied_force2:
      return CPPType::get<float3>();
    case ConstraintAttribute::applied_torque1:
      return CPPType::get<float3>();
    case ConstraintAttribute::applied_torque2:
      return CPPType::get<float3>();
    case ConstraintAttribute::breaking_impulse_threshold:
      return CPPType::get<float>();
    case ConstraintAttribute::disable_collision:
      return CPPType::get<bool>();
  }
  BLI_assert_unreachable();
  return CPPType::get<int>();
}

static const void *physics_attribute_default_value(PhysicsGeometry::BodyAttribute attribute)
{
  using BodyAttribute = PhysicsGeometry::BodyAttribute;

  const CPPType &type = physics_attribute_type(attribute);
  switch (attribute) {
    case BodyAttribute::collision_shape: {
      static const int default_value = -1;
      return &default_value;
    }
    case BodyAttribute::center_of_mass: {
      static const float4x4 default_value = float4x4::identity();
      return &default_value;
    }
    case BodyAttribute::id:
    case BodyAttribute::is_static:
    case BodyAttribute::is_kinematic:
    case BodyAttribute::mass:
    case BodyAttribute::inertia:
    case BodyAttribute::position:
    case BodyAttribute::rotation:
    case BodyAttribute::velocity:
    case BodyAttribute::angular_velocity:
    case BodyAttribute::activation_state:
    case BodyAttribute::friction:
    case BodyAttribute::rolling_friction:
    case BodyAttribute::spinning_friction:
    case BodyAttribute::restitution:
    case BodyAttribute::linear_damping:
    case BodyAttribute::angular_damping:
    case BodyAttribute::linear_sleeping_threshold:
    case BodyAttribute::angular_sleeping_threshold:
    case BodyAttribute::total_force:
    case BodyAttribute::total_torque:
      return type.default_value();
  }
  BLI_assert_unreachable();
  return nullptr;
}

static const void *physics_attribute_default_value(PhysicsGeometry::ConstraintAttribute attribute)
{
  using ConstraintAttribute = PhysicsGeometry::ConstraintAttribute;

  const CPPType &type = physics_attribute_type(attribute);
  switch (attribute) {
    case ConstraintAttribute::constraint_type:
    case ConstraintAttribute::constraint_body1:
    case ConstraintAttribute::constraint_body2: {
      static const int default_value = -1;
      return &default_value;
    }
    case ConstraintAttribute::constraint_frame1:
    case ConstraintAttribute::constraint_frame2: {
      static const float4x4 default_value = float4x4::identity();
      return &default_value;
    }
    case ConstraintAttribute::constraint_enabled:
    case ConstraintAttribute::applied_impulse:
    case ConstraintAttribute::applied_force1:
    case ConstraintAttribute::applied_force2:
    case ConstraintAttribute::applied_torque1:
    case ConstraintAttribute::applied_torque2:
    case ConstraintAttribute::breaking_impulse_threshold:
    case ConstraintAttribute::disable_collision:
      return type.default_value();
  }
  BLI_assert_unreachable();
  return nullptr;
}

static void physics_attribute_finish(PhysicsGeometryImpl &impl,
                                     const PhysicsGeometry::BodyAttribute attribute)
{
  using BodyAttribute = PhysicsGeometry::BodyAttribute;

  switch (attribute) {
    case BodyAttribute::collision_shape:
      impl.tag_body_collision_shape_changed();
      break;

    case BodyAttribute::center_of_mass:
    case BodyAttribute::id:
    case BodyAttribute::is_static:
    case BodyAttribute::is_kinematic:
    case BodyAttribute::mass:
    case BodyAttribute::inertia:
    case BodyAttribute::position:
    case BodyAttribute::rotation:
    case BodyAttribute::velocity:
    case BodyAttribute::angular_velocity:
    case BodyAttribute::activation_state:
    case BodyAttribute::friction:
    case BodyAttribute::rolling_friction:
    case BodyAttribute::spinning_friction:
    case BodyAttribute::restitution:
    case BodyAttribute::linear_damping:
    case BodyAttribute::angular_damping:
    case BodyAttribute::linear_sleeping_threshold:
    case BodyAttribute::angular_sleeping_threshold:
    case BodyAttribute::total_force:
    case BodyAttribute::total_torque:
      impl.tag_read_cache_changed();
      break;
  }
}

static void physics_attribute_finish(PhysicsGeometryImpl &impl,
                                     const PhysicsGeometry::ConstraintAttribute attribute)
{
  using ConstraintAttribute = PhysicsGeometry::ConstraintAttribute;

  switch (attribute) {
    case ConstraintAttribute::constraint_type:
    case ConstraintAttribute::constraint_body1:
    case ConstraintAttribute::constraint_body2:
    case ConstraintAttribute::constraint_frame1:
    case ConstraintAttribute::constraint_frame2:
    case ConstraintAttribute::constraint_enabled:
    case ConstraintAttribute::applied_impulse:
    case ConstraintAttribute::applied_force1:
    case ConstraintAttribute::applied_force2:
    case ConstraintAttribute::applied_torque1:
    case ConstraintAttribute::applied_torque2:
    case ConstraintAttribute::breaking_impulse_threshold:
      impl.tag_read_cache_changed();
      break;

    case ConstraintAttribute::disable_collision:
      impl.tag_constraint_disable_collision_changed();
      impl.tag_read_cache_changed();
      break;
  }
}

template<typename T> using RigidBodyGetFn = T (*)(const btRigidBody &body);
template<typename T> using RigidBodySetFn = void (*)(btRigidBody &body, T value);

template<typename T> using ConstraintGetFn = T (*)(const btTypedConstraint *constraint);
template<typename T> using ConstraintSetFn = void (*)(btTypedConstraint *constraint, T value);

template<typename ElemT, RigidBodyGetFn<ElemT> GetFn, RigidBodySetFn<ElemT> SetFn>
class VMutableArrayImpl_For_PhysicsBodies final : public VMutableArrayImpl<ElemT> {
 private:
  const PhysicsWorldData *world_data_;

 public:
  VMutableArrayImpl_For_PhysicsBodies(const PhysicsWorldData &world_data)
      : VMutableArrayImpl<ElemT>(world_data.bodies().size()),
        world_data_(&world_data) /*, lock_(impl_->data_mutex)*/
  {
  }

  ~VMutableArrayImpl_For_PhysicsBodies() {}

  template<typename OtherElemT,
           RigidBodyGetFn<OtherElemT> OtherGetFn,
           RigidBodySetFn<OtherElemT> OtherSetFn>
  friend class VMutableArrayImpl_For_PhysicsBodies;

 private:
  ElemT get(const int64_t index) const override
  {
    return GetFn(*world_data_->bodies()[index]);
  }

  void set(const int64_t index, ElemT value) override
  {
    SetFn(*world_data_->bodies()[index], value);
  }

  void materialize(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t i) { dst[i] = GetFn(*world_data_->bodies()[i]); });
  }

  void materialize_to_uninitialized(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t i) { new (dst + i) ElemT(GetFn(*world_data_->bodies()[i])); });
  }

  void materialize_compressed(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t i, const int64_t pos) { dst[pos] = GetFn(*world_data_->bodies()[i]); });
  }

  void materialize_compressed_to_uninitialized(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>([&](const int64_t i, const int64_t pos) {
      new (dst + pos) ElemT(GetFn(*world_data_->bodies()[i]));
    });
  }
};

template<typename ElemT, ConstraintGetFn<ElemT> GetFn, ConstraintSetFn<ElemT> SetFn>
class VMutableArrayImpl_For_PhysicsConstraints final : public VMutableArrayImpl<ElemT> {
 private:
  const PhysicsWorldData *world_data_;

 public:
  VMutableArrayImpl_For_PhysicsConstraints(const PhysicsWorldData &world_data)
      : VMutableArrayImpl<ElemT>(world_data.constraints().size()), world_data_(&world_data)
  {
  }

  ~VMutableArrayImpl_For_PhysicsConstraints() {}

  template<typename OtherElemT,
           ConstraintGetFn<OtherElemT> OtherGetFn,
           ConstraintSetFn<OtherElemT> OtherSetFn>
  friend class VMutableArrayImpl_For_PhysicsConstraints;

 private:
  ElemT get(const int64_t index) const override
  {
    return GetFn(world_data_->constraints()[index]);
  }

  void set(const int64_t index, ElemT value) override
  {
    SetFn(world_data_->constraints()[index], value);
  }

  void materialize(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t i) { dst[i] = GetFn(world_data_->constraints()[i]); });
  }

  void materialize_to_uninitialized(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t i) { new (dst + i) ElemT(GetFn(world_data_->constraints()[i])); });
  }

  void materialize_compressed(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>([&](const int64_t i, const int64_t pos) {
      dst[pos] = GetFn(world_data_->constraints()[i]);
    });
  }

  void materialize_compressed_to_uninitialized(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>([&](const int64_t i, const int64_t pos) {
      new (dst + pos) ElemT(GetFn(world_data_->constraints()[i]));
    });
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
  using PhysicsGetter = PhysicsGeometryImpl *(*)(void *owner);
  using ConstPhysicsGetter = const PhysicsGeometryImpl *(*)(const void *owner);

  PhysicsGetter get_physics;
  ConstPhysicsGetter get_const_physics;
};

/* Base class that adds some utility functions for cache access. */
class BuiltinPhysicsAttributeBase : public bke::BuiltinAttributeProvider {
 protected:
  const PhysicsAccessInfo physics_access_;

 public:
  BuiltinPhysicsAttributeBase(std::string attribute_name,
                              const AttrDomain domain,
                              const eCustomDataType data_type,
                              const DeletableEnum deletable,
                              const PhysicsAccessInfo physics_access,
                              const AttributeValidator validator)
      : BuiltinAttributeProvider(
            std::move(attribute_name), domain, data_type, deletable, validator),
        physics_access_(physics_access)
  {
  }
};

/**
 * Provider for builtin rigid body attributes.
 */
template<typename T, RigidBodyGetFn<T> GetFn, RigidBodySetFn<T> SetFn>
class BuiltinRigidBodyAttributeProvider final : public BuiltinPhysicsAttributeBase {
 public:
  PhysicsGeometry::BodyAttribute attribute_;
  bke::BuiltinCustomDataLayerProvider custom_data_provider_;
  bool allow_cache_;

  BLI_STATIC_ASSERT(GetFn != nullptr, "Attribute must have a get function");
  BLI_STATIC_ASSERT(SetFn != nullptr, "Attribute must have a set function");

  static CustomDataAccessInfo custom_data_access_info()
  {
    return {[](void *owner) -> CustomData * {
              PhysicsGeometryImpl *impl = static_cast<PhysicsGeometryImpl *>(owner);
              return &impl->body_data_;
            },
            [](const void *owner) -> const CustomData * {
              const PhysicsGeometryImpl *impl = static_cast<const PhysicsGeometryImpl *>(owner);
              return &impl->body_data_;
            },
            [](const void *owner) -> int {
              const PhysicsGeometryImpl *impl = static_cast<const PhysicsGeometryImpl *>(owner);
              return impl->body_num_;
            }};
  }

  BuiltinRigidBodyAttributeProvider(PhysicsGeometry::BodyAttribute attribute,
                                    const DeletableEnum deletable,
                                    const PhysicsAccessInfo physics_access,
                                    const bool allow_cache,
                                    const AttributeValidator validator = {})
      : BuiltinPhysicsAttributeBase(physics_attribute_name(attribute),
                                    bke::AttrDomain::Point,
                                    cpp_type_to_custom_data_type(CPPType::get<T>()),
                                    deletable,
                                    physics_access,
                                    validator),
        attribute_(attribute),
        custom_data_provider_(physics_attribute_name(attribute),
                              bke::AttrDomain::Point,
                              cpp_type_to_custom_data_type(CPPType::get<T>()),
                              deletable,
                              custom_data_access_info(),
                              {},
                              validator),
        allow_cache_(allow_cache)
  {
    BLI_assert(&physics_attribute_type(attribute) == &CPPType::get<T>());
  }

  GAttributeReader try_get_for_read(const void *owner) const final
  {
    const PhysicsGeometryImpl *impl = physics_access_.get_const_physics(owner);
    if (impl == nullptr) {
      return {};
    }

    if (allow_cache_) {
      impl->ensure_read_cache();
      return custom_data_provider_.try_get_for_read(owner);
    }

    GVArray varray = VArray<T>::template For<VMutableArrayImpl_For_PhysicsBodies<T, GetFn, SetFn>>(
        *impl->world_data);

    return {std::move(varray), domain_, nullptr};
  }

  GAttributeWriter try_get_for_write(void *owner) const final
  {
    PhysicsGeometryImpl *impl = physics_access_.get_physics(owner);
    if (impl == nullptr) {
      return {};
    }

    if (impl->world_data == nullptr && allow_cache_) {
      impl->ensure_custom_data_attribute(attribute_);
      return custom_data_provider_.try_get_for_write(owner);
    }

    GVMutableArray varray =
        VMutableArray<T>::template For<VMutableArrayImpl_For_PhysicsBodies<T, GetFn, SetFn>>(
            *impl->world_data);

    std::function<void()> tag_modified_fn =
        [physics_access = physics_access_, attribute = attribute_, owner]() {
          PhysicsGeometryImpl &impl = *physics_access.get_physics(owner);
          physics_attribute_finish(impl, attribute);
        };
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
 * Reads from customdata if use_cached_read is true.
 */
template<typename T, ConstraintGetFn<T> GetFn, ConstraintSetFn<T> SetFn = nullptr>
class BuiltinConstraintAttributeProvider final : public BuiltinPhysicsAttributeBase {
 public:
  PhysicsGeometry::ConstraintAttribute attribute_;
  bke::BuiltinCustomDataLayerProvider custom_data_provider_;
  bool allow_cache_;

  BLI_STATIC_ASSERT(GetFn != nullptr, "Attribute must have a get function");
  BLI_STATIC_ASSERT(SetFn != nullptr, "Attribute must have a set function");

  static CustomDataAccessInfo custom_data_access_info()
  {
    return {[](void *owner) -> CustomData * {
              PhysicsGeometryImpl *impl = static_cast<PhysicsGeometryImpl *>(owner);
              return &impl->constraint_data_;
            },
            [](const void *owner) -> const CustomData * {
              const PhysicsGeometryImpl *impl = static_cast<const PhysicsGeometryImpl *>(owner);
              return &impl->constraint_data_;
            },
            [](const void *owner) -> int {
              const PhysicsGeometryImpl *impl = static_cast<const PhysicsGeometryImpl *>(owner);
              return impl->constraint_num_;
            }};
  }

  BuiltinConstraintAttributeProvider(PhysicsGeometry::ConstraintAttribute attribute,
                                     const DeletableEnum deletable,
                                     const PhysicsAccessInfo physics_access,
                                     const bool allow_cache,
                                     const AttributeValidator validator = {})
      : BuiltinPhysicsAttributeBase(physics_attribute_name(attribute),
                                    bke::AttrDomain::Edge,
                                    cpp_type_to_custom_data_type(CPPType::get<T>()),
                                    deletable,
                                    physics_access,
                                    validator),
        attribute_(attribute),
        custom_data_provider_(physics_attribute_name(attribute),
                              bke::AttrDomain::Edge,
                              cpp_type_to_custom_data_type(CPPType::get<T>()),
                              deletable,
                              custom_data_access_info(),
                              {},
                              validator),
        allow_cache_(allow_cache)
  {
    BLI_assert(&physics_attribute_type(attribute) == &CPPType::get<T>());
  }

  GAttributeReader try_get_for_read(const void *owner) const final
  {
    BLI_STATIC_ASSERT(GetFn != nullptr, "Attribute must have a get function");

    const PhysicsGeometryImpl *impl = physics_access_.get_const_physics(owner);
    if (impl == nullptr) {
      return {};
    }

    if (allow_cache_) {
      impl->ensure_read_cache();
      return custom_data_provider_.try_get_for_read(owner);
    }

    GVArray varray =
        VArray<T>::template For<VMutableArrayImpl_For_PhysicsConstraints<T, GetFn, SetFn>>(
            *impl->world_data);

    return {std::move(varray), domain_, nullptr};
  }

  GAttributeWriter try_get_for_write(void *owner) const final
  {
    PhysicsGeometryImpl *impl = physics_access_.get_physics(owner);
    if (impl == nullptr) {
      return {};
    }

    if (impl->world_data == nullptr && allow_cache_) {
      impl->ensure_custom_data_attribute(attribute_);
      return custom_data_provider_.try_get_for_write(owner);
    }

    GVMutableArray varray =
        VMutableArray<T>::template For<VMutableArrayImpl_For_PhysicsConstraints<T, GetFn, SetFn>>(
            *impl->world_data);

    std::function<void()> tag_modified_fn =
        [physics_access = physics_access_, attribute = attribute_, owner]() {
          PhysicsGeometryImpl &impl = *physics_access.get_physics(owner);
          physics_attribute_finish(impl, attribute);
        };
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

const AttributeAccessorFunctions &get_physics_accessor_functions_ref();
const AttributeAccessorFunctions &get_physics_custom_data_accessor_functions_ref();
const AttributeAccessorFunctions &get_physics_world_data_accessor_functions_ref();

}  // namespace blender::bke
