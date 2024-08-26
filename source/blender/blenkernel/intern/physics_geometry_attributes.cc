/* SPDX-FileCopyrightText: 2004-2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sim
 */

#include <functional>

#include "BKE_attribute.hh"

#include "BLI_assert.h"
#include "BLI_math_matrix.hh"
#include "BLI_math_quaternion_types.hh"
#include "BLI_utildefines.h"
#include "BLI_virtual_array.hh"

#include "attribute_access_intern.hh"
#include "physics_geometry_attributes.hh"
#include "physics_geometry_intern.hh"
#include "physics_geometry_world.hh"

#ifdef WITH_BULLET
#  include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#  include <BulletCollision/CollisionShapes/btBoxShape.h>
#  include <BulletCollision/CollisionShapes/btCollisionShape.h>
#  include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
#  include <BulletCollision/Gimpact/btGImpactShape.h>
#  include <BulletDynamics/ConstraintSolver/btConeTwistConstraint.h>
#  include <BulletDynamics/ConstraintSolver/btFixedConstraint.h>
#  include <BulletDynamics/ConstraintSolver/btGearConstraint.h>
#  include <BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h>
#  include <BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h>
#  include <BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.h>
#  include <BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h>
#  include <BulletDynamics/ConstraintSolver/btTypedConstraint.h>
#  include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#  include <BulletDynamics/Dynamics/btRigidBody.h>
#  include <LinearMath/btDefaultMotionState.h>
#  include <LinearMath/btMotionState.h>
#  include <LinearMath/btTransform.h>
#  include <btBulletDynamicsCommon.h>
#endif

namespace blender::bke {

#ifdef WITH_BULLET

struct DefaultOverlapFilter : public btOverlapFilterCallback {
  virtual bool needBroadphaseCollision(btBroadphaseProxy *proxy0, btBroadphaseProxy *proxy1) const
  {
    return (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) &&
           (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);
  }
};

struct OverlapFilterWrapper : public btOverlapFilterCallback {
  using OverlapFilterFn = std::function<bool(const int a, const int b)>;

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

/* -------------------------------------------------------------------- */
/** \name Physics Geometry
 * \{ */

template<bool allow_cache>
static ComponentAttributeProviders create_attribute_providers_for_physics()
{
  using BodyAttribute = PhysicsBodyAttribute;
  using ConstraintAttribute = PhysicsConstraintAttribute;

  static CustomDataAccessInfo body_custom_data_access =
      PhysicsWorldState::body_custom_data_access_info();
  static CustomDataAccessInfo constraint_custom_data_access =
      PhysicsWorldState::constraint_custom_data_access_info();

  const Span<BodyAttribute> body_attributes = all_body_attributes();
  const Span<ConstraintAttribute> constraint_attributes = all_constraint_attributes();
  static Vector<PhysicsStateBodyAttributeProvider> body_attribute_providers;
  static Vector<PhysicsStateConstraintAttributeProvider> constraint_attribute_providers;
  body_attribute_providers.reserve(body_attributes.size());
  constraint_attribute_providers.reserve(constraint_attributes.size());
  for (const BodyAttribute attribute : body_attributes) {
    PhysicsStateBodyAttributeProvider provider(attribute, allow_cache, {});
    body_attribute_providers.append(std::move(provider));
  }
  for (const ConstraintAttribute attribute : constraint_attributes) {
    PhysicsStateConstraintAttributeProvider provider(attribute, allow_cache, {});
    constraint_attribute_providers.append(std::move(provider));
  }

  static Array<const BuiltinAttributeProvider *> builtin_providers(
      body_attribute_providers.size() + constraint_attribute_providers.size());
  const IndexRange body_attribute_provider_range = body_attribute_providers.index_range();
  const IndexRange constraint_attribute_provider_range = body_attribute_provider_range.after(
      constraint_attribute_providers.size());
  for (const int i : body_attribute_providers.index_range()) {
    builtin_providers[body_attribute_provider_range[i]] = &body_attribute_providers[i];
  }
  for (const int i : constraint_attribute_providers.index_range()) {
    builtin_providers[constraint_attribute_provider_range[i]] = &constraint_attribute_providers[i];
  }

  static CustomDataAttributeProvider body_custom_data(AttrDomain::Point, body_custom_data_access);
  static CustomDataAttributeProvider constraint_custom_data(AttrDomain::Point,
                                                            constraint_custom_data_access);

  return ComponentAttributeProviders(builtin_providers,
                                     {&body_custom_data, &constraint_custom_data});
}

static ComponentAttributeProviders create_attribute_providers_for_physics_custom_data()
{
  using BodyAttribute = PhysicsBodyAttribute;
  using ConstraintAttribute = PhysicsConstraintAttribute;

  static CustomDataAccessInfo body_custom_data_access =
      PhysicsWorldState::body_custom_data_access_info();
  static CustomDataAccessInfo constraint_custom_data_access =
      PhysicsWorldState::constraint_custom_data_access_info();

  const Span<BodyAttribute> body_attributes = all_body_attributes();
  const Span<ConstraintAttribute> constraint_attributes = all_constraint_attributes();
  static Vector<BuiltinCustomDataLayerProvider> builtin_providers_data;
  builtin_providers_data.reserve(body_attributes.size() + constraint_attributes.size());
  for (const BodyAttribute attribute : body_attributes) {
    BuiltinCustomDataLayerProvider provider(
        physics_attribute_name(attribute),
        AttrDomain::Point,
        cpp_type_to_custom_data_type(physics_attribute_type(attribute)),
        BuiltinAttributeProvider::NonDeletable,
        body_custom_data_access,
        {},
        {});
    builtin_providers_data.append(std::move(provider));
  }
  for (const ConstraintAttribute attribute : constraint_attributes) {
    BuiltinCustomDataLayerProvider provider(
        physics_attribute_name(attribute),
        AttrDomain::Edge,
        cpp_type_to_custom_data_type(physics_attribute_type(attribute)),
        BuiltinAttributeProvider::NonDeletable,
        constraint_custom_data_access,
        {},
        {});
    builtin_providers_data.append(std::move(provider));
  }

  Array<const BuiltinAttributeProvider *> builtin_providers(builtin_providers_data.size());
  for (const int i : builtin_providers.index_range()) {
    builtin_providers[i] = &builtin_providers_data[i];
  }

  static CustomDataAttributeProvider body_custom_data(AttrDomain::Point, body_custom_data_access);
  static CustomDataAttributeProvider constraint_custom_data(AttrDomain::Point,
                                                            constraint_custom_data_access);

  return ComponentAttributeProviders(builtin_providers,
                                     {&body_custom_data, &constraint_custom_data});
}

static GVArray adapt_physics_attribute_domain(const PhysicsWorldState & /*state*/,
                                              const GVArray &varray,
                                              const AttrDomain from,
                                              const AttrDomain to)
{
  if (from == to) {
    return varray;
  }
  return {};
}

static AttributeAccessorFunctions get_physics_accessor_functions(const bool enable_custom_data,
                                                                 const bool enable_world_data)
{
  static const ComponentAttributeProviders providers =
      create_attribute_providers_for_physics<true>();
  static const ComponentAttributeProviders custom_data_providers =
      create_attribute_providers_for_physics_custom_data();
  static const ComponentAttributeProviders world_data_providers =
      create_attribute_providers_for_physics<false>();
  BLI_assert(enable_custom_data || enable_world_data);
  AttributeAccessorFunctions fn =
      (enable_world_data ?
           (enable_custom_data ?
                attribute_accessor_functions::accessor_functions_for_providers<providers>() :
                attribute_accessor_functions::accessor_functions_for_providers<
                    world_data_providers>()) :
           attribute_accessor_functions::accessor_functions_for_providers<
               custom_data_providers>());
  fn.domain_size = [](const void *owner, const AttrDomain domain) {
    if (owner == nullptr) {
      return 0;
    }

    const PhysicsWorldState &state = *static_cast<const PhysicsWorldState *>(owner);
    switch (domain) {
      case AttrDomain::Point:
        return state.bodies_num();
      case AttrDomain::Edge:
        return state.constraints_num();
      case blender::bke::AttrDomain::Instance:
        return int(state.shapes_num());
      default:
        return 0;
    }
  };
  fn.domain_supported = [](const void * /*owner*/, const AttrDomain domain) {
    return ELEM(domain, AttrDomain::Point, AttrDomain::Edge, AttrDomain::Instance);
  };
  fn.adapt_domain = [](const void *owner,
                       const GVArray &varray,
                       const AttrDomain from_domain,
                       const AttrDomain to_domain) -> GVArray {
    if (owner == nullptr) {
      return {};
    }
    const PhysicsWorldState &state = *static_cast<const PhysicsWorldState *>(owner);
    return adapt_physics_attribute_domain(state, varray, from_domain, to_domain);
  };
  return fn;
}

const AttributeAccessorFunctions &get_physics_accessor_functions_ref()
{
  static const AttributeAccessorFunctions fn = get_physics_accessor_functions(true, true);
  return fn;
}

const AttributeAccessorFunctions &get_physics_custom_data_accessor_functions_ref()
{
  static const AttributeAccessorFunctions fn = get_physics_accessor_functions(true, false);
  return fn;
}

const AttributeAccessorFunctions &get_physics_world_data_accessor_functions_ref()
{
  static const AttributeAccessorFunctions fn = get_physics_accessor_functions(false, true);
  return fn;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Attribute Info for Physics Geometry
 * \{ */

Span<PhysicsBodyAttribute> all_body_attributes()
{
  using BodyAttribute = PhysicsBodyAttribute;
  static Array<BodyAttribute> attributes = {
      BodyAttribute::id,
      BodyAttribute::collision_shape,
      BodyAttribute::is_static,
      BodyAttribute::is_kinematic,
      BodyAttribute::mass,
      BodyAttribute::inertia,
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

Span<PhysicsConstraintAttribute> all_constraint_attributes()
{
  using ConstraintAttribute = PhysicsConstraintAttribute;
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

StringRef physics_attribute_name(PhysicsBodyAttribute attribute)
{
  using BodyAttribute = PhysicsBodyAttribute;

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

StringRef physics_attribute_name(PhysicsConstraintAttribute attribute)
{
  using ConstraintAttribute = PhysicsConstraintAttribute;

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

Span<std::string> all_body_attribute_names()
{
  static Array<std::string> all_names = []() {
    const Span<PhysicsBodyAttribute> all_attributes = all_body_attributes();
    Array<std::string> all_names(all_attributes.size());
    for (const int i : all_names.index_range()) {
      all_names[i] = physics_attribute_name(all_attributes[i]);
    }
    return all_names;
  }();
  return all_names;
}

Span<std::string> all_constraint_attribute_names()
{
  static Array<std::string> all_names = []() {
    const Span<PhysicsConstraintAttribute> all_attributes = all_constraint_attributes();
    Array<std::string> all_names(all_attributes.size());
    for (const int i : all_names.index_range()) {
      all_names[i] = physics_attribute_name(all_attributes[i]);
    }
    return all_names;
  }();
  return all_names;
}

const CPPType &physics_attribute_type(PhysicsBodyAttribute attribute)
{
  using BodyAttribute = PhysicsBodyAttribute;

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

const CPPType &physics_attribute_type(PhysicsConstraintAttribute attribute)
{
  using ConstraintAttribute = PhysicsConstraintAttribute;

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

const void *physics_attribute_default_value(PhysicsBodyAttribute attribute)
{
  using BodyAttribute = PhysicsBodyAttribute;

  const CPPType &type = physics_attribute_type(attribute);
  switch (attribute) {
    case BodyAttribute::collision_shape: {
      static const int default_value = -1;
      return &default_value;
    }
    case BodyAttribute::mass: {
      static const float default_value = 1.0f;
      return &default_value;
    }
    case BodyAttribute::inertia: {
      static const float3 default_value = float3(1.0f);
      return &default_value;
    }
    case BodyAttribute::activation_state: {
      static const int default_value = int(PhysicsWorldData::BodyActivationState::Active);
      return &default_value;
    }
    case BodyAttribute::friction: {
      static const float default_value = 0.5f;
      return &default_value;
    }
    case BodyAttribute::linear_sleeping_threshold: {
      static const float default_value = 0.8f;
      return &default_value;
    }
    case BodyAttribute::angular_sleeping_threshold: {
      static const float default_value = 1.0f;
      return &default_value;
    }

    case BodyAttribute::id:
    case BodyAttribute::is_static:
    case BodyAttribute::is_kinematic:
    case BodyAttribute::position:
    case BodyAttribute::rotation:
    case BodyAttribute::velocity:
    case BodyAttribute::angular_velocity:
    case BodyAttribute::rolling_friction:
    case BodyAttribute::spinning_friction:
    case BodyAttribute::restitution:
    case BodyAttribute::linear_damping:
    case BodyAttribute::angular_damping:
    case BodyAttribute::total_force:
    case BodyAttribute::total_torque:
      return type.default_value();
  }
  BLI_assert_unreachable();
  return nullptr;
}

const void *physics_attribute_default_value(PhysicsConstraintAttribute attribute)
{
  using ConstraintAttribute = PhysicsConstraintAttribute;

  const CPPType &type = physics_attribute_type(attribute);
  switch (attribute) {
    case ConstraintAttribute::constraint_type: {
      static const int default_value = int(PhysicsConstraintType::Fixed);
      return &default_value;
    }
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
    case ConstraintAttribute::constraint_enabled: {
      static const bool default_value = true;
      return &default_value;
    }
    case ConstraintAttribute::breaking_impulse_threshold: {
      static const float default_value = std::numeric_limits<float>::infinity();
      return &default_value;
    }

    case ConstraintAttribute::applied_impulse:
    case ConstraintAttribute::applied_force1:
    case ConstraintAttribute::applied_force2:
    case ConstraintAttribute::applied_torque1:
    case ConstraintAttribute::applied_torque2:
    case ConstraintAttribute::disable_collision:
      return type.default_value();
  }
  BLI_assert_unreachable();
  return nullptr;
}

/** True if the attribute is stored in the physics state and world data is updated after writing.
 */
bool physics_attribute_is_state_local(const PhysicsBodyAttribute attribute)
{
  using BodyAttribute = PhysicsBodyAttribute;

  switch (attribute) {
    case BodyAttribute::collision_shape:
    case BodyAttribute::is_static:
    case BodyAttribute::mass:
      return true;

    case BodyAttribute::id:
    case BodyAttribute::is_kinematic:
    case BodyAttribute::inertia:
    case BodyAttribute::position:
    case BodyAttribute::rotation:
    case BodyAttribute::velocity:
    case BodyAttribute::angular_velocity:
    case BodyAttribute::friction:
    case BodyAttribute::rolling_friction:
    case BodyAttribute::spinning_friction:
    case BodyAttribute::restitution:
    case BodyAttribute::activation_state:
    case BodyAttribute::linear_damping:
    case BodyAttribute::angular_damping:
    case BodyAttribute::linear_sleeping_threshold:
    case BodyAttribute::angular_sleeping_threshold:
    case BodyAttribute::total_force:
    case BodyAttribute::total_torque:
      return false;
  }
  BLI_assert_unreachable();
  return true;
}

/** True if the attribute is stored in the physics state and world data is updated after writing.
 */
bool physics_attribute_is_state_local(const PhysicsConstraintAttribute attribute)
{
  using ConstraintAttribute = PhysicsConstraintAttribute;

  switch (attribute) {
    case ConstraintAttribute::constraint_type:
    case ConstraintAttribute::constraint_body1:
    case ConstraintAttribute::constraint_body2:
    case ConstraintAttribute::disable_collision:
      return true;

    case ConstraintAttribute::constraint_frame1:
    case ConstraintAttribute::constraint_frame2:
    case ConstraintAttribute::constraint_enabled:
    case ConstraintAttribute::breaking_impulse_threshold:
    case ConstraintAttribute::applied_impulse:
    case ConstraintAttribute::applied_force1:
    case ConstraintAttribute::applied_force2:
    case ConstraintAttribute::applied_torque1:
    case ConstraintAttribute::applied_torque2:
      return false;
  }
  BLI_assert_unreachable();
  return true;
}

/** \} */

using CustomDataUpdateOnChange = void (*)(void *owner);
using WorldDataUpdateOnChange = void (*)(void *owner);

static CustomDataUpdateOnChange custom_data_update_on_change_fn(
    const PhysicsBodyAttribute attribute)
{
  using BodyAttribute = PhysicsBodyAttribute;

  switch (attribute) {
    case BodyAttribute::collision_shape: {
      static CustomDataUpdateOnChange update = [](void *owner) {
        PhysicsWorldState &state = get_physics_owner(owner);
        state.tag_body_collision_shape_changed();
      };
      return update;
    }
    case BodyAttribute::is_static: {
      static CustomDataUpdateOnChange update = [](void *owner) {
        PhysicsWorldState &state = get_physics_owner(owner);
        state.tag_body_is_static_changed();
      };
      return update;
    }
    case BodyAttribute::mass: {
      static CustomDataUpdateOnChange update = [](void *owner) {
        PhysicsWorldState &state = get_physics_owner(owner);
        state.tag_body_mass_changed();
      };
      return update;
    }

    case BodyAttribute::id:
    case BodyAttribute::is_kinematic:
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
      return nullptr;
  }
  BLI_assert_unreachable();
  return nullptr;
}

static CustomDataUpdateOnChange custom_data_update_on_change_fn(
    const PhysicsConstraintAttribute attribute)
{
  using ConstraintAttribute = PhysicsConstraintAttribute;

  switch (attribute) {
    case ConstraintAttribute::constraint_type:
    case ConstraintAttribute::constraint_body1:
    case ConstraintAttribute::constraint_body2: {
      static CustomDataUpdateOnChange update = [](void *owner) {
        PhysicsWorldState &state = get_physics_owner(owner);
        state.tag_constraints_changed();
      };
      return update;
    }
    case ConstraintAttribute::disable_collision: {
      static CustomDataUpdateOnChange update = [](void *owner) {
        PhysicsWorldState &state = get_physics_owner(owner);
        state.tag_constraint_disable_collision_changed();
      };
      return update;
    }

    case ConstraintAttribute::constraint_frame1:
    case ConstraintAttribute::constraint_frame2:
    case ConstraintAttribute::constraint_enabled:
    case ConstraintAttribute::applied_impulse:
    case ConstraintAttribute::applied_force1:
    case ConstraintAttribute::applied_force2:
    case ConstraintAttribute::applied_torque1:
    case ConstraintAttribute::applied_torque2:
    case ConstraintAttribute::breaking_impulse_threshold:
      return nullptr;
  }
  BLI_assert_unreachable();
  return nullptr;
}

static WorldDataUpdateOnChange world_data_update_on_change_fn(const PhysicsBodyAttribute attribute)
{
  using BodyAttribute = PhysicsBodyAttribute;

  switch (attribute) {
    case BodyAttribute::collision_shape:
    case BodyAttribute::is_static:
    case BodyAttribute::mass:
      return nullptr;

    case BodyAttribute::id:
    case BodyAttribute::is_kinematic:
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
    case BodyAttribute::total_torque: {
      static WorldDataUpdateOnChange update = [](void *owner) {
        PhysicsWorldState &state = get_physics_owner(owner);
        state.tag_read_cache_changed();
      };
      return update;
    }
  }
  BLI_assert_unreachable();
  return nullptr;
}

static WorldDataUpdateOnChange world_data_update_on_change_fn(
    const PhysicsConstraintAttribute attribute)
{
  using ConstraintAttribute = PhysicsConstraintAttribute;

  switch (attribute) {
    case ConstraintAttribute::constraint_type:
    case ConstraintAttribute::constraint_body1:
    case ConstraintAttribute::constraint_body2:
    case ConstraintAttribute::disable_collision:
      return nullptr;

    case ConstraintAttribute::constraint_frame1:
    case ConstraintAttribute::constraint_frame2:
    case ConstraintAttribute::constraint_enabled:
    case ConstraintAttribute::applied_impulse:
    case ConstraintAttribute::applied_force1:
    case ConstraintAttribute::applied_force2:
    case ConstraintAttribute::applied_torque1:
    case ConstraintAttribute::applied_torque2:
    case ConstraintAttribute::breaking_impulse_threshold: {
      static WorldDataUpdateOnChange update = [](void *owner) {
        PhysicsWorldState &state = get_physics_owner(owner);
        state.tag_read_cache_changed();
      };
      return update;
    }
  }
  BLI_assert_unreachable();
  return nullptr;
}

PhysicsStateBodyAttributeProvider::PhysicsStateBodyAttributeProvider(
    PhysicsBodyAttribute attribute, const bool allow_cache, const AttributeValidator validator)
    : BuiltinAttributeProvider(physics_attribute_name(attribute),
                               bke::AttrDomain::Point,
                               cpp_type_to_custom_data_type(physics_attribute_type(attribute)),
                               NonDeletable,
                               validator),
      attribute_(attribute),
      custom_data_provider_(physics_attribute_name(attribute),
                            bke::AttrDomain::Point,
                            cpp_type_to_custom_data_type(physics_attribute_type(attribute)),
                            NonDeletable,
                            PhysicsWorldState::body_custom_data_access_info(),
                            custom_data_update_on_change_fn(attribute),
                            validator),
      world_data_provider_(attribute,
                           PhysicsWorldState::world_data_access_info(),
                           world_data_update_on_change_fn(attribute),
                           {}),
      allow_cache_(allow_cache)
{
}

GAttributeReader PhysicsStateBodyAttributeProvider::try_get_for_read(const void *owner) const
{
  const PhysicsWorldState &state = get_physics_owner(owner);

  if (allow_cache_) {
    state.ensure_read_cache();
    return custom_data_provider_.try_get_for_read(owner);
  }

  return world_data_provider_.try_get_for_read(owner);
}

GAttributeWriter PhysicsStateBodyAttributeProvider::try_get_for_write(void *owner) const
{
  PhysicsWorldState &state = get_physics_owner(owner);

  if (allow_cache_ && (!state.has_world_data() || physics_attribute_is_state_local(attribute_))) {
    state.ensure_custom_data_attribute(attribute_);
    return custom_data_provider_.try_get_for_write(owner);
  }

  BLI_assert(state.has_world_data());
  return world_data_provider_.try_get_for_write(owner);
}

bool PhysicsStateBodyAttributeProvider::try_delete(void * /*owner*/) const
{
  return false;
}

bool PhysicsStateBodyAttributeProvider::try_create(void * /*owner*/,
                                                   const AttributeInit & /*initializer*/) const
{
  return false;
}

bool PhysicsStateBodyAttributeProvider::exists(const void * /*owner*/) const
{
  return true;
}

PhysicsStateConstraintAttributeProvider::PhysicsStateConstraintAttributeProvider(
    PhysicsConstraintAttribute attribute,
    const bool allow_cache,
    const AttributeValidator validator)
    : BuiltinAttributeProvider(physics_attribute_name(attribute),
                               bke::AttrDomain::Edge,
                               cpp_type_to_custom_data_type(physics_attribute_type(attribute)),
                               NonDeletable,
                               validator),
      attribute_(attribute),
      custom_data_provider_(physics_attribute_name(attribute),
                            bke::AttrDomain::Edge,
                            cpp_type_to_custom_data_type(physics_attribute_type(attribute)),
                            NonDeletable,
                            PhysicsWorldState::constraint_custom_data_access_info(),
                            custom_data_update_on_change_fn(attribute),
                            validator),
      world_data_provider_(attribute,
                           PhysicsWorldState::world_data_access_info(),
                           world_data_update_on_change_fn(attribute),
                           {}),
      allow_cache_(allow_cache)
{
}

GAttributeReader PhysicsStateConstraintAttributeProvider::try_get_for_read(const void *owner) const
{
  const PhysicsWorldState &state = get_physics_owner(owner);

  if (allow_cache_) {
    state.ensure_read_cache();
    return custom_data_provider_.try_get_for_read(owner);
  }

  return world_data_provider_.try_get_for_read(owner);
}

GAttributeWriter PhysicsStateConstraintAttributeProvider::try_get_for_write(void *owner) const
{
  PhysicsWorldState &state = get_physics_owner(owner);

  if (allow_cache_ && (!state.has_world_data() || physics_attribute_is_state_local(attribute_))) {
    state.ensure_custom_data_attribute(attribute_);
    return custom_data_provider_.try_get_for_write(owner);
  }

  BLI_assert(state.has_world_data());
  return world_data_provider_.try_get_for_write(owner);
}

bool PhysicsStateConstraintAttributeProvider::try_delete(void * /*owner*/) const
{
  return false;
}

bool PhysicsStateConstraintAttributeProvider::try_create(
    void * /*owner*/, const AttributeInit & /*initializer*/) const
{
  return false;
}

bool PhysicsStateConstraintAttributeProvider::exists(const void * /*owner*/) const
{
  return true;
}

#endif

}  // namespace blender::bke
