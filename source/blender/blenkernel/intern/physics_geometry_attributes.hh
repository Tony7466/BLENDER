/* SPDX-FileCopyrightText: Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sim
 */

#pragma once

#include <limits>
#include <shared_mutex>

#include "BLI_assert.h"
#include "BLI_virtual_array.hh"

#include "BKE_attribute.hh"
#include "BKE_customdata.hh"

#include "DNA_customdata_types.h"

#include "attribute_access_intern.hh"
#include "physics_geometry_world.hh"

class btRigidBody;
class btMotionState;
class btCollisionShape;
class btTypedConstraint;

namespace blender::bke {

/* -------------------------------------------------------------------- */
/** \name Attribute Info for Physics Geometry
 * \{ */

Span<PhysicsBodyAttribute> all_body_attributes();
Span<PhysicsConstraintAttribute> all_constraint_attributes();

StringRef physics_attribute_name(PhysicsBodyAttribute attribute);
StringRef physics_attribute_name(PhysicsConstraintAttribute attribute);

Span<std::string> all_body_attribute_names();
Span<std::string> all_constraint_attribute_names();

const CPPType &physics_attribute_type(PhysicsBodyAttribute attribute);
const CPPType &physics_attribute_type(PhysicsConstraintAttribute attribute);

const void *physics_attribute_default_value(PhysicsBodyAttribute attribute);
const void *physics_attribute_default_value(PhysicsConstraintAttribute attribute);

/* True if the attribute is stored in the physics state and world data is updated after writing. */
bool physics_attribute_is_state_local(const PhysicsBodyAttribute attribute);
bool physics_attribute_is_state_local(const PhysicsConstraintAttribute attribute);

/** \} */

/* -------------------------------------------------------------------- */
/** \name Attribute Providers for Physics Geometry
 * \{ */

inline PhysicsWorldState &get_physics_owner(void *owner)
{
  return *static_cast<PhysicsWorldState *>(owner);
}

inline const PhysicsWorldState &get_physics_owner(const void *owner)
{
  return *static_cast<const PhysicsWorldState *>(owner);
}

/**
 * Utility to group together multiple functions that are used to access custom data on geometry
 * components in a generic way.
 */
struct PhysicsWorldDataAccessInfo {
  using Getter = PhysicsWorldData *(*)(void *owner);
  using ConstGetter = const PhysicsWorldData *(*)(const void *owner);

  Getter get_world_data;
  ConstGetter get_const_world_data;
};

/**
 * Provider for builtin body attributes.
 */
class PhysicsWorldBodyAttributeProvider final : public BuiltinAttributeProvider {
 public:
  using BodyAttribute = PhysicsBodyAttribute;
  using UpdateOnChange = void (*)(void *owner);

  BodyAttribute attribute_;
  PhysicsWorldDataAccessInfo access_info_;
  UpdateOnChange update_on_change_;

  PhysicsWorldBodyAttributeProvider(const BodyAttribute attribute,
                                    PhysicsWorldDataAccessInfo access_info,
                                    UpdateOnChange update_on_change,
                                    const AttributeValidator validator = {});

  GAttributeReader try_get_for_read(const void *owner) const final;
  GAttributeWriter try_get_for_write(void *owner) const final;

  bool try_delete(void *owner) const final;
  bool try_create(void *owner, const AttributeInit &initializer) const final;
  bool exists(const void *owner) const final;
};

/**
 * Provider for builtin constraint attributes.
 */
class PhysicsWorldConstraintAttributeProvider final : public BuiltinAttributeProvider {
 public:
  using ConstraintAttribute = PhysicsConstraintAttribute;
  using UpdateOnChange = void (*)(void *owner);

  ConstraintAttribute attribute_;
  PhysicsWorldDataAccessInfo access_info_;
  UpdateOnChange update_on_change_;

  PhysicsWorldConstraintAttributeProvider(const ConstraintAttribute attribute,
                                          PhysicsWorldDataAccessInfo access_info,
                                          UpdateOnChange update_on_change,
                                          const AttributeValidator validator = {});

  GAttributeReader try_get_for_read(const void *owner) const final;
  GAttributeWriter try_get_for_write(void *owner) const final;

  bool try_delete(void *owner) const final;
  bool try_create(void *owner, const AttributeInit &initializer) const final;
  bool exists(const void *owner) const final;
};

/**
 * Provider for builtin rigid body attributes.
 */
class PhysicsStateBodyAttributeProvider final : public BuiltinAttributeProvider {
 public:
  PhysicsBodyAttribute attribute_;
  BuiltinCustomDataLayerProvider custom_data_provider_;
  PhysicsWorldBodyAttributeProvider world_data_provider_;
  bool allow_cache_;

  PhysicsStateBodyAttributeProvider(PhysicsBodyAttribute attribute,
                                    const bool allow_cache,
                                    const AttributeValidator validator = {});

  GAttributeReader try_get_for_read(const void *owner) const final;
  GAttributeWriter try_get_for_write(void *owner) const final;

  bool try_delete(void *owner) const final;
  bool try_create(void *owner, const AttributeInit &initializer) const final;
  bool exists(const void *owner) const final;
};

/**
 * Provider for builtin constraint attributes.
 * Reads from customdata if use_cached_read is true.
 */
class PhysicsStateConstraintAttributeProvider final : public BuiltinAttributeProvider {
 public:
  PhysicsConstraintAttribute attribute_;
  bke::BuiltinCustomDataLayerProvider custom_data_provider_;
  PhysicsWorldConstraintAttributeProvider world_data_provider_;
  bool allow_cache_;

  PhysicsStateConstraintAttributeProvider(PhysicsConstraintAttribute attribute,
                                          const bool allow_cache,
                                          const AttributeValidator validator = {});

  GAttributeReader try_get_for_read(const void *owner) const final;
  GAttributeWriter try_get_for_write(void *owner) const final;
  bool try_delete(void *owner) const final;
  bool try_create(void *owner, const AttributeInit &initializer) const final;
  bool exists(const void *owner) const final;
};

/** \} */

const AttributeAccessorFunctions &get_physics_accessor_functions_ref();
const AttributeAccessorFunctions &get_physics_custom_data_accessor_functions_ref();
const AttributeAccessorFunctions &get_physics_world_data_accessor_functions_ref();

}  // namespace blender::bke
