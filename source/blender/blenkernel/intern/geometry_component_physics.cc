/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_collision_shape.hh"
#include "BKE_geometry_set.hh"
#include "BKE_physics_geometry.hh"

#include "attribute_access_intern.hh"

namespace blender::bke {

/* -------------------------------------------------------------------- */
/** \name Geometry Component Implementation
 * \{ */

PhysicsComponent::PhysicsComponent() : GeometryComponent(Type::Physics) {}

PhysicsComponent::PhysicsComponent(PhysicsGeometry *physics, GeometryOwnershipType ownership)
    : GeometryComponent(Type::Physics), physics_(physics), ownership_(ownership)
{
}

PhysicsComponent::~PhysicsComponent()
{
  this->clear();
}

GeometryComponentPtr PhysicsComponent::copy() const
{
  PhysicsComponent *new_component = new PhysicsComponent();
  if (physics_ != nullptr) {
    new_component->physics_ = new PhysicsGeometry(*physics_);
    new_component->ownership_ = GeometryOwnershipType::Owned;
  }
  return GeometryComponentPtr(new_component);
}

void PhysicsComponent::clear()
{
  BLI_assert(this->is_mutable() || this->is_expired());
  if (physics_ != nullptr) {
    if (ownership_ == GeometryOwnershipType::Owned) {
      delete physics_;
    }
    physics_ = nullptr;
  }
}

bool PhysicsComponent::has_world() const
{
  return physics_ != nullptr;
}

void PhysicsComponent::replace(PhysicsGeometry *physics, GeometryOwnershipType ownership)
{
  BLI_assert(this->is_mutable());
  this->clear();
  physics_ = physics;
  ownership_ = ownership;
}

PhysicsGeometry *PhysicsComponent::release()
{
  BLI_assert(this->is_mutable());
  PhysicsGeometry *physics = physics_;
  physics_ = nullptr;
  return physics;
}

const PhysicsGeometry *PhysicsComponent::get() const
{
  return physics_;
}

PhysicsGeometry *PhysicsComponent::get_for_write()
{
  BLI_assert(this->is_mutable());
  if (ownership_ == GeometryOwnershipType::ReadOnly) {
    physics_ = new PhysicsGeometry(*physics_);
    ownership_ = GeometryOwnershipType::Owned;
  }
  return physics_;
}

bool PhysicsComponent::is_empty() const
{
  return physics_ == nullptr;
}

int PhysicsComponent::bodies_num() const
{
  return physics_ ? physics_->rigid_bodies_num() : 0;
}

int PhysicsComponent::constraints_num() const
{
  return physics_ ? physics_->constraints_num() : 0;
}

int PhysicsComponent::shapes_num() const
{
  return 0;
}

bool PhysicsComponent::owns_direct_data() const
{
  return ownership_ == GeometryOwnershipType::Owned;
}

void PhysicsComponent::ensure_owns_direct_data()
{
  BLI_assert(this->is_mutable());
  if (ownership_ != GeometryOwnershipType::Owned) {
    if (physics_) {
      physics_ = new PhysicsGeometry(*physics_);
    }
    ownership_ = GeometryOwnershipType::Owned;
  }
}

static ComponentAttributeProviders create_attribute_providers_for_physics()
{
  return ComponentAttributeProviders({}, {});
}

static GVArray adapt_physics_attribute_domain(const PhysicsGeometry & /*physics*/,
                                              const GVArray &varray,
                                              const AttrDomain from,
                                              const AttrDomain to)
{
  if (from == to) {
    return varray;
  }
  return {};
}

static AttributeAccessorFunctions get_physics_accessor_functions()
{
  static const ComponentAttributeProviders providers = create_attribute_providers_for_physics();
  AttributeAccessorFunctions fn =
      attribute_accessor_functions::accessor_functions_for_providers<providers>();
  fn.domain_size = [](const void *owner, const AttrDomain domain) {
    if (owner == nullptr) {
      return 0;
    }
    const PhysicsGeometry &physics = *static_cast<const PhysicsGeometry *>(owner);
    switch (domain) {
      case AttrDomain::Point:
        return int(physics.rigid_bodies_num());
      case AttrDomain::Edge:
        return int(physics.constraints_num());
      default:
        return 0;
    }
  };
  fn.domain_supported = [](const void * /*owner*/, const AttrDomain domain) {
    return ELEM(domain, AttrDomain::Point, AttrDomain::Edge);
  };
  fn.adapt_domain = [](const void *owner,
                       const GVArray &varray,
                       const AttrDomain from_domain,
                       const AttrDomain to_domain) -> GVArray {
    if (owner == nullptr) {
      return {};
    }
    const PhysicsGeometry &physics = *static_cast<const PhysicsGeometry *>(owner);
    return adapt_physics_attribute_domain(physics, varray, from_domain, to_domain);
  };
  return fn;
}

static const AttributeAccessorFunctions &get_physics_accessor_functions_ref()
{
  static const AttributeAccessorFunctions fn = get_physics_accessor_functions();
  return fn;
}

}  // namespace blender::bke

namespace blender::bke {

std::optional<AttributeAccessor> PhysicsComponent::attributes() const
{
  return AttributeAccessor(physics_, get_physics_accessor_functions_ref());
}

std::optional<MutableAttributeAccessor> PhysicsComponent::attributes_for_write()
{
  PhysicsGeometry *physics = this->get_for_write();
  return MutableAttributeAccessor(physics, get_physics_accessor_functions_ref());
}

AttributeAccessor PhysicsGeometry::attributes() const
{
  return AttributeAccessor(this, bke::get_physics_accessor_functions_ref());
}

MutableAttributeAccessor PhysicsGeometry::attributes_for_write()
{
  return MutableAttributeAccessor(this, bke::get_physics_accessor_functions_ref());
}

}  // namespace blender::bke
