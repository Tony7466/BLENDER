/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_collision_shape.hh"
#include "BKE_geometry_set.hh"
#include "BKE_physics_geometry.hh"

#include "attribute_access_intern.hh"
#include <optional>
#include <type_traits>

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

bool PhysicsComponent::has_physics() const
{
  return physics_ != nullptr;
}

bool PhysicsComponent::has_world() const
{
  return physics_ != nullptr && physics_->state().has_world_data();
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
  return physics_ ? physics_->state().bodies_num() : 0;
}

int PhysicsComponent::constraints_num() const
{
  return physics_ ? physics_->state().constraints_num() : 0;
}

int PhysicsComponent::shapes_num() const
{
  return physics_ ? physics_->state().shapes_num() : 0;
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

}  // namespace blender::bke

namespace blender::bke {

std::optional<AttributeAccessor> PhysicsComponent::attributes() const
{
  return std::make_optional(physics_ ? physics_->attributes() :
                                       PhysicsGeometry::dummy_attributes());
}

std::optional<MutableAttributeAccessor> PhysicsComponent::attributes_for_write()
{
  return physics_ ? physics_->attributes_for_write() :
                    PhysicsGeometry::dummy_attributes_for_write();
}

}  // namespace blender::bke
