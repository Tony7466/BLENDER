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

CollisionShapeComponent::CollisionShapeComponent() : GeometryComponent(Type::CollisionShape) {}

CollisionShapeComponent::CollisionShapeComponent(CollisionShape *shape,
                                                 GeometryOwnershipType ownership)
    : GeometryComponent(Type::CollisionShape), shape_(shape), ownership_(ownership)
{
}

CollisionShapeComponent::~CollisionShapeComponent()
{
  this->clear();
}

GeometryComponentPtr CollisionShapeComponent::copy() const
{
  CollisionShapeComponent *new_component = new CollisionShapeComponent();
  if (shape_ != nullptr) {
    new_component->shape_ = new CollisionShape(*shape_);
    new_component->ownership_ = GeometryOwnershipType::Owned;
  }
  return GeometryComponentPtr(new_component);
}

void CollisionShapeComponent::clear()
{
  BLI_assert(this->is_mutable() || this->is_expired());
  if (shape_ != nullptr) {
    if (ownership_ == GeometryOwnershipType::Owned) {
      delete shape_;
    }
    shape_ = nullptr;
  }
}

bool CollisionShapeComponent::has_shape() const
{
  return shape_ != nullptr;
}

void CollisionShapeComponent::replace(CollisionShape *shape, GeometryOwnershipType ownership)
{
  BLI_assert(this->is_mutable());
  this->clear();
  shape_ = shape;
  ownership_ = ownership;
}

CollisionShape *CollisionShapeComponent::release()
{
  BLI_assert(this->is_mutable());
  CollisionShape *shape = shape_;
  shape_ = nullptr;
  return shape;
}

const CollisionShape *CollisionShapeComponent::get() const
{
  return shape_;
}

CollisionShape *CollisionShapeComponent::get_for_write()
{
  BLI_assert(this->is_mutable());
  if (ownership_ == GeometryOwnershipType::ReadOnly) {
    shape_ = new CollisionShape(*shape_);
    ownership_ = GeometryOwnershipType::Owned;
  }
  return shape_;
}

bool CollisionShapeComponent::is_empty() const
{
  return shape_ == nullptr;
}

bool CollisionShapeComponent::owns_direct_data() const
{
  return ownership_ == GeometryOwnershipType::Owned;
}

void CollisionShapeComponent::ensure_owns_direct_data()
{
  BLI_assert(this->is_mutable());
  if (ownership_ != GeometryOwnershipType::Owned) {
    if (shape_) {
      shape_ = new CollisionShape(*shape_);
    }
    ownership_ = GeometryOwnershipType::Owned;
  }
}

}  // namespace blender::bke
