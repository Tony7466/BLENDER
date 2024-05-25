/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_geometry_set.hh"
#include "BKE_lib_id.hh"

#include "SIM_rigid_body.hh"

#include "attribute_access_intern.hh"

namespace blender::bke {

using simulation::RigidBodyWorld;

/* -------------------------------------------------------------------- */
/** \name Geometry Component Implementation
 * \{ */

RigidBodyComponent::RigidBodyComponent() : GeometryComponent(Type::RigidBody) {}

RigidBodyComponent::RigidBodyComponent(RigidBodyWorld *rigid_body_world,
                                       GeometryOwnershipType ownership)
    : GeometryComponent(Type::RigidBody),
      rigid_body_world_(rigid_body_world),
      ownership_(ownership)
{
}

RigidBodyComponent::~RigidBodyComponent()
{
  this->clear();
}

GeometryComponentPtr RigidBodyComponent::copy() const
{
  RigidBodyComponent *new_component = new RigidBodyComponent();
  if (rigid_body_world_ != nullptr) {
    // TODO
    // new_component->rigid_body_world_ = BKE_rigid_body_world_copy_for_eval(rigid_body_world_);
    new_component->ownership_ = GeometryOwnershipType::Owned;
  }
  return GeometryComponentPtr(new_component);
}

void RigidBodyComponent::clear()
{
  BLI_assert(this->is_mutable() || this->is_expired());
  if (rigid_body_world_ != nullptr) {
    if (ownership_ == GeometryOwnershipType::Owned) {
      // TODO
      // BKE_id_free(nullptr, rigid_body_world_);
    }
    rigid_body_world_ = nullptr;
  }
}

bool RigidBodyComponent::has_world() const
{
  return rigid_body_world_ != nullptr;
}

void RigidBodyComponent::replace(RigidBodyWorld *rigid_body_world, GeometryOwnershipType ownership)
{
  BLI_assert(this->is_mutable());
  this->clear();
  rigid_body_world_ = rigid_body_world;
  ownership_ = ownership;
}

RigidBodyWorld *RigidBodyComponent::release()
{
  BLI_assert(this->is_mutable());
  RigidBodyWorld *rigid_body_world = rigid_body_world_;
  rigid_body_world_ = nullptr;
  return rigid_body_world;
}

const RigidBodyWorld *RigidBodyComponent::get() const
{
  return rigid_body_world_;
}

RigidBodyWorld *RigidBodyComponent::get_for_write()
{
  BLI_assert(this->is_mutable());
  if (ownership_ == GeometryOwnershipType::ReadOnly) {
    // TODO
    // rigid_body_world_ = BKE_rigid_body_world_copy_for_eval(rigid_body_world_);
    ownership_ = GeometryOwnershipType::Owned;
  }
  return rigid_body_world_;
}

bool RigidBodyComponent::is_empty() const
{
  return rigid_body_world_ == nullptr;
}

int RigidBodyComponent::bodies_num() const
{
  return rigid_body_world_ ? rigid_body_world_->bodies_num() : 0;
}

int RigidBodyComponent::constraints_num() const
{
  return rigid_body_world_ ? rigid_body_world_->constraints_num() : 0;
}

int RigidBodyComponent::shapes_num() const
{
  return rigid_body_world_ ? rigid_body_world_->shapes_num() : 0;
}

bool RigidBodyComponent::owns_direct_data() const
{
  return ownership_ == GeometryOwnershipType::Owned;
}

void RigidBodyComponent::ensure_owns_direct_data()
{
  BLI_assert(this->is_mutable());
  if (ownership_ != GeometryOwnershipType::Owned) {
    if (rigid_body_world_) {
      // TODO
      // rigid_body_world_ = BKE_rigid_body_world_copy_for_eval(rigid_body_world_);
    }
    ownership_ = GeometryOwnershipType::Owned;
  }
}

static ComponentAttributeProviders create_attribute_providers_for_rigid_body_world()
{
  // TODO
  // static CustomDataAccessInfo layers_access = {
  //    [](void *owner) -> CustomData * {
  //      RigidBodyWorld &rigid_body_world = *static_cast<RigidBodyWorld *>(owner);
  //      return &rigid_body_world.layers_data;
  //    },
  //    [](const void *owner) -> const CustomData * {
  //      const RigidBodyWorld &rigid_body_world = *static_cast<const RigidBodyWorld *>(owner);
  //      return &rigid_body_world.layers_data;
  //    },
  //    [](const void *owner) -> int {
  //      const RigidBodyWorld &rigid_body_world = *static_cast<const RigidBodyWorld *>(owner);
  //      return rigid_body_world.layers().size();
  //    }};

  // static CustomDataAttributeProvider layer_custom_data(AttrDomain::Layer, layers_access);

  // return ComponentAttributeProviders({}, {&layer_custom_data});
  return ComponentAttributeProviders({}, {});
}

static GVArray adapt_rigid_body_attribute_domain(const RigidBodyWorld & /*rigid_body_world*/,
                                                 const GVArray &varray,
                                                 const AttrDomain from,
                                                 const AttrDomain to)
{
  if (from == to) {
    return varray;
  }
  return {};
}

static AttributeAccessorFunctions get_rigid_body_world_accessor_functions()
{
  static const ComponentAttributeProviders providers =
      create_attribute_providers_for_rigid_body_world();
  AttributeAccessorFunctions fn =
      attribute_accessor_functions::accessor_functions_for_providers<providers>();
  fn.domain_size = [](const void *owner, const AttrDomain domain) {
    if (owner == nullptr) {
      return 0;
    }
    const RigidBodyWorld &rigid_body_world = *static_cast<const RigidBodyWorld *>(owner);
    switch (domain) {
      case AttrDomain::Point:
        // TODO
        // return int(rigid_body_world.bodies().size());
        return 0;
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
    const RigidBodyWorld &rigid_body_world = *static_cast<const RigidBodyWorld *>(owner);
    return adapt_rigid_body_attribute_domain(rigid_body_world, varray, from_domain, to_domain);
  };
  return fn;
}

static const AttributeAccessorFunctions &get_rigid_body_world_accessor_functions_ref()
{
  static const AttributeAccessorFunctions fn = get_rigid_body_world_accessor_functions();
  return fn;
}

}  // namespace blender::bke

namespace blender::bke {

std::optional<AttributeAccessor> RigidBodyComponent::attributes() const
{
  return AttributeAccessor(rigid_body_world_, get_rigid_body_world_accessor_functions_ref());
}

std::optional<MutableAttributeAccessor> RigidBodyComponent::attributes_for_write()
{
  RigidBodyWorld *rigid_body_world = this->get_for_write();
  return MutableAttributeAccessor(rigid_body_world, get_rigid_body_world_accessor_functions_ref());
}

}  // namespace blender::bke
