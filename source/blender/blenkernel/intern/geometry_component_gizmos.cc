/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_geometry_set.hh"
#include "attribute_access_intern.hh"

/* -------------------------------------------------------------------- */
/** \name Geometry Component Implementation
 * \{ */

GizmosComponent::GizmosComponent() : GeometryComponent(GEO_COMPONENT_TYPE_GIZMO) {}

GizmosComponent::~GizmosComponent()
{
  this->clear();
}

GeometryComponent *GizmosComponent::copy() const
{
  GizmosComponent *new_component = new GizmosComponent();
  if (gizmos_ != nullptr) {
    new_component->gizmos_ = gizmos_->copy();
    new_component->ownership_ = GeometryOwnershipType::Owned;
  }
  return new_component;
}

void GizmosComponent::clear()
{
  BLI_assert(this->is_mutable() || this->is_expired());
  if (gizmos_ != nullptr) {
    if (ownership_ == GeometryOwnershipType::Owned) {
      delete gizmos_;
    }
    gizmos_ = nullptr;
  }
}
void GizmosComponent::replace(blender::bke::GizmosGeometry *gizmos,
                              GeometryOwnershipType ownership)
{
  BLI_assert(this->is_mutable());
  this->clear();
  gizmos_ = gizmos;
  ownership_ = ownership;
}

bool GizmosComponent::has_gizmos() const
{
  return gizmos_ != nullptr;
}

const blender::bke::GizmosGeometry *GizmosComponent::get_for_read() const
{
  return gizmos_;
}

blender::bke::GizmosGeometry *GizmosComponent::get_for_write()
{
  BLI_assert(this->is_mutable());
  if (ownership_ == GeometryOwnershipType::ReadOnly) {
    gizmos_ = gizmos_->copy();
    ownership_ = GeometryOwnershipType::Owned;
  }
  return gizmos_;
}

bool GizmosComponent::is_empty() const
{
  return gizmos_ == nullptr;
}

bool GizmosComponent::owns_direct_data() const
{
  return ownership_ == GeometryOwnershipType::Owned;
}

void GizmosComponent::ensure_owns_direct_data()
{
  BLI_assert(this->is_mutable());
  if (ownership_ != GeometryOwnershipType::Owned) {
    gizmos_ = gizmos_->copy();
    ownership_ = GeometryOwnershipType::Owned;
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Attribute Access
 * \{ */

namespace blender::bke {

static ComponentAttributeProviders create_attribute_providers_for_gizmos()
{
  static CustomDataAccessInfo gizmos_access = {
      [](void *owner) -> CustomData * {
        GizmosGeometry *gizmos = static_cast<GizmosGeometry *>(owner);
        return &gizmos->custom_data_attributes().data;
      },
      [](const void *owner) -> const CustomData * {
        const GizmosGeometry *gizmos = static_cast<const GizmosGeometry *>(owner);
        return &gizmos->custom_data_attributes().data;
      },
      [](const void *owner) -> int {
        const GizmosGeometry *gizmos = static_cast<const GizmosGeometry *>(owner);
        return gizmos->gizmos_num();
      }};

  static BuiltinCustomDataLayerProvider position("position",
                                                 ATTR_DOMAIN_POINT,
                                                 CD_PROP_FLOAT3,
                                                 CD_PROP_FLOAT3,
                                                 BuiltinAttributeProvider::Creatable,
                                                 BuiltinAttributeProvider::NonDeletable,
                                                 gizmos_access,
                                                 nullptr);
  static BuiltinCustomDataLayerProvider id("id",
                                           ATTR_DOMAIN_POINT,
                                           CD_PROP_INT32,
                                           CD_PROP_INT32,
                                           BuiltinAttributeProvider::Creatable,
                                           BuiltinAttributeProvider::Deletable,
                                           gizmos_access,
                                           nullptr);
  static CustomDataAttributeProvider gizmos_custom_data(ATTR_DOMAIN_POINT, gizmos_access);
  return ComponentAttributeProviders({&position, &id}, {&gizmos_custom_data});
}

static AttributeAccessorFunctions get_gizmos_accessor_functions()
{
  static const ComponentAttributeProviders providers = create_attribute_providers_for_gizmos();
  AttributeAccessorFunctions fn =
      attribute_accessor_functions::accessor_functions_for_providers<providers>();
  fn.domain_size = [](const void *owner, const eAttrDomain domain) {
    if (owner == nullptr) {
      return 0;
    }
    const GizmosGeometry &gizmos = *static_cast<const GizmosGeometry *>(owner);
    switch (domain) {
      case ATTR_DOMAIN_POINT:
        return gizmos.gizmos_num();
      default:
        return 0;
    }
  };
  fn.domain_supported = [](const void * /*owner*/, const eAttrDomain domain) {
    return domain == ATTR_DOMAIN_POINT;
  };
  fn.adapt_domain = [](const void * /*owner*/,
                       const blender::GVArray &varray,
                       const eAttrDomain from_domain,
                       const eAttrDomain to_domain) {
    if (from_domain == to_domain && from_domain == ATTR_DOMAIN_POINT) {
      return varray;
    }
    return blender::GVArray{};
  };
  return fn;
}

static const AttributeAccessorFunctions &get_gizmos_accessor_functions_ref()
{
  static const AttributeAccessorFunctions fn = get_gizmos_accessor_functions();
  return fn;
}

}  // namespace blender::bke

blender::bke::AttributeAccessor blender::bke::GizmosGeometry::attributes() const
{
  return blender::bke::AttributeAccessor(this, blender::bke::get_gizmos_accessor_functions_ref());
}

blender::bke::MutableAttributeAccessor blender::bke::GizmosGeometry::attributes_for_write()
{
  return blender::bke::MutableAttributeAccessor(this,
                                                blender::bke::get_gizmos_accessor_functions_ref());
}

std::optional<blender::bke::AttributeAccessor> GizmosComponent::attributes() const
{
  return blender::bke::AttributeAccessor(gizmos_,
                                         blender::bke::get_gizmos_accessor_functions_ref());
}

std::optional<blender::bke::MutableAttributeAccessor> GizmosComponent::attributes_for_write()
{
  blender::bke::GizmosGeometry *gizmos = this->get_for_write();
  return blender::bke::MutableAttributeAccessor(gizmos,
                                                blender::bke::get_gizmos_accessor_functions_ref());
}

/** \} */
