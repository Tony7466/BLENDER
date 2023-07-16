/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_geometry_set.hh"
#include "attribute_access_intern.hh"

namespace blender::bke {

/* -------------------------------------------------------------------- */
/** \name Geometry Component Implementation
 * \{ */

GizmosComponent::GizmosComponent() : GeometryComponent(GeometryComponent::Type::Gizmos) {}

GizmosComponent::~GizmosComponent()
{
  std::cout << ">> " << __func__ << ": " << this << ";\n";
  this->clear();
}

GeometryComponent *GizmosComponent::copy() const
{
  std::cout << ">> " << __func__ << ": " << this << ";\n";
  GizmosComponent *new_component = new GizmosComponent();
  if (gizmos_ != nullptr) {
    new_component->gizmos_ = gizmos_->copy();
    new_component->ownership_ = GeometryOwnershipType::Owned;
  }
  return new_component;
}

void GizmosComponent::clear()
{
  std::cout << ">> " << __func__ << ": " << this << ";\n";
  BLI_assert(this->is_mutable() || this->is_expired());
  if (gizmos_ != nullptr) {
    if (ownership_ == GeometryOwnershipType::Owned) {
      delete gizmos_;
    }
    gizmos_ = nullptr;
  }
}
void GizmosComponent::replace(GizmosGeometry *gizmos, GeometryOwnershipType ownership)
{
  std::cout << ">> " << __func__ << ": " << this << " >>: " << gizmos << ";\n";
  BLI_assert(this->is_mutable());
  this->clear();
  gizmos_ = gizmos;
  ownership_ = ownership;
}

bool GizmosComponent::has_gizmos() const
{
  std::cout << ">> " << __func__ << ": " << this << ";\n";
  return gizmos_ != nullptr;
}

const GizmosGeometry *GizmosComponent::get_for_read() const
{
  std::cout << ">> " << __func__ << ": " << this << ";\n";
  return gizmos_;
}

GizmosGeometry *GizmosComponent::get_for_write()
{
  std::cout << ">> " << __func__ << ": " << this << ";\n";
  BLI_assert(this->is_mutable());
  if (ownership_ == GeometryOwnershipType::ReadOnly) {
    gizmos_ = gizmos_->copy();
    ownership_ = GeometryOwnershipType::Owned;
  }
  return gizmos_;
}

bool GizmosComponent::is_empty() const
{
  std::cout << ">> " << __func__ << ": " << this << ";\n";
  return gizmos_ == nullptr;
}

bool GizmosComponent::owns_direct_data() const
{
  std::cout << ">> " << __func__ << ": " << this << ";\n";
  return ownership_ == GeometryOwnershipType::Owned;
}

void GizmosComponent::ensure_owns_direct_data()
{
  std::cout << ">> " << __func__ << ": " << this << ";\n";
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
                       const GVArray &varray,
                       const eAttrDomain from_domain,
                       const eAttrDomain to_domain) {
    if (from_domain == to_domain && from_domain == ATTR_DOMAIN_POINT) {
      return varray;
    }
    return GVArray{};
  };
  return fn;
}

static const AttributeAccessorFunctions &get_gizmos_accessor_functions_ref()
{
  static const AttributeAccessorFunctions fn = get_gizmos_accessor_functions();
  return fn;
}

AttributeAccessor GizmosGeometry::attributes() const
{
  return AttributeAccessor(this, get_gizmos_accessor_functions_ref());
}

MutableAttributeAccessor GizmosGeometry::attributes_for_write()
{
  return MutableAttributeAccessor(this, get_gizmos_accessor_functions_ref());
}

std::optional<AttributeAccessor> GizmosComponent::attributes() const
{
  return AttributeAccessor(gizmos_, get_gizmos_accessor_functions_ref());
}

std::optional<MutableAttributeAccessor> GizmosComponent::attributes_for_write()
{
  GizmosGeometry *gizmos = this->get_for_write();
  return MutableAttributeAccessor(gizmos, get_gizmos_accessor_functions_ref());
}

}  // namespace blender::bke

/** \} */
