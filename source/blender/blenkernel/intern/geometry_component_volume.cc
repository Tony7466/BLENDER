/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

// !!! DEBUGGING !!!
// #define WITH_OPENVDB

#include <cstdint>

#include "DNA_volume_types.h"

#include "BKE_attribute.h"
#include "BKE_geometry_set.hh"
#include "BKE_lib_id.h"
#include "BKE_volume.h"
#include "BKE_volume_geometry.hh"

#include "attribute_access_volume.hh"

#ifdef WITH_OPENVDB
#  include <openvdb/Grid.h>
#  include <openvdb/openvdb.h>
#endif

namespace blender::bke {

/* -------------------------------------------------------------------- */
/** \name Geometry Component Implementation
 * \{ */

VolumeComponent::VolumeComponent() : GeometryComponent(GeometryComponent::Type::Volume) {}

VolumeComponent::~VolumeComponent()
{
  clear_volume();
}

GeometryComponent *VolumeComponent::copy() const
{
  VolumeComponent *new_component = new VolumeComponent();
  if (volume_ != nullptr) {
    new_component->volume_ = BKE_volume_copy_for_eval(volume_);
    new_component->ownership_ = GeometryOwnershipType::Owned;
  }
  return new_component;
}

void VolumeComponent::clear_volume()
{
  BLI_assert(this->is_mutable() || this->is_expired());
  if (volume_ != nullptr) {
    if (ownership_ == GeometryOwnershipType::Owned) {
      BKE_id_free(nullptr, volume_);
    }
    volume_ = nullptr;
  }
}

void VolumeComponent::clear()
{
  clear_volume();
}

bool VolumeComponent::has_volume() const
{
  return volume_ != nullptr;
}

void VolumeComponent::replace(Volume *volume, GeometryOwnershipType ownership)
{
  BLI_assert(this->is_mutable());
  this->clear();
  volume_ = volume;
  ownership_ = ownership;
}

Volume *VolumeComponent::release()
{
  BLI_assert(this->is_mutable());
  Volume *volume = volume_;
  volume_ = nullptr;
  return volume;
}

const Volume *VolumeComponent::get_for_read() const
{
  return volume_;
}

Volume *VolumeComponent::get_for_write()
{
  BLI_assert(this->is_mutable());
  if (ownership_ == GeometryOwnershipType::ReadOnly) {
    volume_ = BKE_volume_copy_for_eval(volume_);
    ownership_ = GeometryOwnershipType::Owned;
  }
  return volume_;
}

bool VolumeComponent::owns_direct_data() const
{
  return ownership_ == GeometryOwnershipType::Owned;
}

void VolumeComponent::ensure_owns_direct_data()
{
  BLI_assert(this->is_mutable());
  if (ownership_ != GeometryOwnershipType::Owned) {
    volume_ = BKE_volume_copy_for_eval(volume_);
    ownership_ = GeometryOwnershipType::Owned;
  }
}

#ifdef WITH_OPENVDB

/* -------------------------------------------------------------------- */
/** \name Attribute Access Helper Functions
 * \{ */

// static void tag_component_radii_changed(void *owner)
//{
//   CurvesGeometry &curves = *static_cast<CurvesGeometry *>(owner);
//   curves.tag_radii_changed();
// }

/** \} */

/**
 * In this function all the attribute providers for a volume component are created.
 * Most data in this function is statically allocated, because it does not change over time.
 */
static ComponentAttributeProviders create_attribute_providers_for_volume()
{
  static VolumeGridAccessInfo grid_access = {
      [](void *owner) -> VolumeGeometryGrid & {
        return *static_cast<VolumeGeometryGrid *>(owner);
      },
      [](const void *owner) -> const VolumeGeometryGrid & {
        return *static_cast<const VolumeGeometryGrid *>(owner);
      },
  };

  static auto update_on_change = [](void * /*owner*/) {};

  // static BuiltinVolumeAttributeProvider position("position",
  //                                                ATTR_DOMAIN_POINT,
  //                                                CD_PROP_FLOAT3,
  //                                                BuiltinAttributeProvider::NonCreatable,
  //                                                BuiltinAttributeProvider::NonDeletable,
  //                                                grid_access,
  //                                                update_on_change);

  // static VolumeAttributeProvider voxel_custom_data(ATTR_DOMAIN_POINT, grid_access);

  // return ComponentAttributeProviders({&position}, {&voxel_custom_data});
  return ComponentAttributeProviders({}, {});
}

static AttributeAccessorFunctions get_volume_accessor_functions()
{
  static const ComponentAttributeProviders providers = create_attribute_providers_for_volume();
  AttributeAccessorFunctions fn =
      attribute_accessor_functions::accessor_functions_for_providers<providers>();
  /* Set domain callbacks that are not defined yet. */
  fn.domain_size = [](const void *owner, const eAttrDomain domain) {
    if (owner == nullptr) {
      return 0;
    }
    const VolumeGeometry &geometry = *static_cast<const VolumeGeometry *>(owner);
    return geometry.domain_size(domain);
  };
  fn.domain_supported = [](const void * /*owner*/, const eAttrDomain domain) {
    return ELEM(domain, ATTR_DOMAIN_POINT);
  };
  fn.adapt_domain = [](const void *owner,
                       const GVArray &varray,
                       const eAttrDomain from_domain,
                       const eAttrDomain to_domain) -> GVArray {
    if (owner == nullptr) {
      return {};
    }
    const VolumeGeometry &geometry = *static_cast<const VolumeGeometry *>(owner);
    return geometry.adapt_domain(varray, from_domain, to_domain);
  };
  return fn;
}

static const AttributeAccessorFunctions &get_volume_accessor_functions_ref()
{
  static const AttributeAccessorFunctions fn = get_volume_accessor_functions();
  return fn;
}

std::optional<AttributeAccessor> VolumeComponent::attributes() const
{
  return AttributeAccessor(volume_ ? &volume_->geometry.grid : nullptr,
                           blender::bke::get_volume_accessor_functions_ref());
}

std::optional<MutableAttributeAccessor> VolumeComponent::attributes_for_write()
{
  Volume *volume = this->get_for_write();
  return MutableAttributeAccessor(volume ? &volume->geometry.grid : nullptr,
                                  blender::bke::get_volume_accessor_functions_ref());
}

#else

std::optional<AttributeAccessor> VolumeComponent::attributes() const
{
  return {};
}

std::optional<MutableAttributeAccessor> VolumeComponent::attributes_for_write()
{
  return {};
}

#endif  // WITH_OPENVDB

}  // namespace blender::bke
