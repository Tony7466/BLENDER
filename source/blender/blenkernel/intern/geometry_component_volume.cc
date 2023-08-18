/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <cstdint>

#include "DNA_volume_types.h"

#include "BKE_attribute.h"
#include "BKE_geometry_set.hh"
#include "BKE_lib_id.h"
#include "BKE_volume.h"

#include "attribute_access_volume.hh"

#include "intern/volume_grids.hh"

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

const Volume *VolumeComponent::get() const
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

std::optional<AttributeAccessor> VolumeComponent::attributes() const
{
  return volume_ ? volume_->runtime.grids->attributes() : std::optional<AttributeAccessor>{};
}

std::optional<MutableAttributeAccessor> VolumeComponent::attributes_for_write()
{
  return volume_ ? volume_->runtime.grids->attributes_for_write() :
                   std::optional<MutableAttributeAccessor>{};
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
