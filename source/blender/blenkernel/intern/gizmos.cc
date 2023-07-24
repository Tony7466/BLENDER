/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_gizmos.hh"

namespace blender::bke {

GizmosGeometry::GizmosGeometry(std::string path) : paths_{std::move(path)}, mapping(1, 0)
{
  std::cout << ">> " << __func__ << ": " << this << ";\n";
  attributes_.reallocate(1);
  attributes_.create("position", CD_PROP_FLOAT3);
  this->positions_for_write().fill(float3());
  attributes_.create("rotation", CD_PROP_QUATERNION);
  this->rotations_for_write().fill(math::Quaternion::identity());
  attributes_.create("size", CD_PROP_FLOAT3);
  this->sizes_for_write().fill(float3(1.0f));
}

GizmosGeometry::GizmosGeometry(const int gizmos, VectorSet<std::string> paths)
    : paths_{std::move(paths)}, mapping(gizmos, 0)
{
  std::cout << ">> " << __func__ << ": " << this << ";\n";
  attributes_.reallocate(gizmos);
  attributes_.create("position", CD_PROP_FLOAT3);
  this->positions_for_write().fill(float3());
  attributes_.create("rotation", CD_PROP_QUATERNION);
  this->rotations_for_write().fill(math::Quaternion::identity());
  attributes_.create("size", CD_PROP_FLOAT3);
  this->sizes_for_write().fill(float3(1.0f));
}

GizmosGeometry *GizmosGeometry::copy() const
{
  std::cout << ">> " << __func__ << ": " << this << ";\n";
  GizmosGeometry *gizmos = new GizmosGeometry;
  gizmos->paths_ = paths_;
  gizmos->mapping = mapping;
  gizmos->attributes_ = attributes_;
  return gizmos;
}

int GizmosGeometry::pathes_num() const
{
  return paths_.size();
}

int GizmosGeometry::gizmos_num() const
{
  return mapping.size();
}

Span<float3> GizmosGeometry::positions() const
{
  const bke::CustomDataAttributes attributes = this->custom_data_attributes();
  const VArray<float3> positions = attributes.get_for_read<float3>("position", float3());
  return positions.get_internal_span();
}

MutableSpan<float3> GizmosGeometry::positions_for_write()
{
  bke::CustomDataAttributes attributes = this->custom_data_attributes();
  GMutableSpan positions = *attributes.get_for_write("position");
  return positions.typed<float3>();
}

Span<math::Quaternion> GizmosGeometry::rotations() const
{
  const bke::CustomDataAttributes attributes = this->custom_data_attributes();
  const VArray<math::Quaternion> rotations = attributes.get_for_read<math::Quaternion>(
      "rotation", math::Quaternion::identity());
  return rotations.get_internal_span();
}

MutableSpan<math::Quaternion> GizmosGeometry::rotations_for_write()
{
  bke::CustomDataAttributes attributes = this->custom_data_attributes();
  GMutableSpan rotations = *attributes.get_for_write("rotation");
  return rotations.typed<math::Quaternion>();
}

Span<float3> GizmosGeometry::sizes() const
{
  const bke::CustomDataAttributes attributes = this->custom_data_attributes();
  const VArray<float3> sizes = attributes.get_for_read<float3>("size", float3(1.0f));
  return sizes.get_internal_span();
}

MutableSpan<float3> GizmosGeometry::sizes_for_write()
{
  bke::CustomDataAttributes attributes = this->custom_data_attributes();
  GMutableSpan sizes = *attributes.get_for_write("size");
  return sizes.typed<float3>();
}

}  // namespace blender::bke
