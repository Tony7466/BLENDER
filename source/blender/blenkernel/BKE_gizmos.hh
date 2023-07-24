/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_math_quaternion_types.hh"
#include "BLI_vector_set.hh"

#include "BKE_attribute.hh"

#include "DNA_node_types.h"

namespace blender::bke {

class GizmosGeometry {
 private:
  VectorSet<std::string> paths_;
  Array<int> mapping;

  CustomDataAttributes attributes_;

 public:
  GizmosGeometry() = default;
  GizmosGeometry(std::string path);
  GizmosGeometry(int gizmos, VectorSet<std::string> paths);

  GizmosGeometry *copy() const;

  void remove_unused_pathes();

  int pathes_num() const;
  int gizmos_num() const;

  const Span<std::string> pathes() const
  {
    return paths_;
  }

  Span<int> paths_mapping() const
  {
    return mapping;
  }

  MutableSpan<int> mapping_for_write()
  {
    return mapping;
  }

  Span<float3> positions() const;
  MutableSpan<float3> positions_for_write();

  Span<math::Quaternion> rotations() const;
  MutableSpan<math::Quaternion> rotations_for_write();

  Span<float3> sizes() const;
  MutableSpan<float3> sizes_for_write();

  /**
   * Remove the indices that are not contained in the mask input, and remove unused pathes
   * afterwards.
   */
  void remove(const blender::IndexMask &mask,
              const blender::bke::AnonymousAttributePropagationInfo &propagation_info);

  blender::bke::AttributeAccessor attributes() const;
  blender::bke::MutableAttributeAccessor attributes_for_write();

  CustomDataAttributes &custom_data_attributes();
  const CustomDataAttributes &custom_data_attributes() const;
};

inline CustomDataAttributes &GizmosGeometry::custom_data_attributes()
{
  return attributes_;
}

inline const CustomDataAttributes &GizmosGeometry::custom_data_attributes() const
{
  return attributes_;
}

}  // namespace blender::bke
