/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_vector_set.hh"

#include "BKE_attribute.hh"

namespace blender::bke {

class GizmosGeometry {
 private:
  blender::VectorSet<std::string> paths_;
  blender::Array<int> mapping;

  CustomDataAttributes attributes_;

 public:
  GizmosGeometry() = default;
  GizmosGeometry(std::string);
  // GizmosGeometry(int pathes, int gizmos);
  // GizmosGeometry(const GizmosGeometry &other);

  GizmosGeometry *copy() const;

  void remove_unused_pathes();

  int pathes_num() const;
  int gizmos_num() const;

  const Span<std::string> pathes() const
  {
    return paths_;
  }

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
