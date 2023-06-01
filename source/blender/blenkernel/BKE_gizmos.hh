/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BKE_attribute.hh"

namespace blender::bke {

class GizmosGeometry {
 private:
  blender::VectorSet<std::string> paths_;
  blender::Array<int> mapping;

  CustomDataAttributes attributes_;

 public:
  GizmosGeometry() = default;
  //GizmosGeometry(int pathes, int gizmos);
  //GizmosGeometry(const GizmosGeometry &other);

  GizmosGeometry *copy() const;

  void remove_unused_pathes();

  int pathes_num() const;
  int gizmos_num() const;

  /**
   * Remove the indices that are not contained in the mask input, and remove unused pathes afterwards.
   */
  void remove(const blender::IndexMask &mask,
              const blender::bke::AnonymousAttributePropagationInfo &propagation_info);

  blender::bke::AttributeAccessor attributes() const;
  blender::bke::MutableAttributeAccessor attributes_for_write();

  CustomDataAttributes &custom_data_attributes();
  const CustomDataAttributes &custom_data_attributes() const;
};

}  // namespace blender::bke
