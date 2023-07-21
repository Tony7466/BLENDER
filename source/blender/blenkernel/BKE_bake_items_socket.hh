/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_function_ref.hh"

#include "DNA_node_types.h"

#include "BKE_bake_items.hh"
#include "BKE_geometry_fields.hh"

namespace blender::bke {

struct BakeSocketConfig {
  Vector<eNodeSocketDatatype> types;
  Vector<eAttrDomain> domains;
  Vector<Vector<int, 1>> geometries_by_attribute;
};

Vector<std::unique_ptr<BakeItem>> move_socket_values_to_bake_items(Span<void *> socket_values,
                                                                   const BakeSocketConfig &config);

void move_bake_items_to_socket_values(
    Span<BakeItem *> bake_items,
    const BakeSocketConfig &config,
    FunctionRef<std::shared_ptr<AnonymousAttributeFieldInput>(int, const CPPType &)>
        make_attribute_field,
    Span<void *> r_socket_values);

void copy_bake_items_to_socket_values(
    Span<const BakeItem *> bake_items,
    const BakeSocketConfig &config,
    FunctionRef<std::shared_ptr<AnonymousAttributeFieldInput>(int, const CPPType &)>
        make_attribute_field,
    Span<void *> r_socket_values);

}  // namespace blender::bke
