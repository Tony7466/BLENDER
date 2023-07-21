/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_bake_items_socket.hh"

namespace blender::bke {

Vector<std::unique_ptr<BakeItem>> move_socket_values_to_bake_items(
    Span<void *> socket_values,
    Span<eNodeSocketDatatype> socket_types,
    Span<Vector<int>> geometries_by_attribute)
{
  UNUSED_VARS(socket_values, socket_types, geometries_by_attribute);
  return {};
}

void move_bake_items_to_socket_values(Span<BakeItem *> bake_items,
                                      Span<eNodeSocketDatatype> socket_types,
                                      Span<Vector<int>> geometries_by_attribute,
                                      Span<void *> r_socket_values)
{
  UNUSED_VARS(bake_items, socket_types, geometries_by_attribute, r_socket_values);
  return;
}

void copy_bake_items_to_socket_values(Span<const BakeItem *> bake_items,
                                      Span<eNodeSocketDatatype> socket_types,
                                      Span<Vector<int>> geometries_by_attribute,
                                      Span<void *> r_socket_values)
{
  UNUSED_VARS(bake_items, socket_types, geometries_by_attribute, r_socket_values);
  return;
}

}  // namespace blender::bke
