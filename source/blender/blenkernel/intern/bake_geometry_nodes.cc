/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_bake_geometry_nodes.hh"

namespace blender::bke {

GeometryBakeItem::GeometryBakeItem(GeometrySet geometry) : geometry(std::move(geometry)) {}

PrimitiveBakeItem::PrimitiveBakeItem(const CPPType &type, const void *value) : type_(type)
{
  value_ = MEM_mallocN_aligned(type.size(), type.alignment(), __func__);
  type.copy_construct(value, value_);
}

PrimitiveBakeItem::~PrimitiveBakeItem()
{
  type_.destruct(value_);
  MEM_freeN(value_);
}

StringBakeItem::StringBakeItem(std::string value) : value_(std::move(value)) {}

}  // namespace blender::bke
