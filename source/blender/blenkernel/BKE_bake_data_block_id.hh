/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bke
 */

#include "BLI_string_ref.hh"
#include "BLI_struct_equality_utils.hh"
#include "BLI_vector.hh"

struct ID;

namespace blender::bke::bake {

struct BakeDataBlockID {
  std::string id_name;
  std::string lib_name;

  BakeDataBlockID() = default;
  BakeDataBlockID(std::string id_name, std::string lib_name);
  BakeDataBlockID(const ID &id);

  uint64_t hash() const;

  BLI_STRUCT_EQUALITY_OPERATORS_2(BakeDataBlockID, id_name, lib_name)
};

struct BakeMaterialsList : public Vector<std::optional<BakeDataBlockID>> {};

}  // namespace blender::bke::bake
