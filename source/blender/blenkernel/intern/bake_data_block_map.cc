/* SPDX-FileCopyrightText: 2005 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_hash.hh"

#include "BKE_bake_data_block_map.hh"

#include "DNA_ID.h"

namespace blender::bke::bake {

BakeDataBlockID::BakeDataBlockID(std::string id_name, std::string lib_name)
    : id_name(std::move(id_name)), lib_name(std::move(lib_name))
{
}

BakeDataBlockID::BakeDataBlockID(const ID &id)
{
  this->id_name = id.name + 2;
  if (id.lib) {
    this->lib_name = id.lib->id.name + 2;
  }
}

uint64_t BakeDataBlockID::hash() const
{
  return get_default_hash_2(this->id_name, this->lib_name);
}

}  // namespace blender::bke::bake
