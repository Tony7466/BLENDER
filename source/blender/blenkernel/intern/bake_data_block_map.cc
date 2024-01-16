/* SPDX-FileCopyrightText: 2005 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <ostream>

#include "BLI_hash.hh"

#include "BKE_bake_data_block_map.hh"

#include "DNA_ID.h"

namespace blender::bke::bake {

BakeDataBlockID::BakeDataBlockID(ID_Type type, std::string id_name, std::string lib_name)
    : type(type), id_name(std::move(id_name)), lib_name(std::move(lib_name))
{
}

BakeDataBlockID::BakeDataBlockID(const ID &id)
{
  this->type = GS(id.name);
  this->id_name = id.name + 2;
  if (id.lib) {
    this->lib_name = id.lib->id.name + 2;
  }
}

std::ostream &operator<<(std::ostream &stream, const BakeDataBlockID &id)
{
  return stream << "(" << id.id_name << ", Lib: " << id.lib_name << ")";
}

uint64_t BakeDataBlockID::hash() const
{
  return get_default_hash_3(this->type, this->id_name, this->lib_name);
}

}  // namespace blender::bke::bake
