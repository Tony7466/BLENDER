/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <mutex>

#include "BLI_hash.hh"
#include "BLI_linear_allocator.hh"
#include "BLI_map.hh"
#include "BLI_set.hh"
#include "BLI_string_ref.hh"

#include "DNA_ID.h"
#include "DNA_ID_enums.h"

namespace blender::bke::bake {

struct BakeIDMappingKey {
  StringRef id_name;
  StringRef lib_name;

  uint64_t hash() const
  {
    return get_default_hash_2(this->id_name, this->lib_name);
  }

  friend bool operator==(const BakeIDMappingKey &a, const BakeIDMappingKey &b)
  {
    return a.id_name == b.id_name && a.lib_name == b.lib_name;
  }
};

class BakeIDMapping {
 public:
  virtual ID *get(const BakeIDMappingKey &key, const ID_Type type) const = 0;

  /**
   * Try to add an ID to the mapping. This may not succeed in all cases. The method is thread-safe.
   */
  virtual void add(const ID &id) = 0;
};

}  // namespace blender::bke::bake
