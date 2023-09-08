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

namespace blender::bke {

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

struct BakeIDMapping {
  Map<BakeIDMappingKey, ID *> mappings;

  ID *get(const BakeIDMappingKey &key, const ID_Type type) const
  {
    ID *id = this->mappings.lookup_default(key, nullptr);
    if (id == nullptr) {
      return nullptr;
    }
    if (GS(id->name) != type) {
      return nullptr;
    }
    return id;
  }
};

struct BakeIDMappingIssuesLog {
  std::mutex mutex;
  LinearAllocator<> allocator;
  Set<std::pair<BakeIDMappingKey, ID_Type>> missing_mappings;

  void add(const BakeIDMappingKey &key, ID_Type type)
  {
    std::lock_guard lock{this->mutex};
    missing_mappings.add(
        {{this->allocator.copy_string(key.id_name), this->allocator.copy_string(key.lib_name)},
         type});
  }
};

}  // namespace blender::bke
