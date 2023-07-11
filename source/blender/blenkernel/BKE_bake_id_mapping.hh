/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_hash.hh"
#include "BLI_set.hh"
#include "BLI_string_ref.hh"

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

struct BakeIDMappingIssuesLog {
  std::mutex mutex;
  Set<std::pair<BakeIDMappingKey, ID_Type>> missing_mappings;
};

}  // namespace blender::bke
