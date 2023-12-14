/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bke
 */

#include "BKE_volume_grid.hh"

#include "BLI_hash.hh"

namespace blender::bke::volume_file_cache {

struct CacheKey {
  std::string file_path;
  std::string grid_name;
  int simplify_level = 0;

  BLI_STRUCT_EQUALITY_OPERATORS_3(CacheKey, file_path, grid_name, simplify_level)

  uint64_t hash() const
  {
    return get_default_hash_3(this->file_path, this->grid_name, this->simplify_level);
  }
};

GVolumeGrid get(const CacheKey &key);

void unload_unused();

}  // namespace blender::bke::volume_file_cache
