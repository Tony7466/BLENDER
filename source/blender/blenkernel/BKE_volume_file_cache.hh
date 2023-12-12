/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <mutex>
#include <string>

#include "BLI_function_ref.hh"
#include "BLI_hash.hh"
#include "BLI_map.hh"

#include "BKE_volume_grid_ptr.hh"

/** \file
 * \ingroup bke
 */

namespace blender::bke {

struct VolumeGrid;

/* -------------------------------------------------------------------- */
/** \name Volume File Cache
 *
 * Global cache of grids read from VDB files. This is used for sharing grids
 * between multiple volume datablocks with the same filepath, and sharing grids
 * between original and copy-on-write datablocks created by the depsgraph.
 * \{ */

/**
 * Identifier for grids in a VDB file used by the cache.
 */
struct VolumeFileCacheKey {
  std::string filepath;
  std::string name;
  int simplify_level;

  VolumeFileCacheKey(StringRef filepath, StringRef name, int simplify_level = 0)
      : filepath(std::move(filepath)),
        name(std::move(name)),
        simplify_level(std::move(simplify_level))
  {
  }
  VolumeFileCacheKey(const VolumeFileCacheKey &other)
      : filepath(other.filepath), name(other.name), simplify_level(other.simplify_level)
  {
  }
  VolumeFileCacheKey(VolumeFileCacheKey &&other)
      : filepath(std::move(other.filepath)),
        name(std::move(other.name)),
        simplify_level(std::move(other.simplify_level))
  {
  }

  inline bool is_valid() const
  {
    return !this->filepath.empty() && !this->name.empty();
  }

  inline operator bool() const
  {
    return this->is_valid();
  }

  BLI_STRUCT_EQUALITY_OPERATORS_3(VolumeFileCacheKey, filepath, name, simplify_level)

  uint64_t hash() const
  {
    return get_default_hash_3(this->filepath, this->name, this->simplify_level);
  }
};

/**
 * Cache for volume grids loaded from VDB files and their simplification levels.
 */
class VolumeFileCache {
 private:
  /* Mutex for multithreaded access. */
  std::mutex mutex_;
  /* Registry of files loaded from grids. */
  Map<VolumeFileCacheKey, const GVolumeGridPtr> file_grids_;

 public:
  /* Get the global file cache instance. */
  static VolumeFileCache &get();

  bool has_grid(const VolumeFileCacheKey &key) const;
  GVolumeGridPtr get_grid(const VolumeFileCacheKey &key) const;
  /* Returns the original grid or a simplified version depending on the given #simplify_level. */
  GVolumeGridPtr get_simplified_grid(const VolumeFileCacheKey &key, const int simplify_level);
  bool add_grid(const VolumeFileCacheKey &key, const GVolumeGridPtr &grid);
  void remove_grid(const VolumeFileCacheKey &key);

  void unload_unused_grids() const;
};

/** \} */

}  // namespace blender::bke
