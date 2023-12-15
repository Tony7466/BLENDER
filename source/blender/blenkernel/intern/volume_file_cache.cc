/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_volume_file_cache.hh"
#include "BKE_volume_grid.hh"
#include "BKE_volume_openvdb.hh"

#include "CLG_log.h"

// static CLG_LogRef LOG = {"bke.volume"};

namespace blender::bke {

VolumeFileCache &VolumeFileCache::get()
{
  static VolumeFileCache global_cache = VolumeFileCache{};
  return global_cache;
}

bool VolumeFileCache::has_grid(const VolumeFileCacheKey &key) const
{
  return file_grids_.contains(key);
}

GVolumeGridPtr VolumeFileCache::get_grid(const VolumeFileCacheKey &key) const
{
  return file_grids_.lookup_default(key, nullptr);
}

bool VolumeFileCache::add_grid(const VolumeFileCacheKey &key, const GVolumeGridPtr &grid)
{
  BLI_assert(key.is_valid());
  std::lock_guard<std::mutex> lock(mutex_);
  return file_grids_.add(key, grid);
}

void VolumeFileCache::remove_grid(const VolumeFileCacheKey &key)
{
  std::lock_guard<std::mutex> lock(mutex_);
  file_grids_.remove(key);
}

void VolumeFileCache::unload_unused_grids() const
{
  /* TODO */
  BLI_assert_unreachable();
}

/* Returns the original grid or a simplified version depending on the given #simplify_level. */
GVolumeGridPtr VolumeFileCache::get_simplified_grid(const VolumeFileCacheKey &key,
                                                    const int simplify_level)
{
  BLI_assert(simplify_level >= 0);

  std::lock_guard<std::mutex> lock(mutex_);
  const GVolumeGridPtr grid = file_grids_.lookup(key);
  if (key.simplify_level == simplify_level) {
    /* No change in simplify level. */
    return grid;
  }

  GVolumeGridPtr simple_grid;
  /* Isolate creating grid since that's multithreaded and we are
   * holding a mutex lock. */
  blender::threading::isolate_task([&] {
    const VolumeFileCacheKey simplified_key(key.filepath, key.name, simplify_level);
    simple_grid = file_grids_.lookup_or_add_cb(simplified_key, [&]() {
      const float resolution_factor = 1.0f / (1 << simplify_level);
      const VolumeGridType grid_type = BKE_volume_grid_type_openvdb(*grid->grid());
      const openvdb::GridBase::Ptr vdb_grid = BKE_volume_grid_create_with_changed_resolution(
          grid_type, *grid->grid(), resolution_factor);
      return make_volume_grid_ptr(vdb_grid, VOLUME_TREE_SOURCE_FILE_SIMPLIFIED);
    });
  });
  return simple_grid;
}

}  // namespace blender::bke
