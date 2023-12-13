/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_volume_file_cache2.hh"
#include "BKE_volume_openvdb.hh"

#include "BLI_map.hh"

#ifdef WITH_OPENVDB
#  include <openvdb/openvdb.h>
#endif

namespace blender::bke::volume_file_cache {

struct Cache {
  std::mutex mutex;
  Map<CacheKey, GVolumeGrid2> map;
};

static Cache &get_global_cache()
{
  static Cache cache;
  return cache;
}

static std::shared_ptr<openvdb::GridBase> load_grid_from_disk(const CacheKey &key)
{
  if (key.simplify_level == 0) {
    /* Disable delay loading and file copying, this has poor performance
     * on network drivers. */
    const bool delay_load = false;

    openvdb::io::File file(key.file_path);
    file.setCopyMaxBytes(0);
    file.open(delay_load);
    std::shared_ptr<openvdb::GridBase> grid = file.readGrid(key.grid_name);
    BLI_assert(grid);
    return grid;
  }

  const GVolumeGrid2 main_grid = volume_file_cache::get({key.file_path, key.grid_name, 0});
  const VolumeGridType grid_type = main_grid->grid_type();
  const float resolution_factor = 1.0f / (1 << key.simplify_level);
  std::shared_ptr<openvdb::GridBase> simplified_grid =
      BKE_volume_grid_create_with_changed_resolution(
          grid_type, main_grid->grid(), resolution_factor);
  return simplified_grid;
}

GVolumeGrid2 get(const CacheKey &key)
{
  Cache &cache = get_global_cache();
  std::lock_guard lock{cache.mutex};
  return cache.map.lookup_or_add_cb(key, [&]() {
    auto lazy_load_function = [key]() { return load_grid_from_disk(key); };
    auto *volume_grid = MEM_new<VolumeGridData>(__func__, std::move(lazy_load_function));
    return GVolumeGrid2(volume_grid);
  });
}

void unload_unused()
{
  Cache &cache = get_global_cache();
  std::lock_guard lock{cache.mutex};
  cache.map.remove_if(
      [](MapItem<CacheKey, GVolumeGrid2> item) { return item.value->is_mutable(); });
}

}  // namespace blender::bke::volume_file_cache
