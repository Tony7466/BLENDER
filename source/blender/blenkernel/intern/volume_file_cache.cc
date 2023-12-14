/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_volume_file_cache.hh"
#include "BKE_volume_openvdb.hh"

#include "BLI_map.hh"

#ifdef WITH_OPENVDB
#  include <openvdb/openvdb.h>
#endif

namespace blender::bke::volume_file_cache {

struct GridCacheKey {
  std::string file_path;
  std::string grid_name;
  int simplify_level = 0;

  BLI_STRUCT_EQUALITY_OPERATORS_3(GridCacheKey, file_path, grid_name, simplify_level)

  uint64_t hash() const
  {
    return get_default_hash_3(this->file_path, this->grid_name, this->simplify_level);
  }
};

struct GridCache {
  openvdb::GridBase::Ptr meta_data_grid;
  Map<int, GVolumeGrid> grid_by_simplify_level;
};

struct FileCache {
  std::string error_message;
  openvdb::MetaMap meta_data;
  Vector<GridCache> grids;

  GridCache *grid_cache_by_name(const StringRef name)
  {
    for (GridCache &grid_cache : this->grids) {
      if (grid_cache.meta_data_grid->getName() == name) {
        return &grid_cache;
      }
    }
    return nullptr;
  }
};

struct Cache {
  std::mutex mutex;
  Map<std::string, FileCache> file_map;
};

static Cache &get_global_cache()
{
  static Cache cache;
  return cache;
}

static FileCache create_file_cache(const StringRef file_path)
{
  FileCache file_cache;

  openvdb::io::File file(file_path);
  openvdb::GridPtrVec vdb_grids;
  try {
    /* Disable delay loading and file copying, this has poor performance
     * on network drives. */
    const bool delay_load = false;
    file.setCopyMaxBytes(0);
    file.open(delay_load);
    vdb_grids = *(file.readAllGridMetadata());
    file_cache.meta_data = *file.getMetadata();
  }
  catch (const openvdb::IoError &e) {
    file_cache.error_message = e.what();
  }
  catch (...) {
    file_cache.error_message = "Unknown error reading VDB file";
  }
  if (!file_cache.error_message.empty()) {
    return file_cache;
  }

  for (openvdb::GridBase::Ptr &vdb_grid : vdb_grids) {
    if (!vdb_grid) {
      continue;
    }
    GridCache grid_cache;
    grid_cache.meta_data_grid = vdb_grid;
    file_cache.grids.append(std::move(grid_cache));
  }

  return file_cache;
}

static FileCache &get_file_cache(const StringRef file_path)
{
  Cache &cache = get_global_cache();
  BLI_assert(!cache.mutex.try_lock());
  return cache.file_map.lookup_or_add_cb_as(file_path,
                                            [&]() { return create_file_cache(file_path); });
}

static openvdb::GridBase::Ptr load_single_grid_from_disk(const StringRef file_path,
                                                         const StringRef grid_name)
{
  /* Disable delay loading and file copying, this has poor performance
   * on network drivers. */
  const bool delay_load = false;

  openvdb::io::File file(file_path);
  file.setCopyMaxBytes(0);
  file.open(delay_load);
  return file.readGrid(grid_name);
}

static GVolumeGrid get_cached_grid(const StringRef file_path,
                                   GridCache &grid_cache,
                                   const int simplify_level)
{
  if (GVolumeGrid *grid = grid_cache.grid_by_simplify_level.lookup_ptr(simplify_level)) {
    return *grid;
  }
  auto load_grid_fn = [file_path = std::string(file_path),
                       grid_name = std::string(grid_cache.meta_data_grid->getName()),
                       simplify_level]() {
    if (simplify_level == 0) {
      return load_single_grid_from_disk(file_path, grid_name);
    }
    const GVolumeGrid main_grid = volume_file_cache::get_grid_from_file(file_path, grid_name, 0);
    const VolumeGridType grid_type = main_grid->grid_type();
    const float resolution_factor = 1.0f / (1 << simplify_level);
    return BKE_volume_grid_create_with_changed_resolution(
        grid_type, main_grid->grid(), resolution_factor);
  };
  GVolumeGrid grid{
      MEM_new<VolumeGridData>(__func__, load_grid_fn, grid_cache.meta_data_grid->copyGrid())};
  grid_cache.grid_by_simplify_level.add(simplify_level, grid);
  return grid;
}

GVolumeGrid get_grid_from_file(const StringRef file_path,
                               const StringRef grid_name,
                               const int simplify_level)
{
  Cache &cache = get_global_cache();
  std::lock_guard lock{cache.mutex};
  FileCache &file_cache = get_file_cache(file_path);
  if (GridCache *grid_cache = file_cache.grid_cache_by_name(grid_name)) {
    return get_cached_grid(file_path, *grid_cache, simplify_level);
  }
  return {};
}

Vector<GVolumeGrid> get_all_grids_from_file(const StringRef file_path, const int simplify_level)
{
  Cache &cache = get_global_cache();
  std::lock_guard lock{cache.mutex};
  Vector<GVolumeGrid> grids;
  FileCache &file_cache = get_file_cache(file_path);
  for (GridCache &grid_cache : file_cache.grids) {
    grids.append(get_cached_grid(file_path, grid_cache, simplify_level));
  }
  return grids;
}

void unload_unused()
{
  Cache &cache = get_global_cache();
  std::lock_guard lock{cache.mutex};
  for (FileCache &file_cache : cache.file_map.values()) {
    for (GridCache &grid_cache : file_cache.grids) {
      grid_cache.grid_by_simplify_level.remove_if(
          [&](const GVolumeGrid &grid) { return grid->is_mutable(); });
    }
  }
}

}  // namespace blender::bke::volume_file_cache
