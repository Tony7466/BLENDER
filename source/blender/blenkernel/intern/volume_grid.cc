/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_volume_grid.hh"
#include "BKE_volume_openvdb.hh"

#include "BLI_ghash.h"
#include "BLI_string_ref.hh"

#include "CLG_log.h"

#ifdef WITH_OPENVDB
#  include <openvdb/openvdb.h>
#endif

#ifdef WITH_OPENVDB

static CLG_LogRef LOG = {"bke.volume"};

/* Global Volume File Cache
 *
 * Global cache of grids read from VDB files. This is used for sharing grids
 * between multiple volume datablocks with the same filepath, and sharing grids
 * between original and copy-on-write datablocks created by the depsgraph.
 *
 * There are two types of users. Some datablocks only need the grid metadata,
 * example an original datablock volume showing the list of grids in the
 * properties editor. Other datablocks also need the tree and voxel data, for
 * rendering for example. So, depending on the users the grid in the cache may
 * have a tree or not.
 *
 * When the number of users drops to zero, the grid data is immediately deleted.
 *
 * TODO: also add a cache for OpenVDB files rather than individual grids,
 * so getting the list of grids is also cached.
 * TODO: Further, we could cache openvdb::io::File so that loading a grid
 * does not re-open it every time. But then we have to take care not to run
 * out of file descriptors or prevent other applications from writing to it.
 */

/* Cache Entry */
struct VolumeFileCacheEntry {
  VolumeFileCacheEntry(blender::StringRef filepath, const openvdb::GridBase::Ptr &grid)
      : filepath(filepath), grid_name(grid->getName()), grid(grid)
  {
  }

  VolumeFileCacheEntry(const VolumeFileCacheEntry &other)
      : filepath(other.filepath),
        grid_name(other.grid_name),
        grid(other.grid),
        is_loaded(other.is_loaded)
  {
  }

  /* Returns the original grid or a simplified version depending on the given #simplify_level. */
  openvdb::GridBase::Ptr simplified_grid(const int simplify_level)
  {
    BLI_assert(simplify_level >= 0);
    if (simplify_level == 0 || !is_loaded) {
      return grid;
    }

    std::lock_guard<std::mutex> lock(mutex);
    openvdb::GridBase::Ptr simple_grid;

    /* Isolate creating grid since that's multithreaded and we are
     * holding a mutex lock. */
    blender::threading::isolate_task([&] {
      simple_grid = simplified_grids.lookup_or_add_cb(simplify_level, [&]() {
        const float resolution_factor = 1.0f / (1 << simplify_level);
        const VolumeGridType grid_type = BKE_volume_grid_type_openvdb(*grid);
        return BKE_volume_grid_create_with_changed_resolution(grid_type, *grid, resolution_factor);
      });
    });
    return simple_grid;
  }

  /* Unique key: filename + grid name. */
  std::string filepath;
  std::string grid_name;

  /* OpenVDB grid. */
  openvdb::GridBase::Ptr grid;

  /* Simplified versions of #grid. The integer key is the simplification level. */
  blender::Map<int, openvdb::GridBase::Ptr> simplified_grids;

  /* Has the grid tree been loaded? */
  mutable bool is_loaded = false;
  /* Error message if an error occurred while loading. */
  std::string error_msg;
  /* User counting. */
  int num_metadata_users = 0;
  int num_tree_users = 0;
  /* Mutex for on-demand reading of tree. */
  mutable std::mutex mutex;
};

static struct VolumeFileCache {
  using Entry = VolumeFileCacheEntry;

  struct EntryHasher {
    std::size_t operator()(const Entry &entry) const
    {
      std::hash<std::string> string_hasher;
      return BLI_ghashutil_combine_hash(string_hasher(entry.filepath),
                                        string_hasher(entry.grid_name));
    }
  };

  struct EntryEqual {
    bool operator()(const Entry &a, const Entry &b) const
    {
      return a.filepath == b.filepath && a.grid_name == b.grid_name;
    }
  };

  /* Cache */
  ~VolumeFileCache()
  {
    BLI_assert(cache.empty());
  }

  Entry *add_metadata_user(const Entry &template_entry)
  {
    std::lock_guard<std::mutex> lock(mutex);
    EntrySet::iterator it = cache.find(template_entry);
    if (it == cache.end()) {
      it = cache.emplace(template_entry).first;
    }

    /* Casting const away is weak, but it's convenient having key and value in one. */
    Entry &entry = (Entry &)*it;
    entry.num_metadata_users++;

    /* NOTE: pointers to unordered_set values are not invalidated when adding
     * or removing other values. */
    return &entry;
  }

  void copy_user(Entry &entry, const bool tree_user)
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (tree_user) {
      entry.num_tree_users++;
    }
    else {
      entry.num_metadata_users++;
    }
  }

  void remove_user(Entry &entry, const bool tree_user)
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (tree_user) {
      entry.num_tree_users--;
    }
    else {
      entry.num_metadata_users--;
    }
    update_for_remove_user(entry);
  }

  void change_to_tree_user(Entry &entry)
  {
    std::lock_guard<std::mutex> lock(mutex);
    entry.num_tree_users++;
    entry.num_metadata_users--;
    update_for_remove_user(entry);
  }

  void change_to_metadata_user(Entry &entry)
  {
    std::lock_guard<std::mutex> lock(mutex);
    entry.num_metadata_users++;
    entry.num_tree_users--;
    update_for_remove_user(entry);
  }

 protected:
  void update_for_remove_user(Entry &entry)
  {
    /* Isolate file unloading since that's multithreaded and we are
     * holding a mutex lock. */
    blender::threading::isolate_task([&] {
      if (entry.num_metadata_users + entry.num_tree_users == 0) {
        cache.erase(entry);
      }
      else if (entry.num_tree_users == 0) {
        /* Note we replace the grid rather than clearing, so that if there is
         * any other shared pointer to the grid it will keep the tree. */
        entry.grid = entry.grid->copyGridWithNewTree();
        entry.simplified_grids.clear();
        entry.is_loaded = false;
      }
    });
  }

  /* Cache contents */
  using EntrySet = std::unordered_set<Entry, EntryHasher, EntryEqual>;
  EntrySet cache;
  /* Mutex for multithreaded access. */
  std::mutex mutex;
} GLOBAL_CACHE;

VolumeGrid::VolumeGrid(const VolumeFileCacheEntry &template_entry, const int simplify_level)
    : entry(nullptr), simplify_level(simplify_level), is_loaded(false)
{
  entry = GLOBAL_CACHE.add_metadata_user(template_entry);
}

VolumeGrid::VolumeGrid(const char *file_path, const GridPtr &vdb_grid, const int simplify_level)
    : entry(nullptr), simplify_level(simplify_level), is_loaded(false)
{
  entry = GLOBAL_CACHE.add_metadata_user(VolumeFileCacheEntry(file_path, vdb_grid));
}

VolumeGrid::VolumeGrid(const openvdb::GridBase::Ptr &grid)
    : entry(nullptr), local_grid(grid), is_loaded(true)
{
}

VolumeGrid::VolumeGrid(const VolumeGrid &other)
    : entry(other.entry),
      simplify_level(other.simplify_level),
      local_grid(other.local_grid),
      is_loaded(other.is_loaded)
{
  if (entry) {
    GLOBAL_CACHE.copy_user(*entry, is_loaded);
  }
}

VolumeGrid::~VolumeGrid()
{
  if (entry) {
    GLOBAL_CACHE.remove_user(*entry, is_loaded);
  }
}

void VolumeGrid::load(const char *volume_name, const char *filepath) const
{
  /* If already loaded or not file-backed, nothing to do. */
  if (is_loaded || entry == nullptr) {
    return;
  }

  /* Double-checked lock. */
  std::lock_guard<std::mutex> lock(entry->mutex);
  if (is_loaded) {
    return;
  }

  /* Change metadata user to tree user. */
  GLOBAL_CACHE.change_to_tree_user(*entry);

  /* If already loaded by another user, nothing further to do. */
  if (entry->is_loaded) {
    is_loaded = true;
    return;
  }

  /* Load grid from file. */
  CLOG_INFO(&LOG, 1, "Volume %s: load grid '%s'", volume_name, name());

  openvdb::io::File file(filepath);

  /* Isolate file loading since that's potentially multi-threaded and we are
   * holding a mutex lock. */
  blender::threading::isolate_task([&] {
    try {
      /* Disable delay loading and file copying, this has poor performance
       * on network drivers. */
      const bool delay_load = false;
      file.setCopyMaxBytes(0);
      file.open(delay_load);
      openvdb::GridBase::Ptr vdb_grid = file.readGrid(name());
      entry->grid->setTree(vdb_grid->baseTreePtr());
    }
    catch (const openvdb::IoError &e) {
      entry->error_msg = e.what();
    }
    catch (...) {
      entry->error_msg = "Unknown error reading VDB file";
    }
  });

  std::atomic_thread_fence(std::memory_order_release);
  entry->is_loaded = true;
  is_loaded = true;
}

void VolumeGrid::unload(const char *volume_name) const
{
  /* Not loaded or not file-backed, nothing to do. */
  if (!is_loaded || entry == nullptr) {
    return;
  }

  /* Double-checked lock. */
  std::lock_guard<std::mutex> lock(entry->mutex);
  if (!is_loaded) {
    return;
  }

  CLOG_INFO(&LOG, 1, "Volume %s: unload grid '%s'", volume_name, name());

  /* Change tree user to metadata user. */
  GLOBAL_CACHE.change_to_metadata_user(*entry);

  /* Indicate we no longer have a tree. The actual grid may still
   * have it due to another user. */
  std::atomic_thread_fence(std::memory_order_release);
  is_loaded = false;
}

void VolumeGrid::clear_reference(const char * /*volume_name*/)
{
  /* Clear any reference to a grid in the file cache. */
  local_grid = grid()->copyGridWithNewTree();
  if (entry) {
    GLOBAL_CACHE.remove_user(*entry, is_loaded);
    entry = nullptr;
  }
  is_loaded = true;
}

void VolumeGrid::duplicate_reference(const char *volume_name, const char *filepath)
{
  /* Make a deep copy of the grid and remove any reference to a grid in the
   * file cache. Load file grid into memory first if needed. */
  load(volume_name, filepath);
  /* TODO: avoid deep copy if we are the only user. */
  local_grid = grid()->deepCopyGrid();
  if (entry) {
    GLOBAL_CACHE.remove_user(*entry, is_loaded);
    entry = nullptr;
  }
  is_loaded = true;
}

const char *VolumeGrid::name() const
{
  /* Don't use vdb.getName() since it copies the string, we want a pointer to the
   * original so it doesn't get freed out of scope. */
  openvdb::StringMetadata::ConstPtr name_meta = main_grid()->getMetadata<openvdb::StringMetadata>(
      openvdb::GridBase::META_GRID_NAME);
  return (name_meta) ? name_meta->value().c_str() : "";
}

const char *VolumeGrid::error_message() const
{
  if (is_loaded && entry && !entry->error_msg.empty()) {
    return entry->error_msg.c_str();
  }

  return nullptr;
}

bool VolumeGrid::grid_is_loaded() const
{
  return is_loaded;
}

openvdb::GridBase::Ptr VolumeGrid::grid() const
{
  if (entry) {
    return entry->simplified_grid(simplify_level);
  }
  return local_grid;
}

void VolumeGrid::set_simplify_level(const int simplify_level)
{
  BLI_assert(simplify_level >= 0);
  this->simplify_level = simplify_level;
}

const openvdb::GridBase::Ptr &VolumeGrid::main_grid() const
{
  return (entry) ? entry->grid : local_grid;
}

#endif
