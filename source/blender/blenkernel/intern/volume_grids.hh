/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#pragma once

#include "BLI_ghash.h"
#include "BLI_map.hh"
#include "BLI_path_util.h"
#include "BLI_volume.hh"

#include "BKE_attribute.h"
#include "BKE_volume.h"

#include "DNA_volume_types.h"

#include <list>
#include <mutex>
#include <unordered_set>

#ifdef WITH_OPENVDB

namespace blender::bke {
class AttributeAccessor;
class MutableAttributeAccessor;
class AttributeIDRef;
}  // namespace blender::bke

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

static struct VolumeFileCache {
  /* Cache Entry */
  struct Entry {
    using GridPtr = std::shared_ptr<openvdb::GridBase>;

    Entry(const std::string &filepath, const GridPtr &grid);
    Entry(const Entry &other);

    /* Returns the original grid or a simplified version depending on the given #simplify_level. */
    GridPtr simplified_grid(const int simplify_level);

    /* Unique key: filename + grid name. */
    std::string filepath;
    std::string grid_name;

    /* OpenVDB grid. */
    GridPtr grid;

    /* Simplified versions of #grid. The integer key is the simplification level. */
    blender::Map<int, GridPtr> simplified_grids;

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
  ~VolumeFileCache();

  Entry *add_metadata_user(const Entry &template_entry);
  void copy_user(Entry &entry, const bool tree_user);
  void remove_user(Entry &entry, const bool tree_user);

  void change_to_tree_user(Entry &entry);
  void change_to_metadata_user(Entry &entry);

 protected:
  void update_for_remove_user(Entry &entry);

  /* Cache contents */
  using EntrySet = std::unordered_set<Entry, EntryHasher, EntryEqual>;
  EntrySet cache;
  /* Mutex for multithreaded access. */
  std::mutex mutex;
} GLOBAL_CACHE;

/* VolumeGrid
 *
 * Wrapper around OpenVDB grid. Grids loaded from OpenVDB files are always
 * stored in the global cache. Procedurally generated grids are not. */

struct VolumeGrid {
  using GridPtr = std::shared_ptr<openvdb::GridBase>;

  VolumeGrid(const VolumeFileCache::Entry &template_entry, const int simplify_level);
  VolumeGrid(const GridPtr &grid);
  VolumeGrid(const VolumeGrid &other);
  ~VolumeGrid();

  void load(const char *volume_name, const char *filepath) const;
  void unload(const char *volume_name) const;

  void clear_reference(const char * /*volume_name*/);
  void duplicate_reference(const char *volume_name, const char *filepath);

  const char *name() const;
  const char *error_message() const;
  bool grid_is_loaded() const;
  VolumeGridType grid_type() const;
  GridPtr grid() const;

  void set_simplify_level(const int simplify_level);

 private:
  const GridPtr &main_grid() const;

 protected:
  /* File cache entry when grid comes directly from a file and may be shared
   * with other volume datablocks. */
  VolumeFileCache::Entry *entry;
  /* If this volume grid is in the global file cache, we can reference a simplified version of it,
   * instead of the original high resolution grid. */
  int simplify_level = 0;
  /* OpenVDB grid if it's not shared through the file cache. */
  GridPtr local_grid;
  /**
   * Indicates if the tree has been loaded for this grid. Note that vdb.tree()
   * may actually be loaded by another user while this is false. But only after
   * calling load() and is_loaded changes to true is it safe to access.
   *
   * `const` write access to this must be protected by `entry->mutex`.
   */
  mutable bool is_loaded;
};

/* Volume Grid Vector
 *
 * List of grids contained in a volume datablock. This is runtime-only data,
 * the actual grids are always saved in a VDB file. */

struct VolumeGridVector : public std::list<VolumeGrid> {
  using MetaMapPtr = std::shared_ptr<openvdb::MetaMap>;

  VolumeGridVector();
  VolumeGridVector(const VolumeGridVector &other);

  bool is_loaded() const;
  void clear_all();

  int domain_size(eAttrDomain domain) const;

  VolumeGrid *find_grid(const blender::bke::AttributeIDRef &attribute_id);
  const VolumeGrid *find_grid(const blender::bke::AttributeIDRef &attribute_id) const;

  blender::bke::AttributeAccessor attributes() const;
  blender::bke::MutableAttributeAccessor attributes_for_write();

  /* Mutex for file loading of grids list. `const` write access to the fields after this must be
   * protected by locking with this mutex. */
  mutable std::mutex mutex;
  /* Absolute file path that grids have been loaded from. */
  char filepath[FILE_MAX];
  /* File loading error message. */
  std::string error_msg;
  /* File Metadata. */
  MetaMapPtr metadata;
};

#endif
