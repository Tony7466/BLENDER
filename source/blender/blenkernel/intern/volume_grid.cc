/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_volume_grid.hh"
#include "BKE_volume_grid_ptr.hh"
#include "BKE_volume_openvdb.hh"

#include "BLI_string_ref.hh"

#include "CLG_log.h"

#ifdef WITH_OPENVDB
#  include <openvdb/openvdb.h>
#endif

static CLG_LogRef LOG = {"bke.volume"};

namespace blender::bke {

#ifdef WITH_OPENVDB
VolumeGrid::VolumeGrid(const GridBasePtr &grid, VolumeTreeSource tree_source)
    : grid_(grid), tree_source_(tree_source)
{
  BLI_assert(grid);
}

VolumeGrid::~VolumeGrid() {}

VolumeGrid *VolumeGrid::copy() const
{
  GridBasePtr grid_copy = grid_ ? grid_->deepCopyGrid() : nullptr;
  BLI_assert(grid_copy);
  return new VolumeGrid(grid_copy, tree_source_);
}

const char *VolumeGrid::name() const
{
  /* Don't use vdb.getName() since it copies the string, we want a pointer to the
   * original so it doesn't get freed out of scope. */
  BLI_assert(grid_);
  openvdb::StringMetadata::ConstPtr name_meta = grid_->getMetadata<openvdb::StringMetadata>(
      openvdb::GridBase::META_GRID_NAME);
  return (name_meta) ? name_meta->value().c_str() : "";
}

VolumeTreeSource VolumeGrid::tree_source() const
{
  return tree_source_;
}

bool VolumeGrid::ensure_tree_loaded(StringRef filepath,
                                    FunctionRef<void(StringRef)> error_fn) const
{
  /* If already loaded or not file-backed, nothing to do. */
  if (tree_source_ != VOLUME_TREE_SOURCE_FILE_PLACEHOLDER) {
    return true;
  }

  /* Double-checked lock. */
  std::lock_guard<std::mutex> lock(mutex_);
  if (tree_source_ != VOLUME_TREE_SOURCE_FILE_PLACEHOLDER) {
    return true;
  }

  /* Load grid from file. */
  CLOG_INFO(&LOG, 1, "Load grid '%s'", name());

  openvdb::io::File file(filepath);

  /* Isolate file loading since that's potentially multi-threaded and we are
   * holding a mutex lock. */
  bool success = true;
  blender::threading::isolate_task([&] {
    try {
      /* Disable delay loading and file copying, this has poor performance
       * on network drivers. */
      const bool delay_load = false;
      file.setCopyMaxBytes(0);
      file.open(delay_load);
      grid_ = file.readGrid(name());
      BLI_assert(grid_);
      tree_source_ = VOLUME_TREE_SOURCE_FILE_LOADED;
    }
    catch (const openvdb::IoError &e) {
      error_fn(e.what());
      success = false;
    }
    catch (...) {
      error_fn("Unknown error reading VDB file");
      success = false;
    }
  });
  return success;
}

void VolumeGrid::unload_tree() const
{
  /* Not loaded or not file-backed, nothing to do. */
  if (!ELEM(tree_source_, VOLUME_TREE_SOURCE_FILE_LOADED, VOLUME_TREE_SOURCE_FILE_SIMPLIFIED)) {
    return;
  }

  /* Double-checked lock. */
  std::lock_guard<std::mutex> lock(mutex_);
  if (!ELEM(tree_source_, VOLUME_TREE_SOURCE_FILE_LOADED, VOLUME_TREE_SOURCE_FILE_SIMPLIFIED)) {
    return;
  }

  CLOG_INFO(&LOG, 1, "Unload grid '%s'", name());

  /* Grid pointer should never be null, replace with an empty tree. */
  BLI_assert(grid_);
  /* Note we replace the grid rather than clearing, so that if there is
   * any other shared pointer to the grid it will keep the tree. */
  grid_ = grid_->copyGridWithNewTree();
  /* Indicate we no longer have a tree. */
  std::atomic_thread_fence(std::memory_order_release);
  tree_source_ = VOLUME_TREE_SOURCE_FILE_PLACEHOLDER;
}

VolumeGrid::GridBaseConstPtr VolumeGrid::grid() const
{
  return grid_;
}

VolumeGrid::GridBasePtr VolumeGrid::grid_for_write()
{
  BLI_assert(this->is_mutable());
  return grid_;
}
#endif

void VolumeGrid::delete_self()
{
  delete this;
}

void VolumeGrid::delete_data_only()
{
  grid_ = nullptr;
}

GVolumeGridPtr::GridConstPtr GVolumeGridPtr::grid() const
{
#ifdef WITH_OPENVDB
  return this->get()->grid();
#else
  return nullptr;
#endif
}

GVolumeGridPtr::GridPtr GVolumeGridPtr::grid_for_write() const
{
#ifdef WITH_OPENVDB
  return const_cast<VolumeGrid *>(this->get())->grid_for_write();
#else
  return nullptr;
#endif
}

}  // namespace blender::bke
