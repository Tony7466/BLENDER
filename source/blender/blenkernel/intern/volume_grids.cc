/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include "intern/volume_grids.hh"

#include "BLI_task.hh"
#include "BLI_volume_openvdb.hh"

#include "BKE_attribute.hh"
#include "BKE_volume.h"
#include "BKE_volume_openvdb.hh"

#include "CLG_log.h"

#include "attribute_access_volume.hh"

#ifdef WITH_OPENVDB

static CLG_LogRef LOG = {"bke.volume_grids"};

VolumeFileCache::Entry::Entry(const std::string &filepath, const openvdb::GridBase::Ptr &grid)
    : filepath(filepath), grid_name(grid->getName()), grid(grid)
{
}

VolumeFileCache::Entry::Entry(const Entry &other)
    : filepath(other.filepath),
      grid_name(other.grid_name),
      grid(other.grid),
      is_loaded(other.is_loaded)
{
}

/* Returns the original grid or a simplified version depending on the given #simplify_level. */
openvdb::GridBase::Ptr VolumeFileCache::Entry::simplified_grid(const int simplify_level)
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

/* Cache */
VolumeFileCache::~VolumeFileCache()
{
  BLI_assert(cache.empty());
}

VolumeFileCache::Entry *VolumeFileCache::add_metadata_user(const Entry &template_entry)
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

void VolumeFileCache::copy_user(Entry &entry, const bool tree_user)
{
  std::lock_guard<std::mutex> lock(mutex);
  if (tree_user) {
    entry.num_tree_users++;
  }
  else {
    entry.num_metadata_users++;
  }
}

void VolumeFileCache::remove_user(Entry &entry, const bool tree_user)
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

void VolumeFileCache::change_to_tree_user(Entry &entry)
{
  std::lock_guard<std::mutex> lock(mutex);
  entry.num_tree_users++;
  entry.num_metadata_users--;
  update_for_remove_user(entry);
}

void VolumeFileCache::change_to_metadata_user(Entry &entry)
{
  std::lock_guard<std::mutex> lock(mutex);
  entry.num_metadata_users++;
  entry.num_tree_users--;
  update_for_remove_user(entry);
}

void VolumeFileCache::update_for_remove_user(Entry &entry)
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

VolumeGrid::VolumeGrid(const VolumeFileCache::Entry &template_entry, const int simplify_level)
    : entry(nullptr), simplify_level(simplify_level), is_loaded(false)
{
  entry = GLOBAL_CACHE.add_metadata_user(template_entry);
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

VolumeGridType VolumeGrid::grid_type() const
{
  return BKE_volume_grid_type(this);
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

VolumeGridVector::VolumeGridVector() : metadata(new openvdb::MetaMap())
{
  filepath[0] = '\0';
}

VolumeGridVector::VolumeGridVector(const VolumeGridVector &other)
    : std::list<VolumeGrid>(other), error_msg(other.error_msg), metadata(other.metadata)
{
  memcpy(filepath, other.filepath, sizeof(filepath));
}

bool VolumeGridVector::is_loaded() const
{
  return filepath[0] != '\0';
}

void VolumeGridVector::clear_all()
{
  std::list<VolumeGrid>::clear();
  filepath[0] = '\0';
  error_msg.clear();
  metadata.reset();
}

namespace blender::bke {

/**
 * In this function all the attribute providers for a volume component are created.
 * Most data in this function is statically allocated, because it does not change over time.
 */
static ComponentAttributeProviders create_attribute_providers_for_volume()
{
  static VolumeGridAccessInfo grid_access = {
      [](void *owner) -> VolumeGridVector & { return *static_cast<VolumeGridVector *>(owner); },
      [](const void *owner) -> const VolumeGridVector & {
        return *static_cast<const VolumeGridVector *>(owner);
      },
  };

  static auto update_on_change = [](void * /*owner*/) {};

  static VolumeCellCenterAttributeGridProvider cell_center_provider(grid_access, update_on_change);

  static VolumeCustomAttributeGridProvider grid_custom_provider(
      ATTR_DOMAIN_VOXEL, grid_access, update_on_change);

  return ComponentAttributeProviders({&cell_center_provider}, {&grid_custom_provider});
}

static AttributeAccessorFunctions get_volume_accessor_functions()
{
  static const ComponentAttributeProviders providers = create_attribute_providers_for_volume();
  AttributeAccessorFunctions fn =
      attribute_accessor_functions::accessor_functions_for_providers<providers>();
  /* Set domain callbacks that are not defined yet. */
  fn.domain_size = [](const void *owner, const eAttrDomain domain) {
    if (owner == nullptr) {
      return 0;
    }
    const VolumeGridVector &grids = *static_cast<const VolumeGridVector *>(owner);
    return grids.domain_size(domain);
  };
  fn.domain_grid_transform = [](const void *owner, const eAttrDomain domain) -> float4x4 {
    const VolumeGridVector &grids = *static_cast<const VolumeGridVector *>(owner);
    return grids.domain_transform(domain);
  };
  fn.domain_grid_mask =
      [](const void *owner, const eAttrDomain /*domain*/, const int main_grid) -> GVGrid {
    if (owner == nullptr || main_grid < 0) {
      return {nullptr};
    }
    const VolumeGridVector &grids = *static_cast<const VolumeGridVector *>(owner);
    if (grids.empty()) {
      return {nullptr};
    }
    for (const VolumeGrid &grid : grids) {
      int index = main_grid;
      if (index-- == 0) {
        return GVGrid::ForGrid(*grid.grid());
      }
    }
    /* Default if main_grid is invalid. */
    return GVGrid::ForGrid(*grids.front().grid());
  };
  fn.domain_supported = [](const void * /*owner*/, const eAttrDomain domain) {
    return ELEM(domain, ATTR_DOMAIN_VOXEL);
  };
  fn.adapt_domain = [](const void *owner,
                       const GVArray &varray,
                       const eAttrDomain from_domain,
                       const eAttrDomain to_domain) -> GVArray {
    if (owner == nullptr) {
      return {};
    }
    if (from_domain == ATTR_DOMAIN_VOXEL && to_domain == ATTR_DOMAIN_VOXEL) {
      return varray;
    }
    return {};
  };
  return fn;
}

static const AttributeAccessorFunctions &get_volume_accessor_functions_ref()
{
  static const AttributeAccessorFunctions fn = get_volume_accessor_functions();
  return fn;
}

}  // namespace blender::bke

int VolumeGridVector::domain_size(const eAttrDomain domain) const
{
  switch (domain) {
    case ATTR_DOMAIN_VOXEL:
      return empty() ? 0 : front().grid()->activeVoxelCount();
    default:
      return 0;
  }
}

blender::float4x4 VolumeGridVector::domain_transform(const eAttrDomain domain) const
{
  switch (domain) {
    case ATTR_DOMAIN_VOXEL:
      if (empty()) {
        return blender::float4x4::identity();
      }
      blender::float4x4 result;
      BKE_volume_grid_transform_matrix(&front(), result.ptr());
      return result;
    default:
      return blender::float4x4::identity();
  }
}

VolumeGrid *VolumeGridVector::find_grid(const blender::bke::AttributeIDRef &attribute_id)
{
  if (!attribute_id) {
    return nullptr;
  }
  auto result = std::find_if(begin(), end(), [attribute_id](VolumeGrid &grid) {
    return grid.name() == attribute_id.name();
  });
  return result != end() ? &*result : nullptr;
}

const VolumeGrid *VolumeGridVector::find_grid(
    const blender::bke::AttributeIDRef &attribute_id) const
{
  if (!attribute_id) {
    return nullptr;
  }
  auto result = std::find_if(begin(), end(), [attribute_id](const VolumeGrid &grid) {
    return grid.name() == attribute_id.name();
  });
  return result != end() ? &*result : nullptr;
}

blender::bke::AttributeAccessor VolumeGridVector::attributes() const
{
  return blender::bke::AttributeAccessor(this, blender::bke::get_volume_accessor_functions_ref());
}

blender::bke::MutableAttributeAccessor VolumeGridVector::attributes_for_write()
{
  return blender::bke::MutableAttributeAccessor(this,
                                                blender::bke::get_volume_accessor_functions_ref());
}

#endif
