/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_volume_grid.hh"
#include "BKE_volume_openvdb.hh"

#include "BLI_task.hh"

#include <openvdb/Grid.h>

namespace blender::bke {

class OpenvdbTreeSharingInfo : public ImplicitSharingInfo {
 private:
  std::shared_ptr<openvdb::tree::TreeBase> tree_;

 public:
  OpenvdbTreeSharingInfo(std::shared_ptr<openvdb::tree::TreeBase> tree) : tree_(std::move(tree)) {}

  void delete_self_with_data() override
  {
    MEM_freeN(this);
  }

  void delete_data_only() override
  {
    tree_.reset();
  }
};

VolumeGridData::VolumeGridData()
{
  tree_user_token_ = std::make_shared<TreeUserToken>();
}

VolumeGridData::VolumeGridData(std::shared_ptr<openvdb::GridBase> grid)
    : tree_loaded_(true), transform_loaded_(true), meta_data_loaded_(true), grid_(std::move(grid))
{
  BLI_assert(grid_);
  BLI_assert(grid_.unique());
  BLI_assert(grid_->isTreeUnique());

  tree_sharing_info_ = MEM_new<OpenvdbTreeSharingInfo>(__func__, grid_->baseTreePtr());
  tree_user_token_ = std::make_shared<TreeUserToken>();
}

VolumeGridData::VolumeGridData(std::function<std::shared_ptr<openvdb::GridBase>()> lazy_load_grid,
                               std::shared_ptr<openvdb::GridBase> meta_data_and_transform_grid)
    : grid_(std::move(meta_data_and_transform_grid)), lazy_load_grid_(std::move(lazy_load_grid))
{
  if (grid_) {
    transform_loaded_ = true;
    meta_data_loaded_ = true;
  }
  tree_user_token_ = std::make_shared<TreeUserToken>();
}

VolumeGridData::~VolumeGridData()
{
  if (tree_sharing_info_) {
    tree_sharing_info_->remove_user_and_delete_if_last();
  }
}

void VolumeGridData::delete_self()
{
  MEM_delete(this);
}

VolumeTreeUser VolumeGridData::tree_user() const
{
  VolumeTreeUser user;
  user.token_ = tree_user_token_;
  return user;
}

const openvdb::GridBase &VolumeGridData::grid(const VolumeTreeUser &tree_user) const
{
  return *this->grid_ptr(tree_user);
}

openvdb::GridBase &VolumeGridData::grid_for_write(const VolumeTreeUser &tree_user)
{
  return *this->grid_ptr_for_write(tree_user);
}

std::shared_ptr<const openvdb::GridBase> VolumeGridData::grid_ptr(
    const VolumeTreeUser & /*tree_user*/) const
{
  std::lock_guard lock{mutex_};
  this->ensure_grid_loaded();
  return grid_;
}

std::shared_ptr<openvdb::GridBase> VolumeGridData::grid_ptr_for_write(
    const VolumeTreeUser & /*tree_user*/)
{
  BLI_assert(this->is_mutable());
  std::lock_guard lock{mutex_};
  this->ensure_grid_loaded();
  if (tree_sharing_info_->is_mutable()) {
    tree_sharing_info_->tag_ensured_mutable();
  }
  else {
    auto tree_copy = grid_->baseTree().copy();
    grid_->setTree(tree_copy);
    tree_sharing_info_->remove_user_and_delete_if_last();
    tree_sharing_info_ = MEM_new<OpenvdbTreeSharingInfo>(__func__, std::move(tree_copy));
  }
  return grid_;
}

const openvdb::math::Transform &VolumeGridData::transform() const
{
  std::lock_guard lock{mutex_};
  if (!transform_loaded_) {
    this->ensure_grid_loaded();
  }
  return grid_->transform();
}

openvdb::math::Transform &VolumeGridData::transform_for_write()
{
  BLI_assert(this->is_mutable());
  std::lock_guard lock{mutex_};
  if (!transform_loaded_) {
    this->ensure_grid_loaded();
  }
  return grid_->transform();
}

StringRefNull VolumeGridData::name() const
{
  std::lock_guard lock{mutex_};
  if (!meta_data_loaded_) {
    this->ensure_grid_loaded();
  }
  return grid_->getName();
}

void VolumeGridData::set_name(const StringRef name)
{
  BLI_assert(this->is_mutable());
  std::lock_guard lock{mutex_};
  if (!meta_data_loaded_) {
    this->ensure_grid_loaded();
  }
  grid_->setName(name);
}

VolumeGridType VolumeGridData::grid_type() const
{
  std::lock_guard lock{mutex_};
  if (!meta_data_loaded_) {
    this->ensure_grid_loaded();
  }
  return BKE_volume_grid_type_openvdb(*grid_);
}

bool VolumeGridData::can_be_reloaded() const
{
  return bool(lazy_load_grid_);
}

void VolumeGridData::unload_tree_if_possible() const
{
  std::lock_guard lock{mutex_};
  if (!grid_) {
    return;
  }
  if (!this->can_be_reloaded()) {
    return;
  }
  if (!tree_user_token_.unique()) {
    /* Some code is using the tree currently, so it can't be freed. */
    return;
  }
  grid_->newTree();
  tree_loaded_ = false;
}

GVolumeGrid VolumeGridData::copy() const
{
  std::lock_guard lock{mutex_};
  this->ensure_grid_loaded();
  /* Can't use #MEM_new because the default construtor is private. */
  VolumeGridData *new_copy = new (MEM_mallocN(sizeof(VolumeGridData), __func__)) VolumeGridData();
  /* Makes a deep copy of the meta-data but shares the tree. */
  new_copy->grid_ = grid_->copyGrid();
  new_copy->tree_sharing_info_ = tree_sharing_info_;
  new_copy->tree_sharing_info_->add_user();
  new_copy->tree_loaded_ = tree_loaded_;
  new_copy->transform_loaded_ = transform_loaded_;
  new_copy->meta_data_loaded_ = meta_data_loaded_;
  return GVolumeGrid(new_copy);
}

void VolumeGridData::ensure_grid_loaded() const
{
  /* Assert that the mutex is locked. */
  BLI_assert(!mutex_.try_lock());

  if (tree_loaded_ && transform_loaded_ && meta_data_loaded_) {
    return;
  }
  BLI_assert(lazy_load_grid_);
  /* TODO: exception handling, reuse old transform/meta-data*/
  std::shared_ptr<openvdb::GridBase> loaded_grid;
  threading::isolate_task([&]() {
    error_message_.clear();
    try {
      loaded_grid = lazy_load_grid_();
    }
    catch (const openvdb::IoError &e) {
      error_message_ = e.what();
    }
    catch (...) {
      error_message_ = "Unknown error reading VDB file";
    }
  });
  if (!loaded_grid) {
    /* TODO: Create empty grid (potentially of correct type). */
    BLI_assert_unreachable();
  }
  BLI_assert(loaded_grid);
  BLI_assert(loaded_grid.unique());
  BLI_assert(loaded_grid->isTreeUnique());

  if (grid_) {
    /* Keep the existing grid pointer and just insert the newly loaded data. */
    BLI_assert(!tree_loaded_);
    BLI_assert(meta_data_loaded_);
    grid_->setTree(loaded_grid->baseTreePtr());
    if (!transform_loaded_) {
      grid_->setTransform(loaded_grid->transformPtr());
    }
  }
  else {
    grid_ = std::move(loaded_grid);
  }

  tree_sharing_info_ = MEM_new<OpenvdbTreeSharingInfo>(__func__, grid_->baseTreePtr());

  tree_loaded_ = true;
  transform_loaded_ = true;
  meta_data_loaded_ = true;
}

GVolumeGrid::GVolumeGrid(std::shared_ptr<openvdb::GridBase> grid)
{
  data_ = ImplicitSharingPtr(MEM_new<VolumeGridData>(__func__, std::move(grid)));
}

VolumeGridData &GVolumeGrid::get_for_write()
{
  BLI_assert(*this);
  if (data_->is_mutable()) {
    data_->tag_ensured_mutable();
  }
  else {
    *this = data_->copy();
  }
  return const_cast<VolumeGridData &>(*data_);
}

}  // namespace blender::bke
