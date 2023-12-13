/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_volume_grid2.hh"
#include "BKE_volume_openvdb.hh"

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

VolumeGridData::VolumeGridData(std::shared_ptr<openvdb::GridBase> grid) : grid_(std::move(grid))
{
  BLI_assert(grid_);
  BLI_assert(grid_.unique());
  BLI_assert(grid_->isTreeUnique());

  tree_sharing_info_ = MEM_new<OpenvdbTreeSharingInfo>(__func__, grid_->baseTreePtr());
}

VolumeGridData::VolumeGridData(std::function<std::shared_ptr<openvdb::GridBase>()> lazy_load_grid)
    : lazy_load_grid_(std::move(lazy_load_grid))
{
}

VolumeGridData::~VolumeGridData()
{
  tree_sharing_info_->remove_user_and_delete_if_last();
}

void VolumeGridData::delete_self()
{
  MEM_delete(this);
}

const openvdb::GridBase &VolumeGridData::grid() const
{
  this->ensure_grid_loaded();
  std::lock_guard lock{grid_mutex_};
  return *grid_;
}

openvdb::GridBase &VolumeGridData::grid_for_write()
{
  BLI_assert(this->is_mutable());
  this->ensure_grid_loaded();
  std::lock_guard lock{grid_mutex_};
  if (tree_sharing_info_->is_mutable()) {
    tree_sharing_info_->tag_ensured_mutable();
  }
  else {
    auto tree_copy = grid_->baseTree().copy();
    grid_->setTree(tree_copy);
    tree_sharing_info_->remove_user_and_delete_if_last();
    tree_sharing_info_ = MEM_new<OpenvdbTreeSharingInfo>(__func__, std::move(tree_copy));
  }
  return *grid_;
}

const openvdb::math::Transform &VolumeGridData::transform() const
{
  return this->grid().transform();
}

openvdb::math::Transform &VolumeGridData::transform_for_write()
{
  BLI_assert(this->is_mutable());
  this->ensure_grid_loaded();
  return grid_->transform();
}

VolumeGridType VolumeGridData::grid_type() const
{
  return BKE_volume_grid_type_openvdb(this->grid());
}

GVolumeGrid2 VolumeGridData::copy() const
{
  this->ensure_grid_loaded();
  std::lock_guard lock{grid_mutex_};
  /* Can't use #MEM_new because the default construtor is private. */
  VolumeGridData *new_copy = new (MEM_mallocN(sizeof(VolumeGridData), __func__)) VolumeGridData();
  /* Makes a deep copy of the meta-data but shares the tree. */
  new_copy->grid_ = grid_->copyGrid();
  new_copy->tree_sharing_info_ = tree_sharing_info_;
  new_copy->tree_sharing_info_->add_user();
  return GVolumeGrid2(new_copy);
}

void VolumeGridData::ensure_grid_loaded() const
{
  std::lock_guard lock{grid_mutex_};
  if (grid_) {
    return;
  }
  BLI_assert(lazy_load_grid_);
  grid_ = lazy_load_grid_();
  BLI_assert(grid_);
  BLI_assert(grid_.unique());
  BLI_assert(grid_->isTreeUnique());

  tree_sharing_info_ = MEM_new<OpenvdbTreeSharingInfo>(__func__, grid_->baseTreePtr());
}

GVolumeGrid2::GVolumeGrid2(std::shared_ptr<openvdb::GridBase> grid)
{
  data_ = ImplicitSharingPtr(MEM_new<VolumeGridData>(__func__, std::move(grid)));
}

VolumeGridData &GVolumeGrid2::get_for_write()
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
