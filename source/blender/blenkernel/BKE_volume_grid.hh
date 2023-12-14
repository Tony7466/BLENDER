/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bke
 */

#include <atomic>
#include <functional>
#include <mutex>

#include "BKE_volume_enums.hh"
#include "BKE_volume_grid_type_traits.hh"

#include "BLI_implicit_sharing_ptr.hh"
#include "BLI_string_ref.hh"

#include "openvdb_fwd.hh"

namespace blender::bke {

class GVolumeGrid;
class VolumeTreeUser;

class VolumeGridData : public ImplicitSharingMixin {
 private:
  struct TreeUserToken {
  };

  mutable std::mutex mutex_;
  mutable std::shared_ptr<openvdb::GridBase> grid_;
  mutable const ImplicitSharingInfo *tree_sharing_info_ = nullptr;
  mutable bool tree_loaded_ = false;
  mutable bool transform_loaded_ = false;
  mutable bool meta_data_loaded_ = false;
  std::function<std::shared_ptr<openvdb::GridBase>()> lazy_load_grid_;
  mutable std::string error_message_;

  std::shared_ptr<TreeUserToken> tree_user_token_;

  friend class VolumeTreeUser;

  VolumeGridData();

 public:
  explicit VolumeGridData(std::shared_ptr<openvdb::GridBase> grid);
  explicit VolumeGridData(std::function<std::shared_ptr<openvdb::GridBase>()> lazy_load_grid,
                          std::shared_ptr<openvdb::GridBase> meta_data_and_transform_grid = {});
  ~VolumeGridData();

  VolumeTreeUser tree_user() const;

  GVolumeGrid copy() const;

  const openvdb::GridBase &grid(const VolumeTreeUser &tree_user) const;
  openvdb::GridBase &grid_for_write(const VolumeTreeUser &tree_user);

  StringRefNull name() const;
  void set_name(StringRef name);

  std::shared_ptr<const openvdb::GridBase> grid_ptr(const VolumeTreeUser &tree_user) const;
  std::shared_ptr<openvdb::GridBase> grid_ptr_for_write(const VolumeTreeUser &tree_user);

  const openvdb::math::Transform &transform() const;
  openvdb::math::Transform &transform_for_write();

  VolumeGridType grid_type() const;
  openvdb::GridClass grid_class() const;

  bool is_loaded() const;
  bool can_be_reloaded() const;
  void unload_tree_if_possible() const;

 private:
  void ensure_grid_loaded() const;
  void delete_self();
};

class VolumeTreeUser {
 private:
  std::shared_ptr<VolumeGridData::TreeUserToken> token_;

  friend VolumeGridData;

 public:
  bool valid_for(const VolumeGridData &grid) const;
};

class GVolumeGrid {
 private:
  ImplicitSharingPtr<VolumeGridData> data_;

 public:
  GVolumeGrid() = default;
  explicit GVolumeGrid(const VolumeGridData *data);
  explicit GVolumeGrid(std::shared_ptr<openvdb::GridBase> grid);

  const VolumeGridData &get() const;
  VolumeGridData &get_for_write();

  const VolumeGridData *operator->() const;

  operator bool() const;
};

template<typename T> class VolumeGrid : public GVolumeGrid {
 public:
  VolumeGrid() = default;
  explicit VolumeGrid(const VolumeGridData *data);
  explicit VolumeGrid(std::shared_ptr<OpenvdbGridType<T>> grid);

 private:
  void assert_correct_type() const;
};

inline GVolumeGrid::GVolumeGrid(const VolumeGridData *data) : data_(data) {}

inline const VolumeGridData &GVolumeGrid::get() const
{
  BLI_assert(*this);
  return *data_.get();
}

inline GVolumeGrid::operator bool() const
{
  return bool(data_);
}

inline const VolumeGridData *GVolumeGrid::operator->() const
{
  BLI_assert(*this);
  return data_.get();
}

template<typename T>
inline VolumeGrid<T>::VolumeGrid(const VolumeGridData *data) : GVolumeGrid(data)
{
  this->assert_correct_type();
}

template<typename T> inline void VolumeGrid<T>::assert_correct_type() const
{
#ifndef NDEBUG
  if (data_) {
    const VolumeGridType expected_type = VolumeGridTraits<T>::EnumType;
    const VolumeGridType actual_type = data_->grid_type();
    BLI_assert(expected_type == actual_type);
  }
#endif
}

inline bool VolumeTreeUser::valid_for(const VolumeGridData &grid) const
{
  return grid.tree_user_token_ == token_;
}

}  // namespace blender::bke
