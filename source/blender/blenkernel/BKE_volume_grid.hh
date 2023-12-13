/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bke
 */

#include <functional>
#include <mutex>

#include "BKE_volume_enums.hh"
#include "BKE_volume_grid_type_traits.hh"

#include "BLI_implicit_sharing_ptr.hh"

#include "openvdb_fwd.hh"

namespace blender::bke {

class GVolumeGrid2;

class VolumeGridData : public ImplicitSharingMixin {
 private:
  mutable std::mutex grid_mutex_;
  mutable std::shared_ptr<openvdb::GridBase> grid_;
  mutable const ImplicitSharingInfo *tree_sharing_info_ = nullptr;
  std::function<std::shared_ptr<openvdb::GridBase>()> lazy_load_grid_;

  VolumeGridData() = default;

 public:
  explicit VolumeGridData(std::shared_ptr<openvdb::GridBase> grid);
  explicit VolumeGridData(std::function<std::shared_ptr<openvdb::GridBase>()> lazy_load_grid);
  ~VolumeGridData();

  GVolumeGrid2 copy() const;

  const openvdb::GridBase &grid() const;
  openvdb::GridBase &grid_for_write();

  const openvdb::math::Transform &transform() const;
  openvdb::math::Transform &transform_for_write();

  VolumeGridType grid_type() const;

 private:
  void ensure_grid_loaded() const;
  void delete_self();
};

class GVolumeGrid2 {
 private:
  ImplicitSharingPtr<VolumeGridData> data_;

 public:
  GVolumeGrid2() = default;
  explicit GVolumeGrid2(const VolumeGridData *data);
  explicit GVolumeGrid2(std::shared_ptr<openvdb::GridBase> grid);

  const VolumeGridData &get() const;
  VolumeGridData &get_for_write();

  const VolumeGridData *operator->() const;

  operator bool() const;
};

template<typename T> class VolumeGrid2 : public GVolumeGrid2 {
 public:
  VolumeGrid2() = default;
  explicit VolumeGrid2(const VolumeGridData *data);
  explicit VolumeGrid2(std::shared_ptr<OpenvdbGridType<T>> grid);

 private:
  void assert_correct_type() const;
};

inline GVolumeGrid2::GVolumeGrid2(const VolumeGridData *data) : data_(data) {}

inline const VolumeGridData &GVolumeGrid2::get() const
{
  BLI_assert(*this);
  return *data_.get();
}

inline GVolumeGrid2::operator bool() const
{
  return bool(data_);
}

inline const VolumeGridData *GVolumeGrid2::operator->() const
{
  BLI_assert(*this);
  return data_.get();
}

template<typename T>
inline VolumeGrid2<T>::VolumeGrid2(const VolumeGridData *data) : GVolumeGrid2(data)
{
  this->assert_correct_type();
}

template<typename T> inline void VolumeGrid2<T>::assert_correct_type() const
{
#ifndef NDEBUG
  if (data_) {
    const VolumeGridType expected_type = VolumeGridTraits<T>::EnumType;
    const VolumeGridType actual_type = data_->grid_type();
    BLI_assert(expected_type == actual_type);
  }
#endif
}

}  // namespace blender::bke
