/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bli
 */

#include "BLI_cpp_type.hh"
#include "BLI_math_base.hh"
#include "BLI_resource_scope.hh"
#include "BLI_volume.hh"

namespace blender {

#ifdef WITH_OPENVDB

namespace volume {

bool VolumeMask::is_empty() const
{
  return grid_.empty();
}

int64_t VolumeMask::min_voxel_count() const
{
  return grid_.activeVoxelCount();
}

int64_t VolumeGrid::voxel_count() const
{
  return grid_ ? grid_->activeVoxelCount() : 0;
}

bool VolumeGrid::is_empty() const
{
  return grid_ ? grid_->empty() : true;
}

VolumeGrid::operator bool() const
{
  return grid_ != nullptr;
}

VolumeGrid VolumeGrid::create(ResourceScope &scope,
                              const CPPType &type,
                              const void *background_value)
{
  openvdb::GridBase::Ptr grid;
  volume::field_to_static_type(type, [&grid, background_value](auto type_tag) {
    using ValueType = typename decltype(type_tag)::type;
    const ValueType &value = *static_cast<const ValueType *>(background_value);
    grid = grid_types::GridCommon<ValueType>::create(value);
  });

  return VolumeGrid{scope.add_value<openvdb::GridBase::Ptr>(std::move(grid))};
}

VolumeGrid VolumeGrid::create(ResourceScope &scope, const CPPType &type)
{
  openvdb::GridBase::Ptr grid;
  volume::field_to_static_type(type, [&grid](auto type_tag) {
    using ValueType = typename decltype(type_tag)::type;
    const CPPType &type = CPPType::get<ValueType>();
    const ValueType &value = *static_cast<const ValueType *>(type.default_value());
    grid = grid_types::GridCommon<ValueType>::create(value);
  });

  return VolumeGrid{scope.add_value<openvdb::GridBase::Ptr>(std::move(grid))};
}

VolumeGrid VolumeGrid::create(ResourceScope &scope,
                              const CPPType &type,
                              const VolumeMask &mask,
                              const void *inactive_value,
                              const void *active_value)
{
  openvdb::GridBase::Ptr grid;
  volume::field_to_static_type(type, [&](auto type_tag) {
    using ValueType = typename decltype(type_tag)::type;
    using TreeType = grid_types::TreeCommon<ValueType>;
    using GridType = grid_types::GridCommon<ValueType>;

    const ValueType &typed_inactive_value = *static_cast<const ValueType *>(inactive_value);
    const ValueType &typed_active_value = *static_cast<const ValueType *>(active_value);
    const TreeType::Ptr tree = TreeType::Ptr(new TreeType(
        mask.grid().tree(), typed_inactive_value, typed_active_value, openvdb::TopologyCopy{}));
    grid = GridType::Ptr(new GridType(tree));
  });

  return VolumeGrid{scope.add_value<openvdb::GridBase::Ptr>(std::move(grid))};
}

}  // namespace volume

#else

namespace volume {

bool VolumeMask::is_empty() const
{
  return true;
}

int64_t VolumeMask::min_voxel_count() const
{
  return 0;
}

VolumeGrid::operator bool() const
{
  return false;
}

int64_t VolumeGrid::voxel_count() const
{
  return 0;
}

bool VolumeGrid::is_empty() const
{
  return true;
}

VolumeGrid::operator bool() const
{
  return false;
}

VolumeGrid VolumeGrid::create(ResourceScope &scope, const CPPType &type, const int64_t voxel_count)
{
  return VolumeGrid{};
}

VolumeGrid VolumeGrid::create(ResourceScope &scope,
                              const CPPType &type,
                              const void *background_value)
{
  return VolumeGrid{};
}

}  // namespace volume

#endif

}  // namespace blender
