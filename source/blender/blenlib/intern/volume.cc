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

namespace blender::volume {

#ifdef WITH_OPENVDB

bool GridMask::is_empty() const
{
  return grid_.empty();
}

int64_t GridMask::min_voxel_count() const
{
  return grid_.activeVoxelCount();
}

int64_t Grid::voxel_count() const
{
  return grid_ ? grid_->activeVoxelCount() : 0;
}

bool Grid::is_empty() const
{
  return grid_ ? grid_->empty() : true;
}

Grid::operator bool() const
{
  return grid_ != nullptr;
}

Grid Grid::create(ResourceScope &scope, const CPPType &type, const void *background_value)
{
  openvdb::GridBase::Ptr grid;
  volume::field_to_static_type(type, [&grid, background_value](auto type_tag) {
    using ValueType = typename decltype(type_tag)::type;
    const ValueType &value = *static_cast<const ValueType *>(background_value);
    grid = grid_types::GridCommon<ValueType>::create(value);
  });

  return Grid{scope.add_value<openvdb::GridBase::Ptr>(std::move(grid))};
}

Grid Grid::create(ResourceScope &scope, const CPPType &type)
{
  openvdb::GridBase::Ptr grid;
  volume::field_to_static_type(type, [&grid](auto type_tag) {
    using ValueType = typename decltype(type_tag)::type;
    const CPPType &type = CPPType::get<ValueType>();
    const ValueType &value = *static_cast<const ValueType *>(type.default_value());
    grid = grid_types::GridCommon<ValueType>::create(value);
  });

  return Grid{scope.add_value<openvdb::GridBase::Ptr>(std::move(grid))};
}

Grid Grid::create(ResourceScope &scope,
                  const CPPType &type,
                  const GridMask &mask,
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

  return Grid{scope.add_value<openvdb::GridBase::Ptr>(std::move(grid))};
}

#else

bool GridMask::is_empty() const
{
  return true;
}

int64_t GridMask::min_voxel_count() const
{
  return 0;
}

Grid::operator bool() const
{
  return false;
}

int64_t Grid::voxel_count() const
{
  return 0;
}

bool Grid::is_empty() const
{
  return true;
}

Grid::operator bool() const
{
  return false;
}

Grid Grid::create(ResourceScope &scope, const CPPType &type, const int64_t voxel_count)
{
  return Grid{};
}

Grid Grid::create(ResourceScope &scope, const CPPType &type, const void *background_value)
{
  return Grid{};
}

#endif

}  // namespace blender::volume
