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

const openvdb::MaskGrid::ConstPtr empty_grid_ = openvdb::MaskGrid::create();

bool GridMask::is_empty() const
{
  return grid_.empty();
}

int64_t GridMask::min_voxel_count() const
{
  return grid_.activeVoxelCount();
}

int64_t GGrid::voxel_count() const
{
  return grid_ ? grid_->activeVoxelCount() : 0;
}

bool GGrid::is_empty() const
{
  return grid_ ? grid_->empty() : true;
}

GGrid::operator bool() const
{
  return grid_ != nullptr;
}

GGrid GGrid::create(ResourceScope &scope, const CPPType &type, const void *background_value)
{
  openvdb::GridBase::Ptr grid;
  volume::field_to_static_type(type, [&grid, background_value](auto type_tag) {
    using ValueType = typename decltype(type_tag)::type;
    const ValueType &value = *static_cast<const ValueType *>(background_value);
    grid = grid_types::GridCommon<ValueType>::create(value);
  });

  return GGrid{scope.add_value<openvdb::GridBase::Ptr>(std::move(grid))};
}

GGrid GGrid::create(ResourceScope &scope, const CPPType &type)
{
  openvdb::GridBase::Ptr grid;
  volume::field_to_static_type(type, [&grid](auto type_tag) {
    using ValueType = typename decltype(type_tag)::type;
    const CPPType &type = CPPType::get<ValueType>();
    const ValueType &value = *static_cast<const ValueType *>(type.default_value());
    grid = grid_types::GridCommon<ValueType>::create(value);
  });

  return GGrid{scope.add_value<openvdb::GridBase::Ptr>(std::move(grid))};
}

GGrid GGrid::create(ResourceScope &scope,
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
    const typename TreeType::Ptr tree = typename TreeType::Ptr(new TreeType(
        mask.grid().tree(), typed_inactive_value, typed_active_value, openvdb::TopologyCopy{}));
    grid = typename GridType::Ptr(new GridType(tree));
  });

  return GGrid{scope.add_value<openvdb::GridBase::Ptr>(std::move(grid))};
}

template<typename T> Grid<T> Grid<T>::create(ResourceScope &scope, const T &background_value)
{
  typename GridType::Ptr grid = grid_types::GridCommon<ValueType>::create(background_value);
  return Grid<T>{scope.add_value<typename GridType::Ptr>(std::move(grid))};
}

template<typename T> Grid<T> Grid<T>::create(ResourceScope &scope)
{
  ValueType value = *static_cast<const ValueType *>(CPPType::get<T>().default_value_);
  typename GridType::Ptr grid = grid_types::GridCommon<ValueType>::create(value);
  return Grid<T>{scope.add_value<typename GridType::Ptr>(std::move(grid))};
}

template<typename T>
Grid<T> Grid<T>::create(ResourceScope &scope,
                        const GridMask &mask,
                        const T &inactive_value,
                        const T &active_value)
{
  const typename TreeType::Ptr tree = TreeType::Ptr(
      new TreeType(mask.grid().tree(), inactive_value, active_value, openvdb::TopologyCopy{}));
  typename GridType::Ptr grid(new GridType(tree));
  return Grid<T>{scope.add_value<typename GridType::Ptr>(std::move(grid))};
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

GGrid::operator bool() const
{
  return false;
}

int64_t GGrid::voxel_count() const
{
  return 0;
}

bool GGrid::is_empty() const
{
  return true;
}

GGrid GGrid::create(ResourceScope & /*scope*/,
                    const CPPType & /*type*/,
                    const int64_t /*voxel_count*/)
{
  return GGrid{};
}

GGrid GGrid::create(ResourceScope & /*scope*/,
                    const CPPType & /*type*/,
                    const void * /*background_value*/)
{
  return GGrid{};
}

GGrid GGrid::create(ResourceScope & /*scope*/,
                    const CPPType & /*type*/,
                    const GridMask & /*mask*/,
                    const void * /*inactive_value*/,
                    const void * /*active_value*/)
{
  return GGrid{};
}

template<typename T>
Grid<T> Grid<T>::create(ResourceScope & /*scope*/, const T & /*background_value*/)
{
  return Grid<T>{};
}

template<typename T> Grid<T> Grid<T>::create(ResourceScope & /*scope*/)
{
  return Grid<T>{};
}

template<typename T>
Grid<T> Grid<T>::create(ResourceScope & /*scope*/,
                        const GridMask & /*mask*/,
                        const T & /*inactive_value*/,
                        const T & /*active_value*/)
{
  return Grid<T>{};
}

template<typename T> int64_t Grid<T>::voxel_count() const
{
  return 0;
}

template<typename T> bool Grid<T>::is_empty() const
{
  return true;
}

template<typename T> operator Grid<T>::bool() const
{
  return false;
}
#endif

}  // namespace blender::volume
