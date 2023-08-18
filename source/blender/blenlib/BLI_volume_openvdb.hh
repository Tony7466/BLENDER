/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 */

#include "BLI_cpp_type.hh"
#include "BLI_math_base.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_parameter_pack_utils.hh"
#include "BLI_volume.hh"

#ifdef WITH_OPENVDB
#  include <openvdb/openvdb.h>
#endif

/**
 * Warning: Avoid including this inside header files.
 * OpenVDB is heavily templated and exposing this can affect build times severely.
 */

namespace blender::volume {

#ifdef WITH_OPENVDB

template<typename T> Grid<T> GGrid::typed() const
{
#  ifdef WITH_OPENVDB
  using GridType = typename Grid<T>::GridType;
  using GridPtr = typename Grid<T>::GridConstPtr;

  if (!grid_) {
    return {};
  }
  GridPtr typed_grid = openvdb::GridBase::grid<GridType>(grid_);
  if (!typed_grid) {
    return {};
  }
  return {typed_grid};
#  else
  return {};
#  endif
}

template<typename T> MutableGrid<T> GMutableGrid::typed() const
{
#  ifdef WITH_OPENVDB
  using TypedGrid = typename MutableGrid<T>::GridType;
  using TypedGridPtr = typename MutableGrid<T>::GridPtr;

  if (!grid_) {
    return {};
  }
  TypedGridPtr typed_grid = openvdb::GridBase::grid<TypedGrid>(grid_);
  if (!typed_grid) {
    return {};
  }
  return {typed_grid};
#  else
  return {};
#  endif
}

namespace detail {

template<typename Func> struct FilterVoidOp {
  Func func;
  void operator()(TypeTag<void> /*type_tag*/) const {}
  template<typename T> void operator()(TypeTag<T> type_tag) const
  {
    func(type_tag);
  }
};

/* Helper function to turn a tuple into a parameter pack by means of the dummy argument. */
template<typename... Types, typename Func>
void field_to_static_type_resolve(std::tuple<Types...> /*dummy*/, const CPPType &type, Func func)
{
  FilterVoidOp<Func> wrapper{func};
  type.to_static_type_tag<Types...>(wrapper);
}

}  // namespace detail

/* Helper function to evaluate a function with a static field type. */
template<typename Func> void field_to_static_type(const CPPType &type, Func func)
{
  detail::field_to_static_type_resolve(grid_types::SupportedAttributeValueTypes(), type, func);
}

/* Helper function to evaluate a function with a static field type. */
template<typename Func>
void grid_to_static_type(const std::shared_ptr<openvdb::GridBase> &grid, Func func)
{
  grid->apply<grid_types::SupportedGridTypes>(func);
}

/* Helper function to evaluate a function with a static field type. */
template<typename Func>
void grid_to_static_type(const std::shared_ptr<const openvdb::GridBase> &grid, Func func)
{
  grid->apply<grid_types::SupportedGridTypes>(func);
}

template<typename T> MutableGrid<T> MutableGrid<T>::create(const T &background_value)
{
  typename GridType::Ptr grid = GridType::create(
      Converter::single_value_to_grid(background_value));
  return MutableGrid<T>{std::move(grid)};
}

template<typename T> MutableGrid<T> MutableGrid<T>::create()
{
  const T &value = *CPPType::get<T>().default_value_;
  typename GridType::Ptr grid = GridType::create(Converter::single_value_to_grid(value));
  return MutableGrid<T>{std::move(grid)};
}

template<typename T>
MutableGrid<T> MutableGrid<T>::create(const GGrid &mask,
                                      const T &inactive_value,
                                      const T &active_value)
{
  if (mask.is_empty()) {
    typename GridType::Ptr grid = GridType::create();
    return MutableGrid<T>{std::move(grid)};
  }

  const typename TreeType::Ptr tree = nullptr;
  volume::grid_to_static_type(mask.grid_, [&](auto &typed_mask) {
    tree = typename TreeType::Ptr(new TreeType(typed_mask.grid_->tree(),
                                               Converter::single_value_to_grid(inactive_value),
                                               Converter::single_value_to_grid(active_value),
                                               openvdb::TopologyCopy{}));
  });
  typename GridType::Ptr grid(new GridType(tree));
  return MutableGrid<T>{std::move(grid)};
}

template<typename T> int64_t MutableGrid<T>::voxel_count() const
{
  return grid_ ? grid_->activeVoxelCount() : 0;
}

template<typename T> bool MutableGrid<T>::is_empty() const
{
  return grid_ ? grid_->empty() : true;
}

template<typename T> MutableGrid<T>::operator bool() const
{
  return grid_ != nullptr;
}

template<typename T> const CPPType *MutableGrid<T>::value_type() const
{
  return &CPPType::get<T>();
}

template<typename T> const CPPType *Grid<T>::value_type() const
{
  return &CPPType::get<T>();
}

template<typename T> Grid<T>::operator GGrid()
{
  return {grid_};
}
template<typename T> Grid<T>::operator GGrid const() const
{
  return {grid_};
}

template<typename T> int64_t Grid<T>::voxel_count() const
{
  return grid_ ? grid_->activeVoxelCount() : 0;
}

template<typename T> bool Grid<T>::is_empty() const
{
  return grid_ ? grid_->empty() : true;
}

template<typename T> Grid<T>::operator bool() const
{
  return grid_ != nullptr;
}

#else

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

template<typename T> Grid<T>::operator bool() const
{
  return false;
}

template<typename T> const CPPType *Grid<T>::value_type() const
{
  return nullptr;
}

template<typename T> Grid<T>::operator GGrid()
{
  return {};
}

template<typename T> Grid<T>::operator GGrid const() const
{
  return {};
}

#endif

}  // namespace blender::volume
