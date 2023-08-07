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

namespace blender {

namespace volume {

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
  detail::field_to_static_type_resolve(grid_types::SupportedGridValueTypes(), type, func);
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

#endif

}  // namespace volume

}  // namespace blender
