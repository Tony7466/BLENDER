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
#  include <openvdb/tree/Tree.h>
#endif

/**
 * Warning: Avoid including this inside header files.
 * OpenVDB is heavily templated and exposing this can affect build times severely.
 */

namespace blender::volume {

#ifdef WITH_OPENVDB

inline openvdb::math::Mat4f from_m4(const float4x4 &mat)
{
  /* Blender column-major and OpenVDB right-multiplication conventions match. */
  openvdb::math::Mat4f vdb_mat;
  for (int col = 0; col < 4; col++) {
    for (int row = 0; row < 4; row++) {
      vdb_mat(col, row) = mat[col][row];
    }
  }
  return vdb_mat;
}

inline float4x4 to_m4(const openvdb::math::Mat4f &vdb_mat)
{
  /* Blender column-major and OpenVDB right-multiplication conventions match. */
  float4x4 mat;
  for (int col = 0; col < 4; col++) {
    for (int row = 0; row < 4; row++) {
      mat[col][row] = vdb_mat(col, row);
    }
  }
  return mat;
}

inline openvdb::math::Transform::Ptr transform_from_m4(const float4x4 &mat)
{
  openvdb::math::Mat4f vdb_mat = from_m4(mat);
  return std::make_shared<openvdb::math::Transform>(
      std::make_shared<openvdb::math::AffineMap>(vdb_mat));
}

inline float4x4 transform_to_m4(const openvdb::math::Transform &transform)
{
  /* Perspective not supported for now, getAffineMap() will leave out the
   * perspective part of the transform. */
  openvdb::math::Mat4f vdb_mat = transform.baseMap()->getAffineMap()->getMat4();
  return to_m4(vdb_mat);
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
template<typename Func> void grid_to_static_type(openvdb::GridBase &grid, Func func)
{
  grid.apply<grid_types::SupportedGridTypes>(func);
}

/* Helper function to evaluate a function with a static field type. */
template<typename Func> void grid_to_static_type(const openvdb::GridBase &grid, Func func)
{
  grid.apply<grid_types::SupportedGridTypes>(func);
}

template<typename GridType> const CPPType &grid_attribute_type(const GridType & /*grid*/)
{
  using Converter = grid_types::Converter<GridType>;
  using AttributeValueType = typename Converter::AttributeValueType;

  return CPPType::get<AttributeValueType>();
}

const CPPType &grid_base_attribute_type(const openvdb::GridBase &grid);

GVArray get_varray_for_leaf(uint32_t log2dim, const int3 &origin, const openvdb::GridBase &grid)
{
  const uint32_t num_voxels = 1 << 3 * log2dim;

  GVArray result = {};
  grid_to_static_type(grid, [&](auto &typed_grid) {
    using GridType = typename std::decay<decltype(typed_grid)>::type;
    using TreeType = typename GridType::TreeType;
    using Accessor = typename GridType::ConstAccessor;
    using Converter = volume::grid_types::Converter<GridType>;
    using AttributeValueType = typename Converter::AttributeValueType;

    Accessor accessor = typed_grid.getAccessor();
    result = VArray<AttributeValueType>::ForFunc(
        num_voxels, [log2dim, origin, &accessor](const int64_t index) {
          const openvdb::Coord xyz = volume::offset_to_global_coord(
              log2dim, origin, int32_t(index));
          return Converter::single_value_to_attribute(accessor.getValue(xyz));
        });
  });
  return result;
}

#else

GVArray get_varray_for_leaf(uint32_t /*log2dim*/,
                            const int3 & /*origin*/,
                            const openvdb::GridBase & /*grid*/)
{
  return {};
}

#endif

}  // namespace blender::volume
