/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 */

#include "BLI_cpp_type.hh"
#include "BLI_generic_virtual_array.hh"
#include "BLI_math_base.hh"
#include "BLI_math_matrix_types.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_parameter_pack_utils.hh"

/* Note: version header included here to enable correct forward declaration of some types. No other
 * OpenVDB headers should be included here, especially openvdb.h, to avoid affecting other
 * compilation units. */
#ifdef WITH_OPENVDB
#  include <openvdb/Types.h>
#  include <openvdb/version.h>
#endif

#ifdef WITH_OPENVDB
/* Forward declaration for basic OpenVDB types. */
namespace openvdb {
OPENVDB_USE_VERSION_NAMESPACE
namespace OPENVDB_VERSION_NAME {

class GridBase;
class MetaMap;
template<typename TreeType> class Grid;

namespace tree {
class TreeBase;
template<typename T, Index Log2Dim> class LeafNode;
template<typename ChildNodeType, Index Log2Dim> class InternalNode;
template<typename ChildNodeType> class RootNode;
template<typename RootNodeType> class Tree;

/* Forward-declared version of Tree4, can't use the actual Tree4 alias because it can't be
 * forward-declared. */
template<typename T, Index N1 = 5, Index N2 = 4, Index N3 = 3> struct Tree4Fwd {
  using Type = openvdb::tree::Tree<openvdb::tree::RootNode<
      openvdb::tree::InternalNode<openvdb::tree::InternalNode<openvdb::tree::LeafNode<T, N3>, N2>,
                                  N1>>>;
};
}  // namespace tree
namespace points {
template<typename T, Index Log2Dim> class PointDataLeafNode;
using PointDataTree = tree::Tree<tree::RootNode<
    tree::InternalNode<tree::InternalNode<PointDataLeafNode<PointDataIndex32, 3>, 4>, 5>>>;
using PointDataGrid = Grid<PointDataTree>;
}  // namespace points

/// Common tree types
using BoolTree = tree::Tree4Fwd<bool, 5, 4, 3>::Type;
using DoubleTree = tree::Tree4Fwd<double, 5, 4, 3>::Type;
using FloatTree = tree::Tree4Fwd<float, 5, 4, 3>::Type;
using Int32Tree = tree::Tree4Fwd<int32_t, 5, 4, 3>::Type;
using Int64Tree = tree::Tree4Fwd<int64_t, 5, 4, 3>::Type;
using MaskTree = tree::Tree4Fwd<ValueMask, 5, 4, 3>::Type;
using UInt32Tree = tree::Tree4Fwd<uint32_t, 5, 4, 3>::Type;
using Vec2DTree = tree::Tree4Fwd<Vec2d, 5, 4, 3>::Type;
using Vec2ITree = tree::Tree4Fwd<Vec2i, 5, 4, 3>::Type;
using Vec2STree = tree::Tree4Fwd<Vec2s, 5, 4, 3>::Type;
using Vec3DTree = tree::Tree4Fwd<Vec3d, 5, 4, 3>::Type;
using Vec3ITree = tree::Tree4Fwd<Vec3i, 5, 4, 3>::Type;
using Vec3STree = tree::Tree4Fwd<Vec3f, 5, 4, 3>::Type;
using ScalarTree = FloatTree;
using TopologyTree = MaskTree;
using Vec3dTree = Vec3DTree;
using Vec3fTree = Vec3STree;
using VectorTree = Vec3fTree;

/// Common grid types
using BoolGrid = Grid<BoolTree>;
using DoubleGrid = Grid<DoubleTree>;
using FloatGrid = Grid<FloatTree>;
using Int32Grid = Grid<Int32Tree>;
using Int64Grid = Grid<Int64Tree>;
using UInt32Grid = Grid<UInt32Tree>;
using MaskGrid = Grid<MaskTree>;
using Vec3DGrid = Grid<Vec3DTree>;
using Vec2IGrid = Grid<Vec2ITree>;
using Vec3IGrid = Grid<Vec3ITree>;
using Vec2SGrid = Grid<Vec2STree>;
using Vec3SGrid = Grid<Vec3STree>;
using ScalarGrid = FloatGrid;
using TopologyGrid = MaskGrid;
using Vec3dGrid = Vec3DGrid;
using Vec2fGrid = Vec2SGrid;
using Vec3fGrid = Vec3SGrid;
using VectorGrid = Vec3fGrid;

}  // namespace OPENVDB_VERSION_NAME
}  // namespace openvdb
#endif

namespace blender {
class ResourceScope;
}

namespace blender::volume {

#ifdef WITH_OPENVDB
inline int32_t coord_to_offset(uint32_t log2dim, const openvdb::Coord &xyz)
{
  /* Dimension along one coordinate direction. */
  const uint32_t DIM = 1 << log2dim;

  BLI_assert((xyz[0] & (DIM - 1u)) < DIM && (xyz[1] & (DIM - 1u)) < DIM &&
             (xyz[2] & (DIM - 1u)) < DIM);
  return ((xyz[0] & (DIM - 1u)) << 2 * log2dim) + ((xyz[1] & (DIM - 1u)) << log2dim) +
         (xyz[2] & (DIM - 1u));
}

inline openvdb::Coord offset_to_local_coord(uint32_t log2dim, int n)
{
  BLI_assert(n < (1 << 3 * log2dim));
  openvdb::Coord xyz;
  xyz.setX(n >> 2 * log2dim);
  n &= ((1 << 2 * log2dim) - 1);
  xyz.setY(n >> log2dim);
  xyz.setZ(n & ((1 << log2dim) - 1));
  return xyz;
}

inline openvdb::Coord offset_to_global_coord(uint32_t log2dim, const int3 &origin, int n)
{
  return offset_to_local_coord(log2dim, n) + openvdb::Coord(origin);
}

namespace grid_types {

/* XXX Generic template for attribute grids, this can cause excessive code generation,
 * so the allowed types are declared below to be on the safe side.
 */
// template<typename T> using AttributeTree = typename openvdb::tree::Tree4Fwd<T, 5, 4, 3>::Type;
// template<typename T> using AttributeGrid = openvdb::Grid<AttributeTree<T>>;

template<typename T> struct AttributeTreeImpl;
template<> struct AttributeTreeImpl<bool> {
  using Type = openvdb::BoolTree;
};
template<> struct AttributeTreeImpl<float> {
  using Type = openvdb::FloatTree;
};
template<> struct AttributeTreeImpl<float2> {
  using Type = openvdb::Vec2STree;
};
template<> struct AttributeTreeImpl<float3> {
  using Type = openvdb::Vec3STree;
};
template<> struct AttributeTreeImpl<double> {
  using Type = openvdb::DoubleTree;
};
template<> struct AttributeTreeImpl<double3> {
  using Type = openvdb::Vec3DTree;
};
template<> struct AttributeTreeImpl<int32_t> {
  using Type = openvdb::Int32Tree;
};
template<> struct AttributeTreeImpl<int2> {
  using Type = openvdb::Vec2ITree;
};
template<> struct AttributeTreeImpl<int3> {
  using Type = openvdb::Vec3ITree;
};
template<> struct AttributeTreeImpl<uint32_t> {
  using Type = openvdb::UInt32Tree;
};

template<typename T> using AttributeTree = typename AttributeTreeImpl<T>::Type;
template<typename T> using AttributeGrid = typename openvdb::Grid<AttributeTree<T>>;

template<typename AttributeValueType, typename GridValueType, typename LeafBufferValueType>
struct ReinterpretCastConverter {
  static GridValueType single_value_to_grid(const AttributeValueType &value)
  {
    return reinterpret_cast<const GridValueType &>(value);
  }

  static AttributeValueType single_value_to_attribute(const GridValueType &value)
  {
    return reinterpret_cast<const AttributeValueType &>(value);
  }

  static MutableSpan<AttributeValueType> leaf_buffer_to_varray(
      const MutableSpan<LeafBufferValueType> values)
  {
    return {reinterpret_cast<AttributeValueType *>(values.begin()), values.size()};
  }
};

/* Default implementation, only used when grid and attribute types are exactly the same. */
template<typename GridType>
struct Converter : public ReinterpretCastConverter<typename GridType::ValueType,
                                                   typename GridType::ValueType,
                                                   typename GridType::ValueType> {
  using AttributeValueType = typename GridType::ValueType;
};

/* Vector implementation, casts Blender vectors to OpenVDB vectors and vice versa. */
template<>
struct Converter<openvdb::Vec2fGrid>
    : public ReinterpretCastConverter<float2, openvdb::math::Vec2s, openvdb::math::Vec2s> {
  using AttributeValueType = float2;
};
template<>
struct Converter<openvdb::Vec3fGrid>
    : public ReinterpretCastConverter<float3, openvdb::math::Vec3s, openvdb::math::Vec3s> {
  using AttributeValueType = float3;
};
template<>
struct Converter<openvdb::Vec3DGrid>
    : public ReinterpretCastConverter<double3, openvdb::math::Vec3d, openvdb::math::Vec3d> {
  using AttributeValueType = double3;
};
template<>
struct Converter<openvdb::Vec2IGrid>
    : public ReinterpretCastConverter<int2, openvdb::math::Vec2i, openvdb::math::Vec2i> {
  using AttributeValueType = int2;
};
template<>
struct Converter<openvdb::Vec3IGrid>
    : public ReinterpretCastConverter<int3, openvdb::math::Vec3i, openvdb::math::Vec3i> {
  using AttributeValueType = int3;
};

/* Specialization for MaskGrid: Leaf buffers directly expose the activation state bit fields. */
template<> struct Converter<openvdb::MaskGrid> {
  using GridValueType = openvdb::ValueMask;
  // using LeafBufferValueType = unsigned long int;
  using LeafBufferValueType = uint64_t;
  using AttributeValueType = bool;

  static GridValueType single_value_to_grid(const AttributeValueType & /*value*/)
  {
    return {};
  }

  /* MaskGrid accessor also returns a bool. */
  static AttributeValueType single_value_to_attribute(const bool value)
  {
    return value;
  }

  static MutableSpan<AttributeValueType> leaf_buffer_to_varray(
      const MutableSpan<LeafBufferValueType> values)
  {
    return {reinterpret_cast<AttributeValueType *>(values.begin()), values.size()};
  }
};

/* Specialization for BoolGrid: Leaf buffers directly expose the activation state bit fields. */
template<> struct Converter<openvdb::BoolGrid> {
  using GridValueType = bool;
  // using LeafBufferValueType = unsigned long int;
  using LeafBufferValueType = uint64_t;
  using AttributeValueType = bool;

  static GridValueType single_value_to_grid(const AttributeValueType &value)
  {
    return value;
  }

  static AttributeValueType single_value_to_attribute(const AttributeValueType &value)
  {
    return value;
  }

  static MutableSpan<AttributeValueType> leaf_buffer_to_varray(
      const MutableSpan<LeafBufferValueType> values)
  {
    return {reinterpret_cast<AttributeValueType *>(values.begin()), values.size()};
  }
};

/* TODO add more as needed. */
/* TODO some types disabled because of missing CPPType registration. */

using SupportedAttributeValueTypes = std::tuple<bool,
                                                float,
                                                float2,
                                                float3,
                                                /*float4,*/
                                                int32_t,
                                                int2,
                                                /*int3,*/ /*int4,*/ uint32_t
                                                /*,uint2*/
                                                /*,uint3*/
                                                /*,uint4*/>;

using SupportedGridTypes = openvdb::TypeList<openvdb::BoolGrid,
                                             openvdb::MaskGrid,
                                             openvdb::FloatGrid,
                                             openvdb::Vec2fGrid,
                                             openvdb::Vec3fGrid,
                                             /*openvdb::Vec4fGrid,*/
                                             openvdb::Int32Grid,
                                             openvdb::Vec2IGrid,
                                             /*openvdb::Vec3IGrid,*/
                                             /*openvdb::Vec4IGrid,*/
                                             openvdb::UInt32Grid,
                                             /*openvdb::Vec2UInt32Grid,*/
                                             /*openvdb::Vec3UInt32Grid,*/
                                             /*openvdb::Vec4UInt32Grid,*/
                                             openvdb::ScalarGrid,
                                             openvdb::TopologyGrid>;

}  // namespace grid_types

std::shared_ptr<openvdb::tree::TreeBase> make_tree_for_attribute_type(ResourceScope &scope,
                                                                      const CPPType &type,
                                                                      const void *value = nullptr);
std::shared_ptr<openvdb::tree::TreeBase> make_tree_for_attribute_type(const CPPType &type,
                                                                      const void *value = nullptr);

openvdb::GridBase *make_grid_for_attribute_type(ResourceScope &scope,
                                                const CPPType &type,
                                                const float4x4 &transform,
                                                const void *value = nullptr);
openvdb::GridBase *make_grid_for_attribute_type(const CPPType &type,
                                                const float4x4 &transform,
                                                const void *value = nullptr);

GVArray get_varray_for_leaf(uint32_t log2dim, const int3 &origin, const openvdb::GridBase &grid);

#else
//
// inline GVArray get_varray_for_leaf(uint32_t log2dim,
//                                   const int3 &origin,
//                                   const openvdb::GridBase &grid)
//{
//  return {};
//}

#endif

}  // namespace blender::volume
