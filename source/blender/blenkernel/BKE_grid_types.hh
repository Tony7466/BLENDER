/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 */

#include "BLI_color.hh"
#include "BLI_cpp_type.hh"
#include "BLI_math_base.hh"
#include "BLI_math_matrix_types.hh"
#include "BLI_math_quaternion_types.hh"
#include "BLI_math_vector_types.hh"

/* Note: version header included here to enable correct forward declaration of some types. No other
 * OpenVDB headers should be included here, especially openvdb.h, to avoid affecting other
 * compilation units. */
#ifdef WITH_OPENVDB
#  include <openvdb/Types.h>
#  include <openvdb/version.h>
#endif

/* -------------------------------------------------------------------- */
/** \name OpenVDB Forward Declaration
 * \{ */

#ifdef WITH_OPENVDB

/* Forward declaration for basic OpenVDB types. */
namespace openvdb {
OPENVDB_USE_VERSION_NAMESPACE
namespace OPENVDB_VERSION_NAME {

class GridBase;
class MetaMap;
template<typename TreeType> class Grid;

namespace math {
class Transform;
}

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

namespace tools {
template<typename T, Index Log2Dim> struct PointIndexLeafNode;
using PointIndexTree = tree::Tree<tree::RootNode<
    tree::InternalNode<tree::InternalNode<PointIndexLeafNode<PointIndex32, 3>, 4>, 5>>>;
using PointIndexGrid = Grid<PointIndexTree>;
}  // namespace tools

namespace points {
template<typename T, Index Log2Dim> class PointDataLeafNode;
using PointDataTree = tree::Tree<tree::RootNode<
    tree::InternalNode<tree::InternalNode<PointDataLeafNode<PointDataIndex32, 3>, 4>, 5>>>;
using PointDataGrid = Grid<PointDataTree>;
struct NullCodec;
template<typename ValueType, typename Codec> class TypedAttributeArray;
}  // namespace points

/// Common tree types
using BoolTree = tree::Tree4Fwd<bool, 5, 4, 3>::Type;
using DoubleTree = tree::Tree4Fwd<double, 5, 4, 3>::Type;
using FloatTree = tree::Tree4Fwd<float, 5, 4, 3>::Type;
using Int8Tree = tree::Tree4Fwd<int8_t, 5, 4, 3>::Type;
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
using Vec4STree = tree::Tree4Fwd<Vec4f, 5, 4, 3>::Type;
using ScalarTree = FloatTree;
using TopologyTree = MaskTree;
using Vec3dTree = Vec3DTree;
using Vec3fTree = Vec3STree;
using Vec4fTree = Vec4STree;
using VectorTree = Vec3fTree;

/// Common grid types
using BoolGrid = Grid<BoolTree>;
using DoubleGrid = Grid<DoubleTree>;
using FloatGrid = Grid<FloatTree>;
using Int8Grid = Grid<Int8Tree>;
using Int32Grid = Grid<Int32Tree>;
using Int64Grid = Grid<Int64Tree>;
using UInt32Grid = Grid<UInt32Tree>;
using MaskGrid = Grid<MaskTree>;
using Vec3DGrid = Grid<Vec3DTree>;
using Vec2IGrid = Grid<Vec2ITree>;
using Vec3IGrid = Grid<Vec3ITree>;
using Vec2SGrid = Grid<Vec2STree>;
using Vec3SGrid = Grid<Vec3STree>;
using Vec4SGrid = Grid<Vec4STree>;
using ScalarGrid = FloatGrid;
using TopologyGrid = MaskGrid;
using Vec3dGrid = Vec3DGrid;
using Vec2fGrid = Vec2SGrid;
using Vec3fGrid = Vec3SGrid;
using Vec4fGrid = Vec4SGrid;
using VectorGrid = Vec3fGrid;

}  // namespace OPENVDB_VERSION_NAME
}  // namespace openvdb

#endif /* WITH_OPENVDB */

/** \} */

/* -------------------------------------------------------------------- */
/** \name Attribute Grid
 *
 * Grid class template using Blender data types.
 * \{ */

namespace blender::bke::grid_types {

#ifdef WITH_OPENVDB

using SupportedAttributeValueTypes = std::tuple<bool,
                                                float,
                                                float2,
                                                float3,
                                                /*float4,*/
                                                int32_t,
                                                int64_t,
                                                int2,
                                                /*int3,*/ /*int4,*/ uint32_t
                                                /*,uint2*/
                                                /*,uint3*/
                                                /*,uint4*/>;

using SupportedPointGridAttributeTypes = openvdb::TypeList<
    openvdb::points::TypedAttributeArray<bool, openvdb::points::NullCodec>,
    openvdb::points::TypedAttributeArray<float, openvdb::points::NullCodec>,
    openvdb::points::TypedAttributeArray<openvdb::Vec2f, openvdb::points::NullCodec>,
    openvdb::points::TypedAttributeArray<openvdb::Vec3f, openvdb::points::NullCodec>,
    openvdb::points::TypedAttributeArray<int32_t, openvdb::points::NullCodec>,
    openvdb::points::TypedAttributeArray<int64_t, openvdb::points::NullCodec>,
    //    openvdb::points::TypedAttributeArray<openvdb::Vec2I, openvdb::points::NullCodec>,
    openvdb::points::TypedAttributeArray<uint32_t, openvdb::points::NullCodec>>;

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

/**
 * Tree types for commonly used values.
 */
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
template<> struct AttributeTreeImpl<int8_t> {
  using Type = openvdb::Int8Grid;
};
template<> struct AttributeTreeImpl<int32_t> {
  using Type = openvdb::Int32Tree;
};
template<> struct AttributeTreeImpl<int64_t> {
  using Type = openvdb::Int64Tree;
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
template<> struct AttributeTreeImpl<ColorGeometry4f> {
  using Type = openvdb::Vec4fTree;
};
template<> struct AttributeTreeImpl<blender::ColorGeometry4b> {
  using Type = openvdb::UInt32Tree;
};
template<> struct AttributeTreeImpl<math::Quaternion> {
  using Type = openvdb::Vec4fTree;
};
/* Stub class for string attributes, not supported. */
template<> struct AttributeTreeImpl<std::string> {
  using Type = openvdb::MaskTree;
};

template<typename T> using AttributeTree = typename AttributeTreeImpl<T>::Type;
template<typename T> using AttributeGrid = typename openvdb::Grid<AttributeTree<T>>;

#else /* WITH_OPENVDB */

template<typename T> struct AttributeTree {
};
template<typename T> struct AttributeGrid {
};

#endif /* WITH_OPENVDB */

}  // namespace blender::bke::grid_types

/** \} */

/* -------------------------------------------------------------------- */
/** \name Grid Type Converter
 *
 * Converter between Blender types and OpenVDB types.
 * \{ */

namespace blender::bke::grid_types {

#ifdef WITH_OPENVDB

namespace detail {

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

template<typename AttributeValueType, typename GridValueType, typename LeafBufferValueType>
struct CopyConstructConverter {
  static GridValueType single_value_to_grid(const AttributeValueType &value)
  {
    return GridValueType(value);
  }

  static AttributeValueType single_value_to_attribute(const GridValueType &value)
  {
    return AttributeValueType(value);
  }

  // XXX can't do this, will only work as a DerivedSpan varray.
  //  static MutableSpan<AttributeValueType> leaf_buffer_to_varray(
  //      const MutableSpan<LeafBufferValueType> values)
  //  {
  //    return {reinterpret_cast<AttributeValueType *>(values.begin()), values.size()};
  //  }
};

template<typename AttributeValueType, typename GridValueType, typename LeafBufferValueType>
struct Vector3CopyConstructConverter {
  using GridBaseType = typename GridValueType::ValueType;
  using AttributeBaseType = typename AttributeValueType::base_type;

  static GridValueType single_value_to_grid(const AttributeValueType &value)
  {
    return GridValueType(GridBaseType(value[0]), GridBaseType(value[1]), GridBaseType(value[2]));
  }

  static AttributeValueType single_value_to_attribute(const GridValueType &value)
  {
    return AttributeValueType(
        AttributeBaseType(value[0]), AttributeBaseType(value[1]), AttributeBaseType(value[2]));
  }

  // XXX can't do this, will only work as a DerivedSpan varray.
  //  static MutableSpan<AttributeValueType> leaf_buffer_to_varray(
  //      const MutableSpan<LeafBufferValueType> values)
  //  {
  //    return {reinterpret_cast<AttributeValueType *>(values.begin()), values.size()};
  //  }
};

}  // namespace detail

/* Default implementation, only used when grid and attribute types are exactly the same. */
template<typename GridType>
struct Converter : public detail::ReinterpretCastConverter<typename GridType::ValueType,
                                                           typename GridType::ValueType,
                                                           typename GridType::ValueType> {
  using AttributeValueType = typename GridType::ValueType;
};

/* TODO Some base types (double, int3, ...) don't have a CPPType registered yet, and even then
 * there are other layers like attributes, customdata, node sockets, which need to support these
 * types. In the meantime the converters will use smaller base types for fields and attributes. */

#  ifndef USE_CPPTYPE_DOUBLE
template<>
struct Converter<openvdb::DoubleGrid>
    : public detail::CopyConstructConverter<float, double, double> {
  using AttributeValueType = float;
};
#  endif

/* Vector implementation, casts Blender vectors to OpenVDB vectors and vice versa. */
template<>
struct Converter<openvdb::Vec2fGrid>
    : public detail::ReinterpretCastConverter<float2, openvdb::math::Vec2s, openvdb::math::Vec2s> {
  using AttributeValueType = float2;
};

template<>
struct Converter<openvdb::Vec3fGrid>
    : public detail::ReinterpretCastConverter<float3, openvdb::math::Vec3s, openvdb::math::Vec3s> {
  using AttributeValueType = float3;
};

#  ifdef USE_CPPTYPE_DOUBLE3
template<>
struct Converter<openvdb::Vec3DGrid>
    : public detail::
          ReinterpretCastConverter<double3, openvdb::math::Vec3d, openvdb::math::Vec3d> {
  using AttributeValueType = double3;
};
#  else
/* Store double3 data as float3 */
template<>
struct Converter<openvdb::Vec3DGrid>
    : public detail::
          Vector3CopyConstructConverter<float3, openvdb::math::Vec3d, openvdb::math::Vec3d> {
  using AttributeValueType = float3;
};
#  endif

template<>
struct Converter<openvdb::Vec2IGrid>
    : public detail::ReinterpretCastConverter<int2, openvdb::math::Vec2i, openvdb::math::Vec2i> {
  using AttributeValueType = int2;
};

#  ifdef USE_CPPTYPE_INT3
template<>
struct Converter<openvdb::Vec3IGrid>
    : public detail::ReinterpretCastConverter<int3, openvdb::math::Vec3i, openvdb::math::Vec3i> {
  using AttributeValueType = int3;
};
#  else
/* Store int3 data as float3 */
template<>
struct Converter<openvdb::Vec3IGrid>
    : public detail::
          Vector3CopyConstructConverter<float3, openvdb::math::Vec3i, openvdb::math::Vec3i> {
  using AttributeValueType = float3;
};
#  endif

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

#endif /* WITH_OPENVDB */

}  // namespace blender::bke::grid_types

/** \} */
