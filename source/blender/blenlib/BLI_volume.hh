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
template<typename TreeType> class Grid;
class MetaMap;

namespace tree {
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
// template<typename IntType_, Index Kind> struct PointIndex;
// using PointDataIndex32 = PointIndex<Index32, 1>;
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

///* XXX OpenVDB expects some math functions on vector types. */
// template<typename T, int Size> inline VecBase<T, Size> Abs(VecBase<T, Size> v)
//{
//   VecBase<T, Size> r;
//   for (int i = 0; i < Size; i++) {
//     r[i] = math::abs(v[i]);
//   }
//   return r;
// }
///* Specialization: math::abs is not defined for unsigned types. */
// template<int Size> inline VecBase<uint32_t, Size> Abs(VecBase<uint32_t, Size> v)
//{
//   return v;
// }

namespace volume {

#ifdef WITH_OPENVDB
namespace grid_types {

// template<typename GridValueType, typename MFValueType> struct ConverterImpl {
//  static GridValueType to_grid(const MFValueType &value)
//  {
//    return reinterpret_cast<const GridValueType &>(value);
//  }
//  static Span<GridValueType> to_grid(const Span<MFValueType> values)
//  {
//    return {reinterpret_cast<const GridValueType *>(values.begin()), values.size()};
//  }
//  static MutableSpan<GridValueType> to_grid(const MutableSpan<MFValueType> values)
//  {
//    return {reinterpret_cast<GridValueType *>(values.begin()), values.size()};
//  }

//  static MFValueType to_attribute(const GridValueType &value)
//  {
//    return reinterpret_cast<const MFValueType &>(value);
//  }
//  static Span<MFValueType> to_attribute(const Span<GridValueType> values)
//  {
//    return {reinterpret_cast<const MFValueType *>(values.begin()), values.size()};
//  }
//  static MutableSpan<MFValueType> to_attribute(const MutableSpan<GridValueType> values)
//  {
//    return {reinterpret_cast<MFValueType *>(values.begin()), values.size()};
//  }
//};

///* No-op converter utility: OpenVDB types are interpreted as their Blender counterpart so they
/// can
// * be used directly with attributes.
// * Supported types are instantiated explicitly below.
// */
// template<typename T> struct AttributeValueConverter : public ConverterImpl<T, T> {
//  using ValueType = T;
//};

///* No-op converter utility: OpenVDB types are interpreted as their Blender counterpart so they
/// can
// * be used directly with attributes.
// * Supported types are instantiated explicitly below.
// *
// * GridValueConverter uses the GridType rather than ValueType as argument.
// * This is because MaskGrid has a bool value type, but any leaf buffer arrays
// * are actually of type ValueMask, which is an empty dummy type.
// * A specialization for MaskGrid is implemented below.
// */
// template<typename GridType>
// struct GridValueConverter
//    : public ConverterImpl<typename GridType::ValueType, typename GridType::ValueType> {
//  using ValueType = typename GridType::ValueType;
//  using MFType = ValueType;
//};

///* Specialization for MaskGrid, converting between bool and ValueMask. */
// template<>
// struct GridValueConverter<openvdb::MaskGrid> : public ConverterImpl<long unsigned int, bool> {
//  using ValueType = bool;
//  using MFType = long unsigned int;
//};

template<typename GridType> struct Converter {
  using LeafBufferValueType = typename GridType::ValueType;
  using AttributeValueType = typename GridType::ValueType;

  static MutableSpan<AttributeValueType> leaf_buffer_to_varray(
      const MutableSpan<LeafBufferValueType> values)
  {
    return {reinterpret_cast<AttributeValueType *>(values.begin()), values.size()};
  }
};

/* Specialization for MaskGrid: Leaf buffers directly expose the activation state bit fields. */
template<> struct Converter<openvdb::MaskGrid> {
  using LeafBufferValueType = unsigned long int;
  using AttributeValueType = bool;

  static MutableSpan<AttributeValueType> leaf_buffer_to_varray(
      const MutableSpan<LeafBufferValueType> values)
  {
    return {reinterpret_cast<AttributeValueType *>(values.begin()), values.size()};
  }
};

/* Specialization for BoolGrid: Leaf buffers directly expose the activation state bit fields. */
template<> struct Converter<openvdb::BoolGrid> {
  using LeafBufferValueType = unsigned long int;
  using AttributeValueType = bool;

  static MutableSpan<AttributeValueType> leaf_buffer_to_varray(
      const MutableSpan<LeafBufferValueType> values)
  {
    return {reinterpret_cast<AttributeValueType *>(values.begin()), values.size()};
  }
};

template<typename T> using AttributeTree = typename openvdb::tree::Tree4Fwd<T, 5, 4, 3>::Type;
template<typename T> using AttributeGrid = openvdb::Grid<AttributeTree<T>>;

/* TODO add more as needed. */
/* TODO could use template magic to generate all from 1 list, but not worth it for now. */
/* TODO some types disabled because of missing CPPType registration. */

// using BoolTree = TreeCommon<bool>;
// using MaskTree = TreeCommon<openvdb::ValueMask>;
// using FloatTree = TreeCommon<float>;
// using Float2Tree = TreeCommon<float2>;
// using Float3Tree = TreeCommon<float3>;
//// using Float4Tree = TreeCommon<float4>;
// using DoubleTree = TreeCommon<double>;
// using Double3Tree = TreeCommon<double3>;
// using IntTree = TreeCommon<int32_t>;
// using Int2Tree = TreeCommon<int2>;
// using Int3Tree = TreeCommon<int3>;
//// using Int4Tree = TreeCommon<int4>;
// using UIntTree = TreeCommon<uint32_t>;
//// using UInt2Tree = TreeCommon<uint2>;
//// using UInt3Tree = TreeCommon<uint3>;
//// using UInt4Tree = TreeCommon<uint4>;
// using ScalarTree = FloatTree;
// using TopologyTree = MaskTree;
//
// using BoolGrid = openvdb::Grid<BoolTree>;
// using MaskGrid = openvdb::Grid<MaskTree>;
// using FloatGrid = openvdb::Grid<FloatTree>;
// using Float2Grid = openvdb::Grid<Float2Tree>;
// using Float3Grid = openvdb::Grid<Float3Tree>;
//// using Float4Grid = openvdb::Grid<Float4Tree>;
// using DoubleGrid = openvdb::Grid<DoubleTree>;
// using Double3Grid = openvdb::Grid<Double3Tree>;
// using IntGrid = openvdb::Grid<IntTree>;
// using Int2Grid = openvdb::Grid<Int2Tree>;
// using Int3Grid = openvdb::Grid<Int3Tree>;
//// using Int4Grid = openvdb::Grid<Int4Tree>;
// using UIntGrid = openvdb::Grid<UIntTree>;
//// using UInt2Grid = openvdb::Grid<UInt2Tree>;
//// using UInt3Grid = openvdb::Grid<UInt3Tree>;
//// using UInt4Grid = openvdb::Grid<UInt4Tree>;
// using ScalarGrid = openvdb::Grid<ScalarTree>;
// using TopologyGrid = openvdb::Grid<TopologyTree>;
// using PointDataGrid = openvdb::points::PointDataGrid;

using SupportedGridValueTypes = std::tuple<bool,
                                           float,
                                           openvdb::Vec2f,
                                           openvdb::Vec3f,
                                           /*openvdb::Vec4f,*/
                                           int32_t,
                                           openvdb::Vec2i,
                                           /*openvdb::Vec3i,*/ /*openvdb::Vec4i,*/ uint32_t
                                           /*,openvdb::Vec2u*/
                                           /*,openvdb::Vec3u*/
                                           /*,openvdb::Vec4u*/>;

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
#endif

class GGrid;
class GMutableGrid;
template<typename T> class Grid;
template<typename T> class MutableGrid;

/* Mask defined by active voxels of the grid. */
class GridMask {
#ifdef WITH_OPENVDB
  using GridPtr = std::shared_ptr<openvdb::MaskGrid>;
  GridPtr grid_;

  static GridPtr empty_grid();
#endif

 public:
  GridMask()
#ifdef WITH_OPENVDB
      : grid_(empty_grid())
#endif
  {
  }

  GridMask(const GridMask &other) = default;
  GridMask &operator=(const GridMask &other)
  {
#ifdef WITH_OPENVDB
    grid_ = other.grid_;
#else
    UNUSED_VARS(other);
#endif
    return *this;
  }

#ifdef WITH_OPENVDB
  GridMask(const GridPtr &grid) : grid_(grid) {}
#endif

  static GridMask from_bools(const GGrid &full_mask, const Grid<bool> &selection);

  bool is_empty() const;
  int64_t min_voxel_count() const;

#ifdef WITH_OPENVDB
  const GridPtr &grid() const
  {
    return grid_;
  }
#endif
};

/* -------------------------------------------------------------------- */
/** \name Grid pointer wrappers
 *  \note Using wrappers avoids checking for WITH_OPENVDB everywhere.
 * \{ */

/* Generic grid reference. */
class GGrid {
 public:
#ifdef WITH_OPENVDB
  using GridPtr = std::shared_ptr<const openvdb::GridBase>;
  GridPtr grid_ = nullptr;
#endif

  int64_t voxel_count() const;
  bool is_empty() const;
  operator bool() const;

  const CPPType *value_type() const;

  template<typename T> Grid<T> typed() const;
};

/* Generic grid reference. */
class GMutableGrid {
 public:
#ifdef WITH_OPENVDB
  using GridPtr = std::shared_ptr<openvdb::GridBase>;
  GridPtr grid_ = nullptr;
#endif

  operator GGrid() const
  {
#ifdef WITH_OPENVDB
    return GGrid{grid_};
#else
    return GGrid{};
#endif
  }

  /* Create an empty grid with a background value. */
  static GMutableGrid create(const CPPType &type, const void *background_value);
  /* Create an empty grid with the type default as background value. */
  static GMutableGrid create(const CPPType &type);
  /* Create a grid with the active volume mask voxels. */
  static GMutableGrid create(const CPPType &type,
                             const GGrid &mask,
                             const void *inactive_value,
                             const void *active_value);

  bool try_copy_masked(const GGrid &other, const GGrid &selection);

  int64_t voxel_count() const;
  bool is_empty() const;
  operator bool() const;

  const CPPType *value_type() const;

  template<typename T> MutableGrid<T> typed() const;
};

template<typename T> class Grid {
 public:
#ifdef WITH_OPENVDB
  using GridType = grid_types::AttributeGrid<T>;
  using TreeType = typename GridType::TreeType;
  using GridPtr = typename GridType::Ptr;
  using GridConstPtr = typename GridType::ConstPtr;
  using ValueType = typename GridType::ValueType;

  GridConstPtr grid_ = nullptr;
#endif

  operator GGrid();
  operator GGrid const() const;

  int64_t voxel_count() const;
  bool is_empty() const;
  operator bool() const;

  const CPPType *value_type() const;
};

template<typename T> class MutableGrid {
 public:
#ifdef WITH_OPENVDB
  using GridType = grid_types::AttributeGrid<T>;
  using TreeType = typename GridType::TreeType;
  using GridPtr = typename GridType::Ptr;
  using GridConstPtr = typename GridType::ConstPtr;
  using ValueType = typename GridType::ValueType;

  GridPtr grid_ = nullptr;
#endif

  /* Create an empty grid with a background value. */
  static MutableGrid<T> create(const T &background_value);
  /* Create an empty grid with the type default as background value. */
  static MutableGrid<T> create();
  /* Create a grid with the active volume mask voxels. */
  static MutableGrid<T> create(const GGrid &mask, const T &inactive_value, const T &active_value);

  operator GGrid();
  operator GGrid const() const;
  operator GMutableGrid();
  operator GMutableGrid const() const;

  int64_t voxel_count() const;
  bool is_empty() const;
  operator bool() const;

  const CPPType *value_type() const;
};

template<typename T> MutableGrid<T>::operator GGrid()
{
#ifdef WITH_OPENVDB
  return GGrid{grid_};
#else
  return GGrid{};
#endif
}

template<typename T> MutableGrid<T>::operator GGrid const() const
{
#ifdef WITH_OPENVDB
  return GGrid{grid_};
#else
  return GGrid{};
#endif
}

template<typename T> MutableGrid<T>::operator GMutableGrid()
{
#ifdef WITH_OPENVDB
  return GMutableGrid{grid_};
#else
  return GMutableGrid{};
#endif
}

template<typename T> MutableGrid<T>::operator GMutableGrid const() const
{
#ifdef WITH_OPENVDB
  return GMutableGrid{grid_};
#else
  return GMutableGrid{};
#endif
}

/** \} */

}  // namespace volume

}  // namespace blender
