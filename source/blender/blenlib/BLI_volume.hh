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
}  // namespace OPENVDB_VERSION_NAME
}  // namespace openvdb
#endif

namespace blender {

/* XXX OpenVDB expects some math functions on vector types. */
template<typename T, int Size> inline VecBase<T, Size> Abs(VecBase<T, Size> v)
{
  VecBase<T, Size> r;
  for (int i = 0; i < Size; i++) {
    r[i] = math::abs(v[i]);
  }
  return r;
}
/* Specialization: math::abs is not defined for unsigned types. */
template<int Size> inline VecBase<uint32_t, Size> Abs(VecBase<uint32_t, Size> v)
{
  return v;
}

namespace volume {

#ifdef WITH_OPENVDB
namespace grid_types {

template<typename ValueType>
using TreeCommon = typename openvdb::tree::Tree4Fwd<ValueType, 5, 4, 3>::Type;
template<typename ValueType> using GridCommon = openvdb::Grid<TreeCommon<ValueType>>;

/* TODO add more as needed. */
/* TODO could use template magic to generate all from 1 list, but not worth it for now. */
/* TODO some types disabled because of missing CPPType registration. */

using BoolTree = TreeCommon<bool>;
using MaskTree = TreeCommon<openvdb::ValueMask>;
using FloatTree = TreeCommon<float>;
using Float2Tree = TreeCommon<float2>;
using Float3Tree = TreeCommon<float3>;
// using Float4Tree = TreeCommon<float4>;
using DoubleTree = TreeCommon<double>;
using Double3Tree = TreeCommon<double3>;
using IntTree = TreeCommon<int32_t>;
using Int2Tree = TreeCommon<int2>;
using Int3Tree = TreeCommon<int3>;
// using Int4Tree = TreeCommon<int4>;
using UIntTree = TreeCommon<uint32_t>;
// using UInt2Tree = TreeCommon<uint2>;
// using UInt3Tree = TreeCommon<uint3>;
// using UInt4Tree = TreeCommon<uint4>;
using ScalarTree = FloatTree;
using TopologyTree = MaskTree;

using BoolGrid = openvdb::Grid<BoolTree>;
using MaskGrid = openvdb::Grid<MaskTree>;
using FloatGrid = openvdb::Grid<FloatTree>;
using Float2Grid = openvdb::Grid<Float2Tree>;
using Float3Grid = openvdb::Grid<Float3Tree>;
// using Float4Grid = openvdb::Grid<Float4Tree>;
using DoubleGrid = openvdb::Grid<DoubleTree>;
using Double3Grid = openvdb::Grid<Double3Tree>;
using IntGrid = openvdb::Grid<IntTree>;
using Int2Grid = openvdb::Grid<Int2Tree>;
using Int3Grid = openvdb::Grid<Int3Tree>;
// using Int4Grid = openvdb::Grid<Int4Tree>;
using UIntGrid = openvdb::Grid<UIntTree>;
// using UInt2Grid = openvdb::Grid<UInt2Tree>;
// using UInt3Grid = openvdb::Grid<UInt3Tree>;
// using UInt4Grid = openvdb::Grid<UInt4Tree>;
using ScalarGrid = openvdb::Grid<ScalarTree>;
using TopologyGrid = openvdb::Grid<TopologyTree>;
using PointDataGrid = openvdb::points::PointDataGrid;

using SupportedGridValueTypes = std::tuple<bool,
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

using SupportedGridTypes = openvdb::TypeList<BoolGrid,
                                             MaskGrid,
                                             FloatGrid,
                                             Float2Grid,
                                             Float3Grid,
                                             /*Float4Grid,*/
                                             IntGrid,
                                             Int2Grid,
                                             /*Int3Grid,*/
                                             /*Int4Grid,*/
                                             UIntGrid,
                                             /*UInt2Grid,*/
                                             /*UInt3Grid,*/
                                             /*UInt4Grid,*/
                                             ScalarGrid,
                                             TopologyGrid>;

}  // namespace grid_types
#endif

class GGrid;
class GMutableGrid;
template<typename T> class Grid;
template<typename T> class MutableGrid;

/* Mask defined by active voxels of the grid. */
class GridMask {
#ifdef WITH_OPENVDB
  using GridPtr = std::shared_ptr<grid_types::MaskGrid>;
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

  bool try_assign(const GGrid &other);
  bool try_copy_masked(const GGrid &other, const GGrid &selection);

  int64_t voxel_count() const;
  bool is_empty() const;
  operator bool() const;

  const CPPType *value_type() const;

  template<typename T> MutableGrid<T> typed() const;
};

template<typename T> class Grid {
 public:
  using ValueType = T;
#ifdef WITH_OPENVDB
  using GridType = grid_types::GridCommon<T>;
  using TreeType = typename GridType::TreeType;
  using GridPtr = typename GridType::Ptr;
  using GridConstPtr = typename GridType::ConstPtr;

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
  using ValueType = T;
#ifdef WITH_OPENVDB
  using GridType = grid_types::GridCommon<T>;
  using TreeType = typename GridType::TreeType;
  using GridPtr = typename GridType::Ptr;
  using GridConstPtr = typename GridType::ConstPtr;

  GridPtr grid_ = nullptr;
#endif

  /* Create an empty grid with a background value. */
  static MutableGrid<T> create(const T &background_value);
  /* Create an empty grid with the type default as background value. */
  static MutableGrid<T> create();
  /* Create a grid with the active volume mask voxels. */
  static MutableGrid<T> create(const GGrid &mask, const T &inactive_value, const T &active_value);

  operator GMutableGrid();
  operator GMutableGrid const() const;

  int64_t voxel_count() const;
  bool is_empty() const;
  operator bool() const;

  const CPPType *value_type() const;
};

template<typename T> Grid<T> GGrid::typed() const
{
#ifdef WITH_OPENVDB
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
#else
  return {};
#endif
}

template<typename T> MutableGrid<T> GMutableGrid::typed() const
{
#ifdef WITH_OPENVDB
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
#else
  return {};
#endif
}

/** \} */

}  // namespace volume

}  // namespace blender
