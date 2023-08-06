/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_cpp_type.hh"
#include "BLI_generic_virtual_array.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_volume.hh"

#include "BKE_attribute.h"
#include "BKE_volume.h"

namespace blender::bke {

/**
 * Result when looking up an attribute from some geometry with the intention of only reading from
 * it.
 */
template<typename T> struct AttributeGridReader {
  /**
   * Virtual array that provides access to the attribute data. This may be empty.
   */
  volume::Grid<T> grid;
  /**
   * Domain where the attribute is stored. This also determines the size of the virtual array.
   */
  eAttrDomain domain;

  /**
   * Information about shared ownership of the attribute array. This will only be provided
   * if the virtual array directly references the contiguous original attribute array.
   */
  const ImplicitSharingInfo *sharing_info;

  const volume::Grid<T> &operator*() const
  {
    return this->grid;
  }
  volume::Grid<T> &operator*()
  {
    return this->grid;
  }

  operator bool() const
  {
    return this->varray;
  }
};

/**
 * Result when looking up an attribute from some geometry with read and write access. After
 * writing to the attribute, the #finish method has to be called. This may invalidate caches based
 * on this attribute.
 */
template<typename T> struct AttributeGridWriter {
  /**
   * Grid pointer giving read and write access to the attribute. This may be empty.
   */
  volume::MutableGrid<T> grid;
  /**
   * Domain where the attribute is stored on the geometry. Also determines the size of the
   * virtual array.
   */
  eAttrDomain domain;
  /**
   * A function that has to be called after the attribute has been edited. This may be empty.
   */
  std::function<void()> tag_modified_fn;

  operator bool() const
  {
    return this->grid != nullptr;
  }

  /**
   * Has to be called after the attribute has been modified.
   */
  void finish()
  {
    if (this->tag_modified_fn) {
      this->tag_modified_fn();
    }
  }
};

/**
 * A generic version of #AttributeReader.
 */
struct GAttributeGridReader {
  volume::GGrid grid;
  eAttrDomain domain;
  const ImplicitSharingInfo *sharing_info;

  operator bool() const
  {
    return this->grid;
  }

  const volume::GGrid &operator*() const
  {
    return this->grid;
  }
  volume::GGrid &operator*()
  {
    return this->grid;
  }

  template<typename T> AttributeGridReader<T> typed() const
  {
    return {grid.typed<T>(), domain, sharing_info};
  }
};

/**
 * A generic version of #AttributeWriter.
 */
struct GAttributeGridWriter {
  volume::GMutableGrid grid;
  eAttrDomain domain;
  std::function<void()> tag_modified_fn;

  operator bool() const
  {
    return this->grid;
  }

  void finish()
  {
    if (this->tag_modified_fn) {
      this->tag_modified_fn();
    }
  }

  template<typename T> AttributeGridWriter<T> typed() const
  {
    return {grid.typed<T>(), domain};
  }
};

/* OLD CODE
 * OLD CODE
 * OLD CODE
 * OLD CODE
 * OLD CODE */

#if 0

#  ifdef WITH_OPENVDB

namespace detail {

template<typename Fn> struct TypeTagExecutor {
  const Fn &fn;

  template<typename T> void operator()() const
  {
    fn(TypeTag<T>{});
  }

  void operator()() const
  {
    fn(TypeTag<void>{});
  }
};

}  // namespace detail

/**
 * Call operator for static type of the form:
 *   void MyOperator<GridType>();
 *
 * e.g.
 *   void MyOperator<openvdb::FloatGrid>();
 *   void MyOperator<openvdb::Vec3fGrid>();
 */
template<typename Fn> auto volume_grid_to_static_type(const VolumeGridType grid_type, Fn &&fn)
{
  switch (grid_type) {
    case VOLUME_GRID_FLOAT:
      return fn.template operator()<openvdb::FloatGrid>();
    case VOLUME_GRID_VECTOR_FLOAT:
      return fn.template operator()<openvdb::Vec3fGrid>();
    case VOLUME_GRID_BOOLEAN:
      return fn.template operator()<openvdb::BoolGrid>();
    case VOLUME_GRID_DOUBLE:
      return fn.template operator()<openvdb::DoubleGrid>();
    case VOLUME_GRID_INT:
      return fn.template operator()<openvdb::Int32Grid>();
    case VOLUME_GRID_INT64:
      return fn.template operator()<openvdb::Int64Grid>();
    case VOLUME_GRID_VECTOR_INT:
      return fn.template operator()<openvdb::Vec3IGrid>();
    case VOLUME_GRID_VECTOR_DOUBLE:
      return fn.template operator()<openvdb::Vec3dGrid>();
    case VOLUME_GRID_MASK:
      //   return fn.template operator()<openvdb::MaskGrid>();
      break;
    case VOLUME_GRID_POINTS:
      //    return fn.template operator()<openvdb::points::PointDataGrid>();
      break;
    case VOLUME_GRID_UNKNOWN:
      break;
  }

  /* Should never be called. */
  BLI_assert_msg(0, "should never be reached");
  return fn.template operator()<openvdb::FloatGrid>();
}

/**
 * Call operator for static type with a dummy tag instance
 * (easier to use with lambdas), e.g :
 *   void MyOperator(TypeTag<GridType> dummy);
 *
 * e.g.
 *   void MyOperator(TypeTag<openvdb::FloatGrid>);
 *   void MyOperator(TypeTag<openvdb::Vec3fGrid>);
 */
template<typename Fn> auto volume_grid_to_static_type_tag(const VolumeGridType grid_type, Fn &&fn)
{
  detail::TypeTagExecutor<Fn> executor{fn};
  return volume_grid_to_static_type(grid_type, executor);
}

/* -------------------------------------------------------------------- */
/** \name Grid Value Converter
 * \{ */

template<typename GridValueT> struct GridValueConverter {
  using GridValueType = GridValueT;
  using AttributeType = GridValueT;

  static AttributeType to_attribute(const GridValueType &value)
  {
    return value;
  }
  static GridValueType to_grid(const AttributeType &value)
  {
    return value;
  }
};

template<> struct GridValueConverter<double> {
  using GridValueType = double;
  using AttributeType = float;

  static AttributeType to_attribute(const GridValueType &value)
  {
    return double(value);
  }
  static GridValueType to_grid(const AttributeType &value)
  {
    return value;
  }
};

template<> struct GridValueConverter<openvdb::Vec3d> {
  using GridValueType = openvdb::Vec3d;
  using AttributeType = blender::float3;

  static AttributeType to_attribute(const GridValueType &value)
  {
    return float3(float(value.x()), float(value.y()), float(value.z()));
  }
  static GridValueType to_grid(const AttributeType &value)
  {
    return openvdb::Vec3d(value.x, value.y, value.z);
  }
};

template<> struct GridValueConverter<openvdb::Vec3i> {
  using GridValueType = openvdb::Vec3i;
  using AttributeType = blender::float3;

  static AttributeType to_attribute(const GridValueType &value)
  {
    return float3(float(value.x()), float(value.y()), float(value.z()));
  }
  static GridValueType to_grid(const AttributeType &value)
  {
    return openvdb::Vec3i(int(value.x), int(value.y), int(value.z));
  }
};

template<> struct GridValueConverter<openvdb::Vec3f> {
  using GridValueType = openvdb::Vec3f;
  using AttributeType = blender::float3;

  static AttributeType to_attribute(const GridValueType &value)
  {
    return float3(value.x(), value.y(), value.z());
  }
  static GridValueType to_grid(const AttributeType &value)
  {
    return openvdb::Vec3f(value.x, value.y, value.z);
  }
};

template<> struct GridValueConverter<openvdb::PointDataIndex32> {
  using GridValueType = openvdb::PointDataIndex32;
  using AttributeType = int32_t;

  static AttributeType to_attribute(const GridValueType &value)
  {
    return int32_t(value);
  }
  static GridValueType to_grid(const AttributeType &value)
  {
    return GridValueType(value);
  }
};

/** \} */

template<template<typename> typename VArrayImplT>
GVArray get_volume_varray(const openvdb::GridBase &vdb_grid, VolumeGridType grid_type)
{
  GVArray result;
  volume_grid_to_static_type_tag(grid_type, [&](auto tag) {
    using GridType = typename decltype(tag)::type;
    using VArrayImplType = VArrayImplT<GridType>;
    using AttributeType = typename VArrayImplType::AttributeType;

    result = VArray<AttributeType>::template For<VArrayImplType>(
        static_cast<const GridType &>(vdb_grid));
  });
  return result;
}

template<template<typename> typename VArrayImplT>
GVArray get_volume_varray(const VolumeGridVector &grids,
                          const AttributeIDRef &attribute_id,
                          bool use_first_grid)
{
  if (grids.empty()) {
    return {};
  }
  const VolumeGrid *grid = use_first_grid ? (grids.empty() ? nullptr : &grids.front()) :
                                            grids.find_grid(attribute_id);
  if (grid == nullptr) {
    return {};
  }

  return get_volume_varray<VArrayImplT>(*grid->grid(), grid->grid_type());
}

template<template<typename> typename VArrayImplT>
GVMutableArray get_volume_vmutablearray(openvdb::GridBase &vdb_grid, VolumeGridType grid_type)
{
  GVMutableArray result;
  volume_grid_to_static_type_tag(grid_type, [&](auto tag) {
    using GridType = typename decltype(tag)::type;
    using VArrayImplType = VArrayImplT<GridType>;
    using AttributeType = typename VArrayImplType::AttributeType;

    result = VMutableArray<AttributeType>::template For<VArrayImplType>(
        static_cast<const GridType &>(vdb_grid));
  });
  return result;
}

template<template<typename> typename VArrayImplT>
GVMutableArray get_volume_vmutablearray(VolumeGridVector &grids,
                                        const AttributeIDRef &attribute_id)
{
  if (grids.empty()) {
    return {};
  }
  VolumeGrid *grid = grids.find_grid(attribute_id);
  if (grid == nullptr) {
    return {};
  }

  return get_volume_vmutablearray<VArrayImplT>(*grid->grid(), grid->grid_type());
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name CPP Type for grid type
 * \{ */

inline const CPPType &volume_grid_type_to_cpp_type(const VolumeGridType grid_type)
{
  const CPPType *cpptype = nullptr;
  volume_grid_to_static_type_tag(grid_type, [&cpptype](auto tag) {
    using GridType = typename decltype(tag)::type;
    using Converter = typename bke::template GridValueConverter<typename GridType::ValueType>;
    using AttributeType = typename Converter::AttributeType;
    cpptype = &CPPType::get<AttributeType>();
  });
  return *cpptype;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Grid Value Virtual Array
 * \{ */

template<typename _GridType>
class VArrayImpl_For_VolumeGridValue final
    : public VMutableArrayImpl<
          typename bke::GridValueConverter<typename _GridType::ValueType>::AttributeType> {
 public:
  using GridType = typename std::remove_cv<_GridType>::type;
  using TreeType = typename GridType::TreeType;
  using ValueType = typename GridType::ValueType;
  using Converter = bke::GridValueConverter<ValueType>;
  using AttributeType = typename Converter::AttributeType;
  using LeafNodeType = typename TreeType::LeafNodeType;
  using LeafManager = openvdb::tree::LeafManager<TreeType>;
  using LeafRange = typename LeafManager::LeafRange;
  using BufferType = typename LeafManager::BufferType;

 protected:
  GridType &grid_;
  LeafManager leaf_manager_;
  /* Offset indices for leaf node buffers. */
  /* XXX this should be shared between VArrays accessing the same grid. */
  Array<size_t> prefix_sum_;

 public:
  VArrayImpl_For_VolumeGridValue(GridType &grid)
      : VMutableArrayImpl<AttributeType>(grid.activeVoxelCount()),
        grid_(grid),
        leaf_manager_(*grid_.treePtr()),
        prefix_sum_(leaf_manager_.leafCount())
  {
    size_t *prefix_data = prefix_sum_.data();
    size_t prefix_size = prefix_sum_.size();
    leaf_manager_.getPrefixSum(prefix_data, prefix_size);
  }

  VArrayImpl_For_VolumeGridValue(const GridType &grid)
      : VMutableArrayImpl<AttributeType>(grid.activeVoxelCount()),
        grid_(const_cast<GridType &>(grid)),
        leaf_manager_(*grid_.treePtr()),
        prefix_sum_(leaf_manager_.leafCount())
  {
    size_t *prefix_data = prefix_sum_.data();
    size_t prefix_size = prefix_sum_.size();
    leaf_manager_.getPrefixSum(prefix_data, prefix_size);
  }

  AttributeType get(const int64_t index) const override
  {
    /**
     * TODO Index-based access is very inefficient for leaf node buffers,
     * since linear search through active voxels is needed per leaf.
     * Implement all available alternative access methods to avoid
     * this as much as possible.
     */
    const size_t *buffer_index_ptr = std::upper_bound(
                                         prefix_sum_.begin(), prefix_sum_.end(), index) -
                                     1;
    const size_t leaf_index = buffer_index_ptr - prefix_sum_.begin();
    BLI_assert(IndexRange(leaf_manager_.leafCount()).contains(leaf_index));
    const LeafNodeType &leaf = leaf_manager_.leaf(leaf_index);
    int64_t i = *buffer_index_ptr;
    for (typename LeafNodeType::ValueOnCIter iter = leaf.cbeginValueOn(); iter; ++iter, ++i) {
      if (i == index) {
        return Converter::to_attribute(iter.getValue());
      }
    }
    return AttributeType{0};
  }

  void set(const int64_t index, const AttributeType value) override
  {
    /**
     * TODO Index-based access is very inefficient for leaf node buffers,
     * since linear search through active voxels is needed per leaf.
     * Implement all available alternative access methods to avoid
     * this as much as possible.
     */
    const size_t *buffer_index_ptr = std::upper_bound(
                                         prefix_sum_.begin(), prefix_sum_.end(), index) -
                                     1;
    const size_t leaf_index = buffer_index_ptr - prefix_sum_.begin();
    BLI_assert(IndexRange(leaf_manager_.leafCount()).contains(leaf_index));
    LeafNodeType &leaf = leaf_manager_.leaf(leaf_index);
    int64_t i = *buffer_index_ptr;
    for (typename LeafNodeType::ValueOnIter iter = leaf.beginValueOn(); iter; ++iter, ++i) {
      if (i == index) {
        iter.setValue(Converter::to_grid(value));
        break;
      }
    }
  }

  void set_all(Span<AttributeType> src) override
  {
    tbb::parallel_for(leaf_manager_.leafRange(), [&](const LeafRange &range) {
      for (auto leaf_iter = range.begin(); leaf_iter; ++leaf_iter) {
        const size_t leaf_index = leaf_iter.pos();
        const IndexRange leaf_buffer_range(prefix_sum_[leaf_index], leaf_iter->onVoxelCount());
        // std::cout << "Leaf range: " << leaf_buffer_range.start() << " + "
        //           << leaf_buffer_range.size() << std::endl;

        auto iter = leaf_iter->beginValueOn();
        for (const int src_i : leaf_buffer_range) {
          iter.setValue(Converter::to_grid(src[src_i]));
          // std::cout << "  set [" << src_i << "] = " << Converter::to_grid(src[src_i]) <<
          // std::endl;
        }
      }
    });
  }

  void materialize(const IndexMask &mask, AttributeType *dst) const override
  {
    LeafManager leaf_mgr(*grid_.treePtr());

    /* Offset indices for leaf node buffers. */
    Array<size_t> prefix_sum(leaf_mgr.activeLeafVoxelCount());
    size_t *prefix_sum_data = prefix_sum.data();
    size_t prefix_sum_size = prefix_sum.size();
    leaf_mgr.getPrefixSum(prefix_sum_data, prefix_sum_size);

    const LeafRange leaf_range = leaf_mgr.leafRange();
    // int64_t segment_start = 0;
    // int64_t leaf_start = 0;
    // for (const int64_t segment_i : IndexRange(segments_num_)) {
    //   const IndexMaskSegment segment = mask->segment(segment_i);
    mask.foreach_range([&](IndexRange segment, const int64_t segment_pos) {
      if (segment.is_empty()) {
        return;
      }
      threading::parallel_for(segment, 4096, [&](const IndexRange range) {
        const int leaf_begin = *std::lower_bound(
            prefix_sum.begin(), prefix_sum.end(), segment.first());
        const int leaf_end = *std::upper_bound(
            prefix_sum.begin(), prefix_sum.end(), segment.last());
        tbb::parallel_for(leaf_range, [&](const LeafRange &leaf_range) {
          UNUSED_VARS(dst, leaf_begin, leaf_end, segment_pos, range, leaf_range);
          // for (auto leaf_iter = range.begin(); leaf_iter; ++leaf_iter) {
          //   const size_t leaf_index = leaf_iter.pos();
          //   const IndexRange leaf_buffer_range(prefix_sum[leaf_index],
          //   leaf_iter->onVoxelCount());

          //  auto iter = leaf_iter->beginValueOn();
          //  for (const int src_i : leaf_buffer_range) {
          //    iter.setValue(Converter::to_grid(src[src_i]));
          //  }
          //}
        });

        // segment_start += segment.size();
      });
    });

    // mask.foreach_index(GrainSize(4096), [&](const int64_t i) {
    //   if (const MDeformWeight *weight = this->find_weight_at_index(i)) {
    //     dst[i] = weight->weight;
    //   }
    //   else {
    //     dst[i] = 0.0f;
    //   }
    // });
  }

  void materialize_to_uninitialized(const IndexMask &mask, AttributeType *dst) const override
  {
    this->materialize(mask, dst);
  }
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name Grid Position Virtual Array
 * \{ */

template<typename _GridType>
class VArrayImpl_For_VolumeGridPosition final : public VArrayImpl<float3> {
 public:
  using AttributeType = float3;
  using Converter = bke::GridValueConverter<openvdb::Vec3d>;
  using GridType = typename std::remove_cv<_GridType>::type;
  using TreeType = typename GridType::TreeType;
  using LeafNodeType = typename TreeType::LeafNodeType;
  using LeafManager = openvdb::tree::LeafManager<TreeType>;
  using LeafRange = typename LeafManager::LeafRange;

 protected:
  GridType &grid_;
  LeafManager leaf_manager_;
  /* Offset indices for leaf node buffers. */
  /* XXX this should be shared between VArrays accessing the same grid. */
  Array<size_t> prefix_sum_;

 public:
  VArrayImpl_For_VolumeGridPosition(const GridType &grid)
      : VArrayImpl<float3>(grid.activeVoxelCount()),
        grid_(const_cast<GridType &>(grid)),
        leaf_manager_(*grid_.treePtr()),
        prefix_sum_(leaf_manager_.leafCount())
  {
    size_t *prefix_data = prefix_sum_.data();
    size_t prefix_size = prefix_sum_.size();
    leaf_manager_.getPrefixSum(prefix_data, prefix_size);
  }

  float3 get(const int64_t index) const override
  {
    /**
     * TODO Index-based access is very inefficient for leaf node buffers,
     * since linear search through active voxels is needed per leaf.
     * Implement all available alternative access methods to avoid
     * this as much as possible.
     */
    const size_t *buffer_index_ptr = std::upper_bound(
                                         prefix_sum_.begin(), prefix_sum_.end(), index) -
                                     1;
    const size_t leaf_index = buffer_index_ptr - prefix_sum_.begin();
    BLI_assert(IndexRange(leaf_manager_.leafCount()).contains(leaf_index));
    const LeafNodeType &leaf = leaf_manager_.leaf(leaf_index);
    int64_t i = *buffer_index_ptr;
    for (typename LeafNodeType::ValueOnCIter iter = leaf.cbeginValueOn(); iter; ++iter, ++i) {
      if (i == index) {
        return Converter::to_attribute(grid_.indexToWorld(iter.getCoord()));
      }
    }
    return AttributeType{0};
  }

  void materialize(const IndexMask &mask, float3 *dst) const override
  {
    LeafManager leaf_mgr(*grid_.treePtr());

    /* Offset indices for leaf node buffers. */
    Array<size_t> prefix_sum(leaf_mgr.activeLeafVoxelCount());
    size_t *prefix_sum_data = prefix_sum.data();
    size_t prefix_sum_size = prefix_sum.size();
    leaf_mgr.getPrefixSum(prefix_sum_data, prefix_sum_size);

    const LeafRange leaf_range = leaf_mgr.leafRange();
    // int64_t segment_start = 0;
    // int64_t leaf_start = 0;
    // for (const int64_t segment_i : IndexRange(segments_num_)) {
    //   const IndexMaskSegment segment = mask->segment(segment_i);
    mask.foreach_range([&](IndexRange segment, const int64_t segment_pos) {
      if (segment.is_empty()) {
        return;
      }
      threading::parallel_for(segment, 4096, [&](const IndexRange range) {
        const int leaf_begin = *std::lower_bound(
            prefix_sum.begin(), prefix_sum.end(), segment.first());
        const int leaf_end = *std::upper_bound(
            prefix_sum.begin(), prefix_sum.end(), segment.last());
        tbb::parallel_for(leaf_range, [&](const LeafRange &leaf_range) {
          UNUSED_VARS(dst, leaf_begin, leaf_end, segment_pos, range, leaf_range);
          // for (auto leaf_iter = range.begin(); leaf_iter; ++leaf_iter) {
          //   const size_t leaf_index = leaf_iter.pos();
          //   const IndexRange leaf_buffer_range(prefix_sum[leaf_index],
          //   leaf_iter->onVoxelCount());

          //  auto iter = leaf_iter->beginValueOn();
          //  for (const int src_i : leaf_buffer_range) {
          //    iter.setValue(Converter::to_grid(src[src_i]));
          //  }
          //}
        });

        // segment_start += segment.size();
      });
    });

    // mask.foreach_index(GrainSize(4096), [&](const int64_t i) {
    //   if (const MDeformWeight *weight = this->find_weight_at_index(i)) {
    //     dst[i] = weight->weight;
    //   }
    //   else {
    //     dst[i] = 0.0f;
    //   }
    // });
  }

  void materialize_to_uninitialized(const IndexMask &mask, float3 *dst) const override
  {
    this->materialize(mask, dst);
  }
};

/** \} */

#  endif

#endif

}  // namespace blender::bke
