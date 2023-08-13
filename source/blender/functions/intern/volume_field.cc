/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_cpp_type.hh"
#include "BLI_function_ref.hh"
#include "BLI_math_base.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_resource_scope.hh"
#include "BLI_virtual_array.hh"
#include "BLI_volume_openvdb.hh"

#include "FN_field.hh"
#include "FN_multi_function_builder.hh"
#include "FN_multi_function_procedure_executor.hh"
#include "FN_volume_field.hh"

#ifdef WITH_OPENVDB

#  include <openvdb/tools/ValueTransformer.h>

namespace blender::fn {

namespace mf = multi_function;

/* A VArray implementation using OpenVDB grid accessors.
 * The index is converted to global voxel coordinate
 * by interpreting it as a leaf buffer index. */
template<typename AccessorType, typename LeafNodeType>
class VArrayImpl_For_GridLeaf final : public VArrayImpl<typename AccessorType::ValueType> {
 public:
  using ValueType = typename AccessorType::ValueType;
  using Coord = openvdb::Coord;

 protected:
  const AccessorType &accessor_;
  const Coord leaf_origin_;

 public:
  VArrayImpl_For_GridLeaf(const AccessorType &accessor, const Coord &leaf_origin)
      : VArrayImpl<ValueType>(LeafNodeType::size()), accessor_(accessor), leaf_origin_(leaf_origin)
  {
  }
  VArrayImpl_For_GridLeaf(const AccessorType &accessor, const LeafNodeType &leaf)
      : VArrayImpl<ValueType>(LeafNodeType::size()),
        accessor_(accessor),
        leaf_origin_(leaf.origin())
  {
  }

  Coord index_to_global_coord(const int64_t index) const
  {
    return LeafNodeType::offsetToLocalCoord(index) + leaf_origin_;
  }

  int64_t global_coord_to_index(const Coord &coord) const
  {
    return LeafNodeType::coordToOffset(coord - leaf_origin_);
  }

  ValueType get(const int64_t index) const override
  {
    const Coord coord = index_to_global_coord(index);
    return accessor_.getValue(coord);
  }

  void materialize(const IndexMask &mask, ValueType *dst) const override
  {
    mask.foreach_index([this, dst](const int64_t i) {
      const Coord coord = index_to_global_coord(i);
      dst[i] = accessor_.getValue(coord);
    });
  }

  void materialize_to_uninitialized(const IndexMask &mask, ValueType *dst) const override
  {
    this->materialize(mask, dst);
  }
};

/* A VArray implementation using OpenVDB grid accessors.
 * The index is converted to global voxel coordinate
 * by interpreting it as a leaf buffer index. */
template<typename AccessorType, typename LeafNodeType>
class VMutableArrayImpl_For_GridLeaf final
    : public VMutableArrayImpl<typename AccessorType::ValueType> {
 public:
  using ValueType = typename AccessorType::ValueType;
  using Coord = openvdb::Coord;
  const int32_t leaf_size = LeafNodeType::size();

 protected:
  AccessorType &accessor_;
  const Coord leaf_origin_;

 public:
  VMutableArrayImpl_For_GridLeaf(AccessorType &accessor, const Coord &leaf_origin)
      : VArrayImpl<ValueType>(leaf_size), accessor_(accessor), leaf_origin_(leaf_origin)
  {
  }
  VMutableArrayImpl_For_GridLeaf(AccessorType &accessor, const LeafNodeType &leaf)
      : VArrayImpl<ValueType>(leaf_size), accessor_(accessor), leaf_origin_(leaf.origin())
  {
  }

  Coord index_to_global_coord(const int64_t index) const
  {
    return LeafNodeType::offsetToLocalCoord(index) + leaf_origin_;
  }

  int64_t global_coord_to_index(const Coord &coord) const
  {
    return LeafNodeType::coordToOffset(coord - leaf_origin_);
  }

  ValueType get(const int64_t index) const override
  {
    const Coord coord = index_to_global_coord(index);
    return accessor_.getValue(coord);
  }

  void set(const int64_t index, const ValueType value) override
  {
    const Coord coord = index_to_global_coord(index);
    return accessor_.setValue(coord, value);
  }

  void set_all(Span<ValueType> src) override
  {
    for (const int i : src.index_range()) {
      const Coord coord = index_to_global_coord(i);
      accessor_.setValue(coord, src[i]);
    }
  }

  void materialize(const IndexMask &mask, ValueType *dst) const override
  {
    mask.foreach_index([this, dst](const int64_t i) {
      const Coord coord = index_to_global_coord(i);
      dst[i] = accessor_.getValue(coord);
    });
  }

  void materialize_to_uninitialized(const IndexMask &mask, ValueType *dst) const override
  {
    this->materialize(mask, dst);
  }
};

/* Wrapper holding an accessor for an input field. */
template<typename LeafNodeType> struct AccessorWrapper {
  virtual ~AccessorWrapper() {}

  /* VArray for a specific leaf node using the accessor. */
  virtual GVArray make_varray_for_leaf(const LeafNodeType &leaf) const = 0;
  ///* VMutableArray for a specific leaf node using the accessor. */
  // virtual GVMutableArray make_vmutablearray_for_leaf(const LeafNodeType &leaf) const = 0;
};

template<typename LeafNodeType, typename AccessorType>
struct TypedAccessorWrapper : public AccessorWrapper<LeafNodeType> {
  using ValueType = typename AccessorType::ValueType;

  AccessorType accessor_;

  TypedAccessorWrapper(const AccessorType &accessor) : accessor_(accessor) {}
  virtual ~TypedAccessorWrapper() {}

  virtual GVArray make_varray_for_leaf(const LeafNodeType &leaf) const override
  {
    using VArrayImplType = VArrayImpl_For_GridLeaf<AccessorType, LeafNodeType>;
    return VArray<ValueType>::template For<VArrayImplType>(accessor_, leaf);
  }

  // virtual GVMutableArray make_vmutablearray_for_leaf(const LeafNodeType &leaf) const override
  //{
  //   using VMutableArrayImplType = VMutableArrayImpl_For_GridLeaf<AccessorType, LeafNodeType>;
  //   return VMutableArray<ValueType>::For<VMutableArrayImplType>(accessor_, leaf);
  // }
};

template<typename GridType, typename MaskGridType> struct EvalPerLeafOp {
  using GGrid = volume::GGrid;
  using GMutableGrid = volume::GMutableGrid;
  using GridMask = volume::GridMask;
  using TreeType = typename GridType::TreeType;
  using ValueType = typename GridType::ValueType;

  using LeafNode = typename TreeType::LeafNodeType;
  using AccessorWrapperType = AccessorWrapper<LeafNode>;

  using Coord = openvdb::Coord;
  const int32_t leaf_size = LeafNode::size();

  const mf::ProcedureExecutor &procedure_executor_;

  /* Each thread gets its own copy of the functor,
   * so the accessor and context don't have to be thread-safe. */
  mutable typename MaskGridType::ConstAccessor mask_accessor_;
  mutable IndexMaskMemory index_mask_memory_;
  mutable mf::ContextBuilder mf_context_;

  /* Accessors for input fields. */
  /* XXX have to use shared_ptr instead of unique_ptr because some parts
   * of BLI_memory_utils try to copy the values and that's forbidden for unique_ptr.
   */
  Array<std::shared_ptr<AccessorWrapperType>> input_accessors_;

  void make_input_accessors(Span<GGrid> field_context_inputs)
  {
    input_accessors_.reinitialize(field_context_inputs.size());

    for (const int i : field_context_inputs.index_range()) {
      volume::grid_to_static_type(field_context_inputs[i].grid_, [&](auto &input_grid) {
        using InputGridType = typename std::decay<decltype(input_grid)>::type;
        using AccessorType = typename InputGridType::ConstAccessor;

        std::shared_ptr<AccessorWrapperType> accessor_ptr =
            std::make_shared<TypedAccessorWrapper<LeafNode, AccessorType>>(
                input_grid.getConstAccessor());
        input_accessors_[i] = std::move(accessor_ptr);
      });
    }
  }

  EvalPerLeafOp(Span<GGrid> field_context_inputs,
                const mf::ProcedureExecutor &procedure_executor,
                const MaskGridType &mask)
      : procedure_executor_(procedure_executor), mask_accessor_(mask.getAccessor())
  {
    make_input_accessors(field_context_inputs);
  }
  EvalPerLeafOp(const EvalPerLeafOp &other)
      : procedure_executor_(other.procedure_executor_),
        mask_accessor_(other.mask_accessor_),
        input_accessors_(other.input_accessors_)
  {
  }

  inline void operator()(const typename TreeType::LeafIter &leaf_iter) const
  {
    LeafNode &leaf = *leaf_iter;

    /* TODO direct construction from the mask could be faster,
     * not sure how to do that best. */
    IndexMask index_mask = IndexMask::from_predicate(
        IndexMask(leaf_size),
        GrainSize(leaf_size),
        index_mask_memory_,
        [this, leaf](const int64_t index) -> bool {
          const Coord coord = leaf.offsetToGlobalCoord(index);
          return mask_accessor_.isValueOn(coord);
        });

    mf::ParamsBuilder mf_params{procedure_executor_, &index_mask};

    /* Provide inputs to the procedure executor. */
    for (const std::shared_ptr<AccessorWrapperType> &accessor_wrapper : input_accessors_) {
      mf_params.add_readonly_single_input(accessor_wrapper->make_varray_for_leaf(leaf));
    }

    /* Pass output buffer to the procedure executor. */
    const CPPType &out_type = CPPType::get<ValueType>();
    const GMutableSpan out_span{out_type, leaf.buffer().data(), leaf_size};
    mf_params.add_uninitialized_single_output(out_span);

    procedure_executor_.call_auto(index_mask, mf_params, mf_context_);
  }
};

void evaluate_procedure_on_varying_volume_fields(ResourceScope &scope,
                                                 const volume::GGrid &mask,
                                                 const mf::Procedure &procedure,
                                                 Span<volume::GGrid> field_context_inputs,
                                                 Span<GFieldRef> fields_to_evaluate,
                                                 Span<int> field_indices,
                                                 Span<volume::GMutableGrid *> dst_grids,
                                                 MutableSpan<volume::GGrid> r_grids,
                                                 MutableSpan<bool> r_is_output_written_to_dst)
{
  /* Execute a multifunction procedure on each leaf buffer of the mask.
   * Each leaf buffer is a contiguous array that can be used as a span.
   * The leaf buffers' active voxel masks are used as index masks. */

  using volume::GGrid;
  using volume::GMutableGrid;
  using volume::GridMask;

  if (mask.is_empty()) {
    return;
  }

  /* Destination arrays are optional. Create a small utility method to access them. */
  auto get_dst_grid = [&](int index) -> GMutableGrid * {
    if (dst_grids.is_empty()) {
      return {};
    }
    GMutableGrid *grid_ptr = dst_grids[index];
    if (!grid_ptr) {
      return nullptr;
    }
    BLI_assert(grid_ptr->voxel_count() >= (mask.grid_ ? mask.grid_->activeVoxelCount() : 0));
    return grid_ptr;
  };

  mf::ProcedureExecutor procedure_executor{procedure};

  for (const int i : fields_to_evaluate.index_range()) {
    const GFieldRef &field = fields_to_evaluate[i];
    const CPPType &type = field.cpp_type();
    const int out_index = field_indices[i];

    /* Try to get an existing virtual array that the result should be written into. */
    GMutableGrid *dst_grid_ptr = get_dst_grid(out_index);
    if (!dst_grid_ptr) {
      /* Create a destination grid for the computed result. */
      GMutableGrid dst_grid = GMutableGrid::create(
          type, mask, type.default_value(), type.default_value());
      scope.add_value<GMutableGrid>(std::move(dst_grid));
      dst_grid_ptr = &dst_grid;
      r_grids[out_index] = dst_grid;
    }
    else {
      /* Write the result into the existing grid. */
      r_grids[out_index] = *dst_grid_ptr;
      r_is_output_written_to_dst[out_index] = true;
    }

    /* Execute the multifunction procedure on each leaf buffer of the mask.
     * Each leaf buffer is a contiguous array that can be used as a span.
     * The leaf buffers' active voxel masks are used as index masks. */

    volume::grid_to_static_type(dst_grid_ptr->grid_, [&](auto &dst_grid) {
      using GridType = typename std::decay<decltype(dst_grid)>::type;

      volume::grid_to_static_type(mask.grid_, [&](auto &mask_grid) {
        using MaskGridType = typename std::decay<decltype(mask_grid)>::type;

        EvalPerLeafOp<GridType, MaskGridType> func(
            field_context_inputs, procedure_executor, mask_grid);
        openvdb::tools::foreach (dst_grid.tree().beginLeaf(),
                                 func,
                                 /*threaded=*/true,
                                 /*shareOp=*/false);
      });
    });
  }
}

void evaluate_procedure_on_constant_volume_fields(ResourceScope & /*scope*/,
                                                  const mf::Procedure &procedure,
                                                  Span<volume::GGrid> field_context_inputs,
                                                  Span<GFieldRef> fields_to_evaluate,
                                                  Span<int> field_indices,
                                                  MutableSpan<volume::GGrid> r_grids)
{
  using volume::GGrid;
  using volume::GMutableGrid;
  using volume::GridMask;

  mf::ProcedureExecutor procedure_executor{procedure};
  const IndexMask mask(1);
  mf::ParamsBuilder mf_params{procedure_executor, &mask};
  mf::ContextBuilder mf_context;

  /* Provide inputs to the procedure executor. */
  for (const int i : field_context_inputs.index_range()) {
    volume::grid_to_static_type(field_context_inputs[i].grid_, [&](auto &input_grid) {
      using InputGridType = typename std::decay<decltype(input_grid)>::type;
      using ValueType = typename InputGridType::ValueType;

      /* XXX not all grid types have a background property. */
      // const ValueType input_value = input_grid.background();
      typename InputGridType::ConstAccessor accessor = input_grid.getAccessor();
      const ValueType input_value = accessor.getValue(openvdb::Coord(0, 0, 0));

      VArray<ValueType> varray = VArray<ValueType>::ForSingle(input_value, 1);
      mf_params.add_readonly_single_input(varray);
    });
  }

  /* Temporary buffers for output values, these are stored as background values on empty grids
   * after the prodcure execution. */
  Array<void *> output_buffers(fields_to_evaluate.size());
  for (const int i : fields_to_evaluate.index_range()) {
    const GFieldRef &field = fields_to_evaluate[i];
    const CPPType &type = field.cpp_type();

    output_buffers[i] = MEM_mallocN(type.size(), __func__);

    /* Pass output buffer to the procedure executor. */
    mf_params.add_uninitialized_single_output({type, output_buffers[i], 1});
  }

  procedure_executor.call(mask, mf_params, mf_context);

  for (const int i : fields_to_evaluate.index_range()) {
    const GFieldRef &field = fields_to_evaluate[i];
    const CPPType &type = field.cpp_type();
    const int out_index = field_indices[i];

    volume::field_to_static_type(type, [&](auto type_tag) {
      using ValueType = typename decltype(type_tag)::type;

      const ValueType &value = *static_cast<ValueType *>(output_buffers[i]);
      r_grids[out_index] = GGrid{volume::grid_types::GridCommon<ValueType>::create(value)};
    });

    /* Destruct output value buffers, value is stored in grid backgrounds now. */
    type.destruct(output_buffers[i]);
  }
}

}  // namespace blender::fn

#endif
