/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_cpp_type.hh"
#include "BLI_function_ref.hh"
#include "BLI_math_base.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_resource_scope.hh"
#include "BLI_virtual_array.hh"
#include "BLI_virtual_grid.hh"
#include "BLI_volume_openvdb.hh"

#include "FN_field.hh"
#include "FN_multi_function_builder.hh"
#include "FN_multi_function_procedure_executor.hh"
#include "FN_volume_field.hh"

#ifdef WITH_OPENVDB

#  include <openvdb/tools/ValueTransformer.h>

namespace blender::fn {

namespace mf = multi_function;

///* A VArray implementation using OpenVDB grid accessors.
// * The index is converted to global voxel coordinate
// * by interpreting it as a leaf buffer index. */
// template<typename AccessorType, typename LeafNodeType, typename Converter>
// class VArrayImpl_For_GridLeaf final : public VArrayImpl<typename Converter::AttributeValueType>
// {
// public:
//  using Coord = openvdb::Coord;
//  using AttributeValueType = typename Converter::AttributeValueType;
//
// protected:
//  const AccessorType &accessor_;
//  const Coord leaf_origin_;
//
// public:
//  VArrayImpl_For_GridLeaf(const AccessorType &accessor, const Coord &leaf_origin)
//      : VArrayImpl<AttributeValueType>(LeafNodeType::size()),
//        accessor_(accessor),
//        leaf_origin_(leaf_origin)
//  {
//  }
//  VArrayImpl_For_GridLeaf(const AccessorType &accessor, const LeafNodeType &leaf)
//      : VArrayImpl<AttributeValueType>(LeafNodeType::size()),
//        accessor_(accessor),
//        leaf_origin_(leaf.origin())
//  {
//  }
//
//  Coord index_to_global_coord(const int64_t index) const
//  {
//    return LeafNodeType::offsetToLocalCoord(index) + leaf_origin_;
//  }
//
//  int64_t global_coord_to_index(const Coord &coord) const
//  {
//    return LeafNodeType::coordToOffset(coord - leaf_origin_);
//  }
//
//  AttributeValueType get(const int64_t index) const override
//  {
//    const Coord coord = index_to_global_coord(index);
//    return Converter::single_value_to_attribute(accessor_.getValue(coord));
//  }
//
//  void materialize(const IndexMask &mask, AttributeValueType *dst) const override
//  {
//    mask.foreach_index([this, dst](const int64_t i) {
//      const Coord coord = index_to_global_coord(i);
//      dst[i] = Converter::single_value_to_attribute(accessor_.getValue(coord));
//    });
//  }
//
//  void materialize_to_uninitialized(const IndexMask &mask, AttributeValueType *dst) const
//  override
//  {
//    this->materialize(mask, dst);
//  }
//};

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

template<typename GridType>
void merge_input_field_topology(GridType &buffer, Span<GVGrid> field_context_inputs)
{
  for (const GVGrid &input_grid : field_context_inputs) {
    if (!input_grid.is_grid()) {
      /* Non-grids have undefined topology, nothing to merge. */
      continue;
    }
    volume::grid_to_static_type(*input_grid.get_internal_grid(), [&](auto &typed_input_grid) {
      buffer.topologyUnion(typed_input_grid);
    });
  }
}

template<typename GridType, typename MaskGridType = openvdb::MaskGrid> struct EvalPerLeafOp {
  using TreeType = typename GridType::TreeType;
  using ValueType = typename GridType::ValueType;

  using LeafNode = typename TreeType::LeafNodeType;

  using Coord = openvdb::Coord;
  const int32_t leaf_size = LeafNode::size();
  static const int32_t LOG2DIM = static_cast<int32_t>(LeafNode::LOG2DIM);

  const mf::ProcedureExecutor &procedure_executor_;

  /* Each thread gets its own copy of the functor,
   * so the accessor and context don't have to be thread-safe. */
  mutable std::optional<typename MaskGridType::ConstAccessor> mask_accessor_;
  mutable IndexMaskMemory index_mask_memory_;
  mutable mf::ContextBuilder mf_context_;

  Span<GVGrid> field_context_inputs_;

  EvalPerLeafOp(Span<GVGrid> field_context_inputs,
                const mf::ProcedureExecutor &procedure_executor,
                const MaskGridType &mask)
      : procedure_executor_(procedure_executor),
        mask_accessor_(mask.getAccessor()),
        field_context_inputs_(field_context_inputs)
  {
  }

  EvalPerLeafOp(Span<GVGrid> field_context_inputs, const mf::ProcedureExecutor &procedure_executor)
      : procedure_executor_(procedure_executor), field_context_inputs_(field_context_inputs)
  {
  }

  EvalPerLeafOp(const EvalPerLeafOp &other)
      : procedure_executor_(other.procedure_executor_),
        mask_accessor_(other.mask_accessor_),
        field_context_inputs_(other.field_context_inputs_)
  {
  }

  inline void operator()(const typename TreeType::LeafIter &leaf_iter) const
  {
    LeafNode &leaf = *leaf_iter;

    IndexMask index_mask(leaf_size);
    if (mask_accessor_) {
      /* TODO direct construction from the mask could be faster,
       * not sure how to do that best. */
      index_mask = IndexMask::from_predicate(IndexMask(leaf_size),
                                             GrainSize(leaf_size),
                                             index_mask_memory_,
                                             [this, leaf](const int64_t index) -> bool {
                                               const Coord coord = leaf.offsetToGlobalCoord(index);
                                               return mask_accessor_.value().isValueOn(coord);
                                             });
    }

    mf::ParamsBuilder mf_params{procedure_executor_, &index_mask};

    /* Provide inputs to the procedure executor. */
    for (const GVGrid &input : field_context_inputs_) {
      const int3 origin = int3(leaf.origin().data());
      mf_params.add_readonly_single_input(input.get_varray_for_leaf(LOG2DIM, origin));
    }

    /* Pass output buffer to the procedure executor. */
    auto *leaf_values = leaf.buffer().data();
    MutableSpan<std::decay_t<decltype(*leaf_values)>> leaf_span = {leaf_values, leaf_size};
    const GMutableSpan out_span = volume::grid_types::Converter<GridType>::leaf_buffer_to_varray(
        leaf_span);
    mf_params.add_uninitialized_single_output(out_span);

    procedure_executor_.call_auto(index_mask, mf_params, mf_context_);
  }
};

void evaluate_procedure_on_varying_volume_fields(ResourceScope &scope,
                                                 const float4x4 &transform,
                                                 const GVGrid &mask,
                                                 const mf::Procedure &procedure,
                                                 Span<GVGrid> field_context_inputs,
                                                 Span<GFieldRef> fields_to_evaluate,
                                                 Span<int> field_indices,
                                                 Span<GVMutableGrid> dst_grids,
                                                 MutableSpan<GVGrid> r_grids,
                                                 MutableSpan<bool> r_is_output_written_to_dst)
{
  /* Execute a multifunction procedure on each leaf buffer of the mask.
   * Each leaf buffer is a contiguous array that can be used as a span.
   * The leaf buffers' active voxel masks are used as index masks. */

  if (!mask) {
    return;
  }

  /* Destination arrays are optional. Create a small utility method to access them. */
  auto get_dst_grid = [&](int index) -> GVMutableGrid {
    if (dst_grids.is_empty()) {
      return {};
    }
    GVMutableGrid grid = dst_grids[index];
    if (!grid) {
      return nullptr;
    }
    return grid;
  };

  mf::ProcedureExecutor procedure_executor{procedure};

  for (const int i : fields_to_evaluate.index_range()) {
    const GFieldRef &field = fields_to_evaluate[i];
    const CPPType &type = field.cpp_type();
    const int out_index = field_indices[i];

    /* Try to get an existing virtual grid that the result should be written into. */
    GVMutableGrid dst_grid = get_dst_grid(out_index);
    openvdb::GridBase *buffer;
    if (!dst_grid || !dst_grid.is_grid()) {
      /* Create a destination grid pointer in the resource scope. */
      buffer = volume::make_grid_for_attribute_type(scope, type, transform);
      r_grids[out_index] = GVMutableGrid::ForGrid(*buffer);
      dst_grid = GVMutableGrid::ForGrid(*buffer);
    }
    else {
      /* Write the result into the existing grid. */
      buffer = dst_grid.get_internal_grid();
      r_grids[out_index] = dst_grid;
      r_is_output_written_to_dst[out_index] = true;
    }

    /* Execute the multifunction procedure on each leaf buffer of the mask.
     * Each leaf buffer is a contiguous array that can be used as a span.
     * The leaf buffers' active voxel masks are used as index masks. */

    volume::grid_to_static_type(*buffer, [&](auto &dst_grid) {
      using GridType = typename std::decay<decltype(dst_grid)>::type;

      merge_input_field_topology(dst_grid, field_context_inputs);

      /* Make sure every active tile has leaf node buffers to write into.
       * A tree can have active (non-leaf) tiles with only the background value.
       * We need to "densify" these tiles so that the leaf iterator actually
       * yields buffers on which we can evaluate the multifunction.
       * XXX Ideally we'd be able to determine in advance if all values
       * in a leaf node would be constant, and only evaluate the background value!
       */
      dst_grid.tree().voxelizeActiveTiles();

      const CommonVGridInfo mask_info = mask.common_info();
      switch (mask_info.type) {
        case CommonVGridInfo::Type::Single:
        case CommonVGridInfo::Type::Any: {
          /* Ignore mask, counts as active everywhere */
          EvalPerLeafOp<GridType> func(field_context_inputs, procedure_executor);
          openvdb::tools::foreach (dst_grid.tree().beginLeaf(),
                                   func,
                                   /*threaded=*/true,
                                   /*shareOp=*/false);
          break;
        }
        case CommonVGridInfo::Type::Grid: {
          volume::grid_to_static_type(*mask.get_internal_grid(), [&](auto &mask_grid) {
            using MaskGridType = typename std::decay<decltype(mask_grid)>::type;

            EvalPerLeafOp<GridType, MaskGridType> func(
                field_context_inputs, procedure_executor, mask_grid);
            openvdb::tools::foreach (dst_grid.tree().beginLeaf(),
                                     func,
                                     /*threaded=*/true,
                                     /*shareOp=*/false);
          });
          break;
        }
      }
    });
  }
}

void evaluate_procedure_on_constant_volume_fields(ResourceScope & /*scope*/,
                                                  const mf::Procedure &procedure,
                                                  Span<GVGrid> field_context_inputs,
                                                  Span<GFieldRef> fields_to_evaluate,
                                                  Span<int> field_indices,
                                                  MutableSpan<GVGrid> r_grids)
{
  mf::ProcedureExecutor procedure_executor{procedure};
  const IndexMask mask(1);
  mf::ParamsBuilder mf_params{procedure_executor, &mask};
  mf::ContextBuilder mf_context;

  /* Provide inputs to the procedure executor. */
  for (const int i : field_context_inputs.index_range()) {
    BLI_assert(field_context_inputs[i].is_single());
    const CPPType &type = field_context_inputs[i].type();
    BUFFER_FOR_CPP_TYPE_VALUE(type, buffer);
    field_context_inputs[i].get_internal_single_to_uninitialized(buffer);
    GVArray varray = GVArray::ForSingle(field_context_inputs[i].type(), 1, buffer);
    mf_params.add_readonly_single_input(varray);
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

    r_grids[out_index] = GVGrid::ForSingle(type, output_buffers[out_index]);

    /* Destruct output value buffers, value is stored in #r_grids now. */
    type.destruct(output_buffers[i]);
  }
}

}  // namespace blender::fn

#endif
