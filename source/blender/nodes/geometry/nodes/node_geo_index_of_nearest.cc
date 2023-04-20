/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_kdtree.h"
#include "BLI_multi_value_map.hh"
#include "BLI_task.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_index_of_nearest_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Vector>(N_("Position")).implicit_field(implicit_field_inputs::position);
  b.add_input<decl::Int>(N_("Group ID")).supports_field().hide_value();

  b.add_output<decl::Int>(N_("Index")).field_source().description(N_("Index of nearest element"));
  b.add_output<decl::Bool>(N_("Has Neighbor")).field_source();
}

static KDTree_3d *build_kdtree(const Span<float3> &positions, const IndexMask mask)
{
  KDTree_3d *tree = BLI_kdtree_3d_new(mask.size());
  mask.foreach_index(
      [tree, positions](const int index) { BLI_kdtree_3d_insert(tree, i, positions[i]); });
  BLI_kdtree_3d_balance(tree);
  return tree;
}

static int find_nearest_non_self(const KDTree_3d &tree, const float3 &position, const int index)
{
  return BLI_kdtree_3d_find_nearest_cb_cpp(
      &tree, position, 0, [index](const int other, const float * /*co*/, const float /*dist_sq*/) {
        return index == other ? 0 : 1;
      });
}

static void find_neighbors(const KDTree_3d &tree,
                           const Span<float3> positions,
                           const IndexMask mask,
                           MutableSpan<int> indices)
{
  threading::parallel_for(mask.index_range(), 1024, [&](const IndexRange range) {
    mask.foreach_index([tree, positions](const int index) {
      indices[i] = find_nearest_non_self(tree, positions[i], i);
    });
  });
}

static Vector<IndexMask> masks_from_group_ids(const Span<int> group_ids,
                                              const IndexMask mask,
                                              MultiValueMap<int, int64_t> &storage)
{
  mask.foreach_index([&](const int i) { storage.add(group_ids[i], i); });
  Vector<IndexMask> masks;
  masks.reserve(storage.size());
  for (const Span<int64_t> indices : storage.values()) {
    masks.append(indices);
  }
  return masks;
}

class IndexOfNearestFieldInput final : public bke::GeometryFieldInput {
 private:
  const Field<float3> positions_field_;
  const Field<int> group_field_;

 public:
  IndexOfNearestFieldInput(Field<float3> positions_field, Field<int> group_field)
      : bke::GeometryFieldInput(CPPType::get<int>(), "Index of Nearest"),
        positions_field_(std::move(positions_field)),
        group_field_(std::move(group_field))
  {
  }

  GVArray get_varray_for_context(const bke::GeometryFieldContext &context,
                                 const IndexMask mask) const final
  {
    if (!context.attributes()) {
      return {};
    }
    const int domain_size = context.attributes()->domain_size(context.domain());
    fn::FieldEvaluator evaluator{context, domain_size};
    evaluator.add(positions_field_);
    evaluator.add(group_field_);
    evaluator.evaluate();
    const VArraySpan<float3> positions = evaluator.get_evaluated<float3>(0);
    const VArray<int> group = evaluator.get_evaluated<int>(1);

    Array<int> result(mask.min_array_size());

    if (group.is_single()) {
      const IndexMask full_mask = positions.index_range();
      KDTree_3d *tree = build_kdtree(positions, full_mask);
      find_neighbors(*tree, positions, mask, result);
      BLI_kdtree_3d_free(tree);
      return VArray<int>::ForContainer(std::move(result));
    }

    /* The goal is to build each tree and use it immediately, rather than building all trees and
     * sampling them later. That should help to keep the tree in caches before balancing and when
     * sampling many points. */
    const VArraySpan<int> group_ids(group);
    MultiValueMap<int, int64_t> group_mask_storage;
    const Vector<IndexMask> tree_masks = masks_from_group_ids(
        group_ids, group_ids.index_mask(), group_mask_storage);

    MultiValueMap<int, int64_t> evaluate_masks_storage;
    Vector<IndexMask> evaluate_masks;
    if (mask.size() < domain_size) {
      /* Separate masks for evaluation are only necessary if the mask mask
       * for field input evaluation doesn't have every element selected. */
      evaluate_masks = masks_from_group_ids(group_ids, mask, evaluate_masks_storage);
    }

    /* The grain size should be larger as each tree gets smaller. */
    const int avg_tree_size = group_ids.size() / group_mask_storage.size();
    const int grain_size = std::max(8192 / avg_tree_size, 1);
    threading::parallel_for(tree_masks.index_range(), grain_size, [&](const IndexRange range) {
      for (const int i : range) {
        const IndexMask tree_mask = tree_masks[i];
        const IndexMask evaluate_mask = evaluate_masks.is_empty() ? tree_mask : evaluate_masks[i];
        if (tree_masks[i].size() < 2) {
          result.as_mutable_span().fill_indices(evaluate_mask.indices(), 0);
        }
        else {
          KDTree_3d *tree = build_kdtree(positions, tree_mask);
          find_neighbors(*tree, positions, evaluate_mask, result);
          BLI_kdtree_3d_free(tree);
        }
      }
    });
    return VArray<int>::ForContainer(std::move(result));
  }

 public:
  void for_each_field_input_recursive(FunctionRef<void(const FieldInput &)> fn) const
  {
    positions_field_.node().for_each_field_input_recursive(fn);
    group_field_.node().for_each_field_input_recursive(fn);
  }

  uint64_t hash() const final
  {
    return get_default_hash_2(positions_field_, group_field_);
  }

  bool is_equal_to(const fn::FieldNode &other) const final
  {
    if (const auto *other_field = dynamic_cast<const IndexOfNearestFieldInput *>(&other)) {
      return positions_field_ == other_field->positions_field_ &&
             group_field_ == other_field->group_field_;
    }
    return false;
  }

  std::optional<eAttrDomain> preferred_domain(const GeometryComponent &component) const final
  {
    return bke::try_detect_field_domain(component, positions_field_);
  }
};

class HasNeighborFieldInput final : public bke::GeometryFieldInput {
 private:
  const Field<int> group_field_;

 public:
  HasNeighborFieldInput(Field<int> group_field)
      : bke::GeometryFieldInput(CPPType::get<bool>(), "Has Neighbor"),
        group_field_(std::move(group_field))
  {
  }

  GVArray get_varray_for_context(const bke::GeometryFieldContext &context,
                                 const IndexMask mask) const final
  {
    if (!context.attributes()) {
      return {};
    }
    const int domain_size = context.attributes()->domain_size(context.domain());
    fn::FieldEvaluator evaluator{context, domain_size};
    evaluator.add(group_field_);
    evaluator.evaluate();
    const VArray<int> group = evaluator.get_evaluated<int>(0);

    if (group.is_single()) {
      return VArray<bool>::ForSingle(true, mask.min_array_size());
    }

    /* When a group ID is contained in the set, it means there is only one element with that ID. */
    Map<int, int> counts;
    const VArraySpan<int> group_span(group);
    mask.foreach_index([&](const int i) {
      counts.add_or_modify(
          group_span[i], [](int *count) { *count = 0; }, [](int *count) { (*count)++; });
    });
    Array<bool> result(mask.min_array_size());
    mask.foreach_index([&](const int i) { result[i] = counts.lookup(group_span[i]) > 1; });
    return VArray<bool>::ForContainer(std::move(result));
  }

 public:
  void for_each_field_input_recursive(FunctionRef<void(const FieldInput &)> fn) const
  {
    group_field_.node().for_each_field_input_recursive(fn);
  }

  uint64_t hash() const final
  {
    return get_default_hash_2(3984756934876, group_field_);
  }

  bool is_equal_to(const fn::FieldNode &other) const final
  {
    if (const auto *other_field = dynamic_cast<const HasNeighborFieldInput *>(&other)) {
      return group_field_ == other_field->group_field_;
    }
    return false;
  }

  std::optional<eAttrDomain> preferred_domain(const GeometryComponent &component) const final
  {
    return bke::try_detect_field_domain(component, group_field_);
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  Field<float3> position_field = params.extract_input<Field<float3>>("Position");
  Field<int> group_field = params.extract_input<Field<int>>("Group ID");

  if (params.output_is_required("Index")) {
    params.set_output("Index",
                      Field<int>(std::make_shared<IndexOfNearestFieldInput>(
                          std::move(position_field), group_field)));
  }

  if (params.output_is_required("Has Neighbor")) {
    params.set_output(
        "Has Neighbor",
        Field<bool>(std::make_shared<HasNeighborFieldInput>(std::move(group_field))));
  }
}

}  // namespace blender::nodes::node_geo_index_of_nearest_cc

void register_node_type_geo_index_of_nearest()
{
  namespace file_ns = blender::nodes::node_geo_index_of_nearest_cc;

  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_INDEX_OF_NEAREST, "Index of Nearest", NODE_CLASS_CONVERTER);
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  ntype.declare = file_ns::node_declare;
  nodeRegisterType(&ntype);
}
