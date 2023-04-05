/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute_math.hh"

#include "BLI_kdtree.h"
#include "BLI_multi_value_map.hh"
#include "BLI_task.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_index_of_nearest_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Vector>(N_("Position")).implicit_field(implicit_field_inputs::position);

  b.add_input<decl::Int>(N_("Self Group ID")).supports_field().hide_value().default_value(0);
  b.add_input<decl::Int>(N_("Nearest Group ID")).supports_field().hide_value().default_value(0);

  b.add_output<decl::Int>(N_("Index")).field_source().description(N_("Index of nearest element"));
  b.add_output<decl::Bool>(N_("Valid")).field_source();
}

class IndexOfNearestFieldInput final : public bke::GeometryFieldInput {
 private:
  const Field<float3> positions_field_;
  const Field<int> group_field_;
  const Field<int> search_group_field_;

 public:
  IndexOfNearestFieldInput(const Field<float3> positions_field,
                           const Field<int> group_field,
                           const Field<int> search_group_field)
      : bke::GeometryFieldInput(CPPType::get<int>(), "Nearest to"),
        positions_field_(std::move(positions_field)),
        group_field_(std::move(group_field)),
        search_group_field_(std::move(search_group_field))
  {
  }

  GVArray get_varray_for_context(const bke::GeometryFieldContext &context,
                                 IndexMask mask) const final
  {
    fn::FieldEvaluator evaluator{context, &mask};
    evaluator.add(positions_field_);
    evaluator.add(group_field_);
    evaluator.add(search_group_field_);
    evaluator.evaluate();

    const VArray<float3> &positions = evaluator.get_evaluated<float3>(0);
    const VArray<int> &group = evaluator.get_evaluated<int>(1);
    const VArray<int> &search_group = evaluator.get_evaluated<int>(2);

    const bool use_group = !group.is_single();
    const bool use_search_group = !search_group.is_single();

    MultiValueMap<int, int64_t> in_group;
    MultiValueMap<int, int64_t> out_group;
    Array<int> indices(mask.min_array_size());

    threading::parallel_invoke(
        (indices.size() > 512) && use_search_group && use_group,
        [&]() {
          if (use_group) {
            mask.foreach_index([&](const auto index) { in_group.add(group[index], index); });
            return;
          }
          const int group_key = group.get_internal_single();
          in_group.add_multiple(group_key, {});
        },
        [&]() {
          if (use_search_group) {
            mask.foreach_index(
                [&](const auto index) { out_group.add(search_group[index], index); });
          }
        });

    for (const int key : in_group.keys()) {
      /* Never empty. */
      const IndexMask self_points(use_group ? IndexMask(in_group.lookup(key)) : mask);
      const IndexMask search_points(use_search_group ? IndexMask(out_group.lookup(key)) :
                                                       self_points);

      if (search_points.is_empty()) {
        indices.as_mutable_span().fill_indices(self_points, -1);
        continue;
      }

      KDTree_3d *tree = BLI_kdtree_3d_new(search_points.size());

      for (const int64_t index : search_points) {
        BLI_kdtree_3d_insert(tree, index, positions[index]);
      }

      BLI_kdtree_3d_balance(tree);

      threading::parallel_for(self_points.index_range(), 512, [&](const IndexRange range) {
        for (const int64_t index : self_points.slice(range)) {
          indices[index] = this->kdtree_find_neighboard(tree, positions[index], index);
        }
      });

      BLI_kdtree_3d_free(tree);
    }

    return VArray<int>::ForContainer(std::move(indices));
  }

 protected:
  int kdtree_find_neighboard(KDTree_3d *tree, const float3 &position, const int index) const
  {
    return BLI_kdtree_3d_find_nearest_cb_cpp(
        tree,
        position,
        0,
        [index](const int other_new_i, const float * /*co*/, const float /*dist_sq*/) {
          if (index == other_new_i) {
            return 0;
          }
          return 1;
        });
  }

 public:
  void for_each_field_input_recursive(FunctionRef<void(const FieldInput &)> fn) const
  {
    positions_field_.node().for_each_field_input_recursive(fn);
    group_field_.node().for_each_field_input_recursive(fn);
    search_group_field_.node().for_each_field_input_recursive(fn);
  }

  uint64_t hash() const override
  {
    return get_default_hash_3(positions_field_, group_field_, search_group_field_);
  }

  bool is_equal_to(const fn::FieldNode &other) const override
  {
    if (const IndexOfNearestFieldInput *other_field =
            dynamic_cast<const IndexOfNearestFieldInput *>(&other)) {
      return positions_field_ == other_field->positions_field_ &&
             group_field_ == other_field->group_field_ &&
             search_group_field_ == other_field->search_group_field_;
    }
    return false;
  }

  std::optional<eAttrDomain> preferred_domain(const GeometryComponent &component) const override
  {
    return bke::try_detect_field_domain(component, positions_field_);
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  Field<float3> position_field = params.extract_input<Field<float3>>("Position");

  Field<int> self_group_field = params.extract_input<Field<int>>("Self Group ID");
  Field<int> search_group_field = params.extract_input<Field<int>>("Nearest Group ID");

  Field<int> index_of_nearest_field(std::make_shared<IndexOfNearestFieldInput>(
      std::move(position_field), std::move(self_group_field), std::move(search_group_field)));

  if (params.output_is_required("Index")) {
    static auto clamp_fn = mf::build::SI1_SO<int, int>(
        "Index Clamping",
        [](const int index) { return math::max(0, index); },
        mf::build::exec_presets::Materialized());
    auto clamp_op = std::make_shared<FieldOperation>(
        FieldOperation(std::move(clamp_fn), {index_of_nearest_field}));
    params.set_output("Index", Field<int>(clamp_op, 0));
  }

  if (params.output_is_required("Valid")) {
    static auto valid_fn = mf::build::SI1_SO<int, bool>(
        "Index Validating",
        [](const int index) { return index != -1; },
        mf::build::exec_presets::Materialized());
    auto valid_op = std::make_shared<FieldOperation>(
        FieldOperation(std::move(valid_fn), {std::move(index_of_nearest_field)}));
    params.set_output("Valid", Field<bool>(valid_op, 0));
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
