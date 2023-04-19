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
  b.add_input<decl::Int>(N_("Group ID")).supports_field().hide_value();

  b.add_output<decl::Int>(N_("Index")).field_source().description(N_("Index of nearest element"));
  b.add_output<decl::Bool>(N_("Has Neighbor")).field_source();
}

class IndexOfNearestFieldInput final : public bke::GeometryFieldInput {
 private:
  const Field<float3> positions_field_;
  const Field<int> group_field_;

 public:
  IndexOfNearestFieldInput(Field<float3> positions_field, Field<int> group_field)
      : bke::GeometryFieldInput(CPPType::get<int>(), "Nearest to"),
        positions_field_(std::move(positions_field)),
        group_field_(std::move(group_field))
  {
  }

  GVArray get_varray_for_context(const bke::GeometryFieldContext &context,
                                 IndexMask mask) const final
  {
    fn::FieldEvaluator evaluator{context, &mask};
    evaluator.add(positions_field_);
    evaluator.add(group_field_);
    evaluator.evaluate();

    const VArray<float3> &positions = evaluator.get_evaluated<float3>(0);
    const VArray<int> &group = evaluator.get_evaluated<int>(1);

    MultiValueMap<int, int64_t> group_masks;
    mask.foreach_index([&](const int index) { group_masks.add(group[index], index); });

    Array<int> indices(mask.min_array_size());

    const auto nearest_for = [this, &positions](const IndexMask mask, MutableSpan<int> r_indices) {
      devirtualize_varray(positions, [mask, r_indices, this](const auto positions) {
        KDTree_3d *tree = BLI_kdtree_3d_new(mask.size());
        mask.foreach_index([tree, positions](const int index) {
          BLI_kdtree_3d_insert(tree, index, positions[index]);
        });

        BLI_kdtree_3d_balance(tree);

        threading::parallel_for(mask.index_range(), 512, [&](const IndexRange range) {
          mask.slice(range).foreach_index([&](const auto index) {
            r_indices[index] = this->kdtree_find_neighboard(tree, positions[index], index);
          });
        });

        BLI_kdtree_3d_free(tree);
      });
    };

    for (const Span<int64_t> mask_span : group_masks.values()) {
      if (mask_span.size() == 1) {
        indices[mask_span.first()] = -1;
      }
      nearest_for(mask_span, indices);
    }

    return VArray<int>::ForContainer(std::move(indices));
  }

 protected:
  static int kdtree_find_neighboard(KDTree_3d *tree, const float3 &position, const int &index)
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
  }

  uint64_t hash() const override
  {
    return get_default_hash_2(positions_field_, group_field_);
  }

  bool is_equal_to(const fn::FieldNode &other) const override
  {
    if (const IndexOfNearestFieldInput *other_field =
            dynamic_cast<const IndexOfNearestFieldInput *>(&other)) {
      return positions_field_ == other_field->positions_field_ &&
             group_field_ == other_field->group_field_;
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
  Field<int> group_field = params.extract_input<Field<int>>("Group ID");

  Field<int> index_of_nearest_field(std::make_shared<IndexOfNearestFieldInput>(
      std::move(position_field), std::move(group_field)));

  if (params.output_is_required("Index")) {
    static auto clamp_fn = mf::build::SI1_SO<int, int>(
        "Index Clamping",
        [](const int index) { return math::max(0, index); },
        mf::build::exec_presets::Materialized());
    auto clamp_op = std::make_shared<FieldOperation>(
        FieldOperation(std::move(clamp_fn), {index_of_nearest_field}));
    params.set_output("Index", Field<int>(clamp_op, 0));
  }

  if (params.output_is_required("Has Neighbor")) {
    static auto valid_fn = mf::build::SI1_SO<int, bool>(
        "Index Validating",
        [](const int index) { return index != -1; },
        mf::build::exec_presets::Materialized());
    auto valid_op = std::make_shared<FieldOperation>(
        FieldOperation(std::move(valid_fn), {std::move(index_of_nearest_field)}));
    params.set_output("Has Neighbor", Field<bool>(valid_op, 0));
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
