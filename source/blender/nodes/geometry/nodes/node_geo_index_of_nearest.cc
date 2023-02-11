/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute_math.hh"

#include "NOD_socket_search_link.hh"

#include "BLI_kdtree.h"
#include "BLI_multi_value_map.hh"
#include "BLI_task.hh"

#include "BKE_geometry_fields.hh"

#include "node_geometry_util.hh"

#include "UI_interface.h"
#include "UI_resources.h"

namespace blender::nodes::node_geo_index_of_nearest_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Vector>("Position").implicit_field(implicit_field_inputs::position);

  b.add_input<decl::Int>("Self Group ID").supports_field().hide_value().default_value(0);
  b.add_input<decl::Int>("Group ID to Search").supports_field().hide_value().default_value(0);

  b.add_output<decl::Int>("Index").field_source().description(N_("Index of nearest element"));
}

int kdtree_find_neighboard(KDTree_3d *tree, const float3 &position, int index)
{
  return BLI_kdtree_3d_find_nearest_cb(
      tree, position, 0, [index](const int other_new_i, const float * /*co*/, float /*dist_sq*/) {
        if (index == other_new_i) {
          return 0;
        }
        return 1;
      });
}

class IndexOfNearestFieldInput final : public bke::GeometryFieldInput {
 private:
  const Field<float3> positions_;
  const Field<int> group_;
  const Field<int> search_group_;

 public:
  IndexOfNearestFieldInput(const Field<float3> positions,
                           const Field<int> group,
                           const Field<int> search_group)
      : bke::GeometryFieldInput(CPPType::get<int>(), "Nearest to"),
        positions_(std::move(positions)),
        group_(std::move(group)),
        search_group_(std::move(search_group))
  {
  }

  GVArray get_varray_for_context(const bke::GeometryFieldContext &context,
                                 IndexMask mask) const final
  {
    fn::FieldEvaluator evaluator{context, &mask};
    evaluator.add(positions_);
    evaluator.add(group_);
    evaluator.add(search_group_);
    evaluator.evaluate();

    const VArray<float3> positions = evaluator.get_evaluated<float3>(0);
    const VArray<int> group = evaluator.get_evaluated<int>(1);
    const VArray<int> search_group = evaluator.get_evaluated<int>(2);

    const bool group_use = !group.is_single();
    const bool group_to_find_use = !search_group.is_single();

    MultiValueMap<int, int64_t> in_group;
    MultiValueMap<int, int64_t> out_group;
    Array<int> indices(mask.min_array_size());

    threading::parallel_invoke((indices.size() > 512) && group_to_find_use && group_use,
                               [&]() {
                                 if (group_use) {
                                   for (const int64_t i : mask.index_range()) {
                                     in_group.add(group[mask[i]], i);
                                   }
                                 }
                                 else {
                                   const int group_key = group.get_internal_single();
                                   in_group.add_multiple(group_key, {});
                                 }
                               },
                               [&]() {
                                 if (group_to_find_use) {
                                   for (const int64_t i : mask.index_range()) {
                                     out_group.add(search_group[mask[i]], i);
                                   }
                                 }
                               });

    for (int key : in_group.keys()) {
      /* Never empty. */
      const Span<int64_t> self_points = group_use ? in_group.lookup(key) : mask.indices();
      const Span<int64_t> search_points = group_to_find_use ? out_group.lookup(key) : self_points;

      if (search_points.is_empty()) {
        indices.as_mutable_span().fill_indices(self_points, 0);
        continue;
      }

      KDTree_3d *tree = BLI_kdtree_3d_new(search_points.size());
      for (const int64_t index : search_points) {
        BLI_kdtree_3d_insert(tree, index, positions[index]);
      }

      BLI_kdtree_3d_balance(tree);

      threading::parallel_for(self_points.index_range(), 128, [&](const IndexRange range) {
        for (const int64_t index : self_points.slice(range)) {
          const int index_of_nearest = kdtree_find_neighboard(tree, positions[index], index);
          if (index == -1) {
            indices[index] = index;
          }
          else {
            indices[index] = index_of_nearest;
          }
        }
      });

      BLI_kdtree_3d_free(tree);
    }

    return VArray<int>::ForContainer(std::move(indices));
  }

  void for_each_field_input_recursive(FunctionRef<void(const FieldInput &)> fn) const
  {
    positions_.node().for_each_field_input_recursive(fn);
    group_.node().for_each_field_input_recursive(fn);
    search_group_.node().for_each_field_input_recursive(fn);
  }

  uint64_t hash() const override
  {
    return get_default_hash_3(positions_, group_, search_group_);
  }

  bool is_equal_to(const fn::FieldNode &other) const override
  {
    if (const IndexOfNearestFieldInput *other_field =
            dynamic_cast<const IndexOfNearestFieldInput *>(&other)) {
      return positions_ == other_field->positions_ && group_ == other_field->group_ &&
             search_group_ == other_field->search_group_;
    }
    return false;
  }

  std::optional<eAttrDomain> preferred_domain(const GeometryComponent &component) const override
  {
    return bke::try_detect_field_domain(component, positions_);
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  const Field<float3> position = params.extract_input<Field<float3>>("Position");

  const Field<int> self_group = params.extract_input<Field<int>>("Self Group ID");
  const Field<int> search_group = params.extract_input<Field<int>>("Group ID to Search");

  params.set_output("Index",
                    Field<int>{std::make_shared<IndexOfNearestFieldInput>(
                        std::move(position), std::move(self_group), std::move(search_group))});
}

}  // namespace blender::nodes::node_geo_index_of_nearest_cc

void register_node_type_geo_index_of_nearest()
{
  namespace file_ns = blender::nodes::node_geo_index_of_nearest_cc;

  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_INDEX_OF_NEAREST, "Index Of Nearest", NODE_CLASS_CONVERTER);
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  ntype.declare = file_ns::node_declare;
  nodeRegisterType(&ntype);
}
