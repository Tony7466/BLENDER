/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <queue>

#include "BLI_generic_array.hh"
#include "BLI_map.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_set.hh"
#include "BLI_task.hh"
#include "BLI_vector.hh"

#include "BLI_timeit.hh"

#include "BKE_mesh.hh"
#include "BKE_mesh_mapping.hh"
#include "BKE_type_conversions.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_input_shortest_edge_paths_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Bool>("End Vertex").default_value(false).hide_value().supports_field();
  b.add_input<decl::Float>("Edge Cost").default_value(1.0f).hide_value().supports_field();
  b.add_output<decl::Int>("Next Vertex Index").reference_pass_all();
  b.add_output<decl::Float>("Total Cost").reference_pass_all();
}

using VertPriorityUniform = std::pair<int, int>;

static void shortest_paths_uniform(const Mesh &mesh,
                                   const GroupedSpan<int> vert_to_edge,
                                   const IndexMask end_selection,
                                   MutableSpan<int> r_next_index,
                                   MutableSpan<int> r_cost)
{
  const Span<int2> edges = mesh.edges();
  Array<bool> visited(mesh.totvert, false);

  std::priority_queue<VertPriorityUniform,
                      std::vector<VertPriorityUniform>,
                      std::greater<VertPriorityUniform>>
      queue;

  end_selection.foreach_index([&](const int start_vert_i) {
    r_cost[start_vert_i] = 0;
    queue.emplace(0, start_vert_i);
  });

  Array<int> other_vertex(vert_to_edge.data.size());
  threading::parallel_for(vert_to_edge.index_range(), 2048, [&](const IndexRange range) {
    for (const int vert_i : range) {
      for (const int edge_i : vert_to_edge.offsets[vert_i]) {
        other_vertex[edge_i] = bke::mesh::edge_other_vert(edges[vert_to_edge.data[edge_i]],
                                                          vert_i);
      }
    }
  });
  const GroupedSpan<int> other_verts(vert_to_edge.offsets, other_vertex);

  while (!queue.empty()) {
    const int cost_i = queue.top().first;
    const int vert_i = queue.top().second;
    queue.pop();
    if (visited[vert_i]) {
      continue;
    }
    visited[vert_i] = true;
    for (const int neighbor_vert_i : other_verts[vert_i]) {
      if (visited[neighbor_vert_i]) {
        continue;
      }
      const int new_neighbour_cost = cost_i + 1;
      if (new_neighbour_cost < r_cost[neighbor_vert_i]) {
        r_cost[neighbor_vert_i] = new_neighbour_cost;
        r_next_index[neighbor_vert_i] = vert_i;
        queue.emplace(new_neighbour_cost, neighbor_vert_i);
      }
    }
  }
}

using VertPriority = std::pair<float, int>;

static void shortest_paths(const Mesh &mesh,
                           const GroupedSpan<int> vert_to_edge,
                           const IndexMask end_selection,
                           const VArray<float> &input_cost,
                           MutableSpan<int> r_next_index,
                           MutableSpan<float> r_cost)
{
  const Span<int2> edges = mesh.edges();
  Array<bool> visited(mesh.totvert, false);

  std::priority_queue<VertPriority, std::vector<VertPriority>, std::greater<VertPriority>> queue;

  end_selection.foreach_index([&](const int start_vert_i) {
    r_cost[start_vert_i] = 0.0f;
    queue.emplace(0.0f, start_vert_i);
  });

  Array<int> other_vertex(vert_to_edge.data.size());
  threading::parallel_for(vert_to_edge.index_range(), 2048, [&](const IndexRange range) {
    for (const int vert_i : range) {
      for (const int edge_i : vert_to_edge.offsets[vert_i]) {
        other_vertex[edge_i] = bke::mesh::edge_other_vert(edges[vert_to_edge.data[edge_i]],
                                                          vert_i);
      }
    }
  });

  while (!queue.empty()) {
    const float cost_i = queue.top().first;
    const int vert_i = queue.top().second;
    queue.pop();
    if (visited[vert_i]) {
      continue;
    }
    visited[vert_i] = true;
    for (const int index : vert_to_edge.offsets[vert_i]) {
      const int edge_i = vert_to_edge.data[index];
      const int neighbor_vert_i = other_vertex[index];
      if (visited[neighbor_vert_i]) {
        continue;
      }
      const float edge_cost = std::max(0.0f, input_cost[edge_i]);
      const float new_neighbour_cost = cost_i + edge_cost;
      if (new_neighbour_cost < r_cost[neighbor_vert_i]) {
        r_cost[neighbor_vert_i] = new_neighbour_cost;
        r_next_index[neighbor_vert_i] = vert_i;
        queue.emplace(new_neighbour_cost, neighbor_vert_i);
      }
    }
  }
}

template<typename T>
static void replace(MutableSpan<T> values, const T old_value, const T new_value)
{
  threading::parallel_for(values.index_range(), 1024, [&](const IndexRange range) {
    MutableSpan<T> values_slice = values.slice(range);
    std::replace(values_slice.begin(), values_slice.end(), old_value, new_value);
  });
}

class ShortestEdgePathsNextVertFieldInput final : public bke::MeshFieldInput {
 private:
  Field<bool> end_selection_;
  Field<float> cost_;

 public:
  ShortestEdgePathsNextVertFieldInput(Field<bool> end_selection, Field<float> cost)
      : bke::MeshFieldInput(CPPType::get<int>(), "Shortest Edge Paths Next Vertex Field"),
        end_selection_(end_selection),
        cost_(cost)
  {
    category_ = Category::Generated;
  }

  GVArray get_varray_for_context(const Mesh &mesh,
                                 const eAttrDomain domain,
                                 const IndexMask & /*mask*/) const final
  {
    const bke::MeshFieldContext edge_context{mesh, ATTR_DOMAIN_EDGE};
    fn::FieldEvaluator edge_evaluator{edge_context, mesh.totedge};
    edge_evaluator.add(cost_);
    edge_evaluator.evaluate();
    const VArray<float> input_cost = edge_evaluator.get_evaluated<float>(0);

    const bke::MeshFieldContext point_context{mesh, ATTR_DOMAIN_POINT};
    fn::FieldEvaluator point_evaluator{point_context, mesh.totvert};
    point_evaluator.add(end_selection_);
    point_evaluator.evaluate();
    const IndexMask end_selection = point_evaluator.get_evaluated_as_mask(0);

    Array<int> next_index(mesh.totvert, -1);

    if (end_selection.is_empty()) {
      array_utils::fill_index_range<int>(next_index);
      return mesh.attributes().adapt_domain<int>(
          VArray<int>::ForContainer(std::move(next_index)), ATTR_DOMAIN_POINT, domain);
    }

    const Span<int2> edges = mesh.edges();
    Array<int> vert_to_edge_offset_data;
    Array<int> vert_to_edge_indices;
    const GroupedSpan<int> vert_to_edge = bke::mesh::build_vert_to_edge_map(
        edges, mesh.totvert, vert_to_edge_offset_data, vert_to_edge_indices);

    if (input_cost.is_single()) {
      Array<int> cost(mesh.totvert, std::numeric_limits<int>::max());
      shortest_paths_uniform(mesh, vert_to_edge, end_selection, next_index, cost);
    }
    else {
      Array<float> cost(mesh.totvert, std::numeric_limits<float>::max());
      shortest_paths(mesh, vert_to_edge, end_selection, input_cost, next_index, cost);
    }

    replace<int>(next_index, -1, 0);
    return mesh.attributes().adapt_domain<int>(
        VArray<int>::ForContainer(std::move(next_index)), ATTR_DOMAIN_POINT, domain);
  }

  void for_each_field_input_recursive(FunctionRef<void(const FieldInput &)> fn) const override
  {
    end_selection_.node().for_each_field_input_recursive(fn);
    cost_.node().for_each_field_input_recursive(fn);
  }

  uint64_t hash() const override
  {
    /* Some random constant hash. */
    return 8466507837;
  }

  bool is_equal_to(const fn::FieldNode &other) const override
  {
    if (const ShortestEdgePathsNextVertFieldInput *other_field =
            dynamic_cast<const ShortestEdgePathsNextVertFieldInput *>(&other))
    {
      return other_field->end_selection_ == end_selection_ && other_field->cost_ == cost_;
    }
    return false;
  }

  std::optional<eAttrDomain> preferred_domain(const Mesh & /*mesh*/) const override
  {
    return ATTR_DOMAIN_POINT;
  }
};

class ShortestEdgePathsCostFieldInput final : public bke::MeshFieldInput {
 private:
  Field<bool> end_selection_;
  Field<float> cost_;

 public:
  ShortestEdgePathsCostFieldInput(Field<bool> end_selection, Field<float> cost)
      : bke::MeshFieldInput(CPPType::get<float>(), "Shortest Edge Paths Cost Field"),
        end_selection_(end_selection),
        cost_(cost)
  {
    category_ = Category::Generated;
  }

  GVArray get_varray_for_context(const Mesh &mesh,
                                 const eAttrDomain domain,
                                 const IndexMask & /*mask*/) const final
  {
    const bke::MeshFieldContext edge_context{mesh, ATTR_DOMAIN_EDGE};
    fn::FieldEvaluator edge_evaluator{edge_context, mesh.totedge};
    edge_evaluator.add(cost_);
    edge_evaluator.evaluate();
    const VArray<float> input_cost = edge_evaluator.get_evaluated<float>(0);

    const bke::MeshFieldContext point_context{mesh, ATTR_DOMAIN_POINT};
    fn::FieldEvaluator point_evaluator{point_context, mesh.totvert};
    point_evaluator.add(end_selection_);
    point_evaluator.evaluate();
    const IndexMask end_selection = point_evaluator.get_evaluated_as_mask(0);

    if (end_selection.is_empty()) {
      return mesh.attributes().adapt_domain<float>(
          VArray<float>::ForSingle(0.0f, mesh.totvert), ATTR_DOMAIN_POINT, domain);
    }

    Array<int> next_index(mesh.totvert, -1);
    GArray<> cost;

    const Span<int2> edges = mesh.edges();
    Array<int> vert_to_edge_offset_data;
    Array<int> vert_to_edge_indices;
    const GroupedSpan<int> vert_to_edge = bke::mesh::build_vert_to_edge_map(
        edges, mesh.totvert, vert_to_edge_offset_data, vert_to_edge_indices);

    if (input_cost.is_single()) {
      cost = GArray<>(CPPType::get<int>(), mesh.totvert);
      MutableSpan<int> typed_cost = cost.as_mutable_span().typed<int>();
      typed_cost.fill(std::numeric_limits<int>::max());
      shortest_paths_uniform(mesh, vert_to_edge, end_selection, next_index, typed_cost);
      replace(typed_cost, std::numeric_limits<int>::max(), 0);
    }
    else {
      cost = GArray<>(CPPType::get<float>(), mesh.totvert);
      MutableSpan<float> typed_cost = cost.as_mutable_span().typed<float>();
      typed_cost.fill(std::numeric_limits<float>::max());
      shortest_paths(mesh, vert_to_edge, end_selection, input_cost, next_index, typed_cost);
      replace(typed_cost, std::numeric_limits<float>::max(), 0.0f);
    }

    const bke::DataTypeConversions &conversion = bke::get_implicit_type_conversions();
    GVArray result_cost = conversion.try_convert(GVArray::ForGArray(std::move(cost)),
                                                 CPPType::get<float>());
    return mesh.attributes().adapt_domain(std::move(result_cost), ATTR_DOMAIN_POINT, domain);
  }

  void for_each_field_input_recursive(FunctionRef<void(const FieldInput &)> fn) const override
  {
    end_selection_.node().for_each_field_input_recursive(fn);
    cost_.node().for_each_field_input_recursive(fn);
  }

  uint64_t hash() const override
  {
    return get_default_hash_2(end_selection_, cost_);
  }

  bool is_equal_to(const fn::FieldNode &other) const override
  {
    if (const ShortestEdgePathsCostFieldInput *other_field =
            dynamic_cast<const ShortestEdgePathsCostFieldInput *>(&other))
    {
      return other_field->end_selection_ == end_selection_ && other_field->cost_ == cost_;
    }
    return false;
  }

  std::optional<eAttrDomain> preferred_domain(const Mesh & /*mesh*/) const override
  {
    return ATTR_DOMAIN_POINT;
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  Field<bool> end_selection = params.extract_input<Field<bool>>("End Vertex");
  Field<float> cost = params.extract_input<Field<float>>("Edge Cost");

  Field<int> next_vert_field{
      std::make_shared<ShortestEdgePathsNextVertFieldInput>(end_selection, cost)};
  Field<float> cost_field{std::make_shared<ShortestEdgePathsCostFieldInput>(end_selection, cost)};
  params.set_output("Next Vertex Index", std::move(next_vert_field));
  params.set_output("Total Cost", std::move(cost_field));
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(
      &ntype, GEO_NODE_INPUT_SHORTEST_EDGE_PATHS, "Shortest Edge Paths", NODE_CLASS_INPUT);
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_input_shortest_edge_paths_cc
