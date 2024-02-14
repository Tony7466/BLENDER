/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <atomic>

#include "BKE_curves.hh"

#include "DNA_mesh_types.h"

#include "BLI_timeit.hh"

#include "GEO_mesh_to_curve.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_edge_paths_to_curves_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Mesh").supported_type(GeometryComponent::Type::Mesh);
  b.add_input<decl::Bool>("Start Vertices").default_value(true).hide_value().field_on_all();
  b.add_input<decl::Int>("Next Vertex Index").default_value(-1).hide_value().field_on_all();
  b.add_output<decl::Geometry>("Curves").propagate_all();
}

constexpr int non_checked = 0;

template<typename Func>
static int foreach (const Span<int> next_index, const int first_i, Func && func)
{
  int iter = first_i;
  for (; func(next_index[iter]); iter = next_index[iter]) {
  }
  return iter;
}

static int size_between(const int a, const int b)
{
  return math::abs<int>(a - b) + 1;
}

static int size_of_branch(const Span<int> next_index,
                          MutableSpan<int> branch_size,
                          const int vert_i)
{
  BLI_assert(!std::any_of(
      branch_size.begin(), branch_size.end(), [](const int size) { return size < 0; }));
  if (branch_size[vert_i] != non_checked) {
    return branch_size[vert_i];
  }

  int size = 1;
  branch_size[vert_i] = -1;
  const int last_i = foreach (next_index, vert_i, [&](const int i) -> bool {
    if (branch_size[i] != non_checked) {
      return false;
    }
    size++;
    branch_size[i] = -size;
    return true;
  });
  const int cycle_start_i = next_index[last_i];

  const int prefix_end = branch_size[cycle_start_i] > 0 ? branch_size[last_i] - 1 :
                                                          branch_size[cycle_start_i];
  const int prefix_size = size_between(math::abs<int>(branch_size[vert_i]),
                                       math::abs<int>(prefix_end));

  const bool unknown_end = branch_size[cycle_start_i] < 0;
  if (unknown_end) {
    const int cycle_size = size_between(math::abs<int>(branch_size[last_i]),
                                        math::abs<int>(branch_size[cycle_start_i]));
    foreach (next_index, last_i, [&](const int i) -> bool {
      branch_size[i] = cycle_size;
      return i != last_i;
    })
      ;
  }

  const int total_size = prefix_size + branch_size[cycle_start_i] - 1;
  BLI_assert(total_size > 0);

  int total = total_size;
  branch_size[vert_i] = total_size;
  foreach (next_index, vert_i, [&](const int i) -> bool {
    if (i == next_index[last_i]) {
      return false;
    }
    total--;
    BLI_assert(total > 0);
    branch_size[i] = total;
    return true;
  })
    ;

  return total_size;
}

static Curves *edge_paths_to_curves_convert(
    const Mesh &mesh,
    const IndexMask &start_verts_mask,
    const AnonymousAttributePropagationInfo &propagation_info,
    MutableSpan<int> next_indices)
{
  const IndexRange vert_range(mesh.verts_num);

  /* Do not create curves from first point with incorrect index. */
  IndexMaskMemory memory;
  IndexMask valid_start_verts = IndexMask::from_predicate(
      start_verts_mask, GrainSize(4096), memory, [&](const int vert_i) {
        return vert_range.contains(next_indices[vert_i]);
      });

  /* All the next points can pointing to itself. */
  threading::parallel_for(next_indices.index_range(), 4096, [&](const IndexRange range) {
    MutableSpan<int> next_indices_range = next_indices.slice(range);
    for (const int vert_i : range) {
      int &next_vert = next_indices[vert_i];
      if (UNLIKELY(!vert_range.contains(next_vert))) {
        next_vert = vert_i;
      }
    }
  });

  Array<int> branch_size(mesh.verts_num, non_checked);
  Array<int> curve_offsets(valid_start_verts.size() + 1);
  valid_start_verts.foreach_index([&](const int first_vert, const int vert_pos) {
    curve_offsets[vert_pos] = size_of_branch(next_indices, branch_size, first_vert);
  });

  OffsetIndices<int> curves = offset_indices::accumulate_counts_to_offsets(curve_offsets);
  if (curves.is_empty()) {
    return nullptr;
  }

  Array<int> indices(curves.total_size());
  valid_start_verts.foreach_index(GrainSize(1024), [&](const int first_vert, const int pos) {
    MutableSpan<int> curve_indices = indices.as_mutable_span().slice(curves[pos]);
    int current_vert = first_vert;
    for (const int i : curve_indices.index_range()) {
      curve_indices[i] = current_vert;
      current_vert = next_indices[current_vert];
    }
  });

  Curves *curves_id = bke::curves_new_nomain(
      geometry::create_curve_from_vert_indices(mesh.attributes(),
                                               indices,
                                               curve_offsets.as_span().drop_back(1),
                                               IndexRange(0),
                                               propagation_info));
  return curves_id;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Mesh");

  geometry_set.modify_geometry_sets([&](GeometrySet &geometry_set) {
    const Mesh *mesh = geometry_set.get_mesh();
    if (mesh == nullptr) {
      geometry_set.keep_only({GeometryComponent::Type::Instance});
      return;
    }

    const bke::MeshFieldContext context{*mesh, AttrDomain::Point};
    fn::FieldEvaluator evaluator{context, mesh->verts_num};
    Array<int> next_vert(mesh->verts_num);
    evaluator.add_with_destination(params.get_input<Field<int>>("Next Vertex Index"),
                                   next_vert.as_mutable_span());
    evaluator.add(params.get_input<Field<bool>>("Start Vertices"));
    evaluator.evaluate();
    IndexMask start_verts = evaluator.get_evaluated_as_mask(1);

    if (start_verts.is_empty()) {
      geometry_set.keep_only({GeometryComponent::Type::Instance});
      return;
    }

    geometry_set.replace_curves(edge_paths_to_curves_convert(
        *mesh, start_verts, params.get_output_propagation_info("Curves"), next_vert));
    geometry_set.keep_only({GeometryComponent::Type::Curve, GeometryComponent::Type::Instance});
  });

  params.set_output("Curves", std::move(geometry_set));
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(
      &ntype, GEO_NODE_EDGE_PATHS_TO_CURVES, "Edge Paths to Curves", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_edge_paths_to_curves_cc
