/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_curves.hh"

#include "DNA_mesh_types.h"

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
static std::optional<int> while_next(const Span<int> next_index, const int first_i, Func &&func)
{
  if (!func(first_i)) {
    return std::nullopt;
  }
  int iter = first_i;
  while (func(next_index[iter])) {
    iter = next_index[iter];
  }
  return iter;
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

  int total_iter = 0;
  const int last_i = *while_next(next_index, vert_i, [&](const int i) -> bool {
    if (branch_size[i] != non_checked) {
      return false;
    }
    total_iter++;
    branch_size[i] = -total_iter;
    return true;
  });
  BLI_assert(total_iter >= 1);
  const int one_after_last_i = next_index[last_i];
  const bool unknown_end = branch_size[one_after_last_i] < 0;

  const int next_after_last = branch_size[last_i] - 1;
  const int first_in_cycle = branch_size[one_after_last_i];

  const int first = math::abs<int>(branch_size[vert_i]);
  const int last = math::abs<int>(unknown_end ? first_in_cycle : next_after_last);
  const int known_size = last - first + 1;

  if (unknown_end) {
    /* Usually cycle have just one vertex. */
    const int cycle_first = math::abs<int>(first_in_cycle);
    const int cycle_last = math::abs<int>(branch_size[last_i]);
    const int size_of_known_cycle = cycle_last - cycle_first + 1;
    BLI_assert(size_of_known_cycle > 0);
    while_next(next_index, one_after_last_i, [&](const int i) -> bool {
      branch_size[i] = size_of_known_cycle;
      return i != last_i;
    });
  }

  /* Other part is or already computed branch, or cycle. */
  const int prefix_of_other_part = branch_size[one_after_last_i];
  BLI_assert(prefix_of_other_part > 0);

  const constexpr int shared_vert_num = 1;
  const int total_size = known_size + prefix_of_other_part - shared_vert_num;
  BLI_assert(total_size > 0);

  int total = total_size;
  while_next(next_index, vert_i, [&](const int i) -> bool {
    BLI_assert(total > 0);
    branch_size[i] = total;
    total--;
    return i != one_after_last_i;
  });

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
  static blender::bke::bNodeType ntype;

  geo_node_type_base(
      &ntype, GEO_NODE_EDGE_PATHS_TO_CURVES, "Edge Paths to Curves", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  blender::bke::nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_edge_paths_to_curves_cc
