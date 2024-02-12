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

static bool is_dead_end(const Span<int> next_indices, const int vert)
{
  return vert == next_indices[vert];
}

static int size_of_loop(const Span<int> next_indices,
                        const Span<std::atomic<int>> rank,
                        const int vert)
{
  if (const int cycle_rank = rank[vert]; cycle_rank > 0) {
    return cycle_rank;
  }

  int cycle_size = 1;
  for (int cycle_vert = next_indices[vert]; cycle_vert != vert;
       cycle_vert = next_indices[cycle_vert])
  {
    BLI_assert(!is_dead_end(next_indices, cycle_vert));
    cycle_size++;
  }
  return cycle_size;
}

/**
 * Fill part of cycle, marked by mask value.
 * This part should be continuous sequence of connected vertices.
 * Filling by value of size of this cycle. */
static void replace_in_cycle(const Span<int> next_indices,
                             const int vert,
                             const int src,
                             const int dst,
                             MutableSpan<std::atomic<int>> rank)
{
  BLI_assert(src != dst);

  int iter_vert = vert;
  while (rank[iter_vert].load() != src) {
    iter_vert = next_indices[iter_vert];
    BLI_assert(!is_dead_end(next_indices, iter_vert));
  }

  while (rank[iter_vert].load() == src) {
    rank[iter_vert].store(dst);
    iter_vert = next_indices[iter_vert];
    BLI_assert(!is_dead_end(next_indices, iter_vert));
  }
}

static Curves *edge_paths_to_curves_convert(
    const Mesh &mesh,
    const IndexMask &start_verts_mask,
    const AnonymousAttributePropagationInfo &propagation_info,
    MutableSpan<int> next_indices)
{
  const IndexRange vert_range(mesh.verts_num);

  /* All the next points can pointing to itself. */
  {
    SCOPED_TIMER_AVERAGED("Loops incorrect indices");
    threading::parallel_for(next_indices.index_range(), 4096, [&](const IndexRange range) {
      MutableSpan<int> next_indices_range = next_indices.slice(range);
      for (const int vert_i : range) {
        int &next_vert = next_indices[vert_i];
        if (UNLIKELY(!vert_range.contains(next_vert))) {
          next_vert = vert_i;
        }
      }
    });
  }

  /* Do not create curves from first point with incorrect index. */
  IndexMaskMemory memory;
  IndexMask valid_start_verts;
  {
    SCOPED_TIMER_AVERAGED("Roots mask");
    valid_start_verts = IndexMask::from_predicate(
        start_verts_mask, GrainSize(4096), memory, [&](const int vert_i) {
          return !is_dead_end(next_indices, vert_i);
        });
  }

  const constexpr int non_checked = 0;

  Array<std::atomic<int>> rank(mesh.verts_num);
  {
    SCOPED_TIMER_AVERAGED("Init rank");
    threading::parallel_for(rank.index_range(), 4096, [&](const IndexRange range) {
      for (std::atomic<int> &rank : rank.as_mutable_span().slice(range)) {
        rank.store(non_checked, std::memory_order_relaxed);
      }
    });
  }

  const auto rank_for_vertex = [&](const int vertex) -> int {
    if (rank[vertex] > 0) {
      return rank[vertex];
    }
    const int vert_path_id = -(vertex + 1);
    int total_rank = 0;
    for (int current_vert = vertex; true; current_vert = next_indices[current_vert]) {
      int expected_rank = non_checked;
      /* Marking all points of this branch be id of this branch. Other connected branches will
       * read-only final rank of this point. */
      if (rank[current_vert].compare_exchange_strong(expected_rank, vert_path_id)) {
        total_rank++;
        if (is_dead_end(next_indices, current_vert)) {
          break;
        }
        continue;
      }

      if (expected_rank == vert_path_id) {
        /* Cycle formed by this branch. Other branches might be connected, but they will read-only
         * rank of cycle. */
        const int cycle_size = size_of_loop(next_indices, rank, current_vert);
        replace_in_cycle(next_indices, current_vert, vert_path_id, cycle_size, rank);
        total_rank += cycle_size;
        break;
      }

      /* One last point of marked by another branch, but this is last one, and it rank is known as
       * one. */
      if (is_dead_end(next_indices, current_vert)) {
        total_rank++;
        break;
      }

      // if (next_indices[next_indices[current_vert]] == current_vert && rank[...] < non_checked) {
      /* TODO: DEADLOCK OF an "A->B->C->...->A" CYCLE OF POINTS AND THREADS!!!!!. Have to solve
       * this case. */
      // }

      /* Await info from another thread to get final rank. Info is coming due to this is not zero,
       * but marked by other branch id. */
      while (rank[current_vert].load() < non_checked) {
      }
      const int current_rank = rank[current_vert].load();
      while (rank[next_indices[current_vert]].load() < non_checked) {
      }
      const int next_rank = rank[next_indices[current_vert]].load();

      const bool is_cycle = current_rank == next_rank;
      BLI_assert(ELEM(current_rank, next_rank, next_rank + 1));
      if (LIKELY(!is_cycle)) {
        total_rank += current_rank;
        break;
      }

      const int cycle_size = size_of_loop(next_indices, rank, current_vert);
      replace_in_cycle(next_indices, current_vert, vert_path_id, cycle_size, rank);
      total_rank += cycle_size;
    }

    for (int current_vert = vertex; rank[current_vert].load() == vert_path_id;
         current_vert = next_indices[current_vert])
    {
      rank[current_vert].store(total_rank);
      total_rank--;
    }

    return rank[vertex].load(std::memory_order_relaxed);
  };

  Array<int> curve_offsets(valid_start_verts.size() + 1);
  {
    SCOPED_TIMER_AVERAGED("Count rank");
    valid_start_verts.foreach_index(GrainSize(2048),
                                    [&](const int first_vert, const int vert_pos) {
                                      curve_offsets[vert_pos] = rank_for_vertex(first_vert);
                                    });
  }

  OffsetIndices<int> curves;
  {
    SCOPED_TIMER_AVERAGED("Accumulate");
    curves = offset_indices::accumulate_counts_to_offsets(curve_offsets);
  }
  if (curves.is_empty()) {
    return nullptr;
  }

  Array<int> indices(curves.total_size());
  {
    SCOPED_TIMER_AVERAGED("Gather indices");
    valid_start_verts.foreach_index(GrainSize(1024), [&](const int first_vert, const int pos) {
      MutableSpan<int> curve_indices = indices.as_mutable_span().slice(curves[pos]);
      int current_vert = first_vert;
      for (const int i : curve_indices.index_range()) {
        curve_indices[i] = current_vert;
        current_vert = next_indices[current_vert];
      }
    });
  }

  SCOPED_TIMER_AVERAGED("New curves");
  Curves *curves_id = bke::curves_new_nomain(
      geometry::create_curve_from_vert_indices(mesh.attributes(),
                                               indices,
                                               curve_offsets.as_span().drop_back(1),
                                               IndexRange(0),
                                               propagation_info));
  return curves_id;
}

static Curves *edge_paths_to_curves_convert_new(
    const Mesh &mesh,
    const IndexMask &start_verts_mask,
    const AnonymousAttributePropagationInfo &propagation_info,
    MutableSpan<int> next_indices)
{
  const IndexRange vert_range(mesh.verts_num);

  /* All the next points can pointing to itself. */
  {
    SCOPED_TIMER_AVERAGED("Loops incorrect indices");
    threading::parallel_for(next_indices.index_range(), 4096, [&](const IndexRange range) {
      for (const int vert_i : range) {
        int &next_vert = next_indices[vert_i];
        if (UNLIKELY(!vert_range.contains(next_vert))) {
          next_vert = vert_i;
        }
      }
    });
  }

  /* Do not create curves from first point with incorrect index. */
  IndexMaskMemory memory;
  IndexMask valid_start_verts;
  {
    SCOPED_TIMER_AVERAGED("Roots mask");
    valid_start_verts = IndexMask::from_predicate(
        start_verts_mask, GrainSize(4096), memory, [&](const int vert_i) {
          return !is_dead_end(next_indices, vert_i);
        });
  }

  Array<int> rank();
  Vector<int> threads

      Array<int>
          curve_offsets(valid_start_verts.size() + 1);
  {
    SCOPED_TIMER_AVERAGED("Count rank");
    valid_start_verts.foreach_index(GrainSize(2048),
                                    [&](const int first_vert, const int vert_pos) {
                                      curve_offsets[vert_pos] = rank_for_vertex(first_vert);
                                    });
  }

  OffsetIndices<int> curves;
  {
    SCOPED_TIMER_AVERAGED("Accumulate");
    curves = offset_indices::accumulate_counts_to_offsets(curve_offsets);
  }
  if (curves.is_empty()) {
    return nullptr;
  }

  Array<int> indices(curves.total_size());
  {
    SCOPED_TIMER_AVERAGED("Gather indices");
    valid_start_verts.foreach_index(GrainSize(1024), [&](const int first_vert, const int pos) {
      MutableSpan<int> curve_indices = indices.as_mutable_span().slice(curves[pos]);
      int current_vert = first_vert;
      for (const int i : curve_indices.index_range()) {
        curve_indices[i] = current_vert;
        current_vert = next_indices[current_vert];
      }
    });
  }

  SCOPED_TIMER_AVERAGED("New curves");
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
