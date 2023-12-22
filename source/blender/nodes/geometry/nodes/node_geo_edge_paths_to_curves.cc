/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <atomic>

#include "BKE_curves.hh"

#include "DNA_mesh_types.h"

#include "BLI_atomic_disjoint_set.hh"
#include "BLI_timeit.hh"

#include "GEO_mesh_to_curve.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_edge_paths_to_curves_cc {

/*
class IndexSequence {
 protected:
  Span<int> indices_;

 public:
  IndexSequence(const Span<int> indices) : indices_(indices) {}

  class BaseIterator {
   protected:
    Span<int> indices_;

   public:
    BaseIterator(const Span<int> indices) : indices_(indices) {}

    int next_for(const int index) const
    {
      return this->indices_[index];
    }

    bool is_end(const int index) const
    {
      return index == this->indices_[index];
    }
  };

  class Iterator : public BaseIterator {

  }
};
*/

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Mesh").supported_type(GeometryComponent::Type::Mesh);
  b.add_input<decl::Bool>("Start Vertices").default_value(true).hide_value().field_on_all();
  b.add_input<decl::Int>("Next Vertex Index").default_value(-1).hide_value().field_on_all();
  b.add_output<decl::Geometry>("Curves").propagate_all();
}

static int index_after(const Span<int> indices, const int start, const int range)
{
  int last = start;
  for ([[maybe_unused]] const int i : IndexRange(range)) {
    // printf("%s;\n", AT);
    last = indices[last];
  }
  return last;
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
  IndexMask valid_start_verts;
  {
    SCOPED_TIMER_AVERAGED("Roots mask");
    valid_start_verts = IndexMask::from_predicate(
        start_verts_mask, GrainSize(4096), memory, [&](const int vert_i) {
          return vert_range.contains(next_indices[vert_i]);
        });
  }

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

  const auto vert_in_cycle = [&](const int vert) -> bool {
    BLI_assert(vert != next_indices[vert]);
    const int current_rank = rank[vert].load();
    BLI_assert(current_rank > 0);

    const int next_vert = next_indices[vert];
    while (true) {
      // printf("%s;\n", AT);
      /* In some odd case, other thread might already write current vert rank, but still don't
       * finish next one. Await for this. */
      const int next_rank = rank[next_vert].load();
      if (next_rank > 0) {
        return current_rank == next_rank;
      }
    }
  };

  const auto rank_for_vertex = [&](const int vertex) -> int {
    if (rank[vertex].load(std::memory_order_relaxed) > 0) {
      return rank[vertex];
    }

    const int vert_path_id = -(vertex + 1);

    int total_rank = 0;

    for (int current_vert = vertex; true; current_vert = next_indices[current_vert]) {
      // printf("%s;\n", AT);
      total_rank++;

      int expected_rank = non_checked;
      if (rank[current_vert].compare_exchange_strong(expected_rank, vert_path_id)) {
        // printf("<< Next: %d;\n", expected_rank);
        // printf("%s;\n", AT);
        continue;
      }
      // printf(">> Next: %d;\n", expected_rank);

      if (expected_rank == vert_path_id) {
        int cycle_size = 1;
        for (int cycle_vert = next_indices[current_vert]; cycle_vert != current_vert;
             cycle_vert = next_indices[cycle_vert])
        {
          // printf("%s;\n", AT);
          cycle_size++;
        }
        rank[current_vert].store(cycle_size);
        for (int cycle_vert = next_indices[current_vert]; cycle_vert != current_vert;
             cycle_vert = next_indices[cycle_vert])
        {
          // printf("%s;\n", AT);
          rank[cycle_vert].store(cycle_size);
        }
        total_rank += cycle_size - 1;

        break;
      }

      if (expected_rank > 0) {
        const bool in_cycle = vert_in_cycle(current_vert);
        if (!in_cycle) {
          total_rank += expected_rank - 1;
          break;
        }

        const int cycle_size = expected_rank;

        int affected_part_of_cycle = 0;
        for (int cycle_vert = current_vert; rank[cycle_vert].load() != vert_path_id;
             cycle_vert = next_indices[cycle_vert])
        {
          // printf("%s;\n", AT);
          affected_part_of_cycle++;
        }
        const int gradient_depth = total_rank - (cycle_size - affected_part_of_cycle);

        int affected_cycle_vert = index_after(next_indices, vertex, gradient_depth);
        for ([[maybe_unused]] const int i : IndexRange(affected_part_of_cycle)) {
          // printf("%s;\n", AT);
          rank[affected_cycle_vert].store(cycle_size);
          affected_cycle_vert = next_indices[affected_cycle_vert];
        }

        break;
      }

      if (expected_rank < 0) {
        while (true) {
          // printf("%s;\n", AT);
          if (rank[current_vert].load() > 0) {
            continue;
          }
        }
      }
    }

    int last = -1;
    for (int current_vert = vertex; rank[current_vert].load() == vert_path_id;
         current_vert = next_indices[current_vert])
    {
      last = current_vert;
      // printf("%s ++;\n", AT);
      rank[current_vert].store(total_rank);
      total_rank--;
    }

    BLI_assert(rank[next_indices[last]] > 0);

    BLI_assert(total_rank >= 0);

    BLI_assert(!rank.as_span().contains(vert_path_id));

    return rank[vertex].load(std::memory_order_relaxed);
  };

  Array<int> curve_offsets(valid_start_verts.size() + 1);
  {
    SCOPED_TIMER_AVERAGED("Count rank");
    valid_start_verts.foreach_index(GrainSize(2048),
                                    [&](const int first_vert, const int vert_pos) {
                                      curve_offsets[vert_pos] = rank_for_vertex(first_vert);
                                      // printf("Rank: %d;\n", curve_offsets[vert_pos]);
                                    });
  }

  OffsetIndices<int> curves;
  {
    SCOPED_TIMER_AVERAGED("Accumulate");
    curves = offset_indices::accumulate_counts_to_offsets(curve_offsets);
  }
  if (curves.is_empty()) {
    // printf("HMMM?\n");
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

    const bke::MeshFieldContext context{*mesh, ATTR_DOMAIN_POINT};
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
