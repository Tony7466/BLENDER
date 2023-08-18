/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"

#include "BKE_mesh_mapping.hh"

#include "BLI_array_utils.hh"
#include "BLI_disjoint_set.hh"
#include "BLI_vector_set.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_input_mesh_edge_rings_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Int>("Parallel Group Index")
      .field_source()
      .description("Edge group index of the edge ring from parallel edges");
  b.add_output<decl::Int>("Linear Group Index").field_source();
}

class EdgesLineGroupFieldInput final : public bke::MeshFieldInput {
 public:
  EdgesLineGroupFieldInput() : bke::MeshFieldInput(CPPType::get<int>(), "Edges Line Rings Field")
  {
  }

  GVArray get_varray_for_context(const Mesh &mesh,
                                 const eAttrDomain domain,
                                 const IndexMask &mask) const final
  {
    const Span<int> corner_verts = mesh.corner_verts();

    Array<int2> corner_edge_pairs(mesh.totloop);

    const Span<int2> edges = mesh.edges();
    const Span<int> corner_edges = mesh.corner_edges();
    const OffsetIndices<int> faces = mesh.faces();
    for (const int face_index : IndexRange(mesh.faces_num)) {
      const IndexRange face_corners = faces[face_index];
      for (const int corner_index : face_corners) {
        const int vert_index = corner_verts[corner_index];
        const int edge_a = corner_edges[corner_index];
        corner_edge_pairs[corner_index][0] = edge_a;
        BLI_assert(ELEM(vert_index, edges[edge_a][0], edges[edge_a][1]));
        const int next = bke::mesh::face_corner_next(face_corners, corner_index);
        const int edge_b = corner_edges[next];
        if (ELEM(vert_index, edges[edge_b][0], edges[edge_b][1])) {
          corner_edge_pairs[corner_index][1] = edge_b;
          continue;
        }
        const int prev = bke::mesh::face_corner_prev(face_corners, corner_index);
        const int edge_c = corner_edges[prev];
        if (ELEM(vert_index, edges[edge_c][0], edges[edge_c][1])) {
          corner_edge_pairs[corner_index][1] = edge_c;
          continue;
        }
        BLI_assert_unreachable();
      }
    }

    Array<int> vert_to_edge_offsets;
    Array<int> vert_to_edge_indices;
    const GroupedSpan<int> vert_to_edge_map = bke::mesh::build_vert_to_edge_map(
        edges, mesh.totvert, vert_to_edge_offsets, vert_to_edge_indices);

    Array<int> vert_to_loop_offsets;
    Array<int> vert_to_loop_indices;
    const GroupedSpan<int> vert_to_loop_map = bke::mesh::build_vert_to_loop_map(
        corner_verts, mesh.totvert, vert_to_loop_offsets, vert_to_loop_indices);
    Array<int2> vert_to_loop_edge_pairs(mesh.totloop);
    array_utils::gather(corner_edge_pairs.as_span(),
                        vert_to_loop_indices.as_span(),
                        vert_to_loop_edge_pairs.as_mutable_span());

    for (const int vert_index : IndexRange(mesh.totvert)) {
      const IndexRange corners = vert_to_loop_map.offsets[vert_index];
      const Span<int2> star_pairs = vert_to_loop_edge_pairs.as_span().slice(corners);
      for (const int2 edge_indices : star_pairs) {
        const int2 edge_a = edges[edge_indices[0]];
        const int2 edge_b = edges[edge_indices[1]];
        BLI_assert(ELEM(vert_index, edge_a[0], edge_a[1]));
        BLI_assert(ELEM(vert_index, edge_b[0], edge_b[1]));
      }
    }

    DisjointSet<int> parallel_edges(mesh.totedge);

    for (const int vert_index : IndexRange(mesh.totvert)) {
      const IndexRange corners = vert_to_loop_map.offsets[vert_index];
      MutableSpan<int2> star_pairs = vert_to_loop_edge_pairs.as_mutable_span().slice(corners);
      const VectorSet<int> star_edges(vert_to_edge_map[vert_index]);
      Array<int2> star_indices(corners.size(), int2(-1, 0));
      if (corners.size() != star_edges.size()) {
        continue;
      }
      for (const int i : star_pairs.index_range()) {
        const int index = star_edges.index_of(star_pairs[i][0]);
        if (star_indices[index][0] == -1) {
          star_indices[index][0] = i;
        }
        else {
          star_indices[index][1] = i;
        }
      }
      for (const int i : star_pairs.index_range()) {
        const int index = star_edges.index_of(star_pairs[i][1]);
        if (star_indices[index][0] == -1) {
          star_indices[index][0] = i;
        }
        else {
          star_indices[index][1] = i;
        }
      }

      Array<int2> star_connections(corners.size());
      for (const int i : star_pairs.index_range()) {
        const int index = star_edges.index_of(star_pairs[i][0]);
        const int2 connection = star_indices[index];
        star_connections[i][0] = connection[0] == i ? connection[1] : connection[0];
      }
      for (const int i : star_pairs.index_range()) {
        const int index = star_edges.index_of(star_pairs[i][1]);
        const int2 connection = star_indices[index];
        star_connections[i][1] = connection[0] == i ? connection[1] : connection[0];
      }

      Array<int> revers_indices(corners.size(), -1);
      revers_indices.first() = 0;
      int index = 1;
      for (int i = 0, next_i = star_connections.first()[0]; next_i != 0;) {
        int2 step = star_connections[next_i];
        const bool flipped_node = step[1] == i;
        BLI_assert(flipped_node != (step[0] == i));
        if (flipped_node) {
          std::swap(step[0], step[1]);
        }
        i = next_i;
        next_i = step[1];
        revers_indices[index] = i;
        index++;
      }

      BLI_assert(!revers_indices.as_span().contains(-1));

      Array<int2> buffer(corners.size());
      array_utils::copy(star_pairs.as_span(), buffer.as_mutable_span());
      array_utils::gather(buffer.as_span(), revers_indices.as_span(), star_pairs);
      if (ELEM(star_pairs.first()[1], star_pairs.last()[0], star_pairs.last()[1])) {
        int2 &start = star_pairs.first();
        std::swap(start[0], start[1]);
      }
      for (const int index : star_pairs.index_range().drop_front(1)) {
        int2 &current = star_pairs[index];
        const int2 previos = star_pairs[index - 1];
        if (ELEM(current[1], previos[0], previos[1])) {
          std::swap(current[0], current[1]);
        }
      }
    }
    for (const int vert_index : IndexRange(mesh.totvert)) {
      const IndexRange corners = vert_to_loop_map.offsets[vert_index];
      const Span<int2> star_pairs = vert_to_loop_edge_pairs.as_span().slice(corners);

      if (corners.size() % 2) {
        continue;
      }

      const int semicircle_size = corners.size() / 2;
      const Span<int2> north_corners = star_pairs.take_front(semicircle_size);
      const Span<int2> south_corners = star_pairs.take_back(semicircle_size);
      for (const int index : IndexRange(semicircle_size)) {
        const int north_edge = north_corners[index][0];
        const int south_edge = south_corners[index][0];
        parallel_edges.join(north_edge, south_edge);
      }
    }

    const IndexMask edge_mask = IndexMask(mesh.totedge);
    const IndexMask &mask_to_result = domain == ATTR_DOMAIN_EDGE ? mask : edge_mask;
    Array<int> edge_group(mesh.totedge);
    VectorSet<int> ordered_rings_roots;
    mask_to_result.foreach_index_optimized<int>([&](const int edge_index) {
      const int root = parallel_edges.find_root(edge_index);
      edge_group[edge_index] = ordered_rings_roots.index_of_or_add(root);
    });

    return mesh.attributes().adapt_domain<int>(
        VArray<int>::ForContainer(std::move(edge_group)), ATTR_DOMAIN_EDGE, domain);
  }

  uint64_t hash() const final
  {
    return 736758993181174;
  }

  bool is_equal_to(const fn::FieldNode &other) const final
  {
    return dynamic_cast<const EdgesLineGroupFieldInput *>(&other) != nullptr;
  }

  std::optional<eAttrDomain> preferred_domain(const Mesh & /*mesh*/) const final
  {
    return ATTR_DOMAIN_EDGE;
  }
};

class ParallelEdgeGroupFieldInput final : public bke::MeshFieldInput {
 public:
  ParallelEdgeGroupFieldInput()
      : bke::MeshFieldInput(CPPType::get<int>(), "Parallel Edge Rings Field")
  {
  }

  GVArray get_varray_for_context(const Mesh &mesh,
                                 const eAttrDomain domain,
                                 const IndexMask &mask) const final
  {
    DisjointSet<int> parallel_edges(mesh.totedge);

    const Span<int> corner_edges = mesh.corner_edges();
    const blender::OffsetIndices<int> faces = mesh.faces();
    for (const int poly_index : IndexRange(mesh.faces_num)) {
      const IndexRange poly_range = faces[poly_index];
      if (poly_range.size() % 2) {
        continue;
      }
      /* Split corners of polygon to semicircle segments. */
      const int semicircle_size = poly_range.size() / 2;
      const Span<int> north_corners = corner_edges.slice(poly_range).take_front(semicircle_size);
      const Span<int> south_corners = corner_edges.slice(poly_range).take_back(semicircle_size);
      for (const int index : IndexRange(semicircle_size)) {
        const int north_edge = north_corners[index];
        const int south_edge = south_corners[index];
        parallel_edges.join(north_edge, south_edge);
      }
    }

    const IndexMask edge_mask = IndexMask(mesh.totedge);
    const IndexMask &mask_to_result = domain == ATTR_DOMAIN_EDGE ? mask : edge_mask;
    Array<int> edge_group(mesh.totedge);
    VectorSet<int> ordered_rings_roots;
    mask_to_result.foreach_index_optimized<int>([&](const int edge_index) {
      const int root = parallel_edges.find_root(edge_index);
      edge_group[edge_index] = ordered_rings_roots.index_of_or_add(root);
    });

    return mesh.attributes().adapt_domain<int>(
        VArray<int>::ForContainer(std::move(edge_group)), ATTR_DOMAIN_EDGE, domain);
  }

  uint64_t hash() const final
  {
    return 736758776181174;
  }

  bool is_equal_to(const fn::FieldNode &other) const final
  {
    return dynamic_cast<const ParallelEdgeGroupFieldInput *>(&other) != nullptr;
  }

  std::optional<eAttrDomain> preferred_domain(const Mesh & /*mesh*/) const final
  {
    return ATTR_DOMAIN_EDGE;
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  Field<int> parallel_edges{std::make_shared<ParallelEdgeGroupFieldInput>()};
  params.set_output("Parallel Group Index", std::move(parallel_edges));
  Field<int> linear_edges{std::make_shared<EdgesLineGroupFieldInput>()};
  params.set_output("Linear Group Index", std::move(linear_edges));
}

static void node_register()
{
  static bNodeType ntype;
  geo_node_type_base(&ntype, GEO_NODE_INPUT_MESH_EDGE_RINGS, "Edge Rings", NODE_CLASS_INPUT);
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_input_mesh_edge_rings_cc
