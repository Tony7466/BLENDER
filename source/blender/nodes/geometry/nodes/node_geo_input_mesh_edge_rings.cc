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

// Edges in planar face grid. Vertex can belong to multiple surfaces.
// Bound edges. Vertex can belong to multiple faces with it own bound edges or to planar face
// grids. loos edges. Loos edges can be connected to edges in planar face grid.

static void make_ordered(const int2 prev, int2 &current)
{
  if (ELEM(current[1], prev[0], prev[1])) {
    std::swap(current[0], current[1]);
  }
}

template<typename T, typename Func> static T &find_if(MutableSpan<T> span, const Func &&func)
{
  return *std::find_if(span.begin(), span.end(), func);
}

template<typename T, typename Func> static const T &find_if(const Span<T> span, const Func &&func)
{
  return *std::find_if(span.begin(), span.end(), func);
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
    const Span<int2> edges = mesh.edges();
    const Span<int> corner_edges = mesh.corner_edges();
    const Span<int> corner_verts = mesh.corner_verts();

    Array<int> vert_to_edge_offsets;
    Array<int> vert_to_edge_indices;
    const GroupedSpan<int> vert_to_edge_map = bke::mesh::build_vert_to_edge_map(
        edges, mesh.totvert, vert_to_edge_offsets, vert_to_edge_indices);

    Array<int> edge_faces_total(mesh.totedge, 0);
    array_utils::count_indices(corner_edges, edge_faces_total.as_mutable_span());

    IndexMaskMemory memory;
    const IndexMask vert_mask = IndexMask::from_predicate(
        IndexMask(mesh.totvert), GrainSize(1024), memory, [&](const int vert_index) {
          for (const int edge_index : vert_to_edge_map[vert_index]) {
            if (!ELEM(edge_faces_total[edge_index], 0, 1, 2)) {
              return false;
            }
          }
          return true;
        });

    Array<int2> corner_edge_pairs(mesh.totloop);
    const OffsetIndices<int> faces = mesh.faces();
    for (const int face_index : IndexRange(mesh.faces_num)) {
      const IndexRange face_corners = faces[face_index];
      const Span<int> face_corner_edges = corner_edges.slice(face_corners);
      const Span<int> face_corner_verts = corner_verts.slice(face_corners);
      for (const int corner_i : face_corners.index_range()) {
        const int corner_index = face_corners[corner_i];
        corner_edge_pairs[corner_index][0] = face_corner_edges[corner_i];
        corner_edge_pairs[corner_index][1] = bke::mesh::other_corner_edge(
            edges, face_corner_edges, face_corner_verts, corner_i);
      }
    }

    Array<int> vert_to_loop_offsets;
    Array<int> vert_to_loop_indices;
    const GroupedSpan<int> vert_to_loop_map = bke::mesh::build_vert_to_loop_map(
        corner_verts, mesh.totvert, vert_to_loop_offsets, vert_to_loop_indices);

    Array<int2> stars(mesh.totloop);
    Array<int> star_index(mesh.totloop);
    array_utils::gather(
        corner_edge_pairs.as_span(), vert_to_loop_indices.as_span(), stars.as_mutable_span());
    vert_mask.foreach_index([&](const int vert_index) {
      const IndexRange range = vert_to_loop_map.offsets[vert_index];
      if (range.is_empty()) {
        return;
      }
      MutableSpan<int2> vertex_stars = stars.as_mutable_span().slice(range);
      MutableSpan<int> vertex_star_index = star_index.as_mutable_span().slice(range);
      vertex_star_index.first() = 0;
      for (const int index : vertex_stars.index_range().drop_back(1)) {
        int2 &current = vertex_stars[index];
        MutableSpan<int2> next_range = vertex_stars.drop_front(index + 1);
        vertex_star_index[index + 1] = vertex_star_index[index];
        int2 &next = find_if(next_range,
                             [&](const int2 next) { return ELEM(current[1], next[0], next[1]); });
        if (&next != next_range.end()) {
          make_ordered(current, next);
          std::swap(next_range.first(), next);
          continue;
        }
        /* This is the first element in a non-cyclic star and looking for unconnected edge was
         * failure. Try again. */
        int2 &other_next = find_if(
            next_range, [&](const int2 next) { return ELEM(current[0], next[0], next[1]); });
        if (&other_next != next_range.end()) {
          std::swap(current[0], current[1]);
          make_ordered(current, other_next);
          std::swap(next_range.first(), other_next);
          continue;
        }
        vertex_star_index[index + 1]++;
      }
    });

    DisjointSet<int> parallel_edges(mesh.totedge);

    vert_mask.foreach_index([&](const int vert_index) {
      for (IndexRange range = vert_to_loop_map.offsets[vert_index]; !range.is_empty();) {
        const Span<int> vertex_star_index = star_index.as_span().slice(range);
        const int &first = vertex_star_index.first();
        const int &end = find_if(vertex_star_index,
                                 [&](const int other) { return other != first; });
        const int size = int(&end - &first);
        const Span<int2> current_edge_star = stars.as_span().slice(range.take_front(size));
        range = range.drop_front(size);

        const bool is_cyclic = current_edge_star.first()[0] == current_edge_star.last()[1];
        if (!is_cyclic) {
          const int north_edge = current_edge_star.first()[0];
          const int south_edge = current_edge_star.last()[1];
          parallel_edges.join(north_edge, south_edge);
          continue;
        }
        if (size % 2) {
          continue;
        }
        const int semicircle_size = size / 2;
        const Span<int2> north_corners = current_edge_star.take_front(semicircle_size);
        const Span<int2> south_corners = current_edge_star.take_back(semicircle_size);
        for (const int index : IndexRange(semicircle_size)) {
          const int north_edge = north_corners[index][0];
          const int south_edge = south_corners[index][0];
          parallel_edges.join(north_edge, south_edge);
        }
      }
    });

    for (const int vert_index : IndexRange(mesh.totvert)) {
      if (vert_to_edge_map[vert_index].size() != 2) {
        continue;
      }
      if (edge_faces_total[vert_to_edge_map[vert_index][0]] != 0) {
        continue;
      }
      if (edge_faces_total[vert_to_edge_map[vert_index][1]] != 0) {
        continue;
      }
      parallel_edges.join(vert_to_edge_map[vert_index][0], vert_to_edge_map[vert_index][1]);
    }

    vert_mask.foreach_index([&](const int vert_index) {
      if (vert_to_edge_map[vert_index].size() != 4) {
        return;
      }
      const IndexRange range = vert_to_loop_map.offsets[vert_index];
      if (range.size() != 2) {
        return;
      }
      const Span<int> vertex_star_index = star_index.as_span().slice(range);
      const int &first = vertex_star_index.first();
      const int &end = find_if(vertex_star_index, [&](const int other) { return other != first; });
      const int size = int(&end - &first);
      if (size != range.size()) {
        return;
      }
      const int central_edge = stars.as_span().slice(range).first()[1];
      const int loos_edge = [&]() {
        for (const int edge_index : vert_to_edge_map[vert_index]) {
          if (edge_faces_total[edge_index] == 0) {
            return edge_index;
          }
        }
        BLI_assert_unreachable();
      }();
      parallel_edges.join(central_edge, loos_edge);
    });

    const IndexMask edge_mask = domain == ATTR_DOMAIN_EDGE ? mask : IndexMask(mesh.totedge);
    Array<int> edge_group(mesh.totedge);
    VectorSet<int> ordered_rings_roots;
    edge_mask.foreach_index_optimized<int>([&](const int edge_index) {
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

    const IndexMask edge_mask = domain == ATTR_DOMAIN_EDGE ? mask : IndexMask(mesh.totedge);
    Array<int> edge_group(mesh.totedge);
    VectorSet<int> ordered_rings_roots;
    edge_mask.foreach_index_optimized<int>([&](const int edge_index) {
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
