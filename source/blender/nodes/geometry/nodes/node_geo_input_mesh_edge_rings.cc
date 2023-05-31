/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"

#include "BLI_disjoint_set.hh"
#include "BLI_vector_set.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_input_mesh_edge_rings_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Int>(N_("Group Index"))
      .field_source()
      .description(N_("Edge group index of the edge ring from parallel edges"));
}

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
    const blender::OffsetIndices<int> polys = mesh.polys();
    for (const int poly_index : IndexRange(mesh.totpoly)) {
      const IndexRange poly_range = polys[poly_index];
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
  params.set_output("Group Index", std::move(parallel_edges));
}

}  // namespace blender::nodes::node_geo_input_mesh_edge_rings_cc

void register_node_type_geo_input_mesh_edge_rings()
{
  namespace file_ns = blender::nodes::node_geo_input_mesh_edge_rings_cc;

  static bNodeType ntype;
  geo_node_type_base(&ntype, GEO_NODE_INPUT_MESH_EDGE_RINGS, "Edge Rings", NODE_CLASS_INPUT);
  ntype.declare = file_ns::node_declare;
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  nodeRegisterType(&ntype);
}
