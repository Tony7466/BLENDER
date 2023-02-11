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
                                 const IndexMask mask) const final
  {
    const Span<MLoop> loops = mesh.loops();
    const Span<MPoly> polys = mesh.polys();
    DisjointSet<int> parallels(mesh.totedge);

    for (const MPoly poly : polys) {
      if (!(poly.totloop % 2)) {
        const int half = poly.totloop / 2;
        const Span<MLoop> face_loops = loops.slice(poly.loopstart, poly.totloop);
        for (const int i : IndexRange(half)) {
          const MLoop parallel_loop_a = face_loops[i];
          const MLoop parallel_loop_b = face_loops[i + half];
          parallels.join(parallel_loop_a.e, parallel_loop_b.e);
        }
      }
    }

    const IndexMask edge_mask = domain == ATTR_DOMAIN_EDGE ? mask : IndexMask(mesh.totedge);
    Array<int> edge_group(mesh.totedge);
    VectorSet<int> ordered_roots;
    for (const int i : edge_mask) {
      const int root = parallels.find_root(i);
      edge_group[i] = ordered_roots.index_of_or_add(root);
    }

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
