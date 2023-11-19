/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "atomic_ops.h"

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"

#include "BKE_mesh.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_mesh_face_group_boundaries_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Int>("Face Group ID", "Face Set")
      .default_value(0)
      .hide_value()
      .supports_field()
      .description(
          "An identifier for the group of each face. All contiguous faces with the "
          "same value are in the same region");
  b.add_output<decl::Bool>("Boundary Edges")
      .field_source_reference_all()
      .description("The edges that lie on the boundaries between the different face groups");
}

class BoundaryFieldInput final : public bke::MeshFieldInput {
 private:
  const Field<int> face_set_;

 public:
  BoundaryFieldInput(const Field<int> face_set)
      : bke::MeshFieldInput(CPPType::get<bool>(), "Face Group Boundaries"), face_set_(face_set)
  {
    category_ = Category::Generated;
  }

  GVArray get_varray_for_context(const Mesh &mesh,
                                 const eAttrDomain domain,
                                 const IndexMask & /*mask*/) const final
  {
    const bke::MeshFieldContext face_context{mesh, ATTR_DOMAIN_FACE};
    FieldEvaluator face_evaluator{face_context, mesh.faces_num};
    face_evaluator.add(face_set_);
    face_evaluator.evaluate();
    const VArray<int> face_set = face_evaluator.get_evaluated<int>(0);
    if (face_set.is_single()) {
      return {};
    }

    const constexpr int is_first_face = -1;
    const constexpr int is_bad_group = -2;

    Array<bool> boundary(mesh.totedge, false);
    Array<int> previos_face(mesh.totedge, is_first_face);
    const GroupedSpan<int> face_edges(mesh.face_offsets(), mesh.corner_edges());
    threading::parallel_for(face_edges.index_range(), 2048, [&](const IndexRange range) {
      for (const int face_i : range) {
        const int group_id = face_set[face_i];
        for (const int edge_i : face_edges[face_i]) {
          while (true) {
            const int last_face_i = atomic_load_int32(&previos_face[edge_i]);
            if (last_face_i == is_bad_group) {
              break;
            }
            else if (last_face_i == is_first_face) {
              const int last_face_i_test = atomic_cas_int32(
                  &previos_face[edge_i], is_first_face, face_i);
              if (last_face_i_test == last_face_i) {
                break;
              }
            }
            else if (face_set[last_face_i] != group_id) {
              const int last_face_i_test = atomic_cas_int32(
                  &previos_face[edge_i], last_face_i, is_bad_group);
              if (last_face_i_test == last_face_i) {
                boundary[edge_i] = true;
                break;
              }
            }
            else {
              const int last_face_i_test = atomic_cas_int32(
                  &previos_face[edge_i], last_face_i, face_i);
              if (last_face_i_test == face_i) {
                break;
              }
            }
          }
        }
      }
    });
    return mesh.attributes().adapt_domain<bool>(
        VArray<bool>::ForContainer(std::move(boundary)), ATTR_DOMAIN_EDGE, domain);
  }

  void for_each_field_input_recursive(FunctionRef<void(const FieldInput &)> fn) const override
  {
    face_set_.node().for_each_field_input_recursive(fn);
  }

  std::optional<eAttrDomain> preferred_domain(const Mesh & /*mesh*/) const override
  {
    return ATTR_DOMAIN_EDGE;
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  const Field<int> face_set_field = params.extract_input<Field<int>>("Face Set");
  Field<bool> face_set_boundaries{std::make_shared<BoundaryFieldInput>(face_set_field)};
  params.set_output("Boundary Edges", std::move(face_set_boundaries));
}

static void node_register()
{
  static bNodeType ntype;
  geo_node_type_base(
      &ntype, GEO_NODE_MESH_FACE_GROUP_BOUNDARIES, "Face Group Boundaries", NODE_CLASS_INPUT);
  bke::node_type_size_preset(&ntype, bke::eNodeSizePreset::MIDDLE);
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_mesh_face_group_boundaries_cc
