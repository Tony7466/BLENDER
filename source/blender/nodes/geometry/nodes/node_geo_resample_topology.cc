/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_mesh_types.h"

#include "BLI_virtual_array.hh"

#include "GEO_mesh_resample_topology.hh"

#include "BKE_attribute.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_resample_topology_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Mesh").supported_type(GeometryComponent::Type::Mesh);
  b.add_input<decl::Int>("Count")
      .description("Number of points to resample edges")
      .hide_value()
      .field_on_all();
  b.add_input<decl::Bool>("Grid").default_value(true).field_on_all();

  b.add_output<decl::Geometry>(N_("Mesh")).propagate_all();
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Mesh");

  const Field<int> count_field = params.extract_input<Field<int>>("Count");
  Field<bool> grid_field = params.extract_input<Field<bool>>("Grid");

  Map<bke::AttributeIDRef, bke::AttributeKind> attributes;
  geometry_set.gather_attributes_for_propagation({GeometryComponent::Type::Mesh},
                                                 GeometryComponent::Type::Mesh,
                                                 false,
                                                 params.get_output_propagation_info("Mesh"),
                                                 attributes);

  geometry_set.modify_geometry_sets([&](GeometrySet &geometry_set) {
    if (!geometry_set.has_mesh()) {
      return;
    }
    const Mesh &mesh = *geometry_set.get_mesh();

    bke::MeshFieldContext edges_context{mesh, ATTR_DOMAIN_EDGE};
    fn::FieldEvaluator edges_evaluator{edges_context, mesh.totedge};
    edges_evaluator.add(count_field);
    edges_evaluator.evaluate();
    VArraySpan<int> count_field = edges_evaluator.get_evaluated<int>(0);

    bke::MeshFieldContext faces_context{mesh, ATTR_DOMAIN_FACE};
    fn::FieldEvaluator faces_evaluator{faces_context, mesh.faces_num};
    faces_evaluator.set_selection(grid_field);
    faces_evaluator.evaluate();
    const IndexMask face_selection = faces_evaluator.get_evaluated_selection_as_mask();

    Mesh *result = geometry::resample_topology(mesh, count_field, face_selection, attributes);
    geometry_set.replace_mesh(result);
  });

  params.set_output("Mesh", std::move(geometry_set));
}

void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_RESAMPLE_TOPOLOGY, "Resample Topology", NODE_CLASS_GEOMETRY);
  ntype.geometry_node_execute = node_geo_exec;
  ntype.declare = node_declare;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_resample_topology_cc
