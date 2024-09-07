/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_array_utils.hh"

#include "DNA_pointcloud_types.h"

#include "BKE_customdata.hh"
#include "BKE_mesh.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_points_to_vertices_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Points").supported_type(GeometryComponent::Type::PointCloud);
  b.add_input<decl::Bool>("Selection").default_value(true).field_on_all().hide_value();
  b.add_output<decl::Geometry>("Mesh").propagate_all();
}

static void geometry_set_points_to_vertices(GeometrySet &geometry_set,
                                            Field<bool> &selection_field,
                                            const AttributeFilter &attribute_filter)
{
  const PointCloud *points = geometry_set.get_pointcloud();
  if (points == nullptr) {
    geometry_set.remove_geometry_during_modify();
    return;
  }
  if (points->totpoint == 0) {
    geometry_set.remove_geometry_during_modify();
    return;
  }

  const bke::PointCloudFieldContext field_context{*points};
  fn::FieldEvaluator selection_evaluator{field_context, points->totpoint};
  selection_evaluator.add(selection_field);
  selection_evaluator.evaluate();

  const IndexMask selection = selection_evaluator.get_evaluated_as_mask(0);
  Mesh *mesh = bke::mesh_new_no_attributes(selection.size(), 0, 0, 0);
  bke::gather_attributes(points->attributes(),
                         bke::AttrDomain::Point,
                         bke::AttrDomain::Point,
                         attribute_filter,
                         selection,
                         mesh->attributes_for_write());

  mesh->tag_loose_edges_none();
  mesh->tag_overlapping_none();

  geometry_set.replace_mesh(mesh);
  geometry_set.keep_only_during_modify({GeometryComponent::Type::Mesh});
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Points");
  Field<bool> selection_field = params.extract_input<Field<bool>>("Selection");

  geometry_set.modify_geometry_sets([&](GeometrySet &geometry_set) {
    geometry_set_points_to_vertices(
        geometry_set, selection_field, params.get_attribute_filter("Mesh"));
  });

  params.set_output("Mesh", std::move(geometry_set));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(
      &ntype, GEO_NODE_POINTS_TO_VERTICES, "Points to Vertices", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  blender::bke::node_register_type(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_points_to_vertices_cc
