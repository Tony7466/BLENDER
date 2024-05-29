/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_pointcloud_types.h"

#include "BKE_collision_shape.hh"
#include "BKE_physics_geometry.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_points_to_rigid_bodies_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Points").supported_type(GeometryComponent::Type::PointCloud);
  b.add_input<decl::Bool>("Selection").default_value(true).field_on({1}).hide_value();
  b.add_output<decl::Geometry>("Rigid Bodies").propagate_all();
}

static void geometry_set_points_to_rigid_bodies(
    GeometrySet &geometry_set,
    Field<bool> &selection_field,
    const AnonymousAttributePropagationInfo & /*propagation_info*/)
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

  const int num_bodies = selection.size();
  Array<bke::CollisionShape *> shapes_library(3);
  shapes_library[0] = new bke::BoxCollisionShape(float3(.3f, .5f, .7f));
  shapes_library[1] = new bke::BoxCollisionShape(float3(1, 1, .1f));
  shapes_library[2] = new bke::SphereCollisionShape(1.2f);

  auto *physics = new bke::PhysicsGeometry(num_bodies);
  VMutableArray<bke::CollisionShape *> shapes = physics->body_collision_shapes_for_write();
  VMutableArray<float> masses = physics->body_masses_for_write();
  VMutableArray<float3> inertias = physics->body_inertias_for_write();

  for (const int i : physics->rigid_bodies_range()) {
    shapes.set(i, shapes_library[i % 3]);
    masses.set(i, 2.34f);
    inertias.set(i, float3(0.0f));
  }

  geometry_set.replace_physics(physics);
  geometry_set.keep_only_during_modify({GeometryComponent::Type::Physics});
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Points");
  Field<bool> selection_field = params.extract_input<Field<bool>>("Selection");

  geometry_set.modify_geometry_sets([&](GeometrySet &geometry_set) {
    geometry_set_points_to_rigid_bodies(
        geometry_set, selection_field, params.get_output_propagation_info("Rigid Bodies"));
  });

  params.set_output("Rigid Bodies", std::move(geometry_set));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(
      &ntype, GEO_NODE_POINTS_TO_RIGID_BODIES, "Points to Rigid Bodies", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  blender::bke::nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_points_to_rigid_bodies_cc
