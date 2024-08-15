/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute.hh"
#include "BKE_node.hh"
#include "BKE_physics_geometry.hh"

#include "FN_field.hh"

#include "GEO_join_geometries.hh"

#include "NOD_rna_define.hh"

#include "NOD_socket_declarations.hh"
#include "NOD_socket_declarations_geometry.hh"
#include "UI_interface.hh"
#include "UI_resources.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_physics_constraints_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Physics").supported_type(bke::GeometryComponent::Type::Physics);
  b.add_input<decl::Int>("Count").default_value(1).min(0).description(
      "The number of points to create");
  b.add_input<decl::Int>("Body 1")
      .default_value(-1)
      .min(-1)
      .supports_field()
      .description("Index of the first constrained body")
      .hide_value();
  b.add_input<decl::Int>("Body 2")
      .default_value(-1)
      .min(-1)
      .supports_field()
      .description("Index of the second constrained body")
      .hide_value();
  b.add_input<decl::Matrix>("Frame 1").supports_field().description(
      "Placement of the constraint relative to the first body");
  b.add_input<decl::Matrix>("Frame 2").supports_field().description(
      "Placement of the constraint relative to the second body");
  b.add_input<decl::Bool>("Disable Collision")
      .default_value(true)
      .supports_field()
      .description("Disable collisions between the constrained bodies");
  b.add_output<decl::Geometry>("Physics");
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "constraint_type", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  using ConstraintType = bke::PhysicsGeometry::ConstraintType;
  node->custom1 = int(ConstraintType::Fixed);
}

static void node_geo_exec(GeoNodeExecParams params)
{
  using ConstraintType = bke::PhysicsGeometry::ConstraintType;

  const ConstraintType constraint_type = ConstraintType(params.node().custom1);
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Physics");
  const int count = params.extract_input<int>("Count");
  const Field<int> body1_field = params.extract_input<Field<int>>("Body 1");
  const Field<int> body2_field = params.extract_input<Field<int>>("Body 2");
  const Field<float4x4> frame1_field = params.extract_input<Field<float4x4>>("Frame 1");
  const Field<float4x4> frame2_field = params.extract_input<Field<float4x4>>("Frame 2");
  const Field<bool> disable_collision_field = params.extract_input<Field<bool>>(
      "Disable Collision");

  bke::PhysicsGeometry *physics = new bke::PhysicsGeometry(0, count, 0);
  const IndexMask constraints = physics->constraints_range();

  const bke::PhysicsFieldContext field_context{*physics, bke::AttrDomain::Edge};
  fn::FieldEvaluator field_evaluator{field_context, &constraints};
  field_evaluator.add(body1_field);
  field_evaluator.add(body2_field);
  field_evaluator.add(frame1_field);
  field_evaluator.add(frame2_field);
  field_evaluator.add(disable_collision_field);
  field_evaluator.evaluate();

  const VArray<int> src_types = VArray<int>::ForSingle(int(constraint_type),
                                                       constraints.min_array_size());
  const VArray<int> src_body1 = field_evaluator.get_evaluated<int>(0);
  const VArray<int> src_body2 = field_evaluator.get_evaluated<int>(1);
  const VArray<float4x4> src_frame1 = field_evaluator.get_evaluated<float4x4>(2);
  const VArray<float4x4> src_frame2 = field_evaluator.get_evaluated<float4x4>(3);
  const VArray<bool> src_disable_collision = field_evaluator.get_evaluated<bool>(4);

  bke::AttributeWriter<int> dst_types = physics->constraint_types_for_write();
  bke::AttributeWriter<int> dst_body1 = physics->constraint_body1_for_write();
  bke::AttributeWriter<int> dst_body2 = physics->constraint_body2_for_write();
  bke::AttributeWriter<float4x4> dst_frame1 = physics->constraint_frame1_for_write();
  bke::AttributeWriter<float4x4> dst_frame2 = physics->constraint_frame2_for_write();
  bke::AttributeWriter<bool> dst_disable_collision =
      physics->constraint_disable_collision_for_write();
  constraints.foreach_index([&](const int index) {
    dst_types.varray.set(index, src_types[index]);
    dst_body1.varray.set(index, src_body1[index]);
    dst_body2.varray.set(index, src_body2[index]);
    dst_frame1.varray.set(index, src_frame1[index]);
    dst_frame2.varray.set(index, src_frame2[index]);
    dst_disable_collision.varray.set(index, src_disable_collision[index]);
  });
  dst_types.finish();
  dst_body1.finish();
  dst_body2.finish();
  dst_frame1.finish();
  dst_frame2.finish();
  dst_disable_collision.finish();

  GeometrySet output_geometry = geometry::join_geometries(
      {std::move(geometry_set), bke::GeometrySet::from_physics(physics)}, {});

  params.set_output("Physics", std::move(output_geometry));
}

static void node_rna(StructRNA *srna)
{
  using ConstraintType = bke::PhysicsGeometry::ConstraintType;

  static EnumPropertyItem constraint_type_items[] = {
      {int(ConstraintType::None), "NONE", 0, "None", ""},
      {int(ConstraintType::Fixed), "FIXED", 0, "Fixed", ""},
      {int(ConstraintType::Point), "POINT", 0, "Point", ""},
      {int(ConstraintType::Hinge), "HINGE", 0, "Hinge", ""},
      {int(ConstraintType::Slider), "SLIDER", 0, "Slider", ""},
      {int(ConstraintType::ConeTwist), "CONE_TWIST", 0, "Cone-Twist", ""},
      {int(ConstraintType::SixDoF), "SIX_DOF", 0, "6DoF", ""},
      {int(ConstraintType::SixDoFSpring), "SIX_DOF_SPRING", 0, "6DoF Spring", ""},
      {int(ConstraintType::SixDoFSpring2), "SIX_DOF_SPRING2", 0, "6DoF Spring 2", ""},
      {int(ConstraintType::Contact), "CONTACT", 0, "Contact", ""},
      {int(ConstraintType::Gear), "GEAR", 0, "Gear", ""},
      {0, nullptr, 0, nullptr, nullptr},
  };

  RNA_def_node_enum(srna,
                    "constraint_type",
                    "Constraint Type",
                    "",
                    constraint_type_items,
                    NOD_inline_enum_accessors(custom1),
                    int(ConstraintType::Fixed));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(
      &ntype, GEO_NODE_PHYSICS_CONSTRAINTS, "Physics Constraints", NODE_CLASS_GEOMETRY);
  ntype.geometry_node_execute = node_geo_exec;
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.draw_buttons = node_layout;
  blender::bke::nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_physics_constraints_cc
