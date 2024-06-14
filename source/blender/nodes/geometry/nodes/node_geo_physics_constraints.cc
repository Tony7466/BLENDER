/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute.hh"
#include "BKE_node.hh"
#include "BKE_physics_geometry.hh"

#include "FN_field.hh"

#include "NOD_rna_define.hh"

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
  int count = params.extract_input<int>("Count");
  Field<int> body1_field = params.extract_input<Field<int>>("Body 1");
  Field<int> body2_field = params.extract_input<Field<int>>("Body 2");

  if (bke::PhysicsGeometry *physics = geometry_set.get_physics_for_write()) {
    physics->resize(physics->bodies_num(), physics->constraints_num() + std::max(count, 0));
    const IndexMask new_constraints = physics->constraints_range().take_back(count);

    const bke::PhysicsFieldContext field_context{*physics, bke::AttrDomain::Edge};
    fn::FieldEvaluator field_evaluator{field_context, &new_constraints};
    field_evaluator.add(body1_field);
    field_evaluator.add(body2_field);
    field_evaluator.evaluate();

    const VArray<int> src_type = VArray<int>::ForSingle(int(constraint_type),
                                                        new_constraints.min_array_size());
    const VArray<int> src_body1 = field_evaluator.get_evaluated<int>(0);
    const VArray<int> src_body2 = field_evaluator.get_evaluated<int>(1);

    physics->create_constraints(new_constraints, src_type, src_body1, src_body2);
  }

  params.set_output("Physics", std::move(geometry_set));
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
