/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_geometry_fields.hh"
#include "BKE_node.hh"
#include "BKE_physics_geometry.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_input_body_activation_state_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Bool>("Always Active").field_source();
  b.add_output<decl::Bool>("Active").field_source();
  b.add_output<decl::Bool>("Wants to Sleep").field_source();
  b.add_output<decl::Bool>("Sleeping").field_source();
  b.add_output<decl::Bool>("Always Sleeping").field_source();
}

template<int EnumValue> class AttributeEnumToBoolFieldInput : public bke::AttributeFieldInput {
 public:
  AttributeEnumToBoolFieldInput(std::string name)
      : bke::AttributeFieldInput(name, CPPType::get<int>())
  {
  }

  static fn::GField Create(std::string name)
  {
    auto field_input = std::make_shared<AttributeEnumToBoolFieldInput>(std::move(name));
    return fn::Field<int>(field_input);
  }

  GVArray get_varray_for_context(const bke::GeometryFieldContext &context,
                                 const IndexMask &mask) const override
  {
    VArray<int> varray =
        bke::AttributeFieldInput::get_varray_for_context(context, mask).template typed<int>();
    /* TODO could use a specialized impl here to optimize for structured access. */
    return VArray<bool>::ForFunc(varray.size(),
                                 [varray](const int index) { return varray[index] == EnumValue; });
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  using BodyActivationState = bke::PhysicsGeometry::BodyActivationState;

  const StringRef attribute = bke::PhysicsGeometry::body_attribute_name(
      bke::PhysicsGeometry::BodyAttribute::activation_state);
  Field<bool> always_active_field =
      AttributeEnumToBoolFieldInput<int(BodyActivationState::AlwaysActive)>::Create(attribute);
  Field<bool> active_field =
      AttributeEnumToBoolFieldInput<int(BodyActivationState::Active)>::Create(attribute);
  Field<bool> wants_to_sleep_field =
      AttributeEnumToBoolFieldInput<int(BodyActivationState::WantsSleep)>::Create(attribute);
  Field<bool> sleeping_field =
      AttributeEnumToBoolFieldInput<int(BodyActivationState::Sleeping)>::Create(attribute);
  Field<bool> always_sleeping_field =
      AttributeEnumToBoolFieldInput<int(BodyActivationState::AlwaysSleeping)>::Create(attribute);

  params.set_output("Always Active", std::move(always_active_field));
  params.set_output("Active", std::move(active_field));
  params.set_output("Wants to Sleep", std::move(wants_to_sleep_field));
  params.set_output("Sleeping", std::move(sleeping_field));
  params.set_output("Always Sleeping", std::move(always_sleeping_field));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(
      &ntype, GEO_NODE_INPUT_BODY_ACTIVATION_STATE, "Body Activation State", NODE_CLASS_INPUT);
  ntype.geometry_node_execute = node_geo_exec;
  ntype.declare = node_declare;
  blender::bke::node_register_type(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_input_body_activation_state_cc
