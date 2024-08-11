/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute.hh"
#include "BKE_geometry_set.hh"
#include "BKE_node.hh"
#include "BKE_physics_geometry.hh"

#include "FN_field.hh"

#include "NOD_rna_define.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_set_body_activation_state_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Physics").supported_type(bke::GeometryComponent::Type::Physics);
  b.add_input<decl::Bool>("Selection").default_value(true).hide_value().field_on_all();
  b.add_output<decl::Geometry>("Physics").propagate_all();
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "activation_state", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  using BodyActivationState = bke::PhysicsGeometry::BodyActivationState;
  node->custom1 = int(BodyActivationState::Active);
}

static void node_geo_exec(GeoNodeExecParams params)
{
  using BodyActivationState = bke::PhysicsGeometry::BodyActivationState;

  GeometrySet geometry_set = params.extract_input<GeometrySet>("Physics");
  Field<bool> selection_field = params.extract_input<Field<bool>>("Selection");
  const BodyActivationState state = BodyActivationState(params.node().custom1);

  if (bke::PhysicsGeometry *physics = geometry_set.get_physics_for_write()) {
    bke::PhysicsFieldContext context(*physics, AttrDomain::Point);
    fn::FieldEvaluator evaluator(context, physics->bodies_num());
    evaluator.add(selection_field);
    evaluator.evaluate();
    IndexMask selection = evaluator.get_evaluated_as_mask(0);

    bke::AttributeWriter<int> activation_state_writer =
        physics->attributes_for_write().lookup_for_write<int>(
            bke::PhysicsGeometry::body_attribute_name(
                bke::PhysicsGeometry::BodyAttribute::activation_state));

    selection.foreach_index(
        [&](const int index) { activation_state_writer.varray.set(index, int(state)); });

    activation_state_writer.finish();
  }

  params.set_output("Physics", std::move(geometry_set));
}

static void node_rna(StructRNA *srna)
{
  using BodyActivationState = bke::PhysicsGeometry::BodyActivationState;

  static EnumPropertyItem activation_state_items[] = {
      {int(BodyActivationState::AlwaysActive),
       "ALWAYS_ACTIVE",
       0,
       "Always Active",
       "Body is always actively simulated"},
      {int(BodyActivationState::Active),
       "ACTIVE",
       0,
       "Active",
       "Body is active but can go to sleep below threshold velocity"},
      {int(BodyActivationState::WantsSleep),
       "WANTS_TO_SLEEP",
       0,
       "Wants to sleep",
       "Body will go to sleep when slowing down"},
      {int(BodyActivationState::Sleeping),
       "SLEEPING",
       0,
       "Sleeping",
       "Body is currently inactive and will wake up when impulse is applied"},
      {int(BodyActivationState::AlwaysSleeping),
       "ALWAYS_SLEEPING",
       0,
       "Always Sleeping",
       "Body is sleeping until state is changed exlicitly"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  RNA_def_node_enum(srna,
                    "activation_state",
                    "Activation State",
                    "",
                    activation_state_items,
                    NOD_inline_enum_accessors(custom1),
                    int(BodyActivationState::Active));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype,
                     GEO_NODE_SET_BODY_ACTIVATION_STATE,
                     "Set Body Activation State",
                     NODE_CLASS_GEOMETRY);
  ntype.geometry_node_execute = node_geo_exec;
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.draw_buttons = node_layout;
  blender::bke::nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_set_body_activation_state_cc
