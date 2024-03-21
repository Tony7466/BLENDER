/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "BKE_instances.hh"

#include "GEO_realize_instances.hh"

#include "UI_resources.hh"

namespace blender::nodes::node_geo_realize_instances_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Geometry");
  b.add_input<decl::Bool>("Selection")
      .default_value(true)
      .hide_value()
      .supports_field()
      .description("Which top-level instances to realize");
  b.add_input<decl::Bool>("Realize All")
      .default_value(true)
      .supports_field()
      .description("Determine wether to realize nested instances completly");
  b.add_input<decl::Int>("Depth").default_value(0).min(0).supports_field().description(
      "Number of levels of nested instances to realize for each top-level instance");
  b.add_output<decl::Geometry>("Geometry").propagate_all();
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Geometry");
  if (!geometry_set.has_instances()) {
    params.set_output("Geometry", std::move(geometry_set));
    return;
  }

  GeometryComponentEditData::remember_deformed_positions_if_necessary(geometry_set);
  Field<bool> selection_field = params.extract_input<Field<bool>>("Selection");
  Field<bool> realize_all_filed = params.extract_input<Field<bool>>("Realize All");
  Field<int> depth_field = params.extract_input<Field<int>>("Depth");

  static auto depth_override = mf::build::SI2_SO<int, bool, int>(
      "depth_override",
      [](int value, bool realize) { return realize ? -1 : std::max(value, 0); },
      mf::build::exec_presets::AllSpanOrSingle());

  static auto selection_override = mf::build::SI2_SO<int, bool, bool>(
      "selection_override",
      [](int value, bool selection) { return value == 0 ? false : selection; },
      mf::build::exec_presets::AllSpanOrSingle());

  Field<int> depth_field_overrided(
      FieldOperation::Create(depth_override, {depth_field, realize_all_filed}));
  Field<bool> selection_field_overrided(
      FieldOperation::Create(selection_override, {depth_field_overrided, selection_field}));

  const bke::Instances &instances = *geometry_set.get_instances();
  const bke::InstancesFieldContext field_context(instances);
  fn::FieldEvaluator evaluator(field_context, instances.instances_num());

  const int evaluated_depth_index = evaluator.add(depth_field_overrided);
  evaluator.set_selection(selection_field_overrided);
  evaluator.evaluate();

  geometry::VariedDepthOption varied_depth_option;
  varied_depth_option.depths = evaluator.get_evaluated<int>(evaluated_depth_index);
  varied_depth_option.selection = evaluator.get_evaluated_selection_as_mask();

  geometry::RealizeInstancesOptions options;
  options.keep_original_ids = false;
  options.realize_instance_attributes = true;
  options.propagation_info = params.get_output_propagation_info("Geometry");
  geometry_set = geometry::realize_instances(geometry_set, options, varied_depth_option);
  params.set_output("Geometry", std::move(geometry_set));
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_REALIZE_INSTANCES, "Realize Instances", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_realize_instances_cc
