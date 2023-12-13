/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_matrix.hh"

#include "node_function_util.hh"

namespace blender::nodes::node_fn_scale_to_transform_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.add_input<decl::Vector>("Scale").subtype(PROP_XYZ);
  b.add_output<decl::Matrix>("Transform");
};

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  static auto fn = mf::build::SI1_SO<float3, float4x4>(
      "Scale to Transform", [](float3 scale) { return math::from_scale<float4x4>(scale); });
  builder.set_matching_fn(fn);
}

static void node_register()
{
  static bNodeType ntype;
  fn_node_type_base(
      &ntype, FN_NODE_SCALE_TO_TRANSFORM, "Scale to Transform", NODE_CLASS_CONVERTER);
  ntype.declare = node_declare;
  ntype.build_multi_function = node_build_multi_function;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_fn_scale_to_transform_cc
