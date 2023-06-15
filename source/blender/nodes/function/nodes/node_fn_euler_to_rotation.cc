/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_euler.hh"

#include "node_function_util.hh"

namespace blender::nodes::node_fn_euler_to_rotation_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.add_input<decl::Vector>("Forward").default_value({0.0f, 1.0f, 0.0f});
  b.add_input<decl::Vector>("Up").default_value({0.0f, 0.0f, 1.0f});
  b.add_output<decl::Rotation>("Rotation");
};

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  static auto fn = mf::build::SI1_SO<float3, math::Quaternion>(
      "Euler XYZ to Quaternion",
      [](float3 euler) { return math::to_quaternion(math::EulerXYZ(euler)); });
  builder.set_matching_fn(fn);
}

}  // namespace blender::nodes::node_fn_euler_to_rotation_cc

void register_node_type_fn_euler_to_rotation()
{
  namespace file_ns = blender::nodes::node_fn_euler_to_rotation_cc;
  static bNodeType ntype;
  fn_node_type_base(&ntype, FN_NODE_AXES_TO_ROTATION, "Axes to Rotation", NODE_CLASS_CONVERTER);
  ntype.declare = file_ns::node_declare;
  ntype.build_multi_function = file_ns::node_build_multi_function;
  nodeRegisterType(&ntype);
}
