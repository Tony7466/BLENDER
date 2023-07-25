/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_quaternion.hh"

#include "node_function_util.hh"

namespace blender::nodes::node_fn_invert_rotation_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.add_input<decl::Rotation>("Rotation");
  b.add_output<decl::Rotation>("Rotation");
};

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  static auto fn = mf::build::SI1_SO<math::Quaternion, math::Quaternion>(
      "Invert Quaternion", [](math::Quaternion quat) { return math::invert(quat); });
  builder.set_matching_fn(fn);
}

}  // namespace blender::nodes::node_fn_invert_rotation_cc

void register_node_type_fn_invert_rotation()
{
  namespace file_ns = blender::nodes::node_fn_invert_rotation_cc;
  static bNodeType ntype;
  fn_node_type_base(&ntype, FN_NODE_INVERT_ROTATION, "Invert Rotation", NODE_CLASS_CONVERTER);
  ntype.declare = file_ns::node_declare;
  ntype.build_multi_function = file_ns::node_build_multi_function;
  nodeRegisterType(&ntype);
}
