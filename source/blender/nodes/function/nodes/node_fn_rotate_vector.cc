/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_quaternion.hh"

#include "node_function_util.hh"

namespace blender::nodes::node_fn_rotate_vector_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.add_input<decl::Rotation>("Rotation");
  b.add_input<decl::Vector>("Vector");
  b.add_output<decl::Vector>("Vector");
};

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  static auto fn = mf::build::SI2_SO<math::Quaternion, float3, float3>(
      "Rotate Vector",
      [](math::Quaternion quat, float3 vector) { return math::transform_point(quat, vector); });
  builder.set_matching_fn(fn);
}

}  // namespace blender::nodes::node_fn_rotate_vector_cc

void register_node_type_fn_rotate_vector()
{
  namespace file_ns = blender::nodes::node_fn_rotate_vector_cc;
  static bNodeType ntype;
  fn_node_type_base(&ntype, FN_NODE_ROTATE_VECTOR, "Rotation Vector", NODE_CLASS_CONVERTER);
  ntype.declare = file_ns::node_declare;
  ntype.build_multi_function = file_ns::node_build_multi_function;
  nodeRegisterType(&ntype);
}
