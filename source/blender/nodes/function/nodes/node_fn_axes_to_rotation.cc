/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_matrix.hh"
#include "BLI_math_quaternion.hh"

#include "node_function_util.hh"

namespace blender::nodes::node_fn_axes_to_rotation_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.add_input<decl::Vector>("Forward").default_value({0.0f, 1.0f, 0.0f});
  b.add_input<decl::Vector>("Up").default_value({0.0f, 0.0f, 1.0f});
  b.add_output<decl::Rotation>("Rotation");
};

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  static auto fn = mf::build::SI2_SO<float3, float3, math::Quaternion>(
      "Axes to Quaternion", [](float3 forward, float3 up) {
        const float3x3 matrix = math::from_orthonormal_axes<float3x3>(math::normalize(forward),
                                                                      math::normalize(up));
        return math::to_quaternion(matrix);
      });
  builder.set_matching_fn(fn);
}

}  // namespace blender::nodes::node_fn_axes_to_rotation_cc

void register_node_type_fn_axes_to_rotation()
{
  namespace file_ns = blender::nodes::node_fn_axes_to_rotation_cc;
  static bNodeType ntype;
  fn_node_type_base(&ntype, FN_NODE_AXES_TO_ROTATION, "Axes to Rotation", NODE_CLASS_CONVERTER);
  ntype.declare = file_ns::node_declare;
  ntype.build_multi_function = file_ns::node_build_multi_function;
  nodeRegisterType(&ntype);
}
