/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_function_util.hh"

#include "BLI_math_rotation.h"

namespace blender::nodes::node_fn_rotate_matrix_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.add_input<decl::Matrix>(N_("Matrix"));
  b.add_input<decl::Vector>(N_("Rotation"));
  b.add_output<decl::Matrix>(N_("Matrix"));
};

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  static auto fn = mf::build::SI2_SO<float4x4, float3, float4x4>(
      "rotate_matrix", [](const float4x4 &mat, const float3 &rot) {
        float4x4 rot_mat;
        eulO_to_mat4(rot_mat.ptr(), rot, EULER_ORDER_DEFAULT);
        return rot_mat * mat;
      });
  builder.set_matching_fn(&fn);
}

}  // namespace blender::nodes::node_fn_rotate_matrix_cc

void register_node_type_fn_rotate_matrix(void)
{
  namespace file_ns = blender::nodes::node_fn_rotate_matrix_cc;

  static bNodeType ntype;

  fn_node_type_base(&ntype, FN_NODE_ROTATE_MATRIX, "Rotate Matrix", NODE_CLASS_CONVERTER);
  ntype.declare = file_ns::node_declare;
  ntype.build_multi_function = file_ns::node_build_multi_function;

  nodeRegisterType(&ntype);
}
