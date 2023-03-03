/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_function_util.hh"

#include "BLI_math_vector.h"

namespace blender::nodes::node_fn_scale_matrix_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.add_input<decl::Matrix4x4>(N_("Matrix"));
  b.add_input<decl::Vector>(N_("Scale"));
  b.add_output<decl::Matrix4x4>(N_("Matrix"));
};

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  static auto fn = mf::build::SI2_SO<float4x4, float3, float4x4>(
      "scale_matrix", [](const float4x4 &mat, const float3 &scale) {
        float4x4 result;
        mul_v3_v3fl(result.view()[0], mat[0], scale[0]);
        mul_v3_v3fl(result.view()[1], mat[1], scale[1]);
        mul_v3_v3fl(result.view()[2], mat[2], scale[2]);
        return result;
      });
  builder.set_matching_fn(&fn);
}

}  // namespace blender::nodes::node_fn_scale_matrix_cc

void register_node_type_fn_scale_matrix(void)
{
  namespace file_ns = blender::nodes::node_fn_scale_matrix_cc;

  static bNodeType ntype;

  fn_node_type_base(&ntype, FN_NODE_SCALE_MATRIX, "Scale Matrix", NODE_CLASS_CONVERTER);
  ntype.declare = file_ns::node_declare;
  ntype.build_multi_function = file_ns::node_build_multi_function;

  nodeRegisterType(&ntype);
}
