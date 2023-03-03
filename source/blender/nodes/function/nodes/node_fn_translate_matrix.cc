/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_function_util.hh"

namespace blender::nodes::node_fn_translate_matrix_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.add_input<decl::Matrix>(N_("Matrix"));
  b.add_input<decl::Vector>(N_("Translation"));
  b.add_output<decl::Matrix>(N_("Matrix"));
};

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  static auto fn = mf::build::SI2_SO<float4x4, float3, float4x4>(
      "translate_matrix", [](const float4x4 &mat, const float3 &translation) {
        float4x4 result;
        result.view()[0] = mat.view()[0];
        result.view()[1] = mat.view()[1];
        result.view()[2] = mat.view()[2];
        result.view()[3] = mat.view()[3] + float4(translation, 0.0f);
        return result;
      });
  builder.set_matching_fn(&fn);
}

}  // namespace blender::nodes::node_fn_translate_matrix_cc

void register_node_type_fn_translate_matrix(void)
{
  namespace file_ns = blender::nodes::node_fn_translate_matrix_cc;

  static bNodeType ntype;

  fn_node_type_base(&ntype, FN_NODE_TRANSLATE_MATRIX, "Translate Matrix", NODE_CLASS_CONVERTER);
  ntype.declare = file_ns::node_declare;
  ntype.build_multi_function = file_ns::node_build_multi_function;

  nodeRegisterType(&ntype);
}
