/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_matrix.hh"

#include "node_function_util.hh"

namespace blender::nodes::node_fn_combine_matrix_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.use_custom_socket_order();

  b.add_output<decl::Matrix>("Matrix");

  PanelDeclarationBuilder &column_a = b.add_panel("Column A").default_closed(true);
  column_a.add_input<decl::Float>("Column A Row A").default_value(1.0f);
  column_a.add_input<decl::Float>("Column A Row B");
  column_a.add_input<decl::Float>("Column A Row C");
  column_a.add_input<decl::Float>("Column A Row D");

  PanelDeclarationBuilder &column_b = b.add_panel("Column B").default_closed(true);
  column_b.add_input<decl::Float>("Column B Row A");
  column_b.add_input<decl::Float>("Column B Row B").default_value(1.0f);
  column_b.add_input<decl::Float>("Column B Row C");
  column_b.add_input<decl::Float>("Column B Row D");

  PanelDeclarationBuilder &column_c = b.add_panel("Column C").default_closed(true);
  column_c.add_input<decl::Float>("Column C Row A");
  column_c.add_input<decl::Float>("Column C Row B");
  column_c.add_input<decl::Float>("Column C Row C").default_value(1.0f);
  column_c.add_input<decl::Float>("Column C Row D");

  PanelDeclarationBuilder &column_d = b.add_panel("Column D").default_closed(true);
  column_d.add_input<decl::Float>("Column D Row A");
  column_d.add_input<decl::Float>("Column D Row B");
  column_d.add_input<decl::Float>("Column D Row C");
  column_d.add_input<decl::Float>("Column D Row D").default_value(1.0f);
}

static void step_copy(const IndexMask &mask,
                      const VArray<float> &src,
                      const int64_t src_step,
                      const int64_t src_begin,
                      const int64_t dst_step,
                      const int64_t dst_begin,
                      MutableSpan<float> dst)
{
  BLI_assert(src_begin < src_step);
  BLI_assert(dst_begin < dst_step);
  BLI_assert(src.size() / src_step == dst.size() / dst_step);
  devirtualize_varray(src, [&](const auto src) {
    mask.foreach_index_optimized<int>([&](const int64_t index) {
      dst[dst_begin + dst_step * index] = src[src_begin + src_step * index];
    });
  });
}

class CombineMatrixFunction : public mf::MultiFunction {
 public:
  CombineMatrixFunction()
  {
    static const mf::Signature signature = []() {
      mf::Signature signature;
      mf::SignatureBuilder builder{"Combine Matrix", signature};
      builder.single_input<float>("Column A Row A");
      builder.single_input<float>("Column A Row B");
      builder.single_input<float>("Column A Row C");
      builder.single_input<float>("Column A Row D");

      builder.single_input<float>("Column B Row A");
      builder.single_input<float>("Column B Row B");
      builder.single_input<float>("Column B Row C");
      builder.single_input<float>("Column B Row D");

      builder.single_input<float>("Column C Row A");
      builder.single_input<float>("Column C Row B");
      builder.single_input<float>("Column C Row C");
      builder.single_input<float>("Column C Row D");

      builder.single_input<float>("Column D Row A");
      builder.single_input<float>("Column D Row B");
      builder.single_input<float>("Column D Row C");
      builder.single_input<float>("Column D Row D");

      builder.single_output<float4x4>("Matrix");
      return signature;
    }();
    this->set_signature(&signature);
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    const VArray<float> &column_a_row_a = params.readonly_single_input<float>(0, "Column A Row A");
    const VArray<float> &column_a_row_b = params.readonly_single_input<float>(1, "Column A Row B");
    const VArray<float> &column_a_row_c = params.readonly_single_input<float>(2, "Column A Row C");
    const VArray<float> &column_a_row_d = params.readonly_single_input<float>(3, "Column A Row D");

    const VArray<float> &column_b_row_a = params.readonly_single_input<float>(4, "Column B Row A");
    const VArray<float> &column_b_row_b = params.readonly_single_input<float>(5, "Column B Row B");
    const VArray<float> &column_b_row_c = params.readonly_single_input<float>(6, "Column B Row C");
    const VArray<float> &column_b_row_d = params.readonly_single_input<float>(7, "Column B Row D");

    const VArray<float> &column_c_row_a = params.readonly_single_input<float>(8, "Column C Row A");
    const VArray<float> &column_c_row_b = params.readonly_single_input<float>(9, "Column C Row B");
    const VArray<float> &column_c_row_c = params.readonly_single_input<float>(10,
                                                                              "Column C Row C");
    const VArray<float> &column_c_row_d = params.readonly_single_input<float>(11,
                                                                              "Column C Row D");

    const VArray<float> &column_d_row_a = params.readonly_single_input<float>(12,
                                                                              "Column D Row A");
    const VArray<float> &column_d_row_b = params.readonly_single_input<float>(13,
                                                                              "Column D Row B");
    const VArray<float> &column_d_row_c = params.readonly_single_input<float>(14,
                                                                              "Column D Row C");
    const VArray<float> &column_d_row_d = params.readonly_single_input<float>(15,
                                                                              "Column D Row D");

    MutableSpan<float4x4> matrices = params.uninitialized_single_output<float4x4>(16, "Matrix");
    MutableSpan<float> components = matrices.cast<float>();

    step_copy(mask, column_a_row_a, 1, 0, 16, 0, components);
    step_copy(mask, column_a_row_b, 1, 0, 16, 1, components);
    step_copy(mask, column_a_row_c, 1, 0, 16, 2, components);
    step_copy(mask, column_a_row_d, 1, 0, 16, 3, components);

    step_copy(mask, column_b_row_a, 1, 0, 16, 4, components);
    step_copy(mask, column_b_row_b, 1, 0, 16, 5, components);
    step_copy(mask, column_b_row_c, 1, 0, 16, 6, components);
    step_copy(mask, column_b_row_d, 1, 0, 16, 7, components);

    step_copy(mask, column_c_row_a, 1, 0, 16, 8, components);
    step_copy(mask, column_c_row_b, 1, 0, 16, 9, components);
    step_copy(mask, column_c_row_c, 1, 0, 16, 10, components);
    step_copy(mask, column_c_row_d, 1, 0, 16, 11, components);

    step_copy(mask, column_d_row_a, 1, 0, 16, 12, components);
    step_copy(mask, column_d_row_b, 1, 0, 16, 13, components);
    step_copy(mask, column_d_row_c, 1, 0, 16, 14, components);
    step_copy(mask, column_d_row_d, 1, 0, 16, 15, components);
  }
};

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  const static CombineMatrixFunction fn;
  builder.set_matching_fn(fn);
}

static void node_register()
{
  static bNodeType ntype;
  fn_node_type_base(&ntype, FN_NODE_COMBINE_MATRIX, "Combine Matrix", NODE_CLASS_CONVERTER);
  ntype.declare = node_declare;
  ntype.build_multi_function = node_build_multi_function;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_fn_combine_matrix_cc
