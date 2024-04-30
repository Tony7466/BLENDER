/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_matrix.hh"

#include "node_function_util.hh"

namespace blender::nodes::node_fn_separate_matrix_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.use_custom_socket_order();

  b.add_input<decl::Matrix>("Matrix");

  PanelDeclarationBuilder &column_a = b.add_panel("Column A").default_closed(true);
  column_a.add_output<decl::Float>("A", "Column A Row A");
  column_a.add_output<decl::Float>("B", "Column A Row B");
  column_a.add_output<decl::Float>("C", "Column A Row C");
  column_a.add_output<decl::Float>("D", "Column A Row D");

  PanelDeclarationBuilder &column_b = b.add_panel("Column B").default_closed(true);
  column_b.add_output<decl::Float>("A", "Column B Row A");
  column_b.add_output<decl::Float>("B", "Column B Row B");
  column_b.add_output<decl::Float>("C", "Column B Row C");
  column_b.add_output<decl::Float>("D", "Column B Row D");

  PanelDeclarationBuilder &column_c = b.add_panel("Column C").default_closed(true);
  column_c.add_output<decl::Float>("A", "Column C Row A");
  column_c.add_output<decl::Float>("B", "Column C Row B");
  column_c.add_output<decl::Float>("C", "Column C Row C");
  column_c.add_output<decl::Float>("D", "Column C Row D");

  PanelDeclarationBuilder &column_d = b.add_panel("Column D").default_closed(true);
  column_d.add_output<decl::Float>("A", "Column D Row A");
  column_d.add_output<decl::Float>("B", "Column D Row B");
  column_d.add_output<decl::Float>("C", "Column D Row C");
  column_d.add_output<decl::Float>("D", "Column D Row D");
}

static void step_copy(const IndexMask &mask,
                      const Span<float> src,
                      const int64_t src_step,
                      const int64_t src_begin,
                      const int64_t dst_step,
                      const int64_t dst_begin,
                      MutableSpan<float> dst)
{
  BLI_assert(src_begin < src_step);
  BLI_assert(dst_begin < dst_step);
  BLI_assert(src.size() / src_step == dst.size() / dst_step);
  mask.foreach_index_optimized<int>([&](const int64_t index) {
    dst[dst_begin + dst_step * index] = src[src_begin + src_step * index];
  });
}

class SeparateMatrixFunction : public mf::MultiFunction {
 public:
  SeparateMatrixFunction()
  {
    static const mf::Signature signature = []() {
      mf::Signature signature;
      mf::SignatureBuilder builder{"Separate Matrix", signature};
      builder.single_input<float4x4>("Matrix");

      builder.single_output<float>("Column A Row A", mf::ParamFlag::SupportsUnusedOutput);
      builder.single_output<float>("Column A Row B", mf::ParamFlag::SupportsUnusedOutput);
      builder.single_output<float>("Column A Row C", mf::ParamFlag::SupportsUnusedOutput);
      builder.single_output<float>("Column A Row D", mf::ParamFlag::SupportsUnusedOutput);

      builder.single_output<float>("Column B Row A", mf::ParamFlag::SupportsUnusedOutput);
      builder.single_output<float>("Column B Row B", mf::ParamFlag::SupportsUnusedOutput);
      builder.single_output<float>("Column B Row C", mf::ParamFlag::SupportsUnusedOutput);
      builder.single_output<float>("Column B Row D", mf::ParamFlag::SupportsUnusedOutput);

      builder.single_output<float>("Column C Row A", mf::ParamFlag::SupportsUnusedOutput);
      builder.single_output<float>("Column C Row B", mf::ParamFlag::SupportsUnusedOutput);
      builder.single_output<float>("Column C Row C", mf::ParamFlag::SupportsUnusedOutput);
      builder.single_output<float>("Column C Row D", mf::ParamFlag::SupportsUnusedOutput);

      builder.single_output<float>("Column D Row A", mf::ParamFlag::SupportsUnusedOutput);
      builder.single_output<float>("Column D Row B", mf::ParamFlag::SupportsUnusedOutput);
      builder.single_output<float>("Column D Row C", mf::ParamFlag::SupportsUnusedOutput);
      builder.single_output<float>("Column D Row D", mf::ParamFlag::SupportsUnusedOutput);
      return signature;
    }();
    this->set_signature(&signature);
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    const VArray<float4x4> matrices = params.readonly_single_input<float4x4>(0, "Matrix");

    MutableSpan<float> column_a_row_a = params.uninitialized_single_output_if_required<float>(
        1, "Column A Row A");
    MutableSpan<float> column_a_row_b = params.uninitialized_single_output_if_required<float>(
        2, "Column A Row B");
    MutableSpan<float> column_a_row_c = params.uninitialized_single_output_if_required<float>(
        3, "Column A Row C");
    MutableSpan<float> column_a_row_d = params.uninitialized_single_output_if_required<float>(
        4, "Column A Row D");

    MutableSpan<float> column_b_row_a = params.uninitialized_single_output_if_required<float>(
        5, "Column B Row A");
    MutableSpan<float> column_b_row_b = params.uninitialized_single_output_if_required<float>(
        6, "Column B Row B");
    MutableSpan<float> column_b_row_c = params.uninitialized_single_output_if_required<float>(
        7, "Column B Row C");
    MutableSpan<float> column_b_row_d = params.uninitialized_single_output_if_required<float>(
        8, "Column B Row D");

    MutableSpan<float> column_c_row_a = params.uninitialized_single_output_if_required<float>(
        9, "Column C Row A");
    MutableSpan<float> column_c_row_b = params.uninitialized_single_output_if_required<float>(
        10, "Column C Row B");
    MutableSpan<float> column_c_row_c = params.uninitialized_single_output_if_required<float>(
        11, "Column C Row C");
    MutableSpan<float> column_c_row_d = params.uninitialized_single_output_if_required<float>(
        12, "Column C Row D");

    MutableSpan<float> column_d_row_a = params.uninitialized_single_output_if_required<float>(
        13, "Column D Row A");
    MutableSpan<float> column_d_row_b = params.uninitialized_single_output_if_required<float>(
        14, "Column D Row B");
    MutableSpan<float> column_d_row_c = params.uninitialized_single_output_if_required<float>(
        15, "Column D Row C");
    MutableSpan<float> column_d_row_d = params.uninitialized_single_output_if_required<float>(
        16, "Column D Row D");

    if (const std::optional<float4x4> single = matrices.get_if_single()) {
      printf("Single!\n");
      const float4x4 matrix = *single;
      column_a_row_a.fill(matrix[0][0]);
      column_a_row_b.fill(matrix[0][1]);
      column_a_row_c.fill(matrix[0][2]);
      column_a_row_d.fill(matrix[0][3]);

      column_b_row_a.fill(matrix[1][0]);
      column_b_row_b.fill(matrix[1][1]);
      column_b_row_c.fill(matrix[1][2]);
      column_b_row_d.fill(matrix[1][3]);

      column_c_row_a.fill(matrix[2][0]);
      column_c_row_b.fill(matrix[2][1]);
      column_c_row_c.fill(matrix[2][2]);
      column_c_row_d.fill(matrix[2][3]);

      column_d_row_a.fill(matrix[3][0]);
      column_d_row_b.fill(matrix[3][1]);
      column_d_row_c.fill(matrix[3][2]);
      column_d_row_d.fill(matrix[3][3]);
      return;
    }

    const VArraySpan<float4x4> span_matrices(matrices);
    const Span<float> components = span_matrices.cast<float>();

    step_copy(mask, components, 16, 0, 1, 0, column_a_row_a);
    step_copy(mask, components, 16, 1, 1, 0, column_a_row_b);
    step_copy(mask, components, 16, 2, 1, 0, column_a_row_c);
    step_copy(mask, components, 16, 3, 1, 0, column_a_row_d);

    step_copy(mask, components, 16, 4, 1, 0, column_b_row_a);
    step_copy(mask, components, 16, 5, 1, 0, column_b_row_b);
    step_copy(mask, components, 16, 6, 1, 0, column_b_row_c);
    step_copy(mask, components, 16, 7, 1, 0, column_b_row_d);

    step_copy(mask, components, 16, 8, 1, 0, column_c_row_a);
    step_copy(mask, components, 16, 9, 1, 0, column_c_row_b);
    step_copy(mask, components, 16, 10, 1, 0, column_c_row_c);
    step_copy(mask, components, 16, 11, 1, 0, column_c_row_d);

    step_copy(mask, components, 16, 12, 1, 0, column_d_row_a);
    step_copy(mask, components, 16, 13, 1, 0, column_d_row_b);
    step_copy(mask, components, 16, 14, 1, 0, column_d_row_c);
    step_copy(mask, components, 16, 15, 1, 0, column_d_row_d);
  }
};

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  const static SeparateMatrixFunction fn;
  builder.set_matching_fn(fn);
}

static void node_register()
{
  static bNodeType ntype;
  fn_node_type_base(&ntype, FN_NODE_SEPARATE_MATRIX, "Separate Matrix", NODE_CLASS_CONVERTER);
  ntype.declare = node_declare;
  ntype.build_multi_function = node_build_multi_function;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_fn_separate_matrix_cc
