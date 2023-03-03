/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_function_util.hh"

#include "BLI_math_matrix.hh"

#include "NOD_socket_search_link.hh"

#include "UI_interface.h"
#include "UI_resources.h"

namespace blender::nodes::node_fn_matrix_math_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.add_input<decl::Matrix4x4>(N_("Matrix"));
  b.add_input<decl::Matrix4x4>(N_("Matrix"), "Matrix_001");
  b.add_input<decl::Float>(N_("Scale")).default_value(1.0f);
  b.add_output<decl::Matrix4x4>(N_("Matrix"));
  b.add_output<decl::Float>(N_("Value"));
};

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "operation", UI_ITEM_R_SPLIT_EMPTY_NAME, "", ICON_NONE);
}

static void node_update(bNodeTree *tree, bNode *node)
{
  const NodeMatrixMathOperation op = (NodeMatrixMathOperation)node->custom1;

  bNodeSocket *in_matrix_a = (bNodeSocket *)BLI_findlink(&node->inputs, 0);
  bNodeSocket *in_matrix_b = (bNodeSocket *)BLI_findlink(&node->inputs, 1);
  bNodeSocket *in_scale = (bNodeSocket *)BLI_findlink(&node->inputs, 2);
  bNodeSocket *out_matrix = (bNodeSocket *)BLI_findlink(&node->outputs, 0);
  bNodeSocket *out_value = (bNodeSocket *)BLI_findlink(&node->outputs, 1);

  nodeSetSocketAvailability(tree, in_matrix_a, true);
  nodeSetSocketAvailability(
      tree,
      in_matrix_b,
      ELEM(op, NODE_MATRIX_MATH_ADD, NODE_MATRIX_MATH_SUBTRACT, NODE_MATRIX_MATH_MULTIPLY));
  nodeSetSocketAvailability(tree, in_scale, ELEM(op, NODE_MATRIX_MATH_SCALAR_MULTIPLY));

  nodeSetSocketAvailability(tree,
                            out_matrix,
                            ELEM(op,
                                 NODE_MATRIX_MATH_ADD,
                                 NODE_MATRIX_MATH_SUBTRACT,
                                 NODE_MATRIX_MATH_SCALAR_MULTIPLY,
                                 NODE_MATRIX_MATH_MULTIPLY,
                                 NODE_MATRIX_MATH_TRANSPOSE,
                                 NODE_MATRIX_MATH_INVERSE));
  nodeSetSocketAvailability(
      tree, out_value, ELEM(op, NODE_MATRIX_MATH_DETERMINANT, NODE_MATRIX_MATH_TRACE));

  /* Labels */
  node_sock_label_clear(in_matrix_a);
  node_sock_label_clear(in_matrix_b);
  node_sock_label_clear(in_scale);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = NODE_MATRIX_MATH_ADD;
}

static const mf::MultiFunction *get_multi_function(NodeMatrixMathOperation op)
{
  static auto exec_preset = mf::build::exec_presets::AllSpanOrSingle();

  switch (op) {
    case NODE_MATRIX_MATH_ADD: {
      static auto fn = mf::build::SI2_SO<float4x4, float4x4, float4x4>(
          "add",
          [](const float4x4 &a, const float4x4 &b) -> float4x4 { return a + b; },
          exec_preset);
      return &fn;
    }
    case NODE_MATRIX_MATH_SUBTRACT: {
      static auto fn = mf::build::SI2_SO<float4x4, float4x4, float4x4>(
          "subtract",
          [](const float4x4 &a, const float4x4 &b) -> float4x4 { return a - b; },
          exec_preset);
      return &fn;
    }
    case NODE_MATRIX_MATH_SCALAR_MULTIPLY: {
      static auto fn = mf::build::SI2_SO<float4x4, float, float4x4>(
          "scalar_multiply",
          [](const float4x4 &a, const float &s) -> float4x4 { return a * s; },
          exec_preset);
      return &fn;
    }
    case NODE_MATRIX_MATH_MULTIPLY: {
      static auto fn = mf::build::SI2_SO<float4x4, float4x4, float4x4>(
          "multiply",
          [](const float4x4 &a, const float4x4 &b) -> float4x4 { return a * b; },
          exec_preset);
      return &fn;
    }
    case NODE_MATRIX_MATH_TRANSPOSE: {
      static auto fn = mf::build::SI1_SO<float4x4, float4x4>(
          "transpose",
          [](const float4x4 &a) -> float4x4 { return math::transpose(a); },
          exec_preset);
      return &fn;
    }
    case NODE_MATRIX_MATH_INVERSE: {
      static auto fn = mf::build::SI1_SO<float4x4, float4x4>(
          "inverse",
          [](const float4x4 &a) -> float4x4 { return math::invert(a); },
          exec_preset);
      return &fn;
    }
    case NODE_MATRIX_MATH_DETERMINANT: {
      static auto fn = mf::build::SI1_SO<float4x4, float>(
          "determinant",
          [](const float4x4 &a) -> float { return math::determinant(a); },
          exec_preset);
      return &fn;
    }
    case NODE_MATRIX_MATH_TRACE: {
      static auto fn = mf::build::SI1_SO<float4x4, float>(
          "trace",
          [](const float4x4 &a) -> float { return a[0][0] + a[1][1] + a[2][2] + a[3][3]; },
          exec_preset);
      return &fn;
    }
  }

  return nullptr;
}

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  const NodeMatrixMathOperation op = (NodeMatrixMathOperation)builder.node().custom1;
  const mf::MultiFunction *fn = get_multi_function(op);
  builder.set_matching_fn(fn);
}

}  // namespace blender::nodes::node_fn_matrix_math_cc

void register_node_type_fn_matrix_math(void)
{
  namespace file_ns = blender::nodes::node_fn_matrix_math_cc;

  static bNodeType ntype;

  fn_node_type_base(&ntype, FN_NODE_MATRIX_MATH, "Matrix Math", NODE_CLASS_CONVERTER);
  ntype.declare = file_ns::node_declare;
  ntype.updatefunc = file_ns::node_update;
  ntype.initfunc = file_ns::node_init;
  ntype.build_multi_function = file_ns::node_build_multi_function;
  ntype.draw_buttons = file_ns::node_layout;

  nodeRegisterType(&ntype);
}
