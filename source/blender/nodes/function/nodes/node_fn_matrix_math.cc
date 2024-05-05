/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_listbase.h"
#include "BLI_math_matrix.hh"
#include "BLI_string.h"
#include "BLI_string_utf8.h"

#include "RNA_enum_types.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "NOD_rna_define.hh"

#include "node_function_util.hh"

namespace blender::nodes::node_fn_matrix_math_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.add_input<decl::Matrix>("Matrix");
  b.add_input<decl::Matrix>("Matrix", "Matrix_001");
  b.add_output<decl::Matrix>("Matrix");
  b.add_output<decl::Float>("Float");
  b.add_output<decl::Bool>("Boolean");
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "operation", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_update(bNodeTree *ntree, bNode *node)
{
  bNodeSocket *sockB = (bNodeSocket *)BLI_findlink(&node->inputs, 1);

  bNodeSocket *sockMatrix = nodeFindSocket(node, SOCK_OUT, "Matrix");
  bNodeSocket *sockFloat = nodeFindSocket(node, SOCK_OUT, "Float");
  bNodeSocket *sockBool = nodeFindSocket(node, SOCK_OUT, "Boolean");

  bke::nodeSetSocketAvailability(ntree, sockB, ELEM(node->custom1, NODE_MATRIX_MATH_MULTIPLY));

  bke::nodeSetSocketAvailability(
      ntree, sockMatrix, !ELEM(node->custom1, NODE_MATRIX_MATH_DETERMINANT));
  bke::nodeSetSocketAvailability(
      ntree, sockFloat, ELEM(node->custom1, NODE_MATRIX_MATH_DETERMINANT));
  bke::nodeSetSocketAvailability(ntree, sockBool, ELEM(node->custom1, NODE_MATRIX_MATH_INVERT));

  /* Labels */
  node_sock_label_clear(sockFloat);
  node_sock_label_clear(sockBool);
  switch (node->custom1) {
    case NODE_MATRIX_MATH_DETERMINANT:
      node_sock_label(sockFloat, "Determinant");
      break;
    case NODE_MATRIX_MATH_INVERT:
      node_sock_label(sockBool, "Invertable");
      break;
  }
}

static void node_label(const bNodeTree * /*tree*/,
                       const bNode *node,
                       char *label,
                       int label_maxncpy)
{
  const char *name;
  bool enum_label = RNA_enum_name(rna_enum_node_matrix_math_items, node->custom1, &name);
  if (!enum_label) {
    name = "Unknown";
  }
  BLI_strncpy_utf8(label, IFACE_(name), label_maxncpy);
}

class InvertMatrixFunction : public mf::MultiFunction {
 public:
  InvertMatrixFunction()
  {
    static mf::Signature signature = []() {
      mf::Signature signature;
      mf::SignatureBuilder builder{"Invert Matrix", signature};
      builder.single_input<float4x4>("Matrix");
      builder.single_output<float4x4>("Matrix", mf::ParamFlag::SupportsUnusedOutput);
      builder.single_output<bool>("Invertable", mf::ParamFlag::SupportsUnusedOutput);
      return signature;
    }();
    this->set_signature(&signature);
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context context) const override
  {
    const VArraySpan<float4x4> in_matrices = params.readonly_single_input<float4x4>(0, "Matrix");
    MutableSpan<float4x4> out_matrices = params.uninitialized_single_output_if_required<float4x4>(
        1, "Matrix");
    MutableSpan<bool> out_invertable = params.uninitialized_single_output_if_required<bool>(
        2, "Invertable");
    mask.foreach_index([&](const int64_t i) {
      const float4x4 &matrix = in_matrices[i];
      bool success;
      float4x4 inverted_matrix = math::invert(matrix, success);
      if (!out_matrices.is_empty()) {
        out_matrices[i] = success ? inverted_matrix : float4x4::identity();
      }
      if (!out_invertable.is_empty()) {
        out_invertable[i] = success;
      }
    });
  }
};

static const mf::MultiFunction *get_multi_function(const bNode &bnode)
{
  static auto multiply_fn = mf::build::SI2_SO<float4x4, float4x4, float4x4>(
      "Multiply Matrices", [](float4x4 a, float4x4 b) { return a * b; });
  static InvertMatrixFunction invert_fn;
  static auto transpose_fn = mf::build::SI1_SO<float4x4, float4x4>(
      "Transpose Matrix", [](float4x4 matrix) { return math::transpose(matrix); });
  static auto normalize_fn = mf::build::SI1_SO<float4x4, float4x4>(
      "Normalize Matrix", [](float4x4 matrix) { return math::normalize(matrix); });
  static auto determinant_fn = mf::build::SI1_SO<float4x4, float>(
      "Matrix Determinant", [](float4x4 matrix) { return math::determinant(matrix); });
  static auto adjoint_fn = mf::build::SI1_SO<float4x4, float4x4>(
      "Adjugate Matrix", [](float4x4 matrix) { return math::adjoint(matrix); });

  switch (bnode.custom1) {
    case NODE_MATRIX_MATH_MULTIPLY:
      return &multiply_fn;
    case NODE_MATRIX_MATH_INVERT:
      return &invert_fn;
    case NODE_MATRIX_MATH_TRANSPOSE:
      return &transpose_fn;
    case NODE_MATRIX_MATH_NORMALIZE:
      return &normalize_fn;
    case NODE_MATRIX_MATH_DETERMINANT:
      return &determinant_fn;
    case NODE_MATRIX_MATH_ADJUGATE:
      return &adjoint_fn;
  }

  BLI_assert_unreachable();
  return nullptr;
}

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  const mf::MultiFunction *fn = get_multi_function(builder.node());
  builder.set_matching_fn(fn);
}

static void node_rna(StructRNA *srna)
{
  RNA_def_node_enum(srna,
                    "operation",
                    "Operation",
                    "",
                    rna_enum_node_matrix_math_items,
                    NOD_inline_enum_accessors(custom1));
}

static void node_register()
{
  static bNodeType ntype;

  fn_node_type_base(&ntype, FN_NODE_MATRIX_MATH, "Matrix Math", NODE_CLASS_CONVERTER);
  ntype.declare = node_declare;
  ntype.labelfunc = node_label;
  ntype.updatefunc = node_update;
  ntype.build_multi_function = node_build_multi_function;
  ntype.draw_buttons = node_layout;
  nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_fn_matrix_math_cc
