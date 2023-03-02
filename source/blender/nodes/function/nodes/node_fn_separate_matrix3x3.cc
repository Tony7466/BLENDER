/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_function_util.hh"

#include "UI_interface.h"
#include "UI_resources.h"

namespace blender::nodes::node_fn_separate_matrix3x3_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.add_input<decl::Matrix4x4>(N_("Matrix")).default_value(float4x4::identity());
  b.add_output<decl::Vector>(N_("Vec0"));
  b.add_output<decl::Vector>(N_("Vec1"));
  b.add_output<decl::Vector>(N_("Vec2"));
  b.add_output<decl::Float>(N_("Row 0 Col 0"));
  b.add_output<decl::Float>(N_("Row 1 Col 0"));
  b.add_output<decl::Float>(N_("Row 2 Col 0"));
  b.add_output<decl::Float>(N_("Row 0 Col 1"));
  b.add_output<decl::Float>(N_("Row 1 Col 1"));
  b.add_output<decl::Float>(N_("Row 2 Col 1"));
  b.add_output<decl::Float>(N_("Row 0 Col 2"));
  b.add_output<decl::Float>(N_("Row 1 Col 2"));
  b.add_output<decl::Float>(N_("Row 2 Col 2"));
};

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "mode", 0, "", ICON_NONE);
}

static void node_update(bNodeTree *tree, bNode *node)
{
  const NodeCombSepMatrixMode mode = (NodeCombSepMatrixMode)node->custom1;

  const IndexRange vector_sockets(0, 3);
  const IndexRange scalar_sockets(3, 9);
  const bool show_vector_sockets = ELEM(
      mode, NODE_COMBSEP_MATRIX_COLUMNS, NODE_COMBSEP_MATRIX_ROWS);
  const bool show_scalar_sockets = ELEM(mode, NODE_COMBSEP_MATRIX_ELEMENTS);

  for (const int i : vector_sockets) {
    nodeSetSocketAvailability(
        tree, (bNodeSocket *)BLI_findlink(&node->outputs, i), show_vector_sockets);
  }
  for (const int i : scalar_sockets) {
    nodeSetSocketAvailability(
        tree, (bNodeSocket *)BLI_findlink(&node->outputs, i), show_scalar_sockets);
  }

  switch (mode) {
    case NODE_COMBSEP_MATRIX_COLUMNS:
      node_sock_label((bNodeSocket *)BLI_findlink(&node->outputs, vector_sockets[0]), "Column 0");
      node_sock_label((bNodeSocket *)BLI_findlink(&node->outputs, vector_sockets[1]), "Column 1");
      node_sock_label((bNodeSocket *)BLI_findlink(&node->outputs, vector_sockets[2]), "Column 2");
      break;
    case NODE_COMBSEP_MATRIX_ROWS:
      node_sock_label((bNodeSocket *)BLI_findlink(&node->outputs, vector_sockets[0]), "Row 0");
      node_sock_label((bNodeSocket *)BLI_findlink(&node->outputs, vector_sockets[1]), "Row 1");
      node_sock_label((bNodeSocket *)BLI_findlink(&node->outputs, vector_sockets[2]), "Row 2");
      break;
    case NODE_COMBSEP_MATRIX_ELEMENTS:
      break;
  }
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = NODE_COMBSEP_MATRIX_COLUMNS;
}

class SeparateColumnsFunction : public mf::MultiFunction {
 public:
  SeparateColumnsFunction()
  {
    static const mf::Signature signature = []() {
      mf::Signature signature;
      mf::SignatureBuilder builder{"Separate Matrix 3x3", signature};
      builder.single_input<float3x3>("Matrix");
      builder.single_output<float3>("Column0");
      builder.single_output<float3>("Column1");
      builder.single_output<float3>("Column2");
      return signature;
    }();
    this->set_signature(&signature);
  }

  void call(IndexMask mask, mf::Params params, mf::Context /*context*/) const override
  {
    const VArray<float3x3> &matrices = params.readonly_single_input<float3x3>(0, "Matrix");
    MutableSpan<float3> col0 = params.uninitialized_single_output<float3>(1, "Column0");
    MutableSpan<float3> col1 = params.uninitialized_single_output<float3>(2, "Column1");
    MutableSpan<float3> col2 = params.uninitialized_single_output<float3>(3, "Column2");

    for (int64_t i : mask) {
      const float3x3 &mat = matrices[i];
      col0[i] = float3(mat[0]);
      col1[i] = float3(mat[1]);
      col2[i] = float3(mat[2]);
    }
  }
};

class SeparateRowsFunction : public mf::MultiFunction {
 public:
  SeparateRowsFunction()
  {
    static const mf::Signature signature = []() {
      mf::Signature signature;
      mf::SignatureBuilder builder{"Separate Matrix 3x3", signature};
      builder.single_input<float3x3>("Matrix");
      builder.single_output<float3>("Row0");
      builder.single_output<float3>("Row1");
      builder.single_output<float3>("Row2");
      return signature;
    }();
    this->set_signature(&signature);
  }

  void call(IndexMask mask, mf::Params params, mf::Context /*context*/) const override
  {
    const VArray<float3x3> &matrices = params.readonly_single_input<float3x3>(0, "Matrix");
    MutableSpan<float3> row0 = params.uninitialized_single_output<float3>(1, "Row0");
    MutableSpan<float3> row1 = params.uninitialized_single_output<float3>(2, "Row1");
    MutableSpan<float3> row2 = params.uninitialized_single_output<float3>(3, "Row2");

    for (int64_t i : mask) {
      const float3x3 &mat = matrices[i];
      row0[i] = float3(mat[0][0], mat[1][0], mat[2][0]);
      row1[i] = float3(mat[0][1], mat[1][1], mat[2][1]);
      row2[i] = float3(mat[0][2], mat[1][2], mat[2][2]);
    }
  }
};

class SeparateElementsFunction : public mf::MultiFunction {
 public:
  SeparateElementsFunction()
  {
    static const mf::Signature signature = []() {
      mf::Signature signature;
      mf::SignatureBuilder builder{"Separate Matrix 3x3", signature};
      builder.single_input<float3x3>("Matrix");
      builder.single_output<float>("Row0Column0");
      builder.single_output<float>("Row0Column1");
      builder.single_output<float>("Row0Column2");
      builder.single_output<float>("Row1Column0");
      builder.single_output<float>("Row1Column1");
      builder.single_output<float>("Row1Column2");
      builder.single_output<float>("Row2Column0");
      builder.single_output<float>("Row2Column1");
      builder.single_output<float>("Row2Column2");
      return signature;
    }();
    this->set_signature(&signature);
  }

  void call(IndexMask mask, mf::Params params, mf::Context /*context*/) const override
  {
    const VArray<float3x3> &matrices = params.readonly_single_input<float3x3>(0, "Matrix");
    MutableSpan<float> m00 = params.uninitialized_single_output<float>(0, "Row0Column0");
    MutableSpan<float> m10 = params.uninitialized_single_output<float>(1, "Row0Column1");
    MutableSpan<float> m20 = params.uninitialized_single_output<float>(2, "Row0Column2");
    MutableSpan<float> m01 = params.uninitialized_single_output<float>(3, "Row1Column0");
    MutableSpan<float> m11 = params.uninitialized_single_output<float>(4, "Row1Column1");
    MutableSpan<float> m21 = params.uninitialized_single_output<float>(5, "Row1Column2");
    MutableSpan<float> m02 = params.uninitialized_single_output<float>(6, "Row2Column0");
    MutableSpan<float> m12 = params.uninitialized_single_output<float>(7, "Row2Column1");
    MutableSpan<float> m22 = params.uninitialized_single_output<float>(8, "Row2Column2");

    for (int64_t i : mask) {
      const float3x3 &mat = matrices[i];
      m00[i] = mat[0][0];
      m01[i] = mat[0][1];
      m02[i] = mat[0][2];
      m10[i] = mat[1][0];
      m11[i] = mat[1][1];
      m12[i] = mat[1][2];
      m20[i] = mat[2][0];
      m21[i] = mat[2][1];
      m22[i] = mat[2][2];
    }
  }
};

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  const NodeCombSepMatrixMode mode = (NodeCombSepMatrixMode)builder.node().custom1;

  switch (mode) {
    case NODE_COMBSEP_MATRIX_COLUMNS: {
      static SeparateColumnsFunction fn;
      builder.set_matching_fn(fn);
      break;
    }
    case NODE_COMBSEP_MATRIX_ROWS: {
      static SeparateRowsFunction fn;
      builder.set_matching_fn(fn);
      break;
    }
    case NODE_COMBSEP_MATRIX_ELEMENTS: {
      static SeparateElementsFunction fn;
      builder.set_matching_fn(fn);
      break;
    }
    default: {
      BLI_assert_unreachable();
      break;
    }
  }
}

}  // namespace blender::nodes::node_fn_separate_matrix3x3_cc

void register_node_type_fn_separate_matrix_3x3(void)
{
  namespace file_ns = blender::nodes::node_fn_separate_matrix3x3_cc;

  static bNodeType ntype;

  fn_node_type_base(&ntype, FN_NODE_SEPARATE_MATRIX_3X3, "Separate 3x3 Matrix", NODE_CLASS_CONVERTER);
  ntype.declare = file_ns::node_declare;
  ntype.updatefunc = file_ns::node_update;
  ntype.initfunc = file_ns::node_init;
  ntype.build_multi_function = file_ns::node_build_multi_function;
  ntype.draw_buttons = file_ns::node_layout;

  nodeRegisterType(&ntype);
}
