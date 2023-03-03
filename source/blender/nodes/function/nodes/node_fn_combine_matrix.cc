/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_function_util.hh"

#include "UI_interface.h"
#include "UI_resources.h"

namespace blender::nodes::node_fn_combine_matrix_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.add_input<decl::Vector>(N_("Vec0")).default_value({1.0f, 0.0f, 0.0f});
  b.add_input<decl::Vector>(N_("Vec1")).default_value({0.0f, 1.0f, 0.0f});
  b.add_input<decl::Vector>(N_("Vec2")).default_value({0.0f, 0.0f, 1.0f});
  b.add_input<decl::Vector>(N_("Vec3")).default_value({0.0f, 0.0f, 0.0f});
  b.add_input<decl::Float>(N_("Row 0 Col 0")).default_value(1.0f);
  b.add_input<decl::Float>(N_("Row 1 Col 0")).default_value(0.0f);
  b.add_input<decl::Float>(N_("Row 2 Col 0")).default_value(0.0f);
  b.add_input<decl::Float>(N_("Row 3 Col 0")).default_value(0.0f);
  b.add_input<decl::Float>(N_("Row 0 Col 1")).default_value(0.0f);
  b.add_input<decl::Float>(N_("Row 1 Col 1")).default_value(1.0f);
  b.add_input<decl::Float>(N_("Row 2 Col 1")).default_value(0.0f);
  b.add_input<decl::Float>(N_("Row 3 Col 1")).default_value(0.0f);
  b.add_input<decl::Float>(N_("Row 0 Col 2")).default_value(0.0f);
  b.add_input<decl::Float>(N_("Row 1 Col 2")).default_value(0.0f);
  b.add_input<decl::Float>(N_("Row 2 Col 2")).default_value(1.0f);
  b.add_input<decl::Float>(N_("Row 3 Col 2")).default_value(0.0f);
  b.add_input<decl::Float>(N_("Row 0 Col 3")).default_value(0.0f);
  b.add_input<decl::Float>(N_("Row 1 Col 3")).default_value(0.0f);
  b.add_input<decl::Float>(N_("Row 2 Col 3")).default_value(0.0f);
  b.add_input<decl::Float>(N_("Row 3 Col 3")).default_value(1.0f);
  b.add_output<decl::Matrix4x4>(N_("Matrix"));
};

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "mode", 0, "", ICON_NONE);
}

static void node_update(bNodeTree *tree, bNode *node)
{
  const NodeCombSepMatrixMode mode = (NodeCombSepMatrixMode)node->custom1;

  const IndexRange vector_sockets(0, 4);
  const IndexRange scalar_sockets(4, 16);
  const bool show_vector_sockets = ELEM(
      mode, NODE_COMBSEP_MATRIX_COLUMNS, NODE_COMBSEP_MATRIX_ROWS);
  const bool show_scalar_sockets = ELEM(mode, NODE_COMBSEP_MATRIX_ELEMENTS);

  for (const int i : vector_sockets) {
    nodeSetSocketAvailability(
        tree, (bNodeSocket *)BLI_findlink(&node->inputs, i), show_vector_sockets);
  }
  for (const int i : scalar_sockets) {
    nodeSetSocketAvailability(
        tree, (bNodeSocket *)BLI_findlink(&node->inputs, i), show_scalar_sockets);
  }

  switch (mode) {
    case NODE_COMBSEP_MATRIX_COLUMNS:
      node_sock_label((bNodeSocket *)BLI_findlink(&node->inputs, vector_sockets[0]), "Column 0");
      node_sock_label((bNodeSocket *)BLI_findlink(&node->inputs, vector_sockets[1]), "Column 1");
      node_sock_label((bNodeSocket *)BLI_findlink(&node->inputs, vector_sockets[2]), "Column 2");
      node_sock_label((bNodeSocket *)BLI_findlink(&node->inputs, vector_sockets[3]), "Column 3");
      break;
    case NODE_COMBSEP_MATRIX_ROWS:
      node_sock_label((bNodeSocket *)BLI_findlink(&node->inputs, vector_sockets[0]), "Row 0");
      node_sock_label((bNodeSocket *)BLI_findlink(&node->inputs, vector_sockets[1]), "Row 1");
      node_sock_label((bNodeSocket *)BLI_findlink(&node->inputs, vector_sockets[2]), "Row 2");
      node_sock_label((bNodeSocket *)BLI_findlink(&node->inputs, vector_sockets[3]), "Row 3");
      break;
    case NODE_COMBSEP_MATRIX_ELEMENTS:
      break;
  }
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = NODE_COMBSEP_MATRIX_COLUMNS;
}

static const mf::MultiFunction *get_multi_function(const bNode &bnode)
{
  const NodeCombSepMatrixMode mode = (NodeCombSepMatrixMode)bnode.custom1;

  static auto columns_fn = mf::build::SI4_SO<float3, float3, float3, float3, float4x4>(
      "columns_to_matrix",
      [](const float3 &c0, const float3 &c1, const float3 &c2, const float3 &c3) {
        float4x4 m;
        m.view()[0] = float4(c0, 0.0f);
        m.view()[1] = float4(c1, 0.0f);
        m.view()[2] = float4(c2, 0.0f);
        m.view()[3] = float4(c3, 1.0f);
        return m;
      });
  static auto rows_fn = mf::build::SI4_SO<float3, float3, float3, float3, float4x4>(
      "rows_to_matrix",
      [](const float3 &r0, const float3 &r1, const float3 &r2, const float3 &r3) {
        float4x4 m;
        m[0][0] = r0[0];
        m[0][1] = r1[0];
        m[0][2] = r2[0];
        m[0][3] = r3[0];
        m[1][0] = r0[1];
        m[1][1] = r1[1];
        m[1][2] = r2[1];
        m[1][3] = r3[1];
        m[2][0] = r0[2];
        m[2][1] = r1[2];
        m[2][2] = r2[2];
        m[2][3] = r3[2];
        m[3][0] = 0.0f;
        m[3][1] = 0.0f;
        m[3][2] = 0.0f;
        m[3][3] = 1.0f;
        return m;
      });
  static auto exec_preset = mf::build::exec_presets::AllSpanOrSingle();
  static auto elements_fn =
      mf::build::detail::build_multi_function_with_n_inputs_one_output<float4x4>(
          "elements_to_matrix",
          [](float m00,
             float m01,
             float m02,
             float m03,
             float m10,
             float m11,
             float m12,
             float m13,
             float m20,
             float m21,
             float m22,
             float m23,
             float m30,
             float m31,
             float m32,
             float m33) {
            const float elements[] = {
                m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33};
            return float4x4(elements);
          },
          exec_preset,
          TypeSequence<float, float, float, float, float, float, float, float, float, float, float, float, float, float, float, float>());

  switch (mode) {
    case NODE_COMBSEP_MATRIX_COLUMNS:
      return &columns_fn;
    case NODE_COMBSEP_MATRIX_ROWS:
      return &rows_fn;
    case NODE_COMBSEP_MATRIX_ELEMENTS:
      return &elements_fn;
  }

  BLI_assert_unreachable();
  return nullptr;
}

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  const mf::MultiFunction *fn = get_multi_function(builder.node());
  builder.set_matching_fn(fn);
}

}  // namespace blender::nodes::node_fn_combine_matrix_cc

void register_node_type_fn_combine_matrix(void)
{
  namespace file_ns = blender::nodes::node_fn_combine_matrix_cc;

  static bNodeType ntype;

  fn_node_type_base(&ntype, FN_NODE_COMBINE_MATRIX, "Combine Matrix", NODE_CLASS_CONVERTER);
  ntype.declare = file_ns::node_declare;
  ntype.updatefunc = file_ns::node_update;
  ntype.initfunc = file_ns::node_init;
  ntype.build_multi_function = file_ns::node_build_multi_function;
  ntype.draw_buttons = file_ns::node_layout;

  nodeRegisterType(&ntype);
}
