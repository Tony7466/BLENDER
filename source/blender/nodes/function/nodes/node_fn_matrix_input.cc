/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_matrix.hh"

#include "node_function_util.hh"

namespace blender::nodes::node_fn_matrix_input_cc {

NODE_STORAGE_FUNCS(NodeInputMatrix)

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.add_output<decl::Matrix>("Matrix");
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "matrix", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeInputMatrix *data = MEM_cnew<NodeInputMatrix>(__func__);
  // float4x4_mutableview(&data->matrix) = float4x4::identity();
  node->storage = data;
}

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  const NodeInputMatrix &storage = node_storage(builder.node());
  builder.construct_and_set_matching_fn<mf::CustomMF_Constant<float4x4>>(float4x4(storage.matrix));
}

static void node_register()
{
  static bNodeType ntype;
  fn_node_type_base(&ntype, FN_NODE_MATRIX, "Matrix", NODE_CLASS_CONVERTER);
  ntype.declare = node_declare;
  ntype.build_multi_function = node_build_multi_function;
  ntype.initfunc = node_init;
  node_type_size_preset(&ntype, bke::eNodeSizePreset::LARGE);
  node_type_storage(
      &ntype, "NodeInputMatrix", node_free_standard_storage, node_copy_standard_storage);
  ntype.draw_buttons = node_layout;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_fn_matrix_input_cc
