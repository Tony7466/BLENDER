/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_function_util.hh"

#include "BLI_math_matrix.h"

#include "UI_interface.h"
#include "UI_resources.h"

namespace blender::nodes::node_fn_input_matrix_cc {

NODE_STORAGE_FUNCS(NodeInputMatrix);

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Matrix4x4>(N_("Matrix"));
};

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "matrix", 0, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeInputMatrix *data = MEM_new<NodeInputMatrix>(__func__);
  unit_m4(data->matrix);
  node->storage = data;
}

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  const NodeInputMatrix &storage = node_storage(builder.node());
  builder.construct_and_set_matching_fn<mf::CustomMF_Constant<float4x4>>(float4x4{storage.matrix});
}

}  // namespace blender::nodes::node_fn_input_matrix_cc

void register_node_type_fn_input_matrix(void)
{
  namespace file_ns = blender::nodes::node_fn_input_matrix_cc;

  static bNodeType ntype;

  fn_node_type_base(&ntype, FN_NODE_INPUT_MATRIX, "Matrix", NODE_CLASS_INPUT);
  node_type_storage(
      &ntype, "NodeInputMatrix", node_free_standard_storage, node_copy_standard_storage);
  ntype.declare = file_ns::node_declare;
  ntype.initfunc = file_ns::node_init;
  ntype.build_multi_function = file_ns::node_build_multi_function;
  ntype.draw_buttons = file_ns::node_layout;
  /* Matrix columns need more horizontal space */
  node_type_size_preset(&ntype, NODE_SIZE_LARGE);

  nodeRegisterType(&ntype);
}
