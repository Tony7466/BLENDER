/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_function_util.hh"

#include "BLI_math_matrix.h"

#include "UI_interface.h"
#include "UI_resources.h"

namespace blender::nodes::node_fn_input_matrix3x3_cc {

NODE_STORAGE_FUNCS(NodeInputMatrix3x3);

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
  NodeInputMatrix3x3 *data = MEM_new<NodeInputMatrix3x3>(__func__);
  unit_m3(data->matrix);
  node->storage = data;
}

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  const NodeInputMatrix3x3 &storage = node_storage(builder.node());
  builder.construct_and_set_matching_fn<mf::CustomMF_Constant<float3x3>>(float3x3{storage.matrix});
}

}  // namespace blender::nodes::node_fn_input_matrix3x3_cc

void register_node_type_fn_input_matrix_3x3(void)
{
  namespace file_ns = blender::nodes::node_fn_input_matrix3x3_cc;

  static bNodeType ntype;

  fn_node_type_base(&ntype, FN_NODE_INPUT_MATRIX_3X3, "3x3 Matrix", NODE_CLASS_INPUT);
  node_type_storage(
      &ntype, "NodeInputMatrix3x3", node_free_standard_storage, node_copy_standard_storage);
  ntype.declare = file_ns::node_declare;
  ntype.initfunc = file_ns::node_init;
  ntype.build_multi_function = file_ns::node_build_multi_function;
  ntype.draw_buttons = file_ns::node_layout;
  /* Matrix columns need more horizontal space */
  node_type_size_preset(&ntype, NODE_SIZE_LARGE);

  nodeRegisterType(&ntype);
}
