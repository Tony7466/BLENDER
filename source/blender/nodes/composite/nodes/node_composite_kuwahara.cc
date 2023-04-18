/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2020 Blender Foundation */

/** \file
 * \ingroup cmpnodes
 */


//#include "COM_shader_node.hh"
#include "COM_node_operation.hh"

#include "node_composite_util.hh"

/* **************** Kuwahara ******************** */

namespace blender::nodes::node_composite_kuwahara_cc {

NODE_STORAGE_FUNCS(NodeKuwaharaData)

static void cmp_node_kuwahara_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Color>(N_("Image"))
      .default_value({1.0f, 1.0f, 1.0f, 1.0f})
      .compositor_domain_priority(0);
  b.add_output<decl::Color>(N_("Image"));
}

static void node_composit_init_kuwahara(bNodeTree * /*ntree*/, bNode *node)
{
  NodeKuwaharaData *data = MEM_cnew<NodeKuwaharaData>(__func__);
  node->storage = data;

  data->kernel_size = 4;
}

static void node_composit_buts_kuwahara(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiLayout *col;

  col = uiLayoutColumn(layout, false);

  uiItemR(col, ptr, "variation", 0, nullptr, ICON_NONE);
  uiItemR(col, ptr, "kernel_size", 0, nullptr, ICON_NONE);
}



//using namespace blender::realtime_compositor;
//
//class KuwaharaShaderNode : public NodeOperation {
// public:
//  using NodeOperation::NodeOperation;
//
//  void execute() override
//  {
//    get_input("Image").pass_through(get_result("Image"));
//    context().set_info_message("Viewport compositor setup not fully supported");
//  }
//  using ShaderNode::ShaderNode;
//
//  void compile(GPUMaterial *material) override
//  {
//    GPUNodeStack *inputs = get_inputs_array();
//    GPUNodeStack *outputs = get_outputs_array();
//
//    GPU_stack_link(material, &bnode(), "node_composite_kuwahara", inputs, outputs);
//  }
//};
//
//static NodeOperation *get_compositor_operation(Context *context, DNode node)
//{
//  return new KuwaharaOperation(context, node);
//}

//static ShaderNode *get_compositor_shader_node(DNode node)
//{
//  return new KuwaharaShaderNode(node);
//}

}  // namespace blender::nodes::node_composite_kuwahara_cc

void register_node_type_cmp_kuwahara()
{
  namespace file_ns = blender::nodes::node_composite_kuwahara_cc;

  static bNodeType ntype;

  cmp_node_type_base(&ntype, CMP_NODE_KUWAHARA, "Kuwahara", NODE_CLASS_OP_COLOR);
  ntype.declare = file_ns::cmp_node_kuwahara_declare;
  ntype.draw_buttons = file_ns::node_composit_buts_kuwahara;
  ntype.initfunc = file_ns::node_composit_init_kuwahara;
  node_type_storage(
      &ntype, "NodeKuwaharaData", node_free_standard_storage, node_copy_standard_storage);
//  ntype.get_compositor_shader_node = file_ns::get_compositor_shader_node;

  nodeRegisterType(&ntype);
}
