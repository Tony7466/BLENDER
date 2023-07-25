/* SPDX-FileCopyrightText: 2005 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_shader_util.hh"

namespace blender::nodes::node_shader_layer_shader_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Shader>("Base");
  b.add_input<decl::Shader>("Top");
  b.add_output<decl::Shader>("Shader");
}

static int node_shader_gpu_layer_shader(GPUMaterial *mat,
                                        bNode *node,
                                        bNodeExecData * /*execdata*/,
                                        GPUNodeStack *in,
                                        GPUNodeStack *out)
{
  /* TODO */
  return GPU_stack_link(mat, node, "node_add_shader", in, out);
}

}  // namespace blender::nodes::node_shader_layer_shader_cc

/* node type definition */
void register_node_type_sh_layer_shader()
{
  namespace file_ns = blender::nodes::node_shader_layer_shader_cc;

  static bNodeType ntype;

  sh_node_type_base(&ntype, SH_NODE_LAYER_SHADER, "Layer Shader", NODE_CLASS_SHADER);
  ntype.declare = file_ns::node_declare;
  ntype.gpu_fn = file_ns::node_shader_gpu_layer_shader;

  nodeRegisterType(&ntype);
}
