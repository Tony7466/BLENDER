/* SPDX-FileCopyrightText: 2005 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_shader_util.hh"

namespace blender::nodes::node_shader_add_shader_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Shader>("Shader");
  b.add_input<decl::Shader>("Shader", "Shader_001");
  b.add_output<decl::Shader>("Shader");
}

static int node_shader_gpu_add_shader(GPUMaterial *mat,
                                      bNode *node,
                                      bNodeExecData * /*execdata*/,
                                      GPUNodeStack *in,
                                      GPUNodeStack *out)
{
  return GPU_stack_link(mat, node, "node_add_shader", in, out);
}

NODE_SHADER_MATERIALX_BEGIN
{
  NodeItem res = empty();
  switch (to_type_) {
    case NodeItem::Type::BSDF:
    case NodeItem::Type::EDF: {
      NodeItem shader1 = get_input_link(0, to_type_);
      NodeItem shader2 = get_input_link(1, to_type_);

      if (shader1 && !shader2) {
        res = shader1;
      }
      else if (!shader1 && shader2) {
        res = shader2;
      }
      else if (shader1 && shader2) {
        res = shader1 + shader2;
      }
      break;
    }
    case NodeItem::Type::SurfaceShader: {
      /* SurfaceShaders can't be added, returning the first one connected */
      res = get_input_link(0, to_type_);
      if (!res) {
        res = get_input_link(1, to_type_);
      }
      break;
    }
    default:
      BLI_assert_unreachable();
  }
  return res;
}
NODE_SHADER_MATERIALX_END

}  // namespace blender::nodes::node_shader_add_shader_cc

/* node type definition */
void register_node_type_sh_add_shader()
{
  namespace file_ns = blender::nodes::node_shader_add_shader_cc;

  static bNodeType ntype;

  sh_node_type_base(&ntype, SH_NODE_ADD_SHADER, "Add Shader", NODE_CLASS_SHADER);
  ntype.declare = file_ns::node_declare;
  ntype.gpu_fn = file_ns::node_shader_gpu_add_shader;

  nodeRegisterType(&ntype);
}
