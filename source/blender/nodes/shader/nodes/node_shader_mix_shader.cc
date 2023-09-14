/* SPDX-FileCopyrightText: 2005 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_shader_util.hh"

namespace blender::nodes::node_shader_mix_shader_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Float>("Fac").default_value(0.5f).min(0.0f).max(1.0f).subtype(PROP_FACTOR);
  b.add_input<decl::Shader>("Shader");
  b.add_input<decl::Shader>("Shader", "Shader_001");
  b.add_output<decl::Shader>("Shader");
}

static int node_shader_gpu_mix_shader(GPUMaterial *mat,
                                      bNode *node,
                                      bNodeExecData * /*execdata*/,
                                      GPUNodeStack *in,
                                      GPUNodeStack *out)
{
  return GPU_stack_link(mat, node, "node_mix_shader", in, out);
}

NODE_SHADER_MATERIALX_BEGIN
#ifdef WITH_MATERIALX
{
  NodeItem res = empty();
  switch (to_type_) {
    case NodeItem::Type::BSDF:
    case NodeItem::Type::EDF: {
      NodeItem fac = get_input_value(0, NodeItem::Type::Float);
      NodeItem shader1 = get_input_link(1, to_type_);
      NodeItem shader2 = get_input_link(2, to_type_);

      if (shader1 && !shader2) {
        res = shader1 * (val(1.0f) - fac);
      }
      else if (!shader1 && shader2) {
        res = shader2 * fac;
      }
      else if (shader1 && shader2) {
        res = create_node("mix", to_type_);
        res.set_input("fg", shader1);
        res.set_input("bg", shader2);
        res.set_input("mix", fac);
      }
      break;
    }
    case NodeItem::Type::SurfaceShader: {
      /* SurfaceShaders can't be mixed, returning the first one connected */
      res = get_input_link(1, NodeItem::Type::SurfaceShader);
      if (!res) {
        res = get_input_link(2, NodeItem::Type::SurfaceShader);
      }
      break;
    }
    default:
      BLI_assert_unreachable();
  }
  return res;
}
#endif
NODE_SHADER_MATERIALX_END

}  // namespace blender::nodes::node_shader_mix_shader_cc

/* node type definition */
void register_node_type_sh_mix_shader()
{
  namespace file_ns = blender::nodes::node_shader_mix_shader_cc;

  static bNodeType ntype;

  sh_node_type_base(&ntype, SH_NODE_MIX_SHADER, "Mix Shader", NODE_CLASS_SHADER);
  ntype.declare = file_ns::node_declare;
  ntype.gpu_fn = file_ns::node_shader_gpu_mix_shader;
  ntype.materialx_fn = file_ns::node_shader_materialx;

  nodeRegisterType(&ntype);
}
