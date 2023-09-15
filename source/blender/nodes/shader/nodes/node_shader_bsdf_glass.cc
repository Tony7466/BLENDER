/* SPDX-FileCopyrightText: 2005 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_shader_util.hh"

namespace blender::nodes::node_shader_bsdf_glass_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Color>("Color").default_value({1.0f, 1.0f, 1.0f, 1.0f});
  b.add_input<decl::Float>("Roughness")
      .default_value(0.0f)
      .min(0.0f)
      .max(1.0f)
      .subtype(PROP_FACTOR);
  b.add_input<decl::Float>("IOR").default_value(1.45f).min(0.0f).max(1000.0f);
  b.add_input<decl::Vector>("Normal").hide_value();
  b.add_input<decl::Float>("Weight").unavailable();
  b.add_output<decl::Shader>("BSDF");
}

static void node_shader_init_glass(bNodeTree * /*ntree*/, bNode *node)
{
  node->custom1 = SHD_GLOSSY_MULTI_GGX;
}

static int node_shader_gpu_bsdf_glass(GPUMaterial *mat,
                                      bNode *node,
                                      bNodeExecData * /*execdata*/,
                                      GPUNodeStack *in,
                                      GPUNodeStack *out)
{
  if (!in[3].link) {
    GPU_link(mat, "world_normals_get", &in[3].link);
  }

  GPU_material_flag_set(mat, GPU_MATFLAG_GLOSSY | GPU_MATFLAG_REFRACT);

  float use_multi_scatter = (node->custom1 == SHD_GLOSSY_MULTI_GGX) ? 1.0f : 0.0f;

  return GPU_stack_link(mat, node, "node_bsdf_glass", in, out, GPU_constant(&use_multi_scatter));
}

NODE_SHADER_MATERIALX_BEGIN
#ifdef WITH_MATERIALX
{
  if (to_type_ != NodeItem::Type::BSDF) {
    return empty();
  }

  NodeItem color = get_input_value("Color", NodeItem::Type::Color3);
  NodeItem roughness = get_input_value("Roughness", NodeItem::Type::Vector2);
  NodeItem ior = get_input_value("IOR", NodeItem::Type::Float);
  NodeItem normal = get_input_link("Normal", NodeItem::Type::Vector3);

  NodeItem dielectric = create_node("dielectric_bsdf", NodeItem::Type::BSDF);
  if (normal) {
    dielectric.set_input("normal", normal);
  }
  dielectric.set_input("tint", color);
  dielectric.set_input("roughness", roughness);
  dielectric.set_input("ior", ior);
  dielectric.set_input("scatter_mode", val(std::string("RT")));

  NodeItem artistic_ior = create_node("artistic_ior", NodeItem::Type::Multioutput);
  artistic_ior.add_output("ior", NodeItem::Type::Color3);
  artistic_ior.add_output("extinction", NodeItem::Type::Color3);
  artistic_ior.set_input("reflectivity", color);
  artistic_ior.set_input("edge_color", color);

  NodeItem conductor = create_node("conductor_bsdf", NodeItem::Type::BSDF);
  if (normal) {
    conductor.set_input("normal", normal);
  }
  conductor.set_input_output("ior", artistic_ior, "ior");
  conductor.set_input_output("extinction", artistic_ior, "extinction");
  conductor.set_input("roughness", roughness);

  NodeItem res = create_node("mix", NodeItem::Type::BSDF);
  res.set_input("fg", dielectric);
  res.set_input("bg", conductor);
  res.set_input("mix", val(0.5f));

  return res ;
}
#endif
NODE_SHADER_MATERIALX_END

}  // namespace blender::nodes::node_shader_bsdf_glass_cc

/* node type definition */
void register_node_type_sh_bsdf_glass()
{
  namespace file_ns = blender::nodes::node_shader_bsdf_glass_cc;

  static bNodeType ntype;

  sh_node_type_base(&ntype, SH_NODE_BSDF_GLASS, "Glass BSDF", NODE_CLASS_SHADER);
  ntype.declare = file_ns::node_declare;
  ntype.add_ui_poll = object_shader_nodes_poll;
  blender::bke::node_type_size_preset(&ntype, blender::bke::eNodeSizePreset::MIDDLE);
  ntype.initfunc = file_ns::node_shader_init_glass;
  ntype.gpu_fn = file_ns::node_shader_gpu_bsdf_glass;
  ntype.materialx_fn = file_ns::node_shader_materialx;

  nodeRegisterType(&ntype);
}
