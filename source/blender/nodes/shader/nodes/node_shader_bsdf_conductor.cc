/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_shader_util.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

namespace blender::nodes::node_shader_bsdf_conductor_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Color>("Color").default_value({0.8f, 0.8f, 0.8f, 1.0f});
  b.add_input<decl::Color>("Tint").default_value({1.0f, 1.0f, 1.0f, 1.0f});
  b.add_input<decl::Float>("Roughness")
      .default_value(0.5f)
      .min(0.0f)
      .max(1.0f)
      .subtype(PROP_FACTOR);
  b.add_input<decl::Float>("Anisotropy")
      .default_value(0.0f)
      .min(0.0f)
      .max(1.0f)
      .subtype(PROP_FACTOR);
  b.add_input<decl::Float>("Rotation")
      .default_value(0.0f)
      .min(0.0f)
      .max(1.0f)
      .subtype(PROP_FACTOR);
  b.add_input<decl::Vector>("Normal").hide_value();
  b.add_input<decl::Vector>("Tangent").hide_value();
  b.add_input<decl::Float>("Weight").unavailable();
  b.add_output<decl::Shader>("BSDF");
}

static void node_shader_buts_conductor(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "distribution", UI_ITEM_R_SPLIT_EMPTY_NAME, "", ICON_NONE);
}

static void node_shader_init_conductor(bNodeTree * /*ntree*/, bNode *node)
{
  node->custom1 = SHD_GLOSSY_MULTI_GGX;
}

static int node_shader_gpu_bsdf_conductor(GPUMaterial *mat,
                                          bNode *node,
                                          bNodeExecData * /*execdata*/,
                                          GPUNodeStack *in,
                                          GPUNodeStack *out)
{
  if (!in[5].link) {
    GPU_link(mat, "world_normals_get", &in[5].link);
  }

  GPU_material_flag_set(mat, GPU_MATFLAG_GLOSSY);

  float use_multi_scatter = (node->custom1 == SHD_GLOSSY_MULTI_GGX) ? 1.0f : 0.0f;

  return GPU_stack_link(
      mat, node, "node_bsdf_conductor", in, out, GPU_constant(&use_multi_scatter));
}

NODE_SHADER_MATERIALX_BEGIN
#ifdef WITH_MATERIALX
{
  if (to_type_ != NodeItem::Type::BSDF) {
    return empty();
  }

  NodeItem color = get_input_value("Color", NodeItem::Type::Color3);
  NodeItem tint = get_input_value("Tint", NodeItem::Type::Color3);
  NodeItem roughness = get_input_value("Roughness", NodeItem::Type::Vector2);
  NodeItem anisotropy = get_input_value("Anisotropy", NodeItem::Type::Color3);
  NodeItem normal = get_input_link("Normal", NodeItem::Type::Vector3);
  NodeItem tangent = get_input_link("Tangent", NodeItem::Type::Vector3);

  NodeItem artistic_ior = create_node("artistic_ior",
                                      NodeItem::Type::Multioutput,
                                      {{"reflectivity", color}, {"edge_color", tint}});
  NodeItem ior_out = artistic_ior.add_output("ior", NodeItem::Type::Color3);
  NodeItem extinction_out = artistic_ior.add_output("extinction", NodeItem::Type::Color3);

  return create_node("conductor_bsdf",
                     NodeItem::Type::BSDF,
                     {{"normal", normal},
                      {"tangent", tangent},
                      {"ior", ior_out},
                      {"extinction", extinction_out},
                      {"roughness", roughness}});
}
#endif
NODE_SHADER_MATERIALX_END

}  // namespace blender::nodes::node_shader_bsdf_conductor_cc

/* node type definition */
void register_node_type_sh_bsdf_conductor()
{
  namespace file_ns = blender::nodes::node_shader_bsdf_conductor_cc;

  static bNodeType ntype;

  sh_node_type_base(&ntype, SH_NODE_BSDF_CONDUCTOR, "Conductor BSDF", NODE_CLASS_SHADER);
  ntype.declare = file_ns::node_declare;
  ntype.add_ui_poll = object_shader_nodes_poll;
  ntype.draw_buttons = file_ns::node_shader_buts_conductor;
  blender::bke::node_type_size_preset(&ntype, blender::bke::eNodeSizePreset::MIDDLE);
  ntype.initfunc = file_ns::node_shader_init_conductor;
  ntype.gpu_fn = file_ns::node_shader_gpu_bsdf_conductor;
  ntype.materialx_fn = file_ns::node_shader_materialx;

  nodeRegisterType(&ntype);
}
