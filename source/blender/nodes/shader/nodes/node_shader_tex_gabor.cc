/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_texture.h"

#include "node_shader_util.hh"
#include "node_util.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

namespace blender::nodes::node_shader_tex_gabor_cc {

NODE_STORAGE_FUNCS(NodeTexGabor)

static void sh_node_tex_gabor_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Vector>("Vector").implicit_field(implicit_field_inputs::position);
  b.add_input<decl::Float>("Scale").default_value(6.4f);
  b.add_input<decl::Float>("Impulses").default_value(64.0f);
  b.add_input<decl::Float>("Orientation").default_value(0.7f).subtype(PROP_ANGLE);
  b.add_input<decl::Float>("Frequency").default_value(1.25f);
  b.add_input<decl::Float>("Anisotropy")
      .default_value(1.0f)
      .min(0.0f)
      .max(1.0f)
      .subtype(PROP_FACTOR);
  b.add_output<decl::Float>("Value");
}

static void node_shader_buts_tex_gabor(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiLayout *split = uiLayoutSplit(layout, 0.33f, true);
  uiItemR(split, ptr, "gabor_type", UI_ITEM_R_SPLIT_EMPTY_NAME, "", ICON_NONE);
}

static void node_shader_init_tex_gabor(bNodeTree * /*ntree*/, bNode *node)
{
  NodeTexGabor *storage = MEM_cnew<NodeTexGabor>(__func__);
  BKE_texture_mapping_default(&storage->base.tex_mapping, TEXMAP_TYPE_POINT);
  BKE_texture_colormapping_default(&storage->base.color_mapping);

  storage->type = SHD_GABOR_TYPE_2D;

  node->storage = storage;
}

static int node_shader_gpu_tex_gabor(GPUMaterial *material,
                                     bNode *node,
                                     bNodeExecData * /* execdata */,
                                     GPUNodeStack *in,
                                     GPUNodeStack *out)
{
  node_shader_gpu_default_tex_coord(material, node, &in[0].link);
  node_shader_gpu_tex_mapping(material, node, in, out);

  const float type = float(node_storage(*node).type);
  return GPU_stack_link(material, node, "node_tex_gabor", in, out, GPU_constant(&type));
}

}  // namespace blender::nodes::node_shader_tex_gabor_cc

void register_node_type_sh_tex_gabor()
{
  namespace file_ns = blender::nodes::node_shader_tex_gabor_cc;

  static bNodeType ntype;

  sh_node_type_base(&ntype, SH_NODE_TEX_GABOR, "Gabor Texture", NODE_CLASS_TEXTURE);
  ntype.declare = file_ns::sh_node_tex_gabor_declare;
  ntype.draw_buttons = file_ns::node_shader_buts_tex_gabor;
  ntype.initfunc = file_ns::node_shader_init_tex_gabor;
  node_type_storage(
      &ntype, "NodeTexGabor", node_free_standard_storage, node_copy_standard_storage);
  ntype.gpu_fn = file_ns::node_shader_gpu_tex_gabor;

  nodeRegisterType(&ntype);
}
