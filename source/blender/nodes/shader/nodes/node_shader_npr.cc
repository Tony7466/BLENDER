/* SPDX-FileCopyrightText: 2005 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup shdnodes
 */

#include "node_shader_util.hh"
#include "node_util.hh"

#include "RNA_access.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

namespace blender::nodes::node_shader_npr_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Shader>("Shader");
  /* TODO(NPR): Remove. */
  b.add_input<decl::Float>("Weight").unavailable();
  /* TODO(NPR): Output should have its own socket type. */
  b.add_output<decl::Shader>("NPR");
}

static void node_shader_buts_npr(uiLayout *layout, bContext *C, PointerRNA *ptr)
{
  uiTemplateID(layout, C, ptr, "nprtree", "render.npr_new", nullptr, nullptr, 0, false, nullptr);
}

static void node_shader_buts_npr_ex(uiLayout *layout, bContext *C, PointerRNA *ptr)
{
  uiItemS(layout);

  node_shader_buts_npr(layout, C, ptr);
}

static int node_shader_fn(GPUMaterial *mat,
                          bNode *node,
                          bNodeExecData * /*execdata*/,
                          GPUNodeStack *in,
                          GPUNodeStack *out)
{
  GPU_material_flag_set(mat, GPU_MATFLAG_NPR);
  return GPU_stack_link(mat, node, "npr_passthrough", in, out);
}

}  // namespace blender::nodes::node_shader_npr_cc

void register_node_type_sh_npr()
{
  namespace file_ns = blender::nodes::node_shader_npr_cc;

  static blender::bke::bNodeType ntype;

  sh_node_type_base(&ntype, SH_NODE_NPR, "NPR", NODE_CLASS_SHADER);
  ntype.declare = file_ns::node_declare;
  ntype.add_ui_poll = object_eevee_shader_nodes_poll;
  ntype.draw_buttons = file_ns::node_shader_buts_npr;
  ntype.draw_buttons_ex = file_ns::node_shader_buts_npr_ex;
  ntype.gpu_fn = file_ns::node_shader_fn;

  blender::bke::node_register_type(&ntype);
}
