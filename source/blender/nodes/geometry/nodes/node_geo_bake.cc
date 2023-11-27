/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "NOD_rna_define.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_bake_cc {

static void node_declare(NodeDeclarationBuilder &b) {}

static void node_layout(uiLayout *layout, bContext *C, PointerRNA *ptr)
{
  UNUSED_VARS(layout, C, ptr);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  UNUSED_VARS(node);
}

static void node_rna(StructRNA *srna)
{
  UNUSED_VARS(srna);
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_BAKE, "Bake", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.draw_buttons = node_layout;
  ntype.initfunc = node_init;
  nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_bake_cc
