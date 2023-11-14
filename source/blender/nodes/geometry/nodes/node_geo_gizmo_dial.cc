/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "NOD_rna_define.hh"

#include "RNA_enum_types.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

namespace blender::nodes::node_geo_gizmo_dial_cc {

NODE_STORAGE_FUNCS(NodeGeometryDialGizmo)

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Float>("Value").hide_value().multi_input();
  b.add_input<decl::Vector>("Position");
  b.add_input<decl::Vector>("Up").default_value({0, 0, 1});
  b.add_output<decl::Geometry>("Transform");
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometryArrowGizmo *storage = MEM_cnew<NodeGeometryArrowGizmo>(__func__);
  node->storage = storage;
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "color_id", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_rna(StructRNA *srna)
{
  RNA_def_node_enum(srna,
                    "color_id",
                    "Color",
                    "",
                    rna_enum_geometry_nodes_gizmo_color_items,
                    NOD_storage_enum_accessors(color_id));
}

static void node_register()
{
  static bNodeType ntype;
  geo_node_type_base(&ntype, GEO_NODE_GIZMO_DIAL, "Dial Gizmo", NODE_CLASS_INTERFACE);
  node_type_storage(
      &ntype, "NodeGeometryDialGizmo", node_free_standard_storage, node_copy_standard_storage);
  ntype.declare = node_declare;
  ntype.draw_buttons = node_layout;
  ntype.initfunc = node_init;
  nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_gizmo_dial_cc
