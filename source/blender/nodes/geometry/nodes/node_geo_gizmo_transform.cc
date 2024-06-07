/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "NOD_rna_define.hh"

#include "RNA_enum_types.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

namespace blender::nodes::node_geo_gizmo_transform_cc {

NODE_STORAGE_FUNCS(NodeGeometryTransformGizmo)

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Matrix>("Value").multi_input();
  b.add_input<decl::Matrix>("Base");
  b.add_output<decl::Geometry>("Transform");
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometryTransformGizmo *storage = MEM_cnew<NodeGeometryTransformGizmo>(__func__);
  node->storage = storage;
}

static void node_layout(uiLayout * /*layout*/, bContext * /*C*/, PointerRNA * /*ptr*/) {}

static void node_rna(StructRNA * /*srna*/) {}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, GEO_NODE_GIZMO_TRANSFORM, "Transform Gizmo", NODE_CLASS_INTERFACE);
  bke::node_type_storage(&ntype,
                         "NodeGeometryTransformGizmo",
                         node_free_standard_storage,
                         node_copy_standard_storage);
  ntype.declare = node_declare;
  ntype.draw_buttons = node_layout;
  ntype.initfunc = node_init;
  bke::nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_gizmo_transform_cc
