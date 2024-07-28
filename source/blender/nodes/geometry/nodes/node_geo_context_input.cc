/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "NOD_rna_define.hh"

#include "BLI_string.h"

#include "RNA_enum_types.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_context_input_cc {

NODE_STORAGE_FUNCS(NodeGeometryContextInput)

static void node_declare(NodeDeclarationBuilder &b)
{
  const bNode *node = b.node_or_null();

  if (node != nullptr) {
    const eNodeSocketDatatype data_type = eNodeSocketDatatype(node_storage(*node).socket_type);
    b.add_output(data_type, "Value");
  }
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometryContextInput *data = MEM_cnew<NodeGeometryContextInput>(__func__);
  data->socket_type = SOCK_FLOAT;
  node->storage = data;
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiLayout *col = uiLayoutColumn(layout, false);
  uiItemR(col, ptr, "socket_type", UI_ITEM_NONE, "", ICON_NONE);
  uiItemR(col, ptr, "context_identifier", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_copy_storage(bNodeTree * /*dst_tree*/, bNode *dst_node, const bNode *src_node)
{
  const NodeGeometryContextInput &src_storage = node_storage(*src_node);
  auto *dst_storage = MEM_cnew<NodeGeometryContextInput>(__func__, src_storage);
  dst_storage->identifier = BLI_strdup_null(dst_storage->identifier);
  dst_node->storage = dst_storage;
}

static void node_free_storage(bNode *node)
{
  NodeGeometryContextInput &storage = node_storage(*node);
  MEM_SAFE_FREE(storage.identifier);
  MEM_freeN(&storage);
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_CONTEXT_INPUT, "Context Input", NODE_CLASS_INPUT);
  blender::bke::node_type_storage(&ntype, "Context Input", node_free_storage, node_copy_storage);
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.draw_buttons = node_layout;
  ntype.no_muting = true;
  blender::bke::nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_context_input_cc
