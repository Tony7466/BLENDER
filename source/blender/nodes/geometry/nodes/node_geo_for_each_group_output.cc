/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_compute_contexts.hh"
#include "BKE_scene.h"

#include "DEG_depsgraph_query.h"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "NOD_geometry.hh"
#include "NOD_socket.hh"

#include "BLI_string_utils.h"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_for_each_group_output_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Group Part");

  b.add_output<decl::Geometry>("Geometry").propagate_all();
}

static void node_register()
{
  static bNodeType ntype;
  geo_node_type_base(&ntype, GEO_NODE_FOR_EACH_GROUP_OUTPUT, "For Each Group Output", NODE_CLASS_INTERFACE);
  ntype.declare = node_declare;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::

/*
blender::Span<NodeRepeatItem> NodeGeometryForEachGroupOutput::items_span() const
{
  return blender::Span<NodeRepeatItem>(items, items_num);
}

blender::MutableSpan<NodeRepeatItem> NodeGeometryRepeatOutput::items_span()
{
  return blender::MutableSpan<NodeRepeatItem>(items, items_num);
}

bool NodeRepeatItem::supports_type(const eNodeSocketDatatype type)
{
  return ELEM(type,
              SOCK_FLOAT,
              SOCK_VECTOR,
              SOCK_RGBA,
              SOCK_BOOLEAN,
              SOCK_ROTATION,
              SOCK_INT,
              SOCK_STRING,
              SOCK_GEOMETRY,
              SOCK_OBJECT,
              SOCK_MATERIAL,
              SOCK_IMAGE,
              SOCK_COLLECTION);
}

std::string NodeRepeatItem::identifier_str() const
{
  return "Item_" + std::to_string(this->identifier);
}

NodeRepeatItem *NodeGeometryRepeatOutput::add_item(const char *name,
                                                   const eNodeSocketDatatype type)
{
  if (!NodeRepeatItem::supports_type(type)) {
    return nullptr;
  }
  const int insert_index = this->items_num;
  NodeRepeatItem *old_items = this->items;

  this->items = MEM_cnew_array<NodeRepeatItem>(this->items_num + 1, __func__);
  std::copy_n(old_items, insert_index, this->items);
  NodeRepeatItem &new_item = this->items[insert_index];
  std::copy_n(old_items + insert_index + 1,
              this->items_num - insert_index,
              this->items + insert_index + 1);

  new_item.identifier = this->next_identifier++;
  this->set_item_name(new_item, name);
  new_item.socket_type = type;

  this->items_num++;
  MEM_SAFE_FREE(old_items);
  return &new_item;
}

void NodeGeometryRepeatOutput::set_item_name(NodeRepeatItem &item, const char *name)
{
  char unique_name[MAX_NAME + 4];
  STRNCPY(unique_name, name);

  struct Args {
    NodeGeometryRepeatOutput *storage;
    const NodeRepeatItem *item;
  } args = {this, &item};

  const char *default_name = nodeStaticSocketLabel(item.socket_type, 0);
  BLI_uniquename_cb(
      [](void *arg, const char *name) {
        const Args &args = *static_cast<Args *>(arg);
        for (const NodeRepeatItem &item : args.storage->items_span()) {
          if (&item != args.item) {
            if (STREQ(item.name, name)) {
              return true;
            }
          }
        }
        return false;
      },
      &args,
      default_name,
      '.',
      unique_name,
      ARRAY_SIZE(unique_name));

  MEM_SAFE_FREE(item.name);
  item.name = BLI_strdup(unique_name);
}
*/