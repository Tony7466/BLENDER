/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_collection_types.h"
#include "DNA_space_types.h"

#include "BKE_context.hh"
#include "BKE_global.hh"
#include "BKE_lib_id.hh"
#include "BKE_main.hh"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"
#include "BKE_report.hh"

#include "ED_node.hh"
#include "ED_render.hh"
#include "ED_screen.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"

#include "DEG_depsgraph_build.hh"

#include "node_intern.hh"

namespace blender::ed::space_node {

struct NodeClipboardItemIDInfo {
  ID *id;
  std::string id_name;
  /* Set but not used? */
  std::string library_name;
};

struct NodeClipboardItem {
  bNode *node;
  /**
   * The offset and size of the node from when it was drawn. Stored here since it doesn't remain
   * valid for the nodes in the clipboard.
   */
  rctf draw_rect;

  /* Extra info to validate the node on creation. Otherwise we may reference missing data. */
  NodeClipboardItemIDInfo id_info;
  Map<bNodeSocket *, NodeClipboardItemIDInfo> id_info_sockets;
};

struct NodeClipboard {
  Vector<NodeClipboardItem> nodes;
  Vector<bNodeLink> links;

  void clear()
  {
    for (NodeClipboardItem &item : this->nodes) {
      bke::node_free_node(nullptr, item.node);
    }
    this->nodes.clear_and_shrink();
    this->links.clear_and_shrink();
  }

  /**
   * Replace node IDs that are no longer available in the current file. Return false when one or
   * more IDs are lost.
   */
  bool validate()
  {
    bool ok = true;

    for (NodeClipboardItem &item : this->nodes) {
      bNode &node = *item.node;
      /* Reassign each loop since we may clear, open a new file where the ID is valid, and paste
       * again. */
      node.id = item.id_info.id;

      if (node.id) {
        const ListBase *lb = which_libbase(G_MAIN, GS(item.id_info.id_name.c_str()));
        if (BLI_findindex(lb, item.id_info.id) == -1) {
          /* May assign null. */
          node.id = static_cast<ID *>(
              BLI_findstring(lb, item.id_info.id_name.c_str() + 2, offsetof(ID, name) + 2));
          if (!node.id) {
            ok = false;
          }
        }
      }

      /* We might also have IDs referenced in sockets e.g. SOCK_OBJECT. */
      LISTBASE_FOREACH (bNodeSocket *, socket, &node.inputs) {
        if (!item.id_info_sockets.contains(socket)) {
          continue;
        }

        NodeClipboardItemIDInfo id_info_socket = item.id_info_sockets.lookup(socket);
        const ListBase *lb = which_libbase(G_MAIN, GS(id_info_socket.id_name.c_str()));
        ID *id_to_use = id_info_socket.id;
        if (BLI_findindex(lb, id_to_use) == -1) {
          id_to_use = reinterpret_cast<ID *>(
              BLI_findstring(lb, id_info_socket.id_name.c_str() + 2, offsetof(ID, name) + 2));
          if (!id_to_use) {
            ok = false;
          }
        }

        switch (eNodeSocketDatatype(socket->type)) {
          case SOCK_OBJECT: {
            bNodeSocketValueObject &default_value =
                *socket->default_value_typed<bNodeSocketValueObject>();
            default_value.value = reinterpret_cast<Object *>(id_to_use);
            break;
          }
          case SOCK_IMAGE: {
            bNodeSocketValueImage &default_value =
                *socket->default_value_typed<bNodeSocketValueImage>();
            default_value.value = reinterpret_cast<Image *>(id_to_use);
            break;
          }
          case SOCK_COLLECTION: {
            bNodeSocketValueCollection &default_value =
                *socket->default_value_typed<bNodeSocketValueCollection>();
            default_value.value = reinterpret_cast<Collection *>(id_to_use);
            break;
          }
          case SOCK_TEXTURE: {
            bNodeSocketValueTexture &default_value =
                *socket->default_value_typed<bNodeSocketValueTexture>();
            default_value.value = reinterpret_cast<Tex *>(id_to_use);
            break;
          }
          case SOCK_MATERIAL: {
            bNodeSocketValueMaterial &default_value =
                *socket->default_value_typed<bNodeSocketValueMaterial>();
            default_value.value = reinterpret_cast<Material *>(id_to_use);
            break;
          }
          case SOCK_FLOAT:
          case SOCK_VECTOR:
          case SOCK_RGBA:
          case SOCK_BOOLEAN:
          case SOCK_ROTATION:
          case SOCK_MATRIX:
          case SOCK_INT:
          case SOCK_STRING:
          case SOCK_CUSTOM:
          case SOCK_SHADER:
          case SOCK_GEOMETRY:
          case SOCK_MENU:
            break;
        }
      }
    }

    return ok;
  }

  void add_node(const bNode &node,
                Map<const bNode *, bNode *> &node_map,
                Map<const bNodeSocket *, bNodeSocket *> &socket_map)
  {
    /* No ID reference-counting, this node is virtual,
     * detached from any actual Blender data currently. */
    bNode *new_node = bke::node_copy_with_mapping(
        nullptr, node, LIB_ID_CREATE_NO_USER_REFCOUNT | LIB_ID_CREATE_NO_MAIN, false, socket_map);
    node_map.add_new(&node, new_node);

    NodeClipboardItem item;
    item.draw_rect = node.runtime->totr;
    item.node = new_node;
    item.id_info.id = new_node->id;
    if (item.id_info.id) {
      item.id_info.id_name = new_node->id->name;
      if (ID_IS_LINKED(new_node->id)) {
        item.id_info.library_name = new_node->id->lib->runtime.filepath_abs;
      }
    }

    /* We might also have IDs in sockets e.g. SOCK_OBJECT, see socket_id_user_increment() */
    LISTBASE_FOREACH (bNodeSocket *, socket, &new_node->inputs) {
      switch (eNodeSocketDatatype(socket->type)) {
        case SOCK_OBJECT: {
          bNodeSocketValueObject &default_value =
              *socket->default_value_typed<bNodeSocketValueObject>();
          NodeClipboardItemIDInfo id_info_socket;
          id_info_socket.id = &default_value.value->id;
          id_info_socket.id_name = id_info_socket.id->name;
          item.id_info_sockets.add(socket, id_info_socket);
          break;
        }
        case SOCK_IMAGE: {
          bNodeSocketValueImage &default_value =
              *socket->default_value_typed<bNodeSocketValueImage>();
          NodeClipboardItemIDInfo id_info_socket;
          id_info_socket.id = &default_value.value->id;
          id_info_socket.id_name = id_info_socket.id->name;
          item.id_info_sockets.add(socket, id_info_socket);
          break;
        }
        case SOCK_COLLECTION: {
          bNodeSocketValueCollection &default_value =
              *socket->default_value_typed<bNodeSocketValueCollection>();
          NodeClipboardItemIDInfo id_info_socket;
          id_info_socket.id = &default_value.value->id;
          id_info_socket.id_name = id_info_socket.id->name;
          item.id_info_sockets.add(socket, id_info_socket);
          break;
        }
        case SOCK_TEXTURE: {
          bNodeSocketValueTexture &default_value =
              *socket->default_value_typed<bNodeSocketValueTexture>();
          NodeClipboardItemIDInfo id_info_socket;
          id_info_socket.id = &default_value.value->id;
          id_info_socket.id_name = id_info_socket.id->name;
          item.id_info_sockets.add(socket, id_info_socket);
          break;
        }
        case SOCK_MATERIAL: {
          bNodeSocketValueMaterial &default_value =
              *socket->default_value_typed<bNodeSocketValueMaterial>();
          NodeClipboardItemIDInfo id_info_socket;
          id_info_socket.id = &default_value.value->id;
          id_info_socket.id_name = id_info_socket.id->name;
          item.id_info_sockets.add(socket, id_info_socket);
          break;
        }
        case SOCK_FLOAT:
        case SOCK_VECTOR:
        case SOCK_RGBA:
        case SOCK_BOOLEAN:
        case SOCK_ROTATION:
        case SOCK_MATRIX:
        case SOCK_INT:
        case SOCK_STRING:
        case SOCK_CUSTOM:
        case SOCK_SHADER:
        case SOCK_GEOMETRY:
        case SOCK_MENU:
          break;
      }
    }

    this->nodes.append(std::move(item));
  }
};

static NodeClipboard &get_node_clipboard()
{
  static NodeClipboard clipboard;
  return clipboard;
}

/* -------------------------------------------------------------------- */
/** \name Copy
 * \{ */

static int node_clipboard_copy_exec(bContext *C, wmOperator * /*op*/)
{
  SpaceNode &snode = *CTX_wm_space_node(C);
  bNodeTree &tree = *snode.edittree;
  NodeClipboard &clipboard = get_node_clipboard();

  clipboard.clear();

  Map<const bNode *, bNode *> node_map;
  Map<const bNodeSocket *, bNodeSocket *> socket_map;

  for (const bNode *node : tree.all_nodes()) {
    if (node->flag & SELECT) {
      clipboard.add_node(*node, node_map, socket_map);
    }
  }

  for (bNode *new_node : node_map.values()) {
    /* Parent pointer must be redirected to new node or detached if parent is not copied. */
    if (new_node->parent) {
      if (node_map.contains(new_node->parent)) {
        new_node->parent = node_map.lookup(new_node->parent);
      }
      else {
        nodeDetachNode(&tree, new_node);
      }
    }
  }

  /* Copy links between selected nodes. */
  LISTBASE_FOREACH (bNodeLink *, link, &tree.links) {
    BLI_assert(link->tonode);
    BLI_assert(link->fromnode);
    if (link->tonode->flag & NODE_SELECT && link->fromnode->flag & NODE_SELECT) {
      bNodeLink new_link{};
      new_link.flag = link->flag;
      new_link.tonode = node_map.lookup(link->tonode);
      new_link.tosock = socket_map.lookup(link->tosock);
      new_link.fromnode = node_map.lookup(link->fromnode);
      new_link.fromsock = socket_map.lookup(link->fromsock);
      new_link.multi_input_sort_id = link->multi_input_sort_id;
      clipboard.links.append(new_link);
    }
  }

  return OPERATOR_FINISHED;
}

void NODE_OT_clipboard_copy(wmOperatorType *ot)
{
  ot->name = "Copy to Clipboard";
  ot->description = "Copy the selected nodes to the internal clipboard";
  ot->idname = "NODE_OT_clipboard_copy";

  ot->exec = node_clipboard_copy_exec;
  ot->poll = ED_operator_node_active;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Paste
 * \{ */

static int node_clipboard_paste_exec(bContext *C, wmOperator *op)
{
  SpaceNode &snode = *CTX_wm_space_node(C);
  bNodeTree &tree = *snode.edittree;
  NodeClipboard &clipboard = get_node_clipboard();

  const bool is_valid = clipboard.validate();

  if (clipboard.nodes.is_empty()) {
    BKE_report(op->reports, RPT_ERROR, "The internal clipboard is empty");
    return OPERATOR_CANCELLED;
  }

  if (!is_valid) {
    BKE_report(op->reports,
               RPT_WARNING,
               "Some nodes references could not be restored, will be left empty");
  }

  ED_preview_kill_jobs(CTX_wm_manager(C), CTX_data_main(C));

  node_deselect_all(tree);

  Map<const bNode *, bNode *> node_map;
  Map<const bNodeSocket *, bNodeSocket *> socket_map;

  /* copy valid nodes from clipboard */
  for (NodeClipboardItem &item : clipboard.nodes) {
    const bNode &node = *item.node;
    const char *disabled_hint = nullptr;
    if (node.typeinfo->poll_instance && node.typeinfo->poll_instance(&node, &tree, &disabled_hint))
    {
      bNode *new_node = bke::node_copy_with_mapping(
          &tree, node, LIB_ID_COPY_DEFAULT, true, socket_map);
      /* Reset socket shape in case a node is copied to a different tree type. */
      LISTBASE_FOREACH (bNodeSocket *, socket, &new_node->inputs) {
        socket->display_shape = SOCK_DISPLAY_SHAPE_CIRCLE;
      }
      LISTBASE_FOREACH (bNodeSocket *, socket, &new_node->outputs) {
        socket->display_shape = SOCK_DISPLAY_SHAPE_CIRCLE;
      }
      node_map.add_new(&node, new_node);
    }
    else {
      if (disabled_hint) {
        BKE_reportf(op->reports,
                    RPT_ERROR,
                    "Cannot add node %s into node tree %s: %s",
                    node.name,
                    tree.id.name + 2,
                    disabled_hint);
      }
      else {
        BKE_reportf(op->reports,
                    RPT_ERROR,
                    "Cannot add node %s into node tree %s",
                    node.name,
                    tree.id.name + 2);
      }
    }
  }

  for (bNode *new_node : node_map.values()) {
    nodeSetSelected(new_node, true);

    new_node->flag &= ~NODE_ACTIVE;

    /* The parent pointer must be redirected to new node. */
    if (new_node->parent) {
      if (node_map.contains(new_node->parent)) {
        new_node->parent = node_map.lookup(new_node->parent);
      }
    }
  }

  PropertyRNA *offset_prop = RNA_struct_find_property(op->ptr, "offset");
  if (RNA_property_is_set(op->ptr, offset_prop)) {
    float2 center(0);
    for (NodeClipboardItem &item : clipboard.nodes) {
      center.x += BLI_rctf_cent_x(&item.draw_rect);
      center.y += BLI_rctf_cent_y(&item.draw_rect);
    }
    /* DPI factor needs to be removed when computing a View2D offset from drawing rects. */
    center /= clipboard.nodes.size();

    float2 mouse_location;
    RNA_property_float_get_array(op->ptr, offset_prop, mouse_location);
    const float2 offset = (mouse_location - center) / UI_SCALE_FAC;

    for (bNode *new_node : node_map.values()) {
      /* Skip the offset for parented nodes since the location is in parent space. */
      if (new_node->parent == nullptr) {
        new_node->locx += offset.x;
        new_node->locy += offset.y;
      }
    }
  }

  /* Add links between existing nodes. */
  for (const bNodeLink &link : clipboard.links) {
    const bNode *fromnode = link.fromnode;
    const bNode *tonode = link.tonode;
    if (node_map.lookup_key_ptr(fromnode) && node_map.lookup_key_ptr(tonode)) {
      bNodeLink *new_link = nodeAddLink(&tree,
                                        node_map.lookup(fromnode),
                                        socket_map.lookup(link.fromsock),
                                        node_map.lookup(tonode),
                                        socket_map.lookup(link.tosock));
      new_link->multi_input_sort_id = link.multi_input_sort_id;
    }
  }

  for (bNode *new_node : node_map.values()) {
    bke::nodeDeclarationEnsure(&tree, new_node);
  }

  remap_node_pairing(tree, node_map);

  tree.ensure_topology_cache();
  for (bNode *new_node : node_map.values()) {
    /* Update multi input socket indices in case all connected nodes weren't copied. */
    update_multi_input_indices_for_removed_links(*new_node);
  }

  Main *bmain = CTX_data_main(C);
  ED_node_tree_propagate_change(C, bmain, &tree);
  /* Pasting nodes can create arbitrary new relations because nodes can reference IDs. */
  DEG_relations_tag_update(bmain);

  return OPERATOR_FINISHED;
}

static int node_clipboard_paste_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{
  const ARegion *region = CTX_wm_region(C);
  float2 cursor;
  UI_view2d_region_to_view(&region->v2d, event->mval[0], event->mval[1], &cursor.x, &cursor.y);
  RNA_float_set_array(op->ptr, "offset", cursor);
  return node_clipboard_paste_exec(C, op);
}

void NODE_OT_clipboard_paste(wmOperatorType *ot)
{
  ot->name = "Paste from Clipboard";
  ot->description = "Paste nodes from the internal clipboard to the active node tree";
  ot->idname = "NODE_OT_clipboard_paste";

  ot->invoke = node_clipboard_paste_invoke;
  ot->exec = node_clipboard_paste_exec;
  ot->poll = ED_operator_node_editable;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  PropertyRNA *prop = RNA_def_float_array(
      ot->srna,
      "offset",
      2,
      nullptr,
      -FLT_MAX,
      FLT_MAX,
      "Location",
      "The 2D view location for the center of the new nodes, or unchanged if not set",
      -FLT_MAX,
      FLT_MAX);
  RNA_def_property_flag(prop, PROP_SKIP_SAVE);
  RNA_def_property_flag(prop, PROP_HIDDEN);
}

/** \} */

}  // namespace blender::ed::space_node

void ED_node_clipboard_free()
{
  using namespace blender::ed::space_node;
  NodeClipboard &clipboard = get_node_clipboard();
  clipboard.validate();
  clipboard.clear();
}
