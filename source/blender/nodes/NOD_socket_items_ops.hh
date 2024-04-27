/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "NOD_socket_items.hh"

#include "WM_api.hh"

#include "BKE_context.hh"
#include "BKE_node_tree_update.hh"

#include "ED_node.hh"

namespace blender::nodes::socket_items::ops {

template<typename Accessor>
inline void remove_item(wmOperatorType *ot,
                        const char *name,
                        const char *idname,
                        const char *description)
{
  ot->name = name;
  ot->idname = idname;
  ot->description = description;

  /* TODO: poll function */
  ot->exec = [](bContext *C, wmOperator * /*op*/) -> int {
    PointerRNA node_ptr = CTX_data_pointer_get(C, "active_node");
    if (node_ptr.data == nullptr) {
      return OPERATOR_CANCELLED;
    }
    bNodeTree *ntree = reinterpret_cast<bNodeTree *>(node_ptr.owner_id);
    bNode *node = static_cast<bNode *>(node_ptr.data);
    if (node->type != Accessor::node_type) {
      return OPERATOR_CANCELLED;
    }
    socket_items::SocketItemsRef ref = Accessor::get_items_from_node(*node);
    dna::array::remove_index(
        ref.items, ref.items_num, ref.active_index, *ref.active_index, Accessor::destruct_item);

    BKE_ntree_update_tag_node_property(ntree, node);
    ED_node_tree_propagate_change(nullptr, CTX_data_main(C), ntree);
    WM_main_add_notifier(NC_NODE | NA_EDITED, ntree);
    return OPERATOR_FINISHED;
  };
}

template<typename Accessor>
inline void add_item_with_name_and_type(wmOperatorType *ot,
                                        const char *name,
                                        const char *idname,
                                        const char *description)
{
  ot->name = name;
  ot->idname = idname;
  ot->description = description;

  ot->exec = [](bContext *C, wmOperator * /*op*/) -> int {
    PointerRNA node_ptr = CTX_data_pointer_get(C, "active_node");
    bNode *node = static_cast<bNode *>(node_ptr.data);
    if (node == nullptr) {
      return OPERATOR_CANCELLED;
    }
    if (node->type != Accessor::node_type) {
      return OPERATOR_CANCELLED;
    }
    socket_items::SocketItemsRef ref = Accessor::get_items_from_node(*node);
    const int old_active_index = *ref.active_index;

    eNodeSocketDatatype socket_type;
    std::string name;
    int dst_index;
    if (old_active_index >= 0 && old_active_index < *ref.items_num) {
      dst_index = old_active_index + 1;
      const typename Accessor::ItemT &active_item = (*ref.items)[old_active_index];
      socket_type = eNodeSocketDatatype(active_item.socket_type);
      name = active_item.name;
    }
    else {
      dst_index = *ref.items_num;
      socket_type = SOCK_GEOMETRY;
      /* Empty name so it is based on the type. */
      name = "";
    }
    socket_items::add_item_with_socket_and_name<Accessor>(*node, socket_type, name.c_str());
    dna::array::move_index(*ref.items, *ref.items_num, *ref.items_num - 1, dst_index);
    *ref.active_index = dst_index;

    bNodeTree *ntree = reinterpret_cast<bNodeTree *>(node_ptr.owner_id);
    BKE_ntree_update_tag_node_property(ntree, node);
    ED_node_tree_propagate_change(nullptr, CTX_data_main(C), ntree);
    WM_main_add_notifier(NC_NODE | NA_EDITED, ntree);
    return OPERATOR_FINISHED;
  };
}

}  // namespace blender::nodes::socket_items::ops
