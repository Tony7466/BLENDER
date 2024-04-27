/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "NOD_socket_items.hh"

#include "WM_api.hh"

#include "BKE_context.hh"
#include "BKE_node_tree_update.hh"

#include "ED_node.hh"

namespace blender::nodes::socket_items {

template<typename Accessor>
inline void remove_item_operator(wmOperatorType *ot,
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

}  // namespace blender::nodes::socket_items
