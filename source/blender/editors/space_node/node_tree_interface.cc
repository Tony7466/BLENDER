/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spnode
 */

#include "BKE_context.h"

#include "ED_screen.h"

#include "RNA_access.h"
#include "RNA_define.h"
#include "RNA_enum_types.h"
#include "RNA_prototypes.h"

#include "WM_api.h"

namespace blender::ed::space_node {

/* -------------------------------------------------------------------- */
/** \name Node Tree Interface Operators
 * \{ */

static int ntree_interface_socket_new_exec(bContext *C, wmOperator *op)
{
  //  SpaceNode *snode = CTX_wm_space_node(C);
  //  bNodeTree *ntree = snode->edittree;
  //  bNodeTreeInterface &interface = ntree->interface;

  //  char name_buf[MAX_NAME];
  //  const char *name = RNA_string_get_alloc(op->ptr, "name", name_buf, sizeof(name_buf),
  //  nullptr); char type_buf[MAX_NAME]; const char *type = RNA_string_get_alloc(op->ptr, "type",
  //  type_buf, sizeof(type_buf), nullptr); const eNodeSocketDeclarationInOut in_out =
  //  eNodeSocketDeclarationInOut(
  //      RNA_enum_get(op->ptr, "in_out"));

  //  const bNodeSocketDeclaration *new_socket = interface.insert_socket(
  //      name, "", type, in_out, interface.active_item);
  //  if (new_socket) {
  //    //    interface.active_item = new_socket.index
  //  }

  //  //  ED_node_tree_propagate_change(C, CTX_data_main(C), snode->edittree);

  //  WM_event_add_notifier(C, NC_NODE | ND_DISPLAY, nullptr);

  return OPERATOR_FINISHED;
}

void NODE_OT_tree_interface_socket_new(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Add Node Tree Interface Socket";
  ot->description = "Add a socket to the node tree interface";
  ot->idname = "NODE_OT_tree_interface_socket_new";

  /* api callbacks */
  ot->exec = ntree_interface_socket_new_exec;
  ot->poll = ED_operator_node_editable;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  RNA_def_string(ot->srna, "name", "Input", 0, "Name", "");
  RNA_def_string(ot->srna, "type", "Type", 0, "Type", "");
  RNA_def_enum_flag(ot->srna,
                    "in_out",
                    rna_enum_node_socket_declaration_in_out_items,
                    SOCKDECL_IN,
                    "Socket Kind",
                    "");
}

static int ntree_interface_panel_new_exec(bContext *C, wmOperator *op)
{
  return OPERATOR_FINISHED;
}

void NODE_OT_tree_interface_panel_new(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Add Node Tree Interface Panel";
  ot->description = "Add a panel to the node tree interface";
  ot->idname = "NODE_OT_tree_interface_panel_new";

  /* api callbacks */
  ot->exec = ntree_interface_panel_new_exec;
  ot->poll = ED_operator_node_editable;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  RNA_def_string(ot->srna, "name", "Input", 0, "Name", "");
}

static int ntree_interface_item_remove_exec(bContext *C, wmOperator *op)
{
  return OPERATOR_FINISHED;
}

void NODE_OT_tree_interface_item_remove(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Remove Node Tree Interface Item";
  ot->description = "Remove the active item from the node tree interface";
  ot->idname = "NODE_OT_tree_interface_item_remove";

  /* api callbacks */
  ot->exec = ntree_interface_item_remove_exec;
  ot->poll = ED_operator_node_editable;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

static int ntree_interface_item_move_exec(bContext *C, wmOperator *op)
{
  return OPERATOR_FINISHED;
}

void NODE_OT_tree_interface_item_move(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Move Node Tree Interface Item";
  ot->description = "Move the active item in the node tree interface";
  ot->idname = "NODE_OT_tree_interface_item_move";

  /* api callbacks */
  ot->exec = ntree_interface_item_move_exec;
  ot->poll = ED_operator_node_editable;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

/** \} */

}  // namespace blender::ed::space_node
