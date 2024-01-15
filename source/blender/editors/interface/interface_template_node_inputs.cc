/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edinterface
 */

#include "BLI_vector.hh"

#include "BKE_context.hh"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"

#include "BLT_translation.h"

#include "NOD_node_declaration.hh"

#include "RNA_access.hh"
#include "RNA_prototypes.h"

#include "UI_interface.hh"
#include "UI_resources.hh"

/* -------------------------------------------------------------------- */
/** \name Node Input Buttons Template
 * \{ */

using blender::nodes::ItemDeclaration;
using blender::nodes::NodeDeclaration;
using blender::nodes::PanelDeclaration;
using blender::nodes::SocketDeclaration;

using ItemIterator = blender::Vector<blender::nodes::ItemDeclarationPtr>::const_iterator;

namespace blender::ui::nodes {

static void draw_node_input(bContext *C,
                            uiLayout *layout,
                            PointerRNA *node_ptr,
                            bNodeSocket &socket)
{
  BLI_assert(socket.typeinfo != nullptr);
  /* Ignore disabled sockets and linked sockets and sockets without a `draw` callback. */
  if (!socket.is_available() || (socket.flag & SOCK_IS_LINKED) || socket.typeinfo->draw == nullptr)
  {
    return;
  }

  PointerRNA socket_ptr = RNA_pointer_create(node_ptr->owner_id, &RNA_NodeSocket, &socket);
  const char *text = IFACE_(bke::nodeSocketLabel(&socket));
  socket.typeinfo->draw(C, layout, &socket_ptr, node_ptr, text);
}

static void draw_node_input(bContext *C,
                            uiLayout *layout,
                            PointerRNA *node_ptr,
                            StringRefNull identifier)
{
  bNode &node = *static_cast<bNode *>(node_ptr->data);
  bNodeSocket *socket = node.runtime->inputs_by_identifier.lookup(identifier);
  draw_node_input(C, layout, node_ptr, *socket);
}

static void draw_node_declaration_items(bContext *C,
                                        uiLayout *layout,
                                        PointerRNA *node_ptr,
                                        ItemIterator &item_iter,
                                        const ItemIterator item_end)
{
  while (item_iter != item_end) {
    const ItemDeclaration *item_decl = item_iter->get();
    ++item_iter;

    if (const SocketDeclaration *socket_decl = dynamic_cast<const SocketDeclaration *>(item_decl))
    {
      if (socket_decl->in_out == SOCK_IN) {
        draw_node_input(C, layout, node_ptr, socket_decl->identifier);
      }
    }
    else if (const PanelDeclaration *panel_decl = dynamic_cast<const PanelDeclaration *>(
                 item_decl))
    {
      /* Draw panel buttons at the top of each panel section. */
      if (panel_decl->draw_buttons) {
        panel_decl->draw_buttons(layout, C, node_ptr);
      }

      const ItemIterator panel_item_end = item_iter + panel_decl->num_child_decls;
      BLI_assert(panel_item_end <= item_end);
      draw_node_declaration_items(C, layout, node_ptr, item_iter, panel_item_end);
    }
  }
}

}  // namespace blender::ui::nodes

void uiTemplateNodeInputs(uiLayout *layout, bContext *C, PointerRNA *ptr)
{
  bNodeTree &tree = *reinterpret_cast<bNodeTree *>(ptr->owner_id);
  bNode &node = *static_cast<bNode *>(ptr->data);

  tree.ensure_topology_cache();

  BLI_assert(node.typeinfo != nullptr);
  /* Draw top-level node buttons. */
  if (node.typeinfo->draw_buttons_ex) {
    node.typeinfo->draw_buttons_ex(layout, C, ptr);
  }
  else if (node.typeinfo->draw_buttons) {
    node.typeinfo->draw_buttons(layout, C, ptr);
  }

  if (node.declaration()) {
    /* Draw socket inputs and panel buttons in the order of declaration panels. */
    ItemIterator item_iter = node.declaration()->items.begin();
    const ItemIterator item_end = node.declaration()->items.end();
    blender::ui::nodes::draw_node_declaration_items(C, layout, ptr, item_iter, item_end);
  }
  else {
    /* Draw socket values using the flat inputs list. */
    for (bNodeSocket *input : node.runtime->inputs) {
      blender::ui::nodes::draw_node_input(C, layout, ptr, *input);
    }
  }
}

/** \} */
