/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edinterface
 */

#include "UI_interface.h"

#include "BKE_context.h"
#include "BKE_node.hh"

#include "BLI_color.hh"

#include "BLT_translation.h"

#include "DNA_node_types.h"

#include "RNA_access.h"
#include "RNA_prototypes.h"

#include "UI_interface.h"
#include "UI_interface.hh"
#include "UI_resources.h"
#include "UI_tree_view.hh"

#include "WM_api.h"

namespace blender::ui::nodes {

namespace {

class NodeGroupSocketViewItem : public BasicTreeViewItem {
 public:
  NodeGroupSocketViewItem(bNodeTree &nodetree, bNodeSocket &socket)
      : BasicTreeViewItem(socket.name, ICON_NONE), nodetree_(nodetree), socket_(socket)
  {
    if (socket_.typeinfo->draw_color) {
      PointerRNA socket_ptr, node_ptr;
      RNA_pointer_create(&nodetree_.id, &RNA_NodeSocket, &socket_, &socket_ptr);
      RNA_pointer_create(&nodetree_.id, &RNA_Node, nullptr, &node_ptr);
      /* XXX we don't have a context but it's not commonly used by the draw_color function. */
      bContext *C = nullptr;
      socket_.typeinfo->draw_color(C, &socket_ptr, &node_ptr, color_);
    }
    else {
      color_ = ColorGeometry4f(1.0f, 0.0f, 1.0f, 1.0f);
    }
  }

  void build_row(uiLayout &row) override
  {
    uiLayoutSetPropDecorate(&row, false);

    uiLayout *input_socket_layout = uiLayoutRow(&row, true);
    if (socket_.in_out == SOCK_IN) {
      /* XXX Socket template only draws in embossed layouts (Julian). */
      uiLayoutSetEmboss(input_socket_layout, UI_EMBOSS);
      /* XXX Context is not used by the template function. */
      bContext *C = nullptr;
      uiTemplateNodeSocket(input_socket_layout, C, color_);
    }
    else {
      /* Blank item to align output socket labels with inputs. */
      uiItemL(input_socket_layout, "", ICON_BLANK1);
    }

    add_label(row);

    uiLayout *output_socket_layout = uiLayoutRow(&row, true);
    if (socket_.in_out == SOCK_OUT) {
      /* XXX Socket template only draws in embossed layouts (Julian). */
      uiLayoutSetEmboss(output_socket_layout, UI_EMBOSS);
      /* XXX Context is not used by the template function. */
      bContext *C = nullptr;
      uiTemplateNodeSocket(output_socket_layout, C, color_);
    }
    else {
      /* Blank item to align input socket labels with outputs. */
      uiItemL(output_socket_layout, "", ICON_BLANK1);
    }
  }

 protected:
  bool matches(const AbstractViewItem &other) const override
  {
    const NodeGroupSocketViewItem *other_panel_item =
        dynamic_cast<const NodeGroupSocketViewItem *>(&other);
    if (other_panel_item == nullptr) {
      return false;
    }

    return &socket_ == &other_panel_item->socket_;
  }

  std::optional<bool> should_be_active() const override
  {
    /* TODO There can only be one active item per tree view and we have 2 separate lists.
     * Let panels' active index take precedence, until we change the underlying data. */
    return nodetree_.active_panel < 0 && (socket_.flag & SELECT) != 0;
  }

  void on_activate() override
  {
    /* Make sure active panel is cleared to avoid conflicting active index. */
    nodetree_.active_panel = -1;

    LISTBASE_FOREACH (bNodeSocket *, tsocket, &nodetree_.inputs) {
      SET_FLAG_FROM_TEST(tsocket->flag, tsocket == &socket_, SELECT);
    }
    LISTBASE_FOREACH (bNodeSocket *, tsocket, &nodetree_.outputs) {
      SET_FLAG_FROM_TEST(tsocket->flag, tsocket == &socket_, SELECT);
    }
  }

 private:
  bNodeTree &nodetree_;
  bNodeSocket &socket_;
  ColorGeometry4f color_;
};

class NodePanelViewItem : public BasicTreeViewItem {
 public:
  NodePanelViewItem(bNodeTree &nodetree, bNodePanel &panel)
      : BasicTreeViewItem(panel.name, ICON_NONE), nodetree_(nodetree), panel_(panel)
  {
  }

  void build_row(uiLayout &row) override
  {
    add_label(row);

    uiLayout *sub = uiLayoutRow(&row, true);
    uiLayoutSetPropDecorate(sub, false);

    //    build_state_button(*sub);
    //    build_remove_button(*sub);
  }

 protected:
  bool matches(const AbstractViewItem &other) const override
  {
    const NodePanelViewItem *other_panel_item = dynamic_cast<const NodePanelViewItem *>(&other);
    if (other_panel_item == nullptr) {
      return false;
    }

    return &panel_ == &other_panel_item->panel_;
  }

  std::optional<bool> should_be_active() const override
  {
    const bNodePanel *active_panel = nodetree_.panels().get(nodetree_.active_panel, nullptr);
    return &panel_ == active_panel;
  }

  void on_activate() override
  {
    nodetree_.active_panel = nodetree_.panels().first_index_try(&panel_);

    /* Make sure active socket is cleared to avoid conflicting active index. */
    LISTBASE_FOREACH (bNodeSocket *, tsocket, &nodetree_.inputs) {
      SET_FLAG_FROM_TEST(tsocket->flag, false, SELECT);
    }
    LISTBASE_FOREACH (bNodeSocket *, tsocket, &nodetree_.outputs) {
      SET_FLAG_FROM_TEST(tsocket->flag, false, SELECT);
    }
  }

 private:
  bNodeTree &nodetree_;
  bNodePanel &panel_;
};

class NodeTreeDeclarationView : public AbstractTreeView {
 public:
  explicit NodeTreeDeclarationView(bNodeTree &nodetree) : nodetree_(nodetree) {}

  void build_tree() override
  {
    /* TODO there should be either a cached map for per-panel sockets
     * or a simple hierarchical struct to begin with, to avoid looping
     * over all sockets for every panel. */

    add_socket_items_for_panel(nullptr, *this);

    for (bNodePanel *panel : nodetree_.panels_for_write()) {
      NodePanelViewItem &panel_item = add_tree_item<NodePanelViewItem>(nodetree_, *panel);
      panel_item.set_collapsed(false);

      add_socket_items_for_panel(panel, panel_item);
    }
  }

 protected:
  void add_socket_items_for_panel(const bNodePanel *panel, ui::TreeViewOrItem &parent_item)
  {
    LISTBASE_FOREACH (bNodeSocket *, socket, &nodetree_.inputs) {
      if (socket->panel != panel) {
        continue;
      }

      parent_item.add_tree_item<NodeGroupSocketViewItem>(nodetree_, *socket);
    }
    LISTBASE_FOREACH (bNodeSocket *, socket, &nodetree_.outputs) {
      if (socket->panel != panel) {
        continue;
      }

      parent_item.add_tree_item<NodeGroupSocketViewItem>(nodetree_, *socket);
    }
  }

 private:
  bNodeTree &nodetree_;
};

}  // namespace

}  // namespace blender::ui::nodes

namespace ui = blender::ui;

void uiTemplateNodeTreeDeclaration(struct uiLayout *layout, struct PointerRNA *ptr)
{
  if (!ptr->data) {
    return;
  }
  if (!RNA_struct_is_a(ptr->type, &RNA_NodeTree)) {
    return;
  }
  bNodeTree &nodetree = *static_cast<bNodeTree *>(ptr->data);

  uiBlock *block = uiLayoutGetBlock(layout);

  ui::AbstractTreeView *tree_view = UI_block_add_view(
      *block,
      "Node Tree Declaration Tree View",
      std::make_unique<blender::ui::nodes::NodeTreeDeclarationView>(nodetree));
  tree_view->set_min_rows(3);

  ui::TreeViewBuilder::build_tree_view(*tree_view, *layout);
}
