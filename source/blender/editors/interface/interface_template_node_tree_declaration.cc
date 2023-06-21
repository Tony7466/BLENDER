/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edinterface
 */

#include "UI_interface.h"

#include "BKE_context.h"
#include "BKE_node_runtime.hh"
#include "BKE_node_tree_update.h"

#include "BLI_color.hh"

#include "BLT_translation.h"

#include "DNA_node_types.h"

#include "ED_node.h"

#include "RNA_access.h"
#include "RNA_prototypes.h"

#include "UI_interface.h"
#include "UI_interface.hh"
#include "UI_resources.h"
#include "UI_tree_view.hh"

#include "WM_api.h"

namespace blender::ui::nodes {

namespace {

class NodeTreeDeclarationView;

enum eNodeTreeDeclarationType {
  /* Item is a node panel declaration. */
  NODE_PANEL,
  /* iem is a socket declaration. */
  NODE_SOCKET,
};

class NodeTreeDeclarationDragController : public AbstractViewItemDragController {
 public:
  explicit NodeTreeDeclarationDragController(NodeTreeDeclarationView &view, bNodeSocket *socket);
  explicit NodeTreeDeclarationDragController(NodeTreeDeclarationView &view, bNodePanel *panel);
  virtual ~NodeTreeDeclarationDragController() = default;

  eWM_DragDataType get_drag_type() const;

  void *create_drag_data() const;

 private:
  eNodeTreeDeclarationType type_;
  void *item_;
};

class NodeGroupSocketDropTarget : public AbstractViewItemDropTarget {
 public:
  explicit NodeGroupSocketDropTarget(NodeTreeDeclarationView &view, bNodeSocket &socket);

  bool can_drop(const wmDrag &drag, const char **r_disabled_hint) const override;
  std::string drop_tooltip(const wmDrag &drag) const override;
  bool on_drop(bContext *C, const wmDrag &drag) const override;

 protected:
  wmDragNodeTreeDeclaration *get_drag_node_tree_declaration(const wmDrag &drag) const;

 private:
  bNodeSocket &socket_;
};

class NodePanelDropTarget : public AbstractViewItemDropTarget {
 public:
  explicit NodePanelDropTarget(NodeTreeDeclarationView &view, bNodePanel &panel);

  bool can_drop(const wmDrag &drag, const char **r_disabled_hint) const override;
  std::string drop_tooltip(const wmDrag &drag) const override;
  bool on_drop(bContext *C, const wmDrag &drag) const override;

 protected:
  wmDragNodeTreeDeclaration *get_drag_node_tree_declaration(const wmDrag &drag) const;

 private:
  bNodePanel &panel_;
};

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

  bool supports_renaming() const override
  {
    return true;
  }
  bool rename(const bContext &C, StringRefNull new_name) override
  {
    BLI_strncpy(socket_.name, new_name.c_str(), sizeof(socket_.name));
    BKE_ntree_update_tag_interface(&nodetree_);
    ED_node_tree_propagate_change(&C, CTX_data_main(&C), &nodetree_);
    return true;
  }
  StringRef get_rename_string() const override
  {
    return socket_.name;
  }

  std::unique_ptr<AbstractViewItemDragController> create_drag_controller() const override;
  std::unique_ptr<AbstractViewItemDropTarget> create_drop_target() override;

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

  bool supports_renaming() const override
  {
    return true;
  }
  bool rename(const bContext &C, StringRefNull new_name) override
  {
    panel_.name = BLI_strdup(new_name.c_str());
    BKE_ntree_update_tag_interface(&nodetree_);
    ED_node_tree_propagate_change(&C, CTX_data_main(&C), &nodetree_);
    return true;
  }
  StringRef get_rename_string() const override
  {
    return panel_.name;
  }

  std::unique_ptr<AbstractViewItemDragController> create_drag_controller() const override;
  std::unique_ptr<AbstractViewItemDropTarget> create_drop_target() override;

 private:
  bNodeTree &nodetree_;
  bNodePanel &panel_;
};

class NodeTreeDeclarationView : public AbstractTreeView {
 public:
  explicit NodeTreeDeclarationView(bNodeTree &nodetree) : nodetree_(nodetree) {}

  bNodeTree &nodetree()
  {
    return nodetree_;
  }

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

std::unique_ptr<AbstractViewItemDragController> NodeGroupSocketViewItem::create_drag_controller()
    const
{
  return std::make_unique<NodeTreeDeclarationDragController>(
      static_cast<NodeTreeDeclarationView &>(get_tree_view()), &socket_);
}

std::unique_ptr<AbstractViewItemDropTarget> NodeGroupSocketViewItem::create_drop_target()
{
  return std::make_unique<NodeGroupSocketDropTarget>(
      static_cast<NodeTreeDeclarationView &>(get_tree_view()), socket_);
}

std::unique_ptr<AbstractViewItemDragController> NodePanelViewItem::create_drag_controller() const
{
  return std::make_unique<NodeTreeDeclarationDragController>(
      static_cast<NodeTreeDeclarationView &>(get_tree_view()), &panel_);
}

std::unique_ptr<AbstractViewItemDropTarget> NodePanelViewItem::create_drop_target()
{
  return std::make_unique<NodePanelDropTarget>(
      static_cast<NodeTreeDeclarationView &>(get_tree_view()), panel_);
}

NodeTreeDeclarationDragController::NodeTreeDeclarationDragController(NodeTreeDeclarationView &view,
                                                                     bNodeSocket *socket)
    : AbstractViewItemDragController(view),
      type_(eNodeTreeDeclarationType::NODE_SOCKET),
      item_(socket)
{
}

NodeTreeDeclarationDragController::NodeTreeDeclarationDragController(NodeTreeDeclarationView &view,
                                                                     bNodePanel *panel)
    : AbstractViewItemDragController(view),
      type_(eNodeTreeDeclarationType::NODE_PANEL),
      item_(panel)
{
}

eWM_DragDataType NodeTreeDeclarationDragController::get_drag_type() const
{
  return WM_DRAG_NODE_TREE_DECLARATION;
}

void *NodeTreeDeclarationDragController::create_drag_data() const
{
  wmDragNodeTreeDeclaration *drag_data = MEM_cnew<wmDragNodeTreeDeclaration>(__func__);
  drag_data->type = type_;
  drag_data->item = item_;
  return drag_data;
}

NodeGroupSocketDropTarget::NodeGroupSocketDropTarget(NodeTreeDeclarationView &view,
                                                     bNodeSocket &socket)
    : AbstractViewItemDropTarget(view), socket_(socket)
{
}

bool NodeGroupSocketDropTarget::can_drop(const wmDrag &drag,
                                         const char ** /*r_disabled_hint*/) const
{
  if (drag.type != WM_DRAG_NODE_TREE_DECLARATION) {
    return false;
  }
  wmDragNodeTreeDeclaration *drag_data = get_drag_node_tree_declaration(drag);
  switch (drag_data->type) {
    case eNodeTreeDeclarationType::NODE_PANEL:
      /* Can't insert panel before socket. */
      return false;
    case eNodeTreeDeclarationType::NODE_SOCKET:
      /* Socket will be inserted before the target socket. */
      return true;
  }
  return false;
}

std::string NodeGroupSocketDropTarget::drop_tooltip(const wmDrag & /*drag*/) const
{
  return N_("Insert before socket");
}

bool NodeGroupSocketDropTarget::on_drop(bContext *C, const wmDrag &drag) const
{
  wmDragNodeTreeDeclaration *drag_data = get_drag_node_tree_declaration(drag);
  BLI_assert(drag_data != nullptr);

  bNodeTree &nodetree = get_view<NodeTreeDeclarationView>().nodetree();

  switch (drag_data->type) {
    case eNodeTreeDeclarationType::NODE_PANEL: {
      BLI_assert_unreachable();
      break;
    }
    case eNodeTreeDeclarationType::NODE_SOCKET: {
      bNodeSocket *drag_socket = static_cast<bNodeSocket *>(drag_data->item);
      ntreeSetSocketInterfacePanel(&nodetree, drag_socket, socket_.panel);
      bNodeSocket *before;
      if (drag_socket->in_out == socket_.in_out) {
        before = &socket_;
      }
      else if (drag_socket->in_out == SOCK_IN) {
        /* Input dragged onto output, move to end of inputs list. */
        before = nullptr;
      }
      else {
        /* Output dragged onto input, move to beginning of outputs list. */
        before = static_cast<bNodeSocket *>(nodetree.outputs.first);
      }
      ntreeInsertSocketInterfaceBefore(&nodetree, drag_socket, before);
      break;
    }
  }

  ED_node_tree_propagate_change(C, CTX_data_main(C), &nodetree);
  return true;
}

wmDragNodeTreeDeclaration *NodeGroupSocketDropTarget::get_drag_node_tree_declaration(
    const wmDrag &drag) const
{
  BLI_assert(drag.type == WM_DRAG_NODE_TREE_DECLARATION);
  return static_cast<wmDragNodeTreeDeclaration *>(drag.poin);
}

NodePanelDropTarget::NodePanelDropTarget(NodeTreeDeclarationView &view, bNodePanel &panel)
    : AbstractViewItemDropTarget(view), panel_(panel)
{
}

bool NodePanelDropTarget::can_drop(const wmDrag &drag, const char ** /*r_disabled_hint*/) const
{
  if (drag.type != WM_DRAG_NODE_TREE_DECLARATION) {
    return false;
  }
  wmDragNodeTreeDeclaration *drag_data = get_drag_node_tree_declaration(drag);
  switch (drag_data->type) {
    case eNodeTreeDeclarationType::NODE_PANEL:
      /* Panel will be inserted before the target panel. */
      return true;
    case eNodeTreeDeclarationType::NODE_SOCKET:
      /* Socket will be inserted at the start of the panel. */
      return true;
  }
  return false;
}

std::string NodePanelDropTarget::drop_tooltip(const wmDrag &drag) const
{
  wmDragNodeTreeDeclaration *drag_data = get_drag_node_tree_declaration(drag);
  BLI_assert(drag_data != nullptr);

  switch (drag_data->type) {
    case eNodeTreeDeclarationType::NODE_PANEL:
      return N_("Insert before panel");
      break;
    case eNodeTreeDeclarationType::NODE_SOCKET:
      return N_("Insert socket into panel");
      break;
  }

  return "";
}

static bNodeSocket *find_first_socket_in_panel(const bNodeTree &nodetree,
                                               const bNodePanel &panel,
                                               eNodeSocketInOut in_out)
{
  switch (in_out) {
    case SOCK_IN:
      LISTBASE_FOREACH (bNodeSocket *, socket, &nodetree.inputs) {
        if (socket->panel == &panel) {
          return socket;
        }
      }
      break;
    case SOCK_OUT:
      LISTBASE_FOREACH (bNodeSocket *, socket, &nodetree.inputs) {
        if (socket->panel == &panel) {
          return socket;
        }
      }
      break;
  }
  return nullptr;
}

bool NodePanelDropTarget::on_drop(bContext *C, const wmDrag &drag) const
{
  wmDragNodeTreeDeclaration *drag_data = get_drag_node_tree_declaration(drag);
  BLI_assert(drag_data != nullptr);

  bNodeTree &nodetree = get_view<NodeTreeDeclarationView>().nodetree();

  switch (drag_data->type) {
    case eNodeTreeDeclarationType::NODE_PANEL: {
      bNodePanel *drag_panel = static_cast<bNodePanel *>(drag_data->item);
      ntreeInsertPanelBefore(&nodetree, drag_panel, &panel_);
      break;
    }
    case eNodeTreeDeclarationType::NODE_SOCKET: {
      bNodeSocket *drag_socket = static_cast<bNodeSocket *>(drag_data->item);

      bNodeSocket *before = find_first_socket_in_panel(
          nodetree, panel_, eNodeSocketInOut(drag_socket->in_out));
      ntreeSetSocketInterfacePanel(&nodetree, drag_socket, &panel_);
      ntreeInsertSocketInterfaceBefore(&nodetree, drag_socket, before);
      break;
    }
  }

  ED_node_tree_propagate_change(C, CTX_data_main(C), &nodetree);
  return true;
}

wmDragNodeTreeDeclaration *NodePanelDropTarget::get_drag_node_tree_declaration(
    const wmDrag &drag) const
{
  BLI_assert(drag.type == WM_DRAG_NODE_TREE_DECLARATION);
  return static_cast<wmDragNodeTreeDeclaration *>(drag.poin);
}

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
