/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edinterface
 */

#include "UI_interface.h"

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
  }

  void build_row(uiLayout &row) override
  {
    add_label(row);

    uiLayout *sub = uiLayoutRow(&row, true);
    uiLayoutSetPropDecorate(sub, false);
  }

 private:
  bNodeTree &nodetree_;
  bNodeSocket &socket_;
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

 private:
  //  int get_state_icon() const
  //  {
  //    /* TODO(sergey): Use proper icons. */
  //    switch (collection_light_linking_.link_state) {
  //      case COLLECTION_LIGHT_LINKING_STATE_INCLUDE:
  //        return ICON_OUTLINER_OB_LIGHT;
  //      case COLLECTION_LIGHT_LINKING_STATE_EXCLUDE:
  //        return ICON_LIGHT;
  //    }
  //    BLI_assert_unreachable();
  //    return ICON_NONE;
  //  }

  //  static void link_state_toggle_cb(bContext * /*C*/,
  //                                   void * /*collection_v*/,
  //                                   void *collection_light_linking_v)
  //  {
  //    CollectionLightLinking &collection_light_linking = *static_cast<CollectionLightLinking *>(
  //        collection_light_linking_v);

  //    switch (collection_light_linking.link_state) {
  //      case COLLECTION_LIGHT_LINKING_STATE_INCLUDE:
  //        collection_light_linking.link_state = COLLECTION_LIGHT_LINKING_STATE_EXCLUDE;
  //        return;
  //      case COLLECTION_LIGHT_LINKING_STATE_EXCLUDE:
  //        collection_light_linking.link_state = COLLECTION_LIGHT_LINKING_STATE_INCLUDE;
  //        return;
  //    }

  //    BLI_assert_unreachable();
  //  }

  //  void build_state_button(uiLayout &row)
  //  {
  //    uiBlock *block = uiLayoutGetBlock(&row);
  //    const int icon = get_state_icon();

  //    PointerRNA collection_light_linking_ptr;
  //    RNA_pointer_create(&collection_.id,
  //                       &RNA_CollectionLightLinking,
  //                       &collection_light_linking_,
  //                       &collection_light_linking_ptr);

  //    uiBut *button = uiDefIconButR(block,
  //                                  UI_BTYPE_BUT,
  //                                  0,
  //                                  icon,
  //                                  0,
  //                                  0,
  //                                  UI_UNIT_X,
  //                                  UI_UNIT_Y,
  //                                  &collection_light_linking_ptr,
  //                                  "link_state",
  //                                  0,
  //                                  0.0f,
  //                                  0.0f,
  //                                  0.0f,
  //                                  0.0f,
  //                                  nullptr);

  //    UI_but_func_set(button, link_state_toggle_cb, &collection_, &collection_light_linking_);
  //  }

  //  void build_remove_button(uiLayout &row)
  //  {
  //    PointerRNA id_ptr;
  //    RNA_id_pointer_create(id_, &id_ptr);

  //    PointerRNA collection_ptr;
  //    RNA_id_pointer_create(&collection_.id, &collection_ptr);

  //    uiLayoutSetContextPointer(&row, "id", &id_ptr);
  //    uiLayoutSetContextPointer(&row, "collection", &collection_ptr);

  //    uiItemO(&row, "", ICON_X, "OBJECT_OT_light_linking_unlink_from_collection");
  //  }

  bNodeTree &nodetree_;
  bNodePanel &panel_;
};

class NodeTreeDeclarationView : public AbstractTreeView {
 public:
  explicit NodeTreeDeclarationView(bNodeTree &nodetree) : nodetree_(nodetree) {}

  void build_tree() override
  {
    //    LISTBASE_FOREACH (CollectionChild *, collection_child, &collection_.children) {
    //      Collection *child_collection = collection_child->collection;
    //      add_tree_item<CollectionViewItem>(collection_,
    //                                        child_collection->id,
    //                                        collection_child->light_linking,
    //                                        ICON_OUTLINER_COLLECTION);
    //    }

    //    LISTBASE_FOREACH (CollectionObject *, collection_object, &collection_.gobject) {
    //      Object *child_object = collection_object->ob;
    //      add_tree_item<CollectionViewItem>(
    //          collection_, child_object->id, collection_object->light_linking, ICON_OBJECT_DATA);
    //    }
    for (bNodePanel *panel : nodetree_.panels_for_write()) {
      add_tree_item<NodePanelViewItem>(nodetree_, *panel);
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
