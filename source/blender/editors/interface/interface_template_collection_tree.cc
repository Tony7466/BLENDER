/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edinterface
 */

#include "DNA_collection_types.h"
#include "DNA_layer_types.h"
#include "DNA_scene_types.h"

#include "BKE_context.h"
#include "BKE_layer.h"

#include "RNA_access.hh"
#include "RNA_prototypes.h"

#include "UI_tree_view.hh"

namespace blender::ui {

class CollectionTreeView : public AbstractTreeView {
  Scene &scene_;
  ViewLayer &view_layer_;
  /** View3D from context, if any. Used for local overrides (local collections toggle). */
  View3D *view3d_ = nullptr;

  bool use_local_collections_ = false;
  bool show_viewport_visibility_ = false;

  friend class CollectionTreeViewItem;

 public:
  CollectionTreeView(Scene &scene, ViewLayer &view_layer, View3D *view3d)
      : scene_(scene), view_layer_(view_layer), view3d_(view3d)
  {
  }
  void build_tree() override;

  void set_use_local_collections(const bool use_local_collections)
  {
    use_local_collections_ = use_local_collections;
  }
  void enable_viewport_visibility_toggle()
  {
    show_viewport_visibility_ = true;
  }
};

/* ---------------------------------------------------------------------- */

class CollectionTreeViewItem : public BasicTreeViewItem {
  LayerCollection &collection_;
  LayerCollection &parent_collection_;

 public:
  CollectionTreeViewItem(LayerCollection &collection, LayerCollection &parent_collection)
      : BasicTreeViewItem(collection.collection->id.name + 2),
        collection_(collection),
        parent_collection_(parent_collection)
  {
  }

  void add_viewport_visibility_toggle(uiLayout &layout, PointerRNA &collection_ptr)
  {
    CollectionTreeView &view = static_cast<CollectionTreeView &>(get_tree_view());
    if (view.use_local_collections_ && view.view3d_) {
      const bool visible_in_viewport =
          ((view.view3d_->local_collections_uuid & collection_.local_collections_bits) &&
           !(parent_collection_.runtime_flag & LAYER_COLLECTION_HIDE_VIEWPORT));

      uiLayoutSetActive(&layout, visible_in_viewport);

      PointerRNA opptr;
      uiItemFullO(&layout,
                  "object.hide_collection",
                  "",
                  ICON_HIDE_OFF,
                  nullptr,
                  WM_OP_INVOKE_DEFAULT,
                  UI_ITEM_NONE,
                  &opptr);
      RNA_int_set(&opptr,
                  "collection_index",
                  BKE_layer_collection_findindex(&view.view_layer_, &collection_));
      RNA_boolean_set(&opptr, "toggle", true);
    }
    else {
      uiLayoutSetActive(&layout,
                        parent_collection_.runtime_flag & LAYER_COLLECTION_VISIBLE_VIEW_LAYER);
      uiItemR(&layout, &collection_ptr, "hide_viewport", UI_ITEM_R_COMPACT, "", ICON_HIDE_OFF);
    }
  }

  void build_row(uiLayout &row) override
  {
    add_label(row);

    uiLayout *sub = uiLayoutRow(&row, false);
    uiLayoutSetAlignment(sub, UI_LAYOUT_ALIGN_RIGHT);
    CollectionTreeView &view = static_cast<CollectionTreeView &>(get_tree_view());

    PointerRNA collection_ptr;
    RNA_pointer_create(&view.scene_.id, &RNA_LayerCollection, &collection_, &collection_ptr);

    if (view.show_viewport_visibility_) {
      add_viewport_visibility_toggle(*sub, collection_ptr);
    }
  }
};

/* ---------------------------------------------------------------------- */

static void add_children_recursive(TreeViewOrItem &parent_item, LayerCollection &parent_collection)
{
  LISTBASE_FOREACH (LayerCollection *, child, &parent_collection.layer_collections) {
    if (child->flag & LAYER_COLLECTION_EXCLUDE) {
      continue;
    }
    if (child->collection->flag & COLLECTION_HIDE_VIEWPORT) {
      continue;
    }

    AbstractTreeViewItem &new_item = parent_item.add_tree_item<CollectionTreeViewItem>(
        *child, parent_collection);
    add_children_recursive(new_item, *child);
  }
}

void CollectionTreeView::build_tree()
{
  LayerCollection &scene_collection = *static_cast<LayerCollection *>(
      view_layer_.layer_collections.first);
  add_children_recursive(*this, scene_collection);
}

}  // namespace blender::ui

/* ---------------------------------------------------------------------- */

using namespace blender;

void uiTemplateCollectionTree(uiLayout *layout, bContext *C)
{
  Scene *scene = CTX_data_scene(C);
  ViewLayer *view_layer = CTX_data_view_layer(C);
  View3D *view3d = CTX_wm_view3d(C);

  uiBlock *block = uiLayoutGetBlock(layout);

  std::unique_ptr collection_view = std::make_unique<ui::CollectionTreeView>(
      *scene, *view_layer, view3d);
  collection_view->set_min_rows(5);
  /* TODO: Pass the following as template options? */
  collection_view->set_use_local_collections(view3d && (view3d->flag & V3D_LOCAL_COLLECTIONS));
  collection_view->enable_viewport_visibility_toggle();

  ui::AbstractTreeView *tree_view = UI_block_add_view(
      *block, "Collection Tree View", std::move(collection_view));
  ui::TreeViewBuilder::build_tree_view(*tree_view, *layout);
}
