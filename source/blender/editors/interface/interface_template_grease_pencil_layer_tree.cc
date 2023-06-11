/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edinterface
 */

#include "BKE_context.h"
#include "BKE_grease_pencil.hh"

#include "BLT_translation.h"

#include "UI_interface.h"
#include "UI_interface.hh"
#include "UI_tree_view.hh"

namespace blender::ui::greasepencil {

using namespace blender::bke::greasepencil;

class LayerTreeViewItem : public AbstractTreeViewItem {
 public:
  LayerTreeViewItem(const TreeNode &node) : node_(node) {}

  void build_row(uiLayout &row) override
  {
    uiLayout *sub = uiLayoutRow(&row, true);
    uiItemS_ex(sub, 0.8f);
    uiItemL(sub, IFACE_(StringRefNull(node_.name).c_str()), ICON_NONE);
  }

 private:
  const TreeNode &node_;
};

class LayerTreeView : public AbstractTreeView {
 public:
  explicit LayerTreeView(GreasePencil &grease_pencil) : grease_pencil_(grease_pencil) {}

  void build_tree() override
  {
    using namespace blender::bke::greasepencil;
    for (const TreeNode *node : grease_pencil_.root_group.wrap().nodes()) {
      add_tree_item<LayerTreeViewItem>(*node);
    }
  }

 private:
  GreasePencil &grease_pencil_;
};

}  // namespace blender::ui::greasepencil

void uiTemplateGreasePencilLayerTree(uiLayout *layout, bContext *C)
{
  using namespace blender;

  Object *object = CTX_data_active_object(C);
  if (!object || object->type != OB_GREASE_PENCIL) {
    return;
  }
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object->data);

  uiBlock *block = uiLayoutGetBlock(layout);

  ui::AbstractTreeView *tree_view = UI_block_add_view(
      *block,
      "Grease Pencil Layer Tree View",
      std::make_unique<blender::ui::greasepencil::LayerTreeView>(grease_pencil));
  tree_view->set_min_rows(3);

  ui::TreeViewBuilder::build_tree_view(*tree_view, *layout);
}
