/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edinterface
 */

#include "BKE_context.hh"
#include "ANIM_bone_collections.hh"

#include "BLT_translation.h"

#include "UI_interface.hh"
#include "UI_tree_view.hh"


namespace blender::ui::bonecollections {

using namespace blender::animrig;

class BoneCollectionItem : public AbstractTreeViewItem {
 public:
  BoneCollectionItem(BoneCollection &bone_collection, BoneCollectionMember &member)
      : bone_collection_(bone_collection), member_(member)
  {
    this->label_ = member.bone->name;
  }


  bool supports_collapsing() const
  {
    return false;
  }

  std::optional<bool> should_be_active() const override
  {

    return {};
  }

  void on_activate()
  {
    
  }

  bool supports_renaming() const override
  {
    return true;
  }

  // bool rename(StringRefNull new_name) override
  // {

  //   return false;
  // }

  StringRef get_rename_string() const override
  {
    return member_.bone->name;
  }

 private:
  BoneCollection &bone_collection_;
  BoneCollectionMember &member_;
};

// TODO do we support bone collection groups? Nested groups? Should they be called groups? 
// Will need _some_ representation of "TreeViewItem" for parent of bone hierarchy. 
// class BoneCollectionGroupViewItem : public AbstractTreeViewItem {
//  public:
//   BoneCollectionGroupViewItem(BoneCollection &bone_collection, BoneCollectionMember &bone_group)
//       : bone_collection_(bone_collection), group_bone_(bone_group)
//   {
//     this->label_ = bone_group.name();
//   }

//   void build_row(uiLayout &row) override
//   {
//     uiLayout *sub = uiLayoutRow(&row, true);
//     uiItemL(sub, IFACE_(group_.name().c_str()), ICON_FILE_FOLDER);
//   }

//  private:
//   BoneCollection &bone_collection_;
//   BoneCollectionMember &bone_group;
// };

class BoneCollectionTreeView : public AbstractTreeView {
 public:
  explicit BoneCollectionTreeView(BoneCollection &bone_collection) : bone_collection_(bone_collection) {}

  void build_tree() override;

 private:
  void build_tree_node_recursive();
  BoneCollection &bone_collection_;
};

void BoneCollectionTreeView::build_tree_node_recursive()
{
  // using namespace blender::animrig
}

void BoneCollectionTreeView::build_tree()
{

}

}  // namespace blender::ui::bonecollection

void uiTemplateBoneCollectionTree(uiLayout *layout, bContext *C)
{
  using namespace blender;

  Object *object = CTX_data_active_object(C);
  if (!object || object->type != OB_ARMATURE) {
    return;
  }
  BoneCollection &bone_collection = *static_cast<BoneCollection *>(object->data);

  uiBlock *block = uiLayoutGetBlock(layout);

  ui::AbstractTreeView *tree_view = UI_block_add_view(
      *block,
      "Bone Collection Tree View",
      std::make_unique<blender::ui::bonecollections::BoneCollectionTreeView>(bone_collection));
  tree_view->set_min_rows(3);

  ui::TreeViewBuilder::build_tree_view(*tree_view, *layout);
}