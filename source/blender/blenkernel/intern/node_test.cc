/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2022 Blender Foundation. */

#include "testing/testing.h"

#include "MEM_guardedalloc.h"

#include "BLI_listbase.h"
#include "BLI_string.h"

#include "BKE_callbacks.h"
#include "BKE_context.h"
#include "BKE_idtype.h"
#include "BKE_lib_id.h"
#include "BKE_main.h"
#include "BKE_main_namemap.h"
#include "BKE_node.h"
#include "BKE_node_runtime.hh"
#include "BKE_node_tree_update.h"

#include "DNA_ID.h"
#include "DNA_node_types.h"

#include "RNA_define.h"

#include "CLG_log.h"

namespace blender::bke::node::tests {

struct NodeTreeTestContext {
  Main *bmain = nullptr;
  bContext *C = nullptr;

  NodeTreeTestContext()
  {
    BKE_idtype_init();
    bmain = BKE_main_new();
    C = CTX_create();
  }
  ~NodeTreeTestContext()
  {
    CTX_free(C);
    BKE_main_free(bmain);
  }
};

class NodeTreeTest : public testing::Test {
 public:
  static void SetUpTestSuite()
  {
    CLG_init();
    BKE_callback_global_init();
    RNA_init();
    BKE_node_system_init();
  }
  static void TearDownTestSuite()
  {
    RNA_exit();
    BKE_node_system_exit();
    BKE_callback_global_finalize();
    CLG_exit();
  }

  void TearDown() override
  {
  }
};

/* Regression test: node groups with trees with undefined type could crash on update. */
TEST_F(NodeTreeTest, NT_undefined_node_group_update)
{
  NodeTreeTestContext ctx;

  /* Invalid idname that results in an "undefined" tree type. */
  bNodeTree *group_tree = ntreeAddTree(ctx.bmain, "GroupTree", "???");
  EXPECT_NE(group_tree, nullptr);
  EXPECT_EQ(group_tree->typeinfo, &NodeTreeTypeUndefined);

  /* Node tree with a node group linking to the undefined tree. */
  bNodeTree *node_tree = ntreeAddTree(ctx.bmain, "NodeTree", "GeometryNodeTree");
  EXPECT_NE(node_tree, nullptr);
  EXPECT_STREQ(node_tree->typeinfo->idname, "GeometryNodeTree");
  bNode *group_node = nodeAddNode(ctx.C, node_tree, "GeometryNodeGroup");
  EXPECT_NE(group_node, nullptr);
  EXPECT_STREQ(group_node->typeinfo->idname, "GeometryNodeGroup");
  group_node->id = &group_tree->id;

  EXPECT_NE(group_tree->runtime, nullptr);
  bNodeTreeRuntime *rt = static_cast<bNodeTreeRuntime *>(group_tree->runtime);
  BKE_ntree_update_tag_all(group_tree);
  EXPECT_NE(rt->changed_flag, 0);

  BKE_ntree_update_main(ctx.bmain, nullptr);
}

}  // namespace blender::bke::node::tests
