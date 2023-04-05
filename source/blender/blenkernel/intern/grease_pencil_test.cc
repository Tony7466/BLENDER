/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2021 Blender Foundation. */

#include "testing/testing.h"

#include "BKE_grease_pencil.hh"
#include "BKE_idtype.h"
#include "BKE_lib_id.h"
#include "BKE_main.h"

using namespace blender::bke::gpencil;

namespace blender::bke::gpencil::tests {

/* --------------------------------------------------------------------------------------------- */
/* Grease Pencil ID Tests. */

struct GreasePencilIDTestContext {
  Main *bmain = nullptr;

  GreasePencilIDTestContext()
  {
    BKE_idtype_init();
    bmain = BKE_main_new();
  }
  ~GreasePencilIDTestContext()
  {
    BKE_main_free(bmain);
  }
};

TEST(gpencil, create_grease_pencil_id)
{
  GreasePencilIDTestContext ctx;

  GreasePencil *grease_pencil = static_cast<GreasePencil *>(BKE_id_new(ctx.bmain, ID_GP, "GP"));
  EXPECT_EQ(grease_pencil->drawings().size(), 0);
  EXPECT_EQ(grease_pencil->root_group().total_num_children(), 0);
}

/* --------------------------------------------------------------------------------------------- */
/* Layer Tree Tests. */

TEST(gpencil, layer_tree_empty)
{
  LayerGroup root{};
}

TEST(gpencil, layer_tree_build_simple)
{
  LayerGroup root{};

  LayerGroup group("Group1");
  group.add_layer(Layer("Layer1"));
  group.add_layer(Layer("Layer2"));
  root.add_group(std::move(group));
}

struct GreasePencilLayerTreeExample {
  StringRefNull names[7] = {"Group1", "Layer1", "Layer2", "Group2", "Layer3", "Layer4", "Layer5"};
  const bool is_layer[7] = {false, true, true, false, true, true, true};
  LayerGroup root;

  GreasePencilLayerTreeExample()
  {
    LayerGroup group(names[0]);
    group.add_layer(Layer(names[1]));
    group.add_layer(Layer(names[2]));

    LayerGroup group2(names[3]);
    group2.add_layer(Layer(names[4]));
    group2.add_layer(Layer(names[5]));

    group.add_group(std::move(group2));
    root.add_group(std::move(group));
    root.add_layer(Layer(names[6]));
  }
};

TEST(gpencil, layer_tree_pre_order_iteration_callback)
{
  GreasePencilLayerTreeExample ex;
  int i = 0;
  ex.root.foreach_children_pre_order(
      [&](TreeNode &child) { EXPECT_EQ(child.name, ex.names[i++]); });
}

TEST(gpencil, layer_tree_pre_order_iteration_loop)
{
  GreasePencilLayerTreeExample ex;
  int i = 0;
  for (TreeNode &child : ex.root.children_in_pre_order()) {
    EXPECT_EQ(child.name, ex.names[i++]);
  }
}

TEST(gpencil, layer_tree_total_size)
{
  GreasePencilLayerTreeExample ex;
  EXPECT_EQ(ex.root.total_num_children(), 7);
}

TEST(gpencil, layer_tree_node_types)
{
  GreasePencilLayerTreeExample ex;
  int i = 0;
  for (TreeNode &child : ex.root.children_in_pre_order()) {
    EXPECT_EQ(child.is_layer(), ex.is_layer[i]);
    EXPECT_EQ(child.is_group(), !ex.is_layer[i]);
    i++;
  }
}

}  // namespace blender::bke::gpencil::tests