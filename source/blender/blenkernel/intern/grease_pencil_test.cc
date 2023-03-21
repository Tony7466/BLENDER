/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2021 Blender Foundation. */

#include "testing/testing.h"

#include "BKE_grease_pencil.hh"

using namespace blender::bke::gpencil;

namespace blender::bke::gpencil::tests {

class gpencil_LayerTreeTest : public testing::Test {
 protected:
  void SetUp() override
  {
    StringRefNull names[] = {"Group1", "Layer1", "Layer2", "Group2", "Layer3", "Layer4", "Layer5"};
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

  LayerGroup root;
};

TEST(gpencil, empty_layer_group)
{
  LayerGroup root{};
}

TEST(gpencil, build_simple_tree)
{
  LayerGroup root{};

  LayerGroup group("Group1");
  group.add_layer(Layer("Layer1"));
  group.add_layer(Layer("Layer2"));
  root.add_group(std::move(group));
}

TEST_F(gpencil_LayerTreeTest, pre_order_iteration1)
{
  StringRefNull names[] = {"Group1", "Layer1", "Layer2", "Group2", "Layer3", "Layer4", "Layer5"};
  int i = 0;
  root.foreach_children_pre_order([&](TreeNode &child) { EXPECT_EQ(child.name, names[i++]); });
}

TEST_F(gpencil_LayerTreeTest, pre_order_iteration2)
{
  StringRefNull names[] = {"Group1", "Layer1", "Layer2", "Group2", "Layer3", "Layer4", "Layer5"};
  int i = 0;
  for (TreeNode &child : root.children_in_pre_order()) {
    EXPECT_EQ(child.name, names[i++]);
  }
}

TEST_F(gpencil_LayerTreeTest, layer_tree_total_size)
{
  EXPECT_EQ(root.total_num_children(), 7);
}

TEST_F(gpencil_LayerTreeTest, layer_tree_node_types)
{
  const bool is_layer[] = {false, true, true, false, true, true, true};
  int i = 0;
  for (TreeNode &child : root.children_in_pre_order()) {
    EXPECT_EQ(child.is_layer(), is_layer[i]);
    EXPECT_EQ(child.is_group(), !is_layer[i]);
    i++;
  }
}

}  // namespace blender::bke::gpencil::tests