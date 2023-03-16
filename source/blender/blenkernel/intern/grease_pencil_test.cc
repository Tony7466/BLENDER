/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2021 Blender Foundation. */

#include "testing/testing.h"

#include "BKE_grease_pencil.hh"

using namespace blender::bke::gpencil;

namespace blender::bke::gpencil::tests {

TEST(gpencil, build_layer_tree)
{
  TreeNode root{};
  
  TreeNode node("Node1");
  node.add_child(TreeNode("Child1"));
  node.add_child(TreeNode("Child2"));
  
  root.add_child(std::move(node));
  root.add_child(TreeNode("Node2"));

  for (TreeNode &node : root.children_in_pre_order()) {
    std::cout << node.name << std::endl;
  }
}

}  // namespace blender::bke::gpencil::tests