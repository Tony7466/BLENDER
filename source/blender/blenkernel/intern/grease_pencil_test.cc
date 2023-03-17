/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2021 Blender Foundation. */

#include "testing/testing.h"

#include "BKE_grease_pencil.hh"

using namespace blender::bke::gpencil;

namespace blender::bke::gpencil::tests {

TEST(gpencil, build_layer_tree)
{
  LayerGroup root{};

  LayerGroup group("Group1");
  group.add_layer(Layer("Layer1"));
  group.add_layer(Layer("Layer2"));

  LayerGroup group2("Group2");
  group2.add_layer(Layer("Layer3"));
  group2.add_layer(Layer("Layer4"));

  group.add_group(std::move(group2));
  root.add_group(std::move(group));
  root.add_layer(Layer("Layer5"));

  root.foreach_children_pre_order([](TreeNode &child) { std::cout << child.name << std::endl; });
}

}  // namespace blender::bke::gpencil::tests