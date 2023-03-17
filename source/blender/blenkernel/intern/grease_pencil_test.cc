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
  group.add_layer(Layer("Child1"));
  group.add_layer(Layer("Child2"));

  root.add_group(std::move(group));
  root.add_layer(Layer("Group2"));

  root.foreach_children_pre_order([](LayerGroup &child) { std::cout << child.name << std::endl; });

  // root.remove_child(0);

  // for (LayerGroup &child : root.children_in_pre_order()) {
  //   std::cout << child.name << std::endl;
  // }
}

}  // namespace blender::bke::gpencil::tests