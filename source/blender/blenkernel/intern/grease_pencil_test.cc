/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2021 Blender Foundation. */

#include "testing/testing.h"

#include "BKE_curves.hh"
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

  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(BKE_id_new(ctx.bmain, ID_GP, "GP"));
  EXPECT_EQ(grease_pencil.drawings().size(), 0);
  EXPECT_EQ(grease_pencil.root_group().total_num_children(), 0);
}

TEST(gpencil, set_active_layer)
{
  GreasePencilIDTestContext ctx;

  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(BKE_id_new(ctx.bmain, ID_GP, "GP"));
  grease_pencil.add_empty_drawings(3);

  Layer layer1("Layer1");
  Layer layer2("Layer2");

  Layer *layer1_ref = &grease_pencil.root_group().add_layer(std::move(layer1));
  Layer *layer2_ref = &grease_pencil.root_group().add_layer(std::move(layer2));

  grease_pencil.runtime->set_active_layer(layer1_ref);
  EXPECT_EQ(layer1_ref, grease_pencil.active_layer());

  grease_pencil.runtime->set_active_layer(layer2_ref);
  EXPECT_EQ(layer2_ref, grease_pencil.active_layer());

  /* Save to storage. */
  grease_pencil.free_layer_tree_storage();
  grease_pencil.save_layer_tree_to_storage();
  MEM_delete(grease_pencil.runtime);

  /* Load from storage. */
  grease_pencil.runtime = MEM_new<blender::bke::GreasePencilRuntime>(__func__);
  grease_pencil.load_layer_tree_from_storage();

  /* Check if the active layer is still the second one. */
  EXPECT_NE(grease_pencil.active_layer(), nullptr);
  EXPECT_STREQ(grease_pencil.active_layer()->name, "Layer2");
}

/* --------------------------------------------------------------------------------------------- */
/* Drawing Array Tests. */

TEST(gpencil, add_empty_drawings)
{
  GreasePencilIDTestContext ctx;
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(BKE_id_new(ctx.bmain, ID_GP, "GP"));
  grease_pencil.add_empty_drawings(3);
  EXPECT_EQ(grease_pencil.drawings().size(), 3);
}

TEST(gpencil, remove_drawing)
{
  GreasePencilIDTestContext ctx;
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(BKE_id_new(ctx.bmain, ID_GP, "GP"));
  grease_pencil.add_empty_drawings(3);

  GreasePencilDrawing *drawing = reinterpret_cast<GreasePencilDrawing *>(
      grease_pencil.drawings_for_write()[1]);
  drawing->geometry.wrap().resize(0, 10);

  Layer layer1("Layer1");
  Layer layer2("Layer2");

  layer1.insert_frame(0, GreasePencilFrame{0});
  layer1.insert_frame(10, GreasePencilFrame{1});
  layer1.insert_frame(20, GreasePencilFrame{2});

  layer2.insert_frame(0, GreasePencilFrame{1});

  grease_pencil.root_group().add_layer(std::move(layer1));
  grease_pencil.root_group().add_layer(std::move(layer2));

  grease_pencil.remove_drawing(1);
  EXPECT_EQ(grease_pencil.drawings().size(), 2);

  static int expected_frames_size[] = {2, 0};
  static int expected_frames_pairs_layer0[][2] = {{0, 0}, {20, 1}};
  int layer_i = 0;
  grease_pencil.root_group().foreach_layer_pre_order([&](Layer &layer) {
    EXPECT_EQ(layer.frames().size(), expected_frames_size[layer_i]);
    if (layer_i == 0) {
      for (const int i : IndexRange(2)) {
        EXPECT_EQ(layer.frames().lookup(expected_frames_pairs_layer0[i][0]).drawing_index,
                  expected_frames_pairs_layer0[i][1]);
      }
    }
    layer_i++;
  });
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