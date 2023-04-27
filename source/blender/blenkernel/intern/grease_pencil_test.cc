/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. */

#include "testing/testing.h"

#include "BKE_curves.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_idtype.h"
#include "BKE_lib_id.h"
#include "BKE_main.h"

using namespace blender::bke::greasepencil;

namespace blender::bke::greasepencil::tests {

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

TEST(greasepencil, create_grease_pencil_id)
{
  GreasePencilIDTestContext ctx;

  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(BKE_id_new(ctx.bmain, ID_GP, "GP"));
  EXPECT_EQ(grease_pencil.drawings().size(), 0);
  EXPECT_EQ(grease_pencil.root_group().num_children_total(), 0);
}

TEST(greasepencil, save_layer_tree_to_storage)
{
  GreasePencilIDTestContext ctx;
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(BKE_id_new(ctx.bmain, ID_GP, "GP"));

  StringRefNull names[7] = {"Group1", "Layer1", "Layer2", "Group2", "Layer3", "Layer4", "Layer5"};

  LayerGroup group(names[0]);
  group.add_layer(Layer(names[1]));
  group.add_layer(Layer(names[2]));

  LayerGroup group2(names[3]);
  group2.add_layer(Layer(names[4]));
  group2.add_layer(Layer(names[5]));

  group.add_group(std::move(group2));
  grease_pencil.root_group_for_write().add_group(std::move(group));
  grease_pencil.root_group_for_write().add_layer(Layer(names[6]));

  /* Save to storage. */
  grease_pencil.free_layer_tree_storage();
  grease_pencil.save_layer_tree_to_storage();
  MEM_delete(grease_pencil.runtime);

  /* Load from storage. */
  grease_pencil.runtime = MEM_new<blender::bke::GreasePencilRuntime>(__func__);
  grease_pencil.load_layer_tree_from_storage();

  Span<const TreeNode *> children = grease_pencil.root_group().children();
  for (const int i : children.index_range()) {
    const TreeNode &child = *children[i];
    EXPECT_STREQ(child.name, names[i].data());
  }
}

TEST(greasepencil, set_active_layer_index)
{
  GreasePencilIDTestContext ctx;

  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(BKE_id_new(ctx.bmain, ID_GP, "GP"));
  grease_pencil.add_empty_drawings(3);

  Layer layer1("Layer1");
  Layer layer2("Layer2");

  const Layer &layer1_ref = grease_pencil.root_group_for_write().add_layer(std::move(layer1));
  const Layer &layer2_ref = grease_pencil.root_group_for_write().add_layer(std::move(layer2));

  grease_pencil.runtime->set_active_layer_index(0);
  EXPECT_TRUE(grease_pencil.runtime->has_active_layer());
  EXPECT_STREQ(layer1_ref.name, grease_pencil.runtime->active_layer().name);

  grease_pencil.runtime->set_active_layer_index(1);
  EXPECT_TRUE(grease_pencil.runtime->has_active_layer());
  EXPECT_STREQ(layer2_ref.name, grease_pencil.runtime->active_layer().name);

  /* Save to storage. */
  grease_pencil.free_layer_tree_storage();
  grease_pencil.save_layer_tree_to_storage();
  MEM_delete(grease_pencil.runtime);

  /* Load from storage. */
  grease_pencil.runtime = MEM_new<blender::bke::GreasePencilRuntime>(__func__);
  grease_pencil.load_layer_tree_from_storage();

  /* Check if the active layer is still the second one. */
  EXPECT_TRUE(grease_pencil.runtime->has_active_layer());
  EXPECT_STREQ(grease_pencil.runtime->active_layer().name, "Layer2");
}

/* --------------------------------------------------------------------------------------------- */
/* Drawing Array Tests. */

TEST(greasepencil, add_empty_drawings)
{
  GreasePencilIDTestContext ctx;
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(BKE_id_new(ctx.bmain, ID_GP, "GP"));
  grease_pencil.add_empty_drawings(3);
  EXPECT_EQ(grease_pencil.drawings().size(), 3);
}

TEST(greasepencil, remove_drawing)
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
  layer1.tag_frames_map_keys_changed();

  layer2.insert_frame(0, GreasePencilFrame{1});
  layer2.tag_frames_map_keys_changed();

  grease_pencil.root_group_for_write().add_layer(std::move(layer1));
  grease_pencil.root_group_for_write().add_layer(std::move(layer2));

  grease_pencil.remove_drawing(1);
  EXPECT_EQ(grease_pencil.drawings().size(), 2);

  static int expected_frames_size[] = {2, 0};
  static int expected_frames_pairs_layer0[][2] = {{0, 0}, {20, 1}};

  Span<const Layer *> layers = grease_pencil.layers();
  EXPECT_EQ(layers[0]->frames().size(), expected_frames_size[0]);
  EXPECT_EQ(layers[1]->frames().size(), expected_frames_size[1]);
  EXPECT_EQ(layers[0]->frames().lookup(expected_frames_pairs_layer0[0][0]).drawing_index,
            expected_frames_pairs_layer0[0][1]);
  EXPECT_EQ(layers[0]->frames().lookup(expected_frames_pairs_layer0[1][0]).drawing_index,
            expected_frames_pairs_layer0[1][1]);
}

/* --------------------------------------------------------------------------------------------- */
/* Layer Tree Tests. */

TEST(greasepencil, layer_tree_empty)
{
  LayerGroup root{};
}

TEST(greasepencil, layer_tree_build_simple)
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

TEST(greasepencil, layer_tree_pre_order_iteration)
{
  GreasePencilLayerTreeExample ex;

  Span<const TreeNode *> children = ex.root.children();
  for (const int i : children.index_range()) {
    const TreeNode &child = *children[i];
    EXPECT_STREQ(child.name, ex.names[i].data());
  }
}

TEST(greasepencil, layer_tree_total_size)
{
  GreasePencilLayerTreeExample ex;
  EXPECT_EQ(ex.root.num_children_total(), 7);
}

TEST(greasepencil, layer_tree_node_types)
{
  GreasePencilLayerTreeExample ex;
  Span<const TreeNode *> children = ex.root.children();
  for (const int i : children.index_range()) {
    const TreeNode &child = *children[i];
    EXPECT_EQ(child.is_layer(), ex.is_layer[i]);
    EXPECT_EQ(child.is_group(), !ex.is_layer[i]);
  }
}

}  // namespace blender::bke::greasepencil::tests