/* SPDX-FileCopyrightText: 2024 Tenkai Raiko
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "node_shader_util.hh"
#include "node_util.hh"

#include "BKE_texture.h"

#include "BLI_noise.hh"

#include "NOD_multi_function.hh"

#include "RNA_access.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

namespace blender::nodes::node_shader_tex_raiko_cc {

NODE_STORAGE_FUNCS(NodeTexRaiko)

static void sh_node_tex_raiko_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.use_custom_socket_order();

  b.add_output<decl::Float>("R_sphere Field").no_muted_links();
  b.add_output<decl::Float>("R_gon Parameter Field")
      .no_muted_links()
      .make_available([](bNode &node) { node_storage(node).mode = SHD_RAIKO_CLOSEST; });
  b.add_output<decl::Float>("Max Unit Parameter Field")
      .no_muted_links()
      .make_available([](bNode &node) { node_storage(node).mode = SHD_RAIKO_CLOSEST; });
  b.add_output<decl::Float>("Segment ID Field").no_muted_links().make_available([](bNode &node) {
    node_storage(node).mode = SHD_RAIKO_CLOSEST;
  });
  b.add_output<decl::Vector>("Index Field").no_muted_links().make_available([](bNode &node) {
    node_storage(node).mode = SHD_RAIKO_CLOSEST;
    node_storage(node).grid_dimensions = 2;
  });
  b.add_output<decl::Float>("Index Field W").no_muted_links().make_available([](bNode &node) {
    node_storage(node).mode = SHD_RAIKO_CLOSEST;
    node_storage(node).grid_dimensions = 1;
  });
  b.add_output<decl::Vector>("Position Field").no_muted_links().make_available([](bNode &node) {
    node_storage(node).mode = SHD_RAIKO_CLOSEST;
  });
  b.add_output<decl::Float>("Position Field W").no_muted_links().make_available([](bNode &node) {
    node_storage(node).mode = SHD_RAIKO_CLOSEST;
  });
  b.add_output<decl::Vector>("R_sphere Coordinates")
      .no_muted_links()
      .make_available([](bNode &node) { node_storage(node).mode = SHD_RAIKO_CLOSEST; });
  b.add_output<decl::Float>("R_sphere Coordinates W")
      .no_muted_links()
      .make_available([](bNode &node) { node_storage(node).mode = SHD_RAIKO_CLOSEST; });

  b.add_input<decl::Vector>("Vector")
      .hide_value()
      .implicit_field(implicit_field_inputs::position)
      .description("XYZ components of the input vector");
  b.add_input<decl::Float>("W").min(-1000.0f).max(1000.0f).default_value(0.0f).description(
      "W component of the input vector");
  b.add_input<decl::Float>("Accuracy")
      .min(0.0f)
      .max(1.0f)
      .default_value(0.1f)
      .subtype(PROP_FACTOR)
      .description(
          "Controls how many grid points contribute to the final outputs. Lowering this value "
          "reduces computation time, however setting it too low causes visual artifacts. "
          "Therefore the Accuracy value should be lowered just above the value where visual "
          "artifacts start to appear");
  b.add_input<decl::Float>("Scale").min(-1000.0f).max(1000.0f).default_value(1.0f).description(
      "Factor by which the entire texture is scaled after all calculations");
  b.add_input<decl::Float>("Smoothness")
      .min(0.0f)
      .max(1000.0f)
      .default_value(0.5)
      .description(
          "Controls the size of the area to smooth over. The higher this value is, the smoother "
          "the outputs are")
      .make_available([](bNode &node) { node_storage(node).mode = SHD_RAIKO_SMOOTH_MINIMUM; });

  /* Panel for r-sphere inputs. */
  PanelDeclarationBuilder &r_sphere =
      b.add_panel("R-sphere Field")
          .default_closed(true)
          .draw_buttons([](uiLayout *layout, bContext * /*C*/, PointerRNA *ptr) {
            uiItemR(layout, ptr, "integer_sides", UI_ITEM_R_SPLIT_EMPTY_NAME, nullptr, ICON_NONE);
            if (RNA_boolean_get(ptr, "integer_sides") == false) {
              uiItemR(layout,
                      ptr,
                      "elliptical_corners",
                      UI_ITEM_R_SPLIT_EMPTY_NAME,
                      nullptr,
                      ICON_NONE);
            }
          });
  r_sphere.add_input<decl::Float>("R_gon Sides")
      .min(2.0f)
      .max(1000.0f)
      .default_value(5.0f)
      .description("Number of r-gon sides");
  r_sphere.add_input<decl::Float>("R_gon Roundness")
      .min(0.0f)
      .max(1.0f)
      .default_value(0.0f)
      .subtype(PROP_FACTOR)
      .description("R-gon corner roundness");
  r_sphere.add_input<decl::Float>("R_gon Exponent")
      .min(0.0f)
      .max(1000.0f)
      .default_value(2.0f)
      .description("Minkowski exponent of the r-gon");
  r_sphere.add_input<decl::Float>("Sphere Exponent")
      .min(0.0f)
      .max(1000.0f)
      .default_value(2.0f)
      .description("Minkowski exponent of the sphere");

  /* Panel for r-sphere randomness inputs. */
  PanelDeclarationBuilder &r_sphere_randomness =
      b.add_panel("Randomize R-sphere Field").default_closed(true);
  r_sphere_randomness.add_input<decl::Float>("R_gon Sides Randomness")
      .min(0.0f)
      .max(1000.0f)
      .default_value(0.0f)
      .description("Maximal absolute random deviation of R-gon Sides from it's value");
  r_sphere_randomness.add_input<decl::Float>("R_gon Roundness Randomness")
      .min(0.0f)
      .max(1.0f)
      .default_value(0.0f)
      .subtype(PROP_FACTOR)
      .description("Maximal absolute random deviation of R-gon Roundness from it's value");
  r_sphere_randomness.add_input<decl::Float>("R_gon Exponent Randomness")
      .min(0.0f)
      .max(1000.0f)
      .default_value(0.0f)
      .description("Maximal absolute random deviation of R-gon Exponent from it's value");
  r_sphere_randomness.add_input<decl::Float>("Sphere Randomness")
      .min(0.0f)
      .max(1000.0f)
      .default_value(0.0f)
      .description("Maximal absolute random deviation of Sphere Exponent from it's value");

  /* Panel for coordinate transformation inputs. */
  PanelDeclarationBuilder &transform =
      b.add_panel("Transform")
          .default_closed(true)
          .draw_buttons([](uiLayout *layout, bContext * /*C*/, PointerRNA *ptr) {
            uiItemR(layout,
                    ptr,
                    "invert_order_of_transformation",
                    UI_ITEM_R_SPLIT_EMPTY_NAME,
                    nullptr,
                    ICON_NONE);
            uiItemR(layout,
                    ptr,
                    "transform_fields_noise",
                    UI_ITEM_R_SPLIT_EMPTY_NAME,
                    nullptr,
                    ICON_NONE);
            if (RNA_enum_get(ptr, "mode") != SHD_RAIKO_ADDITIVE) {
              uiItemR(layout,
                      ptr,
                      "transform_coordinates_noise",
                      UI_ITEM_R_SPLIT_EMPTY_NAME,
                      nullptr,
                      ICON_NONE);
            }
          });
  transform.add_input<decl::Vector>("Rotation Transform")
      .min(-1000.0f)
      .max(1000.0f)
      .default_value(float3{0.0f, 0.0f, 0.0f})
      .description("XYZ Euler rotation of the coordinates used to compute the outputs");
  transform.add_input<decl::Vector>("Scale Transform")
      .min(-1000.0f)
      .max(1000.0f)
      .default_value(float3{1.0f, 1.0f, 1.0f})
      .description("First scaling factor of the coordinates used to compute the outputs");
  transform.add_input<decl::Float>("Scale Transform W")
      .min(-1000.0f)
      .max(1000.0f)
      .default_value(1.0f)
      .description("First scaling factor of the coordinates used to compute the outputs");

  /* Panel for coordinate transformation randomness inputs. */
  PanelDeclarationBuilder &transform_randomness =
      b.add_panel("Randomize Transform")
          .default_closed(true)
          .draw_buttons([](uiLayout *layout, bContext * /*C*/, PointerRNA *ptr) {
            uiItemR(layout,
                    ptr,
                    "uniform_scale_randomness",
                    UI_ITEM_R_SPLIT_EMPTY_NAME,
                    nullptr,
                    ICON_NONE);
          });
  transform_randomness.add_input<decl::Vector>("Rotation Transform Randomness")
      .min(0.0f)
      .max(1000.0f)
      .default_value(float3{0.0f, 0.0f, 0.0f})
      .description("Maximal absolute random deviation of Rotation Transform from it's value");
  transform_randomness.add_input<decl::Vector>("Scale Transform Randomness")
      .min(0.0f)
      .max(1000.0f)
      .default_value(float3{0.0f, 0.0f, 0.0f})
      .description("Maximal absolute random deviation of the second scaling exponent from 0.0")
      .make_available([](bNode &node) { node_storage(node).uniform_scale_randomness = false; });
  transform_randomness.add_input<decl::Float>("Scale Transform W Randomness")
      .min(0.0f)
      .max(1000.0f)
      .default_value(0.0f)
      .description("Maximal absolute random deviation of the second scaling exponent from 0.0");

  /* Panel for noise addition inputs. */
  PanelDeclarationBuilder &noise_addition = b.add_panel("Add Noise").default_closed(true);
  noise_addition.add_input<decl::Float>("Fragmentation")
      .min(0.0f)
      .max(1.0f)
      .default_value(0.0f)
      .subtype(PROP_FACTOR)
      .description("Noise vector rotation to prevent parallel edges");
  noise_addition.add_input<decl::Float>("Fields Strength 1")
      .min(-1000.0f)
      .max(1000.0f)
      .default_value(0.0f)
      .description("Strength of the 1st Noise layer used to compute field outputs");
  noise_addition.add_input<decl::Float>("Coordinates Strength 1")
      .min(-1000.0f)
      .max(1000.0f)
      .default_value(0.0f)
      .description(
          "Strength of the 1st Noise layer used to compute the R-sphere Coordinates output")
      .make_available([](bNode &node) { node_storage(node).mode = SHD_RAIKO_CLOSEST; });
  noise_addition.add_input<decl::Float>("Scale 1")
      .min(-1000.0f)
      .max(1000.0f)
      .default_value(5.0f)
      .description("Scale of the 1st Noise layer");
  noise_addition.add_input<decl::Float>("Detail 1")
      .min(0.0f)
      .max(15.0f)
      .default_value(2.0f)
      .description("Detail of the 1st Noise layer");
  noise_addition.add_input<decl::Float>("Roughness 1")
      .min(0.0f)
      .max(1.0f)
      .default_value(0.5f)
      .subtype(PROP_FACTOR)
      .description("Roughness of the 1st Noise layer");
  noise_addition.add_input<decl::Float>("Lacunarity 1")
      .min(0.0f)
      .max(1000.0f)
      .default_value(2.0f)
      .description("Lacunarity of the 1st Noise layer");
  noise_addition.add_input<decl::Float>("Fields Strength 2")
      .min(-1000.0f)
      .max(1000.0f)
      .default_value(0.0f)
      .description("Strength of the 2nd Noise layer used to compute field outputs");
  noise_addition.add_input<decl::Float>("Coordinates Strength 2")
      .min(-1000.0f)
      .max(1000.0f)
      .default_value(0.0f)
      .description(
          "Strength of the 2nd Noise layer used to compute the R-sphere Coordinates output")
      .make_available([](bNode &node) { node_storage(node).mode = SHD_RAIKO_CLOSEST; });
  noise_addition.add_input<decl::Float>("Scale 2")
      .min(-1000.0f)
      .max(1000.0f)
      .default_value(5.0f)
      .description("Scale of the 2nd Noise layer");
  noise_addition.add_input<decl::Float>("Detail 2")
      .min(0.0f)
      .max(15.0f)
      .default_value(2.0f)
      .description("Detail of the 2nd Noise layer");
  noise_addition.add_input<decl::Float>("Roughness 2")
      .min(0.0f)
      .max(1.0f)
      .default_value(0.5f)
      .subtype(PROP_FACTOR)
      .description("Roughness of the 2nd Noise layer");
  noise_addition.add_input<decl::Float>("Lacunarity 2")
      .min(0.0f)
      .max(1000.0f)
      .default_value(2.0f)
      .description("Lacunarity of the 2nd Noise layer");

  /* Panel for grid inputs. */
  PanelDeclarationBuilder &grid = b.add_panel("Grid").default_closed(true).draw_buttons(
      [](uiLayout *layout, bContext * /*C*/, PointerRNA *ptr) {
        uiItemR(layout, ptr, "grid_dimensions", UI_ITEM_R_SPLIT_EMPTY_NAME, "", ICON_NONE);
      });
  grid.add_input<decl::Vector>("Grid Vector 1")
      .min(-1000.0f)
      .max(1000.0f)
      .default_value(float3{1.0f, 0.0f, 0.0f})
      .make_available([](bNode &node) { node_storage(node).grid_dimensions = 2; })
      .description("1st grid basis vector");
  grid.add_input<decl::Float>("Grid Vector W 1")
      .min(-1000.0f)
      .max(1000.0f)
      .default_value(0.0f)
      .make_available([](bNode &node) { node_storage(node).grid_dimensions = 1; })
      .description("1st grid basis vector");
  grid.add_input<decl::Vector>("Grid Vector 2")
      .min(-1000.0f)
      .max(1000.0f)
      .default_value(float3{0.0f, 1.0f, 0.0f})
      .make_available([](bNode &node) { node_storage(node).grid_dimensions = 2; })
      .description("2nd grid basis vector");
  grid.add_input<decl::Float>("Grid Vector W 2")
      .min(-1000.0f)
      .max(1000.0f)
      .default_value(0.0f)
      .make_available([](bNode &node) { node_storage(node).grid_dimensions = 4; })
      .description("2nd grid basis vector");
  grid.add_input<decl::Vector>("Grid Vector 3")
      .min(-1000.0f)
      .max(1000.0f)
      .default_value(float3{0.0f, 0.0f, 1.0f})
      .make_available([](bNode &node) { node_storage(node).grid_dimensions = 3; })
      .description("3rd grid basis vector");
  grid.add_input<decl::Float>("Grid Vector W 3")
      .min(-1000.0f)
      .max(1000.0f)
      .default_value(0.0f)
      .make_available([](bNode &node) { node_storage(node).grid_dimensions = 4; })
      .description("3rd grid basis vector");
  grid.add_input<decl::Vector>("Grid Vector 4")
      .min(-1000.0f)
      .max(1000.0f)
      .default_value(float3{0.0f, 0.0f, 0.0f})
      .make_available([](bNode &node) { node_storage(node).grid_dimensions = 4; })
      .description("4th grid basis vector");
  grid.add_input<decl::Float>("Grid Vector W 4")
      .min(-1000.0f)
      .max(1000.0f)
      .default_value(1.0f)
      .make_available([](bNode &node) { node_storage(node).grid_dimensions = 4; })
      .description("4th grid basis vector");

  /* Panel for grid randomness inputs. */
  PanelDeclarationBuilder &grid_randomness = b.add_panel("Randomize Grid").default_closed(true);
  grid_randomness.add_input<decl::Vector>("Grid Points Translation Randomness")
      .min(0.0f)
      .max(1000.0f)
      .default_value(float3{0.0f, 0.0f, 0.0f})
      .description("Maximal absolute random translation of grid points from their grid positions");
  grid_randomness.add_input<decl::Float>("Grid Points Translation W Randomness")
      .min(0.0f)
      .max(1000.0f)
      .default_value(0.0f)
      .description("Maximal absolute random translation of grid points from their grid positions");

  /* Panel for chained elliptical remap inputs. */
  PanelDeclarationBuilder &remap = b.add_panel("Remap").default_closed(true).draw_buttons(
      [](uiLayout *layout, bContext * /*C*/, PointerRNA *ptr) {
        if (RNA_enum_get(ptr, "mode") == SHD_RAIKO_ADDITIVE) {
          uiItemR(layout, ptr, "step_count", UI_ITEM_R_SPLIT_EMPTY_NAME, "", ICON_NONE);
        }
      });
  remap.add_input<decl::Float>("Step Center 1")
      .min(0.0f)
      .max(1000.0f)
      .default_value(0.1875f)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 1;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description(
          "Center position of the 1st step gradient. The steps may overlap with each other");
  remap.add_input<decl::Float>("Step Width 1")
      .min(0.0f)
      .max(1000.0f)
      .default_value(0.125f)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 1;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Width of the 1st step gradient. The steps may overlap with each other");
  remap.add_input<decl::Float>("Step Value 1")
      .min(-1000.0f)
      .max(1000.0f)
      .default_value(0.5f)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 1;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Start value of the 1st step");
  remap.add_input<decl::Float>("Ellipse Height 1")
      .min(0.0f)
      .max(1.0f)
      .default_value(0.5f)
      .subtype(PROP_FACTOR)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 1;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Height of the 1st elliptical segments");
  remap.add_input<decl::Float>("Ellipse Width 1")
      .min(0.0f)
      .max(1.0f)
      .default_value(0.5f)
      .subtype(PROP_FACTOR)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 1;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Width of the 1st elliptical segments");
  remap.add_input<decl::Float>("Inflection Point 1")
      .min(0.0f)
      .max(1.0f)
      .default_value(0.5f)
      .subtype(PROP_FACTOR)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 1;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Position of the 1st inflection point");
  remap.add_input<decl::Float>("Step Center 2")
      .min(0.0f)
      .max(1000.0f)
      .default_value(0.34375f)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 2;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description(
          "Center position of the 2nd step gradient. The steps may overlap with each other");
  remap.add_input<decl::Float>("Step Width 2")
      .min(0.0f)
      .max(1000.0f)
      .default_value(0.0625f)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 2;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Width of the 2nd step gradient. The steps may overlap with each other");
  remap.add_input<decl::Float>("Step Value 2")
      .min(-1000.0f)
      .max(1000.0f)
      .default_value(0.25f)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 2;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Start value of the 2nd step. It is also the end value of the 1st step");
  remap.add_input<decl::Float>("Ellipse Height 2")
      .min(0.0f)
      .max(1.0f)
      .default_value(0.5f)
      .subtype(PROP_FACTOR)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 2;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Height of the 2nd elliptical segments");
  remap.add_input<decl::Float>("Ellipse Width 2")
      .min(0.0f)
      .max(1.0f)
      .default_value(0.5f)
      .subtype(PROP_FACTOR)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 2;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Width of the 2nd elliptical segments");
  remap.add_input<decl::Float>("Inflection Point 2")
      .min(0.0f)
      .max(1.0f)
      .default_value(0.5f)
      .subtype(PROP_FACTOR)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 2;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Position of the 2nd inflection point");
  remap.add_input<decl::Float>("Step Center 3")
      .min(0.0f)
      .max(1000.0f)
      .default_value(0.421875f)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 3;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description(
          "Center position of the 3rd step gradient. The steps may overlap with each other");
  remap.add_input<decl::Float>("Step Width 3")
      .min(0.0f)
      .max(1000.0f)
      .default_value(0.03125f)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 3;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Width of the 3rd step gradient. The steps may overlap with each other");
  remap.add_input<decl::Float>("Step Value 3")
      .min(-1000.0f)
      .max(1000.0f)
      .default_value(0.125f)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 3;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Start value of the 3rd step. It is also the end value of the 2nd step");
  remap.add_input<decl::Float>("Ellipse Height 3")
      .min(0.0f)
      .max(1.0f)
      .default_value(0.5f)
      .subtype(PROP_FACTOR)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 3;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Height of the 3rd elliptical segments");
  remap.add_input<decl::Float>("Ellipse Width 3")
      .min(0.0f)
      .max(1.0f)
      .default_value(0.5f)
      .subtype(PROP_FACTOR)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 3;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Width of the 3rd elliptical segments");
  remap.add_input<decl::Float>("Inflection Point 3")
      .min(0.0f)
      .max(1.0f)
      .default_value(0.5f)
      .subtype(PROP_FACTOR)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 3;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Position of the 3rd inflection point");
  remap.add_input<decl::Float>("Step Center 4")
      .min(0.0f)
      .max(1000.0f)
      .default_value(0.4609375f)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 4;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description(
          "Center position of the 4th step gradient. The steps may overlap with each other");
  remap.add_input<decl::Float>("Step Width 4")
      .min(0.0f)
      .max(1000.0f)
      .default_value(0.015625f)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 4;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Width of the 4th step gradient. The steps may overlap with each other");
  remap.add_input<decl::Float>("Step Value 4")
      .min(-1000.0f)
      .max(1000.0f)
      .default_value(0.0625f)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 4;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Start value of the 4th step. It is also the end value of the 3rd step");
  remap.add_input<decl::Float>("Ellipse Height 4")
      .min(0.0f)
      .max(1.0f)
      .default_value(0.5f)
      .subtype(PROP_FACTOR)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 4;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Height of the 4th elliptical segments");
  remap.add_input<decl::Float>("Ellipse Width 4")
      .min(0.0f)
      .max(1.0f)
      .default_value(0.5f)
      .subtype(PROP_FACTOR)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 4;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Width of the 4th elliptical segments");
  remap.add_input<decl::Float>("Inflection Point 4")
      .min(0.0f)
      .max(1.0f)
      .default_value(0.5f)
      .subtype(PROP_FACTOR)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 4;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Position of the 4th inflection point");

  /* Panel for chained elliptical remap randomness inputs. */
  PanelDeclarationBuilder &remap_randomness = b.add_panel("Randomize Remap").default_closed(true);
  remap_randomness.add_input<decl::Float>("Step Center Randomness 1")
      .min(0.0f)
      .max(1000.0f)
      .default_value(0.0f)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 1;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Maximal absolute random deviation of Step Center 1 from it's value");
  remap_randomness.add_input<decl::Float>("Step Width Randomness 1")
      .min(0.0f)
      .max(1000.0f)
      .default_value(0.0f)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 1;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Maximal absolute random deviation of Step Width 1 from it's value");
  remap_randomness.add_input<decl::Float>("Step Value Randomness 1")
      .min(0.0f)
      .max(1000.0f)
      .default_value(0.0f)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 1;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Maximal absolute random deviation of Step Value 1 from it's value");
  remap_randomness.add_input<decl::Float>("Ellipse Height Randomness 1")
      .min(0.0f)
      .max(1.0f)
      .default_value(0.0f)
      .subtype(PROP_FACTOR)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 1;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Maximal absolute random deviation of Ellipse Height 1 from it's value");
  remap_randomness.add_input<decl::Float>("Ellipse Width Randomness 1")
      .min(0.0f)
      .max(1.0f)
      .default_value(0.0f)
      .subtype(PROP_FACTOR)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 1;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Maximal absolute random deviation of Ellipse Width 1 from it's value");
  remap_randomness.add_input<decl::Float>("Inflection Point Randomness 1")
      .min(0.0f)
      .max(1.0f)
      .default_value(0.0f)
      .subtype(PROP_FACTOR)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 1;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Maximal absolute random deviation of Inflection Point 1 from it's value");
  remap_randomness.add_input<decl::Float>("Step Center Randomness 2")
      .min(0.0f)
      .max(1000.0f)
      .default_value(0.0f)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 2;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Maximal absolute random deviation of Step Center 2 from it's value");
  remap_randomness.add_input<decl::Float>("Step Width Randomness 2")
      .min(0.0f)
      .max(1000.0f)
      .default_value(0.0f)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 2;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Maximal absolute random deviation of Step Center 2 from it's value");
  remap_randomness.add_input<decl::Float>("Step Value Randomness 2")
      .min(0.0f)
      .max(1000.0f)
      .default_value(0.0f)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 2;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Maximal absolute random deviation of Step Value 2 from it's value");
  remap_randomness.add_input<decl::Float>("Ellipse Height Randomness 2")
      .min(0.0f)
      .max(1.0f)
      .default_value(0.0f)
      .subtype(PROP_FACTOR)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 2;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Maximal absolute random deviation of Ellipse Height 2 from it's value");
  remap_randomness.add_input<decl::Float>("Ellipse Width Randomness 2")
      .min(0.0f)
      .max(1.0f)
      .default_value(0.0f)
      .subtype(PROP_FACTOR)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 2;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Maximal absolute random deviation of Ellipse Width 2 from it's value");
  remap_randomness.add_input<decl::Float>("Inflection Point Randomness 2")
      .min(0.0f)
      .max(1.0f)
      .default_value(0.0f)
      .subtype(PROP_FACTOR)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 2;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Maximal absolute random deviation of Inflection Point 2 from it's value");
  remap_randomness.add_input<decl::Float>("Step Center Randomness 3")
      .min(0.0f)
      .max(1000.0f)
      .default_value(0.0f)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 3;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Maximal absolute random deviation of Step Center 3 from it's value");
  remap_randomness.add_input<decl::Float>("Step Width Randomness 3")
      .min(0.0f)
      .max(1000.0f)
      .default_value(0.0f)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 3;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Maximal absolute random deviation of Step Width 3 from it's value");
  remap_randomness.add_input<decl::Float>("Step Value Randomness 3")
      .min(0.0f)
      .max(1000.0f)
      .default_value(0.0f)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 3;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Maximal absolute random deviation of Step Value 3 from it's value");
  remap_randomness.add_input<decl::Float>("Ellipse Height Randomness 3")
      .min(0.0f)
      .max(1.0f)
      .default_value(0.0f)
      .subtype(PROP_FACTOR)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 3;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Maximal absolute random deviation of Ellipse Height 3 from it's value");
  remap_randomness.add_input<decl::Float>("Ellipse Width Randomness 3")
      .min(0.0f)
      .max(1.0f)
      .default_value(0.0f)
      .subtype(PROP_FACTOR)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 3;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Maximal absolute random deviation of Ellipse Width 3 from it's value");
  remap_randomness.add_input<decl::Float>("Inflection Point Randomness 3")
      .min(0.0f)
      .max(1.0f)
      .default_value(0.0f)
      .subtype(PROP_FACTOR)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 3;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Maximal absolute random deviation of Inflection Point 3 from it's value");
  remap_randomness.add_input<decl::Float>("Step Center Randomness 4")
      .min(0.0f)
      .max(1000.0f)
      .default_value(0.0f)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 4;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Maximal absolute random deviation of Step Center 4 from it's value");
  remap_randomness.add_input<decl::Float>("Step Width Randomness 4")
      .min(0.0f)
      .max(1000.0f)
      .default_value(0.0f)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 4;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Maximal absolute random deviation of  from it's value");
  remap_randomness.add_input<decl::Float>("Step Value Randomness 4")
      .min(0.0f)
      .max(1000.0f)
      .default_value(0.0f)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 4;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Maximal absolute random deviation of Step Value 4 from it's value");
  remap_randomness.add_input<decl::Float>("Ellipse Height Randomness 4")
      .min(0.0f)
      .max(1.0f)
      .default_value(0.0f)
      .subtype(PROP_FACTOR)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 4;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Maximal absolute random deviation of Ellipse Height 4 from it's value");
  remap_randomness.add_input<decl::Float>("Ellipse Width Randomness 4")
      .min(0.0f)
      .max(1.0f)
      .default_value(0.0f)
      .subtype(PROP_FACTOR)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 4;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Maximal absolute random deviation of Ellipse Width 4 from it's value");
  remap_randomness.add_input<decl::Float>("Inflection Point Randomness 4")
      .min(0.0f)
      .max(1.0f)
      .default_value(0.0f)
      .subtype(PROP_FACTOR)
      .make_available([](bNode &node) {
        node_storage(node).step_count = 4;
        node_storage(node).mode = SHD_RAIKO_ADDITIVE;
      })
      .description("Maximal absolute random deviation of Inflection Point 4 from it's value");
}

static void node_shader_buts_tex_raiko(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "mode", UI_ITEM_R_SPLIT_EMPTY_NAME, "", ICON_NONE);
  if (RNA_enum_get(ptr, "mode") != SHD_RAIKO_ADDITIVE) {
    uiItemR(
        layout, ptr, "normalize_r_gon_parameter", UI_ITEM_R_SPLIT_EMPTY_NAME, nullptr, ICON_NONE);
  }
}

static void node_shader_init_tex_raiko(bNodeTree * /*ntree*/, bNode *node)
{
  NodeTexRaiko *tex = MEM_cnew<NodeTexRaiko>(__func__);
  BKE_texture_mapping_default(&tex->base.tex_mapping, TEXMAP_TYPE_POINT);
  BKE_texture_colormapping_default(&tex->base.color_mapping);
  tex->mode = SHD_RAIKO_ADDITIVE;
  tex->normalize_r_gon_parameter = false;
  tex->integer_sides = false;
  tex->elliptical_corners = false;
  tex->invert_order_of_transformation = false;
  tex->transform_fields_noise = true;
  tex->transform_coordinates_noise = true;
  tex->uniform_scale_randomness = true;
  tex->grid_dimensions = 2;
  tex->step_count = 1;

  node->storage = tex;
}

static const char *gpu_shader_get_name()
{
  return "node_tex_raiko";
}

static int node_shader_gpu_tex_raiko(GPUMaterial *mat,
                                     bNode *node,
                                     bNodeExecData * /*execdata*/,
                                     GPUNodeStack *in,
                                     GPUNodeStack *out)
{
  node_shader_gpu_default_tex_coord(mat, node, &in[0].link);
  node_shader_gpu_tex_mapping(mat, node, in, out);

  /* Sanitize inputs. */
  for (int i = 0; !in[i].end; ++i) {
    if (in[i].link == nullptr) {
      in[i].link = GPU_uniform(in[i].vec);
    }
  }

  const NodeTexRaiko &storage = node_storage(*node);
  float mode = storage.mode;
  float normalize_r_gon_parameter = out[1].hasoutput && storage.normalize_r_gon_parameter;
  float integer_sides = storage.integer_sides;
  float elliptical_corners = storage.elliptical_corners;
  float invert_order_of_transformation = storage.invert_order_of_transformation;
  float transform_fields_noise = storage.transform_fields_noise;
  float transform_coordinates_noise = storage.transform_coordinates_noise;
  float uniform_scale_randomness = storage.uniform_scale_randomness;
  float grid_dimensions = storage.grid_dimensions;
  float step_count = storage.step_count;
  float calculate_r_sphere_field = out[0].hasoutput || out[1].hasoutput || out[2].hasoutput ||
                                   (storage.mode != SHD_RAIKO_CLOSEST);
  float calculate_r_gon_parameter_field = out[1].hasoutput;
  float calculate_max_unit_parameter_field = out[2].hasoutput;
  float calculate_coordinates_outputs = (out[8].hasoutput || out[9].hasoutput) &&
                                        (storage.mode != SHD_RAIKO_ADDITIVE);

  /* Output connections*/
  GPUNodeLink *in_calculate;
  GPU_link(mat,
           "set_rgba_from_components",
           GPU_constant(&calculate_r_sphere_field),
           GPU_constant(&calculate_r_gon_parameter_field),
           GPU_constant(&calculate_max_unit_parameter_field),
           GPU_constant(&calculate_coordinates_outputs),
           &in_calculate);

  /* Node booleans and enums. */
  GPUNodeLink *in_properties_1;
  GPU_link(mat,
           "set_rgba_from_components",
           GPU_constant(&mode),
           GPU_constant(&normalize_r_gon_parameter),
           GPU_constant(&integer_sides),
           GPU_constant(&elliptical_corners),
           &in_properties_1);
  GPUNodeLink *in_properties_2;
  GPU_link(mat,
           "set_rgba_from_components",
           GPU_constant(&invert_order_of_transformation),
           GPU_constant(&uniform_scale_randomness),
           GPU_constant(&grid_dimensions),
           GPU_constant(&step_count),
           &in_properties_2);

  GPUNodeLink *in_coord;
  GPU_link(mat, "set_rgba_from_vec3_and_float", in[0].link, in[1].link, &in_coord);

  GPUNodeLink *in_accuracy_scale_smoothness;
  GPU_link(mat,
           "set_vec3_from_components",
           in[2].link,
           in[3].link,
           in[4].link,
           &in_accuracy_scale_smoothness);

  /* R-sphere */
  GPUNodeLink *in_r_sphere;
  GPU_link(mat,
           "set_rgba_from_components",
           in[5].link,
           in[6].link,
           in[7].link,
           in[8].link,
           &in_r_sphere);

  /* Randomize R-sphere */
  GPUNodeLink *in_r_sphere_randomness;
  GPU_link(mat,
           "set_rgba_from_components",
           in[9].link,
           in[10].link,
           in[11].link,
           in[12].link,
           &in_r_sphere_randomness);

  /* Transform */
  GPUNodeLink *in_transform_scale;
  GPU_link(mat, "set_rgba_from_vec3_and_float", in[14].link, in[15].link, &in_transform_scale);

  /* Randomize Transform */
  GPUNodeLink *in_transform_scale_randomness;
  GPU_link(mat,
           "set_rgba_from_vec3_and_float",
           in[17].link,
           in[18].link,
           &in_transform_scale_randomness);

  /* Add Noise */
  GPUNodeLink *in_noise_1;
  GPU_link(mat,
           "set_rgba_from_components",
           GPU_constant(&transform_fields_noise),
           GPU_constant(&transform_coordinates_noise),
           in[19].link,
           in[20].link,
           &in_noise_1);
  GPUNodeLink *in_noise_2;
  GPU_link(mat,
           "set_rgba_from_components",
           in[21].link,
           in[22].link,
           in[23].link,
           in[24].link,
           &in_noise_2);
  GPUNodeLink *in_noise_3;
  GPU_link(mat,
           "set_rgba_from_components",
           in[25].link,
           in[26].link,
           in[27].link,
           in[28].link,
           &in_noise_3);
  GPUNodeLink *in_noise_4;
  GPU_link(mat, "set_vec3_from_components", in[29].link, in[30].link, in[31].link, &in_noise_4);

  /* Grid */
  GPUNodeLink *in_grid_vector_1;
  GPU_link(mat, "set_rgba_from_vec3_and_float", in[32].link, in[33].link, &in_grid_vector_1);
  GPUNodeLink *in_grid_vector_2;
  GPU_link(mat, "set_rgba_from_vec3_and_float", in[34].link, in[35].link, &in_grid_vector_2);
  GPUNodeLink *in_grid_vector_3;
  GPU_link(mat, "set_rgba_from_vec3_and_float", in[36].link, in[37].link, &in_grid_vector_3);
  GPUNodeLink *in_grid_vector_4;
  GPU_link(mat, "set_rgba_from_vec3_and_float", in[38].link, in[39].link, &in_grid_vector_4);

  /* Randomize Grid */
  GPUNodeLink *in_grid_points_translation_randomness;
  GPU_link(mat,
           "set_rgba_from_vec3_and_float",
           in[40].link,
           in[41].link,
           &in_grid_points_translation_randomness);

  /* Chained elliptical remap */
  GPUNodeLink *in_step_center;
  GPU_link(mat,
           "set_rgba_from_components",
           in[42].link,
           in[48].link,
           in[54].link,
           in[60].link,
           &in_step_center);
  GPUNodeLink *in_step_width;
  GPU_link(mat,
           "set_rgba_from_components",
           in[43].link,
           in[49].link,
           in[55].link,
           in[61].link,
           &in_step_width);
  GPUNodeLink *in_step_value;
  GPU_link(mat,
           "set_rgba_from_components",
           in[44].link,
           in[50].link,
           in[56].link,
           in[62].link,
           &in_step_value);
  GPUNodeLink *in_ellipse_height;
  GPU_link(mat,
           "set_rgba_from_components",
           in[45].link,
           in[51].link,
           in[57].link,
           in[63].link,
           &in_ellipse_height);
  GPUNodeLink *in_ellipse_width;
  GPU_link(mat,
           "set_rgba_from_components",
           in[46].link,
           in[52].link,
           in[58].link,
           in[64].link,
           &in_ellipse_width);
  GPUNodeLink *in_inflection_point;
  GPU_link(mat,
           "set_rgba_from_components",
           in[47].link,
           in[53].link,
           in[59].link,
           in[65].link,
           &in_inflection_point);

  /* Randomize Chained Elliptical Remap */
  GPUNodeLink *in_step_center_randomness;
  GPU_link(mat,
           "set_rgba_from_components",
           in[66].link,
           in[72].link,
           in[78].link,
           in[84].link,
           &in_step_center_randomness);
  GPUNodeLink *in_step_width_randomness;
  GPU_link(mat,
           "set_rgba_from_components",
           in[67].link,
           in[73].link,
           in[79].link,
           in[85].link,
           &in_step_width_randomness);
  GPUNodeLink *in_step_value_randomness;
  GPU_link(mat,
           "set_rgba_from_components",
           in[68].link,
           in[74].link,
           in[80].link,
           in[86].link,
           &in_step_value_randomness);
  GPUNodeLink *in_ellipse_height_randomness;
  GPU_link(mat,
           "set_rgba_from_components",
           in[69].link,
           in[75].link,
           in[81].link,
           in[87].link,
           &in_ellipse_height_randomness);
  GPUNodeLink *in_ellipse_width_randomness;
  GPU_link(mat,
           "set_rgba_from_components",
           in[70].link,
           in[76].link,
           in[82].link,
           in[88].link,
           &in_ellipse_width_randomness);
  GPUNodeLink *in_inflection_point_randomness;
  GPU_link(mat,
           "set_rgba_from_components",
           in[71].link,
           in[77].link,
           in[83].link,
           in[89].link,
           &in_inflection_point_randomness);

  GPUNodeLink *out_fields;
  GPUNodeLink *out_index_field;
  GPUNodeLink *out_position_field;
  GPUNodeLink *out_r_sphere_coordinates;

  const char *name = gpu_shader_get_name();

  GPU_link(mat,
           name,
           in_calculate,
           in_properties_1,
           in_properties_2,
           in_coord,
           in_accuracy_scale_smoothness,
           in_r_sphere,
           in_r_sphere_randomness,
           in[13].link,
           in_transform_scale,
           in[16].link,
           in_transform_scale_randomness,
           in_noise_1,
           in_noise_2,
           in_noise_3,
           in_noise_4,
           in_grid_vector_1,
           in_grid_vector_2,
           in_grid_vector_3,
           in_grid_vector_4,
           in_grid_points_translation_randomness,
           in_step_center,
           in_step_width,
           in_step_value,
           in_ellipse_height,
           in_ellipse_width,
           in_inflection_point,
           in_step_center_randomness,
           in_step_width_randomness,
           in_step_value_randomness,
           in_ellipse_height_randomness,
           in_ellipse_width_randomness,
           in_inflection_point_randomness,
           &out_fields,
           &out_index_field,
           &out_position_field,
           &out_r_sphere_coordinates);

  GPU_link(mat,
           "set_components_from_vec4",
           out_fields,
           &out[0].link,
           &out[1].link,
           &out[2].link,
           &out[3].link);
  GPU_link(mat, "set_vec3_and_float_from_rgba", out_index_field, &out[4].link, &out[5].link);
  GPU_link(mat, "set_vec3_and_float_from_rgba", out_position_field, &out[6].link, &out[7].link);
  return GPU_link(
      mat, "set_vec3_and_float_from_rgba", out_r_sphere_coordinates, &out[8].link, &out[9].link);
}

static void node_shader_update_tex_raiko(bNodeTree *ntree, bNode *node)
{
  bNodeSocket *inSmoothnessSock = bke::node_find_socket(node, SOCK_IN, "Smoothness");

  /* R-sphere */
  bNodeSocket *inR_gonSidesSock = bke::node_find_socket(node, SOCK_IN, "R_gon Sides");
  bNodeSocket *inR_gonRoundnessSock = bke::node_find_socket(node, SOCK_IN, "R_gon Roundness");
  bNodeSocket *inR_gonExponentSock = bke::node_find_socket(node, SOCK_IN, "R_gon Exponent");

  /* Randomize R-sphere */
  bNodeSocket *inR_gonSidesRandomnessSock = bke::node_find_socket(
      node, SOCK_IN, "R_gon Sides Randomness");
  bNodeSocket *inR_gonRoundnessRandomnessSock = bke::node_find_socket(
      node, SOCK_IN, "R_gon Roundness Randomness");
  bNodeSocket *inR_gonExponentRandomnessSock = bke::node_find_socket(
      node, SOCK_IN, "R_gon Exponent Randomness");

  /* Randomize Transform */
  bNodeSocket *inScaleTransformRandomnessSock = bke::node_find_socket(
      node, SOCK_IN, "Scale Transform Randomness");
  bNodeSocket *inScaleTransformWRandomnessSock = bke::node_find_socket(
      node, SOCK_IN, "Scale Transform W Randomness");

  /* Add Noise */
  bNodeSocket *inNoiseCoordinatesStrength1Sock = bke::node_find_socket(
      node, SOCK_IN, "Coordinates Strength 1");
  bNodeSocket *inNoiseCoordinatesStrength2Sock = bke::node_find_socket(
      node, SOCK_IN, "Coordinates Strength 2");

  /* Grid */
  bNodeSocket *inGridVector1Sock = bke::node_find_socket(node, SOCK_IN, "Grid Vector 1");
  bNodeSocket *inGridVectorW1Sock = bke::node_find_socket(node, SOCK_IN, "Grid Vector W 1");
  bNodeSocket *inGridVector2Sock = bke::node_find_socket(node, SOCK_IN, "Grid Vector 2");
  bNodeSocket *inGridVectorW2Sock = bke::node_find_socket(node, SOCK_IN, "Grid Vector W 2");
  bNodeSocket *inGridVector3Sock = bke::node_find_socket(node, SOCK_IN, "Grid Vector 3");
  bNodeSocket *inGridVectorW3Sock = bke::node_find_socket(node, SOCK_IN, "Grid Vector W 3");
  bNodeSocket *inGridVector4Sock = bke::node_find_socket(node, SOCK_IN, "Grid Vector 4");
  bNodeSocket *inGridVectorW4Sock = bke::node_find_socket(node, SOCK_IN, "Grid Vector W 4");

  /* Chained Elliptical Remap */
  bNodeSocket *inStepCenter1Sock = bke::node_find_socket(node, SOCK_IN, "Step Center 1");
  bNodeSocket *inStepWidth1Sock = bke::node_find_socket(node, SOCK_IN, "Step Width 1");
  bNodeSocket *inStepValue1Sock = bke::node_find_socket(node, SOCK_IN, "Step Value 1");
  bNodeSocket *inEllipseWidth1Sock = bke::node_find_socket(node, SOCK_IN, "Ellipse Width 1");
  bNodeSocket *inEllipseHeight1Sock = bke::node_find_socket(node, SOCK_IN, "Ellipse Height 1");
  bNodeSocket *inInflectionPoint1Sock = bke::node_find_socket(node, SOCK_IN, "Inflection Point 1");
  bNodeSocket *inStepCenter2Sock = bke::node_find_socket(node, SOCK_IN, "Step Center 2");
  bNodeSocket *inStepWidth2Sock = bke::node_find_socket(node, SOCK_IN, "Step Width 2");
  bNodeSocket *inStepValue2Sock = bke::node_find_socket(node, SOCK_IN, "Step Value 2");
  bNodeSocket *inEllipseWidth2Sock = bke::node_find_socket(node, SOCK_IN, "Ellipse Width 2");
  bNodeSocket *inEllipseHeight2Sock = bke::node_find_socket(node, SOCK_IN, "Ellipse Height 2");
  bNodeSocket *inInflectionPoint2Sock = bke::node_find_socket(node, SOCK_IN, "Inflection Point 2");
  bNodeSocket *inStepCenter3Sock = bke::node_find_socket(node, SOCK_IN, "Step Center 3");
  bNodeSocket *inStepWidth3Sock = bke::node_find_socket(node, SOCK_IN, "Step Width 3");
  bNodeSocket *inStepValue3Sock = bke::node_find_socket(node, SOCK_IN, "Step Value 3");
  bNodeSocket *inEllipseWidth3Sock = bke::node_find_socket(node, SOCK_IN, "Ellipse Width 3");
  bNodeSocket *inEllipseHeight3Sock = bke::node_find_socket(node, SOCK_IN, "Ellipse Height 3");
  bNodeSocket *inInflectionPoint3Sock = bke::node_find_socket(node, SOCK_IN, "Inflection Point 3");
  bNodeSocket *inStepCenter4Sock = bke::node_find_socket(node, SOCK_IN, "Step Center 4");
  bNodeSocket *inStepWidth4Sock = bke::node_find_socket(node, SOCK_IN, "Step Width 4");
  bNodeSocket *inStepValue4Sock = bke::node_find_socket(node, SOCK_IN, "Step Value 4");
  bNodeSocket *inEllipseWidth4Sock = bke::node_find_socket(node, SOCK_IN, "Ellipse Width 4");
  bNodeSocket *inEllipseHeight4Sock = bke::node_find_socket(node, SOCK_IN, "Ellipse Height 4");
  bNodeSocket *inInflectionPoint4Sock = bke::node_find_socket(node, SOCK_IN, "Inflection Point 4");

  /* Randomize Chained Elliptical Remap */
  bNodeSocket *inStepCenterRandomness1Sock = bke::node_find_socket(
      node, SOCK_IN, "Step Center Randomness 1");
  bNodeSocket *inStepWidthRandomness1Sock = bke::node_find_socket(
      node, SOCK_IN, "Step Width Randomness 1");
  bNodeSocket *inStepValueRandomness1Sock = bke::node_find_socket(
      node, SOCK_IN, "Step Value Randomness 1");
  bNodeSocket *inEllipseWidthRandomness1Sock = bke::node_find_socket(
      node, SOCK_IN, "Ellipse Width Randomness 1");
  bNodeSocket *inEllipseHeightRandomness1Sock = bke::node_find_socket(
      node, SOCK_IN, "Ellipse Height Randomness 1");
  bNodeSocket *inInflectionPointRandomness1Sock = bke::node_find_socket(
      node, SOCK_IN, "Inflection Point Randomness 1");
  bNodeSocket *inStepCenterRandomness2Sock = bke::node_find_socket(
      node, SOCK_IN, "Step Center Randomness 2");
  bNodeSocket *inStepWidthRandomness2Sock = bke::node_find_socket(
      node, SOCK_IN, "Step Width Randomness 2");
  bNodeSocket *inStepValueRandomness2Sock = bke::node_find_socket(
      node, SOCK_IN, "Step Value Randomness 2");
  bNodeSocket *inEllipseWidthRandomness2Sock = bke::node_find_socket(
      node, SOCK_IN, "Ellipse Width Randomness 2");
  bNodeSocket *inEllipseHeightRandomness2Sock = bke::node_find_socket(
      node, SOCK_IN, "Ellipse Height Randomness 2");
  bNodeSocket *inInflectionPointRandomness2Sock = bke::node_find_socket(
      node, SOCK_IN, "Inflection Point Randomness 2");
  bNodeSocket *inStepCenterRandomness3Sock = bke::node_find_socket(
      node, SOCK_IN, "Step Center Randomness 3");
  bNodeSocket *inStepWidthRandomness3Sock = bke::node_find_socket(
      node, SOCK_IN, "Step Width Randomness 3");
  bNodeSocket *inStepValueRandomness3Sock = bke::node_find_socket(
      node, SOCK_IN, "Step Value Randomness 3");
  bNodeSocket *inEllipseWidthRandomness3Sock = bke::node_find_socket(
      node, SOCK_IN, "Ellipse Width Randomness 3");
  bNodeSocket *inEllipseHeightRandomness3Sock = bke::node_find_socket(
      node, SOCK_IN, "Ellipse Height Randomness 3");
  bNodeSocket *inInflectionPointRandomness3Sock = bke::node_find_socket(
      node, SOCK_IN, "Inflection Point Randomness 3");
  bNodeSocket *inStepCenterRandomness4Sock = bke::node_find_socket(
      node, SOCK_IN, "Step Center Randomness 4");
  bNodeSocket *inStepWidthRandomness4Sock = bke::node_find_socket(
      node, SOCK_IN, "Step Width Randomness 4");
  bNodeSocket *inStepValueRandomness4Sock = bke::node_find_socket(
      node, SOCK_IN, "Step Value Randomness 4");
  bNodeSocket *inEllipseWidthRandomness4Sock = bke::node_find_socket(
      node, SOCK_IN, "Ellipse Width Randomness 4");
  bNodeSocket *inEllipseHeightRandomness4Sock = bke::node_find_socket(
      node, SOCK_IN, "Ellipse Height Randomness 4");
  bNodeSocket *inInflectionPointRandomness4Sock = bke::node_find_socket(
      node, SOCK_IN, "Inflection Point Randomness 4");

  /* Outputs */
  bNodeSocket *outR_sphereFieldSock = bke::node_find_socket(node, SOCK_OUT, "R_sphere Field");
  bNodeSocket *outR_gonParameterFieldSock = bke::node_find_socket(
      node, SOCK_OUT, "R_gon Parameter Field");
  bNodeSocket *outMaxUnitParameterFieldSock = bke::node_find_socket(
      node, SOCK_OUT, "Max Unit Parameter Field");
  bNodeSocket *outSegmentIDFieldSock = bke::node_find_socket(node, SOCK_OUT, "Segment ID Field");
  bNodeSocket *outIndexFieldSock = bke::node_find_socket(node, SOCK_OUT, "Index Field");
  bNodeSocket *outIndexFieldWSock = bke::node_find_socket(node, SOCK_OUT, "Index Field W");
  bNodeSocket *outPositionFieldSock = bke::node_find_socket(node, SOCK_OUT, "Position Field");
  bNodeSocket *outPositionFieldWSock = bke::node_find_socket(node, SOCK_OUT, "Position Field W");
  bNodeSocket *outR_sphereCoordinatesSock = bke::node_find_socket(
      node, SOCK_OUT, "R_sphere Coordinates");
  bNodeSocket *outR_sphereCoordinatesWSock = bke::node_find_socket(
      node, SOCK_OUT, "R_sphere Coordinates W");

  const NodeTexRaiko &storage = node_storage(*node);

  bke::node_set_socket_availability(
      ntree, inSmoothnessSock, storage.mode == SHD_RAIKO_SMOOTH_MINIMUM);

  /* R-sphere */
  node_sock_label(inR_gonSidesSock, "R-gon Sides");
  node_sock_label(inR_gonRoundnessSock, "R-gon Roundness");
  node_sock_label(inR_gonExponentSock, "R-gon Exponent");

  /* Randomize R-sphere */
  node_sock_label(inR_gonSidesRandomnessSock, "R-gon Sides Randomness");
  node_sock_label(inR_gonRoundnessRandomnessSock, "R-gon Roundness Randomness");
  node_sock_label(inR_gonExponentRandomnessSock, "R-gon Exponent Randomness");

  /* Randomize Transform */
  bke::node_set_socket_availability(
      ntree, inScaleTransformRandomnessSock, bool(storage.uniform_scale_randomness) == false);
  if (bool(storage.uniform_scale_randomness) == true) {
    node_sock_label(inScaleTransformWRandomnessSock, "Scale Transform Randomness");
  }
  else {
    node_sock_label_clear(inScaleTransformWRandomnessSock);
  }

  /* Add Noise */
  bke::node_set_socket_availability(
      ntree, inNoiseCoordinatesStrength1Sock, storage.mode != SHD_RAIKO_ADDITIVE);
  bke::node_set_socket_availability(
      ntree, inNoiseCoordinatesStrength2Sock, storage.mode != SHD_RAIKO_ADDITIVE);

  /* Grid */
  bke::node_set_socket_availability(ntree, inGridVector1Sock, storage.grid_dimensions >= 2);
  bke::node_set_socket_availability(
      ntree, inGridVectorW1Sock, (storage.grid_dimensions == 1) || (storage.grid_dimensions == 4));
  if (storage.grid_dimensions == 1) {
    node_sock_label(inGridVectorW1Sock, "Grid Vector 1");
  }
  else {
    node_sock_label_clear(inGridVectorW1Sock);
  }
  bke::node_set_socket_availability(ntree, inGridVector2Sock, storage.grid_dimensions >= 2);
  bke::node_set_socket_availability(ntree, inGridVectorW2Sock, storage.grid_dimensions == 4);
  bke::node_set_socket_availability(ntree, inGridVector3Sock, storage.grid_dimensions >= 3);
  bke::node_set_socket_availability(ntree, inGridVectorW3Sock, storage.grid_dimensions == 4);
  bke::node_set_socket_availability(ntree, inGridVector4Sock, storage.grid_dimensions == 4);
  bke::node_set_socket_availability(ntree, inGridVectorW4Sock, storage.grid_dimensions == 4);

  /* Chained Elliptical Remap */
  /* Randomize Chained Elliptical Remap */
  bool step_1_is_available = storage.mode == SHD_RAIKO_ADDITIVE;
  bke::node_set_socket_availability(ntree, inStepCenter1Sock, step_1_is_available);
  bke::node_set_socket_availability(ntree, inStepWidth1Sock, step_1_is_available);
  bke::node_set_socket_availability(ntree, inStepValue1Sock, step_1_is_available);
  bke::node_set_socket_availability(ntree, inEllipseWidth1Sock, step_1_is_available);
  bke::node_set_socket_availability(ntree, inEllipseHeight1Sock, step_1_is_available);
  bke::node_set_socket_availability(ntree, inInflectionPoint1Sock, step_1_is_available);
  bke::node_set_socket_availability(ntree, inStepCenterRandomness1Sock, step_1_is_available);
  bke::node_set_socket_availability(ntree, inStepWidthRandomness1Sock, step_1_is_available);
  bke::node_set_socket_availability(ntree, inStepValueRandomness1Sock, step_1_is_available);
  bke::node_set_socket_availability(ntree, inEllipseWidthRandomness1Sock, step_1_is_available);
  bke::node_set_socket_availability(ntree, inEllipseHeightRandomness1Sock, step_1_is_available);
  bke::node_set_socket_availability(ntree, inInflectionPointRandomness1Sock, step_1_is_available);
  bool step_2_is_available = (storage.step_count >= 2) && (storage.mode == SHD_RAIKO_ADDITIVE);
  bke::node_set_socket_availability(ntree, inStepCenter2Sock, step_2_is_available);
  bke::node_set_socket_availability(ntree, inStepWidth2Sock, step_2_is_available);
  bke::node_set_socket_availability(ntree, inStepValue2Sock, step_2_is_available);
  bke::node_set_socket_availability(ntree, inEllipseWidth2Sock, step_2_is_available);
  bke::node_set_socket_availability(ntree, inEllipseHeight2Sock, step_2_is_available);
  bke::node_set_socket_availability(ntree, inInflectionPoint2Sock, step_2_is_available);
  bke::node_set_socket_availability(ntree, inStepCenterRandomness2Sock, step_2_is_available);
  bke::node_set_socket_availability(ntree, inStepWidthRandomness2Sock, step_2_is_available);
  bke::node_set_socket_availability(ntree, inStepValueRandomness2Sock, step_2_is_available);
  bke::node_set_socket_availability(ntree, inEllipseWidthRandomness2Sock, step_2_is_available);
  bke::node_set_socket_availability(ntree, inEllipseHeightRandomness2Sock, step_2_is_available);
  bke::node_set_socket_availability(ntree, inInflectionPointRandomness2Sock, step_2_is_available);
  bool step_3_is_available = (storage.step_count >= 3) && (storage.mode == SHD_RAIKO_ADDITIVE);
  bke::node_set_socket_availability(ntree, inStepCenter3Sock, step_3_is_available);
  bke::node_set_socket_availability(ntree, inStepWidth3Sock, step_3_is_available);
  bke::node_set_socket_availability(ntree, inStepValue3Sock, step_3_is_available);
  bke::node_set_socket_availability(ntree, inEllipseWidth3Sock, step_3_is_available);
  bke::node_set_socket_availability(ntree, inEllipseHeight3Sock, step_3_is_available);
  bke::node_set_socket_availability(ntree, inInflectionPoint3Sock, step_3_is_available);
  bke::node_set_socket_availability(ntree, inStepCenterRandomness3Sock, step_3_is_available);
  bke::node_set_socket_availability(ntree, inStepWidthRandomness3Sock, step_3_is_available);
  bke::node_set_socket_availability(ntree, inStepValueRandomness3Sock, step_3_is_available);
  bke::node_set_socket_availability(ntree, inEllipseWidthRandomness3Sock, step_3_is_available);
  bke::node_set_socket_availability(ntree, inEllipseHeightRandomness3Sock, step_3_is_available);
  bke::node_set_socket_availability(ntree, inInflectionPointRandomness3Sock, step_3_is_available);
  bool step_4_is_available = (storage.step_count >= 4) && (storage.mode == SHD_RAIKO_ADDITIVE);
  bke::node_set_socket_availability(ntree, inStepCenter4Sock, step_4_is_available);
  bke::node_set_socket_availability(ntree, inStepWidth4Sock, step_4_is_available);
  bke::node_set_socket_availability(ntree, inStepValue4Sock, step_4_is_available);
  bke::node_set_socket_availability(ntree, inEllipseWidth4Sock, step_4_is_available);
  bke::node_set_socket_availability(ntree, inEllipseHeight4Sock, step_4_is_available);
  bke::node_set_socket_availability(ntree, inInflectionPoint4Sock, step_4_is_available);
  bke::node_set_socket_availability(ntree, inStepCenterRandomness4Sock, step_4_is_available);
  bke::node_set_socket_availability(ntree, inStepWidthRandomness4Sock, step_4_is_available);
  bke::node_set_socket_availability(ntree, inStepValueRandomness4Sock, step_4_is_available);
  bke::node_set_socket_availability(ntree, inEllipseWidthRandomness4Sock, step_4_is_available);
  bke::node_set_socket_availability(ntree, inEllipseHeightRandomness4Sock, step_4_is_available);
  bke::node_set_socket_availability(ntree, inInflectionPointRandomness4Sock, step_4_is_available);

  /* Outputs */
  node_sock_label(outR_sphereFieldSock, "R-sphere Field");
  bke::node_set_socket_availability(
      ntree, outR_gonParameterFieldSock, storage.mode != SHD_RAIKO_ADDITIVE);
  node_sock_label(outR_gonParameterFieldSock, "R-gon Parameter Field");
  bke::node_set_socket_availability(
      ntree, outMaxUnitParameterFieldSock, storage.mode != SHD_RAIKO_ADDITIVE);
  bke::node_set_socket_availability(
      ntree, outSegmentIDFieldSock, storage.mode != SHD_RAIKO_ADDITIVE);
  bke::node_set_socket_availability(ntree,
                                    outIndexFieldSock,
                                    (storage.grid_dimensions >= 2) &&
                                        (storage.mode != SHD_RAIKO_ADDITIVE));
  bke::node_set_socket_availability(
      ntree,
      outIndexFieldWSock,
      ((storage.grid_dimensions == 1) || (storage.grid_dimensions == 4)) &&
          (storage.mode != SHD_RAIKO_ADDITIVE));
  if (storage.grid_dimensions == 1) {
    node_sock_label(outIndexFieldWSock, "Index Field");
  }
  else {
    node_sock_label_clear(outIndexFieldWSock);
  }
  bke::node_set_socket_availability(
      ntree, outPositionFieldSock, storage.mode != SHD_RAIKO_ADDITIVE);
  bke::node_set_socket_availability(
      ntree, outPositionFieldWSock, storage.mode != SHD_RAIKO_ADDITIVE);
  bke::node_set_socket_availability(
      ntree, outR_sphereCoordinatesSock, storage.mode != SHD_RAIKO_ADDITIVE);
  bke::node_set_socket_availability(
      ntree, outR_sphereCoordinatesWSock, storage.mode != SHD_RAIKO_ADDITIVE);
  node_sock_label(outR_sphereCoordinatesSock, "R-sphere Coordinates");
  node_sock_label(outR_sphereCoordinatesWSock, "R-sphere Coordinates W");
}

#define ASSIGN_REMAP_INPUTS(X, A, B, C, D, E, F) \
  remap[A] = in_step_center_##X[i]; \
  remap[B] = in_step_width_##X[i]; \
  remap[C] = in_step_value_##X[i]; \
  remap[D] = in_ellipse_height_##X[i]; \
  remap[E] = in_ellipse_width_##X[i]; \
  remap[F] = in_inflection_point_##X[i]; \
  if (in_step_center_randomness_##X[i] != 0.0f) { \
    remap_min[index_count] = math::max(remap[A] - in_step_center_randomness_##X[i], 0.0f); \
    remap_max[index_count] = math::max(remap[A] + in_step_center_randomness_##X[i], 0.0f); \
    remap_index_list[index_count] = A; \
    ++index_count; \
  } \
  if (in_step_width_randomness_##X[i] != 0.0f) { \
    remap_min[index_count] = math::max(remap[B] - in_step_width_randomness_##X[i], 0.0f); \
    remap_max[index_count] = math::max(remap[B] + in_step_width_randomness_##X[i], 0.0f); \
    remap_index_list[index_count] = B; \
    ++index_count; \
  } \
  if (in_step_value_randomness_##X[i] != 0.0f) { \
    remap_min[index_count] = remap[C] - in_step_value_randomness_##X[i]; \
    remap_max[index_count] = remap[C] + in_step_value_randomness_##X[i]; \
    remap_index_list[index_count] = C; \
    ++index_count; \
  } \
  if (in_ellipse_height_randomness_##X[i] != 0.0f) { \
    remap_min[index_count] = math::clamp( \
        remap[D] - in_ellipse_height_randomness_##X[i], 0.0f, 1.0f); \
    remap_max[index_count] = math::clamp( \
        remap[D] + in_ellipse_height_randomness_##X[i], 0.0f, 1.0f); \
    remap_index_list[index_count] = D; \
    ++index_count; \
  } \
  if (in_ellipse_width_randomness_##X[i] != 0.0f) { \
    remap_min[index_count] = math::clamp( \
        remap[E] - in_ellipse_width_randomness_##X[i], 0.0f, 1.0f); \
    remap_max[index_count] = math::clamp( \
        remap[E] + in_ellipse_width_randomness_##X[i], 0.0f, 1.0f); \
    remap_index_list[index_count] = E; \
    ++index_count; \
  } \
  if (in_inflection_point_randomness_##X[i] != 0.0f) { \
    remap_min[index_count] = math::clamp( \
        remap[F] - in_inflection_point_randomness_##X[i], 0.0f, 1.0f); \
    remap_max[index_count] = math::clamp( \
        remap[F] + in_inflection_point_randomness_##X[i], 0.0f, 1.0f); \
    remap_index_list[index_count] = F; \
    ++index_count; \
  }

class RaikoFunction : public mf::MultiFunction {
 private:
  int mode_;
  bool normalize_r_gon_parameter_;
  bool integer_sides_;
  bool elliptical_corners_;
  bool invert_order_of_transformation_;
  bool transform_fields_noise_;
  bool transform_coordinates_noise_;
  bool uniform_scale_randomness_;
  int grid_dimensions_;
  int step_count_;

  mf::Signature signature_;

 public:
  RaikoFunction(int mode,
                bool normalize_r_gon_parameter,
                bool integer_sides,
                bool elliptical_corners,
                bool invert_order_of_transformation,
                bool transform_fields_noise,
                bool transform_coordinates_noise,
                bool uniform_scale_randomness,
                int grid_dimensions,
                int step_count)
      : mode_(mode),
        normalize_r_gon_parameter_(normalize_r_gon_parameter),
        integer_sides_(integer_sides),
        elliptical_corners_(elliptical_corners),
        invert_order_of_transformation_(invert_order_of_transformation),
        transform_fields_noise_(transform_fields_noise),
        transform_coordinates_noise_(transform_coordinates_noise),
        uniform_scale_randomness_(uniform_scale_randomness),
        grid_dimensions_(grid_dimensions),
        step_count_(step_count)
  {
    BLI_assert((mode >= SHD_RAIKO_ADDITIVE) && (mode <= SHD_RAIKO_SMOOTH_MINIMUM));
    BLI_assert((grid_dimensions >= 0) && (grid_dimensions <= 4));
    BLI_assert((step_count >= 1) && (step_count <= 4));

    signature_ = create_signature(mode, uniform_scale_randomness, grid_dimensions, step_count);
    this->set_signature(&signature_);
  }

  static mf::Signature create_signature(int mode,
                                        bool uniform_scale_randomness,
                                        int grid_dimensions,
                                        int step_count)
  {
    mf::Signature signature;
    mf::SignatureBuilder builder{"Raiko", signature};

    builder.single_input<float3>("Vector");
    builder.single_input<float>("W");
    builder.single_input<float>("Accuracy");
    builder.single_input<float>("Scale");
    if (mode == SHD_RAIKO_SMOOTH_MINIMUM) {
      builder.single_input<float>("Smoothness");
    }

    /* R-sphere */
    builder.single_input<float>("R_gon Sides");
    builder.single_input<float>("R_gon Roundness");
    builder.single_input<float>("R_gon Exponent");
    builder.single_input<float>("Sphere Exponent");

    /* Randomize R-sphere */
    builder.single_input<float>("R_gon Sides Randomness");
    builder.single_input<float>("R_gon Roundness Randomness");
    builder.single_input<float>("R_gon Exponent Randomness");
    builder.single_input<float>("Sphere Exponent Randomness");

    /* Transform */
    builder.single_input<float3>("Rotation Transform");
    builder.single_input<float3>("Scale Transform");
    builder.single_input<float>("Scale Transform W");

    /* Randomize Transform */
    builder.single_input<float3>("Rotation Transform Randomness");
    if (!uniform_scale_randomness) {
      builder.single_input<float3>("Scale Transform Randomness");
    }
    builder.single_input<float>("Scale Transform W Randomness");

    /* Add Noise */
    builder.single_input<float>("Fragmentation");
    builder.single_input<float>("Fields Strength 1");
    if (mode != SHD_RAIKO_ADDITIVE) {
      builder.single_input<float>("Coordinates Strength 1");
    }
    builder.single_input<float>("Scale 1");
    builder.single_input<float>("Detail 1");
    builder.single_input<float>("Roughness 1");
    builder.single_input<float>("Lacunarity 1");
    builder.single_input<float>("Fields Strength 2");
    if (mode != SHD_RAIKO_ADDITIVE) {
      builder.single_input<float>("Coordinates Strength 2");
    }
    builder.single_input<float>("Scale 2");
    builder.single_input<float>("Detail 2");
    builder.single_input<float>("Roughness 2");
    builder.single_input<float>("Lacunarity 2");

    /* Grid */
    if (grid_dimensions >= 2) {
      builder.single_input<float3>("Grid Vector 1");
    }
    if ((grid_dimensions == 1) || (grid_dimensions == 4)) {
      builder.single_input<float>("Grid Vector W 1");
    }
    if (grid_dimensions >= 2) {
      builder.single_input<float3>("Grid Vector 2");
    }
    if (grid_dimensions == 4) {
      builder.single_input<float>("Grid Vector W 2");
    }
    if (grid_dimensions >= 3) {
      builder.single_input<float3>("Grid Vector 3");
    }
    if (grid_dimensions == 4) {
      builder.single_input<float>("Grid Vector W 3");
    }
    if (grid_dimensions == 4) {
      builder.single_input<float3>("Grid Vector 4");
      builder.single_input<float>("Grid Vector W 4");
    }

    /* Randomize Grid */
    builder.single_input<float3>("Grid Points Translation Randomness");
    builder.single_input<float>("Grid Points Translation W Randomness");

    if (mode == SHD_RAIKO_ADDITIVE) {
      /* Chained elliptical remap */
      builder.single_input<float>("Step Center 1");
      builder.single_input<float>("Step Width 1");
      builder.single_input<float>("Step Value 1");
      builder.single_input<float>("Ellipse Height 1");
      builder.single_input<float>("Ellipse Width 1");
      builder.single_input<float>("Inflection Point 1");
      if (step_count >= 2) {
        builder.single_input<float>("Step Center 2");
        builder.single_input<float>("Step Width 2");
        builder.single_input<float>("Step Value 2");
        builder.single_input<float>("Ellipse Height 2");
        builder.single_input<float>("Ellipse Width 2");
        builder.single_input<float>("Inflection Point 2");
      }
      if (step_count >= 3) {
        builder.single_input<float>("Step Center 3");
        builder.single_input<float>("Step Width 3");
        builder.single_input<float>("Step Value 3");
        builder.single_input<float>("Ellipse Height 3");
        builder.single_input<float>("Ellipse Width 3");
        builder.single_input<float>("Inflection Point 3");
      }
      if (step_count >= 4) {
        builder.single_input<float>("Step Center 4");
        builder.single_input<float>("Step Width 4");
        builder.single_input<float>("Step Value 4");
        builder.single_input<float>("Ellipse Height 4");
        builder.single_input<float>("Ellipse Width 4");
        builder.single_input<float>("Inflection Point 4");
      }

      /* Randomize Chained Elliptical Remap */
      builder.single_input<float>("Step Center Randomness 1");
      builder.single_input<float>("Step Width Randomness 1");
      builder.single_input<float>("Step Value Randomness 1");
      builder.single_input<float>("Ellipse Height Randomness 1");
      builder.single_input<float>("Ellipse Width Randomness 1");
      builder.single_input<float>("Inflection Point Randomness 1");
      if (step_count >= 2) {
        builder.single_input<float>("Step Center Randomness 2");
        builder.single_input<float>("Step Width Randomness 2");
        builder.single_input<float>("Step Value Randomness 2");
        builder.single_input<float>("Ellipse Height Randomness 2");
        builder.single_input<float>("Ellipse Width Randomness 2");
        builder.single_input<float>("Inflection Point Randomness 2");
      }
      if (step_count >= 3) {
        builder.single_input<float>("Step Center Randomness 3");
        builder.single_input<float>("Step Width Randomness 3");
        builder.single_input<float>("Step Value Randomness 3");
        builder.single_input<float>("Ellipse Height Randomness 3");
        builder.single_input<float>("Ellipse Width Randomness 3");
        builder.single_input<float>("Inflection Point Randomness 3");
      }
      if (step_count >= 4) {
        builder.single_input<float>("Step Center Randomness 4");
        builder.single_input<float>("Step Width Randomness 4");
        builder.single_input<float>("Step Value Randomness 4");
        builder.single_input<float>("Ellipse Height Randomness 4");
        builder.single_input<float>("Ellipse Width Randomness 4");
        builder.single_input<float>("Inflection Point Randomness 4");
      }

      builder.single_output<float>("R_sphere Field", mf::ParamFlag::SupportsUnusedOutput);
    }
    else {
      builder.single_output<float>("R_sphere Field", mf::ParamFlag::SupportsUnusedOutput);
      builder.single_output<float>("R_gon Parameter Field", mf::ParamFlag::SupportsUnusedOutput);
      builder.single_output<float>("Max Unit Parameter Field",
                                   mf::ParamFlag::SupportsUnusedOutput);
      builder.single_output<float>("Segment ID Field", mf::ParamFlag::SupportsUnusedOutput);
      if (grid_dimensions >= 2) {
        builder.single_output<float3>("Index Field", mf::ParamFlag::SupportsUnusedOutput);
      }
      if ((grid_dimensions == 1) || (grid_dimensions == 4)) {
        builder.single_output<float>("Index Field W", mf::ParamFlag::SupportsUnusedOutput);
      }
      builder.single_output<float3>("Position Field", mf::ParamFlag::SupportsUnusedOutput);
      builder.single_output<float>("Position Field W", mf::ParamFlag::SupportsUnusedOutput);
      builder.single_output<float3>("R_sphere Coordinates", mf::ParamFlag::SupportsUnusedOutput);
      builder.single_output<float>("R_sphere Coordinates W", mf::ParamFlag::SupportsUnusedOutput);
    }

    return signature;
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    int param = 0;

    const VArray<float3> &in_vector = params.readonly_single_input<float3>(param++, "Vector");
    const VArray<float> &in_w = params.readonly_single_input<float>(param++, "W");
    /* Use the "Vector" and "W" VArrays as dummies. */
    const VArray<float3> &in_unused_float3 = in_vector;
    const VArray<float> &in_unused_float = in_w;
    const VArray<float> &in_accuracy = params.readonly_single_input<float>(param++, "Accuracy");
    const VArray<float> &in_scale = params.readonly_single_input<float>(param++, "Scale");
    const VArray<float> &in_smoothness = ELEM(mode_, SHD_RAIKO_SMOOTH_MINIMUM) ?
                                             params.readonly_single_input<float>(param++,
                                                                                 "Smoothness") :
                                             in_unused_float;

    /* R-sphere */
    const VArray<float> &in_r_gon_sides = params.readonly_single_input<float>(param++,
                                                                              "R_gon Sides");
    const VArray<float> &in_r_gon_roundness = params.readonly_single_input<float>(
        param++, "R_gon Roundness");
    const VArray<float> &in_r_gon_exponent = params.readonly_single_input<float>(param++,
                                                                                 "R_gon Exponent");
    const VArray<float> &in_sphere_exponent = params.readonly_single_input<float>(
        param++, "Sphere Exponent");

    /* Randomize R-sphere */
    const VArray<float> &in_r_gon_sides_randomness = params.readonly_single_input<float>(
        param++, "R_gon Sides Randomness");
    const VArray<float> &in_r_gon_roundness_randomness = params.readonly_single_input<float>(
        param++, "R_gon Roundness Randomness");
    const VArray<float> &in_r_gon_exponent_randomness = params.readonly_single_input<float>(
        param++, "R_gon Exponent Randomness");
    const VArray<float> &in_sphere_exponent_randomness = params.readonly_single_input<float>(
        param++, "Sphere Exponent Randomness");

    /* Transform */
    const VArray<float3> &in_transform_rotation = params.readonly_single_input<float3>(
        param++, "Rotation Transform");
    const VArray<float3> &in_transform_scale = params.readonly_single_input<float3>(
        param++, "Scale Transform");
    const VArray<float> &in_transform_scale_w = params.readonly_single_input<float>(
        param++, "Scale Transform W");

    /* Randomize Transform */
    const VArray<float3> &in_transform_rotation_randomness = params.readonly_single_input<float3>(
        param++, "Rotation Transform Randomness");
    const VArray<float3> &in_transform_scale_randomness = (!uniform_scale_randomness_) ?
                                                              params.readonly_single_input<float3>(
                                                                  param++,
                                                                  "Scale Transform Randomness") :
                                                              in_unused_float3;
    const VArray<float> &in_transform_scale_w_randomness = params.readonly_single_input<float>(
        param++, "Scale Transform W Randomness");

    /* Add Noise */
    const VArray<float> &in_noise_fragmentation = params.readonly_single_input<float>(
        param++, "Fragmentation");
    const VArray<float> &in_noise_fields_strength_1 = params.readonly_single_input<float>(
        param++, "Fields Strength 1");
    const VArray<float> &in_noise_coordinates_strength_1 = (!ELEM(mode_, SHD_RAIKO_ADDITIVE)) ?
                                                               params.readonly_single_input<float>(
                                                                   param++,
                                                                   "Coordinates Strength 1") :
                                                               in_unused_float;
    const VArray<float> &in_noise_scale_1 = params.readonly_single_input<float>(param++,
                                                                                "Scale 1");
    const VArray<float> &in_noise_detail_1 = params.readonly_single_input<float>(param++,
                                                                                 "Detail 1");
    const VArray<float> &in_noise_roughness_1 = params.readonly_single_input<float>(param++,
                                                                                    "Roughness 1");
    const VArray<float> &in_noise_lacunarity_1 = params.readonly_single_input<float>(
        param++, "Lacunarity 1");
    const VArray<float> &in_noise_fields_strength_2 = params.readonly_single_input<float>(
        param++, "Fields Strength 2");
    const VArray<float> &in_noise_coordinates_strength_2 = (!ELEM(mode_, SHD_RAIKO_ADDITIVE)) ?
                                                               params.readonly_single_input<float>(
                                                                   param++,
                                                                   "Coordinates Strength 2") :
                                                               in_unused_float;
    const VArray<float> &in_noise_scale_2 = params.readonly_single_input<float>(param++,
                                                                                "Scale 2");
    const VArray<float> &in_noise_detail_2 = params.readonly_single_input<float>(param++,
                                                                                 "Detail 2");
    const VArray<float> &in_noise_roughness_2 = params.readonly_single_input<float>(param++,
                                                                                    "Roughness 2");
    const VArray<float> &in_noise_lacunarity_2 = params.readonly_single_input<float>(
        param++, "Lacunarity 2");

    /* Grid */
    const VArray<float3> &in_grid_vector_1 = ELEM(grid_dimensions_, 2, 3, 4) ?
                                                 params.readonly_single_input<float3>(
                                                     param++, "Grid Vector 1") :
                                                 in_unused_float3;
    const VArray<float> &in_grid_vector_w_1 = ELEM(grid_dimensions_, 1, 4) ?
                                                  params.readonly_single_input<float>(
                                                      param++, "Grid Vector W 1") :
                                                  in_unused_float;
    const VArray<float3> &in_grid_vector_2 = ELEM(grid_dimensions_, 2, 3, 4) ?
                                                 params.readonly_single_input<float3>(
                                                     param++, "Grid Vector 2") :
                                                 in_unused_float3;
    const VArray<float> &in_grid_vector_w_2 = ELEM(grid_dimensions_, 4) ?
                                                  params.readonly_single_input<float>(
                                                      param++, "Grid Vector W 2") :
                                                  in_unused_float;
    const VArray<float3> &in_grid_vector_3 = ELEM(grid_dimensions_, 3, 4) ?
                                                 params.readonly_single_input<float3>(
                                                     param++, "Grid Vector 3") :
                                                 in_unused_float3;
    const VArray<float> &in_grid_vector_w_3 = ELEM(grid_dimensions_, 4) ?
                                                  params.readonly_single_input<float>(
                                                      param++, "Grid Vector W 3") :
                                                  in_unused_float;
    const VArray<float3> &in_grid_vector_4 = ELEM(grid_dimensions_, 4) ?
                                                 params.readonly_single_input<float3>(
                                                     param++, "Grid Vector 4") :
                                                 in_unused_float3;
    const VArray<float> &in_grid_vector_w_4 = ELEM(grid_dimensions_, 4) ?
                                                  params.readonly_single_input<float>(
                                                      param++, "Grid Vector W 4") :
                                                  in_unused_float;

    /* Randomize Grid */
    const VArray<float3> &in_grid_points_translation_randomness =
        params.readonly_single_input<float3>(param++, "Grid Points Translation Randomness");
    const VArray<float> &in_grid_points_translation_w_randomness =
        params.readonly_single_input<float>(param++, "Grid Points Translation W Randomness");

    /* Chained Elliptical Remap */
    bool step_1_is_available = ELEM(mode_, SHD_RAIKO_ADDITIVE);
    const VArray<float> &in_step_center_1 = step_1_is_available ?
                                                params.readonly_single_input<float>(
                                                    param++, "Step Center 1") :
                                                in_unused_float;
    const VArray<float> &in_step_width_1 = step_1_is_available ?
                                               params.readonly_single_input<float>(
                                                   param++, "Step Width 1") :
                                               in_unused_float;
    const VArray<float> &in_step_value_1 = step_1_is_available ?
                                               params.readonly_single_input<float>(
                                                   param++, "Step Value 1") :
                                               in_unused_float;
    const VArray<float> &in_ellipse_height_1 = step_1_is_available ?
                                                   params.readonly_single_input<float>(
                                                       param++, "Ellipse Height 1") :
                                                   in_unused_float;
    const VArray<float> &in_ellipse_width_1 = step_1_is_available ?
                                                  params.readonly_single_input<float>(
                                                      param++, "Ellipse Width 1") :
                                                  in_unused_float;
    const VArray<float> &in_inflection_point_1 = step_1_is_available ?
                                                     params.readonly_single_input<float>(
                                                         param++, "Inflection Point 1") :
                                                     in_unused_float;
    bool step_2_is_available = ELEM(mode_, SHD_RAIKO_ADDITIVE) && ELEM(step_count_, 2, 3, 4);
    const VArray<float> &in_step_center_2 = step_2_is_available ?
                                                params.readonly_single_input<float>(
                                                    param++, "Step Center 2") :
                                                in_unused_float;
    const VArray<float> &in_step_width_2 = step_2_is_available ?
                                               params.readonly_single_input<float>(
                                                   param++, "Step Width 2") :
                                               in_unused_float;
    const VArray<float> &in_step_value_2 = step_2_is_available ?
                                               params.readonly_single_input<float>(
                                                   param++, "Step Value 2") :
                                               in_unused_float;
    const VArray<float> &in_ellipse_height_2 = step_2_is_available ?
                                                   params.readonly_single_input<float>(
                                                       param++, "Ellipse Height 2") :
                                                   in_unused_float;
    const VArray<float> &in_ellipse_width_2 = step_2_is_available ?
                                                  params.readonly_single_input<float>(
                                                      param++, "Ellipse Width 2") :
                                                  in_unused_float;
    const VArray<float> &in_inflection_point_2 = step_2_is_available ?
                                                     params.readonly_single_input<float>(
                                                         param++, "Inflection Point 2") :
                                                     in_unused_float;
    bool step_3_is_available = ELEM(mode_, SHD_RAIKO_ADDITIVE) && ELEM(step_count_, 3, 4);
    const VArray<float> &in_step_center_3 = step_3_is_available ?
                                                params.readonly_single_input<float>(
                                                    param++, "Step Center 3") :
                                                in_unused_float;
    const VArray<float> &in_step_width_3 = step_3_is_available ?
                                               params.readonly_single_input<float>(
                                                   param++, "Step Width 3") :
                                               in_unused_float;
    const VArray<float> &in_step_value_3 = step_3_is_available ?
                                               params.readonly_single_input<float>(
                                                   param++, "Step Value 3") :
                                               in_unused_float;
    const VArray<float> &in_ellipse_height_3 = step_3_is_available ?
                                                   params.readonly_single_input<float>(
                                                       param++, "Ellipse Height 3") :
                                                   in_unused_float;
    const VArray<float> &in_ellipse_width_3 = step_3_is_available ?
                                                  params.readonly_single_input<float>(
                                                      param++, "Ellipse Width 3") :
                                                  in_unused_float;
    const VArray<float> &in_inflection_point_3 = step_3_is_available ?
                                                     params.readonly_single_input<float>(
                                                         param++, "Inflection Point 3") :
                                                     in_unused_float;
    bool step_4_is_available = ELEM(mode_, SHD_RAIKO_ADDITIVE) && ELEM(step_count_, 4);
    const VArray<float> &in_step_center_4 = step_4_is_available ?
                                                params.readonly_single_input<float>(
                                                    param++, "Step Center 4") :
                                                in_unused_float;
    const VArray<float> &in_step_width_4 = step_4_is_available ?
                                               params.readonly_single_input<float>(
                                                   param++, "Step Width 4") :
                                               in_unused_float;
    const VArray<float> &in_step_value_4 = step_4_is_available ?
                                               params.readonly_single_input<float>(
                                                   param++, "Step Value 4") :
                                               in_unused_float;
    const VArray<float> &in_ellipse_height_4 = step_4_is_available ?
                                                   params.readonly_single_input<float>(
                                                       param++, "Ellipse Height 4") :
                                                   in_unused_float;
    const VArray<float> &in_ellipse_width_4 = step_4_is_available ?
                                                  params.readonly_single_input<float>(
                                                      param++, "Ellipse Width 4") :
                                                  in_unused_float;
    const VArray<float> &in_inflection_point_4 = step_4_is_available ?
                                                     params.readonly_single_input<float>(
                                                         param++, "Inflection Point 4") :
                                                     in_unused_float;

    /* Randomize Chained Elliptical Remap */
    const VArray<float> &in_step_center_randomness_1 = step_1_is_available ?
                                                           params.readonly_single_input<float>(
                                                               param++,
                                                               "Step Center Randomness 1") :
                                                           in_unused_float;
    const VArray<float> &in_step_width_randomness_1 = step_1_is_available ?
                                                          params.readonly_single_input<float>(
                                                              param++, "Step Width Randomness 1") :
                                                          in_unused_float;
    const VArray<float> &in_step_value_randomness_1 = step_1_is_available ?
                                                          params.readonly_single_input<float>(
                                                              param++, "Step Value Randomness 1") :
                                                          in_unused_float;
    const VArray<float> &in_ellipse_height_randomness_1 = step_1_is_available ?
                                                              params.readonly_single_input<float>(
                                                                  param++,
                                                                  "Ellipse Height Randomness 1") :
                                                              in_unused_float;
    const VArray<float> &in_ellipse_width_randomness_1 = step_1_is_available ?
                                                             params.readonly_single_input<float>(
                                                                 param++,
                                                                 "Ellipse Width Randomness 1") :
                                                             in_unused_float;
    const VArray<float> &in_inflection_point_randomness_1 =
        step_1_is_available ?
            params.readonly_single_input<float>(param++, "Inflection Point Randomness 1") :
            in_unused_float;
    const VArray<float> &in_step_center_randomness_2 = step_2_is_available ?
                                                           params.readonly_single_input<float>(
                                                               param++,
                                                               "Step Center Randomness 2") :
                                                           in_unused_float;
    const VArray<float> &in_step_width_randomness_2 = step_2_is_available ?
                                                          params.readonly_single_input<float>(
                                                              param++, "Step Width Randomness 2") :
                                                          in_unused_float;
    const VArray<float> &in_step_value_randomness_2 = step_2_is_available ?
                                                          params.readonly_single_input<float>(
                                                              param++, "Step Value Randomness 2") :
                                                          in_unused_float;
    const VArray<float> &in_ellipse_height_randomness_2 = step_2_is_available ?
                                                              params.readonly_single_input<float>(
                                                                  param++,
                                                                  "Ellipse Height Randomness 2") :
                                                              in_unused_float;
    const VArray<float> &in_ellipse_width_randomness_2 = step_2_is_available ?
                                                             params.readonly_single_input<float>(
                                                                 param++,
                                                                 "Ellipse Width Randomness 2") :
                                                             in_unused_float;
    const VArray<float> &in_inflection_point_randomness_2 =
        step_2_is_available ?
            params.readonly_single_input<float>(param++, "Inflection Point Randomness 2") :
            in_unused_float;
    const VArray<float> &in_step_center_randomness_3 = step_3_is_available ?
                                                           params.readonly_single_input<float>(
                                                               param++,
                                                               "Step Center Randomness 3") :
                                                           in_unused_float;
    const VArray<float> &in_step_width_randomness_3 = step_3_is_available ?
                                                          params.readonly_single_input<float>(
                                                              param++, "Step Width Randomness 3") :
                                                          in_unused_float;
    const VArray<float> &in_step_value_randomness_3 = step_3_is_available ?
                                                          params.readonly_single_input<float>(
                                                              param++, "Step Value Randomness 3") :
                                                          in_unused_float;
    const VArray<float> &in_ellipse_height_randomness_3 = step_3_is_available ?
                                                              params.readonly_single_input<float>(
                                                                  param++,
                                                                  "Ellipse Height Randomness 3") :
                                                              in_unused_float;
    const VArray<float> &in_ellipse_width_randomness_3 = step_3_is_available ?
                                                             params.readonly_single_input<float>(
                                                                 param++,
                                                                 "Ellipse Width Randomness 3") :
                                                             in_unused_float;
    const VArray<float> &in_inflection_point_randomness_3 =
        step_3_is_available ?
            params.readonly_single_input<float>(param++, "Inflection Point Randomness 3") :
            in_unused_float;
    const VArray<float> &in_step_center_randomness_4 = step_4_is_available ?
                                                           params.readonly_single_input<float>(
                                                               param++,
                                                               "Step Center Randomness 4") :
                                                           in_unused_float;
    const VArray<float> &in_step_width_randomness_4 = step_4_is_available ?
                                                          params.readonly_single_input<float>(
                                                              param++, "Step Width Randomness 4") :
                                                          in_unused_float;
    const VArray<float> &in_step_value_randomness_4 = step_4_is_available ?
                                                          params.readonly_single_input<float>(
                                                              param++, "Step Value Randomness 4") :
                                                          in_unused_float;
    const VArray<float> &in_ellipse_height_randomness_4 = step_4_is_available ?
                                                              params.readonly_single_input<float>(
                                                                  param++,
                                                                  "Ellipse Height Randomness 4") :
                                                              in_unused_float;
    const VArray<float> &in_ellipse_width_randomness_4 = step_4_is_available ?
                                                             params.readonly_single_input<float>(
                                                                 param++,
                                                                 "Ellipse Width Randomness 4") :
                                                             in_unused_float;
    const VArray<float> &in_inflection_point_randomness_4 =
        step_4_is_available ?
            params.readonly_single_input<float>(param++, "Inflection Point Randomness 4") :
            in_unused_float;

    MutableSpan<float> out_r_sphere_field = params.uninitialized_single_output_if_required<float>(
        param++, "R_sphere Field");
    MutableSpan<float> out_r_gon_parameter_field;
    MutableSpan<float> out_max_unit_parameter_field;
    MutableSpan<float> out_segment_id_field;
    MutableSpan<float3> out_index_field;
    MutableSpan<float> out_index_field_w;
    MutableSpan<float3> out_position_field;
    MutableSpan<float> out_position_field_w;
    MutableSpan<float3> out_r_sphere_coordinates;
    MutableSpan<float> out_r_sphere_coordinates_w;
    if (!ELEM(mode_, SHD_RAIKO_ADDITIVE)) {
      out_r_gon_parameter_field = params.uninitialized_single_output_if_required<float>(
          param++, "R_gon Parameter Field");
      out_max_unit_parameter_field = params.uninitialized_single_output_if_required<float>(
          param++, "Max Unit Parameter Field");
      out_segment_id_field = params.uninitialized_single_output_if_required<float>(
          param++, "Segment ID Field");
      if (ELEM(grid_dimensions_, 2, 3, 4)) {
        out_index_field = params.uninitialized_single_output_if_required<float3>(param++,
                                                                                 "Index Field");
      }
      if (ELEM(grid_dimensions_, 1, 4)) {
        out_index_field_w = params.uninitialized_single_output_if_required<float>(param++,
                                                                                  "Index Field W");
      }
      out_position_field = params.uninitialized_single_output_if_required<float3>(
          param++, "Position Field");
      out_position_field_w = params.uninitialized_single_output_if_required<float>(
          param++, "Position Field W");
      out_r_sphere_coordinates = params.uninitialized_single_output_if_required<float3>(
          param++, "R_sphere Coordinates");
      out_r_sphere_coordinates_w = params.uninitialized_single_output_if_required<float>(
          param++, "R_sphere Coordinates W");
    }

    const bool calc_r_sphere_field = !out_r_sphere_field.is_empty();
    const bool calc_r_gon_parameter_field = !out_r_gon_parameter_field.is_empty();
    const bool calc_max_unit_parameter_field = !out_max_unit_parameter_field.is_empty();
    const bool calc_segment_id_field = !out_segment_id_field.is_empty();
    const bool calc_index_field = !out_index_field.is_empty();
    const bool calc_index_field_w = !out_index_field_w.is_empty();
    const bool calc_position_field = !out_position_field.is_empty();
    const bool calc_position_field_w = !out_position_field_w.is_empty();
    const bool calc_r_sphere_coordinates = !out_r_sphere_coordinates.is_empty();
    const bool calc_r_sphere_coordinates_w = !out_r_sphere_coordinates_w.is_empty();

    mask.foreach_index([&](const int64_t i) {
      noise::DeterministicVariables dv;

      dv.calculate_r_sphere_field = calc_r_sphere_field || calc_r_gon_parameter_field ||
                                    calc_max_unit_parameter_field ||
                                    (!ELEM(mode_, SHD_RAIKO_CLOSEST));
      dv.calculate_r_gon_parameter_field = calc_r_gon_parameter_field;
      dv.calculate_max_unit_parameter_field = calc_max_unit_parameter_field;
      dv.calculate_coordinates_outputs = (calc_r_sphere_coordinates ||
                                          calc_r_sphere_coordinates_w) &&
                                         (!ELEM(mode_, SHD_RAIKO_ADDITIVE));

      dv.mode = mode_;
      dv.normalize_r_gon_parameter = normalize_r_gon_parameter_;
      dv.integer_sides = integer_sides_;
      dv.elliptical_corners = elliptical_corners_;
      dv.invert_order_of_transformation = invert_order_of_transformation_;
      dv.transform_fields_noise = transform_fields_noise_;
      dv.transform_coordinates_noise = transform_coordinates_noise_;
      dv.uniform_scale_randomness = uniform_scale_randomness_;
      dv.grid_dimensions = grid_dimensions_;
      dv.step_count = step_count_;

      dv.accuracy = in_accuracy[i];
      dv.scale = in_scale[i];
      dv.coord = dv.scale * float4(in_vector[i].x, in_vector[i].y, in_vector[i].z, in_w[i]);
      dv.smoothness = in_smoothness[i];
      dv.smoothness_non_zero = dv.smoothness != 0.0f;
      /*r_sphere[0] == r_gon_sides; r_sphere[1] == r_gon_roundness; r_sphere[2] == r_gon_exponent;
       * r_sphere[3] == sphere_exponent;*/
      float r_sphere[4] = {
          in_r_gon_sides[i], in_r_gon_roundness[i], in_r_gon_exponent[i], in_sphere_exponent[i]};
      float r_sphere_min[4];
      float r_sphere_max[4];
      int r_sphere_index_list[4];
      int r_sphere_index_count;
      int index_count = 0;
      if (in_r_gon_sides_randomness[i] != 0.0f) {
        r_sphere_min[index_count] = math::max(r_sphere[0] - in_r_gon_sides_randomness[i], 2.0f);
        r_sphere_max[index_count] = math::max(r_sphere[0] + in_r_gon_sides_randomness[i], 2.0f);
        r_sphere_index_list[index_count] = 0;
        ++index_count;
      }
      if (in_r_gon_roundness_randomness[i] != 0.0f) {
        r_sphere_min[index_count] = math::clamp(
            r_sphere[1] - in_r_gon_roundness_randomness[i], 0.0f, 1.0f);
        r_sphere_max[index_count] = math::clamp(
            r_sphere[1] + in_r_gon_roundness_randomness[i], 0.0f, 1.0f);
        r_sphere_index_list[index_count] = 1;
        ++index_count;
      }
      if (in_r_gon_exponent_randomness[i] != 0.0f) {
        r_sphere_min[index_count] = math::max(r_sphere[2] - in_r_gon_exponent_randomness[i], 0.0f);
        r_sphere_max[index_count] = math::max(r_sphere[2] + in_r_gon_exponent_randomness[i], 0.0f);
        r_sphere_index_list[index_count] = 2;
        ++index_count;
      }
      if (in_sphere_exponent_randomness[i] != 0.0f) {
        r_sphere_min[index_count] = math::max(r_sphere[3] - in_sphere_exponent_randomness[i],
                                              0.0f);
        r_sphere_max[index_count] = math::max(r_sphere[3] + in_sphere_exponent_randomness[i],
                                              0.0f);
        r_sphere_index_list[index_count] = 3;
        ++index_count;
      }
      r_sphere_index_count = index_count;
      float translation_rotation[7] = {0.0f,
                                       0.0f,
                                       0.0f,
                                       0.0f,
                                       in_transform_rotation[i].x,
                                       in_transform_rotation[i].y,
                                       in_transform_rotation[i].z};
      float translation_rotation_min[7];
      float translation_rotation_max[7];
      int translation_rotation_index_list[7];
      int translation_rotation_index_count;
      index_count = 0;
      if (in_grid_points_translation_randomness[i].x != 0.0f) {
        translation_rotation_min[index_count] = translation_rotation[0] -
                                                in_grid_points_translation_randomness[i].x;
        translation_rotation_max[index_count] = translation_rotation[0] +
                                                in_grid_points_translation_randomness[i].x;
        translation_rotation_index_list[index_count] = 0;
        ++index_count;
      }
      if (in_grid_points_translation_randomness[i].y != 0.0f) {
        translation_rotation_min[index_count] = translation_rotation[1] -
                                                in_grid_points_translation_randomness[i].y;
        translation_rotation_max[index_count] = translation_rotation[1] +
                                                in_grid_points_translation_randomness[i].y;
        translation_rotation_index_list[index_count] = 1;
        ++index_count;
      }
      if (in_grid_points_translation_randomness[i].z != 0.0f) {
        translation_rotation_min[index_count] = translation_rotation[2] -
                                                in_grid_points_translation_randomness[i].z;
        translation_rotation_max[index_count] = translation_rotation[2] +
                                                in_grid_points_translation_randomness[i].z;
        translation_rotation_index_list[index_count] = 2;
        ++index_count;
      }
      if (in_grid_points_translation_w_randomness[i] != 0.0f) {
        translation_rotation_min[index_count] = translation_rotation[3] -
                                                in_grid_points_translation_w_randomness[i];
        translation_rotation_max[index_count] = translation_rotation[3] +
                                                in_grid_points_translation_w_randomness[i];
        translation_rotation_index_list[index_count] = 3;
        ++index_count;
      }
      if (in_transform_rotation_randomness[i].x != 0.0f) {
        translation_rotation_min[index_count] = translation_rotation[4] -
                                                in_transform_rotation_randomness[i].x;
        translation_rotation_max[index_count] = translation_rotation[4] +
                                                in_transform_rotation_randomness[i].x;
        translation_rotation_index_list[index_count] = 4;
        ++index_count;
      }
      if (in_transform_rotation_randomness[i].y != 0.0f) {
        translation_rotation_min[index_count] = translation_rotation[5] -
                                                in_transform_rotation_randomness[i].y;
        translation_rotation_max[index_count] = translation_rotation[5] +
                                                in_transform_rotation_randomness[i].y;
        translation_rotation_index_list[index_count] = 5;
        ++index_count;
      }
      if (in_transform_rotation_randomness[i].z != 0.0f) {
        translation_rotation_min[index_count] = translation_rotation[6] -
                                                in_transform_rotation_randomness[i].z;
        translation_rotation_max[index_count] = translation_rotation[6] +
                                                in_transform_rotation_randomness[i].z;
        translation_rotation_index_list[index_count] = 6;
        ++index_count;
      }
      translation_rotation_index_count = index_count;
      float scale[4] = {in_transform_scale[i].x,
                        in_transform_scale[i].y,
                        in_transform_scale[i].z,
                        in_transform_scale_w[i]};
      float scale_randomness[4];
      int scale_index_list[4];
      int scale_index_count;
      index_count = 0;
      if (!dv.uniform_scale_randomness) {
        if (in_transform_scale_randomness[i].x != 0.0f) {
          scale_randomness[index_count] = in_transform_scale_randomness[i].x;
          scale_index_list[index_count] = 0;
          ++index_count;
        }
        if (in_transform_scale_randomness[i].y != 0.0f) {
          scale_randomness[index_count] = in_transform_scale_randomness[i].y;
          scale_index_list[index_count] = 1;
          ++index_count;
        }
        if (in_transform_scale_randomness[i].z != 0.0f) {
          scale_randomness[index_count] = in_transform_scale_randomness[i].z;
          scale_index_list[index_count] = 2;
          ++index_count;
        }
      }
      if (in_transform_scale_w_randomness[i] != 0.0f) {
        scale_randomness[index_count] = in_transform_scale_w_randomness[i];
        scale_index_list[index_count] = 3;
        ++index_count;
      }
      scale_index_count = index_count;
      dv.noise_fragmentation = in_noise_fragmentation[i];
      dv.noise_fields_strength_1 = in_noise_fields_strength_1[i];
      dv.noise_coordinates_strength_1 = in_noise_coordinates_strength_1[i] *
                                        float(dv.calculate_coordinates_outputs);
      dv.noise_scale_1 = in_noise_scale_1[i];
      dv.noise_detail_1 = in_noise_detail_1[i];
      dv.noise_roughness_1 = in_noise_roughness_1[i];
      dv.noise_lacunarity_1 = in_noise_lacunarity_1[i];
      dv.noise_fields_strength_2 = in_noise_fields_strength_2[i];
      dv.noise_coordinates_strength_2 = in_noise_coordinates_strength_2[i] *
                                        float(dv.calculate_coordinates_outputs);
      dv.noise_scale_2 = in_noise_scale_2[i];
      dv.noise_detail_2 = in_noise_detail_2[i];
      dv.noise_roughness_2 = in_noise_roughness_2[i];
      dv.noise_lacunarity_2 = in_noise_lacunarity_2[i];
      dv.noise_fragmentation_non_zero = dv.noise_fragmentation != 0.0f;
      dv.calculate_fields_noise_1 = dv.noise_fields_strength_1 != 0.0f;
      dv.calculate_fields_noise_2 = dv.noise_fields_strength_2 != 0.0f;
      dv.calculate_coordinates_noise_1 = (dv.noise_coordinates_strength_1 != 0.0f) &&
                                         (!(dv.calculate_fields_noise_1 &&
                                            (dv.transform_fields_noise ==
                                             dv.transform_coordinates_noise)));
      dv.calculate_coordinates_noise_2 = (dv.noise_coordinates_strength_2 != 0.0f) &&
                                         (!(dv.calculate_fields_noise_2 &&
                                            (dv.transform_fields_noise ==
                                             dv.transform_coordinates_noise)));
      if (dv.grid_dimensions == 1) {
        dv.grid_vector_1.x = in_grid_vector_w_1[i];
      }
      else {
        dv.grid_vector_1 = float4(in_grid_vector_1[i].x,
                                  in_grid_vector_1[i].y,
                                  in_grid_vector_1[i].z,
                                  in_grid_vector_w_1[i]);
        dv.grid_vector_2 = float4(in_grid_vector_2[i].x,
                                  in_grid_vector_2[i].y,
                                  in_grid_vector_2[i].z,
                                  in_grid_vector_w_2[i]);
        dv.grid_vector_3 = float4(in_grid_vector_3[i].x,
                                  in_grid_vector_3[i].y,
                                  in_grid_vector_3[i].z,
                                  in_grid_vector_w_3[i]);
        dv.grid_vector_4 = float4(in_grid_vector_4[i].x,
                                  in_grid_vector_4[i].y,
                                  in_grid_vector_4[i].z,
                                  in_grid_vector_w_4[i]);
      }
      float remap[24];
      float remap_min[24];
      float remap_max[24];
      int remap_index_list[24];
      int remap_index_count;
      index_count = 0;
      if (dv.mode == SHD_RAIKO_ADDITIVE) {
        switch (dv.step_count) {
          case 4: {
            ASSIGN_REMAP_INPUTS(4, 18, 19, 20, 21, 22, 23)
            ATTR_FALLTHROUGH;
          }
          case 3: {
            ASSIGN_REMAP_INPUTS(3, 12, 13, 14, 15, 16, 17)
            ATTR_FALLTHROUGH;
          }
          case 2: {
            ASSIGN_REMAP_INPUTS(2, 6, 7, 8, 9, 10, 11)
            ATTR_FALLTHROUGH;
          }
          case 1: {
            ASSIGN_REMAP_INPUTS(1, 0, 1, 2, 3, 4, 5)
            break;
          }
        }
      }
      remap_index_count = index_count;

      noise::OutVariables ov = noise::raiko_select_grid_dimensions(
          dv,
          r_sphere,
          r_sphere_min,
          r_sphere_max,
          r_sphere_index_list,
          r_sphere_index_count,
          translation_rotation,
          translation_rotation_min,
          translation_rotation_max,
          translation_rotation_index_list,
          translation_rotation_index_count,
          scale,
          scale_randomness,
          scale_index_list,
          scale_index_count,
          remap,
          remap_min,
          remap_max,
          remap_index_list,
          remap_index_count);

      if (calc_r_sphere_field) {
        out_r_sphere_field[i] = ov.out_r_sphere_field;
      }
      if (calc_r_gon_parameter_field) {
        out_r_gon_parameter_field[i] = ov.r_gon_parameter_field;
      }
      if (calc_max_unit_parameter_field) {
        out_max_unit_parameter_field[i] = ov.max_unit_parameter_field;
      }
      if (calc_segment_id_field) {
        out_segment_id_field[i] = ov.segment_id_field;
      }
      if (calc_index_field) {
        out_index_field[i] = float3(
            ov.out_index_field.x, ov.out_index_field.y, ov.out_index_field.z);
      }
      if (calc_index_field_w) {
        out_index_field_w[i] = (dv.grid_dimensions == 1) ? ov.out_index_field.x :
                                                           ov.out_index_field.w;
      }
      if (calc_position_field) {
        out_position_field[i] = float3(
            ov.out_position_field.x, ov.out_position_field.y, ov.out_position_field.z);
      }
      if (calc_position_field_w) {
        out_position_field_w[i] = ov.out_position_field.w;
      }
      if (calc_r_sphere_coordinates) {
        out_r_sphere_coordinates[i] = float3(ov.out_r_sphere_coordinates.x,
                                             ov.out_r_sphere_coordinates.y,
                                             ov.out_r_sphere_coordinates.z);
      }
      if (calc_r_sphere_coordinates_w) {
        out_r_sphere_coordinates_w[i] = ov.out_r_sphere_coordinates.w;
      }
    });
  }

  ExecutionHints get_execution_hints() const override
  {
    ExecutionHints hints;
    hints.allocates_array = false;
    hints.min_grain_size = 50;
    return hints;
  }
};

static void sh_node_raiko_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  const NodeTexRaiko &storage = node_storage(builder.node());
  builder.construct_and_set_matching_fn<RaikoFunction>(storage.mode,
                                                       storage.normalize_r_gon_parameter,
                                                       storage.integer_sides,
                                                       storage.elliptical_corners,
                                                       storage.invert_order_of_transformation,
                                                       storage.transform_fields_noise,
                                                       storage.transform_coordinates_noise,
                                                       storage.uniform_scale_randomness,
                                                       storage.grid_dimensions,
                                                       storage.step_count);
}

}  // namespace blender::nodes::node_shader_tex_raiko_cc

void register_node_type_sh_tex_raiko()
{
  namespace file_ns = blender::nodes::node_shader_tex_raiko_cc;

  static blender::bke::bNodeType ntype;

  sh_fn_node_type_base(&ntype, SH_NODE_TEX_RAIKO, "Raiko Texture", NODE_CLASS_TEXTURE);
  ntype.declare = file_ns::sh_node_tex_raiko_declare;
  ntype.draw_buttons = file_ns::node_shader_buts_tex_raiko;
  ntype.initfunc = file_ns::node_shader_init_tex_raiko;
  blender::bke::node_type_storage(
      &ntype, "NodeTexRaiko", node_free_standard_storage, node_copy_standard_storage);
  ntype.gpu_fn = file_ns::node_shader_gpu_tex_raiko;
  ntype.updatefunc = file_ns::node_shader_update_tex_raiko;
  ntype.build_multi_function = file_ns::sh_node_raiko_build_multi_function;

  blender::bke::node_register_type(&ntype);
}
