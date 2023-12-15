/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "NOD_rna_define.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "RNA_enum_types.hh"

#ifdef WITH_OPENVDB
#  include <openvdb/tools/LevelSetFilter.h>
#endif

namespace blender::nodes::node_geo_grid_level_set_filter_cc {

static const eCustomDataType DummyMaskType = CD_PROP_FLOAT;

static void node_declare(NodeDeclarationBuilder &b)
{
  const bNode *node = b.node_or_null();
  if (!node) {
    return;
  }

  const GeometryNodeGridFilterOperation operation = GeometryNodeGridFilterOperation(node->custom1);

  grids::declare_grid_type_input(b, CD_PROP_FLOAT, "Grid");
  grids::declare_grid_type_input(b, DummyMaskType, "Mask");
  switch (operation) {
    case GEO_NODE_GRID_FILTER_MEAN:
    case GEO_NODE_GRID_FILTER_MEDIAN:
    case GEO_NODE_GRID_FILTER_GAUSSIAN:
      b.add_input<decl::Int>("Width").min(1).max(3).default_value(1);
      break;
    case GEO_NODE_GRID_FILTER_OFFSET:
      b.add_input(CD_PROP_FLOAT, "Offset");
      break;
    case GEO_NODE_GRID_FILTER_MEAN_CURVATURE:
    case GEO_NODE_GRID_FILTER_LAPLACIAN:
      /* No additional inputs. */
      break;
  }

  grids::declare_grid_type_output(b, CD_PROP_FLOAT, "Grid");
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiLayoutSetPropSep(layout, true);
  uiLayoutSetPropDecorate(layout, false);
  uiItemR(layout, ptr, "operation", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = GEO_NODE_GRID_FILTER_MEAN;
}

static bke::GVolumeGridPtr filter_grid(GeoNodeExecParams params)
{
  using GridType = typename bke::VolumeGridPtr<float>::GridType;
  using MaskType = typename bke::VolumeGridPtr<float>::GridType;
  using Converter = bke::grids::Converter<float>;

  const GeometryNodeGridFilterOperation operation = GeometryNodeGridFilterOperation(
      params.node().custom1);

  const bke::VolumeGridPtr<float> grid = grids::extract_grid_input<float>(params, "Grid");
  const bke::VolumeGridPtr<float> mask = grids::extract_grid_input<float>(params, "Mask");
  if (!grid) {
    return nullptr;
  }
  bool has_error = false;
  if (!grid->grid()->hasUniformVoxels()) {
    params.error_message_add(geo_eval_log::NodeWarningType::Error, "Grid has non-uniform scale");
    has_error = true;
  }
  if (grid->grid()->getGridClass() != openvdb::GRID_LEVEL_SET) {
    params.error_message_add(geo_eval_log::NodeWarningType::Error, "Grid must be a level set");
    has_error = true;
  }
  if (has_error) {
    return nullptr;
  }

  const bke::VolumeGridPtr<float> output_grid = grid->is_mutable() ?
                                                    grid :
                                                    bke::VolumeGridPtr<float>{grid->copy()};

  openvdb::tools::LevelSetFilter<GridType, MaskType> filter(*output_grid.grid_for_write());

  switch (operation) {
    case GEO_NODE_GRID_FILTER_MEAN: {
      const int width = params.extract_input<int>("Width");
      filter.mean(width, mask ? mask.grid().get() : nullptr);
      break;
    }
    case GEO_NODE_GRID_FILTER_MEDIAN: {
      const int width = params.extract_input<int>("Width");
      filter.median(width, mask ? mask.grid().get() : nullptr);
      break;
    }
    case GEO_NODE_GRID_FILTER_GAUSSIAN: {
      const int width = params.extract_input<int>("Width");
      filter.gaussian(width, mask ? mask.grid().get() : nullptr);
      break;
    }
    case GEO_NODE_GRID_FILTER_OFFSET: {
      const float offset = params.extract_input<float>("Offset");
      filter.offset(Converter::to_openvdb(offset), mask ? mask.grid().get() : nullptr);
      break;
    }
    case GEO_NODE_GRID_FILTER_MEAN_CURVATURE: {
      filter.meanCurvature(mask ? mask.grid().get() : nullptr);
      break;
    }
    case GEO_NODE_GRID_FILTER_LAPLACIAN: {
      filter.laplacian(mask ? mask.grid().get() : nullptr);
      break;
    }
  }

  return output_grid;
}

static void node_geo_exec(GeoNodeExecParams params)
{
#ifdef WITH_OPENVDB
  bke::GVolumeGridPtr output_grid = filter_grid(params);
  grids::set_output_grid(params, "Grid", CD_PROP_FLOAT, std::move(output_grid));
#else
  params.set_default_remaining_outputs();
  params.error_message_add(NodeWarningType::Error,
                           TIP_("Disabled, Blender was compiled without OpenVDB"));
#endif
}

static void node_rna(StructRNA *srna)
{
  RNA_def_node_enum(srna,
                    "operation",
                    "Operation",
                    "Type of filtering operation",
                    rna_enum_grid_filter_operation_items,
                    NOD_inline_enum_accessors(custom1),
                    GEO_NODE_GRID_FILTER_MEAN);
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(
      &ntype, GEO_NODE_GRID_LEVEL_SET_FILTER, "Level Set Filter", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.draw_buttons = node_layout;
  nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_grid_level_set_filter_cc
