/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "NOD_rna_define.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "RNA_enum_types.hh"

#ifdef WITH_OPENVDB
#  include <openvdb/tools/Filter.h>
#endif

namespace blender::nodes::node_geo_grid_filter_cc {

static const eCustomDataType DummyMaskType = CD_PROP_FLOAT;

static void node_declare(NodeDeclarationBuilder &b)
{
  const bNode *node = b.node_or_null();
  if (!node) {
    return;
  }

  const eCustomDataType data_type = eCustomDataType(node->custom1);

  grids::declare_grid_type_input(b, data_type, "Grid");
  grids::declare_grid_type_input(b, DummyMaskType, "Mask");

  grids::declare_grid_type_output(b, data_type, "Grid");
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiLayoutSetPropSep(layout, true);
  uiLayoutSetPropDecorate(layout, false);
  uiItemR(layout, ptr, "data_type", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = CD_PROP_FLOAT;
}

template<typename T> struct GridMaskedFilterOp {
  GeoNodeExecParams params;

  template<typename U> bke::GVolumeGridPtr operator()()
  {
    using GridType = typename bke::VolumeGridPtr<T>::GridType;
    using MaskType = typename bke::VolumeGridPtr<U>::GridType;

    const GeometryNodeGridFilterOperation operation = GEO_NODE_GRID_FILTER_MEAN;
    const int width = 1;
    const int iterations = 1;

    const bke::VolumeGridPtr<T> grid = grids::extract_grid_input<T>(this->params, "Grid");
    const bke::VolumeGridPtr<U> mask = grids::extract_grid_input<U>(this->params, "Mask");

    const bke::VolumeGridPtr<T> output_grid = grid->is_mutable() ?
                                                  grid :
                                                  bke::VolumeGridPtr<T>{grid->copy()};

    openvdb::tools::Filter<GridType, MaskType> filter(
        *const_cast<VolumeGrid *>(output_grid.get())->grid_for_write());

    switch (operation) {
      case GEO_NODE_GRID_FILTER_MEAN:
        filter.mean(width, iterations, mask ? &*mask->grid() : nullptr);
        break;
      case GEO_NODE_GRID_FILTER_MEDIAN:
        break;
      case GEO_NODE_GRID_FILTER_GAUSSIAN:
        break;
      case GEO_NODE_GRID_FILTER_OFFSET:
        break;
      case GEO_NODE_GRID_FILTER_MEAN_CURVATURE:
        break;
      case GEO_NODE_GRID_FILTER_LAPLACIAN:
        break;
    }

    return output_grid;
  }
};

struct GridFilterOp {
  GeoNodeExecParams params;

  template<typename T> bke::GVolumeGridPtr operator()()
  {
    GridMaskedFilterOp<T> masked_filter_op = {params};
    return grids::apply(DummyMaskType, masked_filter_op);
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
#ifdef WITH_OPENVDB
  const eCustomDataType data_type = eCustomDataType(params.node().custom1);
  BLI_assert(grids::grid_type_supported(data_type));

  GridFilterOp filter_op = {params};
  bke::GVolumeGridPtr grid = grids::apply(data_type, filter_op);

  grids::set_output_grid(params, "Grid", data_type, grid);
#else
  params.set_default_remaining_outputs();
  params.error_message_add(NodeWarningType::Error,
                           TIP_("Disabled, Blender was compiled without OpenVDB"));
#endif
}

static void node_rna(StructRNA *srna)
{
  RNA_def_node_enum(srna,
                    "data_type",
                    "Data Type",
                    "Type of grid data",
                    rna_enum_attribute_type_items,
                    NOD_inline_enum_accessors(custom1),
                    CD_PROP_FLOAT,
                    grids::grid_type_items_fn);
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_GRID_FILTER, "Filter", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.draw_buttons = node_layout;
  nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_grid_filter_cc
