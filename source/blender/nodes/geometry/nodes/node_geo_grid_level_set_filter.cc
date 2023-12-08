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

static void node_declare(NodeDeclarationBuilder &b)
{
  const bNode *node = b.node_or_null();
  if (!node) {
    return;
  }

  eCustomDataType data_type = eCustomDataType(node->custom1);

  grids::declare_grid_type_input(b, data_type, "Grid");

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

struct GridFilterOp {
  GeoNodeExecParams params;

  template<typename T> bke::GVolumeGridPtr operator()()
  {
    const eCustomDataType data_type = eCustomDataType(params.node().custom1);
    const eCustomDataType topo_data_type = eCustomDataType(params.node().custom2);
    BLI_assert(grids::grid_type_supported(topo_data_type));
    const bke::GVolumeGridPtr topo_grid = grids::extract_grid_input(
        this->params, "Topology Grid", topo_data_type);
    const fn::Field<T> value_field = this->params.extract_input<fn::Field<T>>("Value");
    const T background = this->params.extract_input<T>("Background");

    return grids::try_capture_field_as_grid(
        data_type, topo_data_type, topo_grid, value_field, &background);
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
