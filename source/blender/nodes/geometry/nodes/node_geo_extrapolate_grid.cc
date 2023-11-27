/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "NOD_rna_define.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "RNA_enum_types.hh"

namespace blender::nodes::node_geo_extrapolate_grid_cc {

NODE_STORAGE_FUNCS(NodeGeometryExtrapolateGrid)

static void node_declare(NodeDeclarationBuilder &b) {}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiLayoutSetPropSep(layout, true);
  uiLayoutSetPropDecorate(layout, false);
  uiItemR(layout, ptr, "data_type", UI_ITEM_NONE, "", ICON_NONE);
  uiItemR(layout, ptr, "fast_sweeping_region", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometryExtrapolateGrid *data = MEM_cnew<NodeGeometryExtrapolateGrid>(__func__);
  data->data_type = CD_PROP_FLOAT;
  node->storage = data;
}

#ifdef WITH_OPENVDB

#endif /* WITH_OPENVDB */

static void node_geo_exec(GeoNodeExecParams params)
{
#ifdef WITH_OPENVDB
  params.set_default_remaining_outputs();
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
                    NOD_storage_enum_accessors(data_type),
                    CD_PROP_FLOAT,
                    grids::grid_type_items_fn);

  RNA_def_node_enum(srna,
                    "fast_sweeping_region",
                    "Region",
                    "Region of voxels affected by the node",
                    rna_enum_fast_sweeping_region_items,
                    NOD_storage_enum_accessors(fast_sweeping_region));
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_EXTRAPOLATE_GRID, "Extrapolate Grid", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  node_type_storage(&ntype,
                    "NodeGeometryExtrapolateGrid",
                    node_free_standard_storage,
                    node_copy_standard_storage);
  ntype.initfunc = node_init;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.draw_buttons = node_layout;
  nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_extrapolate_grid_cc
