/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "RNA_enum_types.hh"

#include "NOD_socket_search_link.hh"

#ifdef WITH_OPENVDB
#  include <openvdb/tools/Morphology.h>
#endif

namespace blender::nodes::node_geo_dilate_grid_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  const bNode *node = b.node_or_null();
  if (!node) {
    return;
  }

  grids::declare_grid_type_input(b, CD_PROP_FLOAT, "Grid");
  b.add_input<decl::Int>("Iterations").default_value(1).min(0);

  grids::declare_grid_type_output(b, CD_PROP_FLOAT, "Grid");
}

static void search_link_ops(GatherLinkSearchOpParams &params)
{
  if (U.experimental.use_new_volume_nodes) {
    nodes::search_link_ops_for_basic_node(params);
  }
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiLayoutSetPropSep(layout, true);
  uiLayoutSetPropDecorate(layout, false);
  uiItemR(layout, ptr, "data_type", UI_ITEM_NONE, "", ICON_NONE);
  uiItemR(layout, ptr, "neighbors_mode", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = CD_PROP_FLOAT;
  node->custom2 = GEO_NODE_GRID_NEIGHBOR_FACE;
}

template<typename T>
static void try_dilate_grid(GeoNodeExecParams params,
                            const int iterations,
                            const GeometryNodeGridNeighborTopology neighbors_mode)
{
  const bke::SocketValueVariant<T> value = params.extract_input<bke::SocketValueVariant<T>>(
      "Grid");
  if (!value.is_grid()) {
    return;
  }

  using GridType = typename bke::VolumeGridPtr<T>::GridType;
  typename GridType::Ptr grid = value.grid.grid_for_write();
  BLI_assert(grid);

  openvdb::tools::dilateActiveValues(
      grid->tree(), iterations, grids::get_vdb_neighbors_mode(neighbors_mode));

  params.set_output(
      "Grid",
      bke::SocketValueVariant<T>(
          bke::make_volume_grid_ptr(grid, VOLUME_TREE_SOURCE_GENERATED).template typed<T>()));
}

static void node_geo_exec(GeoNodeExecParams params)
{
#ifdef WITH_OPENVDB
  const eCustomDataType data_type = eCustomDataType(params.node().custom1);
  const GeometryNodeGridNeighborTopology neighbors_mode = GeometryNodeGridNeighborTopology(
      params.node().custom2);
  BLI_assert(grids::grid_type_supported(data_type));
  const int iterations = params.extract_input<int>("Iterations");

  switch (data_type) {
    case CD_PROP_FLOAT:
      try_dilate_grid<float>(params, iterations, neighbors_mode);
      break;
    case CD_PROP_FLOAT3:
      try_dilate_grid<float3>(params, iterations, neighbors_mode);
      break;
    default:
      BLI_assert_unreachable();
      break;
  }

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
                    NOD_inline_enum_accessors(custom1),
                    CD_PROP_FLOAT,
                    grids::grid_type_items_fn);

  RNA_def_node_enum(srna,
                    "neighbors_mode",
                    "Neighbors Mode",
                    "Which voxel neighbors to use",
                    rna_enum_grid_neighbors_topology_items,
                    NOD_inline_enum_accessors(custom2),
                    GEO_NODE_GRID_NEIGHBOR_FACE);
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_DILATE_GRID, "Dilate Grid", NODE_CLASS_CONVERTER);

  ntype.declare = node_declare;
  ntype.gather_link_search_ops = search_link_ops;
  ntype.draw_buttons = node_layout;
  ntype.initfunc = node_init;
  ntype.geometry_node_execute = node_geo_exec;
  nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_dilate_grid_cc
