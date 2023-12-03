/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "BKE_volume.hh"
#include "BKE_volume_openvdb.hh"

#include "RNA_enum_types.hh"

#include "NOD_rna_define.hh"
#include "NOD_socket_search_link.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

namespace blender::nodes::node_geo_get_named_grid_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  const bNode *node = b.node_or_null();
  if (!node) {
    return;
  }

  b.add_input<decl::Geometry>("Volume");
  b.add_input<decl::String>("Name");
  b.add_input<decl::Bool>("Remove").default_value(true);

  b.add_output<decl::Geometry>("Volume");
  grids::declare_grid_type_output(b, eCustomDataType(node->custom1), "Grid");
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
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = CD_PROP_FLOAT;
}

static void node_geo_exec(GeoNodeExecParams params)
{
#ifdef WITH_OPENVDB
  const eCustomDataType data_type = eCustomDataType(params.node().custom1);
  BLI_assert(grids::grid_type_supported(data_type));
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Volume");
  const std::string grid_name = params.extract_input<std::string>("Name");
  const bool remove_grid = params.extract_input<bool>("Remove");

  if (Volume *volume = geometry_set.get_volume_for_write()) {
    if (GVolumeGridPtr grid = BKE_volume_grid_find_for_write(volume, grid_name.c_str())) {
      grids::set_output_grid(params, "Grid", data_type, grid);
      params.set_output("Volume", geometry_set);

      if (remove_grid) {
        BKE_volume_grid_remove(volume, grid.get());
      }
      return;
    }
  }

  grids::set_output_grid(params, "Grid", data_type, nullptr);
  params.set_output("Volume", geometry_set);
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

  geo_node_type_base(&ntype, GEO_NODE_GET_NAMED_GRID, "Get Named Grid", NODE_CLASS_GEOMETRY);

  ntype.declare = node_declare;
  ntype.gather_link_search_ops = search_link_ops;
  ntype.draw_buttons = node_layout;
  ntype.initfunc = node_init;
  ntype.geometry_node_execute = node_geo_exec;
  nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_get_named_grid_cc
