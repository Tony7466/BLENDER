/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "BKE_lib_id.h"
#include "BKE_volume.hh"
#include "BKE_volume_openvdb.hh"

#include "RNA_enum_types.hh"

#include "NOD_rna_define.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

namespace blender::nodes::node_geo_store_named_grid_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  const bNode *node = b.node_or_null();
  if (!node) {
    return;
  }

  b.add_input<decl::Geometry>("Volume");
  b.add_input<decl::String>("Name");
  grids::declare_grid_type_input(b, eCustomDataType(node->custom1), "Grid");

  b.add_output<decl::Geometry>("Volume");
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "data_type", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = CD_PROP_FLOAT;
}

template<typename T>
static void try_store_grid(GeoNodeExecParams params, Volume *volume, StringRef name)
{
  const bke::ValueOrField<T> value = params.extract_input<bke::ValueOrField<T>>("Grid");
  if (!value.is_grid()) {
    return;
  }

  if (VolumeGrid *existing_grid = BKE_volume_grid_find_for_write(volume, name.data())) {
    BKE_volume_grid_remove(volume, existing_grid);
  }

  BKE_volume_grid_add_vdb(*volume, name, value.grid->grid);
}

static void node_geo_exec(GeoNodeExecParams params)
{
#ifdef WITH_OPENVDB
  const eCustomDataType data_type = eCustomDataType(params.node().custom1);
  BLI_assert(grids::grid_type_supported(data_type));
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Volume");
  const std::string grid_name = params.extract_input<std::string>("Name");

  Volume *volume = geometry_set.get_volume_for_write();
  if (!volume){
    volume = static_cast<Volume *>(BKE_id_new_nomain(ID_VO, "Store Named Grid Output"));
    geometry_set.replace_volume(volume);
  }

  if (volume) {
    switch (data_type) {
      case CD_PROP_FLOAT:
        try_store_grid<float>(params, volume, grid_name);
        break;
      case CD_PROP_FLOAT3:
        try_store_grid<float3>(params, volume, grid_name);
        break;
      default:
        BLI_assert_unreachable();
        break;
    }

    params.set_output("Volume", geometry_set);
    return;
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
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_STORE_NAMED_GRID, "Store Named Grid", NODE_CLASS_GEOMETRY);

  ntype.declare = node_declare;
  ntype.draw_buttons = node_layout;
  ntype.initfunc = node_init;
  ntype.geometry_node_execute = node_geo_exec;
  nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_store_named_grid_cc
