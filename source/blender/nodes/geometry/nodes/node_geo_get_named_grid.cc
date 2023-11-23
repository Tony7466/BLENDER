/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "BKE_volume.hh"
#include "BKE_volume_openvdb.hh"

#include "RNA_enum_types.hh"

#include "NOD_rna_define.hh"

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

template<typename T>
static bool try_output_grid_value(GeoNodeExecParams params, const openvdb::GridBase::Ptr &grid)
{
  using FVGrid = bke::FieldValueGrid<T>;
  using FVGridPtr = ImplicitSharingPtr<FVGrid>;
  using GridType = typename FVGrid::GridType;

  std::shared_ptr<GridType> typed_grid = openvdb::GridBase::grid<GridType>(grid);
  if (!typed_grid) {
    params.set_output("Grid", ValueOrField<T>());
    return false;
  }

  FVGridPtr grid_ptr(new FVGrid(typed_grid));
  params.set_output("Grid", ValueOrField<T>(std::move(grid_ptr)));
  return true;
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
    if (VolumeGridPtr grid = BKE_volume_grid_find_for_write(volume, grid_name.c_str())) {
      if (openvdb::GridBase::Ptr grid_vdb = BKE_volume_grid_openvdb_for_write(
              volume, grid.get(), false)) {
        switch (data_type) {
          case CD_PROP_FLOAT:
            try_output_grid_value<float>(params, grid_vdb);
            break;
          case CD_PROP_FLOAT3:
            try_output_grid_value<float3>(params, grid_vdb);
            break;
          default:
            BLI_assert_unreachable();
            break;
        }
      }

      if (remove_grid) {
        BKE_volume_grid_remove(volume, grid.get());
      }

      params.set_output("Volume", geometry_set);
      return;
    }
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

  geo_node_type_base(&ntype, GEO_NODE_GET_NAMED_GRID, "Get Named Grid", NODE_CLASS_GEOMETRY);

  ntype.declare = node_declare;
  ntype.draw_buttons = node_layout;
  ntype.initfunc = node_init;
  ntype.geometry_node_execute = node_geo_exec;
  nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_get_named_grid_cc
