/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "DEG_depsgraph_query.h"

#include "BKE_type_conversions.hh"
#include "BKE_volume.h"

#include "BLI_virtual_array.hh"

#include "NOD_add_node_search.hh"
#include "NOD_socket_search_link.hh"

#include "node_geometry_util.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#ifdef WITH_OPENVDB
#  include <openvdb/openvdb.h>
#  include <openvdb/tools/Interpolation.h>
#endif

namespace blender::nodes::node_geo_input_grid_coordinate_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>(CTX_N_(BLT_I18NCONTEXT_ID_ID, "Volume"))
      .translation_context(BLT_I18NCONTEXT_ID_ID)
      .supported_type(GeometryComponent::Type::Volume);

  b.add_output<decl::Vector>(N_("Coordinate"), "Coordinate").field_source();
}

static void search_node_add_ops(GatherAddNodeSearchParams &params)
{
  if (!U.experimental.use_new_volume_nodes) {
    return;
  }
  blender::nodes::search_node_add_ops_for_basic_node(params);
}

static void search_link_ops(GatherLinkSearchOpParams &params)
{
  if (!U.experimental.use_new_volume_nodes) {
    return;
  }
  blender::nodes::search_link_ops_for_basic_node(params);
}

static void node_geo_exec(GeoNodeExecParams /*params*/)
{
  // #ifdef WITH_OPENVDB
  //   GeometrySet geometry_set = params.extract_input<GeometrySet>("Volume");
  //   if (!geometry_set.has_volume()) {
  //     params.set_default_remaining_outputs();
  //     return;
  //   }
  //   const NodeGeometrySampleVolume &storage = node_storage(params.node());
  //   const eCustomDataType output_field_type = eCustomDataType(storage.grid_type);
  //   auto interpolation_mode =
  //   GeometryNodeSampleVolumeInterpolationMode(storage.interpolation_mode);
  //
  //   GField grid_field = get_input_attribute_field(params, output_field_type);
  //   const StringRefNull grid_name = get_grid_name(grid_field);
  //   if (grid_name == "") {
  //     params.error_message_add(NodeWarningType::Error, TIP_("Grid name needs to be specified"));
  //     params.set_default_remaining_outputs();
  //     return;
  //   }
  //
  //   const VolumeComponent *component = geometry_set.get_component_for_read<VolumeComponent>();
  //   const Volume *volume = component->get_for_read();
  //   BKE_volume_load(volume, DEG_get_bmain(params.depsgraph()));
  //   const VolumeGrid *volume_grid = BKE_volume_grid_find_for_read(volume, grid_name.c_str());
  //   if (volume_grid == nullptr) {
  //     params.set_default_remaining_outputs();
  //     return;
  //   }
  //   openvdb::GridBase::ConstPtr base_grid = BKE_volume_grid_openvdb_for_read(volume,
  //   volume_grid); const VolumeGridType grid_type = BKE_volume_grid_type_openvdb(*base_grid);
  //
  //   /* Check that the grid type is supported. */
  //   const CPPType *grid_cpp_type = vdb_grid_type_to_cpp_type(grid_type);
  //   if (grid_cpp_type == nullptr) {
  //     params.set_default_remaining_outputs();
  //     params.error_message_add(NodeWarningType::Error, TIP_("The grid type is unsupported"));
  //     return;
  //   }
  //
  //   /* Use to the Nearest Neighbor sampler for Bool grids (no interpolation). */
  //   if (grid_type == VOLUME_GRID_BOOLEAN &&
  //       interpolation_mode != GEO_NODE_SAMPLE_VOLUME_INTERPOLATION_MODE_NEAREST)
  //   {
  //     interpolation_mode = GEO_NODE_SAMPLE_VOLUME_INTERPOLATION_MODE_NEAREST;
  //   }
  //
  //   Field<float3> position_field = params.extract_input<Field<float3>>("Position");
  //   auto fn = std::make_shared<SampleVolumeFunction>(std::move(base_grid), interpolation_mode);
  //   auto op = FieldOperation::Create(std::move(fn), {position_field});
  //   GField output_field = GField(std::move(op));
  //
  //   output_field = bke::get_implicit_type_conversions().try_convert(
  //       output_field, *bke::custom_data_type_to_cpp_type(output_field_type));
  //
  //   output_attribute_field(params, std::move(output_field));
  // #else
  //   params.set_default_remaining_outputs();
  //   params.error_message_add(NodeWarningType::Error,
  //                            TIP_("Disabled, Blender was compiled without OpenVDB"));
  // #endif
}

}  // namespace blender::nodes::node_geo_input_grid_coordinate_cc

void register_node_type_geo_input_grid_coordinate()
{
  namespace file_ns = blender::nodes::node_geo_input_grid_coordinate_cc;

  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_INPUT_GRID_COORDINATE, "Grid Coordinate", NODE_CLASS_INPUT);
  ntype.declare = file_ns::node_declare;
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  ntype.gather_add_node_search_ops = file_ns::search_node_add_ops;
  ntype.gather_link_search_ops = file_ns::search_link_ops;
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  nodeRegisterType(&ntype);
}
