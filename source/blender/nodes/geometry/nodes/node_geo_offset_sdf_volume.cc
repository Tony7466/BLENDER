/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "DEG_depsgraph_query.h"
#ifdef WITH_OPENVDB
#  include <openvdb/tools/LevelSetFilter.h>
#endif

#include "node_geometry_util.hh"

#include "BKE_geometry_set.h"
#include "BKE_volume.h"

#include "DNA_node_types.h"

#include "UI_interface.h"
#include "UI_resources.h"

namespace blender::nodes::node_geo_offset_sdf_volume_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>(N_("Volume")).supported_type(GEO_COMPONENT_TYPE_VOLUME);
  b.add_input<decl::Float>(N_("Distance")).default_value(0.1f).subtype(PROP_DISTANCE);
  b.add_output<decl::Geometry>(N_("Volume"));
}

static void sdf_volume_offset(Volume &volume, const GeoNodeExecParams &params)
{
  VolumeGrid *volume_grid = BKE_volume_grid_find_for_write(&volume, "distance");
  if (volume_grid == nullptr) {
    return;
  }
  openvdb::GridBase::Ptr base_grid = BKE_volume_grid_openvdb_for_write(
      &volume, volume_grid, false);

  if (!base_grid->isType<openvdb::FloatGrid>()) {
    return;
  }
  openvdb::FloatGrid::Ptr levelset_grid = openvdb::gridPtrCast<openvdb::FloatGrid>(base_grid);
  openvdb::tools::LevelSetFilter<openvdb::FloatGrid> filter(*levelset_grid);

  filter.offset(-params.get_input<float>("Distance"));
}

static void node_geo_exec(GeoNodeExecParams params)
{
#ifdef WITH_OPENVDB
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Volume");

  geometry_set.modify_geometry_sets([&](GeometrySet &geometry_set) {
    if (!geometry_set.has_volume()) {
      return;
    }
    VolumeComponent &component = geometry_set.get_component_for_write<VolumeComponent>();
    Volume *volume = component.get_for_write();
    BKE_volume_load(volume, DEG_get_bmain(params.depsgraph()));
    sdf_volume_offset(*volume, params);
  });
  params.set_output("Volume", std::move(geometry_set));
#else
  params.set_default_remaining_outputs();
  params.error_message_add(NodeWarningType::Error,
                           TIP_("Disabled, Blender was compiled without OpenVDB"));
#endif
}

}  // namespace blender::nodes::node_geo_offset_sdf_volume_cc

void register_node_type_geo_offset_sdf_volume()
{
  namespace file_ns = blender::nodes::node_geo_offset_sdf_volume_cc;

  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_OFFSET_SDF_VOLUME, "Offset SDF Volume", NODE_CLASS_GEOMETRY);
  ntype.declare = file_ns::node_declare;
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  nodeRegisterType(&ntype);
}
