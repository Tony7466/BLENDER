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

namespace blender::nodes::node_geo_sdf_volume_filter_cc {

NODE_STORAGE_FUNCS(NodeGeometrySdfVolumeFilter)

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>(N_("Volume")).supported_type(GEO_COMPONENT_TYPE_VOLUME);
  b.add_input<decl::Float>(N_("Distance")).default_value(0.1f).subtype(PROP_DISTANCE);
  b.add_input<decl::Int>(N_("Iterations")).min(1).max(256).default_value(1);
  b.add_input<decl::Int>(N_("Width")).default_value(1);
  b.add_output<decl::Geometry>(N_("Volume"));
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiLayoutSetPropSep(layout, true);
  uiLayoutSetPropDecorate(layout, false);
  uiItemR(layout, ptr, "operation", 0, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometrySdfVolumeFilter *data = MEM_cnew<NodeGeometrySdfVolumeFilter>(__func__);
  data->operation = GEO_NODE_SDF_VOLUME_FILTER_GAUSSIAN;
  node->storage = data;
}

static void node_update(bNodeTree *ntree, bNode *node)
{
  const NodeGeometrySdfVolumeFilter &storage = node_storage(*node);
  const GeometryNodeSdfVolumeFilterOperation operation = (GeometryNodeSdfVolumeFilterOperation)
                                                             storage.operation;

  bNodeSocket *distance_socket = nodeFindSocket(node, SOCK_IN, "Distance");
  bNodeSocket *iterations_socket = nodeFindSocket(node, SOCK_IN, "Iterations");
  bNodeSocket *width_socket = nodeFindSocket(node, SOCK_IN, "Width");

  nodeSetSocketAvailability(
      ntree, distance_socket, operation == GEO_NODE_SDF_VOLUME_FILTER_OFFSET);
  nodeSetSocketAvailability(ntree,
                            width_socket,
                            ELEM(operation,
                                 GEO_NODE_SDF_VOLUME_FILTER_GAUSSIAN,
                                 GEO_NODE_SDF_VOLUME_FILTER_MEDIAN,
                                 GEO_NODE_SDF_VOLUME_FILTER_MEAN));
  nodeSetSocketAvailability(ntree,
                            iterations_socket,
                            ELEM(operation,
                                 GEO_NODE_SDF_VOLUME_FILTER_GAUSSIAN,
                                 GEO_NODE_SDF_VOLUME_FILTER_MEDIAN,
                                 GEO_NODE_SDF_VOLUME_FILTER_MEAN,
                                 GEO_NODE_SDF_VOLUME_FILTER_MEAN_CURVATURE,
                                 GEO_NODE_SDF_VOLUME_FILTER_LAPLACIAN));
}

static void sdf_volume_filter(Volume &volume,
                              const GeometryNodeSdfVolumeFilterOperation operation,
                              const GeoNodeExecParams &params)
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
  int iterations;

  switch (operation) {
    case GEO_NODE_SDF_VOLUME_FILTER_GAUSSIAN:
      iterations = params.get_input<int>("Iterations");
      for (int i = 0; i < iterations; i++) {
        filter.gaussian(params.get_input<int>("Width"));
      }
      break;
    case GEO_NODE_SDF_VOLUME_FILTER_LAPLACIAN:
      iterations = params.get_input<int>("Iterations");
      for (int i = 0; i < iterations; i++) {
        filter.laplacian();
      }
      break;
    case GEO_NODE_SDF_VOLUME_FILTER_MEAN:
      iterations = params.get_input<int>("Iterations");
      for (int i = 0; i < iterations; i++) {
        filter.mean(params.get_input<int>("Width"));
      }
      break;
    case GEO_NODE_SDF_VOLUME_FILTER_MEAN_CURVATURE:
      iterations = params.get_input<int>("Iterations");
      for (int i = 0; i < iterations; i++) {
        filter.meanCurvature();
      }
      break;
    case GEO_NODE_SDF_VOLUME_FILTER_MEDIAN:
      iterations = params.get_input<int>("Iterations");
      for (int i = 0; i < iterations; i++) {
        filter.median(params.get_input<int>("Width"));
      }
      break;
    case GEO_NODE_SDF_VOLUME_FILTER_OFFSET:
      filter.offset(-params.get_input<float>("Distance"));
      break;
  }
}

static void node_geo_exec(GeoNodeExecParams params)
{
#ifdef WITH_OPENVDB
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Volume");

  const NodeGeometrySdfVolumeFilter &storage = node_storage(params.node());
  const GeometryNodeSdfVolumeFilterOperation operation = (GeometryNodeSdfVolumeFilterOperation)
                                                             storage.operation;

  geometry_set.modify_geometry_sets([&](GeometrySet &geometry_set) {
    if (!geometry_set.has_volume()) {
      return;
    }
    VolumeComponent &component = geometry_set.get_component_for_write<VolumeComponent>();
    Volume *volume = component.get_for_write();
    BKE_volume_load(volume, DEG_get_bmain(params.depsgraph()));
    sdf_volume_filter(*volume, operation, params);
  });
  params.set_output("Volume", std::move(geometry_set));
#else
  params.set_default_remaining_outputs();
  params.error_message_add(NodeWarningType::Error,
                           TIP_("Disabled, Blender was compiled without OpenVDB"));
#endif
}

}  // namespace blender::nodes::node_geo_sdf_volume_filter_cc

void register_node_type_geo_sdf_volume_filter()
{
  namespace file_ns = blender::nodes::node_geo_sdf_volume_filter_cc;

  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_SDF_VOLUME_FILTER, "SDF Volume Filter", NODE_CLASS_GEOMETRY);
  ntype.declare = file_ns::node_declare;
  ntype.initfunc = file_ns::node_init;
  ntype.updatefunc = file_ns::node_update;
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  node_type_storage(&ntype,
                    "NodeGeometrySdfVolumeFilter",
                    node_free_standard_storage,
                    node_copy_standard_storage);
  ntype.draw_buttons = file_ns::node_layout;
  nodeRegisterType(&ntype);
}
