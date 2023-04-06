/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "DEG_depsgraph_query.h"
#include "node_geometry_util.hh"

#include "BKE_lib_id.h"
#include "BKE_mesh.hh"
#include "BKE_mesh_runtime.h"
#include "BKE_mesh_wrapper.h"
#include "BKE_object.h"
#include "BKE_volume.h"

#include "GEO_mesh_to_volume.hh"

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"

#include "UI_interface.h"
#include "UI_resources.h"

namespace blender::nodes::node_geo_mesh_to_volume_cc {

NODE_STORAGE_FUNCS(NodeGeometryMeshToVolume)

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>(N_("Mesh")).supported_type(GEO_COMPONENT_TYPE_MESH);
  b.add_input<decl::Float>(N_("Density")).default_value(1.0f).min(0.01f).max(FLT_MAX);
  b.add_input<decl::Float>(N_("Voxel Size"))
      .default_value(0.3f)
      .min(0.01f)
      .max(FLT_MAX)
      .subtype(PROP_DISTANCE);
  b.add_input<decl::Int>(N_("Voxel Amount")).default_value(64).min(1);
  b.add_input<decl::Float>(N_("Interior Band Width"))
      .default_value(0.2f)
      .min(0.0001f)
      .max(FLT_MAX)
      .subtype(PROP_DISTANCE)
      .description(N_("Width of the gradient inside of the mesh"));
  b.add_input<decl::Int>(N_("Interior Band Voxels"))
      .default_value(3)
      .min(1)
      .max(INT_MAX)
      .description(N_("Width of the gradient inside of the mesh in voxels"));
  b.add_output<decl::Geometry>(N_("Volume"));
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiLayoutSetPropSep(layout, true);
  uiLayoutSetPropDecorate(layout, false);
  uiItemR(layout, ptr, "resolution_mode", 0, IFACE_("Resolution"), ICON_NONE);
  uiItemR(layout, ptr, "units", 0, IFACE_("Units"), ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometryMeshToVolume *data = MEM_cnew<NodeGeometryMeshToVolume>(__func__);
  data->resolution_mode = MESH_TO_VOLUME_RESOLUTION_MODE_VOXEL_AMOUNT;
  data->units = MESH_TO_VOLUME_UNIT_LOCAL;
  node->storage = data;
}

static void node_update(bNodeTree *ntree, bNode *node)
{
  NodeGeometryMeshToVolume &data = node_storage(*node);

  bNodeSocket *voxel_size_socket = nodeFindSocket(node, SOCK_IN, "Voxel Size");
  bNodeSocket *voxel_amount_socket = nodeFindSocket(node, SOCK_IN, "Voxel Amount");
  nodeSetSocketAvailability(ntree,
                            voxel_amount_socket,
                            data.resolution_mode == MESH_TO_VOLUME_RESOLUTION_MODE_VOXEL_AMOUNT);
  nodeSetSocketAvailability(
      ntree, voxel_size_socket, data.resolution_mode == MESH_TO_VOLUME_RESOLUTION_MODE_VOXEL_SIZE);

  bNodeSocket *unit_local_socket = nodeFindSocket(node, SOCK_IN, "Interior Band Width");
  bNodeSocket *unit_voxels_socket = nodeFindSocket(node, SOCK_IN, "Interior Band Voxels");
  nodeSetSocketAvailability(ntree, unit_local_socket, data.units == MESH_TO_VOLUME_UNIT_LOCAL);
  nodeSetSocketAvailability(ntree, unit_voxels_socket, data.units == MESH_TO_VOLUME_UNIT_VOXELS);
}

#ifdef WITH_OPENVDB

static Volume *create_volume_from_mesh(const Mesh &mesh, GeoNodeExecParams &params)
{
  const NodeGeometryMeshToVolume &storage =
      *(const NodeGeometryMeshToVolume *)params.node().storage;

  const float density = params.get_input<float>("Density");

  auto unit_mode = (MeshToVolumeModifierUnits)storage.units;
  float interior_band_width;

  if (unit_mode == MESH_TO_VOLUME_UNIT_LOCAL) {
    interior_band_width = params.get_input<float>("Interior Band Width");
    if (interior_band_width <= 1e-5f) {
      return nullptr;
    }
  }
  else {
    int voxels = params.get_input<int>("Interior Band Voxels");
    if (voxels < 1) {
      return nullptr;
    }
    interior_band_width = (float)voxels;
  }

  const float4x4 mesh_to_volume_space_transform = float4x4::identity();
  const float volume_simplify = BKE_volume_simplify_factor(params.depsgraph());
  if (volume_simplify == 0.0f) {
    return nullptr;
  }

  geometry::MeshToVolumeSettings settings{/* voxels */ 0,
                                          /* voxel_size */ 0.0f,
                                          /* interior_band_width */ interior_band_width,
                                          /* exterior_band_width */ 0.0f,
                                          /* density */ density,
                                          /* simplify */ volume_simplify,
                                          /* fill_volume */ false,
                                          /* use_world_space_units */ unit_mode ==
                                              MESH_TO_VOLUME_UNIT_LOCAL,
                                          /* convert_to_fog */ true,
                                          /* unsigned_distance */ false};

  auto bounds_fn = [&](float3 &r_min, float3 &r_max) {
    float3 min{std::numeric_limits<float>::max()};
    float3 max{-std::numeric_limits<float>::max()};
    BKE_mesh_wrapper_minmax(&mesh, min, max);
    r_min = min;
    r_max = max;
  };

  auto mode = (MeshToVolumeModifierResolutionMode)storage.resolution_mode;
  if (mode == MESH_TO_VOLUME_RESOLUTION_MODE_VOXEL_AMOUNT) {
    settings.voxels = params.get_input<int>("Voxel Amount");
    if (settings.voxels <= 0) {
      return nullptr;
    }
    settings.voxel_size = geometry::volume_compute_voxel_size(
        settings, bounds_fn, mesh_to_volume_space_transform);
  }
  else if (mode == MESH_TO_VOLUME_RESOLUTION_MODE_VOXEL_SIZE) {
    settings.voxel_size = params.get_input<float>("Voxel Size");
  }

  if (settings.voxel_size < 1e-5f) {
    /* The voxel size is too small. */
    return nullptr;
  }

  if (mesh.totpoly == 0) {
    return nullptr;
  }

  Volume *volume = reinterpret_cast<Volume *>(BKE_id_new_nomain(ID_VO, nullptr));

  /* Convert mesh to grid and add to volume. */
  VolumeGrid *volumegrid = geometry::volume_grid_add_from_mesh(
      volume, "density", &mesh, mesh_to_volume_space_transform, settings);

  return volume;
}

#endif /* WITH_OPENVDB */

static void node_geo_exec(GeoNodeExecParams params)
{
#ifdef WITH_OPENVDB
  GeometrySet geometry_set(params.extract_input<GeometrySet>("Mesh"));
  geometry_set.modify_geometry_sets([&](GeometrySet &geometry_set) {
    if (geometry_set.has_mesh()) {
      Volume *volume = create_volume_from_mesh(*geometry_set.get_mesh_for_read(), params);
      geometry_set.replace_volume(volume);
      geometry_set.keep_only_during_modify({GEO_COMPONENT_TYPE_VOLUME});
    }
  });
  params.set_output("Volume", std::move(geometry_set));
#else
  params.set_default_remaining_outputs();
  params.error_message_add(NodeWarningType::Error,
                           TIP_("Disabled, Blender was compiled without OpenVDB"));
  return;
#endif
}

}  // namespace blender::nodes::node_geo_mesh_to_volume_cc

void register_node_type_geo_mesh_to_volume()
{
  namespace file_ns = blender::nodes::node_geo_mesh_to_volume_cc;

  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_MESH_TO_VOLUME, "Mesh to Volume", NODE_CLASS_GEOMETRY);
  ntype.declare = file_ns::node_declare;
  node_type_size(&ntype, 200, 120, 700);
  ntype.initfunc = file_ns::node_init;
  ntype.updatefunc = file_ns::node_update;
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  ntype.draw_buttons = file_ns::node_layout;
  node_type_storage(
      &ntype, "NodeGeometryMeshToVolume", node_free_standard_storage, node_copy_standard_storage);
  nodeRegisterType(&ntype);
}
