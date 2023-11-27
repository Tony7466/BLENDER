/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "DEG_depsgraph_query.hh"
#ifdef WITH_OPENVDB
#  include <openvdb/tools/GridTransformer.h>
#  include <openvdb/tools/VolumeToMesh.h>
#endif

#include "node_geometry_util.hh"

#include "BKE_lib_id.h"
#include "BKE_material.h"
#include "BKE_mesh.hh"
#include "BKE_mesh_runtime.hh"
#include "BKE_volume.hh"
#include "BKE_volume_openvdb.hh"
#include "BKE_volume_to_mesh.hh"

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"

#include "NOD_rna_define.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "GEO_randomize.hh"

namespace blender::nodes::node_geo_extrapolate_grid_cc {

NODE_STORAGE_FUNCS(NodeGeometryExtrapolateGrid)

static void node_declare(NodeDeclarationBuilder &b) {}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiLayoutSetPropSep(layout, true);
  uiLayoutSetPropDecorate(layout, false);
  //  uiItemR(layout, ptr, "resolution_mode", UI_ITEM_NONE, IFACE_("Resolution"), ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometryExtrapolateGrid *data = MEM_cnew<NodeGeometryExtrapolateGrid>(__func__);
  //  data->resolution_mode = VOLUME_TO_MESH_RESOLUTION_MODE_GRID;
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
  //  static EnumPropertyItem resolution_mode_items[] = {
  //      {VOLUME_TO_MESH_RESOLUTION_MODE_GRID,
  //       "GRID",
  //       0,
  //       "Grid",
  //       "Use resolution of the volume grid"},
  //      {VOLUME_TO_MESH_RESOLUTION_MODE_VOXEL_AMOUNT,
  //       "VOXEL_AMOUNT",
  //       0,
  //       "Amount",
  //       "Desired number of voxels along one axis"},
  //      {VOLUME_TO_MESH_RESOLUTION_MODE_VOXEL_SIZE,
  //       "VOXEL_SIZE",
  //       0,
  //       "Size",
  //       "Desired voxel side length"},
  //      {0, nullptr, 0, nullptr, nullptr},
  //  };

  //  RNA_def_node_enum(srna,
  //                    "resolution_mode",
  //                    "Resolution Mode",
  //                    "How the voxel size is specified",
  //                    resolution_mode_items,
  //                    NOD_storage_enum_accessors(resolution_mode));
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
