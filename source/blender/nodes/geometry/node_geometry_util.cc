/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"
#include "node_util.hh"

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_space_types.h"

#include "BKE_context.hh"
#include "BKE_mesh.hh"
#include "BKE_mesh_runtime.hh"
#include "BKE_node.hh"
#include "BKE_pointcloud.h"
#include "BKE_volume.hh"
#include "BKE_volume_openvdb.hh"

#include "NOD_rna_define.hh"
#include "NOD_socket_search_link.hh"

#include "RNA_enum_types.hh"

namespace blender::nodes {

bool check_tool_context_and_error(GeoNodeExecParams &params)
{
  if (!params.user_data()->call_data->operator_data) {
    params.error_message_add(NodeWarningType::Error, TIP_("Node must be run as tool"));
    params.set_default_remaining_outputs();
    return false;
  }
  return true;
}

void search_link_ops_for_tool_node(GatherLinkSearchOpParams &params)
{
  if (params.space_node().geometry_nodes_type == SNODE_GEOMETRY_TOOL) {
    search_link_ops_for_basic_node(params);
  }
}

namespace enums {

const EnumPropertyItem *attribute_type_type_with_socket_fn(bContext * /*C*/,
                                                           PointerRNA * /*ptr*/,
                                                           PropertyRNA * /*prop*/,
                                                           bool *r_free)
{
  *r_free = true;
  return enum_items_filter(rna_enum_attribute_type_items,
                           [](const EnumPropertyItem &item) -> bool {
                             return generic_attribute_type_supported(item) &&
                                    !ELEM(item.value, CD_PROP_BYTE_COLOR, CD_PROP_FLOAT2);
                           });
}

bool generic_attribute_type_supported(const EnumPropertyItem &item)
{
  return ELEM(item.value,
              CD_PROP_FLOAT,
              CD_PROP_FLOAT2,
              CD_PROP_FLOAT3,
              CD_PROP_COLOR,
              CD_PROP_BOOL,
              CD_PROP_INT32,
              CD_PROP_BYTE_COLOR,
              CD_PROP_QUATERNION);
}

const EnumPropertyItem *domain_experimental_grease_pencil_version3_fn(bContext * /*C*/,
                                                                      PointerRNA * /*ptr*/,
                                                                      PropertyRNA * /*prop*/,
                                                                      bool *r_free)
{
  *r_free = true;
  return enum_items_filter(
      rna_enum_attribute_domain_items, [](const EnumPropertyItem &item) -> bool {
        return (item.value == ATTR_DOMAIN_LAYER) ? U.experimental.use_grease_pencil_version3 :
                                                   true;
      });
}

const EnumPropertyItem *domain_without_corner_experimental_grease_pencil_version3_fn(
    bContext * /*C*/, PointerRNA * /*ptr*/, PropertyRNA * /*prop*/, bool *r_free)
{
  *r_free = true;
  return enum_items_filter(
      rna_enum_attribute_domain_without_corner_items, [](const EnumPropertyItem &item) -> bool {
        return (item.value == ATTR_DOMAIN_LAYER) ? U.experimental.use_grease_pencil_version3 :
                                                   true;
      });
}

}  // namespace enums

namespace grids {

bool grid_type_supported(const eCustomDataType data_type)
{
  return ELEM(data_type, CD_PROP_FLOAT, CD_PROP_FLOAT3);
}

const EnumPropertyItem *grid_type_items_fn(bContext * /*C*/,
                                           PointerRNA * /*ptr*/,
                                           PropertyRNA * /*prop*/,
                                           bool *r_free)
{
  *r_free = true;
  return enum_items_filter(rna_enum_attribute_type_items,
                           [](const EnumPropertyItem &item) -> bool {
                             return enums::generic_attribute_type_supported(item) &&
                                    grid_type_supported(eCustomDataType(item.value));
                           });
}

BaseSocketDeclarationBuilder &declare_grid_type_input(NodeDeclarationBuilder &b,
                                                      const eCustomDataType type,
                                                      const StringRef name,
                                                      const StringRef identifier)
{
  switch (type) {
    case CD_PROP_FLOAT:
      return b.add_input<decl::Float>(name, identifier).hide_value();
    case CD_PROP_FLOAT3:
      return b.add_input<decl::Vector>(name, identifier).hide_value();
    default:
      break;
  }
  BLI_assert_unreachable();
  return b.add_input<decl::Float>(name, identifier);
}

BaseSocketDeclarationBuilder &declare_grid_type_output(NodeDeclarationBuilder &b,
                                                       const eCustomDataType type,
                                                       const StringRef name,
                                                       const StringRef identifier)
{
  switch (type) {
    case CD_PROP_FLOAT:
      return b.add_output<decl::Float>(name, identifier);
    case CD_PROP_FLOAT3:
      return b.add_output<decl::Vector>(name, identifier);
    default:
      break;
  }
  BLI_assert_unreachable();
  return b.add_output<decl::Float>(name, identifier);
}

openvdb::tools::NearestNeighbors get_vdb_neighbors_mode(
    GeometryNodeGridNeighborTopology neighbors_mode)
{
  switch (neighbors_mode) {
    case GEO_NODE_GRID_NEIGHBOR_FACE:
      return openvdb::tools::NearestNeighbors::NN_FACE;
    case GEO_NODE_GRID_NEIGHBOR_FACE_EDGE:
      return openvdb::tools::NearestNeighbors::NN_FACE_EDGE;
    case GEO_NODE_GRID_NEIGHBOR_FACE_EDGE_VERTEX:
      return openvdb::tools::NearestNeighbors::NN_FACE_EDGE_VERTEX;
  }
  BLI_assert_unreachable();
  return openvdb::tools::NearestNeighbors::NN_FACE;
}

}  // namespace grids

}  // namespace blender::nodes

bool geo_node_poll_default(const bNodeType * /*ntype*/,
                           const bNodeTree *ntree,
                           const char **r_disabled_hint)
{
  if (!STREQ(ntree->idname, "GeometryNodeTree")) {
    *r_disabled_hint = TIP_("Not a geometry node tree");
    return false;
  }
  return true;
}

void geo_node_type_base(bNodeType *ntype, int type, const char *name, short nclass)
{
  blender::bke::node_type_base(ntype, type, name, nclass);
  ntype->poll = geo_node_poll_default;
  ntype->insert_link = node_insert_link_default;
  ntype->gather_link_search_ops = blender::nodes::search_link_ops_for_basic_node;
}
