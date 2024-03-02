/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#define WITH_OPENVDB
#ifdef WITH_OPENVDB
#  include <openvdb/openvdb.h>
#  include <openvdb/tools/Composite.h>
#  include <openvdb/tools/GridTransformer.h>
#endif

#include "BKE_volume_grid.hh"

#include "NOD_socket_search_link.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_sdf_grid_boolean_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Float>("SDF Grid 1").hide_value();
  b.add_input<decl::Float>("SDF Grid 2").hide_value();
  b.add_output<decl::Float>("SDF Grid").hide_value();
}

static void node_gather_link_search_ops(GatherLinkSearchOpParams &params)
{
  if (U.experimental.use_new_volume_nodes) {
    nodes::search_link_ops_for_basic_node(params);
  }
}

#ifdef WITH_OPENVDB
static openvdb::FloatGrid &get_resampled_grid(bke::VolumeGrid<float> &volume_grid,
                                              bke::VolumeTreeAccessToken &tree_token,
                                              const openvdb::math::Transform::Ptr &transform,
                                              openvdb::FloatGrid::Ptr &storage)
{
  const openvdb::FloatGrid &grid = volume_grid.grid(tree_token);
  if (grid.transform() == *transform) {
    return volume_grid.grid_for_write(tree_token);
  }

  storage = openvdb::FloatGrid::create();
  storage->setTransform(transform);

  /* TODO: #doResampleToMatch when the transform is affine and non-scaled may be faster. */
  openvdb::tools::resampleToMatch<openvdb::tools::BoxSampler>(grid, *storage);
  openvdb::tools::pruneLevelSet(storage->tree());

  return *storage;
}
#endif

static void node_geo_exec(GeoNodeExecParams params)
{
#ifdef WITH_OPENVDB
  bke::VolumeGrid<float> grid_a = params.extract_input<bke::VolumeGrid<float>>("SDF Grid 1");
  bke::VolumeGrid<float> grid_b = params.extract_input<bke::VolumeGrid<float>>("SDF Grid 2");
  if (!grid_a) {
    params.set_default_remaining_outputs();
    return;
  }
  if (!grid_b) {
    params.set_output("SDF Grid", std::move(grid_a));
    return;
  }

  bke::VolumeTreeAccessToken tree_token_a;
  bke::VolumeTreeAccessToken tree_token_b;
  openvdb::FloatGrid &vdb_a = grid_a.grid_for_write(tree_token_a);

  openvdb::FloatGrid::Ptr resampled_storage;
  openvdb::FloatGrid &vdb_b = get_resampled_grid(
      grid_b, tree_token_b, vdb_a.transformPtr(), resampled_storage);

  openvdb::tools::csgDifference(vdb_a, vdb_b);

  params.set_output("SDF Grid", std::move(grid_a));
#else
  node_geo_exec_with_missing_openvdb(params);
#endif
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(
      &ntype, GEO_NODE_SDF_GRID_DIFFERENCE, "SDF Grid Difference", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.gather_link_search_ops = node_gather_link_search_ops;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_sdf_grid_boolean_cc
