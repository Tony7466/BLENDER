/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#ifdef WITH_OPENVDB
#  include <openvdb/openvdb.h>
#  include <openvdb/tools/Composite.h>
#endif

#include "BKE_volume_grid.hh"

#include "GEO_volume_grid_resample.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_sdf_grid_intersection_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Float>("SDF Grids").hide_value().multi_input();
  b.add_output<decl::Float>("SDF Grid").hide_value();
}

static void node_geo_exec(GeoNodeExecParams params)
{
#ifdef WITH_OPENVDB
  Vector<SocketValueVariant> inputs = params.extract_input<Vector<SocketValueVariant>>(
      "SDF Grids");
  Vector<bke::VolumeGrid<float>> grids;
  for (SocketValueVariant &input : inputs) {
    if (bke::VolumeGrid<float> grid = input.extract<bke::VolumeGrid<float>>()) {
      grids.append(std::move(grid));
    }
  }
  if (grids.is_empty()) {
    params.set_default_remaining_outputs();
    return;
  }

  bke::VolumeTreeAccessToken result_token;
  openvdb::FloatGrid &result_grid = grids.first().grid_for_write(result_token);
  const openvdb::math::Transform &transform = result_grid.transform();

  for (bke::VolumeGrid<float> &volume_grid : grids) {
    bke::VolumeTreeAccessToken tree_token;
    openvdb::FloatGrid::Ptr resampled_storage;
    openvdb::FloatGrid &grid = geometry::resample_sdf_grid_if_necessary(
        volume_grid, tree_token, transform, resampled_storage);
    openvdb::tools::csgIntersection(result_grid, grid);
  }

  params.set_output("SDF Grid", std::move(grids.first()));
#else
  node_geo_exec_with_missing_openvdb(params);
#endif
}

static void node_register()
{
  static bNodeType ntype;
  geo_node_type_base(
      &ntype, GEO_NODE_SDF_GRID_INTERSECTION, "SDF Grid Intersection", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.gather_link_search_ops = search_link_ops_for_volume_grid_node;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_sdf_grid_intersection_cc
