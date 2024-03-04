/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */
#define WITH_OPENVDB
#ifdef WITH_OPENVDB
#  include <openvdb/openvdb.h>
#  include <openvdb/tools/Composite.h>
#endif

#include "BKE_volume_grid.hh"

#include "GEO_volume_grid_resample.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_sdf_grid_difference_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Float>("SDF Grid").hide_value();
  b.add_input<decl::Float>("Operands").hide_value().multi_input();
  b.add_output<decl::Float>("SDF Grid").hide_value();
}

static void node_geo_exec(GeoNodeExecParams params)
{
#ifdef WITH_OPENVDB
  bke::VolumeGrid<float> grid_a = params.extract_input<bke::VolumeGrid<float>>("SDF Grid");
  if (!grid_a) {
    params.set_default_remaining_outputs();
    return;
  }
  Vector<SocketValueVariant> inputs = params.extract_input<Vector<SocketValueVariant>>("Operands");
  Vector<bke::VolumeGrid<float>> operands;
  for (SocketValueVariant &input : inputs) {
    if (bke::VolumeGrid<float> grid = input.extract<bke::VolumeGrid<float>>()) {
      operands.append(std::move(grid));
    }
  }
  if (operands.is_empty()) {
    params.set_output("SDF Grid", std::move(grid_a));
    return;
  }

  bke::VolumeTreeAccessToken result_token;
  openvdb::FloatGrid &result_grid = grid_a.grid_for_write(result_token);
  const openvdb::math::Transform &transform = result_grid.transform();

  for (bke::VolumeGrid<float> &volume_grid : operands) {
    bke::VolumeTreeAccessToken tree_token;
    std::shared_ptr<openvdb::FloatGrid> resampled_storage;
    openvdb::FloatGrid &grid = geometry::resample_sdf_grid_if_necessary(
        volume_grid, tree_token, transform, resampled_storage);

    try {
      openvdb::tools::csgDifference(result_grid, grid);
    }
    catch (const openvdb::ValueError & /*ex*/) {
      /* May happen if a grid is empty. */
      params.set_default_remaining_outputs();
      return;
    }
  }

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
  ntype.gather_link_search_ops = search_link_ops_for_volume_grid_node;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_sdf_grid_difference_cc
