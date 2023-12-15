/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "NOD_rna_define.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "RNA_enum_types.hh"

#ifdef WITH_OPENVDB
#  include <openvdb/tools/PoissonSolver.h>
#endif

namespace blender::nodes::node_geo_grid_capture_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  const bNode *node = b.node_or_null();
  if (!node) {
    return;
  }
  // const NodeGeometryExtrapolateGrid &storage = node_storage(*node);
  // const GeometryNodeGridExtrapolationInputType input_type =
  // GeometryNodeGridExtrapolationInputType(
  //    storage.input_type);
  // const eCustomDataType data_type = eCustomDataType(storage.data_type);

  b.add_input<decl::Float>("Grid").hide_value();
  b.add_input<decl::Float>("Boundary").hide_value();

  b.add_output<decl::Float>("Grid");
  b.add_output<decl::Bool>("Success");
  b.add_output<decl::Int>("Iterations");
  b.add_output<decl::Float>("Absolute Error");
  b.add_output<decl::Float>("Relative Error");
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA * /*ptr*/)
{
  uiLayoutSetPropSep(layout, true);
  uiLayoutSetPropDecorate(layout, false);
  // uiItemR(layout, ptr, "input_type", UI_ITEM_NONE, "", ICON_NONE);
  // uiItemR(layout, ptr, "data_type", UI_ITEM_NONE, "", ICON_NONE);
  // uiItemR(layout, ptr, "fast_sweeping_region", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode * /*node*/) {}

struct BoundaryOp {
  using ValueType = openvdb::tools::poisson::LaplacianMatrix::ValueType;

  typename openvdb::FloatGrid::ConstAccessor accessor;

  void operator()(
      const openvdb::Coord &ijk,           // coordinates of a boundary voxel
      const openvdb::Coord &ijk_neighbor,  // coordinates of an exterior neighbor of ijk
      ValueType &source,                   // element of b corresponding to ijk
      ValueType &diagonal                  // element of Laplacian matrix corresponding to ijk
  ) const
  {
    if (accessor.isValueOn(ijk_neighbor)) {
      /* Solid boundary, add source values. */
      source += accessor.getValue(ijk_neighbor);
    }
    else {
      /* Open boundary, remove interaction with neighbor voxel. */
      diagonal -= 1;
    }
    UNUSED_VARS(ijk);
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
#ifdef WITH_OPENVDB
  const bke::VolumeGridPtr<float> input_grid = grids::extract_grid_input<float>(params, "Grid");
  const bke::VolumeGridPtr<float> boundary_grid = grids::extract_grid_input<float>(params,
                                                                                   "Boundary");
  if (!input_grid) {
    grids::set_output_grid(params, "Grid", CD_PROP_FLOAT, input_grid);
    params.set_default_remaining_outputs();
  }

  const openvdb::FloatTree &input_tree = input_grid.grid()->tree();

  BoundaryOp boundary_op{boundary_grid.grid()->getAccessor()};

  const double epsilon = openvdb::math::Delta<float>::value();
  openvdb::math::pcg::State pcg_state = openvdb::math::pcg::terminationDefaults<float>();
  pcg_state.iterations = 200;
  pcg_state.relativeError = pcg_state.absoluteError = epsilon;

  openvdb::util::NullInterrupter interrupter;
  const bool staggered = false;
  using PreconditionerType = openvdb::math::pcg::IncompleteCholeskyPreconditioner<
      openvdb::tools::poisson::LaplacianMatrix>;

  openvdb::FloatTree::Ptr output_tree =
      openvdb::tools::poisson::solveWithBoundaryConditionsAndPreconditioner<PreconditionerType>(
          input_tree, boundary_op, pcg_state, interrupter, staggered);

  openvdb::FloatGrid::Ptr output_grid_vdb = input_grid.grid()->copyWithNewTree();
  output_grid_vdb->setTree(output_tree);
  bke::GVolumeGridPtr output_grid = bke::make_volume_grid_ptr(output_grid_vdb,
                                                              VOLUME_TREE_SOURCE_GENERATED);

  grids::set_output_grid(params, "Grid", CD_PROP_FLOAT, output_grid);
  params.set_output<bool>("Success", std::move(pcg_state.success));
  params.set_output<int>("Iterations", std::move(pcg_state.iterations));
  params.set_output<float>("Absolute Error", float(pcg_state.absoluteError));
  params.set_output<float>("Relative Error", float(pcg_state.relativeError));
#else
  params.set_default_remaining_outputs();
  params.error_message_add(NodeWarningType::Error,
                           TIP_("Disabled, Blender was compiled without OpenVDB"));
#endif
}

static void node_rna(StructRNA * /*srna*/) {}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(
      &ntype, GEO_NODE_GRID_POISSON_SOLVER, "Grid Poisson Solver", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.draw_buttons = node_layout;
  nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_grid_capture_cc
