/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "NOD_rna_define.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "RNA_enum_types.hh"

#ifdef WITH_OPENVDB
#include <openvdb/tools/FastSweeping.h>
#include <openvdb/tools/Interpolation.h>
#endif

namespace blender::nodes::node_geo_extrapolate_grid_cc {

NODE_STORAGE_FUNCS(NodeGeometryExtrapolateGrid)

static void node_declare(NodeDeclarationBuilder &b)
{
  const bNode *node = b.node_or_null();
  if (!node) {
    return;
  }
  const NodeGeometryExtrapolateGrid &storage = node_storage(*node);
  const GeometryNodeGridExtrapolationInputType input_type = GeometryNodeGridExtrapolationInputType(
      storage.input_type);
  const eCustomDataType data_type = eCustomDataType(storage.data_type);

  switch (input_type) {
    case GEO_NODE_EXTRAPOLATE_GRID_INPUT_SDF:
      b.add_input<decl::Float>("SDF", "InputGrid").hide_value();
      break;
    case GEO_NODE_EXTRAPOLATE_GRID_INPUT_DENSITY:
      b.add_input<decl::Float>("Density", "InputGrid").hide_value();
      break;
  }
  grids::declare_grid_type_input(b, data_type, "Grid").hide_value();
  b.add_input(data_type, "Background");
  b.add_input(data_type, "Iso Value");
  b.add_input<decl::Int>("Iterations").default_value(1).min(1);

  grids::declare_grid_type_output(b, data_type, "Grid");
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiLayoutSetPropSep(layout, true);
  uiLayoutSetPropDecorate(layout, false);
  uiItemR(layout, ptr, "mode", UI_ITEM_NONE, "", ICON_NONE);
  uiItemR(layout, ptr, "input_type", UI_ITEM_NONE, "", ICON_NONE);
  uiItemR(layout, ptr, "data_type", UI_ITEM_NONE, "", ICON_NONE);
  uiItemR(layout, ptr, "fast_sweeping_region", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometryExtrapolateGrid *data = MEM_cnew<NodeGeometryExtrapolateGrid>(__func__);
  data->mode = GEO_NODE_EXTRAPOLATE_GRID_BOUNDARY_DIRICHLET;
  data->input_type = GEO_NODE_EXTRAPOLATE_GRID_INPUT_SDF;
  data->data_type = CD_PROP_FLOAT;
  data->fast_sweeping_region = GEO_NODE_FAST_SWEEPING_REGION_ALL;
  node->storage = data;
}

#ifdef WITH_OPENVDB

static openvdb::tools::FastSweepingDomain get_fast_sweeping_domain(
    const GeometryNodeFastSweepingRegion fast_sweeping_region)
{
  switch (fast_sweeping_region) {
  case GEO_NODE_FAST_SWEEPING_REGION_ALL:
      return openvdb::tools::FastSweepingDomain::SWEEP_ALL;
  case GEO_NODE_FAST_SWEEPING_REGION_GREATER_THAN_ISOVALUE:
    return openvdb::tools::FastSweepingDomain::SWEEP_GREATER_THAN_ISOVALUE;
  case GEO_NODE_FAST_SWEEPING_REGION_LESS_THAN_ISOVALUE:
    return openvdb::tools::FastSweepingDomain::SWEEP_LESS_THAN_ISOVALUE;
  }
  return openvdb::tools::FastSweepingDomain::SWEEP_ALL;
}

template<typename GridType> struct DirichletBoundaryOp {
  using ValueType = typename GridType::ValueType;
  using SamplerType = openvdb::tools::GridSampler<GridType, openvdb::tools::BoxSampler>;

  SamplerType sampler;

  DirichletBoundaryOp(const GridType &grid) : sampler(SamplerType(grid)) {}

  ValueType operator()(const openvdb::Vec3d &xyz) const {
    return sampler.isSample(xyz);
  }
};

struct ExtrapolateOp {
  GeoNodeExecParams params;
  GeometryNodeGridExtrapolationBoundaryMode boundary_mode;
  GeometryNodeGridExtrapolationInputType input_type;
  openvdb::tools::FastSweepingDomain fast_sweeping_domain;
  bke::VolumeGridPtr<float> input_grid;

  bke::GVolumeGridPtr result;

  template<typename T, typename BoundaryOpT>
  void extrapolate_with_boundary(const bke::VolumeGridPtr<T> &grid)
  {
    using GridType = typename bke::VolumeGridPtr<T>::GridType;
    using GridPtr = typename bke::VolumeGridPtr<T>::GridPtr;
    using GridConstPtr = typename bke::VolumeGridPtr<T>::GridConstPtr;
    using Converter = bke::GridConverter<T>;

    const GridConstPtr vdb_grid = grid.grid();
    BLI_assert(vdb_grid);
    const T background = this->params.extract_input<T>("Background");
    const float iso_value = this->params.extract_input<float>("Iso Value");
    const typename GridType::ValueType vdb_background = Converter::single_value_to_grid(
        background);

    BoundaryOpT boundary_op(*vdb_grid);

    GridPtr vdb_result;
    switch (this->input_type) {
      case GEO_NODE_EXTRAPOLATE_GRID_INPUT_SDF:
        vdb_result = openvdb::tools::sdfToExt(
            *this->input_grid.grid(), boundary_op, vdb_background, iso_value);
        break;
    case GEO_NODE_EXTRAPOLATE_GRID_INPUT_DENSITY:
        vdb_result = openvdb::tools::fogToExt(
            *this->input_grid.grid(), boundary_op, vdb_background, iso_value);
        break;
    }

    this->result = bke::GVolumeGridPtr(make_implicit_shared<bke::VolumeGrid>(vdb_result));
  }

  template<typename T> void operator()() {
    using GridType = typename bke::VolumeGridPtr<T>::GridType;

    const bke::VolumeGridPtr<T> grid = grids::extract_grid_input<T>(this->params, "Grid");

    if (!grid || !this->input_grid) {
      return;
    }
    switch (this->boundary_mode) {
      case GEO_NODE_EXTRAPOLATE_GRID_BOUNDARY_DIRICHLET:
        this->extrapolate_with_boundary<T, DirichletBoundaryOp<GridType>>(grid);
        break;
    }
  }
};

#endif /* WITH_OPENVDB */

static void node_geo_exec(GeoNodeExecParams params)
{
#ifdef WITH_OPENVDB
  const NodeGeometryExtrapolateGrid &storage = node_storage(params.node());
  const GeometryNodeGridExtrapolationBoundaryMode boundary_mode = GeometryNodeGridExtrapolationBoundaryMode(
      storage.mode);
  const GeometryNodeGridExtrapolationInputType input_type = GeometryNodeGridExtrapolationInputType(
      storage.input_type);
  const GeometryNodeFastSweepingRegion fast_sweeping_region = GeometryNodeFastSweepingRegion(
      storage.fast_sweeping_region);
  const eCustomDataType data_type = eCustomDataType(storage.data_type);

  const bke::VolumeGridPtr<float> input_grid = grids::extract_grid_input<float>(params, "Grid");

  const openvdb::tools::FastSweepingDomain fs_domain = get_fast_sweeping_domain(
      fast_sweeping_region);

  ExtrapolateOp extrapolate_op = {params, boundary_mode, input_type, fs_domain, input_grid};
  grids::apply(data_type, extrapolate_op);

  grids::set_output_grid(params, "Grid", data_type, extrapolate_op.result);
  params.set_default_remaining_outputs();
#else
  params.set_default_remaining_outputs();
  params.error_message_add(NodeWarningType::Error,
                           TIP_("Disabled, Blender was compiled without OpenVDB"));
#endif
}

static void node_rna(StructRNA *srna)
{
  static EnumPropertyItem mode_items[] = {
      {GEO_NODE_EXTRAPOLATE_GRID_BOUNDARY_DIRICHLET,
       "Dirichlet",
       0,
       "Dirichlet",
       "Extrapolate from existing values in the input grid"},
      {0, nullptr, 0, nullptr, nullptr},
  };
  static EnumPropertyItem input_type_items[] = {
      {GEO_NODE_EXTRAPOLATE_GRID_INPUT_SDF, "SDF", 0, "SDF", "Extrapolate an SDF grid"},
      {GEO_NODE_EXTRAPOLATE_GRID_INPUT_DENSITY,
       "DENSITY",
       0,
       "Density",
       "Extrapolate a density grid"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  RNA_def_node_enum(srna,
                    "mode",
                    "Mode",
                    "How the extrapolation is computed",
                    mode_items,
                    NOD_storage_enum_accessors(mode),
                    GEO_NODE_EXTRAPOLATE_GRID_BOUNDARY_DIRICHLET);
  RNA_def_node_enum(srna,
                    "input_type",
                    "Input Type",
                    "Expected kind of grid input data",
                    input_type_items,
                    NOD_storage_enum_accessors(input_type));
  RNA_def_node_enum(srna,
                    "data_type",
                    "Data Type",
                    "Type of grid data",
                    rna_enum_attribute_type_items,
                    NOD_storage_enum_accessors(data_type),
                    CD_PROP_FLOAT,
                    grids::grid_type_items_fn);
  RNA_def_node_enum(srna,
                    "fast_sweeping_region",
                    "Region",
                    "Region of voxels affected by the node",
                    rna_enum_fast_sweeping_region_items,
                    NOD_storage_enum_accessors(fast_sweeping_region));
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
