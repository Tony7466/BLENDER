/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "NOD_rna_define.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "RNA_enum_types.hh"

#ifdef WITH_OPENVDB
#  include <openvdb/tools/FastSweeping.h>
#  include <openvdb/tools/Interpolation.h>
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
  b.add_input(data_type, "Boundary Value").supports_field();
  b.add_input(data_type, "Background");
  b.add_input(data_type, "Iso Value");
  b.add_input<decl::Int>("Iterations").default_value(1).min(1);

  grids::declare_grid_type_output(b, data_type, "Grid");
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiLayoutSetPropSep(layout, true);
  uiLayoutSetPropDecorate(layout, false);
  uiItemR(layout, ptr, "input_type", UI_ITEM_NONE, "", ICON_NONE);
  uiItemR(layout, ptr, "data_type", UI_ITEM_NONE, "", ICON_NONE);
  uiItemR(layout, ptr, "fast_sweeping_region", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometryExtrapolateGrid *data = MEM_cnew<NodeGeometryExtrapolateGrid>(__func__);
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

template<typename GridType> static int64_t get_voxel_count(const GridType &grid)
{
  return grid.tree().activeLeafVoxelCount();
}

template<typename GridType>
static void get_voxel_positions_span(GridType &grid, const MutableSpan<float3> positions)
{
  using TreeType = typename GridType::TreeType;
  using LeafManager = openvdb::tree::LeafManager<TreeType>;
  using LeafNodeType = typename TreeType::LeafNodeType;

  LeafManager leaf_mgr(grid.tree());
  /* XXX calculated twice (see store_voxel_values), can be done externally. */
  size_t *leaf_offsets = static_cast<size_t *>(
      MEM_malloc_arrayN(leaf_mgr.leafCount(), sizeof(size_t), __func__));
  size_t leaf_offsets_size = leaf_mgr.leafCount();
  leaf_mgr.getPrefixSum(leaf_offsets, leaf_offsets_size);

  leaf_mgr.foreach ([&](const LeafNodeType &leaf, const size_t leaf_index) {
    int64_t index = leaf_offsets[leaf_index];
    typename LeafNodeType::ValueOnCIter iter = leaf.cbeginValueOn();
    for (; iter; ++iter, ++index) {
      const openvdb::math::Coord coord = iter.getCoord();
      positions[index] = float3(coord.x(), coord.y(), coord.z());
    }
  });

  MEM_delete(leaf_offsets);
}

template<typename T, typename GridType>
static void store_voxel_values(GridType &grid, const Span<T> values)
{
  using TreeType = typename GridType::TreeType;
  using LeafManager = openvdb::tree::LeafManager<TreeType>;
  using LeafNodeType = typename TreeType::LeafNodeType;
  using Converter = bke::GridConverter<T>;

  LeafManager leaf_mgr(grid.tree());
  size_t *leaf_offsets = static_cast<size_t *>(
      MEM_malloc_arrayN(leaf_mgr.leafCount(), sizeof(size_t), __func__));
  size_t leaf_offsets_size = leaf_mgr.leafCount();
  leaf_mgr.getPrefixSum(leaf_offsets, leaf_offsets_size);

  leaf_mgr.foreach ([&](LeafNodeType &leaf, const size_t leaf_index) {
    int64_t index = leaf_offsets[leaf_index];
    typename LeafNodeType::ValueOnIter iter = leaf.beginValueOn();
    for (; iter; ++iter, ++index) {
      iter.setValue(Converter::single_value_to_grid(values[index]));
    }
  });

  MEM_delete(leaf_offsets);
}

template<typename GridType> class BoundaryFieldContext : public FieldContext {
 private:
  typename GridType::Ptr grid_;

 public:
  BoundaryFieldContext(typename GridType::Ptr grid) : grid_(std::move(grid)) {}

  GVArray get_varray_for_input(const fn::FieldInput &field_input,
                               const IndexMask & /*mask*/,
                               ResourceScope & /*scope*/) const override
  {
    const bke::AttributeFieldInput *attribute_field_input =
        dynamic_cast<const bke::AttributeFieldInput *>(&field_input);
    if (attribute_field_input == nullptr) {
      return {};
    }
    if (attribute_field_input->attribute_name() != "position") {
      return {};
    }

    Array<float3> positions(get_voxel_count(*grid_));
    get_voxel_positions_span(*grid_, positions);
    return VArray<float3>::ForContainer(std::move(positions));
  }
};

template<typename GridType> struct BoundaryOp {
  using ValueType = typename GridType::ValueType;
  using SamplerType = openvdb::tools::GridSampler<GridType, openvdb::tools::BoxSampler>;

  SamplerType sampler;

  BoundaryOp(const GridType &grid) : sampler(SamplerType(grid)) {}

  ValueType operator()(const openvdb::Vec3d &xyz) const
  {
    return sampler.isSample(xyz);
  }
};

struct ExtrapolateOp {
  GeoNodeExecParams params;
  GeometryNodeGridExtrapolationInputType input_type;
  openvdb::tools::FastSweepingDomain fast_sweeping_domain;
  bke::VolumeGridPtr<float> input_grid;

  bke::GVolumeGridPtr result;

  template<typename T> void operator()()
  {
    using GridType = typename bke::VolumeGridPtr<T>::GridType;
    using GridPtr = typename bke::VolumeGridPtr<T>::GridPtr;
    using Converter = bke::GridConverter<T>;

    if (!this->input_grid) {
      return;
    }

    const openvdb::FloatGrid::ConstPtr vdb_input_grid = input_grid.grid();
    const fn::Field<T> boundary_field = this->params.extract_input<fn::Field<T>>("Boundary Value");
    const T background = this->params.extract_input<T>("Background");
    const float iso_value = this->params.extract_input<float>("Iso Value");
    const int num_iter = this->params.extract_input<int>("Iterations");
    const typename GridType::ValueType vdb_background = Converter::single_value_to_grid(
        background);

    /* Compute boundary values on the SDF grid voxels.
     * In theory the boundary could also be evaluated as a VArray, if the boundary callback was
     * able to index it. Since the boundary callback only gets a openvdb::Vec3d there is no easy
     * way to get an index, so storing boundary values in a grid and using sampler is the best
     * option right now. */
    GridPtr vdb_boundary_grid = GridType::create(vdb_background);
    vdb_boundary_grid->insertMeta(*vdb_input_grid);
    vdb_boundary_grid->setTransform(vdb_input_grid->transform().copy());
    vdb_boundary_grid->topologyUnion(*vdb_input_grid);

    /* Evaluate boundary field and fill in the grid. */
    const int64_t voxels_num = get_voxel_count(*vdb_boundary_grid);
    BoundaryFieldContext<GridType> context(vdb_boundary_grid);
    fn::FieldEvaluator evaluator(context, voxels_num);
    Array<T> boundary_values(voxels_num);
    evaluator.add_with_destination(std::move(boundary_field), boundary_values.as_mutable_span());
    evaluator.evaluate();
    store_voxel_values(*vdb_boundary_grid, boundary_values.as_span());

    /* Callback for the fast-sweeping method, samples the boundary grid. */
    BoundaryOp<GridType> boundary_op(*vdb_boundary_grid);

    GridPtr vdb_result;
    switch (this->input_type) {
      case GEO_NODE_EXTRAPOLATE_GRID_INPUT_SDF:
        vdb_result = openvdb::tools::sdfToExt(*vdb_input_grid,
                                              boundary_op,
                                              vdb_background,
                                              iso_value,
                                              num_iter,
                                              fast_sweeping_domain);
        break;
      case GEO_NODE_EXTRAPOLATE_GRID_INPUT_DENSITY:
        vdb_result = openvdb::tools::fogToExt(*vdb_input_grid,
                                              boundary_op,
                                              vdb_background,
                                              iso_value,
                                              num_iter,
                                              fast_sweeping_domain);
        break;
    }
    if (vdb_result) {
      vdb_result->insertMeta(*vdb_input_grid);
      vdb_result->setTransform(vdb_input_grid->transform().copy());
    }
    this->result = bke::GVolumeGridPtr(make_implicit_shared<bke::VolumeGrid>(vdb_result));
  }
};

#endif /* WITH_OPENVDB */

static void node_geo_exec(GeoNodeExecParams params)
{
#ifdef WITH_OPENVDB
  const NodeGeometryExtrapolateGrid &storage = node_storage(params.node());
  const GeometryNodeGridExtrapolationInputType input_type = GeometryNodeGridExtrapolationInputType(
      storage.input_type);
  const GeometryNodeFastSweepingRegion fast_sweeping_region = GeometryNodeFastSweepingRegion(
      storage.fast_sweeping_region);
  const eCustomDataType data_type = eCustomDataType(storage.data_type);

  const bke::VolumeGridPtr<float> input_grid = grids::extract_grid_input<float>(params,
                                                                                "InputGrid");
  const openvdb::tools::FastSweepingDomain fs_domain = get_fast_sweeping_domain(
      fast_sweeping_region);

  ExtrapolateOp extrapolate_op = {params, input_type, fs_domain, input_grid};
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
