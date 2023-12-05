/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "NOD_rna_define.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "RNA_enum_types.hh"

namespace blender::nodes::node_geo_grid_capture_grid_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  const bNode *node = b.node_or_null();
  if (!node) {
    return;
  }

  eCustomDataType data_type = eCustomDataType(node->custom1);
  eCustomDataType topo_data_type = eCustomDataType(node->custom2);

  b.add_input(data_type, "Value").supports_field();
  b.add_input(data_type, "Background");

  b.add_input(topo_data_type, "Topology Grid").hide_value();

  grids::declare_grid_type_output(b, data_type, "Grid");
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiLayoutSetPropSep(layout, true);
  uiLayoutSetPropDecorate(layout, false);
  uiItemR(layout, ptr, "data_type", UI_ITEM_NONE, "", ICON_NONE);
  uiItemR(layout, ptr, "topology_data_type", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = CD_PROP_FLOAT;
  node->custom2 = CD_PROP_FLOAT;
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

  const openvdb::math::Transform &transform = grid.transform();

  leaf_mgr.foreach ([&](const LeafNodeType &leaf, const size_t leaf_index) {
    int64_t index = leaf_offsets[leaf_index];
    typename LeafNodeType::ValueOnCIter iter = leaf.cbeginValueOn();
    for (; iter; ++iter, ++index) {
      const openvdb::Vec3d pos = transform.indexToWorld(iter.getCoord());
      positions[index] = float3(pos.x(), pos.y(), pos.z());
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
  using Converter = bke::grids::Converter<T>;

  LeafManager leaf_mgr(grid.tree());
  size_t *leaf_offsets = static_cast<size_t *>(
      MEM_malloc_arrayN(leaf_mgr.leafCount(), sizeof(size_t), __func__));
  size_t leaf_offsets_size = leaf_mgr.leafCount();
  leaf_mgr.getPrefixSum(leaf_offsets, leaf_offsets_size);

  leaf_mgr.foreach ([&](LeafNodeType &leaf, const size_t leaf_index) {
    int64_t index = leaf_offsets[leaf_index];
    typename LeafNodeType::ValueOnIter iter = leaf.beginValueOn();
    for (; iter; ++iter, ++index) {
      iter.setValue(Converter::to_openvdb(values[index]));
    }
  });

  MEM_delete(leaf_offsets);
}

template<typename GridType> class CaptureFieldContext : public FieldContext {
 private:
  typename GridType::Ptr grid_;

 public:
  CaptureFieldContext(typename GridType::Ptr grid) : grid_(std::move(grid)) {}

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

struct CaptureGridOp {
  GeoNodeExecParams params;

  bke::GVolumeGridPtr result;

  template<typename T> void operator()()
  {
    using GridType = typename bke::VolumeGridPtr<T>::GridType;
    using GridPtr = typename bke::VolumeGridPtr<T>::GridPtr;
    using Converter = bke::grids::Converter<T>;

    const fn::Field<T> value_field = this->params.extract_input<fn::Field<T>>("Value");
    const T background = this->params.extract_input<T>("Background");
    const typename GridType::ValueType vdb_background = Converter::to_openvdb(background);

    /* Evaluate value field and fill in the grid. */
    const eCustomDataType topo_data_type = eCustomDataType(params.node().custom2);
    const bke::GVolumeGridPtr topo_grid = grids::extract_grid_input(
        params, "Topology Grid", topo_data_type);
    if (!topo_grid) {
      /* TODO should use topology union of inputs in this case. */
      return;
    }

    const GridPtr output_grid = GridType::create(vdb_background);
    output_grid->setTransform(topo_grid->grid()->transform().copy());
    output_grid->insertMeta(*topo_grid->grid());
    output_grid->topologyUnion(*topo_grid->grid());

    const int64_t voxels_num = get_voxel_count(*output_grid);
    CaptureFieldContext<GridType> context(output_grid);
    fn::FieldEvaluator evaluator(context, voxels_num);
    Array<T> values(voxels_num);
    evaluator.add_with_destination(std::move(value_field), values.as_mutable_span());
    evaluator.evaluate();
    store_voxel_values(*output_grid, values.as_span());

    this->result = bke::make_volume_grid_ptr(std::move(output_grid));
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
#ifdef WITH_OPENVDB
  const eCustomDataType data_type = eCustomDataType(params.node().custom1);
  const eCustomDataType topo_data_type = eCustomDataType(params.node().custom2);
  BLI_assert(grids::grid_type_supported(data_type));
  BLI_assert(grids::grid_type_supported(topo_data_type));

  CaptureGridOp capture_op = {params};
  grids::apply(data_type, capture_op);

  grids::set_output_grid(params, "Grid", data_type, capture_op.result);
#else
  params.set_default_remaining_outputs();
  params.error_message_add(NodeWarningType::Error,
                           TIP_("Disabled, Blender was compiled without OpenVDB"));
#endif
}

static void node_rna(StructRNA *srna)
{
  RNA_def_node_enum(srna,
                    "data_type",
                    "Data Type",
                    "Type of grid data",
                    rna_enum_attribute_type_items,
                    NOD_inline_enum_accessors(custom1),
                    CD_PROP_FLOAT,
                    grids::grid_type_items_fn);

  RNA_def_node_enum(srna,
                    "topology_data_type",
                    "Topology Data Type",
                    "Type of the topology grid",
                    rna_enum_attribute_type_items,
                    NOD_inline_enum_accessors(custom2),
                    CD_PROP_FLOAT,
                    grids::grid_type_items_fn);
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_GRID_CAPTURE, "Capture Grid", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.draw_buttons = node_layout;
  nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_grid_capture_grid_cc
