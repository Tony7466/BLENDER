/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "BKE_pointcloud.h"

#include "NOD_rna_define.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "RNA_enum_types.hh"

namespace blender::nodes::node_geo_grid_to_points_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  const bNode *node = b.node_or_null();
  if (!node) {
    return;
  }

  eCustomDataType data_type = eCustomDataType(node->custom1);

  grids::declare_grid_type_input(b, data_type, "Grid");
  b.add_input<decl::Bool>("Use Tiles")
      .default_value(false)
      .description("Output tiles as single points instead of generating a point for each voxel");
  b.add_input<decl::Bool>("Use Inactive")
      .default_value(false)
      .description("Output inactive voxels and tiles as well");

  b.add_output<decl::Geometry>("Points").description(
      "Point geometry representing grid voxels and tiles");
  b.add_output<decl::Vector>("Scale").description("Voxel size of the grid");
  b.add_output<decl::Vector>("Origin").description("Origin of the grid");
  b.add_output<decl::Rotation>("Rotation").description("Orientation of the grid");
  b.add_output<decl::Int>("Depth").description("Number of node levels in the tree");
  b.add_output<decl::Vector>("Coordinate")
      .description("Index-space coordinate of the voxel")
      .field_on_all();
  b.add_output<decl::Vector>("Bounds Min")
      .description("Minimum voxel coordinate covered by a node")
      .field_on_all();
  b.add_output<decl::Vector>("Bounds Max")
      .description("Maximum voxel coordinate covered by a node")
      .field_on_all();
  b.add_output<decl::Int>("Level")
      .description("Node level, -1 = leaf voxel, 0 = leaf node, 1,2,.. = other node")
      .field_on_all();
  b.add_output<decl::Bool>("Active")
      .description("Active state of the voxel or tile")
      .field_on_all();

  b.add_output(data_type, "Background").description("Value of the grid in empty regions");
  b.add_output(data_type, "Value")
      .description("Value stored in grid voxels and tiles")
      .field_on_all();
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiLayoutSetPropSep(layout, true);
  uiLayoutSetPropDecorate(layout, false);
  uiItemR(layout, ptr, "data_type", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = CD_PROP_FLOAT;
}

template<typename T> struct PointData {
  using GridType = typename bke::detail::VolumeGridType<T>;
  using GridValueType = typename GridType::ValueType;
  using Converter = bke::grids::Converter<T>;
  using AttributeValueType = typename Converter::AttributeValueType;

  int64_t num_points;
  /* Grid transform. */
  float4x4 transform;
  /* Number of node levels in the tree. */
  int depth;

  /* Coordinates of the node in the tree. */
  Vector<float3> coords;
  /* Node coordinate min/max bounds. */
  Vector<float3> bounds_min;
  Vector<float3> bounds_max;
  /* Depth of the node:
   * 0 = leaf node
   * 1..depth-1 = intermediate node
   * depth = root node
   * -1 = voxel (not an actual node in the tree)
   */
  Vector<int> levels;
  /* Active state of the voxel. */
  Vector<bool> active_state;

  AttributeValueType background_value;
  Vector<AttributeValueType> values;

  PointData(const GridType &grid,
            const bool use_tiles,
            const bool output_bounds_min,
            const bool output_bounds_max,
            const bool output_levels,
            const bool output_active,
            const bool output_values)
  {
    this->transform = grids::vdb_transform_to_matrix(grid.transform());
    this->depth = grid.tree().treeDepth();

    const size_t num_active_voxels = grid.tree().activeVoxelCount();
    const size_t num_active_tiles = grid.tree().activeTileCount();
    this->num_points = use_tiles ? num_active_voxels + num_active_tiles : num_active_voxels;

    this->coords.reinitialize(this->num_points);
    if (output_bounds_min) {
      this->bounds_min.reinitialize(this->num_points);
    }
    if (output_bounds_max) {
      this->bounds_max.reinitialize(this->num_points);
    }
    if (output_levels) {
      this->levels.reinitialize(this->num_points);
    }
    if (output_active) {
      this->active_state.reinitialize(this->num_points);
    }
    if (output_values) {
      this->values.reinitialize(this->num_points);
    }
  }

  void add_voxel_point(int64_t &cur_point,
                       const openvdb::Coord &coord,
                       const bool active,
                       const GridValueType &value)
  {
    this->coords[cur_point] = float3(coord.x(), coord.y(), coord.z());
    if (!this->bounds_min.is_empty()) {
      this->bounds_min[cur_point] = float3(coord.x(), coord.y(), coord.z());
    }
    if (!this->bounds_max.is_empty()) {
      this->bounds_max[cur_point] = float3(coord.x(), coord.y(), coord.z());
    }
    if (!this->levels.is_empty()) {
      /* -1 indicates voxels. */
      this->levels[cur_point] = -1;
    }
    if (!this->active_state.is_empty()) {
      this->active_state[cur_point] = active;
    }
    if (!this->values.is_empty()) {
      this->values[cur_point] = Converter::to_blender(value);
    }
    ++cur_point;
  }
};

struct GridToPointsOp {
  GeoNodeExecParams params;

  template<typename T> void operator()()
  {
    using GridType = typename bke::VolumeGridPtr<T>::GridType;
    using TreeType = typename GridType::TreeType;

    const bke::VolumeGridPtr<T> grid_ptr = grids::extract_grid_input<T>(params, "Grid");
    if (!grid_ptr) {
      params.set_default_remaining_outputs();
      return;
    }
    const GridType &grid = *grid_ptr.grid();
    const bool use_tiles = params.extract_input<bool>("Use Tiles");

    PointData<T> point_data(grid,
                            use_tiles,
                            params.output_is_required("Bounds Min"),
                            params.output_is_required("Bounds Max"),
                            params.output_is_required("Level"),
                            params.output_is_required("Active"),
                            params.output_is_required("Value"));

    int64_t cur_point = 0;
    for (typename TreeType::ValueOnCIter value_iter = grid.tree().cbeginValueOn(); value_iter;
         ++value_iter)
    {
      point_data.add_voxel_point(
          cur_point, value_iter.getCoord(), value_iter.isValueOn(), value_iter.getValue());
    }

    PointCloud *points = BKE_pointcloud_new_nomain(point_data.num_points);
    bke::MutableAttributeAccessor attributes = points->attributes_for_write();

    float3 translation;
    math::Quaternion rotation;
    float3 scale;
    math::to_loc_rot_scale(point_data.transform, translation, rotation, scale);
    params.set_output("Origin", translation);
    params.set_output("Rotation", rotation);
    params.set_output("Scale", scale);
    params.set_output("Depth", point_data.depth);
    params.set_output<T>("Background", std::move(point_data.background_value));

    {
      MutableSpan<float3> positions = points->positions_for_write();
      for (const int i : positions.index_range()) {
        positions[i] = math::transform_point(point_data.transform, point_data.coords[i]);
      }
    }
    if (AnonymousAttributeIDPtr attribute_id = params.get_output_anonymous_attribute_id_if_needed(
            "Coordinate"))
    {
      attributes.add<float3>(*attribute_id,
                             ATTR_DOMAIN_POINT,
                             bke::AttributeInitVArray(VArray<float3>::ForSpan(point_data.coords)));
    }
    if (AnonymousAttributeIDPtr attribute_id = params.get_output_anonymous_attribute_id_if_needed(
            "Bounds Min"))
    {
      attributes.add<float3>(
          *attribute_id,
          ATTR_DOMAIN_POINT,
          bke::AttributeInitVArray(VArray<float3>::ForSpan(point_data.bounds_min)));
    }
    if (AnonymousAttributeIDPtr attribute_id = params.get_output_anonymous_attribute_id_if_needed(
            "Bounds Max"))
    {
      attributes.add<float3>(
          *attribute_id,
          ATTR_DOMAIN_POINT,
          bke::AttributeInitVArray(VArray<float3>::ForSpan(point_data.bounds_max)));
    }
    if (AnonymousAttributeIDPtr attribute_id = params.get_output_anonymous_attribute_id_if_needed(
            "Level"))
    {
      attributes.add<int>(*attribute_id,
                          ATTR_DOMAIN_POINT,
                          bke::AttributeInitVArray(VArray<int>::ForSpan(point_data.levels)));
    }
    if (AnonymousAttributeIDPtr attribute_id = params.get_output_anonymous_attribute_id_if_needed(
            "Active"))
    {
      attributes.add<bool>(
          *attribute_id,
          ATTR_DOMAIN_POINT,
          bke::AttributeInitVArray(VArray<bool>::ForSpan(point_data.active_state)));
    }
    if (AnonymousAttributeIDPtr attribute_id = params.get_output_anonymous_attribute_id_if_needed(
            "Value"))
    {
      attributes.add<T>(*attribute_id,
                        ATTR_DOMAIN_POINT,
                        bke::AttributeInitVArray(VArray<T>::ForSpan(point_data.values)));
    }

    params.set_output("Points", bke::GeometrySet::from_pointcloud(points));
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
#ifdef WITH_OPENVDB
  const eCustomDataType data_type = eCustomDataType(params.node().custom1);
  BLI_assert(grids::grid_type_supported(data_type));

  GridToPointsOp grid_to_points_op = {params};
  grids::apply(data_type, grid_to_points_op);
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
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_GRID_TO_POINTS, "Grid to Points", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.draw_buttons = node_layout;
  nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_grid_to_points_cc
