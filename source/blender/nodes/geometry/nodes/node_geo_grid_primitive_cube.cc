/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#ifdef WITH_OPENVDB
#include <openvdb/tools/Dense.h>
#endif

#include "node_geometry_util.hh"

#include "DNA_mesh_types.h"

#include "BLI_task.hh"

#include "BKE_geometry_set.hh"
#include "BKE_lib_id.hh"
#include "BKE_volume.hh"
#include "BKE_volume_grid.hh"
#include "BKE_volume_openvdb.hh"

namespace blender::nodes::node_geo_grid_primitive_cube_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  const bNode *node = b.node_or_null();
  if (!node) {
    return;
  }

  eCustomDataType data_type = eCustomDataType(node->custom1);

  b.add_input<decl::MaskGrid>("Grid");
  b.add_input(data_type, "Value").supports_field();
  b.add_input(data_type, "Background");

  b.add_input<decl::Vector>("Min")
      .default_value(float3(-1.0f))
      .description("Minimum boundary of volume");
  b.add_input<decl::Vector>("Max")
      .default_value(float3(1.0f))
      .description("Maximum boundary of volume");

  b.add_input<decl::Int>("Resolution X")
      .default_value(32)
      .min(2)
      .description("Number of voxels in the X axis");
  b.add_input<decl::Int>("Resolution Y")
      .default_value(32)
      .min(2)
      .description("Number of voxels in the Y axis");
  b.add_input<decl::Int>("Resolution Z")
      .default_value(32)
      .min(2)
      .description("Number of voxels in the Z axis");

  grids::declare_grid_type_output(b, data_type, "Grid");
}

class DensePrimitiveFieldContext : public FieldContext {
 private:
  float4x4 transform_;
  int3 resolution_;

  Array<float3> positions_;

 public:
  DensePrimitiveFieldContext(const float4x4 &transform, const int3 resolution)
      : transform_(transform), resolution_(resolution)
  {
    positions_.reinitialize(points_num());
    threading::parallel_for_each(IndexRange(resolution_.x), [&](const IndexRange x_range) {
      /* Start indexing at current X slice. */
      int64_t index = x_range.start() * resolution_.y * resolution_.z;
      for (const int64_t x : x_range) {
        for (const int64_t y : IndexRange(resolution_.y)) {
          for (const int64_t z : IndexRange(resolution_.z)) {
            positions_[index] = math::transform_point(transform_, float3(x, y, z));
            index++;
          }
        }
      }
    });
  }

  int64_t points_num() const
  {
    return int64_t(resolution_.x) * int64_t(resolution_.y) * int64_t(resolution_.z);
  }

  IndexRange points_range() const
  {
    return IndexRange(points_num());
  }

  GVArray get_varray_for_input(const FieldInput &field_input,
                               const IndexMask & /*mask*/,
                               ResourceScope & /*scope*/) const
  {
    const bke::AttributeFieldInput *attribute_field_input =
        dynamic_cast<const bke::AttributeFieldInput *>(&field_input);
    if (attribute_field_input == nullptr) {
      return {};
    }
    if (attribute_field_input->attribute_name() == "position") {
      return VArray<float3>::ForSpan(positions_);
    }
    return {};
  }
};

struct MakeCubeOp {
  GeoNodeExecParams params;
  float4x4 transform;
  int3 resolution;

  template<typename T> bke::GVolumeGrid operator()()
  {
    using GridType = bke::OpenvdbGridType<T>;
    using Converter = bke::grids::Converter<T>;
    using ValueType = typename GridType::ValueType;

    //const eCustomDataType data_type = eCustomDataType(params.node().custom1);
    //const auto topo_grid = this->params.extract_input<bke::GVolumeGrid>("Grid");
    const Field<T> value_field = this->params.extract_input<Field<T>>("Value");
    const T background = this->params.extract_input<T>("Background");

    /* Evaluate input field on a 3D grid. */
    DensePrimitiveFieldContext context(transform, resolution);
    FieldEvaluator evaluator(context, context.points_num());
    Array<ValueType> values(context.points_num());
    evaluator.add_with_destination(std::move(value_field), values.as_mutable_span());
    evaluator.evaluate();

    /* Store resulting values in openvdb grid. */
    const ValueType vdb_background = Converter::to_openvdb(background);
    typename GridType::Ptr grid = GridType::create(vdb_background);
    grid->setGridClass(openvdb::GRID_FOG_VOLUME);

    openvdb::tools::Dense<ValueType, openvdb::tools::LayoutZYX> dense_grid{
        openvdb::math::CoordBBox({0, 0, 0},
                                 {resolution.x - 1, resolution.y - 1, resolution.z - 1}),
        values.data()};
    openvdb::tools::copyFromDense(dense_grid, *grid, 0.0f);

    grid->setTransform(BKE_volume_matrix_to_vdb_transform(transform));

    return bke::GVolumeGrid(std::move(grid));
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
#ifdef WITH_OPENVDB
  const float3 bounds_min = params.extract_input<float3>("Min");
  const float3 bounds_max = params.extract_input<float3>("Max");

  const int3 resolution = int3(params.extract_input<int>("Resolution X"),
                               params.extract_input<int>("Resolution Y"),
                               params.extract_input<int>("Resolution Z"));

  if (resolution.x < 2 || resolution.y < 2 || resolution.z < 2) {
    params.error_message_add(NodeWarningType::Error, TIP_("Resolution must be greater than 1"));
    params.set_default_remaining_outputs();
    return;
  }

  if (bounds_min.x == bounds_max.x || bounds_min.y == bounds_max.y || bounds_min.z == bounds_max.z)
  {
    params.error_message_add(NodeWarningType::Error,
                             TIP_("Bounding box volume must be greater than 0"));
    params.set_default_remaining_outputs();
    return;
  }

  const double3 scale_fac = double3(bounds_max - bounds_min) / double3(resolution - 1);
  if (!BKE_volume_grid_determinant_valid(scale_fac.x * scale_fac.y * scale_fac.z)) {
    params.error_message_add(NodeWarningType::Warning,
                             TIP_("Volume scale is lower than permitted by OpenVDB"));
    params.set_default_remaining_outputs();
    return;
  }

  const float4x4 transform = math::from_location<float4x4>(bounds_min) *
                             math::from_scale<float4x4>(float3(scale_fac)) *
                             math::from_location<float4x4>(float3(-0.5f));

  //Field<float> input_field = params.extract_input<Field<float>>("Density");

  ///* Evaluate input field on a 3D grid. */
  //Grid3DFieldContext context(resolution, bounds_min, bounds_max);
  //FieldEvaluator evaluator(context, context.points_num());
  //Array<float> densities(context.points_num());
  //evaluator.add_with_destination(std::move(input_field), densities.as_mutable_span());
  //evaluator.evaluate();

  ///* Store resulting values in openvdb grid. */
  //const float background = params.extract_input<float>("Background");
  //openvdb::FloatGrid::Ptr grid = openvdb::FloatGrid::create(background);
  //grid->setGridClass(openvdb::GRID_FOG_VOLUME);

  //openvdb::tools::Dense<float, openvdb::tools::LayoutZYX> dense_grid{
  //    openvdb::math::CoordBBox({0, 0, 0}, {resolution.x - 1, resolution.y - 1, resolution.z - 1})};
  //openvdb::tools::copyFromDense(dense_grid, *grid, 0.0f);

  //grid->transform().preTranslate(openvdb::math::Vec3<float>(-0.5f));
  //grid->transform().postScale(openvdb::math::Vec3<double>(scale_fac.x, scale_fac.y, scale_fac.z));
  //grid->transform().postTranslate(
  //    openvdb::math::Vec3<float>(bounds_min.x, bounds_min.y, bounds_min.z));

  //Volume *volume = reinterpret_cast<Volume *>(BKE_id_new_nomain(ID_VO, nullptr));
  //BKE_volume_grid_add_vdb(*volume, "density", std::move(grid));

  //GeometrySet r_geometry_set;
  //r_geometry_set.replace_volume(volume);
  //params.set_output("Volume", r_geometry_set);

  const eCustomDataType data_type = eCustomDataType(params.node().custom1);
  BLI_assert(grid_type_supported(data_type));

  MakeCubeOp op = {params, transform, resolution};
  bke::GVolumeGrid grid = grids::apply(data_type, op);

  params.set_output("Grid", grid);
#else
  node_geo_exec_with_missing_openvdb(params);
#endif
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_GRID_PRIMITIVE_CUBE, "Grid Cube", NODE_CLASS_GEOMETRY);

  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_grid_primitive_cube_cc
