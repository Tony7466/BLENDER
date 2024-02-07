/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "DNA_mesh_types.h"

#include "BLI_task.hh"

#include "BKE_geometry_set.hh"
#include "BKE_lib_id.hh"
#include "BKE_volume.hh"
#include "BKE_volume_grid.hh"
#include "BKE_volume_openvdb.hh"

#include "NOD_rna_define.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "RNA_enum_types.hh"

namespace blender::nodes::node_geo_grid_primitive_sphere_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  const bNode *node = b.node_or_null();
  if (!node) {
    return;
  }

  eCustomDataType data_type = eCustomDataType(node->custom1);

  b.add_input(data_type, "Value").supports_field();
  b.add_input(data_type, "Background");
  b.add_input(data_type, "Tolerance")
      .description("Deactivate voxels withing tolerance of the background value");

  b.add_input<decl::Vector>("Min")
      .default_value(float3(-1.0f))
      .description("Minimum boundary of volume");
  b.add_input<decl::Vector>("Max")
      .default_value(float3(1.0f))
      .description("Maximum boundary of volume");

  b.add_input<decl::Int>("Resolution X")
      .default_value(32)
      .min(1)
      .description("Number of voxels in the X axis");
  b.add_input<decl::Int>("Resolution Y")
      .default_value(32)
      .min(1)
      .description("Number of voxels in the Y axis");
  b.add_input<decl::Int>("Resolution Z")
      .default_value(32)
      .min(1)
      .description("Number of voxels in the Z axis");

  grids::declare_grid_type_output(b, data_type, "Grid");
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

static void node_geo_exec(GeoNodeExecParams params)
{
#ifdef WITH_OPENVDB
  const float3 bounds_min = params.extract_input<float3>("Min");
  const float3 bounds_max = params.extract_input<float3>("Max");

  const int3 resolution = int3(params.extract_input<int>("Resolution X"),
                               params.extract_input<int>("Resolution Y"),
                               params.extract_input<int>("Resolution Z"));

  if (resolution.x < 1 || resolution.y < 1 || resolution.z < 1) {
    params.error_message_add(NodeWarningType::Error, TIP_("Resolution must be at least 1"));
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

  const double3 scale_fac = double3(bounds_max - bounds_min) / double3(resolution);
  if (!BKE_volume_grid_determinant_valid(scale_fac.x * scale_fac.y * scale_fac.z)) {
    params.error_message_add(NodeWarningType::Warning,
                             TIP_("Volume scale is lower than permitted by OpenVDB"));
    params.set_default_remaining_outputs();
    return;
  }

  /* Transform from index space to object space voxel corners. */
  const float4x4 transform = math::from_location<float4x4>(bounds_min) *
                             math::from_scale<float4x4>(float3(scale_fac));
  /* Transform voxel centers in object space to normalized bounds space. */
  const float4x4 inv_bounds_transform =
      math::from_scale<float4x4>(math::rcp(0.5f * float3(bounds_max - bounds_min))) *
      math::from_location<float4x4>(-0.5f * (bounds_min + bounds_max));

  const eCustomDataType data_type = eCustomDataType(params.node().custom1);
  BLI_assert(grid_type_supported(data_type));

  bke::GVolumeGrid grid = grids::try_capture_field_as_dense_grid(
      data_type,
      transform,
      resolution,
      [inv_bounds_transform](const float3 &position) {
        return math::length_squared(math::transform_point(inv_bounds_transform, position)) < 1.0f;
      },
      params.extract_input<GField>("Value"),
      params.extract_input<SocketValueVariant>("Background").get_single_ptr(),
      params.extract_input<SocketValueVariant>("Tolerance").get_single_ptr());

  params.set_output("Grid", grid);
#else
  node_geo_exec_with_missing_openvdb(params);
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
                    grid_custom_data_type_items_filter_fn);
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_GRID_PRIMITIVE_SPHERE, "Grid Sphere", NODE_CLASS_GEOMETRY);

  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.draw_buttons = node_layout;
  ntype.geometry_node_execute = node_geo_exec;
  nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_grid_primitive_sphere_cc
