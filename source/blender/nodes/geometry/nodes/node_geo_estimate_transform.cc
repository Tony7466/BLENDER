/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_pointcloud_types.h"

#include "BKE_curves.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_instances.hh"
#include "BKE_material.h"
#include "BKE_mesh.hh"

#include <Eigen/Geometry>

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_estimate_transform_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Source Geometry").description("Source positions to align from");
  b.add_input<decl::Geometry>("Target Geometry").description("Target positions to align to");
  b.add_output<decl::Matrix>("Transform");
}

static Array<float3> get_positions(const GeometrySet &geometry_set)
{
  int count = 0;
  int total_num = 0;

  if (const Mesh *mesh = geometry_set.get_mesh()) {
    if (const VArray positions = *mesh->attributes().lookup<float3>("position")) {
      total_num += positions.size();
    }
    ++count;
  }

  if (const PointCloud *points = geometry_set.get_pointcloud()) {
    if (const VArray positions = *points->attributes().lookup<float3>("position")) {
      total_num += positions.size();
    }
    ++count;
  }

  if (const Curves *curves_id = geometry_set.get_curves()) {
    total_num += curves_id->geometry.wrap().evaluated_positions().size();
    ++count;
  }

  if (count == 0) {
    return Array<float3>();
  }

  Array<float3> positions(total_num);
  int offset = 0;

  if (const Mesh *mesh = geometry_set.get_mesh()) {
    if (const VArray varray = *mesh->attributes().lookup<float3>("position")) {
      varray.materialize(positions.as_mutable_span().slice(offset, varray.size()));
      offset += varray.size();
    }
  }

  if (const PointCloud *points = geometry_set.get_pointcloud()) {
    if (const VArray varray = *points->attributes().lookup<float3>("position")) {
      varray.materialize(positions.as_mutable_span().slice(offset, varray.size()));
      offset += varray.size();
    }
  }

  if (const Curves *curves_id = geometry_set.get_curves()) {
    const bke::CurvesGeometry &curves = curves_id->geometry.wrap();
    Span<float3> array = curves.evaluated_positions();
    positions.as_mutable_span().slice(offset, array.size()).copy_from(array);
    offset += array.size();
  }

  return positions;
}

static void calculate_matrix(const Array<float3> &source_pos,
                             const Array<float3> &target_pos,
                             float4x4 &result)
{
  const int64_t n = source_pos.size();
  BLI_assert(n == target_pos.size());

  // Assemble matrices for Eigen (TODO: find a faster way to do this)
  Eigen::Matrix3Xf source(3, n), target(3, n);
  for (int col = 0; col < n; ++col) {
    for (int row = 0; row < 3; ++row) {
      source(row, col) = source_pos[col][row];
      target(row, col) = target_pos[col][row];
    }
  }

  // Solve matrix using SVD, Eigen has this built-in
  // Scale is disabled since it's unreliable, it only works reliably for uniform, positive scaling
  // TODO: Solve scale/skew using a separate least squares method
  Eigen::Map<Eigen::Matrix4f>(result.base_ptr()) = Eigen::umeyama(source, target, false);
}

static void node_geo_exec(GeoNodeExecParams params)
{
  const GeometrySet source = params.extract_input<GeometrySet>("Source Geometry");
  const GeometrySet target = params.extract_input<GeometrySet>("Target Geometry");

  const Array<float3> source_pos = get_positions(source);
  const Array<float3> target_pos = get_positions(target);

  if (source_pos.size() != target_pos.size()) {
    params.error_message_add(
        NodeWarningType::Error,
        TIP_("Source and target geometry must have the same number of points"));
    params.set_default_remaining_outputs();
    return;
  }
  else if (source_pos.size() == 0) {
    params.error_message_add(NodeWarningType::Error, TIP_("Input has no points"));
    params.set_default_remaining_outputs();
    return;
  }

  float4x4 transform;
  calculate_matrix(source_pos, target_pos, transform);
  params.set_output("Transform", transform);
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(
      &ntype, GEO_NODE_ESTIMATE_TRANSFORM, "Estimate Transform", NODE_CLASS_GEOMETRY);
  ntype.geometry_node_execute = node_geo_exec;
  ntype.declare = node_declare;
  blender::bke::nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_estimate_transform_cc
