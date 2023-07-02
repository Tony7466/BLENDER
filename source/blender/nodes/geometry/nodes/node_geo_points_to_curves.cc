/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

//#include "BLI_array.hh"
//#include "BLI_vector_set.hh"
//#include "BLI_vector.hh"
//#include "BLI_index_range.hh"
//#include "BLI_offsets_indices.hh"

//#include "BKE_curves.hh"
#include "BKE_geometry_set.hh"
//#include "BKE_geometry_fields.hh"

#include "DNA_pointcloud_types.h"

namespace blender::nodes::node_geo_points_to_curves_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Points");
  b.add_input<decl::Int>("Curve Group ID").field_on_all().hide_value();
  b.add_input<decl::Float>("Length in Curve").field_on_all().hide_value();

  b.add_output<decl::Geometry>("Curves").propagate_all();
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet points_geometry = params.extract_input<GeometrySet>("Points");
  Field<int> curves_group_id_field = params.extract_input<Field<int>>("Curve Group ID");
  Field<float> length_in_curve_field = params.extract_input<Field<float>>("Length in Curve");

  if (!points_geometry.has_pointcloud()) {
    params.set_default_remaining_outputs();
    return;
  }

  const PointCloud &points = *points_geometry.get_pointcloud_for_read();

  const bke::PointCloudFieldContext context(points);
  fn::FieldEvaluator evaluator{context, points.totpoint};
  evaluator.add(curves_group_id_field);
  evaluator.add(length_in_curve_field);
  evaluator.evaluate();
  const VArray<int> curves_group_id = evaluator.get_evaluated<int>(0);
  const VArray<float> length_in_curve = evaluator.get_evaluated<float>(1);

  VectorSet<int> curve_index_by_id;
  for (const int index : IndexRange(points.totpoint)) {
    const int curve_id = curves_group_id[index];
    curve_index_by_id.add(curve_id);
  }

  Curves *curves_id = bke::curves_new_nomain_single(12, CURVE_TYPE_POLY);
  bke::CurvesGeometry &curves = curves_id->geometry.wrap();

  curves.resize(points.totpoint, curve_index_by_id.size());
  curves.fill_curve_types(CURVE_TYPE_POLY);

  Array<int> indices_in_curve(points.totpoint);

  MutableSpan<int> curves_offsets = curves.offsets_for_write();
  curves_offsets.fill(0);

  for (const int index : IndexRange(points.totpoint)){
    const int curve_id = curves_group_id[index];
    const int curve_index = curve_index_by_id.index_of(curve_id);
    indices_in_curve[index] = curves_offsets[curve_index];
    curves_offsets[curve_index]++;
  }
  offset_indices::accumulate_counts_to_offsets(curves_offsets);

  const OffsetIndices<int> offsets(curves_offsets);

  for (const int index : curves.curves_range()) {
    curves.cyclic_for_write()[index] = false;
  }

  const Span<float3> points_positions = points.positions();
  MutableSpan<float3> curves_positions = curves.positions_for_write();

  for (const int index : IndexRange(points.totpoint)){
    const int curve_id = curves_group_id[index];
    const int curve_index = curve_index_by_id.index_of(curve_id);
    const int index_in_curve = indices_in_curve[index];
    const IndexRange curve_points = offsets[curve_index];
    const int dst_index = curve_points[index_in_curve];
    curves_positions[dst_index] = points_positions[index];
  }

  params.set_output("Curves", GeometrySet::create_with_curves(curves_id));
}

}  // namespace blender::nodes::node_geo_interpolate_curves_cc

void register_node_type_geo_points_to_curves()
{
  namespace file_ns = blender::nodes::node_geo_points_to_curves_cc;

  static bNodeType ntype;

  geo_node_type_base(
      &ntype, GEO_NODE_POINTS_TO_CURVES, "Points to Curves", NODE_CLASS_GEOMETRY);
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  ntype.declare = file_ns::node_declare;
  nodeRegisterType(&ntype);
}
