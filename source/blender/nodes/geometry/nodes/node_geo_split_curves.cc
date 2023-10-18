/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_curves.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_curve_split_curves_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Curves");
  b.add_input<decl::Bool>("Selection").hide_value().supports_field();
  b.add_input<decl::Bool>("Remove Next")).default_value(true).supports_field();

  b.add_output<decl::Geometry>("Curves");
}

static bke::CurvesGeometry split_curves(const bke::CurvesGeometry &src_curves,
                                        const fn::FieldContext &field_context,
                                        const Field<bool> &selection_field,
                                        const Field<bool> &remove_next_field,
                                        const AnonymousAttributePropagationInfo &propagation_info)
{
  fn::FieldEvaluator evaluator(field_context, src_curves.points_num());
  evaluator.add(selection_field);
  evaluator.add(remove_next_field);
  evaluator.evaluate();
  const VArray<bool> selection = evaluator.get_evaluated<bool>(0);
  if (const std::optional<bool> single = selection.get_if_single()) {
    if (!*single) {
      return src_curves;
    }
  }
  const VArray<bool> remove_next = evaluator.get_evaluated<bool>(1);
  if (const std::optional<bool> single = selection.get_if_single()) {
    if (!*single) {
      // TODO: Special case to just change curve domain.
      return src_curves;
    }
  }

  const OffsetIndices src_points_by_curve = src_curves.points_by_curve();

  Vector<int> old_offsets;
  Vector<int> new_offsets;
  Vector<int> old_curve;

  int new_point = 0;
  const auto start_new_curve = [&](const bool remove_segment) {

  };

  for (const int curve : src_curves.curves_range()) {
    const IndexRange points = src_points_by_curve[curve];
    old_curve.append(curve);
    new_offsets.append(new_point);
    for (const int point : points.drop_front(1)) {
    }
  }

  new_offsets.append(new_point);

  for (const int curve : src_curves.curves_range()) {
    const IndexRange points = src_points_by_curve[curve];
    for (const int point : points) {
      /* start point of each existing curve. Always a new curve*/
      if (point == 0) {
        old_curve.append(curve);
        old_offsets.append(point++);
        new_offsets.append(new_point++);
        continue;
      }

      const bool select_i = selection[point];
      const bool select_i_minus_1 = selection[point - 1];
      const bool next_i_minus_1 = next[point - 1];

      /* previous was selected but there is a segment between current and previous, so create a new
       * curve starting on the last point of the previous curve*/
      if (select_i_minus_1 && !next_i_minus_1 && !prev_i) {
        old_curve.append(curve);
        old_offsets.append(point - 1);
        new_offsets.append(new_point++);

        old_curve.append(curve);
        old_offsets.append(point++);
        new_offsets.append(new_point++);
        continue;
      }

      /* If previous point was selected with next removed or current point was selected with
       * previous removed.*/
      if (select_i_minus_1 && next_i_minus_1 || select_i && prev_i) {
        old_curve.append(curve);
        old_offsets.append(point++);
        new_offsets.append(new_point++);
        continue;
      }

      /* No new curve, just move along the list. */
      point++;
      new_point++;
    }
  }
  /* If the last point of the last curve was selected but kept its segments, add one more point. */
  const int last_point = curves.point_size - 1;
  if (selection[last_point] && !previous[last_point] && !next[last_point - 1]) {
    old_curve.append(curves.curve_size - 1);
    old_offsets.append(point - 1);
    new_offsets.append(new_point++);
  }

  /* Add an additional entry to the offsets for total or virtual "next item" */
  old_offsets.append(point);
  new_offsets.append(new_point);

  bke::CurvesGeometry dst_curves(point, new_offsets.size() - 1);
  dst_curves.offsets_for_write().copy_from(new_offsets);

  const AttributeAccessor src_attributes = src_curves.attributes();
  MutableAttributeAccessor dst_attributes = dst_curves.attributes_for_write();

  bke::gather_attributes(
      src_attributes, ATTR_DOMAIN_CURVE, propagation_info, {}, old_curve, dst_attributes);

  src_attributes.for_all([&](const AttributeIDRef &id, const AttributeMetaData meta_data) {
    if (meta_data.domain != ATTR_DOMAIN_POINT) {
      return true;
    }
    if (id.is_anonymous() && !propagation_info.propagate(id.anonymous_id())) {
      return true;
    }
    const GVArray src = *src_attributes.lookup(id);
    bke::GSpanAttributeWriter dst = dst_attributes.lookup_or_add_for_write_only_span(
        id, ATTR_DOMAIN_POINT, meta_data.data_type);
    bke::attribute_math::convert_to_static_type(dst.span.type(), [&](auto dummy) {
      using T = decltype(dummy);
      for (const int i : new_offsets.index_range().drop_back(1)) {
        const int copy_size = new_offsets[i + 1] - new_offsets[i];
        dst.slice(new_offsets[i], copy_size).copy_from(src.slice(old_offsets[i], copy_size));
      }
    });
    dst.finish();
    return true;
  });

  return dst_curves;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Curves");
  Field<bool> selection = params.extract_input<Field<bool>>("Selection");
  Field<bool> remove_next = params.extract_input<Field<bool>>("Remove Next");
  const AnonymousAttributePropagationInfo propagation_info = params.get_output_propagation_info(
      "Curves");

  geometry_set.modify_geometry_sets([&](GeometrySet &geometry_set) {
    if (const Curves *src_curves_id = geometry_set.get_curves()) {
      const bke::CurvesGeometry &src_curves = src_curves_id->geometry.wrap();
      const bke::CurvesFieldContext field_context(src_curves, ATTR_DOMAIN_POINT);

      bke::CurvesGeometry curves = split_curves(
          src_curves, field_context, selection, remove_next, propagation_info);
      Curves *curves_id = bke::curves_new_nomain(std::move(curves));
      bke::curves_copy_parameters(*src_curves_id, *curves_id);

      geometry_set.replace_curves(curves_id);
    }
  });

  params.set_output("Curves", geometry_set);
}

}  // namespace blender::nodes::node_geo_curve_split_curves_cc

void register_node_type_geo_curve_split_curves()
{
  namespace file_ns = blender::nodes::node_geo_curve_split_curves_cc;
  static bNodeType ntype;
  geo_node_type_base(&ntype, GEO_NODE_SPLIT_CURVES, "Split Curves", NODE_CLASS_GEOMETRY);
  ntype.declare = file_ns::node_declare;
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  nodeRegisterType(&ntype);
}