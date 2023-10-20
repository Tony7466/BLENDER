/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_curves.hh"

#include "BLI_enumerable_thread_specific.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_curve_split_curves_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Curves");
  b.add_input<decl::Bool>("Selection").hide_value().supports_field();
  b.add_input<decl::Bool>("Remove Next").default_value(true).supports_field();

  b.add_output<decl::Geometry>("Curves");
}

// static bke::CurvesGeometry split_curves(const bke::CurvesGeometry &src_curves,
//                                         const fn::FieldContext &field_context,
//                                         const Field<bool> &selection_field,
//                                         const Field<bool> &remove_segment_field,
//                                         const AnonymousAttributePropagationInfo
//                                         &propagation_info)
// {
//   fn::FieldEvaluator evaluator(field_context, src_curves.points_num());
//   evaluator.set_selection(selection_field);
//   evaluator.add(remove_segment_field);
//   evaluator.evaluate();
//   const IndexMask selection = evaluator.get_evaluated_selection_as_mask();
//   const IndexMask remove_segment = evaluator.get_evaluated_as_mask(0);

//   const OffsetIndices points_by_curve = src_curves.points_by_curve();

//   struct TLS {
//     IndexMaskMemory memory;
//   };

//   threading::EnumerableThreadSpecific<TLS> all_tls;

//   Array<int> curves_by_src_curve(src_curves.curves_num() + 1);

//   threading::parallel_for(points_by_curve.index_range(), 64, [&](const IndexRange curves_range)
//   {
//     TLS &tls = all_tls.local();
//     for (const int curve : points_by_curve.index_range()) {
//       const IndexRange points = points_by_curve[curve];
//       const IndexMask curve_selection = selection.slice_and_offset(
//           points, -points.start(), tls.memory);

//       curves_by_src_curve[curve] = curve_selection.size() + 1;
//     }
//   });
// }

template<typename T>
static void copy_groups_from_offsets(const OffsetIndices<int> dst_groups,
                                     const Span<int> src_offsets,
                                     const Span<T> src,
                                     MutableSpan<T> dst)
{
  threading::parallel_for(dst_groups.index_range(), 512, [&](const IndexRange range) {
    for (const int i : range) {
      const IndexRange dst_group = dst_groups[i];
      dst.slice(dst_group).copy_from(src.slice(src_offsets[i], dst_group.size()));
    }
  });
}

static bke::CurvesGeometry split_curves(const bke::CurvesGeometry &src_curves,
                                        const fn::FieldContext &field_context,
                                        const Field<bool> &selection_field,
                                        const Field<bool> &remove_segment_field,
                                        const AnonymousAttributePropagationInfo &propagation_info)
{
  fn::FieldEvaluator evaluator(field_context, src_curves.points_num());
  evaluator.add(selection_field);
  evaluator.add(remove_segment_field);
  evaluator.evaluate();

  const VArray<bool> selection = evaluator.get_evaluated<bool>(0);
  if (const std::optional<bool> single = selection.get_if_single()) {
    if (!*single) {
      return src_curves;
    }
  }
  const VArray<bool> remove_segments = evaluator.get_evaluated<bool>(1);
  if (const std::optional<bool> single = selection.get_if_single()) {
    if (!*single) {
      // TODO: Special case to just change curve domain.
      return src_curves;
    }
  }

  const OffsetIndices src_points_by_curve = src_curves.points_by_curve();
  const VArray<bool> cyclic = src_curves.cyclic();

  Vector<int> old_offsets;
  Vector<int> new_offsets;
  Vector<int> old_curve;

  int new_point = 0;
  const auto add_split = [&](const int src_curve, const int src_point, const bool remove_segment) {
    old_curve.append(src_curve);
    old_offsets.append(src_point);
    new_offsets.append(new_point);
    new_point++;
    if (remove_segment) {
      old_curve.append(src_curve);
      old_offsets.append(src_point + 1);
      new_offsets.append(new_point);
      new_point++;
    }
  };

  for (const int curve : src_curves.curves_range()) {
    const IndexRange points = src_points_by_curve[curve];

    /* Add the start of the new curve. */
    if (selection[points.first()]) {
      add_split(curve, points.first(), remove_segments[points.first()] && points.size() > 1);
    }

    /* Add the splits in the middle of the curve. */
    for (const int point : points.drop_front(1).drop_back(1)) {
      if (selection[point]) {
        add_split(curve, point, remove_segments[point]);
      }
    }

    if (selection[points.last()]) {
      add_split(curve, points.last(), false);
    }
  }

  new_offsets.append(new_point);

  bke::CurvesGeometry dst_curves(new_point, new_offsets.size() - 1);
  dst_curves.offsets_for_write().copy_from(new_offsets);

  const OffsetIndices points_by_curve = dst_curves.points_by_curve();

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
    const GVArraySpan src = *src_attributes.lookup(id);
    bke::GSpanAttributeWriter dst = dst_attributes.lookup_or_add_for_write_only_span(
        id, ATTR_DOMAIN_POINT, meta_data.data_type);
    bke::attribute_math::convert_to_static_type(dst.span.type(), [&](auto dummy) {
      using T = decltype(dummy);
      copy_groups_from_offsets(points_by_curve, old_offsets, src.typed<T>(), dst.span.typed<T>());
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
  Field<bool> remove_segment = params.extract_input<Field<bool>>("Remove Next");
  const AnonymousAttributePropagationInfo propagation_info = params.get_output_propagation_info(
      "Curves");

  geometry_set.modify_geometry_sets([&](GeometrySet &geometry_set) {
    if (const Curves *src_curves_id = geometry_set.get_curves()) {
      const bke::CurvesGeometry &src_curves = src_curves_id->geometry.wrap();
      const bke::CurvesFieldContext field_context(src_curves, ATTR_DOMAIN_POINT);

      bke::CurvesGeometry curves = split_curves(
          src_curves, field_context, selection, remove_segment, propagation_info);
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