/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "atomic_ops.h"

#include "BKE_attribute_math.hh"

#include "BLI_array_utils.hh"
#include "BLI_math_vector.hh"

#include "BLI_sort.hh"
#include "BLI_task.hh"

#include "BLI_timeit.hh"

#include "BKE_geometry_set.hh"

namespace blender::nodes::node_geo_points_to_curves_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Points")
      .supported_type(GeometryComponent::Type::PointCloud)
      .description("Points to build curves");
  b.add_input<decl::Int>("Curve Group ID")
      .field_on_all()
      .hide_value()
      .description(
          "Index of curve for each point. ID groups will be converted to curves indices without "
          "gaps");
  b.add_input<decl::Float>("Weight").field_on_all().hide_value().description(
      "Weight to sort points of curve");

  b.add_output<decl::Geometry>("Curves").propagate_all();
}

static void grouped_sort(const OffsetIndices<int> groups,
                         const Span<float> weights,
                         MutableSpan<int> indices)
{
  const auto comparator = [&](const int index_a, const int index_b) {
    const float weight_a = weights[index_a];
    const float weight_b = weights[index_b];
    if (UNLIKELY(weight_a == weight_b)) {
      /* Approach to make it stable. */
      return index_a > index_b;
    }
    return weight_a > weight_b;
  };

  threading::parallel_for_each(groups.index_range(), [&](const int group_index) {
    MutableSpan<int> group = indices.slice(groups[group_index]);
    parallel_sort(group.begin(), group.end(), comparator);
  });
}

static int groups_to_indices(MutableSpan<int> groups)
{
  VectorSet<int> deduplicated_groups(groups);
  threading::parallel_for(groups.index_range(), 2048, [&](const IndexRange range) {
    for (int &value : groups.slice(range)) {
      value = deduplicated_groups.index_of(value);
    }
  });
  return deduplicated_groups.size();
}

static Array<int> reverse_indices_in_groups(const OffsetIndices<int> offsets,
                                            MutableSpan<int> group_indices)
{
  BLI_assert(!group_indices.is_empty());
  BLI_assert(*std::max_element(group_indices.begin(), group_indices.end()) < offsets.size());
  BLI_assert(*std::min_element(group_indices.begin(), group_indices.end()) >= 0);
  Array<int> counts(offsets.size(), -1);
  threading::parallel_for(group_indices.index_range(), 1024, [&](const IndexRange range) {
    for (int &group_index : group_indices.slice(range)) {
      const int index_in_group = atomic_add_and_fetch_int32(&counts[group_index], 1);
      group_index = offsets[group_index][index_in_group];
    }
  });

  Array<int> results(group_indices.size());
  threading::parallel_for(results.index_range(), 2048, [&](const IndexRange range) {
    for (const int index : range) {
      results[group_indices[index]] = index;
    }
  });
  return results;
}

static Array<int> reverse_indices_in_groups_complex(const OffsetIndices<int> offsets,
                                                    Span<int> group_indices)
{
  Array<int> indices(group_indices.size());
  std::iota(indices.begin(), indices.end(), 0);

  Array<int> results(group_indices.size());

  Array<int> counts(offsets.size(), -1);
  threading::parallel_for(group_indices.index_range(), 1024, [&](const IndexRange range) {
    MutableSpan<int> r_indices = indices.as_mutable_span().slice(range);
    parallel_sort(r_indices.begin(), r_indices.end(), [&](const int a, const int b) {
      return group_indices[a] < group_indices[b];
    });
    Array<int> thread_counts(offsets.size(), 0);
    for (const int i : r_indices) {
      thread_counts[group_indices[i]]++;
    }
    for (const int i : thread_counts.index_range()) {
      const int size = thread_counts[i];
      thread_counts[i] = atomic_add_and_fetch_int32(&counts[i], size) - size;
    }
    for (const int i : r_indices) {
      const int group_index = group_indices[i];
      const int index = thread_counts[group_index];
      thread_counts[group_index]++;
      results[offsets[group_index][index]] = i;
    }
  });

  return results;
}

static Curves *curves_from_points(const PointCloud *points,
                                  const Field<int> &group_id_field,
                                  const Field<float> &weight_field,
                                  const Map<AttributeIDRef, AttributeKind> &attributes)
{
  SCOPED_TIMER_AVERAGED(__func__);
  if (points == nullptr) {
    return nullptr;
  }
  const int domain_size = points->totpoint;

  Array<int> group_ids(domain_size);
  Array<float> weights(domain_size);

  const bke::PointCloudFieldContext context(*points);
  fn::FieldEvaluator evaluator(context, domain_size);
  evaluator.add_with_destination(group_id_field, group_ids.as_mutable_span());
  evaluator.add_with_destination(weight_field, weights.as_mutable_span());
  evaluator.evaluate();

  int total_curves;
  {
    SCOPED_TIMER_AVERAGED("groups_to_indices");
    total_curves = groups_to_indices(group_ids);
  }
  const Span<int> indices_of_curves(group_ids);

  Curves *curves_id = bke::curves_new_nomain(domain_size, total_curves);
  bke::CurvesGeometry &curves = curves_id->geometry.wrap();
  curves.fill_curve_types(CURVE_TYPE_POLY);
  curves.cyclic_for_write().fill(false);
  MutableSpan<int> accumulated_curves = curves.offsets_for_write();
  accumulated_curves.fill(0);
  {
    SCOPED_TIMER_AVERAGED("build_reverse_offsets");
    offset_indices::build_reverse_offsets(indices_of_curves, accumulated_curves);
  }
  const OffsetIndices<int> curves_offsets(accumulated_curves);

  Array<int> src_indices_of_curve_points(group_ids.size());
  {
    SCOPED_TIMER_AVERAGED("reverse_indices_in_groups_complex");
    src_indices_of_curve_points = reverse_indices_in_groups_complex(curves_offsets, group_ids);
  }
  {
    SCOPED_TIMER_AVERAGED("grouped_sort");
    grouped_sort(curves_offsets, weights, src_indices_of_curve_points);
  }

  {
    SCOPED_TIMER_AVERAGED("attributes");

    const AttributeAccessor src_attributes = points->attributes();
    MutableAttributeAccessor dst_attributes = curves.attributes_for_write();
    for (MapItem<AttributeIDRef, AttributeKind> entry : attributes.items()) {
      const AttributeIDRef attribute_id = entry.key;
      const eCustomDataType output_data_type = entry.value.data_type;

      GSpanAttributeWriter dst = dst_attributes.lookup_or_add_for_write_only_span(
          attribute_id, ATTR_DOMAIN_POINT, output_data_type);
      if (!dst) {
        continue;
      }

      const GAttributeReader src = src_attributes.lookup(
          attribute_id, ATTR_DOMAIN_POINT, output_data_type);
      if (!src) {
        continue;
      }

      bke::attribute_math::convert_to_static_type(output_data_type, [&](auto dummy) {
        using T = decltype(dummy);
        array_utils::gather<T, int>(
            src.varray.typed<T>(), src_indices_of_curve_points, dst.span.typed<T>());
      });

      dst.finish();
    }
  }

  return curves_id;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Points");
  const Field<int> group_id_field = params.extract_input<Field<int>>("Curve Group ID");
  const Field<float> weight_field = params.extract_input<Field<float>>("Weight");

  Map<AttributeIDRef, AttributeKind> attributes;
  geometry_set.gather_attributes_for_propagation({GeometryComponent::Type::PointCloud},
                                                 GeometryComponent::Type::Curve,
                                                 false,
                                                 params.get_output_propagation_info("Curves"),
                                                 attributes);

  geometry_set.modify_geometry_sets([&](GeometrySet &geometry_set) {
    const PointCloud *points = geometry_set.get_pointcloud_for_read();
    Curves *curves_id = curves_from_points(points, group_id_field, weight_field, attributes);
    geometry_set.replace_curves(curves_id);
    geometry_set.keep_only_during_modify({GeometryComponent::Type::Curve});
  });

  params.set_output("Curves", std::move(geometry_set));
}

}  // namespace blender::nodes::node_geo_points_to_curves_cc

void register_node_type_geo_points_to_curves()
{
  namespace file_ns = blender::nodes::node_geo_points_to_curves_cc;

  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_POINTS_TO_CURVES, "Points to Curves", NODE_CLASS_GEOMETRY);
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  ntype.declare = file_ns::node_declare;
  nodeRegisterType(&ntype);
}
