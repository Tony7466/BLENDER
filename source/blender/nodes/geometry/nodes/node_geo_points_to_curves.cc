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

  b.add_input<decl::Int>("Type").min(0).max(4);

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
  const VectorSet<int> deduplicated_groups(groups);
  threading::parallel_for(groups.index_range(), 2048, [&](const IndexRange range) {
    for (int &value : groups.slice(range)) {
      value = deduplicated_groups.index_of(value);
    }
  });
  return deduplicated_groups.size();
}




static Array<int> reverse_indices_in_small_groups(const OffsetIndices<int> offsets, const Span<int> group_indices)
{
  SCOPED_TIMER_AVERAGED(__func__);

  Array<int> counts(offsets.size(), -1);
  Array<int> results(group_indices.size());
  threading::parallel_for(group_indices.index_range(), 1024, [&](const IndexRange range) {
    for (const int64_t i : range) {
      const int group_index = group_indices[i];
      const int index_in_group = atomic_add_and_fetch_int32(&counts[group_index], 1);
      results[offsets[group_index][index_in_group]] = int(i);
    }
  });

  threading::parallel_for(offsets.index_range(), 256, [&](const IndexRange range) {
    for (const int64_t index : range) {
      MutableSpan<int> group = results.as_mutable_span().slice(offsets[index]);
      std::sort(group.begin(), group.end());
    }
  });
  return results;
}

static Array<int> gather_reverse(const Span<int> group_indices)
{
  SCOPED_TIMER_AVERAGED(__func__);

  Array<int> results(group_indices.size());
  std::iota(results.begin(), results.end(), 0);
  const auto comparator = [&](const int a, const int b) { return group_indices[a] < group_indices[b]; };
  parallel_sort(results.begin(), results.end(), comparator);
  return results;
}

static Array<int> reverse_indices_in_groups(const OffsetIndices<int> offsets, MutableSpan<int> group_indices)
{
  SCOPED_TIMER_AVERAGED(__func__);

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

static Array<int> reverse_indices_in_groups_simple_sort(const OffsetIndices<int> offsets, const Span<int> group_indices)
{
  SCOPED_TIMER_AVERAGED(__func__);

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

static Array<int> reverse_indices_in_groups_complex_sort(const OffsetIndices<int> offsets, const Span<int> group_indices)
{
  SCOPED_TIMER_AVERAGED(__func__);

  Array<int> indices(group_indices.size());
  std::iota(indices.begin(), indices.end(), 0);

  Array<int> results(group_indices.size());

  Array<int> counts(offsets.size(), 0);
  threading::parallel_for(group_indices.index_range(), 1024, [&](const IndexRange range) {
    MutableSpan<int> r_indices = indices.as_mutable_span().slice(range);
    parallel_sort(r_indices.begin(), r_indices.end(), [&](const int a, const int b) {
      return group_indices[a] < group_indices[b];
    });

    int start = 0;
    while (true) {
      const Span<int> local = r_indices.drop_front(start);
      if (local.is_empty()) {
        break;
      }
      const int group = group_indices[local.first()];
      const int *first_other = std::find_if(local.begin(), local.end(), [&](const int index) {
        return group_indices[index] != group;
      });
      const int current_size = int(std::distance(local.begin(), first_other));
      start += current_size;
      const int current_start = atomic_add_and_fetch_int32(&counts[group], current_size) -
                                current_size;
      const IndexRange finall = offsets[group].slice(current_start, current_size);
      MutableSpan<int> dst = results.as_mutable_span().slice(finall);
      const Span<int> src = local.take_front(current_size);
      std::copy(src.begin(), src.end(), dst.begin());
    }
    return;
  });

  return results;
}



static Curves *curves_from_points(const PointCloud *points,
                                  const Field<int> &group_id_field,
                                  const Field<float> &weight_field,
                                  const Map<AttributeIDRef, AttributeKind> &attributes, const int type)
{
  // SCOPED_TIMER_AVERAGED(__func__);
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
    // SCOPED_TIMER_AVERAGED("groups_to_indices");
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
    // SCOPED_TIMER_AVERAGED("build_reverse_offsets");
    offset_indices::build_reverse_offsets(indices_of_curves, accumulated_curves);
  }
  const OffsetIndices<int> curves_offsets(accumulated_curves);

  Array<int> src_indices_of_curve_points(group_ids.size());

std::cout << std::endl;

  switch (type) {
    case 0:
      src_indices_of_curve_points = reverse_indices_in_small_groups(curves_offsets, group_ids);
      break;
    case 1:
      src_indices_of_curve_points = gather_reverse(group_ids);
      break;
    case 2:
      src_indices_of_curve_points = reverse_indices_in_groups(curves_offsets, group_ids);
      break;
    case 3:
      src_indices_of_curve_points = reverse_indices_in_groups_simple_sort(curves_offsets, group_ids);
      break;
    case 4:
      src_indices_of_curve_points = reverse_indices_in_groups_complex_sort(curves_offsets, group_ids);
      break;
    default:
      std::cout << "WTF?" << std::endl;
  }

  {
    // SCOPED_TIMER_AVERAGED("grouped_sort");
    grouped_sort(curves_offsets, weights, src_indices_of_curve_points);
  }

  {
    // SCOPED_TIMER_AVERAGED("attributes");

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
  const int type = params.extract_input<int>("Type");

  Map<AttributeIDRef, AttributeKind> attributes;
  geometry_set.gather_attributes_for_propagation({GeometryComponent::Type::PointCloud},
                                                 GeometryComponent::Type::Curve,
                                                 false,
                                                 params.get_output_propagation_info("Curves"),
                                                 attributes);

  geometry_set.modify_geometry_sets([&](GeometrySet &geometry_set) {
    const PointCloud *points = geometry_set.get_pointcloud();
    Curves *curves_id = curves_from_points(points, group_id_field, weight_field, attributes, type);
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
