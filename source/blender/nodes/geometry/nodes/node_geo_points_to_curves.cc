/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "BLI_timeit.hh"
#include "BLI_task.hh"
#include "BLI_sort.hh"

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

void inverse_gather(const GVArray &src, const Span<int> &indices, GMutableSpan dst)
{
  bke::attribute_math::convert_to_static_type(dst.type(), [&](auto dummy) {
    using T = decltype(dummy);
    const VArray<T> src_typed = src.typed<T>();
    MutableSpan<T> dst_typed = dst.typed<T>();
    devirtualize_varray(src_typed, [&](const auto src_typed) {
      threading::parallel_for(indices.index_range(), 4096, [&](const IndexRange range) {
        for (const int64_t index : range) {
          dst_typed[indices[index]] = src_typed[index];
        }
      });
    });
  });
}

void inverse_fill(const Span<int> &indices, GMutableSpan dst)
{
  bke::attribute_math::convert_to_static_type(dst.type(), [&](auto dummy) {
    using T = decltype(dummy);
    MutableSpan<T> dst_typed = dst.typed<T>();
    threading::parallel_for(indices.index_range(), 4096, [&](const IndexRange range) {
      for (const int64_t index : range) {
        dst_typed[indices[index]] = T();
      }
    });
  });
}

static void grouped_sort(const OffsetIndices<int> groups, const Span<float> weights, MutableSpan<int> indices)
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

  threading::parallel_for_each(groups.index_range(), [&](const int group_index){
    MutableSpan<int> group = indices.slice(groups[group_index]);
    parallel_sort(group.begin(), group.end(), comparator);
  });
}

static int groups_to_indices(MutableSpan<int> groups)
{
  VectorSet<int> deduplicated_groups(groups);
  threading::parallel_for(groups.index_range(), 2048, [&](const IndexRange range){
    for (int &value : groups.slice(range)) {
      value = deduplicated_groups.index_of(value);
    }
  });
  return deduplicated_groups.size();
}

static Array<int> gather_reverse(const Span<int> indices)
{
  Array<int> results(indices.size());
  std::iota(results.begin(), results.end(), 0);
  const auto comparator = [&](const int a, const int b) { return indices[a] < indices[b]; };
  parallel_sort(results.begin(), results.end(), comparator);
  return results;
}

static Curves *curves_from_points(const PointCloud *points,
                                  const Field<int> &group_id_field,
                                  const Field<float> &weight_field,
                                  const Map<AttributeIDRef, AttributeKind> &attributes)
{
  if (points == nullptr) {
    return nullptr;
  }
  
  std::cout << __func__ << std::endl;

  const int domain_size = points->totpoint;

  Array<int> group_ids(domain_size);
  Array<float> weights(domain_size);

  const bke::PointCloudFieldContext context(*points);
  fn::FieldEvaluator evaluator(context, domain_size);
  evaluator.add_with_destination(group_id_field, group_ids.as_mutable_span());
  evaluator.add_with_destination(weight_field, weights.as_mutable_span());
  {
    SCOPED_TIMER_AVERAGED("evaluator");
    evaluator.evaluate();
  }
  
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

  {
    SCOPED_TIMER_AVERAGED("build_reverse_offsets");
    accumulated_curves.fill(0);
    offset_indices::build_reverse_offsets(indices_of_curves, accumulated_curves);
  }
  const OffsetIndices<int> curves_offsets(accumulated_curves);

  Array<int> src_to_dst_indices;
  {
    SCOPED_TIMER_AVERAGED("gather_reverse");
    src_to_dst_indices = gather_reverse(indices_of_curves);
  }

  GroupedSpan<int> src_unsorted_curve_points(curves_offsets, src_to_dst_indices);

  /*
    for (const int &off : indices_of_curves) {
      std::cout << off << (&off == &indices_of_curves.last() ? "\n" : ", ");
    }

    for (const int &off : accumulated_curves) {
      std::cout << off << (&off == &accumulated_curves.last() ? "\n" : ", ");
    }

    for (const int &off : src_to_dst_indices) {
      std::cout << off << (&off == &src_to_dst_indices.last() ? "\n" : ", ");
    }
  */

  /* Indices from global src to dst in each curve. Not to global dst yet. */
  //Array<int> src_to_dst_indices(domain_size);
  {/*
    SCOPED_TIMER_AVERAGED("src_to_dst_indices");
    for (const int index : IndexRange(domain_size)) {
      const int curve_index = group_ids[index];
      src_to_dst_indices[index] = accumulated_curves[curve_index];
      accumulated_curves[curve_index]++;
    }*/
  }

  /* Global to global. */
  Array<int> dst_to_src_indices(domain_size);
  {
    SCOPED_TIMER_AVERAGED("dst_to_src_indices");
    for (const int index : IndexRange(domain_size)) {
      const int dst_index = src_to_dst_indices[index];
      dst_to_src_indices[dst_index] = index;
    }
  }

  {
    SCOPED_TIMER_AVERAGED("grouped_sort");
    grouped_sort(curves_offsets, weights, dst_to_src_indices);
  }
  

  /* Now indices is from global src to global dst. */
  {
    SCOPED_TIMER_AVERAGED("from global src to global dst");
    for (const int index : IndexRange(domain_size)) {
      const int src_index = dst_to_src_indices[index];
      src_to_dst_indices[src_index] = index;
    }
  }

  const AttributeAccessor src_attributes = points->attributes();
  MutableAttributeAccessor dst_attributes = curves.attributes_for_write();
  {
    SCOPED_TIMER_AVERAGED("attributes");
    for (MapItem<AttributeIDRef, AttributeKind> entry : attributes.items()) {
      const AttributeIDRef attribute_id = entry.key;
      const eCustomDataType output_data_type = entry.value.data_type;

      GSpanAttributeWriter dst = dst_attributes.lookup_or_add_for_write_only_span(
          attribute_id, ATTR_DOMAIN_POINT, output_data_type);
      if (!dst) {
        continue;
      }

      const GAttributeReader src = src_attributes.lookup(attribute_id);
      if (!src || src.domain != ATTR_DOMAIN_POINT) {
        inverse_fill(src_to_dst_indices, dst.span);
      }
      else {
        inverse_gather(src.varray, src_to_dst_indices, dst.span);
      }

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
