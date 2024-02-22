/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_array_utils.hh"
#include "BLI_kdtree.h"
#include "BLI_offset_indices.hh"
#include "BLI_task.hh"

#include "DNA_pointcloud_types.h"

#include "BKE_attribute_math.hh"
#include "BKE_curves.hh"
#include "BKE_curves_utils.hh"
#include "BKE_pointcloud.hh"

#include "GEO_point_merge_by_distance.hh"
#include "GEO_randomize.hh"

namespace blender::geometry {

PointCloud *point_merge_by_distance(const PointCloud &src_points,
                                    const float merge_distance,
                                    const IndexMask &selection,
                                    const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  const bke::AttributeAccessor src_attributes = src_points.attributes();
  const Span<float3> positions = src_points.positions();
  const int src_size = positions.size();

  /* Create the KD tree based on only the selected points, to speed up merge detection and
   * balancing. */
  KDTree_3d *tree = BLI_kdtree_3d_new(selection.size());
  selection.foreach_index_optimized<int64_t>(
      [&](const int64_t i, const int64_t pos) { BLI_kdtree_3d_insert(tree, pos, positions[i]); });
  BLI_kdtree_3d_balance(tree);

  /* Find the duplicates in the KD tree. Because the tree only contains the selected points, the
   * resulting indices are indices into the selection, rather than indices of the source point
   * cloud. */
  Array<int> selection_merge_indices(selection.size(), -1);
  const int duplicate_count = BLI_kdtree_3d_calc_duplicates_fast(
      tree, merge_distance, false, selection_merge_indices.data());
  BLI_kdtree_3d_free(tree);

  /* Create the new point cloud and add it to a temporary component for the attribute API. */
  const int dst_size = src_size - duplicate_count;
  PointCloud *dst_pointcloud = BKE_pointcloud_new_nomain(dst_size);
  bke::MutableAttributeAccessor dst_attributes = dst_pointcloud->attributes_for_write();

  /* By default, every point is just "merged" with itself. Then fill in the results of the merge
   * finding, converting from indices into the selection to indices into the full input point
   * cloud. */
  Array<int> merge_indices(src_size);
  array_utils::fill_index_range<int>(merge_indices);

  selection.foreach_index([&](const int src_index, const int pos) {
    const int merge_index = selection_merge_indices[pos];
    if (merge_index != -1) {
      const int src_merge_index = selection[merge_index];
      merge_indices[src_index] = src_merge_index;
    }
  });

  /* For every source index, find the corresponding index in the result by iterating through the
   * source indices and counting how many merges happened before that point. */
  int merged_points = 0;
  Array<int> src_to_dst_indices(src_size);
  for (const int i : IndexRange(src_size)) {
    src_to_dst_indices[i] = i - merged_points;
    if (merge_indices[i] != i) {
      merged_points++;
    }
  }

  /* In order to use a contiguous array as the storage for every destination point's source
   * indices, first the number of source points must be counted for every result point. */
  Array<int> point_merge_counts(dst_size, 0);
  for (const int i : IndexRange(src_size)) {
    const int merge_index = merge_indices[i];
    const int dst_index = src_to_dst_indices[merge_index];
    point_merge_counts[dst_index]++;
  }

  /* This array stores an offset into `merge_map` for every result point. */
  Array<int> map_offsets_data(dst_size + 1);
  int offset = 0;
  for (const int i : IndexRange(dst_size)) {
    map_offsets_data[i] = offset;
    offset += point_merge_counts[i];
  }
  map_offsets_data.last() = offset;
  OffsetIndices<int> map_offsets(map_offsets_data);

  point_merge_counts.fill(0);

  /* This array stores all of the source indices for every result point. The size is the source
   * size because every input point is either merged with another or copied directly. */
  Array<int> merge_map_indices(src_size);
  for (const int i : IndexRange(src_size)) {
    const int merge_index = merge_indices[i];
    const int dst_index = src_to_dst_indices[merge_index];

    merge_map_indices[map_offsets[dst_index].first() + point_merge_counts[dst_index]] = i;
    point_merge_counts[dst_index]++;
  }

  Set<bke::AttributeIDRef> attribute_ids = src_attributes.all_ids();

  /* Transfer the ID attribute if it exists, using the ID of the first merged point. */
  if (attribute_ids.contains("id")) {
    VArraySpan<int> src = *src_attributes.lookup_or_default<int>("id", bke::AttrDomain::Point, 0);
    bke::SpanAttributeWriter<int> dst = dst_attributes.lookup_or_add_for_write_only_span<int>(
        "id", bke::AttrDomain::Point);

    threading::parallel_for(IndexRange(dst_size), 1024, [&](IndexRange range) {
      for (const int i_dst : range) {
        dst.span[i_dst] = src[map_offsets[i_dst].first()];
      }
    });

    dst.finish();
    attribute_ids.remove_contained("id");
  }

  /* Transfer all other attributes. */
  for (const bke::AttributeIDRef &id : attribute_ids) {
    if (id.is_anonymous() && !propagation_info.propagate(id.anonymous_id())) {
      continue;
    }

    bke::GAttributeReader src_attribute = src_attributes.lookup(id);
    bke::attribute_math::convert_to_static_type(src_attribute.varray.type(), [&](auto dummy) {
      using T = decltype(dummy);
      if constexpr (!std::is_void_v<bke::attribute_math::DefaultMixer<T>>) {
        bke::SpanAttributeWriter<T> dst_attribute =
            dst_attributes.lookup_or_add_for_write_only_span<T>(id, bke::AttrDomain::Point);
        VArraySpan<T> src = src_attribute.varray.typed<T>();

        threading::parallel_for(IndexRange(dst_size), 1024, [&](IndexRange range) {
          for (const int i_dst : range) {
            /* Create a separate mixer for every point to avoid allocating temporary buffers
             * in the mixer the size of the result point cloud and to improve memory locality. */
            bke::attribute_math::DefaultMixer<T> mixer{dst_attribute.span.slice(i_dst, 1)};

            Span<int> src_merge_indices = merge_map_indices.as_span().slice(map_offsets[i_dst]);
            for (const int i_src : src_merge_indices) {
              mixer.mix_in(0, src[i_src]);
            }

            mixer.finalize();
          }
        });

        dst_attribute.finish();
      }
    });
  }

  debug_randomize_point_order(dst_pointcloud);

  return dst_pointcloud;
}

static int curve_merge_by_distance(const Span<float> lengths,
                                   const IndexMask &selection,
                                   const float merge_distance,
                                   MutableSpan<int> r_merge_indices)
{
  /* We use a KDTree_1d here, because we can only merge neighboring points in the curves. */
  KDTree_1d *tree = BLI_kdtree_1d_new(selection.size());
  /* The selection is an IndexMask of the points just in this curve. */
  selection.foreach_index_optimized<int64_t>(
      [&](const int64_t i, const int64_t pos) { BLI_kdtree_1d_insert(tree, pos, &lengths[i]); });
  BLI_kdtree_1d_balance(tree);

  Array<int> selection_merge_indices(selection.size(), -1);
  const int duplicate_count = BLI_kdtree_1d_calc_duplicates_fast(
      tree, merge_distance, false, selection_merge_indices.data());
  BLI_kdtree_1d_free(tree);

  array_utils::fill_index_range<int>(r_merge_indices);

  selection.foreach_index([&](const int src_index, const int pos) {
    const int merge_index = selection_merge_indices[pos];
    if (merge_index != -1) {
      const int src_merge_index = selection[merge_index];
      r_merge_indices[src_index] = src_merge_index;
    }
  });

  return duplicate_count;
}

bke::CurvesGeometry curves_merge_by_distance(
    const bke::CurvesGeometry &src_curves,
    const float merge_distance,
    const IndexMask &selection,
    const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  const int src_point_size = src_curves.points_num();
  OffsetIndices<int> points_by_curve = src_curves.points_by_curve();
  VArray<bool> cyclic = src_curves.cyclic();

  std::atomic<int> total_duplicate_count = 0;
  Array<int> dst_curve_counts(src_curves.curves_num());
  Array<Array<int>> merge_indices_per_curve(src_curves.curves_num());
  threading::parallel_for(src_curves.curves_range(), 512, [&](const IndexRange range) {
    for (const int curve_i : range) {
      const IndexRange points = points_by_curve[curve_i];
      merge_indices_per_curve[curve_i].reinitialize(points.size());

      const Span<float> lengths = src_curves.evaluated_lengths_for_curve(curve_i, cyclic[curve_i]);
      MutableSpan<int> merge_indices = merge_indices_per_curve[curve_i].as_mutable_span();
      const int duplicate_count = curve_merge_by_distance(
          lengths, selection.slice_content(points), merge_distance, merge_indices);

      dst_curve_counts[curve_i] = points.size() - duplicate_count;
      total_duplicate_count += duplicate_count;
    }
  });

  bke::CurvesGeometry dst_curves = bke::curves::copy_only_curve_domain(src_curves);
  const int dst_point_size = src_point_size - total_duplicate_count;
  dst_curves.resize(dst_point_size, src_curves.curves_num());

  array_utils::copy(dst_curve_counts.as_span(), dst_curves.offsets_for_write());
  offset_indices::accumulate_counts_to_offsets(dst_curves.offsets_for_write());

  int merged_points = 0;
  Array<int> src_to_dst_indices(src_point_size);
  for (const int curve_i : src_curves.curves_range()) {
    const IndexRange points = points_by_curve[curve_i];
    const Span<int> merge_indices = merge_indices_per_curve[curve_i].as_span();
    for (const int i : points.index_range()) {
      const int point_i = points.start() + i;
      src_to_dst_indices[point_i] = point_i - merged_points;
      if (merge_indices[i] != i) {
        merged_points++;
      }
    }
  }

  Array<int> point_merge_counts(dst_point_size, 0);
  for (const int curve_i : src_curves.curves_range()) {
    const IndexRange points = points_by_curve[curve_i];
    const Span<int> merge_indices = merge_indices_per_curve[curve_i].as_span();
    for (const int i : points.index_range()) {
      const int merge_index = merge_indices[i];
      const int point_src = points.start() + merge_index;
      const int dst_index = src_to_dst_indices[point_src];
      point_merge_counts[dst_index]++;
    }
  }

  Array<int> map_offsets_data(dst_point_size + 1);
  map_offsets_data.as_mutable_span().drop_back(1).copy_from(point_merge_counts);
  OffsetIndices<int> map_offsets = offset_indices::accumulate_counts_to_offsets(map_offsets_data);

  point_merge_counts.fill(0);

  Array<int> merge_map_indices(src_point_size);
  for (const int curve_i : src_curves.curves_range()) {
    const IndexRange points = points_by_curve[curve_i];
    const Span<int> merge_indices = merge_indices_per_curve[curve_i].as_span();
    for (const int i : points.index_range()) {
      const int point_i = points.start() + i;
      const int merge_index = merge_indices[i];
      const int dst_index = src_to_dst_indices[merge_index];
      merge_map_indices[map_offsets[dst_index].first() + point_merge_counts[dst_index]] = point_i;
      point_merge_counts[dst_index]++;
    }
  }

  Set<bke::AttributeIDRef> attribute_ids = src_curves.attributes().all_ids();

  for (const bke::AttributeIDRef &id : attribute_ids) {
    if (id.is_anonymous() && !propagation_info.propagate(id.anonymous_id())) {
      continue;
    }

    bke::GAttributeReader src_attribute = src_curves.attributes().lookup(id);
    bke::attribute_math::convert_to_static_type(src_attribute.varray.type(), [&](auto dummy) {
      using T = decltype(dummy);
      if constexpr (!std::is_void_v<bke::attribute_math::DefaultMixer<T>>) {
        bke::SpanAttributeWriter<T> dst_attribute =
            dst_attributes.lookup_or_add_for_write_only_span<T>(id, bke::AttrDomain::Point);
        VArraySpan<T> src = src_attribute.varray.typed<T>();

        threading::parallel_for(dst_point_size, 1024, [&](IndexRange range) {
          for (const int dst_point_i : range) {
            /* Create a separate mixer for every point to avoid allocating temporary buffers
             * in the mixer the size of the result point cloud and to improve memory locality. */
            bke::attribute_math::DefaultMixer<T> mixer{dst_attribute.span.slice(dst_point_i, 1)};

            Span<int> src_merge_indices = merge_map_indices.as_span().slice(
                map_offsets[dst_point_i]);
            for (const int src_point_i : src_merge_indices) {
              mixer.mix_in(0, src[src_point_i]);
            }

            mixer.finalize();
          }
        });

        dst_attribute.finish();
      }
    });
  }

  return dst_curves;
}

}  // namespace blender::geometry
