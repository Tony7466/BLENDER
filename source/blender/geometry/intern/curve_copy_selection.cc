/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <iostream>

#include "BKE_anonymous_attribute_id.hh"
#include "BKE_attribute.hh"

#include "BLI_index_mask.hh"

#include "BKE_curves.hh"

namespace blender::geometry {

static void split_mask_for_ranges(const OffsetIndices<int> groups,
                                  const IndexMask mask,
                                  MutableSpan<IndexMask> masks)
{
  threading::parallel_for(groups.index_range(), 2048, [&](const IndexRange range) {
    const IndexMask local_mask = mask.slice_content(groups[range]);
    if (local_mask.is_empty()) {
      return;
    }
    for (const int64_t i : range) {
      masks[i] = local_mask.slice_content(groups[i]);
    }
  });
}

static void count_ranges(const Span<IndexMask> masks, MutableSpan<int> ranges)
{
  threading::parallel_for(masks.index_range(), 2048, [&](const IndexRange range) {
    for (const int64_t i : range) {
      masks[i].foreach_range([&](const IndexRange /*range*/) { ranges[i]++; });
    }
  });
}

static void fill_groups_by_indices(const OffsetIndices<int> groups, MutableSpan<int> indices)
{
  threading::parallel_for(groups.index_range(), 2048, [&](const IndexRange range) {
    for (const int64_t i : range) {
      indices.slice(groups[i]).fill(i);
    }
  });
}

static void ranges_to_sizes(const IndexMask &mask, MutableSpan<int> r_offsets)
{
  int offset_i = 0;
  mask.foreach_range([&](const IndexRange range) {
    r_offsets[offset_i] = range.size();
    offset_i++;
  });
}

bke::CurvesGeometry copy_curve_points(
    const bke::CurvesGeometry &src_curves,
    const IndexMask &points_mask,
    const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  const OffsetIndices<int> points_by_curve = src_curves.points_by_curve();

  IndexMaskMemory memory;
  Array<IndexMask> curves_masks(points_by_curve.size());
  split_mask_for_ranges(points_by_curve, points_mask, curves_masks);
  Array<int> curves_subdivision(points_by_curve.size() + 1, 0);
  count_ranges(curves_masks, curves_subdivision);
  const OffsetIndices<int> curves_subdivision_offset =
      offset_indices::accumulate_counts_to_offsets(curves_subdivision);
  const int64_t total_curves = curves_subdivision_offset.total_size();

  bke::CurvesGeometry dst_curves(points_mask.size(), total_curves);

  MutableSpan<int> dst_offsets = dst_curves.offsets_for_write();
  threading::parallel_for(points_by_curve.index_range(), 2048, [&](const IndexRange range) {
    for (const int64_t i : range) {
      ranges_to_sizes(curves_masks[i], dst_offsets.slice(curves_subdivision_offset[i]));
    }
  });
  offset_indices::accumulate_counts_to_offsets(dst_offsets);

  Array<int> curve_indices(total_curves);
  fill_groups_by_indices(curves_subdivision_offset, curve_indices);

  const bke::AttributeAccessor src_attributes = src_curves.attributes();
  bke::MutableAttributeAccessor dst_attributes = dst_curves.attributes_for_write();

  bke::gather_attributes(
      src_attributes, bke::AttrDomain::Point, propagation_info, {}, points_mask, dst_attributes);
  bke::gather_attributes(src_attributes,
                         bke::AttrDomain::Curve,
                         propagation_info,
                         {},
                         curve_indices.as_span(),
                         dst_attributes);

  dst_curves.update_curve_types();
  return dst_curves;
}

}  // namespace blender::geometry
