/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#ifdef WITH_POTRACE

#include <iostream>

#include "potracelib.h"

#include "BKE_attribute.hh"
#include "BKE_curves.hh"

#include "BLI_array_utils.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_map.hh"
#include "BLI_task.hh"
#include "BLI_offset_indices.hh"

namespace blender::geometry {

using Segment = potrace_word;

static constexpr const int segment_size = sizeof(Segment) * 8;

static Segment from_bools(const Span<bool> src)
{
  BLI_assert(src.size() <= segment_size);
  Segment result = 0;
  for (const int64_t i : src.index_range()) {
    const Segment bit(src[i]);
    BLI_assert(ELEM(bit, 0, 1));
    result |= bit << (31 - i);
  }
  return result;
}

static void gather_list(const potrace_path_t *element, Vector<const potrace_path_t *> &r_list)
{
  for (const potrace_path_t *iter = element; iter != nullptr; iter = iter->next) {
    r_list.append(iter);
  }
}

static potrace_state_t *bools_to_potrace_curves(const int2 resolution, const Span<bool> grid_color)
{
  const int full_segments_num = resolution.x / segment_size;
  const int extra_bits_num = resolution.x % segment_size;
  const int extra_segment = extra_bits_num == 0 ? 0 : 1;
  const int segments_num = full_segments_num + extra_segment;

  Array<Segment> segments(segments_num * resolution.y);
  threading::parallel_for(IndexRange(resolution.y), 4096, [&](const IndexRange range) {
    for (const int y_index : range) {
      const Span<bool> src_line = grid_color.slice(y_index * resolution.x, resolution.x);
      MutableSpan<Segment> line = segments.as_mutable_span().slice(y_index * segments_num, segments_num);
      for (const int i : IndexRange(full_segments_num)) {
        line[i] = from_bools(src_line.slice(i * segment_size, segment_size));
      }
      if (extra_bits_num > 0) {
        line.last() = from_bools(src_line.slice(full_segments_num * segment_size, extra_bits_num));
      }
    }
  }, threading::accumulated_task_sizes([&](const IndexRange range) { return range.size() * resolution.x; }));

  potrace_bitmap_t bitmap;
  bitmap.w = resolution.x;
  bitmap.h = resolution.y;
  bitmap.dy = segments_num;
  bitmap.map = segments.data();
  potrace_param_t *params = potrace_param_default();
  BLI_SCOPED_DEFER([&](){ potrace_param_free(params); });

  BLI_assert(params != nullptr);
  return potrace_trace(params, &bitmap);
}

static float2 to_float2(const potrace_dpoint_s &point)
{
  return float2(point.x, point.y);
}

static void potrace_curves_captuer_parent_index(const Span<const potrace_path_t *> src_curves, Array<int> &r_parent_index)
{
  r_parent_index.reinitialize(src_curves.size());
  
  Map<const potrace_path_t *, int> child_parent_index;
  
  for (const int parent_i : src_curves.index_range()) {
    for (potrace_path_t *iter = src_curves[parent_i]->childlist; iter != nullptr; iter = iter->sibling) {
      child_parent_index.add(iter, parent_i);
    }
  }

  threading::parallel_for(src_curves.index_range(), 4096, [&](const IndexRange range) {
    const Span<const potrace_path_t *> src_slice = src_curves.slice(range);
    MutableSpan<int> dst_slice = r_parent_index.as_mutable_span().slice(range);
    for (const int64_t i : range) {
      dst_slice[i] = child_parent_index.lookup_default(src_slice[i], int(i));
    }
  });
}

static void potrace_curves_to_vector_curve(const Span<const potrace_path_t *> src_curves, Array<std::array<float2, 3>> &r_vector_data, Array<int> &r_offsets, Array<int8_t> &r_types)
{
  BLI_assert(!src_curves.is_empty());
  r_offsets.reinitialize(src_curves.size() + 1);
  threading::parallel_for(src_curves.index_range(), 4096, [&](const IndexRange range) {
    for (const int i : range) {
      r_offsets[i] = src_curves[i]->curve.n;
    }
  });
  const OffsetIndices<int> offsets = offset_indices::accumulate_counts_to_offsets(r_offsets);

  r_vector_data.reinitialize(offsets.total_size());
  r_types.reinitialize(offsets.total_size());
  threading::parallel_for(src_curves.index_range(), 4096, [&](const IndexRange range) {
    for (const int curve_i : range) {
      const IndexRange curve_points = offsets[curve_i];
      MutableSpan<std::array<float2, 3>> points = r_vector_data.as_mutable_span().slice(curve_points);
      for (const int64_t i : curve_points.index_range()) {
        points[i][0] = to_float2(src_curves[curve_i]->curve.c[i][0]);
        points[i][1] = to_float2(src_curves[curve_i]->curve.c[i][1]);
        points[i][2] = to_float2(src_curves[curve_i]->curve.c[i][2]);
      }
      const Span<int> src_point_types(src_curves[curve_i]->curve.tag, src_curves[curve_i]->curve.n);
      MutableSpan<int8_t> dst_point_types = r_types.as_mutable_span().slice(curve_points);
      std::transform(src_point_types.begin(), src_point_types.end(), dst_point_types.begin(), [&](const int type) { return int8_t(type); });
    }
  }, threading::accumulated_task_sizes([&](const IndexRange range) { return offsets[range].size(); }));
}

static void vector_curves_to_bezier_curves(const OffsetIndices<int> offsets, const Span<int8_t> types, MutableSpan<std::array<float2, 3>> vector_data)
{
  threading::parallel_for(offsets.index_range(), 4096, [&](const IndexRange range) {
    IndexMaskMemory memory;
    for (const int curve_i : range) {
      const IndexRange curve_points = offsets[curve_i];
      const Span<int8_t> point_types = types.slice(curve_points);
      MutableSpan<std::array<float2, 3>> points = vector_data.slice(curve_points);

      const IndexMask bezier_points = IndexMask::from_predicate(curve_points.index_range(), GrainSize(4096), memory, [&](const int64_t i) {
        BLI_assert(ELEM(point_types[i], POTRACE_CURVETO, POTRACE_CORNER));
        return point_types[i] == POTRACE_CURVETO;
      });
      const IndexMask poly_line_points = bezier_points.complement(curve_points.index_range(), memory);
      /* Initialize unused polyline point by other point the treat this as handle. */
      poly_line_points.foreach_index_optimized<int64_t>([&](const int64_t i){
        points[i][0] = points[i][1];
      });

      /* Make it so each triplet will contain left right and control point positions. */
      const float2 first_handle = points.first()[0];
      for (const int i : curve_points.index_range().drop_back(1)) {
        points[i][0] = points[i + 1][0];
      }
      points.last()[0] = first_handle;
    }
  }, threading::accumulated_task_sizes([&](const IndexRange range) { return offsets[range].size(); }));
}

Curves *plane_to_curve(const int2 resolution, const Span<bool> grid_color, const float2 min_point, const float2 max_point, const bke::AttributeIDRef &parent_index_id)
{
  Array<std::array<float2, 3>> vector_data;
  Array<int8_t> types;
  Array<int> offsets_data;
  Array<int> parent_index_data;

  potrace_state_t *potrace_result = bools_to_potrace_curves(resolution, grid_color);
  if (potrace_result == nullptr) {
    potrace_state_free(potrace_result);
    return nullptr;
  }

  Vector<const potrace_path_t *> curves_list;
  gather_list(potrace_result->plist, curves_list);
  if (curves_list.is_empty()) {
    potrace_state_free(potrace_result);
    return nullptr;
  }

  potrace_curves_to_vector_curve(curves_list, vector_data, offsets_data, types);
  if (parent_index_id) {
    potrace_curves_captuer_parent_index(curves_list, parent_index_data);
  }
  potrace_state_free(potrace_result);

  const OffsetIndices<int> offsets(offsets_data);
  vector_curves_to_bezier_curves(offsets, types, vector_data);

  Curves *curves_id = bke::curves_new_nomain(offsets.total_size(), offsets.size());
  bke::CurvesGeometry &curves = curves_id->geometry.wrap();
  curves.fill_curve_types(CURVE_TYPE_BEZIER);
  curves.handle_types_left_for_write().fill(BEZIER_HANDLE_FREE);
  curves.handle_types_right_for_write().fill(BEZIER_HANDLE_FREE);
  curves.cyclic_for_write().fill(true);
  array_utils::copy(offsets_data.as_span(), curves.offsets_for_write());

  MutableSpan<float3> positions = curves.positions_for_write();
  MutableSpan<float3> handle_positions_left = curves.handle_positions_left_for_write();
  MutableSpan<float3> handle_positions_right = curves.handle_positions_right_for_write();

  const float2 resolution_factor = float2(1.0f) / float2(resolution) * (max_point - min_point);

  threading::parallel_for(IndexRange(offsets.total_size()), 4096, [&](const IndexRange range) {
    const Span<std::array<float2, 3>> src_slice = vector_data.as_span().slice(range);
    MutableSpan<float3> dst_slice = positions.slice(range);
    std::transform(src_slice.begin(), src_slice.end(), dst_slice.begin(), [&](const std::array<float2, 3> &src_point) {
      return float3(min_point + src_point[2] * resolution_factor, 0);
    });
  });

  threading::parallel_for(IndexRange(offsets.total_size()), 4096, [&](const IndexRange range) {
    const Span<std::array<float2, 3>> src_slice = vector_data.as_span().slice(range);
    MutableSpan<float3> dst_slice = handle_positions_left.slice(range);
    std::transform(src_slice.begin(), src_slice.end(), dst_slice.begin(), [&](const std::array<float2, 3> &src_point) {
      return float3(min_point + src_point[1] * resolution_factor, 0);
    });
  });

  threading::parallel_for(IndexRange(offsets.total_size()), 4096, [&](const IndexRange range) {
    const Span<std::array<float2, 3>> src_slice = vector_data.as_span().slice(range);
    MutableSpan<float3> dst_slice = handle_positions_right.slice(range);
    std::transform(src_slice.begin(), src_slice.end(), dst_slice.begin(), [&](const std::array<float2, 3> &src_point) {
      return float3(min_point + src_point[0] * resolution_factor, 0);
    });
  });

  if (!parent_index_data.is_empty()) {
    bke::SpanAttributeWriter<int> parent_attribute = curves.attributes_for_write().lookup_or_add_for_write_only_span<int>(parent_index_id, bke::AttrDomain::Curve);
    array_utils::copy(parent_index_data.as_span(), parent_attribute.span);
  }

  curves.tag_topology_changed();
  curves.tag_positions_changed();

  return curves_id;
}

}  // namespace blender::geometry

#endif
