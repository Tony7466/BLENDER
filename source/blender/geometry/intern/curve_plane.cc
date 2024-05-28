/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#ifdef WITH_POTRACE

#  include <iostream>

#  include "potracelib.h"

#  include "BKE_attribute.hh"
#  include "BKE_curves.hh"

#  include "BLI_array_utils.hh"
#  include "BLI_map.hh"
#  include "BLI_math_vector_types.hh"
#  include "BLI_offset_indices.hh"
#  include "BLI_task.hh"

namespace blender::geometry {

static constexpr const int segment_size = sizeof(potrace_word) * 8;

static potrace_word from_bools(const Span<bool> src)
{
  BLI_assert(src.size() <= segment_size);
  potrace_word result = 0;
  for (const int64_t i : src.index_range()) {
    const potrace_word bit(src[i]);
    BLI_assert(ELEM(bit, 0, 1));
    result |= bit << (segment_size - 1 - i);
  }
  return result;
}

static potrace_state_t *bools_to_potrace_curves(const int2 resolution, const Span<bool> grid_color)
{
  const int full_segments_num = resolution.x / segment_size;
  const int extra_bits_num = resolution.x % segment_size;
  const int extra_segment = extra_bits_num == 0 ? 0 : 1;
  const int segments_num = full_segments_num + extra_segment;

  Array<potrace_word> segments(segments_num * resolution.y);
  threading::parallel_for(
      IndexRange(resolution.y),
      4096,
      [&](const IndexRange range) {
        for (const int y_index : range) {
          const Span<bool> src_line = grid_color.slice(y_index * resolution.x, resolution.x);
          MutableSpan<potrace_word> line = segments.as_mutable_span().slice(y_index * segments_num,
                                                                            segments_num);
          for (const int i : IndexRange(full_segments_num)) {
            line[i] = from_bools(src_line.slice(i * segment_size, segment_size));
          }
          if (extra_bits_num > 0) {
            line.last() = from_bools(
                src_line.slice(full_segments_num * segment_size, extra_bits_num));
          }
        }
      },
      threading::accumulated_task_sizes(
          [&](const IndexRange range) { return range.size() * resolution.x; }));

  potrace_bitmap_t bitmap;
  bitmap.w = resolution.x;
  bitmap.h = resolution.y;
  bitmap.dy = segments_num;
  bitmap.map = segments.data();
  potrace_param_t *params = potrace_param_default();
  BLI_SCOPED_DEFER([&]() { potrace_param_free(params); });

  BLI_assert(params != nullptr);
  return potrace_trace(params, &bitmap);
}

static void potrace_gather_curves(const potrace_state_t *potrace, Vector<const potrace_path_t *> &r_list)
{
  for (const potrace_path_t *iter = potrace->plist; iter != nullptr; iter = iter->next) {
    r_list.append(iter);
  }
}

static void potrace_curve_parent_index(const Span<const potrace_path_t *> src_curves,
                                       Array<int> &r_parent_index)
{
  r_parent_index.reinitialize(src_curves.size());

  Map<const potrace_path_t *, int> child_parent_index;

  for (const int parent_i : src_curves.index_range()) {
    for (potrace_path_t *iter = src_curves[parent_i]->childlist; iter != nullptr;
         iter = iter->sibling)
    {
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

static float2 to_float2(const potrace_dpoint_s &point)
{
  return float2(point.x, point.y);
}

template<typename T, int num>
std::ostream &operator<<(std::ostream &stream, const std::array<T, num> &data)
{
  stream << "{";
  for (const int64_t i : IndexRange(num)) {
    stream << data[i] << (num - 1 == i ? "" : ", ");
  }
  stream << "}";
  return stream;
}

template<typename T> std::ostream &operator<<(std::ostream &stream, const Span<T> span)
{
  for (const int64_t i : span.index_range()) {
    stream << span[i] << (span.size() - 1 == i ? "" : ", ");
  }
  return stream;
}

template<typename T> std::ostream &operator<<(std::ostream &stream, MutableSpan<T> span)
{
  stream << span.as_span();
  return stream;
}

template<typename In, typename Out, typename Func>
static void parallel_transform(const Span<In> src,
                               const int64_t grain_size,
                               MutableSpan<Out> dst,
                               const Func func)
{
  BLI_assert(src.size() == dst.size());
  threading::parallel_for(src.index_range(), grain_size, [&](const IndexRange range) {
    const Span<In> src_slice = src.slice(range);
    MutableSpan<Out> dst_slice = dst.slice(range);
    std::transform(src_slice.begin(), src_slice.end(), dst_slice.begin(), func);
  });
}

namespace potrace {

enum class CurveType : int8_t {
  BezierSpline,
  PolySpline,
  FirstBezierSegment,
  FirstPolySegment,
};

/* Don't reinterpret potrace types as SegmentType. */
enum struct SegmentType : int8_t {
  Bezier = POTRACE_CURVETO,
  Poly = POTRACE_CORNER,
};

static bool extra_bezier_point_between(const SegmentType a_segment_type,
                                       const SegmentType b_segment_type)
{
  if (a_segment_type == SegmentType::Poly && b_segment_type == SegmentType::Bezier) {
    return true;
  }
  return false;
}

static int extra_points_in_curve(const Span<int> segment_types)
{
  const IndexRange range = segment_types.index_range().drop_back(1);
  const int extra_points_num = std::count_if(range.begin(), range.end(), [&](const int64_t i) {
    return extra_bezier_point_between(SegmentType(segment_types[i]),
                                      SegmentType(segment_types[i + 1]));
  });
  const bool extra_point_in_cycle = extra_bezier_point_between(SegmentType(segment_types.last()),
                                                               SegmentType(segment_types.first()));
  return extra_points_num + int(extra_point_in_cycle);
}

static std::array<float2, 3> extra_point_between(const std::array<float2, 3> &a_point,
                                                 // const SegmentType a_segment_type,
                                                 const std::array<float2, 3> &b_point  // ,
                                                 // const SegmentType b_segment_type
)
{
  // BLI_assert(a_segment_type == SegmentType::Poly && b_segment_type == SegmentType::Bezier);
  const float2 control_point = a_point[2];
  const float2 prev_handle = math::midpoint(a_point[1], a_point[2]);
  const float2 next_handle = b_point[0];
  return {prev_handle, control_point, next_handle};
}

}  // namespace potrace

static void potrace_curve_count_type_switch(const Span<const potrace_path_t *> src_curves, MutableSpan<int> curve_type_switch_offset_num)
{
  static constexpr const int plus_to_store_offsets = 1;
  threading::parallel_for(src_curves.index_range(), 4096, [&](const IndexRange range) {
    for (const int curve_i : range) {
      const Span<int> segment_types(src_curves[curve_i]->curve.tag, src_curves[curve_i]->curve.n);
      const IndexRange range = segment_types.index_range().drop_back(1);
      const int different_types_num = std::count_if(range.begin(), range.end(), [&](const int64_t i) {
        return potrace::SegmentType(segment_types[i]) != potrace::SegmentType(segment_types[i + 1]);
      });
      curve_type_switch_offset_num[curve_i] = different_types_num + plus_to_store_offsets;
    }
  });
}

static void type_group_sizes(const Span<int> segment_types, MutableSpan<int> indices)
{
  int offset = 0;
  int last_i = 0;
  for (const int i : segment_types.index_range().drop_front(1)) {
    if (potrace::SegmentType(segment_types[i - 1]) != potrace::SegmentType(segment_types[i])) {
      indices[offset] = i - last_i;
      offset++;
      last_i = i;
    }
  }
}

static void potrace_curve_type_switch_sizes(const Span<const potrace_path_t *> src_curves,
                                            const OffsetIndices<int> curve_type_switch_offset,
                                            MutableSpan<int> src_type_switch_sizes_num)
{
  threading::parallel_for(src_curves.index_range(), 4096, [&](const IndexRange range) {
    for (const int curve_i : range) {
      const Span<int> segment_types(src_curves[curve_i]->curve.tag, src_curves[curve_i]->curve.n);
      MutableSpan<int> type_switch_sizes_num = src_type_switch_sizes_num.slice(curve_type_switch_offset[curve_i]);
      type_group_sizes(segment_types, type_switch_sizes_num);
    }
  });
}

static void potrace_curve_type_switch_extra_sizes(const Span<const potrace_path_t *> src_curves,
                                                  const OffsetIndices<int> curve_type_switch_offset,
                                                  MutableSpan<int> extra_type_switch_sizes_num)
{
  threading::parallel_for(src_curves.index_range(), 4096, [&](const IndexRange range) {
    for (const int curve_i : range) {
      const Span<int> segment_types(src_curves[curve_i]->curve.tag, src_curves[curve_i]->curve.n);
      MutableSpan<int> type_switch_sizes_num = src_type_switch_sizes_num.slice(curve_type_switch_offset[curve_i]);
      type_group_sizes(segment_types, type_switch_sizes_num);
    }
  });
}













static int count_segment_groups(const Span<int> segment_types)
{
  const IndexRange range = segment_types.index_range().drop_back(1);
  const int different_types_num = std::count_if(range.begin(), range.end(), [&](const int64_t i) {
    return potrace::SegmentType(segment_types[i]) != potrace::SegmentType(segment_types[i + 1]);
  });
  return different_types_num;
}

static int count_extra_points(const Span<potrace::CurveType> types)
{
  /* If there is just 2 type of segments, number of extra points can be found in trivial way. */
  if constexpr (false) {
    const IndexRange range = types.index_range().drop_back(1);
    const int different_types_num = std::count_if(
        range.begin(), range.end(), [&](const int64_t i) {
          return extra_bezier_point_between(potrace::SegmentType(types[i]),
                                            potrace::SegmentType(types[i + 1]));
        });
    const bool different_types_in_cycle = extra_bezier_point_between(
        potrace::SegmentType(types.last()), potrace::SegmentType(types.first()));
    return different_types_num + int(different_types_in_cycle);
  }
  return types.size() / 2;
}

static potrace::CurveType curve_type(const Span<int> segment_types, const int extra_points)
{
  if (extra_points == 0) {
    /* Assumption that there is only poly and bezier types. This means, if number of extra points
     * is zero, here is curve of just one certain type. */
    switch (potrace::SegmentType(segment_types.first())) {
      case potrace::SegmentType::Bezier:
        return potrace::CurveType::BezierSpline;
      case potrace::SegmentType::Poly:
        return potrace::CurveType::PolySpline;
      default:
        BLI_assert_unreachable();
        return {};
    }
  }
  switch (potrace::SegmentType(segment_types.first())) {
    case potrace::SegmentType::Bezier:
      return potrace::CurveType::FirstBezierSegment;
    case potrace::SegmentType::Poly:
      return potrace::CurveType::FirstPolySegment;
    default:
      BLI_assert_unreachable();
      return {};
  }
}

static void potrace_curve_sizes(const Span<const potrace_path_t *> src_curves,
                                MutableSpan<int> offsets)
{
  BLI_assert(!src_curves.is_empty());
  threading::parallel_for(src_curves.index_range(), 4096, [&](const IndexRange range) {
    for (const int curve_i : range) {
      offsets[curve_i] = src_curves[curve_i]->curve.n;
    }
  });
}

static void transform_points(const int2 resolution,
                             const float2 min_point,
                             const float2 max_point,
                             MutableSpan<std::array<float2, 3>> points_data)
{
  const float2 resolution_factor = float2(1.0f) / float2(resolution) * (max_point - min_point);
  parallel_transform(
      points_data.as_span().cast<float2>(),
      4096,
      points_data.cast<float2>(),
      [&](const float2 src_point) { return min_point + src_point * resolution_factor; });
}

static void potrace_curve_segments_group_size(const Span<const potrace_path_t *> src_curves, MutableSpan<int> sizes)
{
  threading::parallel_for(src_curves.index_range(), 4096, [&](const IndexRange range) {
    for (const int curve_i : range) {
      const Span<int> segment_types(src_curves[curve_i]->curve.tag, src_curves[curve_i]->curve.n);
      static const constexpr int zero_and_total_size_elements = 2;
      r_curve_types_offset_data[curve_i] = count_different_types(segment_types) + zero_and_total_size_elements;
    }
  });
}

static void potrace_curve_count_segments_in_groups(const Span<const potrace_path_t *> src_curves, const OffsetIndices<int> offsets, MutableSpan<int> sizes)
{
  threading::parallel_for(src_curves.index_range(), 4096, [&](const IndexRange range) {
    for (const int curve_i : range) {
      const Span<int> segment_types(src_curves[curve_i]->curve.tag, src_curves[curve_i]->curve.n);
      MutableSpan<int> curve_type_indices_data = sizes.slice(offsets[curve_i]);
      type_group_sizes(segment_types, curve_type_indices_data.drop_back(1));
    }
  }, threading::accumulated_task_sizes([&](const IndexRange range) { return offsets[range].size(); }));
}

static void curve_count_segments_in_groups(const Span<const potrace_path_t *> src_curves, const OffsetIndices<int> offsets, MutableSpan<int> sizes)
{
  threading::parallel_for(src_curves.index_range(), 4096, [&](const IndexRange range) {
    for (const int curve_i : range) {
      const Span<int> segment_types(src_curves[curve_i]->curve.tag, src_curves[curve_i]->curve.n);
      MutableSpan<int> curve_type_indices_data = sizes.slice(offsets[curve_i]);
      type_group_sizes(segment_types, curve_type_indices_data.drop_back(1));
    }
  }, threading::accumulated_task_sizes([&](const IndexRange range) { return offsets[range].size(); }));
}

static void potrace_curve_slice_segments(const Span<const potrace_path_t *> src_curves,
                                         Array<int> &r_curve_types_offset_data,
                                         Array<int> &r_curve_type_indices_data)
{
  r_curve_types_offset_data.reinitialize(src_curves.size() + 1);
  threading::parallel_for(src_curves.index_range(), 4096, [&](const IndexRange range) {
    for (const int curve_i : range) {
      const Span<int> segment_types(src_curves[curve_i]->curve.tag, src_curves[curve_i]->curve.n);
      r_curve_types_offset_data[curve_i] = count_different_types(segment_types);
    }
  });
  const OffsetIndices<int> curve_types_offset = offset_indices::accumulate_counts_to_offsets(
      r_curve_types_offset_data);

  r_curve_type_indices_data.reinitialize(curve_types_offset.total_size());
  threading::parallel_for(
      src_curves.index_range(),
      4096,
      [&](const IndexRange range) {
        for (const int curve_i : range) {
          const Span<int> segment_types(src_curves[curve_i]->curve.tag,
                                        src_curves[curve_i]->curve.n);
          MutableSpan<int> curve_type_indices_data =
              r_curve_type_indices_data.as_mutable_span().slice(curve_types_offset[curve_i]);
          if (curve_type_indices_data.is_empty()) {
            continue;
          }
          copy_type_switch_indices(segment_types, curve_type_indices_data);
        }
      },
      threading::accumulated_task_sizes(
          [&](const IndexRange range) { return curve_types_offset[range].size(); }));
}

static void potrace_curve_extra_points(const Span<const potrace_path_t *> src_curves,
                                       const GroupedSpan<int> curve_type_indices,
                                       MutableSpan<int> extra_curve_offset_data)
{
  threading::parallel_for(src_curves.index_range(), 4096, [&](const IndexRange range) {
    for (const int curve_i : range) {
      // std::cout << "potrace_curve_extra_points: " << "curve_i: " << curve_i << ";\n";
      const Span<int> segment_types(src_curves[curve_i]->curve.tag, src_curves[curve_i]->curve.n);
      const Span<int> curve_type_switch_indices = curve_type_indices[curve_i];
      const IndexRange curve_range = curve_type_switch_indices.index_range();
      // std::cout << curve_range << ";\n";
      // std::cout << segment_types << ";\n";
      // std::cout << curve_type_switch_indices << ";\n";
      // std::cout << "\n";
      extra_curve_offset_data[curve_i] = std::count_if(
          curve_range.begin(), curve_range.end(), [&](const int64_t i) {
            const potrace::SegmentType current_type = potrace::SegmentType(
                segment_types[curve_type_switch_indices[i] - 1]);
            const potrace::SegmentType next_type = potrace::SegmentType(
                segment_types[curve_type_switch_indices[i]]);
            return potrace::extra_bezier_point_between(current_type, next_type);
          });
      if (potrace::extra_bezier_point_between(potrace::SegmentType(segment_types.last()),
                                              potrace::SegmentType(segment_types.first())))
      {
        extra_curve_offset_data[curve_i]++;
      }
      // std::cout << "extra_curve_offset_data: " << extra_curve_offset_data[curve_i] << ";\n";
    }
  });
}

static void potrace_curve_type(const Span<const potrace_path_t *> src_curves,
                               const OffsetIndices<int> extra_curve_offsets,
                               MutableSpan<potrace::CurveType> curve_type_data)
{
  threading::parallel_for(src_curves.index_range(), 4096, [&](const IndexRange range) {
    for (const int curve_i : range) {
      const Span<int> segment_types(src_curves[curve_i]->curve.tag, src_curves[curve_i]->curve.n);
      curve_type_data[curve_i] = curve_type(segment_types, extra_curve_offsets[curve_i].size());
    }
  });
}

static void potrace_curve_to_bezier(const Span<std::array<float2, 3>> src,
                                    MutableSpan<std::array<float2, 3>> dst)
{
  BLI_assert(src.size() == dst.size());
  for (const int i : src.index_range()) {
    dst[i][0] = src[i][1];
    dst[i][1] = src[i][2];
    dst[i][2] = src[i][0];
  }

  const float2 first_handle = dst.first()[2];
  for (const int i : dst.index_range().drop_back(1)) {
    dst[i][2] = dst[i + 1][2];
  }
  dst.last()[2] = first_handle;
}

static void potrace_bezeir_after_poly(const std::array<float2, 3> poly,
                                      MutableSpan<std::array<float2, 3>> points_data)
{
  // points_data.first() = potrace::extra_point_between(poly, potrace::SegmentType::Poly,
  // points_data[1], potrace::SegmentType::Bezier);
  points_data = points_data.drop_front(1);

  const float2 first_handle = points_data.first()[2];
  for (const int i : points_data.index_range().drop_back(1)) {
    points_data[i][2] = points_data[i + 1][2];
  }
  points_data.last()[2] = first_handle;
}

static void potrace_bezeir_curve_shift_poly_handles(MutableSpan<std::array<float2, 3>> points_data)
{
  // const float2 first_handle = points_data.last()[2];
  for (const int i : points_data.index_range().drop_back(1)) {
    points_data[i + 1][0] = points_data[i][2];
  }
  // points_data.first()[0] = first_handle;
}

static void potrace_poly_after_bezier(const std::array<float2, 3> bezier,
                                      MutableSpan<std::array<float2, 3>> points_data)
{
  const float2 first_handle = points_data.last()[2];
  for (const int i : points_data.index_range().drop_back(1)) {
    points_data[i + 1][0] = points_data[i][2];
  }
  points_data.first()[0] = first_handle;
}

static void potrace_curve_points(const Span<const potrace_path_t *> src_curves,
                                 const OffsetIndices<int> offsets,
                                 const OffsetIndices<int> extra_point_offsets,
                                 const GroupedSpan<int> curve_type_indices,
                                 MutableSpan<std::array<float2, 3>> vector_data)
{
  // std::cout << "\n";
  threading::parallel_for(
      src_curves.index_range(),
      4096,
      [&](const IndexRange range) {
        for (const int curve_i : range) {
          // std::cout << " curve_i: " << curve_i << ";\n";
          const IndexRange curve_points = range_sum(offsets[curve_i],
                                                    extra_point_offsets[curve_i]);
          const Span<int> segment_types(src_curves[curve_i]->curve.tag,
                                        src_curves[curve_i]->curve.n);
          const Span<int> type_indices = curve_type_indices[curve_i];
          MutableSpan<std::array<float2, 3>> points = vector_data.slice(curve_points);

          // const Span<potrace_dpoint_t [3]> dyd(src_curves[curve_i]->curve.c,
          // src_curves[curve_i]->curve.n); BLI_assert(dyd[0][0].x != 6);

          if (type_indices.is_empty()) {
            // std::cout << "  Single;\n";
            // std::cout << "  curve_points: " << curve_points << ";\n";
            // std::cout << "  segment_types: " << segment_types << ";\n";
            // std::cout << "  type_indices: " << type_indices << ";\n";
            for (const int64_t i : points.index_range()) {
              points[i][0] = to_float2(src_curves[curve_i]->curve.c[i][0]);
              points[i][1] = to_float2(src_curves[curve_i]->curve.c[i][1]);
              points[i][2] = to_float2(src_curves[curve_i]->curve.c[i][2]);
            }
            continue;
          }

          int64_t offset = 0;

          {
            const IndexRange typed_range(type_indices[0]);

            // std::cout << "  First segment;\n";
            // std::cout << "  typed_range: " << typed_range << ";\n";
            // std::cout << "  curve_points: " << curve_points << ";\n";
            // std::cout << "  segment_types: " << segment_types << ";\n";
            // std::cout << "  type_indices: " << type_indices << ";\n";

            BLI_assert(!typed_range.is_empty());
            const potrace::SegmentType range_type = potrace::SegmentType(segment_types[0]);
            const potrace::SegmentType next_range_type = potrace::SegmentType(
                segment_types[type_indices[0]]);

            for (const int64_t i : typed_range) {
              BLI_assert(offset == 0);
              points[i][0] = to_float2(src_curves[curve_i]->curve.c[i][0]);
              points[i][1] = to_float2(src_curves[curve_i]->curve.c[i][1]);
              points[i][2] = to_float2(src_curves[curve_i]->curve.c[i][2]);
            }

            if (potrace::extra_bezier_point_between(range_type, next_range_type)) {
              offset++;
            }
          }

          for (const int range_i : type_indices.index_range().drop_back(1)) {

            // std::cout << AT << ";\n";
            const int range_index = type_indices[range_i];
            const int next_range_index = type_indices[range_i + 1];
            const potrace::SegmentType range_type = potrace::SegmentType(
                segment_types[range_index]);
            const potrace::SegmentType next_range_type = potrace::SegmentType(
                segment_types[next_range_index]);
            const IndexRange typed_range = IndexRange::from_begin_end(range_index,
                                                                      next_range_index);

            // std::cout << "  range_i: " << range_i << ";\n";
            // std::cout << "  range_index: " << range_index << ";\n";
            // std::cout << "  next_range_index: " << next_range_index << ";\n";
            // std::cout << "  typed_range: " << typed_range << ";\n";
            // std::cout << "  curve_points: " << curve_points << ";\n";
            // std::cout << "  segment_types: " << segment_types << ";\n";
            // std::cout << "  type_indices: " << type_indices << ";\n";
            // std::cout << "  offset: " << offset << ";\n";

            for (const int64_t i : typed_range) {
              // std::cout << AT << ";\n";
              points[offset + i][0] = to_float2(src_curves[curve_i]->curve.c[i][0]);
              points[offset + i][1] = to_float2(src_curves[curve_i]->curve.c[i][1]);
              points[offset + i][2] = to_float2(src_curves[curve_i]->curve.c[i][2]);
            }

            if (potrace::extra_bezier_point_between(range_type, next_range_type)) {
              offset++;
            }
          }

          {
            const IndexRange typed_range = IndexRange::from_begin_end(type_indices.last(),
                                                                      segment_types.size());

            // std::cout << "  Last segment;\n";
            // std::cout << "  typed_range: " << typed_range << ";\n";
            // std::cout << "  curve_points: " << curve_points << ";\n";
            // std::cout << "  segment_types: " << segment_types << ";\n";
            // std::cout << "  type_indices: " << type_indices << ";\n";

            BLI_assert(!typed_range.is_empty());
            const potrace::SegmentType range_type = potrace::SegmentType(segment_types[0]);
            const potrace::SegmentType next_range_type = potrace::SegmentType(
                segment_types[type_indices[0]]);

            for (const int64_t i : typed_range) {
              points[offset + i][0] = to_float2(src_curves[curve_i]->curve.c[i][0]);
              points[offset + i][1] = to_float2(src_curves[curve_i]->curve.c[i][1]);
              points[offset + i][2] = to_float2(src_curves[curve_i]->curve.c[i][2]);
            }
          }
        }
      },
      threading::accumulated_task_sizes([&](const IndexRange range) {
        return offsets[range].size() + extra_point_offsets[range].size();
      }));
}

static std::array<float2, 3> bezier_before_bezier(const int index,
                                                  const Span<std::array<float2, 3>> potrace_data)
{
  // std::cout << "    - " << __func__ << ": " << index << ";\n";
  std::array<float2, 3> point = potrace_data[index];
  point[0] = point[1];
  point[1] = point[2];
  const int next_index = math::mod_periodic<int64_t>(index + 1, potrace_data.size());
  point[2] = potrace_data[next_index][0];
  return point;
}

static std::array<float2, 3> bezier_before_poly(const int index,
                                                const Span<std::array<float2, 3>> potrace_data)
{
  // std::cout << "    - " << __func__ << ": " << index << ";\n";
  std::array<float2, 3> point = potrace_data[index];
  point[0] = point[1];
  point[1] = point[2];
  const int next_index = math::mod_periodic<int64_t>(index + 1, potrace_data.size());
  point[2] = math::midpoint(point[1], potrace_data[next_index][1]);
  return point;
}

static std::array<float2, 3> poly_after_poly(const int index,
                                             const Span<std::array<float2, 3>> potrace_data)
{
  // std::cout << "    - " << __func__ << ": " << index << ";\n";
  std::array<float2, 3> point = potrace_data[index];
  const int prev_index = math::mod_periodic<int64_t>(index - 1, potrace_data.size());
  point[0] = potrace_data[prev_index][2];
  return point;
}

static std::array<float2, 3> poly_after_bezier(const int index,
                                               const Span<std::array<float2, 3>> potrace_data)
{
  // std::cout << "    - " << __func__ << ": " << index << ";\n";
  std::array<float2, 3> point = potrace_data[index];
  const int prev_index = math::mod_periodic<int64_t>(index - 1, potrace_data.size());
  point[0] = math::midpoint(potrace_data[prev_index][2], point[1]);
  return point;
}

static std::array<float2, 3> poly_point_bezier(const int index,
                                               const Span<std::array<float2, 3>> potrace_data)
{
  // std::cout << "    - " << __func__ << ": " << index << ";\n";
  const int prev_index = math::mod_periodic<int64_t>(index - 1, potrace_data.size());
  const int next_index = math::mod_periodic<int64_t>(index + 1, potrace_data.size());

  const std::array<float2, 3> &prev_poly = potrace_data[prev_index];
  const std::array<float2, 3> &next_bezier = potrace_data[next_index];

  return {math::midpoint(prev_poly[1], prev_poly[2]), prev_poly[2], next_bezier[0]};
}

static void potrace_to_bezier_or_poly_curves(const Span<const potrace_path_t *> src_curves,
                                             const OffsetIndices<int> curve_offsets,
                                             const OffsetIndices<int> extra_point_offsets,
                                             const GroupedSpan<int> curve_type_indices,
                                             const Span<potrace::CurveType> curve_type_data,
                                             const Span<std::array<float2, 3>> potrace_data,
                                             MutableSpan<std::array<float2, 3>> points_data)
{
  // std::cout << "\n";

  // std::cout << potrace_data << "\n";

  threading::parallel_for(
      curve_offsets.index_range(),
      4096,
      [&](const IndexRange range) {
        for (const int curve_i : range) {
          // std::cout << "curve_i: " << curve_i << ";\n";

          const IndexRange curve_points = range_sum(curve_offsets[curve_i],
                                                    extra_point_offsets[curve_i]);
          const Span<int> segment_types(src_curves[curve_i]->curve.tag,
                                        src_curves[curve_i]->curve.n);
          const Span<int> type_indices = curve_type_indices[curve_i];
          const Span<std::array<float2, 3>> src_points = potrace_data.slice(curve_points);
          MutableSpan<std::array<float2, 3>> dst_points = points_data.slice(curve_points);

          const int last = dst_points.size() - 1;

          // std::cout << "  curve_points: " << curve_points << ";\n";
          // std::cout << "  segment_types: " << segment_types << ";\n";
          // std::cout << "  type_indices: " << type_indices << ";\n";

          const potrace::CurveType curve_type = curve_type_data[curve_i];
          if (type_indices.is_empty()) {
            BLI_assert(ELEM(
                curve_type, potrace::CurveType::BezierSpline, potrace::CurveType::PolySpline));
            if (curve_type == potrace::CurveType::BezierSpline) {
              // std::cout << "  Single Beier;\n";
              potrace_curve_to_bezier(src_points, dst_points);
            }
            else {
              // std::cout << "  Single Poly;\n";
              array_utils::copy(src_points, dst_points);
            }
            continue;
          }

          const bool merge_end_and_begin = segment_types.first() == segment_types.last();

          int64_t offset = 0;
          {
            const IndexRange typed_range(type_indices[0]);
            BLI_assert(!typed_range.is_empty());

            const potrace::SegmentType range_type = potrace::SegmentType(segment_types[0]);
            const potrace::SegmentType next_range_type = potrace::SegmentType(
                segment_types[type_indices[0]]);

            switch (range_type) {
              case potrace::SegmentType::Poly: {
                if (merge_end_and_begin) {
                  dst_points[0] = poly_after_poly(0, src_points);
                }
                else {
                  dst_points[0] = poly_after_bezier(0, src_points);
                }
                for (const int index : typed_range.drop_front(1)) {
                  dst_points[index] = poly_after_poly(index, src_points);
                }
                const int extra_point = typed_range.one_after_last();
                dst_points[extra_point] = poly_point_bezier(extra_point, src_points);
                break;
              }
              case potrace::SegmentType::Bezier: {
                for (const int index : typed_range.drop_back(1)) {
                  dst_points[index] = bezier_before_bezier(index, src_points);
                }
                dst_points[typed_range.last()] = bezier_before_poly(typed_range.last(),
                                                                    src_points);
                break;
              }
            }

            if (potrace::extra_bezier_point_between(range_type, next_range_type)) {
              offset++;
            }
          }

          for (const int range_i : type_indices.index_range().drop_back(1)) {
            const int range_index = type_indices[range_i];
            const int next_range_index = type_indices[range_i + 1];

            const IndexRange segment_range =
                IndexRange::from_begin_end(range_index, next_range_index).shift(offset);
            switch (potrace::SegmentType(segment_types[range_index])) {
              case potrace::SegmentType::Poly: {
                dst_points[range_index + offset] = poly_after_bezier(range_index + offset,
                                                                     src_points);
                for (const int index : segment_range.drop_front(1)) {
                  dst_points[index] = poly_after_poly(index, src_points);
                }
                const int extra_point = segment_range.one_after_last();
                dst_points[extra_point] = poly_point_bezier(extra_point, src_points);
                offset++;
                break;
              }
              case potrace::SegmentType::Bezier: {
                for (const int index : segment_range.drop_back(1)) {
                  dst_points[index] = bezier_before_bezier(index, src_points);
                }
                dst_points[segment_range.last()] = bezier_before_poly(segment_range.last(),
                                                                      src_points);
                break;
              }
            }
          }

          {
            const IndexRange typed_range = IndexRange::from_begin_end(type_indices.last(),
                                                                      segment_types.size());
            BLI_assert(!typed_range.is_empty());

            switch (potrace::SegmentType(segment_types.last())) {
              case potrace::SegmentType::Poly: {
                dst_points[typed_range[0] + offset] = poly_after_bezier(typed_range[0] + offset,
                                                                        src_points);
                for (const int index : typed_range.drop_front(1)) {
                  dst_points[index + offset] = poly_after_poly(index + offset, src_points);
                }
                if (!merge_end_and_begin) {
                  const int extra_point = typed_range.one_after_last() + offset;
                  dst_points[extra_point] = poly_point_bezier(extra_point, src_points);
                }
                break;
              }
              case potrace::SegmentType::Bezier: {
                for (const int index : typed_range.drop_back(1)) {
                  dst_points[index + offset] = bezier_before_bezier(index + offset, src_points);
                }
                if (!merge_end_and_begin) {
                  dst_points[last] = bezier_before_poly(last, src_points);
                }
                else {
                  dst_points[last] = bezier_before_bezier(last, src_points);
                }
                break;
              }
            }
          }
        }
      },
      threading::accumulated_task_sizes(
          [&](const IndexRange range) { return curve_offsets[range].size(); }));
  // std::cout << points_data << "\n";
}

std::optional<Curves *> plane_to_curve(const int2 resolution,
                                       const Span<bool> grid_color,
                                       const float2 min_point,
                                       const float2 max_point,
                                       const bke::AttributeIDRef &parent_index_id)
{
  potrace_state_t *potrace_result = bools_to_potrace_curves(resolution, grid_color);
  if (potrace_result == nullptr) {
    return nullptr;
  }
  BLI_SCOPED_DEFER([&]() { potrace_state_free(potrace_result); });

  Vector<const potrace_path_t *> curves_list;
  potrace_gather_curves(potrace_result, curves_list);
  if (curves_list.is_empty()) {
    return std::nullopt;
  }

  Array<int> parent_index_data;
  if (parent_index_id) {
    potrace_curve_parent_index(curves_list, parent_index_data);
  }

  Array<int> curve_type_switch_offset_num(curves_list.size() + 1);
  potrace_curve_count_type_switch(curves_list, curve_type_switch_offset_num);
  const OffsetIndices<int> curve_type_switch_offset = offset_indices::accumulate_counts_to_offsets(curve_type_switch_offset_num);

  Array<int> src_type_switch_sizes_num(curve_type_switch_offset.total_size());
  potrace_curve_type_switch_sizes(curves_list, curve_type_switch_offset, src_type_switch_sizes_num);
  const int64_t src_total_segments = accumulate_groups_total(curve_type_switch_offset, src_type_switch_sizes_num);

  Array<int> extra_type_switch_sizes_num(curve_type_switch_offset.total_size());
  potrace_curve_type_switch_extra_sizes(curves_list, curve_type_switch_offset, extra_type_switch_sizes_num);
  const int64_t extra_total_segments = accumulate_groups_total(curve_type_switch_offset, extra_type_switch_sizes_num);

  const int64_t total_vertices = src_total_segments + extra_total_segments;

  Curves *curves_id = bke::curves_new_nomain(total_vertices, curves_list.size());
  bke::CurvesGeometry &curves = curves_id->geometry.wrap();

  curves.cyclic_for_write().fill(true);

  MutableSpan<int> dst_curve_offsets = curves.offsets_for_write();
  threading::parallel_for(curves_list.index_range(), 4096, [&](const IndexRange range) {
    for (const int curve_i : range) {
      const IndexRange curve_segments_groups = curve_type_switch_offset[curve_i];
      const OffsetIndices<int> potrace_curve_segments_groups = src_type_switch_sizes_num.as_span().slice(curve_segments_groups);
      const OffsetIndices<int> extra_curve_segments_group_points = extra_type_switch_sizes_num.as_span().slice(curve_segments_groups);
      dst_curve_offsets[curve_i] = potrace_curve_segments_groups.total_size() + extra_curve_segments_group_points.total_size();
    }
  });
  const OffsetIndices<int> curve_offsets = offset_indices::accumulate_counts_to_offsets(dst_curve_offsets);

  IndexMaskMemory memory;
  const IndexMask poly_curves_mask = IndexMask::from_predicate(curves_list.index_range(), GrainSize(4096), memory, [&](const int curve_i) {
    const IndexRange curve_segments_groups = curve_type_switch_offset[curve_i];
    const OffsetIndices<int> potrace_curve_segments_groups = src_type_switch_sizes_num.as_span().slice(curve_segments_groups);
    if (potrace_curve_segments_groups.size() != 1) {
      return false;
    }
    const Span<int> segment_types(curves_list[curve_i]->curve.tag, curves_list[curve_i]->curve.n);
    const potrace::SegmentType curve_type = potrace::SegmentType(segment_types.first());
    if (curve_type != potrace::SegmentType::Poly) {
      return false;
    }
    BLI_assert(std::all_of(segment_types.begin(), segment_types.end(), [&](const int type) {
      return type == segment_types.first();
    }));
    return true;
  });
  const IndexMask bezier_curves_mask = poly_curves_mask.complement(curves_list.index_range(), memory);

  index_mask::masked_fill<int8_t>(curves.curve_types_for_write(), CURVE_TYPE_POLY, poly_curves_mask);
  index_mask::masked_fill<int8_t>(curves.curve_types_for_write(), CURVE_TYPE_BEZIER, bezier_curves_mask);

  MutableSpan<int8_t> handle_types_left = curves.handle_types_left_for_write();
  bezier_curves_mask.foreach_index(GrainSize(4096), [&](const int curve_i) {
    handle_types_left.slice(curve_offsets[curve_i]).fill(BEZIER_HANDLE_FREE);
  });
  MutableSpan<int8_t> handle_types_right = curves.handle_types_right_for_write();
  bezier_curves_mask.foreach_index(GrainSize(4096), [&](const int curve_i) {
    handle_types_right.slice(curve_offsets[curve_i]).fill(BEZIER_HANDLE_FREE);
  });

  MutableSpan<float3> positions = curves.positions_for_write();
  MutableSpan<float3> positions_left = curves.handle_positions_left_for_write();
  MutableSpan<float3> positions_right = curves.handle_positions_right_for_write();

  positions.fill(float3(0.0f, 0.0f, -1.0f));
  positions_left.fill(float3(0.0f, 0.0f, -2.0f));
  positions_right.fill(float3(0.0f, 0.0f, -3.0f));

  poly_curves_mask.foreach_index(GrainSize(4096), [&](const int curve_i) {
    potrace_fill_poly_curve(curves_list[curve_i], positions.slice(curve_offsets[curve_i]));
  });

  bezier_curves_mask.foreach_index(GrainSize(4096), [&](const int curve_i) {
    const IndexRange curve_segments_groups = curve_type_switch_offset[curve_i];
    const OffsetIndices<int> potrace_curve_segments_groups = src_type_switch_sizes_num.as_span().slice(curve_segments_groups);
    const OffsetIndices<int> extra_curve_segments_group_points = extra_type_switch_sizes_num.as_span().slice(curve_segments_groups);

    potrace_fill_bezier_or_poly_curve(curves_list[curve_i],
                                      potrace_curve_segments_groups,
                                      extra_curve_segments_group_points,
                                      positions.slice(curve_offsets[curve_i]),
                                      positions_left.slice(curve_offsets[curve_i]),
                                      positions_right.slice(curve_offsets[curve_i]));
  });

  if (!parent_index_data.is_empty()) {
    bke::SpanAttributeWriter<int> parent_attribute =
      curves.attributes_for_write().lookup_or_add_for_write_only_span<int>(parent_index_id, bke::AttrDomain::Curve);
    array_utils::copy(parent_index_data.as_span(), parent_attribute.span);
  }

  // transform_points(resolution, min_point, max_point, points_data);

  curves.update_curve_types();
  curves.tag_topology_changed();
  curves.tag_positions_changed();

  return curves_id;
}

}  // namespace blender::geometry

#endif
