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

static IndexRange range_sum(const IndexRange a, const IndexRange b)
{
  return IndexRange(a.start() + b.start(), a.size() + b.size());
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

static int count_different_types(const Span<int> segment_types)
{
  const IndexRange range = segment_types.index_range().drop_front(1);
  const int different_types_num = std::count_if(range.begin(), range.end(), [&](const int64_t i) {
    return potrace::SegmentType(segment_types[i - 1]) != potrace::SegmentType(segment_types[i]);
  });
  return different_types_num;
}

static void copy_type_switch_indices(const Span<int> segment_types, MutableSpan<int> indices)
{
  const IndexRange range = segment_types.index_range().drop_front(1);
  std::copy_if(range.begin(), range.end(), indices.begin(), [&](const int64_t i) {
    return potrace::SegmentType(segment_types[i - 1]) != potrace::SegmentType(segment_types[i]);
  });
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
      const Span<int> segment_types(src_curves[curve_i]->curve.tag, src_curves[curve_i]->curve.n);
      const Span<int> curve_type_switch_indices = curve_type_indices[curve_i];
      const IndexRange curve_range = curve_type_switch_indices.index_range();
      std::cout << curve_range << ";\n";
      std::cout << segment_types << ";\n";
      std::cout << curve_type_switch_indices << ";\n";
      std::cout << "\n";
      extra_curve_offset_data[curve_i] = std::count_if(
          curve_range.begin(), curve_range.drop_back(1).end(), [&](const int64_t i) {
            const potrace::SegmentType current_type = potrace::SegmentType(
                segment_types[curve_type_switch_indices[i]]);
            const potrace::SegmentType next_type = potrace::SegmentType(
                segment_types[curve_type_switch_indices[i + 1]]);
            return potrace::extra_bezier_point_between(current_type, next_type);
          });
      if (potrace::extra_bezier_point_between(potrace::SegmentType(segment_types.last()),
                                              potrace::SegmentType(segment_types.first())))
      {
        extra_curve_offset_data[curve_i]++;
      }
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

static void potrace_curve_to_bezier(MutableSpan<std::array<float2, 3>> points_data)
{
  for (std::array<float2, 3> &point : points_data) {
    std::swap(point[1], point[2]);
    std::swap(point[0], point[2]);
  }
}

static void potrace_bezeir_curve_shift_handles(MutableSpan<std::array<float2, 3>> points_data)
{
  // const float2 first_handle = points_data.first()[2];
  for (const int i : points_data.index_range().drop_back(1)) {
    points_data[i][2] = points_data[i + 1][2];
  }
  // points_data.last()[2] = first_handle;
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
  // std::cout << AT << ";\n";
  threading::parallel_for(
      src_curves.index_range(),
      4096,
      [&](const IndexRange range) {
        for (const int curve_i : range) {
          // std::cout << AT << ";\n";
          const IndexRange curve_points = range_sum(offsets[curve_i],
                                                    extra_point_offsets[curve_i]);
          const Span<int> segment_types(src_curves[curve_i]->curve.tag,
                                        src_curves[curve_i]->curve.n);
          const Span<int> type_indices = curve_type_indices[curve_i];
          // std::cout << "DDDD: " << type_indices << ";\n";
          MutableSpan<std::array<float2, 3>> points = vector_data.slice(curve_points);

          if (type_indices.is_empty()) {
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
            BLI_assert(!typed_range.is_empty());
            const potrace::SegmentType range_type = potrace::SegmentType(segment_types[0]);
            const potrace::SegmentType next_range_type = potrace::SegmentType(
                segment_types[type_indices[0]]);

            for (const int64_t i : typed_range) {
              // std::cout << AT << ";\n";
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
        }
      },
      threading::accumulated_task_sizes([&](const IndexRange range) {
        return offsets[range].size() + extra_point_offsets[range].size();
      }));
}

static void potrace_to_bezier_or_poly_curves(const Span<const potrace_path_t *> src_curves,
                                             const OffsetIndices<int> curve_offsets,
                                             const OffsetIndices<int> extra_point_offsets,
                                             const GroupedSpan<int> curve_type_indices,
                                             const Span<potrace::CurveType> curve_type_data,
                                             MutableSpan<std::array<float2, 3>> points_data)
{
  std::cout << "\n";
  threading::parallel_for(
      curve_offsets.index_range(),
      4096,
      [&](const IndexRange range) {
        for (const int curve_i : range) {
          std::cout << "curve_i: " << curve_i << ";\n";

          const IndexRange curve_points = range_sum(curve_offsets[curve_i],
                                                    extra_point_offsets[curve_i]);
          const Span<int> segment_types(src_curves[curve_i]->curve.tag,
                                        src_curves[curve_i]->curve.n);
          const Span<int> type_indices = curve_type_indices[curve_i];
          MutableSpan<std::array<float2, 3>> points = points_data.slice(curve_points);

          const int points_num = curve_points.size();

          std::cout << "  curve_points: " << curve_points << ";\n";
          std::cout << "  segment_types: " << segment_types << ";\n";
          std::cout << "  type_indices: " << type_indices << ";\n";

          const potrace::CurveType curve_type = curve_type_data[curve_i];
          if (type_indices.is_empty()) {
            BLI_assert(ELEM(
                curve_type, potrace::CurveType::BezierSpline, potrace::CurveType::PolySpline));
            if (curve_type == potrace::CurveType::BezierSpline) {
              std::cout << "  Single Beier;\n";
              potrace_curve_to_bezier(points);
              const float2 first_handle = points.first()[2];
              potrace_bezeir_curve_shift_handles(points);
              points.last()[2] = first_handle;
            }
            else {
              std::cout << "  Single Poly;\n";
            }
            continue;
          }

          int64_t offset = 0;
          {
            const IndexRange typed_range(type_indices[0]);
            BLI_assert(!typed_range.is_empty());
            const potrace::SegmentType range_type = potrace::SegmentType(segment_types[0]);
            const potrace::SegmentType next_range_type = potrace::SegmentType(
                segment_types[type_indices[0]]);

            // TODO ....

            if (potrace::extra_bezier_point_between(range_type, next_range_type)) {
              offset++;
            }
          }

          for (const int range_i : type_indices.index_range().drop_back(1)) {
            const int range_index = type_indices[range_i];
            const int next_range_index = type_indices[range_i + 1];

            /* Int64_t is necessary! */
            const int prev_index = math::mod_periodic<int64_t>(offset + range_index - 1,
                                                               points_num);
            const int prev_prev_index = math::mod_periodic<int64_t>(offset + range_index - 2,
                                                                    points_num);

            const IndexRange segment_range =
                IndexRange::from_begin_end(range_index, next_range_index).shift(offset);
            MutableSpan<std::array<float2, 3>> segment_points = points.slice(segment_range);
            switch (potrace::SegmentType(segment_types[range_index])) {
              case potrace::SegmentType::Bezier: {
                std::cout << "   Bezier Segments:\n";
                std::cout << "    segment_range: " << segment_range << ";\n";
                std::cout << "    range_index: " << range_index << ";\n";
                std::cout << "    prev_index: " << prev_index << ";\n";
                std::cout << "    prev_prev_index: " << prev_prev_index << ";\n";

                for (std::array<float2, 3> &point : segment_points) {
                  std::swap(point[1], point[2]);
                  std::swap(point[0], point[2]);
                }

                // points[prev_index][2] = segment_points.first()[2];

                const float2 first_handle = segment_points[0][2];
                for (const int i : segment_points.index_range().drop_back(1)) {
                  segment_points[i][2] = segment_points[i + 1][2];
                }

                const float2 control_point = points[prev_prev_index][2];
                const float2 prev_handle = math::midpoint(points[prev_prev_index][1],
                                                          points[prev_prev_index][2]);
                const float2 next_handle = segment_points.first()[0];
                points[prev_index] = {prev_handle, control_point, first_handle};

                // points[prev_prev_index][2] = prev_handle;
                // points[prev_index] = potrace::extra_point_between(points[prev_prev_index],
                // segment_points.first());
                break;
              }
              case potrace::SegmentType::Poly: {
                std::cout << "   Poly Segments:\n";
                std::cout << "    segment_range: " << segment_range << ";\n";
                std::cout << "    range_index: " << range_index << ";\n";
                std::cout << "    prev_index: " << prev_index << ";\n";
                std::cout << "    prev_prev_index: " << prev_prev_index << ";\n";

                points[prev_index][2] = math::midpoint(segment_points[0][1], segment_points[0][1]);
                segment_points[0][0] = points[prev_index][2];
                for (const int i : segment_points.index_range().drop_back(1)) {
                  segment_points[i + 1][0] = segment_points[i][2];
                }

                offset++;
                break;
              }
              default:
                break;
            }
          }
        }
      },
      threading::accumulated_task_sizes(
          [&](const IndexRange range) { return curve_offsets[range].size(); }));
}

std::optional<Curves *> plane_to_curve(const int2 resolution,
                                       const Span<bool> grid_color,
                                       const float2 min_point,
                                       const float2 max_point,
                                       const bke::AttributeIDRef &parent_index_id)
{
  potrace_state_t *potrace_result = bools_to_potrace_curves(resolution, grid_color);
  if (potrace_result == nullptr) {
    potrace_state_free(potrace_result);
    return nullptr;
  }

  Vector<const potrace_path_t *> curves_list;
  gather_list(potrace_result->plist, curves_list);
  if (curves_list.is_empty()) {
    potrace_state_free(potrace_result);
    return std::nullopt;
  }

  Array<int> parent_index_data;
  if (parent_index_id) {
    potrace_curve_parent_index(curves_list, parent_index_data);
  }

  Array<int> offset_data(curves_list.size() + 1);
  potrace_curve_sizes(curves_list, offset_data);
  const OffsetIndices<int> curve_offsets = offset_indices::accumulate_counts_to_offsets(
      offset_data);

  Array<int> curve_types_offset_data;
  Array<int> curve_type_indices_data;
  potrace_curve_slice_segments(curves_list, curve_types_offset_data, curve_type_indices_data);
  const GroupedSpan<int> curve_type_indices(curve_types_offset_data.as_span(),
                                            curve_type_indices_data);

  Array<int> extra_curve_offset_data(curves_list.size() + 1);
  potrace_curve_extra_points(curves_list, curve_type_indices, extra_curve_offset_data);
  const OffsetIndices<int> extra_point_offsets = offset_indices::accumulate_counts_to_offsets(
      extra_curve_offset_data);

  Array<potrace::CurveType> curve_type_data(curves_list.size());
  potrace_curve_type(curves_list, extra_point_offsets, curve_type_data);

  const int64_t total_vertices = curve_offsets.total_size() + extra_point_offsets.total_size();
  std::cout << "Curves num: " << curve_offsets.size() << ";\nPoints num: " << total_vertices
            << ";\n";

  Array<std::array<float2, 3>> points_data(total_vertices, {float2(-1), float2(-2), float2(-3)});
  potrace_curve_points(
      curves_list, curve_offsets, extra_point_offsets, curve_type_indices, points_data);
  // std::cout << "points_data: " << points_data.as_span() << ";\n";

  potrace_to_bezier_or_poly_curves(curves_list,
                                   curve_offsets,
                                   extra_point_offsets,
                                   curve_type_indices,
                                   curve_type_data,
                                   points_data);
  transform_points(resolution, min_point, max_point, points_data);

  potrace_state_free(potrace_result);

  Curves *curves_id = bke::curves_new_nomain(total_vertices, curve_offsets.size());
  bke::CurvesGeometry &curves = curves_id->geometry.wrap();

  curves.cyclic_for_write().fill(true);

  MutableSpan<int> dst_curve_offsets = curves.offsets_for_write();
  threading::parallel_for(curve_offsets.index_range(), 4096, [&](const IndexRange range) {
    for (const int curve_i : range) {
      dst_curve_offsets[curve_i] = offset_data[curve_i] + extra_curve_offset_data[curve_i];
    }
  });

  parallel_transform(curve_type_data.as_span(),
                     4096,
                     curves.curve_types_for_write(),
                     [&](const potrace::CurveType type) {
                       return type == potrace::CurveType::PolySpline ? CURVE_TYPE_POLY :
                                                                       CURVE_TYPE_BEZIER;
                     });

  // curves.curve_types_for_write().fill(CURVE_TYPE_BEZIER);

  MutableSpan<int8_t> handle_types_left = curves.handle_types_left_for_write();
  MutableSpan<int8_t> handle_types_right = curves.handle_types_right_for_write();
  threading::parallel_for(
      curves_list.index_range(),
      4096,
      [&](const IndexRange range) {
        for (const int curve_i : range) {
          switch (curve_type_data[curve_i]) {
            case potrace::CurveType::PolySpline:
              break;
            case potrace::CurveType::BezierSpline:
            case potrace::CurveType::FirstPolySegment:
            case potrace::CurveType::FirstBezierSegment: {
              const IndexRange finall_curve_range = range_sum(curve_offsets[curve_i],
                                                              extra_point_offsets[curve_i]);
              handle_types_left.slice(finall_curve_range).fill(BEZIER_HANDLE_FREE);
              handle_types_right.slice(finall_curve_range).fill(BEZIER_HANDLE_FREE);
              break;
            }
            default: {
              BLI_assert_unreachable();
            }
          }
        }
      },
      threading::accumulated_task_sizes([&](const IndexRange range) {
        return curve_offsets[range].size() + extra_point_offsets[range].size();
      }));

  curves.positions_for_write().fill(float3(0.0f, 0.0f, -1.0f));
  curves.handle_positions_left_for_write().fill(float3(0.0f, 0.0f, -2.0f));
  curves.handle_positions_right_for_write().fill(float3(0.0f, 0.0f, -3.0f));

  parallel_transform(
      points_data.as_span(),
      4096,
      curves.handle_positions_left_for_write(),
      [](const std::array<float2, 3> &src_point) { return float3(src_point[0], 0.0f); });

  parallel_transform(
      points_data.as_span(),
      4096,
      curves.positions_for_write(),
      [](const std::array<float2, 3> &src_point) { return float3(src_point[1], 0.0f); });

  parallel_transform(
      points_data.as_span(),
      4096,
      curves.handle_positions_right_for_write(),
      [](const std::array<float2, 3> &src_point) { return float3(src_point[2], 0.0f); });

  if (!parent_index_data.is_empty()) {
    bke::SpanAttributeWriter<int> parent_attribute =
        curves.attributes_for_write().lookup_or_add_for_write_only_span<int>(
            parent_index_id, bke::AttrDomain::Curve);
    array_utils::copy(parent_index_data.as_span(), parent_attribute.span);
  }

  curves.update_curve_types();
  curves.tag_topology_changed();
  curves.tag_positions_changed();

  return curves_id;
}

}  // namespace blender::geometry

#endif
