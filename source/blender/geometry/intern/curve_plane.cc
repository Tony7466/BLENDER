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

static void potrace_curves_captuer_parent_index(const Span<const potrace_path_t *> src_curves,
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

template<typename T> std::ostream &operator<<(std::ostream &stream, const Span<T> span)
{
  for (const int64_t i : span.index_range()) {
    stream << span[i] << (span.size() - 1 == i ? "" : ", ");
  }
  return stream;
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
  if (a_segment_type == SegmentType::Bezier && b_segment_type == SegmentType::Poly) {
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

/**
 * Offsets are slices of curve with segments of the same type.
 */
static void curve_segment_offsets(const Span<int> segment_types, MutableSpan<int> indices)
{
  const IndexRange range = segment_types.index_range().drop_back(1);
  indices.last() = -1;
  std::copy_if(range.begin(), range.end(), indices.begin(), [&](const int64_t i) {
    return extra_bezier_point_between(SegmentType(segment_types[i]),
                                      SegmentType(segment_types[i + 1]));
  });
  if (extra_bezier_point_between(SegmentType(segment_types.last()),
                                 SegmentType(segment_types.first())))
  {
    BLI_assert(indices.last() == -1);
    indices.last() = segment_types.size();
  }
}

}  // namespace potrace

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

static void vector_curve_type_check(const Span<const potrace_path_t *> src_curves,
                                    Array<potrace::CurveType> &r_curve_types,
                                    Array<int> &r_type_switch_offsets,
                                    Array<int> &r_type_switch_indices)
{
  BLI_assert(!src_curves.is_empty());

  r_type_switch_offsets.reinitialize(src_curves.size() + 1);
  threading::parallel_for(src_curves.index_range(), 4096, [&](const IndexRange range) {
    for (const int curve_i : range) {
      const Span<int> segment_types(src_curves[curve_i]->curve.tag, src_curves[curve_i]->curve.n);
      r_type_switch_offsets[curve_i] = potrace::extra_points_in_curve(segment_types);
    }
  });

  const OffsetIndices<int> type_offsets = offset_indices::accumulate_counts_to_offsets(
      r_type_switch_offsets);
  if (type_offsets.total_size() == 0) {
    r_type_switch_offsets = {};
    const int type_of_all_curves = *src_curves.first()->curve.tag;
    r_curve_types = {curve_type({type_of_all_curves}, 0)};
    return;
  }

  r_curve_types.reinitialize(src_curves.size());
  threading::parallel_for(src_curves.index_range(), 4096, [&](const IndexRange range) {
    for (const int curve_i : range) {
      const Span<int> segment_types(src_curves[curve_i]->curve.tag, src_curves[curve_i]->curve.n);
      r_curve_types[curve_i] = curve_type(segment_types, type_offsets[curve_i].size());
    }
  });

  r_type_switch_indices.reinitialize(type_offsets.total_size());
  threading::parallel_for(
      src_curves.index_range(),
      4096,
      [&](const IndexRange range) {
        for (const int curve_i : range) {
          const Span<int> segment_types(src_curves[curve_i]->curve.tag,
                                        src_curves[curve_i]->curve.n);
          MutableSpan<int> curve_type_switch_indices =
              r_type_switch_indices.as_mutable_span().slice(type_offsets[curve_i]);
          potrace::curve_segment_offsets(segment_types, curve_type_switch_indices);
        }
      },
      threading::accumulated_task_sizes(
          [&](const IndexRange range) { return type_offsets[range].size(); }));
}

static void potrace_curves_to_vector_curve(const Span<const potrace_path_t *> src_curves,
                                           Array<std::array<float2, 3>> &r_vector_data,
                                           Array<int> &r_offsets)
{
  BLI_assert(!src_curves.is_empty());
  r_offsets.reinitialize(src_curves.size() + 1);
  threading::parallel_for(src_curves.index_range(), 4096, [&](const IndexRange range) {
    for (const int curve_i : range) {
      r_offsets[curve_i] = src_curves[curve_i]->curve.n;
    }
  });
  const OffsetIndices<int> offsets = offset_indices::accumulate_counts_to_offsets(r_offsets);

  r_vector_data.reinitialize(offsets.total_size());
  threading::parallel_for(
      src_curves.index_range(),
      4096,
      [&](const IndexRange range) {
        for (const int curve_i : range) {
          const IndexRange curve_points = offsets[curve_i];
          MutableSpan<std::array<float2, 3>> points = r_vector_data.as_mutable_span().slice(
              curve_points);
          const Span<int> src_point_types(src_curves[curve_i]->curve.tag,
                                          src_curves[curve_i]->curve.n);
          for (const int64_t i : curve_points.index_range()) {
            points[i][0] = to_float2(src_curves[curve_i]->curve.c[i][0]);
            points[i][1] = to_float2(src_curves[curve_i]->curve.c[i][1]);
            points[i][2] = to_float2(src_curves[curve_i]->curve.c[i][2]);
            // std::cout << "{" << points[i][0] << ", " << points[i][1] << ", " << points[i][2] <<
            // ", Type: " << (src_point_types[i] == POTRACE_CURVETO ? "Curve" : "Poly") << "}, ";
          }
          // std::cout << "\n";

          /* Initialize unused polyline point by other point the treat this as handle. */
          // for (const int i : curve_points.index_range()) {
          // BLI_assert(ELEM(src_point_types[i], POTRACE_CURVETO, POTRACE_CORNER));
          // if (src_point_types[i] == POTRACE_CURVETO) {
          // std::swap(points[i][1], points[i][2]);
          // std::swap(points[i][0], points[i][2]);
          // points[i][0] = points[i][1];
          // points[i][1] = points[i][2];
          // std::swap(points[i][2], points[i][1]);
          // }
          // }
        }
      },
      threading::accumulated_task_sizes(
          [&](const IndexRange range) { return offsets[range].size(); }));
}

static void vector_curves_as_bezier(const OffsetIndices<int> offsets,
                                    MutableSpan<std::array<float2, 3>> vector_data)
{
  threading::parallel_for(
      offsets.index_range(),
      4096,
      [&](const IndexRange range) {
        for (const int curve_i : range) {
          const IndexRange curve_points = offsets[curve_i];
          MutableSpan<std::array<float2, 3>> points = vector_data.slice(curve_points);
          for (const int i : curve_points.index_range()) {
            std::swap(points[i][1], points[i][2]);
            std::swap(points[i][0], points[i][2]);
          }
          /* Make it so each triplet will contain left right and control point positions. */
          const float2 first_handle = points.first()[2];
          for (const int i : curve_points.index_range().drop_back(1)) {
            const int r_i = curve_points.index_range().last(i);
            points[i][2] = points[i + 1][2];
          }
          points.last()[2] = first_handle;
        }
      },
      threading::accumulated_task_sizes(
          [&](const IndexRange range) { return offsets[range].size(); }));
}

static void vector_curves_as_bezier_poly(const OffsetIndices<int> offsets,
                                         MutableSpan<std::array<float2, 3>> vector_data)
{
  threading::parallel_for(
      offsets.index_range(),
      4096,
      [&](const IndexRange range) {
        for (const int curve_i : range) {
          const IndexRange curve_points = offsets[curve_i];
          MutableSpan<std::array<float2, 3>> points = vector_data.slice(curve_points);
          for (const int i : curve_points.index_range()) {
            std::swap(points[i][1], points[i][2]);
            std::swap(points[i][0], points[i][2]);
          }
          /* Make it so each triplet will contain left right and control point positions. */
          const float2 first_handle = points.first()[2];
          for (const int i : curve_points.index_range().drop_back(1)) {
            const int r_i = curve_points.index_range().last(i);
            points[i][2] = points[i + 1][2];
          }
          points.last()[2] = first_handle;
        }
      },
      threading::accumulated_task_sizes(
          [&](const IndexRange range) { return offsets[range].size(); }));
}

static void vector_curves_as_bezier_or_poly(const OffsetIndices<int> offsets,
                                            const OffsetIndices<int> extra_points,
                                            const Span<potrace::CurveType> types,
                                            const Span<int> extra_curve_points_indices,
                                            MutableSpan<std::array<float2, 3>> vector_data)
{
  threading::parallel_for(
      offsets.index_range(),
      4096,
      [&](const IndexRange range) {
        for (const int curve_i : range) {
          const IndexRange curve_points = offsets[curve_i];
          MutableSpan<std::array<float2, 3>> points = vector_data.slice(curve_points);

          const potrace::CurveType type = types[curve_i];
          if (extra_points[curve_i].is_empty()) {
            BLI_assert(
                ELEM(type, potrace::CurveType::BezierSpline, potrace::CurveType::PolySpline));
            if (type == potrace::CurveType::BezierSpline) {
              vector_curves_as_bezier(Span{0, int(points.size())}, points);
            }
            continue;
          }

          const Span<int> extra_point_indices = extra_curve_points_indices.slice(
              extra_points[curve_i]);
          const IndexRange first_segments_same_type = IndexRange::from_begin_end(
              0, extra_point_indices[0]);
          if (!first_segments_same_type.is_empty()) {
            if (type == potrace::CurveType::FirstPolySegment) {
              vector_curves_as_bezier_poly(Span{0, int(first_segments_same_type.size())}, points);
            }
            else {
              vector_curves_as_bezier(Span{0, int(first_segments_same_type.size())}, points);
            }
          }

          for (const int type_p : extra_point_indices.index_range().drop_back(1)) {
            const potrace::SegmentType current_type =
                ((type == potrace::CurveType::FirstPolySegment) ^ (type_p % 2)) ?
                    potrace::SegmentType::Poly :
                    potrace::SegmentType::Bezier;
            const IndexRange segments_same_type = IndexRange::from_begin_end(
                extra_point_indices[type_p], extra_point_indices[type_p + 1]);
            const int extra_points_shift = type_p + segments_same_type.first();

            if (current_type == potrace::SegmentType::Poly) {
              vector_curves_as_bezier_poly(
                  Span{extra_points_shift, int(segments_same_type.size())}, points);
            }
            else {
              vector_curves_as_bezier(Span{extra_points_shift, int(segments_same_type.size())},
                                      points);
            }
          }

          const IndexRange last_segments_same_type = IndexRange::from_begin_end(
              extra_point_indices.last(), curve_points.size());
          if (!last_segments_same_type.is_empty()) {
            if (type == potrace::CurveType::FirstPolySegment) {
              vector_curves_as_bezier_poly(
                  Span{extra_point_indices.last(), int(last_segments_same_type.size())}, points);
            }
            else {
              vector_curves_as_bezier(
                  Span{extra_point_indices.last(), int(last_segments_same_type.size())}, points);
            }
          }
        }
      },
      threading::accumulated_task_sizes(
          [&](const IndexRange range) { return offsets[range].size(); }));
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

static void vector_curve_to_poly_or_bezier(const OffsetIndices<int> curves,
                                           const OffsetIndices<int> extra_points,
                                           const Span<potrace::CurveType> types,
                                           const Span<int> extra_curve_points_indices,
                                           const Span<std::array<float2, 3>> src_points,
                                           MutableSpan<float3> positions,
                                           MutableSpan<float3> left_handle,
                                           MutableSpan<float3> right_handle)
{
  left_handle.fill({});
  right_handle.fill({});
  // parallel_transform(vectors, 4096, left_handle,
  //     [](const std::array<float2, 3> &src_point) { return float3(src_point[0], 0.0f); });

  threading::parallel_for(
      curves.index_range(),
      4096,
      [&](const IndexRange range) {
        for (const int curve_i : range) {
          const Span<std::array<float2, 3>> curve_points = src_points.slice(
              curves[curve_i].first(), curves[curve_i].size());
          MutableSpan<float3> curve_positions = positions.slice(
              curves[curve_i].first() + extra_points[curve_i].first(),
              curves[curve_i].size() + extra_points[curve_i].size());
          // MutableSpan<float3> curve_left_handle = left_handle.slice(curves[curve_i].first() +
          // extra_points[curve_i].first(), curves[curve_i].size() + extra_points[curve_i].size());
          // MutableSpan<float3> curve_right_handle = right_handle.slice(curves[curve_i].first() +
          // extra_points[curve_i].first(), curves[curve_i].size() + extra_points[curve_i].size());

          const Span<int> extra_point_indices = extra_curve_points_indices.slice(
              extra_points[curve_i]);

          {
            const IndexRange first_segments_same_type = IndexRange::from_begin_end(
                0, extra_point_indices[0]);
            for (const int64_t i : first_segments_same_type) {
              std::cout << "first_segments_same_type >> " << i << ";\n";
              curve_positions[i] = float3(src_points[i][0], 0.0f);
            }
          }

          std::cout << extra_point_indices << ";\n";

          for (const int64_t type_i : extra_point_indices.index_range().drop_back(1)) {
            const IndexRange segments_same_type = IndexRange::from_begin_end(
                extra_point_indices[type_i], extra_point_indices[type_i + 1]);
            const int64_t extra_points_shift = type_i;

            for (const int64_t i : segments_same_type) {
              std::cout << "segments_same_type >> " << (extra_points_shift + i) << ";\n";
              curve_positions[extra_points_shift + i] = float3(src_points[i][0], 0.0f);
            }
          }

          {
            const IndexRange last_segments_same_type = IndexRange::from_begin_end(
                extra_point_indices.last(), curve_points.size());
            const int64_t extra_points_shift = extra_point_indices.size();
            for (const int64_t i : last_segments_same_type) {
              std::cout << "last_segments_same_type >> " << i << ";\n";
              curve_positions[extra_points_shift + i] = float3(src_points[i][0], 0.0f);
            }
          }
        }
      },
      threading::accumulated_task_sizes([&](const IndexRange range) {
        return curves[range].size() + extra_points[range].size();
      }));

  /*
    parallel_transform(vectors, 4096, positions,
        [](const std::array<float2, 3> &src_point) { return float3(src_point[1], 0.0f); });
  */

  // parallel_transform(vectors, 4096, right_handle,
  //     [](const std::array<float2, 3> &src_point) { return float3(src_point[2], 0.0f); });
}

static void vector_curve_to_poly(const Span<std::array<float2, 3>> src_points,
                                 MutableSpan<float3> positions)
{
  parallel_transform(src_points, 4096, positions, [](const std::array<float2, 3> &src_point) {
    return float3(src_point[1], 0.0f);
  });
}

static void vector_curve_to_bezier(const Span<std::array<float2, 3>> src_points,
                                   MutableSpan<float3> positions,
                                   MutableSpan<float3> left_handle,
                                   MutableSpan<float3> right_handle)
{
  parallel_transform(src_points, 4096, left_handle, [](const std::array<float2, 3> &src_point) {
    return float3(src_point[0], 0.0f);
  });
  parallel_transform(src_points, 4096, positions, [](const std::array<float2, 3> &src_point) {
    return float3(src_point[1], 0.0f);
  });
  parallel_transform(src_points, 4096, right_handle, [](const std::array<float2, 3> &src_point) {
    return float3(src_point[2], 0.0f);
  });
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

  Array<std::array<float2, 3>> vector_data;
  Array<int> offsets_data;
  potrace_curves_to_vector_curve(curves_list, vector_data, offsets_data);
  const OffsetIndices<int> offsets(offsets_data);

  Array<int> parent_index_data;
  if (parent_index_id) {
    potrace_curves_captuer_parent_index(curves_list, parent_index_data);
  }

  Array<potrace::CurveType> curve_types;
  Array<int> extra_curve_points_offsets;
  Array<int> extra_curve_points_indices;
  vector_curve_type_check(
      curves_list, curve_types, extra_curve_points_offsets, extra_curve_points_indices);
  const OffsetIndices<int> extra_points_offsets(extra_curve_points_offsets);

  potrace_state_free(potrace_result);

  if (extra_curve_points_offsets.is_empty()) {
    BLI_assert(curve_types.size() == 1);
    if (curve_types.first() == potrace::CurveType::BezierSpline) {
      vector_curves_as_bezier(offsets, vector_data);
    }
  }
  else {
    vector_curves_as_bezier_or_poly(
        offsets, extra_points_offsets, curve_types, extra_curve_points_indices, vector_data);
  }

  BLI_assert(extra_points_offsets.total_size() >= 0);
  Curves *curves_id = bke::curves_new_nomain(
      offsets.total_size() + extra_points_offsets.total_size(), offsets.size());
  std::cout << "Curves num: " << offsets.size()
            << ";\nPoints num: " << offsets.total_size() + extra_points_offsets.total_size()
            << ";\n";
  bke::CurvesGeometry &curves = curves_id->geometry.wrap();

  curves.cyclic_for_write().fill(true);

  MutableSpan<int> curve_offsets = curves.offsets_for_write();
  if (extra_curve_points_offsets.is_empty()) {
    array_utils::copy(offsets_data.as_span(), curve_offsets);
    std::cout << "curve_offsets range:"
              << IndexRange(offsets_data.first(), offsets_data.last() - offsets_data.first())
              << ";\n";
  }
  else {
    threading::parallel_for(curve_offsets.index_range(), 4096, [&](const IndexRange range) {
      for (const int curve_i : range) {
        curve_offsets[curve_i] = offsets_data[curve_i] + extra_curve_points_offsets[curve_i];
        std::cout << "curve_offsets[curve_i]: " << curve_offsets[curve_i] << "\n";
      }
    });
  }

  if (extra_curve_points_offsets.is_empty()) {
    BLI_assert(curve_types.size() == 1);
    switch (curve_types.first()) {
      case potrace::CurveType::PolySpline: {
        curves.fill_curve_types(CURVE_TYPE_POLY);
        break;
      }
      case potrace::CurveType::BezierSpline: {
        curves.fill_curve_types(CURVE_TYPE_BEZIER);
        curves.handle_types_left_for_write().fill(BEZIER_HANDLE_FREE);
        curves.handle_types_right_for_write().fill(BEZIER_HANDLE_FREE);
        break;
      }
      default: {
        BLI_assert_unreachable();
      }
    }
  }
  else {
    MutableSpan<int8_t> curves_type = curves.curve_types_for_write();
    MutableSpan<int8_t> handle_types_left = curves.handle_types_left_for_write();
    MutableSpan<int8_t> handle_types_right = curves.handle_types_right_for_write();
    threading::parallel_for(
        curves_list.index_range(),
        4096,
        [&](const IndexRange range) {
          for (const int curve_i : range) {
            switch (curve_types[curve_i]) {
              case potrace::CurveType::PolySpline: {
                curves_type[curve_i] = CURVE_TYPE_POLY;
                break;
              }
              case potrace::CurveType::BezierSpline:
              case potrace::CurveType::FirstPolySegment:
              case potrace::CurveType::FirstBezierSegment: {
                curves_type[curve_i] = CURVE_TYPE_BEZIER;
                const IndexRange finall_curve_range = IndexRange(
                    offsets[curve_i].first() + extra_points_offsets[curve_i].first(),
                    offsets[curve_i].size() + extra_points_offsets[curve_i].size());
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
        threading::accumulated_task_sizes(
            [&](const IndexRange range) { return offsets[range].size(); }));
  }

  const float2 resolution_factor = float2(1.0f) / float2(resolution) * (max_point - min_point);
  parallel_transform(
      vector_data.as_span().cast<float2>(),
      4096,
      vector_data.as_mutable_span().cast<float2>(),
      [&](const float2 src_point) { return min_point + src_point * resolution_factor; });

  if (extra_curve_points_offsets.is_empty()) {
    BLI_assert(curve_types.size() == 1);
    switch (curve_types.first()) {
      case potrace::CurveType::PolySpline: {
        vector_curve_to_poly(vector_data, curves.positions_for_write());
        break;
      }
      case potrace::CurveType::BezierSpline: {
        vector_curve_to_bezier(vector_data,
                               curves.positions_for_write(),
                               curves.handle_positions_left_for_write(),
                               curves.handle_positions_right_for_write());
        break;
      }
      default: {
        BLI_assert_unreachable();
      }
    }
  }
  else {
    curves.positions_for_write().fill(float3(0.0f, 0.0f, -1.0f));
    std::cout << "vector_curve_to_poly_or_bezier;\n";
    vector_curve_to_poly_or_bezier(offsets,
                                   extra_points_offsets,
                                   curve_types,
                                   extra_curve_points_indices,
                                   vector_data,
                                   curves.positions_for_write(),
                                   curves.handle_positions_left_for_write(),
                                   curves.handle_positions_right_for_write());
  }

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
