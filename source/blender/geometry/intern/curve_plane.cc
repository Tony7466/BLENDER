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

static void potrace_gather_curves(const potrace_state_t *potrace,
                                  Vector<const potrace_path_t *> &r_list)
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

static float3 to_float3(const potrace_dpoint_s &point)
{
  return float3(point.x, point.y, 0.0f);
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

static const constexpr int bezier_prev_right_handle = 0;
static const constexpr int bezier_left_handle = 1;
static const constexpr int bezier_control = 2;

static const constexpr int poly_control = 1;
static const constexpr int poly_right_handle = 2;
static const constexpr int poly_next_left_handle = 2;

}  // namespace potrace

static void potrace_curve_count_type_switch(const Span<const potrace_path_t *> src_curves,
                                            MutableSpan<int> curve_type_switch_offset_num)
{
  static constexpr const int plus_to_store_offsets = 2;
  threading::parallel_for(
      src_curves.index_range(),
      4096,
      [&](const IndexRange range) {
        for (const int curve_i : range) {
          const Span<int> segment_types(src_curves[curve_i]->curve.tag,
                                        src_curves[curve_i]->curve.n);
          const IndexRange range = segment_types.index_range().drop_back(1);
          const int different_types_num = std::count_if(
              range.begin(), range.end(), [&](const int64_t i) {
                return potrace::SegmentType(segment_types[i]) !=
                       potrace::SegmentType(segment_types[i + 1]);
              });
          curve_type_switch_offset_num[curve_i] = different_types_num + plus_to_store_offsets;
        }
      },
      threading::individual_task_sizes(
          [&](const int64_t curve_i) { return src_curves[curve_i]->curve.n; }));
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
  indices[offset] = int(segment_types.size()) - last_i;
}

static void potrace_curve_type_switch_sizes(const Span<const potrace_path_t *> src_curves,
                                            const OffsetIndices<int> curve_type_switch_offset,
                                            MutableSpan<int> src_type_switch_sizes_num)
{
  threading::parallel_for(
      src_curves.index_range(),
      4096,
      [&](const IndexRange range) {
        for (const int curve_i : range) {
          const Span<int> segment_types(src_curves[curve_i]->curve.tag,
                                        src_curves[curve_i]->curve.n);
          MutableSpan<int> type_switch_sizes_num = src_type_switch_sizes_num.slice(
              curve_type_switch_offset[curve_i]);
          type_group_sizes(segment_types, type_switch_sizes_num);
          // std::cout << "potrace_curve_type_switch_sizes: " << segment_types << ";\n";
          // std::cout << "potrace_curve_type_switch_sizes: " << type_switch_sizes_num << ";\n";
        }
      },
      threading::accumulated_task_sizes(
          [&](const IndexRange range) { return curve_type_switch_offset[range].size(); }));
}

static void type_switch_extra_sizes(const Span<int> segment_types, MutableSpan<int> indices)
{
  int offset = 0;
  int last_i = 0;
  for (const int i : segment_types.index_range().drop_back(1)) {
    const potrace::SegmentType current = potrace::SegmentType(segment_types[i]);
    const potrace::SegmentType next = potrace::SegmentType(segment_types[i + 1]);
    if (current != next) {
      indices[offset] = int(potrace::extra_bezier_point_between(current, next));
      offset++;
      last_i = i;
    }
  }
  indices[offset] = int(potrace::extra_bezier_point_between(
      potrace::SegmentType(segment_types.last()), potrace::SegmentType(segment_types.first())));
}

static void potrace_curve_type_switch_extra_sizes(
    const Span<const potrace_path_t *> src_curves,
    const OffsetIndices<int> curve_type_switch_offset,
    MutableSpan<int> extra_type_switch_sizes_num)
{
  threading::parallel_for(
      src_curves.index_range(),
      4096,
      [&](const IndexRange range) {
        for (const int curve_i : range) {
          const Span<int> segment_types(src_curves[curve_i]->curve.tag,
                                        src_curves[curve_i]->curve.n);
          MutableSpan<int> type_switch_sizes_num = extra_type_switch_sizes_num.slice(
              curve_type_switch_offset[curve_i]);
          type_switch_extra_sizes(segment_types, type_switch_sizes_num);
          // std::cout << "potrace_curve_type_switch_extra_sizes: " << segment_types << ";\n";
          // std::cout << "potrace_curve_type_switch_extra_sizes: " << type_switch_sizes_num <<
          // ";\n";
        }
      },
      threading::accumulated_task_sizes(
          [&](const IndexRange range) { return curve_type_switch_offset[range].size(); }));
}

[[nodiscard]] static int64_t accumulate_groups_total(
    const OffsetIndices<int> curve_type_switch_offset, MutableSpan<int> src_type_switch_sizes_num)
{
  return threading::parallel_reduce<int64_t>(
      curve_type_switch_offset.index_range(),
      4096,
      0,
      [&](const IndexRange range, int64_t total_size) -> int64_t {
        for (const int64_t curve_i : range) {
          const IndexRange curve_segments = curve_type_switch_offset[curve_i];
          total_size += offset_indices::accumulate_counts_to_offsets(
                            src_type_switch_sizes_num.slice(curve_segments))
                            .total_size();
        }
        return total_size;
      },
      std::plus<int64_t>());
}

static void copy_poly_curve(const potrace_path_t &potrace_curve, MutableSpan<float3> positions)
{
  const Span<potrace_dpoint_t[3]> src_curve(potrace_curve.curve.c, potrace_curve.curve.n);
  parallel_transform(src_curve, 4096, positions, [&](const potrace_dpoint_t(&point)[3]) -> float3 {
    return to_float3(point[1]);
  });
}

static IndexRange sum(const IndexRange a, const IndexRange b)
{
  return IndexRange(a.start() + b.start(), a.size() + b.size());
}

static void bezier_copy_as_bezier_positions(const Span<potrace_dpoint_t[3]> src,
                                            MutableSpan<float3> dst)
{
  parallel_transform(src, 4096, dst, [&](const potrace_dpoint_t(&point)[3]) -> float3 {
    return to_float3(point[potrace::bezier_control]);
  });
}

static void bezier_copy_as_bezier_left_handles(const Span<potrace_dpoint_t[3]> src,
                                               MutableSpan<float3> dst)
{
  parallel_transform(src, 4096, dst, [&](const potrace_dpoint_t(&point)[3]) -> float3 {
    return to_float3(point[potrace::bezier_left_handle]);
  });
}

static void bezier_copy_as_bezier_right_handles(const Span<potrace_dpoint_t[3]> src,
                                                const potrace::SegmentType next,
                                                const potrace_dpoint_t (&next_point)[3],
                                                MutableSpan<float3> dst)
{
  parallel_transform(
      src.drop_front(1), 4096, dst.drop_back(1), [&](const potrace_dpoint_t(&point)[3]) -> float3 {
        return to_float3(point[potrace::bezier_prev_right_handle]);
      });
  switch (next) {
    case potrace::SegmentType::Bezier:
      dst.last() = to_float3(next_point[potrace::bezier_prev_right_handle]);
      break;
    case potrace::SegmentType::Poly:
      dst.last() = float3(math::midpoint(to_float2(next_point[potrace::poly_control]),
                                         to_float2(src.last()[potrace::bezier_control])),
                          0.0f);
      break;
  }
}

static void poly_copy_as_bezier_positions(const Span<potrace_dpoint_t[3]> src,
                                          const potrace::SegmentType next,
                                          MutableSpan<float3> dst)
{
  const int extra_point = next == potrace::SegmentType::Bezier ? 1 : 0;
  parallel_transform(
      src, 4096, dst.drop_back(extra_point), [&](const potrace_dpoint_t(&point)[3]) -> float3 {
        return to_float3(point[potrace::poly_control]);
      });
  if (next == potrace::SegmentType::Bezier) {
    dst.last() = to_float3(src.last()[potrace::bezier_control]);
  }
}

static void poly_copy_as_bezier_left_handles(const Span<potrace_dpoint_t[3]> src,
                                             const potrace::SegmentType prev,
                                             const potrace::SegmentType next,
                                             const potrace_dpoint_t (&prev_point)[3],
                                             MutableSpan<float3> dst)
{
  const int extra_point = next == potrace::SegmentType::Bezier ? 1 : 0;
  switch (prev) {
    case potrace::SegmentType::Poly:
      dst.first() = to_float3(prev_point[potrace::poly_right_handle]);
      break;
    case potrace::SegmentType::Bezier:
      dst.first() = float3(math::midpoint(to_float2(prev_point[potrace::bezier_control]),
                                          to_float2(src.first()[potrace::poly_control])),
                           0.0f);
      break;
  }
  parallel_transform(src.drop_back(1),
                     4096,
                     dst.drop_front(1).drop_back(extra_point),
                     [&](const potrace_dpoint_t(&point)[3]) -> float3 {
                       return to_float3(point[potrace::poly_next_left_handle]);
                     });
  if (next == potrace::SegmentType::Bezier) {
    dst.last() = float3(math::midpoint(to_float2(src.last()[potrace::poly_control]),
                                       to_float2(src.last()[potrace::poly_next_left_handle])),
                        0.0f);
  }
}

static void poly_copy_as_bezier_right_handles(const Span<potrace_dpoint_t[3]> src,
                                              const potrace::SegmentType next,
                                              const potrace_dpoint_t (&next_point)[3],
                                              MutableSpan<float3> dst)
{
  const int extra_point = next == potrace::SegmentType::Bezier ? 1 : 0;
  parallel_transform(src.drop_back(extra_point),
                     4096,
                     dst.drop_back(extra_point + extra_point),
                     [&](const potrace_dpoint_t(&point)[3]) -> float3 {
                       return to_float3(point[potrace::poly_right_handle]);
                     });
  switch (next) {
    case potrace::SegmentType::Poly:
      dst.last() = to_float3(src.last()[potrace::poly_right_handle]);
      break;
    case potrace::SegmentType::Bezier:
      dst.last(1) = float3(math::midpoint(to_float2(src.last()[potrace::poly_control]),
                                          to_float2(src.last()[potrace::poly_right_handle])),
                           0.0f);
      dst.last(0) = to_float3(next_point[potrace::bezier_prev_right_handle]);
      break;
  }
}

static void potrace_fill_bezier_or_poly_curve(
    const potrace_path_t &potrace_curve,
    const OffsetIndices<int> potrace_curve_segments_groups,
    const OffsetIndices<int> extra_curve_segments_group_points,
    MutableSpan<float3> positions,
    MutableSpan<float3> positions_left,
    MutableSpan<float3> positions_right)
{
  BLI_assert(potrace_curve_segments_groups.size() == extra_curve_segments_group_points.size());
  const Span<potrace_dpoint_t[3]> src_curve(potrace_curve.curve.c, potrace_curve.curve.n);
  const Span<int> segment_types(potrace_curve.curve.tag, potrace_curve.curve.n);

  threading::parallel_for(
      potrace_curve_segments_groups.index_range(),
      1024,
      [&](const IndexRange range) {
        for (const int segments_i : range) {
          const IndexRange src_segments = potrace_curve_segments_groups[segments_i];
          const IndexRange dst_segments = sum(src_segments,
                                              extra_curve_segments_group_points[segments_i]);

          const int next_i = math::mod_periodic<int64_t>(src_segments.last() + 1,
                                                         potrace_curve.curve.n);
          const potrace_dpoint_t(&next_point)[3] = src_curve[next_i];
          const potrace::SegmentType next_type = potrace::SegmentType(segment_types[next_i]);
          switch (potrace::SegmentType(segment_types[src_segments.first()])) {
            case potrace::SegmentType::Poly: {
              const int prev_i = math::mod_periodic<int64_t>(src_segments.first() - 1,
                                                             potrace_curve.curve.n);
              const potrace::SegmentType prev_type = potrace::SegmentType(segment_types[prev_i]);
              const potrace_dpoint_t(&prev_point)[3] = src_curve[prev_i];
              poly_copy_as_bezier_positions(
                  src_curve.slice(src_segments), next_type, positions.slice(dst_segments));
              poly_copy_as_bezier_left_handles(src_curve.slice(src_segments),
                                               prev_type,
                                               next_type,
                                               prev_point,
                                               positions_left.slice(dst_segments));
              poly_copy_as_bezier_right_handles(src_curve.slice(src_segments),
                                                next_type,
                                                next_point,
                                                positions_right.slice(dst_segments));
              break;
            }
            case potrace::SegmentType::Bezier:
              bezier_copy_as_bezier_positions(src_curve.slice(src_segments),
                                              positions.slice(dst_segments));
              bezier_copy_as_bezier_left_handles(src_curve.slice(src_segments),
                                                 positions_left.slice(dst_segments));
              bezier_copy_as_bezier_right_handles(src_curve.slice(src_segments),
                                                  next_type,
                                                  next_point,
                                                  positions_right.slice(dst_segments));
              break;
          }
        }
      },
      threading::accumulated_task_sizes(
          [&](const IndexRange range) { return potrace_curve_segments_groups[range].size(); }));
}

static void transform_points(const int2 resolution,
                             const float2 min_point,
                             const float2 max_point,
                             MutableSpan<float3> data)
{
  const float2 resolution_factor = float2(1.0f) / float2(resolution) * (max_point - min_point);
  parallel_transform(data.as_span(), 4096, data, [&](const float3 vector) {
    return float3(min_point + vector.xy() * resolution_factor, 0.0f);
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
  const OffsetIndices<int> curve_type_switch_offset = offset_indices::accumulate_counts_to_offsets(
      curve_type_switch_offset_num);

  Array<int> src_type_switch_sizes_num(curve_type_switch_offset.total_size());
  potrace_curve_type_switch_sizes(
      curves_list, curve_type_switch_offset, src_type_switch_sizes_num);
  const int64_t src_total_segments = accumulate_groups_total(curve_type_switch_offset,
                                                             src_type_switch_sizes_num);

  Array<int> extra_type_switch_sizes_num(curve_type_switch_offset.total_size());
  potrace_curve_type_switch_extra_sizes(
      curves_list, curve_type_switch_offset, extra_type_switch_sizes_num);
  const int64_t extra_total_segments = accumulate_groups_total(curve_type_switch_offset,
                                                               extra_type_switch_sizes_num);

  const int64_t total_vertices = src_total_segments + extra_total_segments;
  if (total_vertices == 0) {
    return std::nullopt;
  }

  Curves *curves_id = bke::curves_new_nomain(total_vertices, curves_list.size());
  bke::CurvesGeometry &curves = curves_id->geometry.wrap();
  curves.cyclic_for_write().fill(true);

  MutableSpan<int> dst_curve_offsets = curves.offsets_for_write();
  threading::parallel_for(curves_list.index_range(), 4096, [&](const IndexRange range) {
    for (const int curve_i : range) {
      const IndexRange curve_segments_groups = curve_type_switch_offset[curve_i];
      const OffsetIndices<int> potrace_curve_segments_groups =
          src_type_switch_sizes_num.as_span().slice(curve_segments_groups);
      const OffsetIndices<int> extra_curve_segments_group_points =
          extra_type_switch_sizes_num.as_span().slice(curve_segments_groups);
      dst_curve_offsets[curve_i] = potrace_curve_segments_groups.total_size() +
                                   extra_curve_segments_group_points.total_size();
      BLI_assert(potrace_curve_segments_groups.total_size() > 0);
      BLI_assert(extra_curve_segments_group_points.total_size() >= 0);
    }
  });
  const OffsetIndices<int> curve_offsets = offset_indices::accumulate_counts_to_offsets(
      dst_curve_offsets);

  IndexMaskMemory memory;
  const IndexMask poly_curves_mask = IndexMask::from_predicate(
      curves_list.index_range(), GrainSize(4096), memory, [&](const int curve_i) {
        const IndexRange curve_segments_groups = curve_type_switch_offset[curve_i];
        if (curve_segments_groups.size() > 2) {
          return false;
        }
        BLI_assert(curve_segments_groups.size() == 2);
        const Span<int> segment_types(curves_list[curve_i]->curve.tag,
                                      curves_list[curve_i]->curve.n);
        const potrace::SegmentType curve_type = potrace::SegmentType(segment_types.first());
        if (curve_type != potrace::SegmentType::Poly) {
          return false;
        }
        BLI_assert(std::all_of(segment_types.begin(), segment_types.end(), [&](const int type) {
          return type == segment_types.first();
        }));
        return true;
      });
  const IndexMask bezier_curves_mask = poly_curves_mask.complement(curves_list.index_range(),
                                                                   memory);

  curves.fill_curve_types(poly_curves_mask, CURVE_TYPE_POLY);
  curves.fill_curve_types(bezier_curves_mask, CURVE_TYPE_BEZIER);

  if (!bezier_curves_mask.is_empty()) {
    curves.handle_types_left_for_write().fill(BEZIER_HANDLE_FREE);
    curves.handle_types_right_for_write().fill(BEZIER_HANDLE_FREE);
  }

  MutableSpan<float3> positions = curves.positions_for_write();
  MutableSpan<float3> positions_left = curves.handle_positions_left_for_write();
  MutableSpan<float3> positions_right = curves.handle_positions_right_for_write();

  poly_curves_mask.foreach_index(GrainSize(4096), [&](const int curve_i) {
    copy_poly_curve(*curves_list[curve_i], positions.slice(curve_offsets[curve_i]));
  });

  bezier_curves_mask.foreach_index(GrainSize(4096), [&](const int curve_i) {
    const IndexRange curve_segments_groups = curve_type_switch_offset[curve_i];
    const OffsetIndices<int> potrace_curve_segments_groups =
        src_type_switch_sizes_num.as_span().slice(curve_segments_groups);
    const OffsetIndices<int> extra_curve_segments_group_points =
        extra_type_switch_sizes_num.as_span().slice(curve_segments_groups);
    potrace_fill_bezier_or_poly_curve(*curves_list[curve_i],
                                      potrace_curve_segments_groups,
                                      extra_curve_segments_group_points,
                                      positions.slice(curve_offsets[curve_i]),
                                      positions_left.slice(curve_offsets[curve_i]),
                                      positions_right.slice(curve_offsets[curve_i]));
  });

  transform_points(resolution, min_point, max_point, positions);
  transform_points(resolution, min_point, max_point, positions_left);
  transform_points(resolution, min_point, max_point, positions_right);

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
