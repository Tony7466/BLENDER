/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#ifdef WITH_POTRACE

#  include <cstring>

#  include "BKE_attribute.hh"
#  include "BKE_curves.hh"

#  include "BLI_function_ref.hh"
#  include "BLI_math_vector_types.hh"
#  include "BLI_span.hh"

struct potrace_state_s;
typedef struct potrace_state_s potrace_state_t;

namespace blender::geometry {

namespace potrace {

using LineSegment = int32_t;
inline constexpr const int segment_size = sizeof(LineSegment) * 8;

int2 fixed_resolution(int2 resolution);

potrace_state_t *image_from_line_segments(
    int2 resolution,
    FunctionRef<void(
        int64_t line_i, int64_t segments_start, int64_t segments_num, char *r_segments)> func);

template<typename Func>
inline potrace_state_t *image_from_int32_segments(int2 resolution, Func func)
{
  return image_from_line_segments(
      resolution,
      [&](int64_t line_i,
          int64_t segments_start,
          int64_t segments_num,
          char *__restrict r_segments) {
        for (int64_t segment_iter = 0; segment_iter < segments_num; segment_iter++) {
          const LineSegment segment = func(line_i, segments_start + segment_iter);
          std::memcpy(r_segments, &segment, sizeof(segment));
          r_segments += sizeof(segment);
        }
      });
}

template<typename Func> inline potrace_state_t *image_for_predicate(int2 resolution, Func func)
{
  return image_from_int32_segments(resolution,
                                   [&](int64_t line_i, int64_t segment_index) -> LineSegment {
                                     LineSegment segment = 0;
                                     segment_index *= segment_size;
                                     for (int64_t iter = 0; iter < segment_size; iter++) {
                                       segment <<= 1;
                                       segment |= LineSegment(func(line_i, segment_index + iter));
                                     }
                                     return segment;
                                   });
}

void free_image(potrace_state_t *image);

}  // namespace potrace

Curves *plane_to_curve(const potrace_state_t *image, const bke::AttributeIDRef &uv_map_id);

float4x4 transformation_potrace_to_plane(int2 resolution, float2 min_point, float2 max_point);

}  // namespace blender::geometry

#endif
