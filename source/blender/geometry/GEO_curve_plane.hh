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

namespace blender::geometry::potrace {

struct Params {
  /* Original resolution, not aligned. */
  int2 resolution;

  static constexpr const float max_smooth_threshold = 4.0f / 3.0f;
  float smooth_threshold = 1.0f;

  float optimization_tolerance = 0.2f;
};

int2 aligned_resolution(int2 resolution);

potrace_state_t *image_from_lines(Params params,
                                  FunctionRef<void(int64_t line_i,
                                                   int64_t line_offset,
                                                   int64_t index_in_line,
                                                   int64_t length,
                                                   int8_t *r_segments)> func);

template<typename Func> inline potrace_state_t *image_from_bytes(Params params, Func func)
{
  return image_from_lines(params,
                          [&](int64_t line_i,
                              int64_t line_offset,
                              int64_t index_in_line,
                              int64_t length,
                              int8_t *r_segments) {
                            BLI_assert(length % 8 == 0);
                            BLI_assert(line_offset % 8 == 0);
                            BLI_assert(index_in_line % 8 == 0);
                            const int64_t bytes_num = length >> 3;
                            const int64_t bytes_line_offset = line_offset >> 3;
                            const int64_t bytes_index_in_line = index_in_line >> 3;
                            for (int64_t byte_iter = 0; byte_iter < bytes_num; byte_iter++) {
                              *r_segments = func(
                                  line_i, bytes_line_offset, bytes_index_in_line + byte_iter);
                              r_segments++;
                            }
                          });
}

template<typename Func> inline potrace_state_t *image_for_predicate(Params params, Func func)
{
  return image_from_bytes(
      params, [&](int64_t line_i, int64_t line_offset, int64_t index_in_line) -> int8_t {
        const int64_t pixels_line_offset = line_offset << 3;
        const int64_t pixels_index_in_line = index_in_line << 3;
        int8_t byte = {};
        for (int64_t pixel_iter = 0; pixel_iter < 8; pixel_iter++) {
          byte <<= 1;
          byte |= int8_t(func(line_i, pixels_line_offset, pixels_index_in_line + pixel_iter));
        }
        return byte;
      });
}

void free_image(potrace_state_t *image);

Curves *image_to_curve(const potrace_state_t *image, const bke::AttributeIDRef &uv_map_id);

float4x4 to_plane(int2 resolution, float2 min_point, float2 max_point);

}  // namespace blender::geometry::potrace

#endif
