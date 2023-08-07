/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgreasepencil
 */

#include <limits>

#include "BKE_grease_pencil.hh"

#include "ED_grease_pencil.hh"

extern "C" {
#include "curve_fit_nd.h"
}

namespace blender::ed::greasepencil {

Array<float2> fit_curve_polyline_2d(Span<float2> points,
                                    const float error_threshold,
                                    const IndexMask &corner_mask)
{
  Array<int32_t> indices(corner_mask.size());
  corner_mask.to_indices(indices.as_mutable_span());
  uint *indicies_ptr = (corner_mask.size() > 0) ? reinterpret_cast<uint *>(indices.data()) :
                                                  nullptr;

  float *r_cubic_array;
  uint r_cubic_array_len;
  int error = curve_fit_cubic_to_points_fl(*points.data(),
                                           points.size(),
                                           2,
                                           error_threshold,
                                           CURVE_FIT_CALC_HIGH_QUALIY,
                                           indicies_ptr,
                                           indices.size(),
                                           &r_cubic_array,
                                           &r_cubic_array_len,
                                           nullptr,
                                           nullptr,
                                           nullptr);

  if (error != 0) {
    /* Some error occured. Return. */
    return {};
  }

  Span<float2> r_cubic_array_span(reinterpret_cast<float2 *>(r_cubic_array),
                                  r_cubic_array_len * 3);

  Array<float2> curve_positions(r_cubic_array_span);
  return curve_positions;
}

IndexMask polyline_detect_corners(Span<float2> points,
                                  const float radius_min,
                                  const float radius_max,
                                  const int64_t samples_max,
                                  const float angle_threshold,
                                  IndexMaskMemory &memory)
{
  uint *r_corners;
  uint r_corner_len;
  const int error = curve_fit_corners_detect_fl(*points.data(),
                                                points.size(),
                                                float2::type_length,
                                                radius_min,
                                                radius_max,
                                                samples_max,
                                                angle_threshold,
                                                &r_corners,
                                                &r_corner_len);
  if (error != 0) {
    /* Error occured, return. */
    return IndexMask();
  }
  BLI_assert(samples_max < std::numeric_limits<int32_t>::max());
  Span<int32_t> indices(reinterpret_cast<int32_t *>(r_corners), r_corner_len);
  return IndexMask::from_indices<int32_t>(indices, memory);
}

}  // namespace blender::ed::greasepencil
