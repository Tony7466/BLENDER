/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_assert.h"
#include "BLI_span.hh"
#include "BLI_math_vector.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_euler_types.hh"
#include "BLI_task.hh"

namespace blender::geometry {

void copy_transformed(const Span<float3> src, const float4x4 &transform, MutableSpan<float3> dst)
{
  BLI_assert(src.size() == dst.size());
  threading::parallel_for(src.index_range(), 1024, [&](const IndexRange range) {
    for (const int i : range) {
      dst[i] = math::transform_point(transform, src[i]);
    }
  });
}

void transform(const float4x4 &transform, MutableSpan<float3> positions)
{
  threading::parallel_for(positions.index_range(), 1024, [&](const IndexRange range) {
    for (float3 &position : positions.slice(range)) {
      position = math::transform_point(transform, position);
    }
  });
}

static constexpr float epsilon = 1e-9f;

bool zero_translation(const float3 &translation)
{
  return math::almost_equal_relative(translation, float3(0.0f), epsilon);
}

bool zero_rotation(const math::EulerXYZ &rotation)
{
  return math::almost_equal_relative(float3(rotation), float3(0.0f), epsilon);
}

bool unit_scale(const float3 &scale)
{
  return math::almost_equal_relative(scale, float3(1.0f), epsilon);
}

}  // namespace blender::geometry
