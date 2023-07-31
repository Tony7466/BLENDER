/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_span.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_math_matrix_types.hh"
#include "BLI_math_euler_types.hh"

namespace blender::geometry {

void copy_transformed(const Span<float3> src, const float4x4 &transform, MutableSpan<float3> dst);
void transform(const float4x4 &transform, MutableSpan<float3> positions);

bool zero_translation(const float3 &translation);
bool zero_rotation(const math::EulerXYZ &rotation);
bool unit_scale(const float3 &scale);

}  // namespace blender::geometry
