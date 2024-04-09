/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_math_vector_types.hh"

#include "COM_context.hh"
#include "COM_result.hh"

namespace blender::realtime_compositor {

void recursive_gaussian_blur(Context &context, Result &input, Result &output, float2 radius);

}  // namespace blender::realtime_compositor
