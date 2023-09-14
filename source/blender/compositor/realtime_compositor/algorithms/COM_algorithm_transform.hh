/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_math_matrix_types.hh"

#include "COM_context.hh"
#include "COM_domain.hh"
#include "COM_result.hh"

namespace blender::realtime_compositor {

void transform(Context &context,
               Result &input,
               Result &output,
               float3x3 transformation,
               Interpolation interpolation);

}  // namespace blender::realtime_compositor
