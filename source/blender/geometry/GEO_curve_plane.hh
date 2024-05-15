/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#ifdef WITH_POTRACE

#  include "BKE_attribute.hh"
#  include "BKE_curves.hh"

#  include "BLI_math_vector_types.hh"
#  include "BLI_span.hh"

namespace blender::geometry {

Curves *plane_to_curve(int2 resolution,
                       Span<bool> grid_color,
                       float2 min_point,
                       float2 max_point,
                       const bke::AttributeIDRef &uv_map_id);

}  // namespace blender::geometry

#endif
