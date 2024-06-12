/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_span.hh"
#include "BLI_math_vector_types.hh"

struct Mesh;

namespace blender::geometry::dyntopo {

Mesh *subdivide(const Mesh &src, Span<float2> projection, float2 position, float radius, float max_length);

}  // namespace blender::geometry::dyntopo
