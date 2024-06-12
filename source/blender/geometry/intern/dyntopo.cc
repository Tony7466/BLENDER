/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_mesh.hh"

#include "BLI_span.hh"
#include "BLI_math_vector_types.hh"

#include "GEO_mesh_selection.hh"

namespace blender::geometry::dyntopo {

Mesh *subdivide(const Mesh &src, const Span<float2> projection, const float2 position, const float radius, const float max_length)
{
  return nullptr;
}

}  // namespace blender::geometry::dyntopo