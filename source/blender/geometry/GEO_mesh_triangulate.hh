/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <optional>

#include "BLI_index_mask.hh"

#include "BKE_attribute.hh"

struct Mesh;

namespace blender::geometry {

enum class TriangulateNGonMode {
  Beauty = 0,
  EarClip = 1,
};

enum class TriangulateQuadMode {
  Beauty = 0,
  Fixed = 1,
  Alternate = 2,
  ShortEdge = 3,
  LongEdge = 4,
};

std::optional<Mesh *> mesh_triangulate(
    const Mesh &src_mesh,
    const IndexMask &selection,
    const TriangulateNGonMode ngon_mode,
    const TriangulateQuadMode quad_mode,
    const bke::AnonymousAttributePropagationInfo &propagation_info);

}  // namespace blender::geometry
