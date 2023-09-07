/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

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

void triangulate(Mesh &mesh,
                 const IndexMask &mask,
                 TriangulateNGonMode ngon_mode,
                 TriangulateQuadMode quad_mode,
                 const bke::AnonymousAttributePropagationInfo &propagation_info);

}  // namespace blender::geometry
