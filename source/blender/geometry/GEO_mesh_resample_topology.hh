/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#pragma once

#include "BLI_virtual_array.hh"

#include "BKE_attribute.hh"

struct Mesh;

namespace blender::geometry {

/**
 * Param\ mask: contains only edges valid to resample. With enought count to resample.
 */
Mesh *resample_topology(const Mesh &mesh,
                        const Span<int> resample_edge_num,
                        const IndexMask &face_mask,
                        Map<bke::AttributeIDRef, bke::AttributeKind> attributes);

}  // namespace blender::geometry
