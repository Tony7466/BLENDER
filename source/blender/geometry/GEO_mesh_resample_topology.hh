/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_virtual_array.hh"

#include "BKE_attribute.hh"

struct Mesh;

namespace blender::geometry {

Mesh &resample_topology(const Mesh &mesh,
                        const Span<int> resample_edge_num,
                        const bool try_to_fill_by_grid,
                        Map<bke::AttributeIDRef, bke::AttributeKind> attributes);

}  // namespace blender::geometry
