/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_virtual_array.hh"

#include "BKE_attribute.hh"

struct Mesh;

namespace blender::geometry {

enum ResampleTopologyMode : int8_t {
  FILL_GRID = 0,
  FILL_NGONE = 1,
  FILL_DELONE = 2,
};

Mesh *resample_topology(const Mesh &mesh,
                        const Span<int> resample_edge_num,
                        const ResampleTopologyMode fill_mode,
                        const Map<bke::AttributeIDRef, bke::AttributeKind> attributes);

}  // namespace blender::geometry
