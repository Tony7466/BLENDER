/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_sys_types.h"

namespace blender::nodes {

enum GeometryNodeMeshToPointsMode {
  GEO_NODE_MESH_TO_POINTS_VERTICES = 0,
  GEO_NODE_MESH_TO_POINTS_EDGES = 1,
  GEO_NODE_MESH_TO_POINTS_FACES = 2,
  GEO_NODE_MESH_TO_POINTS_CORNERS = 3,
};

struct NodeGeometryMeshToPoints {
  struct DNA {
    /** #GeometryNodeMeshToPointsMode */
    uint8_t mode;
  } dna_;

  GeometryNodeMeshToPointsMode mode() const
  {
    return GeometryNodeMeshToPointsMode(dna_.mode);
  }
};

}  // namespace blender::nodes
