/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_sys_types.h"

namespace blender::nodes {

enum GeometryNodeMeshToPointsMode {
  Vertices = 0,
  Edges = 1,
  Faces = 2,
  Corners = 3,
};

struct NodeGeometryMeshToPoints {
  // uint8_t mode;
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
