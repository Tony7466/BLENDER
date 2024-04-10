/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bke
 */

#include "BLI_compute_context.hh"
#include "BLI_map.hh"
#include "BLI_math_matrix_types.hh"

namespace blender::bke {

/**
 * A gizmo is identified by a gizmo node (like `Linear Gizmo`) in a specific compute context (e.g.
 * the path of group nodes to get from the geometry nodes modifier to the group containing the
 * gizmo node).
 */
struct GeoNodesGizmoID {
  ComputeContextHash compute_context_hash;
  int node_id;

  BLI_STRUCT_EQUALITY_OPERATORS_2(GeoNodesGizmoID, compute_context_hash, node_id)

  uint64_t hash() const
  {
    return get_default_hash(this->compute_context_hash, this->node_id);
  }
};

struct GizmosEditHints {
  /**
   * Additional transform that is applied to the gizmo because the corresponding geometry has been
   * transformed the same.
   */
  Map<GeoNodesGizmoID, float4x4> gizmo_transforms;
};

}  // namespace blender::bke
