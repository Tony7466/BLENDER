/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_index_mask.hh"

#include "BKE_anonymous_attribute_id.hh"

struct Mesh;

namespace blender::geometry {

Mesh *dissolve_boundary_verts(
    const Mesh &src_mesh,
    const IndexMask &vers_mask,
    const bke::AnonymousAttributePropagationInfo &propagation_info);

Mesh *dissolve_edges(
    const Mesh &src_mesh,
    const IndexMask &edges_mask,
    const bke::AnonymousAttributePropagationInfo &propagation_info);

Mesh *dissolve_faces(
    const Mesh &src_mesh,
    const IndexMask &faces_mask,
    const bke::AnonymousAttributePropagationInfo &propagation_info);

}  // namespace blender::geometry
