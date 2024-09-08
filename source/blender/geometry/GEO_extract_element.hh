/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BKE_attribute_filter.hh"

#include "BLI_array.hh"
#include "BLI_index_mask_fwd.hh"

struct Mesh;

namespace blender::geometry {

Array<Mesh *> extract_vertex_meshes(const Mesh &mesh,
                                    const IndexMask &mask,
                                    const bke::AttributeFilter &attribute_filter);

}  // namespace blender::geometry
