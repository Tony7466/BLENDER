/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_multi_value_map.hh"
#include "BLI_span.hh"

#include "BKE_geometry_set.hh"

#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"

namespace blender::geometry {

const MultiValueMap<bke::GeometryComponent::Type, eAttrDomain> &reorder_supports();

bke::GeometryComponent &reordered_component_copy(const bke::GeometryComponent &src_component,
                                                 const Span<int> old_by_new_map,
                                                 const eAttrDomain domain);

};  // namespace blender::geometry
