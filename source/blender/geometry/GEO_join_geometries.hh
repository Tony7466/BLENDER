/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once
#include "BLI_math_matrix_types.hh"
#include "BKE_anonymous_attribute_id.hh"
#include "BKE_geometry_set.hh"

namespace blender::geometry {

bke::GeometrySet join_geometries(Span<bke::GeometrySet> geometries,
                                 const bke::AnonymousAttributePropagationInfo &propagation_info);

void join_transform_instance_components(Span<const bke::InstancesComponent *> src_components,
                                        Span<blender::float4x4> src_base_transforms,
                                        bke::GeometrySet &result);
}
