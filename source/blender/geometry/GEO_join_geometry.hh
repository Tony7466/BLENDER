/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BKE_geometry_set.hh"

namespace blender::geometry {

/**
 * Join all src_components and add the resulting component to result.
 */
template<typename Component>
void join_component_type(Span<const Component *> src_components,
                         GeometrySet &result,
                         const bke::AnonymousAttributePropagationInfo &propagation_info);

/**
 * Join InstancesComponents and apply corresponding transforms for each.
 */
void join_transform_instance_components(Span<const InstancesComponent *> src_components,
                                        Span<blender::float4x4> src_base_transforms,
                                        GeometrySet &result);

}  // namespace blender::geometry
