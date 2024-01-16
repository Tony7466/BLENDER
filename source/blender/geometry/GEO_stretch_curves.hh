/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_span.hh"
#include "DNA_node_types.h"

#include "BKE_curves.hh"
#include "BKE_curves_utils.hh"
#include "BKE_geometry_set.hh"

namespace blender::geometry {

/*
 * Create a new Curves instance by trimming the input curves. Copying the selected splines
 * between the start and end points.
 */
bke::CurvesGeometry stretch_curves(const bke::CurvesGeometry &src_curves,
                                   const IndexMask &selection,
                                   const VArray<float> &start_lengths,
                                   const VArray<float> &end_lengths,
                                    const VArray<float> &overshoot_fac,
                                    const VArray<bool> &follow_curvature,
                                    const VArray<int> &extra_point_count,
                                    const VArray<float> &segment_influence,
                                    const VArray<float> &max_angle,
                                    const VArray<bool> &invert_curvature,
                                   const bke::AnonymousAttributePropagationInfo &propagation_info);

}  // namespace blender::geometry
