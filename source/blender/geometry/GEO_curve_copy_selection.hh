/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BKE_anonymous_attribute_id.hh"

#include "BLI_index_mask.hh"

#include "BKE_curves.hh"

namespace blender::geometry {

bke::CurvesGeometry copy_curve_points(
    const bke::CurvesGeometry &src_curves,
    const IndexMask &points_mask,
    const bke::AnonymousAttributePropagationInfo &propagation_info);

}  // namespace blender::geometry
