/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "FN_field.hh"

#include "BKE_attribute.hh"
#include "BKE_curves.hh"

namespace blender::geometry {

using bke::CurvesGeometry;

/**
 * Create new curves that are interpolated between "from" and "to" curves.
 */
CurvesGeometry interpolate_curves(const CurvesGeometry &from_curves,
                                  const CurvesGeometry &to_curves,
                                  Span<int> from_curve_indices,
                                  Span<int> to_curve_indices,
                                  const IndexMask &selection,
                                  VArray<int> dst_curve_counts,
                                  VArray<bool> curve_flip_direction,
                                  float mix_factor);

void interpolate_curves(const CurvesGeometry &from_curves,
                        const CurvesGeometry &to_curves,
                        const Span<int> from_curve_indices,
                        const Span<int> to_curve_indices,
                        const IndexMask &selection,
                        VArray<bool> curve_flip_direction,
                        const float mix_factor,
                        CurvesGeometry &dst_curves);

}  // namespace blender::geometry
