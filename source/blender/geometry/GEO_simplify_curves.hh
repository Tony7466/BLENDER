/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BKE_curves.hh"

namespace blender::geometry {

using bke::CurvesGeometry;

/**
 * Simplifies the selected curves using the Ramer-Douglas-Peucker algorithm by removing points that
 * don't change the shape of the curve within distance #epsilon.
 */
CurvesGeometry curves_simplify(const CurvesGeometry &src_curves,
                               const IndexMask &selection,
                               float epsilon);

}  // namespace blender::geometry
