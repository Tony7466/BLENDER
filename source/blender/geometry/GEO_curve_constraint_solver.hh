/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BKE_curves.hh"

namespace blender::geometry::curve_constraint_solver {

void compute_segment_lengths(const bke::CurvesGeometry &curves,
                             IndexMask curve_selection,
                             MutableSpan<float> r_segment_lengths);

void solve_length_constraints(const bke::CurvesGeometry &curves,
                              IndexMask curve_selection,
                              Span<float> segment_lenghts,
                              MutableSpan<float3> positions);

void solve_length_and_collision_constraints(const bke::CurvesGeometry &curves,
                                            IndexMask curve_selection,
                                            Span<float> segment_lengths,
                                            Span<float3> start_positions,
                                            const Mesh &surface,
                                            const bke::CurvesSurfaceTransforms &transforms,
                                            MutableSpan<float3> positions);



}  // namespace blender::geometry::curve_constraint_solver
