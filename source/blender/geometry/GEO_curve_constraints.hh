/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BKE_curves.hh"

namespace blender::geometry::curve_constraints {

void compute_segment_lengths(OffsetIndices<int> points_by_curve,
                             Span<float3> positions,
                             IndexMask curve_selection,
                             MutableSpan<float> r_segment_lengths);

void solve_length_constraints(OffsetIndices<int> points_by_curve,
                              IndexMask curve_selection,
                              Span<float> segment_lenghts,
                              MutableSpan<float3> positions);

void solve_collision_constraints(OffsetIndices<int> points_by_curve,
                                 IndexMask curve_selection,
                                 Span<float> segment_lengths_cu,
                                 Span<float3> start_positions_cu,
                                 const Mesh &surface,
                                 const bke::CurvesSurfaceTransforms &transforms,
                                 MutableSpan<float3> positions_cu);

void solve_slip_constraints(OffsetIndices<int> points_by_curve,
                            IndexMask curve_selection,
                            Span<float3> goals,
                            Span<float> goal_factors,
                            VArray<float> point_factors,
                            float step_size,
                            MutableSpan<float3> positions_cu,
                            MutableSpan<int> closest_points,
                            MutableSpan<float> closest_factors);

}  // namespace blender::geometry::curve_constraints
