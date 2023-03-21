/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BKE_curves.hh"

namespace blender::geometry::curve_constraints {

void compute_segment_lengths(OffsetIndices<int> points_by_curve,
                             Span<float3> positions,
                             IndexMask curve_selection,
                             MutableSpan<float> r_segment_lengths);

void solve_fixed_root_length_constraints(OffsetIndices<int> points_by_curve,
                                         IndexMask curve_selection,
                                         Span<float> segment_lengths,
                                         MutableSpan<float3> positions);

void solve_symmetric_length_constraints(OffsetIndices<int> points_by_curve,
                                        IndexMask curve_selection,
                                        Span<float> segment_lengths,
                                        MutableSpan<float3> positions);

void compute_length_constraints_errors(const OffsetIndices<int> points_by_curve,
                                       const IndexMask curve_selection,
                                       const Span<float> segment_lengths,
                                       const Span<float3> positions,
                                       MutableSpan<float> errors_sq);

void solve_collision_constraints(OffsetIndices<int> points_by_curve,
                                 IndexMask curve_selection,
                                 Span<float> segment_lengths_cu,
                                 Span<float3> start_positions_cu,
                                 const Mesh &surface,
                                 const bke::CurvesSurfaceTransforms &transforms,
                                 MutableSpan<float3> positions_cu);

void solve_keyhole_constraints(OffsetIndices<int> points_by_curve,
                               IndexMask curve_selection,
                               Span<float3> goals,
                               Span<float> goal_factors,
                               VArray<float> point_factors,
                               float step_size,
                               MutableSpan<float3> positions_cu,
                               MutableSpan<int> closest_points,
                               MutableSpan<float> closest_factors);

void compute_keyhole_constraints_errors(OffsetIndices<int> points_by_curve,
                                        IndexMask curve_selection,
                                        Span<float3> goals,
                                        Span<float> goal_factors,
                                        VArray<float> point_factors,
                                        Span<float3> positions_cu,
                                        Span<int> closest_points,
                                        Span<float> closest_factors,
                                        MutableSpan<float> errors_sq);

}  // namespace blender::geometry::curve_constraints
