/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "GEO_curve_constraint_solver.hh"

namespace blender::geometry::curve_constraint_solver {

void compute_segment_lengths(const bke::CurvesGeometry &curves,
                             const IndexMask curve_selection,
                             MutableSpan<float> r_segment_lengths)
{
  BLI_assert(r_segment_lengths.size() == curves.points_num());

  const Span<float3> positions = curves.positions();
  const OffsetIndices points_by_curve = curves.points_by_curve();
  threading::parallel_for(curve_selection.index_range(), 256, [&](const IndexRange range) {
    for (const int curve_i : curve_selection.slice(range)) {
      const IndexRange points = points_by_curve[curve_i].drop_back(1);
      for (const int point_i : points) {
        const float3 &p1 = positions[point_i];
        const float3 &p2 = positions[point_i + 1];
        const float length = math::distance(p1, p2);
        r_segment_lengths[point_i] = length;
      }
    }
  });
}

void solve_length_constraints(const bke::CurvesGeometry &curves,
                              const IndexMask curve_selection,
                              const Span<float> segment_lenghts,
                              MutableSpan<float3> positions)
{
  BLI_assert(segment_lenghts.size() == curves.points_num());

  const OffsetIndices points_by_curve = curves.points_by_curve();
  threading::parallel_for(curve_selection.index_range(), 256, [&](const IndexRange range) {
    for (const int curve_i : curve_selection.slice(range)) {
      const IndexRange points = points_by_curve[curve_i].drop_back(1);
      for (const int point_i : points) {
        const float3 &p1 = positions[point_i];
        float3 &p2 = positions[point_i + 1];
        const float3 direction = math::normalize(p2 - p1);
        const float goal_length = segment_lenghts[point_i];
        p2 = p1 + direction * goal_length;
      }
    }
  });
}

void solve_length_and_collision_constraints(const bke::CurvesGeometry &curves,
                                            const IndexMask curve_selection,
                                            const Span<float> segment_lengths,
                                            const Span<float3> start_positions,
                                            const Mesh &surface,
                                            const bke::CurvesSurfaceTransforms &transforms,
                                            MutableSpan<float3> positions)
{
  UNUSED_VARS(
      curves, curve_selection, segment_lengths, start_positions, surface, transforms, positions);
}

}  // namespace blender::geometry::curve_constraint_solver
