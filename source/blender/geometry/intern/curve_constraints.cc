/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_matrix.hh"
#include "BLI_task.hh"

#include "GEO_curve_constraints.hh"

#include "BKE_bvhutils.h"

/**
 * The code below uses a prefix naming convention to indicate the coordinate space:
 * `cu`: Local space of the curves object that is being edited.
 * `su`: Local space of the surface object.
 * `wo`: World space.
 */

namespace blender::geometry::curve_constraints {

void compute_segment_lengths(const OffsetIndices<int> points_by_curve,
                             const Span<float3> positions,
                             const IndexMask curve_selection,
                             MutableSpan<float> r_segment_lengths)
{
  BLI_assert(r_segment_lengths.size() == points_by_curve.total_size());

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

void solve_fixed_root_length_constraints(const OffsetIndices<int> points_by_curve,
                                         const IndexMask curve_selection,
                                         const Span<float> segment_lenghts,
                                         MutableSpan<float3> positions)
{
  BLI_assert(segment_lenghts.size() == points_by_curve.total_size());

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

void solve_symmetric_length_constraints(const OffsetIndices<int> points_by_curve,
                                        const IndexMask curve_selection,
                                        const Span<float> segment_lenghts,
                                        MutableSpan<float3> positions)
{
  BLI_assert(segment_lenghts.size() == points_by_curve.total_size());

  threading::parallel_for(curve_selection.index_range(), 256, [&](const IndexRange range) {
    for (const int curve_i : curve_selection.slice(range)) {
      /* Preconditioning:
       * Max. distance can not be greater than total curve length up to each point,
       * we can clamp that distance to get closer to the solution and speed up convergence.
       * This also ensures perfect solution of the root pinning constraint. */

      {
        const IndexRange points = points_by_curve[curve_i];

        if (!points.is_empty()) {
          const float3 root = positions[points.first()];
          float total_distance = 0.0f;
          for (const int point_i : points) {
            float3 &p = positions[point_i];
            float distance_sq = math::distance_squared(p, root);
            if (distance_sq > 0.0f && distance_sq >= total_distance * total_distance) {
              p = root + (p - root) * total_distance / math::sqrt(distance_sq);
            }

            total_distance += segment_lenghts[point_i];
          }
        }
      }

      {
        const IndexRange points = points_by_curve[curve_i].drop_back(1);
        for (const int point_i : points) {
          float3 &p0 = positions[point_i];
          float3 &p1 = positions[point_i + 1];

          const float goal_length = segment_lenghts[point_i];
          float length;
          const float3 gradient_p0 = math::normalize_and_get_length(p0 - p1, length) / goal_length;
          const float3 gradient_p1 = -gradient_p0;
          const float distance = length / goal_length - 1.0f;

          /* Implicit pinning constraint for the first point of each curve */
          const float weight_p0 = (point_i > points.first() ? 1.0f : 0.0f);
          const float weight_p1 = 1.0f;

          const float gradient_sq_sum = weight_p0 * math::dot(gradient_p0, gradient_p0) +
                                        weight_p1 * math::dot(gradient_p1, gradient_p1);
          const float lambda = distance / gradient_sq_sum;

          p0 -= weight_p0 * lambda * gradient_p0;
          p1 -= weight_p1 * lambda * gradient_p1;
        }
      }
    }
  });
}

void solve_collision_constraints(const OffsetIndices<int> points_by_curve,
                                 const IndexMask curve_selection,
                                 const Span<float> segment_lengths_cu,
                                 const Span<float3> start_positions_cu,
                                 const Mesh &surface,
                                 const bke::CurvesSurfaceTransforms &transforms,
                                 MutableSpan<float3> positions_cu)
{
  BVHTreeFromMesh surface_bvh;
  BKE_bvhtree_from_mesh_get(&surface_bvh, &surface, BVHTREE_FROM_LOOPTRI, 2);
  BLI_SCOPED_DEFER([&]() { free_bvhtree_from_mesh(&surface_bvh); });

  const float radius = 0.005f;
  const int max_collisions = 5;

  threading::parallel_for(curve_selection.index_range(), 64, [&](const IndexRange range) {
    for (const int curve_i : curve_selection.slice(range)) {
      const IndexRange points = points_by_curve[curve_i];

      /* Sometimes not all collisions can be handled. This happens relatively rarely, but if it
       * happens it's better to just not to move the curve instead of going into the surface. */
      bool revert_curve = false;
      for (const int point_i : points.drop_front(1)) {
        const float goal_segment_length_cu = segment_lengths_cu[point_i - 1];
        const float3 &prev_pos_cu = positions_cu[point_i - 1];
        const float3 &start_pos_cu = start_positions_cu[point_i];

        int used_iterations = 0;
        for ([[maybe_unused]] const int iteration : IndexRange(max_collisions)) {
          used_iterations++;
          const float3 &old_pos_cu = positions_cu[point_i];
          if (start_pos_cu == old_pos_cu) {
            /* The point did not move, done. */
            break;
          }

          /* Check if the point moved through a surface. */
          const float3 start_pos_su = math::transform_point(transforms.curves_to_surface,
                                                            start_pos_cu);
          const float3 old_pos_su = math::transform_point(transforms.curves_to_surface,
                                                          old_pos_cu);
          const float3 pos_diff_su = old_pos_su - start_pos_su;
          float max_ray_length_su;
          const float3 ray_direction_su = math::normalize_and_get_length(pos_diff_su,
                                                                         max_ray_length_su);
          BVHTreeRayHit hit;
          hit.index = -1;
          hit.dist = max_ray_length_su + radius;
          BLI_bvhtree_ray_cast(surface_bvh.tree,
                               start_pos_su,
                               ray_direction_su,
                               radius,
                               &hit,
                               surface_bvh.raycast_callback,
                               &surface_bvh);
          if (hit.index == -1) {
            break;
          }
          const float3 hit_pos_su = hit.co;
          const float3 hit_normal_su = hit.no;
          if (math::dot(hit_normal_su, ray_direction_su) > 0.0f) {
            /* Moving from the inside to the outside is ok. */
            break;
          }

          /* The point was moved through a surface. Now put it back on the correct side of the
           * surface and slide it on the surface to keep the length the same. */

          const float3 hit_pos_cu = math::transform_point(transforms.surface_to_curves,
                                                          hit_pos_su);
          const float3 hit_normal_cu = math::normalize(
              math::transform_direction(transforms.surface_to_curves_normal, hit_normal_su));

          /* Slide on a plane that is slightly above the surface. */
          const float3 plane_pos_cu = hit_pos_cu + hit_normal_cu * radius;
          const float3 plane_normal_cu = hit_normal_cu;

          /* Decompose the current segment into the part normal and tangent to the collision
           * surface. */
          const float3 collided_segment_cu = plane_pos_cu - prev_pos_cu;
          const float3 slide_normal_cu = plane_normal_cu *
                                         math::dot(collided_segment_cu, plane_normal_cu);
          const float3 slide_direction_cu = collided_segment_cu - slide_normal_cu;

          float slide_direction_length_cu;
          const float3 normalized_slide_direction_cu = math::normalize_and_get_length(
              slide_direction_cu, slide_direction_length_cu);
          const float slide_normal_length_sq_cu = math::length_squared(slide_normal_cu);

          if (pow2f(goal_segment_length_cu) > slide_normal_length_sq_cu) {
            /* Use pythagorian theorem to determine how far to slide. */
            const float slide_distance_cu = std::sqrt(pow2f(goal_segment_length_cu) -
                                                      slide_normal_length_sq_cu) -
                                            slide_direction_length_cu;
            positions_cu[point_i] = plane_pos_cu +
                                    normalized_slide_direction_cu * slide_distance_cu;
          }
          else {
            /* Minimum distance is larger than allowed segment length.
             * The unilateral collision constraint is satisfied by just clamping segment length. */
            positions_cu[point_i] = prev_pos_cu + math::normalize(old_pos_su - prev_pos_cu) *
                                                      goal_segment_length_cu;
          }
        }
        if (used_iterations == max_collisions) {
          revert_curve = true;
          break;
        }
      }
      if (revert_curve) {
        positions_cu.slice(points).copy_from(start_positions_cu.slice(points));
      }
    }
  });
}

void solve_keyhole_constraints(const OffsetIndices<int> points_by_curve,
                               const IndexMask curve_selection,
                               const Span<float3> goals,
                               const Span<float> goal_factors,
                               const VArray<float> point_factors,
                               const float step_size,
                               MutableSpan<float3> positions_cu,
                               MutableSpan<int> closest_points,
                               MutableSpan<float> closest_factors)
{
  /* Compensation factor for step-size dependent softness (see XPBD paper) */
  const float alpha_compensation = 1.0f /* / (step_size * step_size)*/;

  threading::parallel_for(curve_selection.index_range(), 64, [&](const IndexRange range) {
    for (const int curve_i : curve_selection.slice(range)) {
      const IndexRange points = points_by_curve[curve_i].drop_back(1);
      const float3 &goal = goals[curve_i];

      int &closest_point = closest_points[curve_i];
      float &closest_u = closest_factors[curve_i];
      float3 p0_fallback = positions_cu[closest_point];
      float3 p3_fallback = positions_cu[closest_point + 1];
      float3 &p0 = closest_point > points.first() ? positions_cu[closest_point - 1] : p0_fallback;
      float3 &p1 = positions_cu[closest_point];
      float3 &p2 = positions_cu[closest_point + 1];
      float3 &p3 = closest_point < points.last() ? positions_cu[closest_point + 2] : p3_fallback;

      const float u = closest_u;
      const float u2 = u * u;
      const float u3 = u2 * u;
      const float v = 1.0 - u;
      const float v2 = v * v;
      const float v3 = v2 * v;

      /* Uniform cubic b-spline basis functions ensure smooth derivatives. */
      const float b0 = v3 / 6.0f;
      const float b1 = (3.0f * u3 - 6.0f * u2 + 4.0f) / 6.0f;
      const float b2 = (-3.0f * u3 + 3.0f * u2 + 3.0f * u + 1.0f) / 6.0f;
      const float b3 = u3 / 6.0f;
      const float db0 = v2 / 2.0f;
      const float db1 = (3.0f * u2 - 4.0f * u) / 2.0f;
      const float db2 = (-3.0f * u2 + 2.0f * u + 1.0f) / 2.0f;
      const float db3 = u2 / 2.0f;

      /* Linear basis for testing */
//      const float b0 = 0.0f;
//      const float b1 = v;
//      const float b2 = u;
//      const float b3 = 0.0f;
//      const float db0 = 0.0f;
//      const float db1 = -1.0f;
//      const float db2 = 1.0f;
//      const float db3 = 0.0f;

      const float3 distance = b0 * p0 + b1 * p1 + b2 * p2 + b3 * p3 - goal;
      /* Note: technically gradients here are float3x3 matrices,
       * but they are scaled identities, so only need to store the scalar factor. */
      const float gradient_p0 = b0;
      const float gradient_p1 = b1;
      const float gradient_p2 = b2;
      const float gradient_p3 = b3;
      const float3 gradient_u = p0 * db0 + p1 * db1 + p2 * db2 + p3 * db3;

      const float weight_p0 = closest_point > points.first() + 1 ? 1.0f : 0.0f;
      const float weight_p1 = closest_point > points.first() ? 1.0f : 0.0f;
      const float weight_p2 = 1.0f;
      const float weight_p3 = 1.0f;
      const float weight_u = 100.0f;

      const float gradient_sq_sum = weight_p0 * gradient_p0 * gradient_p0 +
                                    weight_p1 * gradient_p1 * gradient_p1 +
                                    weight_p2 * gradient_p2 * gradient_p2 +
                                    weight_p3 * gradient_p3 * gradient_p3 +
                                    weight_u * math::dot(gradient_u, gradient_u);
      const float factor = goal_factors[curve_i];
      /* TODO implement warm starting: this is in fact delta_lambda.
       * If lambda is stored between iteration we can use the old value and speed up convergence. */
      const float3 lambda = factor * distance / (factor * gradient_sq_sum + alpha_compensation * (1.0f - factor));

      /* See above: gradients wrt. positions are actually matrices, but with uniform diagonals. */
      p0 -= weight_p0 * lambda * gradient_p0;
      p1 -= weight_p1 * lambda * gradient_p1;
      p2 -= weight_p2 * lambda * gradient_p2;
      p3 -= weight_p3 * lambda * gradient_p3;

      const float curve_factor_unclamped = closest_point + closest_u - weight_u * math::dot(lambda, gradient_u);
      closest_point = math::clamp((int)curve_factor_unclamped, (int)points.first(), (int)points.last());
      closest_u = math::clamp(curve_factor_unclamped, (float)points.first(), (float)points.last() + 1.0f) - (float)closest_point;
    }
  });
}

}  // namespace blender::geometry::curve_constraints
