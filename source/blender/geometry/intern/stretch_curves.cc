/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include "BLI_array_utils.hh"
#include "BLI_length_parameterize.hh"
#include "BLI_math_axis_angle.hh"
#include "BLI_math_matrix.h"
#include "BLI_math_quaternion.hh"
#include "BLI_math_rotation.h"
#include "BLI_math_rotation.hh"
#include "BLI_math_vector.hh"

#include "BKE_attribute.hh"
#include "BKE_attribute_math.hh"
#include "BKE_curves.hh"
#include "BKE_curves_utils.hh"
#include "BKE_geometry_set.hh"

#include "GEO_stretch_curves.hh"

namespace blender::geometry {

bke::CurvesGeometry stretch_curves(
    const bke::CurvesGeometry &src_curves,
    const IndexMask &selection,
    const VArray<float> &start_lengths,
    const VArray<float> &end_lengths,
    const VArray<float> &overshoot_fac,
    const VArray<bool> &follow_curvature,
    const VArray<int> &point_density,
    const VArray<float> &segment_influence,
    const VArray<float> &max_angle,
    const VArray<bool> &invert_curvature,
    const GeometryNodeCurveSampleMode sample_mode,
    const bke::AnonymousAttributePropagationInfo & /*propagation_info*/)
{
  if (src_curves.points_num() < 2) {
    return src_curves;
  }

  const int src_curves_num = src_curves.curves_num();
  Array<int> start_points(src_curves_num);
  Array<int> end_points(src_curves_num);
  Array<float> use_start_lengths(src_curves_num);
  Array<float> use_end_lengths(src_curves_num);

  const OffsetIndices<int> points_by_curve = src_curves.points_by_curve();
  Array<int> dst_point_counts(src_curves_num + 1);

  src_curves.ensure_evaluated_lengths();

  /* Count how many points we need. */
  for (const int curve : src_curves.curves_range()) {
    int point_count = points_by_curve[curve].size();
    dst_point_counts[curve] = point_count;
    /* Curve not suitable for stretching... */
    if (point_count <= 2 || (!selection.contains(curve))) {
      continue;
    }
    float density = point_density[curve];
    if (density < 1e-4f) {
      density = 0.1;
    }

    use_start_lengths[curve] = start_lengths[curve];
    use_end_lengths[curve] = end_lengths[curve];
    if (sample_mode == GEO_NODE_CURVE_SAMPLE_FACTOR) {
      float total_length = src_curves.evaluated_length_total_for_curve(curve, false);
      use_start_lengths[curve] *= total_length;
      use_end_lengths[curve] *= total_length;
    }

    const int count_start = (use_start_lengths[curve] > 0) ?
                                (math::ceil(use_start_lengths[curve] * density)) :
                                0;
    const int count_end = (use_end_lengths[curve] > 0) ?
                              (math::ceil(use_end_lengths[curve] * density)) :
                              0;
    dst_point_counts[curve] += count_start;
    dst_point_counts[curve] += count_end;
    start_points[curve] = count_start;
    end_points[curve] = count_end;
  }
  OffsetIndices<int> dst_point_offsets = offset_indices::accumulate_counts_to_offsets(
      dst_point_counts.as_mutable_span());
  int target_point_count = dst_point_offsets.total_size();

  /* Make dest to source map for points. */
  Array<int> dst_to_src_point(target_point_count);
  for (const int curve : src_curves.curves_range()) {
    const int point_count = points_by_curve[curve].size();
    int local_front = 0;
    MutableSpan<int> new_points = dst_to_src_point.as_mutable_span().slice(
        dst_point_offsets[curve]);
    if (start_points[curve]) {
      MutableSpan<int> starts = new_points.slice(0, start_points[curve]);
      starts.fill(points_by_curve[curve].first());
      local_front = start_points[curve];
    }
    if (end_points[curve]) {
      MutableSpan<int> ends = new_points.slice(new_points.size() - end_points[curve],
                                               end_points[curve]);
      ends.fill(points_by_curve[curve].last());
    }
    MutableSpan<int> original_points = new_points.slice(local_front, point_count);
    for (const int point_i : original_points.index_range()) {
      original_points[point_i] = points_by_curve[curve][point_i];
    }
  }

  /* Copy only curves domain since we are not changing the number of curves here. */
  bke::CurvesGeometry dst_curves = bke::curves::copy_only_curve_domain(src_curves);
  dst_curves.resize(target_point_count, src_curves_num);

  MutableSpan<int> new_curve_offsets = dst_curves.offsets_for_write();
  array_utils::copy(dst_point_counts.as_span(), new_curve_offsets);

  const bke::AttributeAccessor src_attributes = src_curves.attributes();
  bke::MutableAttributeAccessor dst_attributes = dst_curves.attributes_for_write();

  /* Transfer point attributes. */
  gather_attributes(
      src_attributes, bke::AttrDomain::Point, {}, {}, dst_to_src_point, dst_attributes);

  dst_curves.tag_topology_changed();

  MutableSpan<float3> positions = dst_curves.positions_for_write();

  const OffsetIndices<int> new_points = dst_curves.points_by_curve();
  for (const int curve : dst_curves.curves_range()) {
    if (!start_points[curve] && !end_points[curve]) {
      /* Curves should not be touched if they didn't generate extra points before. */
      continue;
    }
    int new_size = new_points[curve].size();
    float used_percent_length = overshoot_fac[curve];
    used_percent_length = std::clamp(used_percent_length, 1e-4f, 1.0f);
    if (!isfinite(used_percent_length)) {
      /* #used_percent_length must always be finite, otherwise a segfault occurs.
       * Since this function should never segfault, set #used_percent_length to a safe fallback. */
      /* NOTE: This fallback is used if `gps->totpoints == 2`, see `MOD_gpencil_legacy_length.cc`.
       */
      used_percent_length = 0.1f;
    }
    /* Handle simple case first, straight stretching. */
    if (!follow_curvature[curve]) {

      float overshoot_point_param = used_percent_length * (new_size - 1);
      float3 result;
      if (start_points[curve]) {
        // TODO: I don't think the algorithm here is correct?
        // I think It should always be endpoint to overshoot point?
        int index1 = math::floor(overshoot_point_param);
        int index2 = math::ceil(overshoot_point_param);
        result = math::interpolate(positions[new_points[curve][index1]],
                                   positions[new_points[curve][index2]],
                                   fmodf(overshoot_point_param, 1.0f));
        result -= positions[new_points[curve].first()];
        if (UNLIKELY(math::is_zero(result))) {
          result = positions[new_points[curve][1]] - positions[new_points[curve][0]];
        }
        madd_v3_v3fl(
            positions[new_points[curve][0]], result, -use_start_lengths[curve] / len_v3(result));
      }

      if (end_points[curve]) {
        int index1 = new_size - 1 - math::floor(overshoot_point_param);
        int index2 = new_size - 1 - math::ceil(overshoot_point_param);
        result = math::interpolate(positions[new_points[curve][index1]],
                                   positions[new_points[curve][index2]],
                                   fmodf(overshoot_point_param, 1.0f));
        result -= positions[new_points[curve].last()];
        if (UNLIKELY(math::is_zero(result))) {
          result = positions[new_points[curve][new_size - 2]] -
                   positions[new_points[curve][new_size - 1]];
        }
        positions[new_points[curve][new_size - 1]] += result *
                                                      (-use_end_lengths[curve] / len_v3(result));
      }
    }
    else { /* Now take care of stretching with curvature. */

      /* Curvature calculation. */

      const int first_old_index = start_points[curve] ? start_points[curve] : 0;
      const int last_old_index = points_by_curve[curve].size() - 1 + first_old_index;
      const int orig_totpoints = points_by_curve[curve].size();

      /* The fractional amount of points to query when calculating the average curvature of the
       * strokes. */
      const float overshoot_parameter = used_percent_length * (orig_totpoints - 2);
      int overshoot_pointcount = math::ceil(overshoot_parameter);
      overshoot_pointcount = std::clamp(overshoot_pointcount, 1, orig_totpoints - 2);

      /* Do for both sides without code duplication. */
      float3 vec1, total_angle;
      for (int k = 0; k < 2; k++) {
        if ((k == 0 && !start_points[curve]) || (k == 1 && !end_points[curve])) {
          continue;
        }

        const int start_i = k == 0 ? first_old_index :
                                     last_old_index; /* first_old_index, last_old_index */
        const int dir_i = 1 - k * 2;                 /* 1, -1 */

        vec1 = positions[new_points[curve][start_i + dir_i]] -
               positions[new_points[curve][start_i]];
        total_angle = float3({0, 0, 0});

        float segment_length;
        vec1 = math::normalize_and_get_length(vec1, segment_length);

        float overshoot_length = 0.0f;

        /* Accumulate rotation angle and length. */
        int j = 0;
        float3 no, vec2;
        for (int i = start_i; j < overshoot_pointcount; i += dir_i, j++) {
          /* Don't fully add last segment to get continuity in overshoot_fac. */
          float fac = math::min(overshoot_parameter - j, 1.0f);

          /* Read segments. */
          vec2 = vec1;
          vec1 = positions[new_points[curve][i + dir_i * 2]] -
                 positions[new_points[curve][i + dir_i]];

          float len;
          vec1 = math::normalize_and_get_length(vec1, len);
          float angle = math::angle_between(vec1, vec2).radian() * fac;

          /* Add half of both adjacent legs of the current angle. */
          const float added_len = (segment_length + len) * 0.5f * fac;
          overshoot_length += added_len;
          segment_length = len;

          if (angle > max_angle[curve]) {
            continue;
          }
          if (angle > M_PI * 0.995f) {
            continue;
          }

          angle *= math::pow(added_len, segment_influence[curve]);

          no = math::cross(vec1, vec2);
          no = math::normalize(no) * angle;
          total_angle += no;
        }

        if (UNLIKELY(overshoot_length == 0.0f)) {
          /* Don't do a proper extension if the used points are all in the same position. */
          continue;
        }

        vec1 = positions[new_points[curve][start_i]] -
               positions[new_points[curve][start_i + dir_i]];
        /* In general curvature = 1/radius. For the case without the
         * weights introduced by #segment_influence, the calculation is:
         * `curvature = delta angle/delta arclength = len_v3(total_angle) / overshoot_length` */
        float curvature = normalize_v3(total_angle) / overshoot_length;
        /* Compensate for the weights powf(added_len, segment_influence). */
        curvature /= math::pow(overshoot_length / math::min(overshoot_parameter, float(j)),
                               segment_influence[curve]);
        if (invert_curvature[curve]) {
          curvature = -curvature;
        }
        const float dist = k == 0 ? use_start_lengths[curve] : use_end_lengths[curve];
        const int extra_point_count = k == 0 ? start_points[curve] : end_points[curve];
        const float angle_step = curvature * dist / extra_point_count;
        float step_length = dist / extra_point_count;
        if (math::abs(angle_step) > FLT_EPSILON) {
          /* Make a direct step length from the assigned arc step length. */
          step_length *= sin(angle_step * 0.5f) / (angle_step * 0.5f);
        }
        else {
          total_angle = float3({0, 0, 0});
        }
        float prev_length;
        vec1 = math::normalize_and_get_length(vec1, prev_length);
        vec1 *= step_length;

        /* Build rotation matrix here to get best performance. */
        math::AxisAngle axis_base(total_angle, angle_step);
        math::Quaternion q = math::to_quaternion(axis_base);
        float3x3 rot = math::from_rotation<float3x3>(q);

        /* Rotate the starting direction to account for change in edge lengths. */
        math::AxisAngle step_base(total_angle,
                                  math::max(0.0f, 1.0f - math::abs(segment_influence[curve])) *
                                      (curvature * prev_length - angle_step) / 2.0f);
        q = math::to_quaternion(step_base);
        vec1 = math::transform_point(q, vec1);

        /* Now iteratively accumulate the segments with a rotating added direction. */
        for (int i = start_i - dir_i, j = 0; j < extra_point_count; i -= dir_i, j++) {
          vec1 = rot * vec1;
          positions[new_points[curve][i]] = vec1 + positions[new_points[curve][i + dir_i]];
        }
      }
    }
  }

  dst_curves.tag_positions_changed();

  return dst_curves;
}

}  // namespace blender::geometry
