/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include "BLI_array_utils.hh"
#include "BLI_length_parameterize.hh"
#include "BLI_math_vector.hh"

#include "BKE_attribute.hh"
#include "BKE_attribute_math.hh"
#include "BKE_curves.hh"
#include "BKE_curves_utils.hh"
#include "BKE_geometry_set.hh"

#include "GEO_stretch_curves.hh"

namespace blender::geometry {

bke::CurvesGeometry stretch_curves(const bke::CurvesGeometry &src_curves,
                                   const IndexMask &selection,
                                   const VArray<float> &start_lengths,
                                   const VArray<float> &end_lengths,
                                   const VArray<float> &overshoot_fac,
                                   const VArray<bool> &follow_curvature,
                                   const VArray<int> &extra_point_count_per_side,
                                   const VArray<float> &segment_influence,
                                   const VArray<float> &max_angle,
                                   const VArray<bool> &invert_curvature,
                                   const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  const int src_curves_num = src_curves.curves_num();
  Array<bool> do_starts(src_curves_num);
  Array<bool> do_ends(src_curves_num);

  const OffsetIndices<int> points_by_curve = src_curves.points_by_curve();
  Array<int> dst_point_counts(src_curves_num + 1);

  /* Count how many points we need. */
  for(const int curve : src_curves.curves_range()){
    int point_count = points_by_curve[curve].size();
    dst_point_counts[curve]=point_count;
    /* Curve not suitable for stretching... */
    if(point_count <= 2 || (!selection.contains(curve))){
      continue;
    }
    const int do_start = (start_lengths[curve]>0)?1:0;
    const int do_end = (end_lengths[curve]>0)?1:0;
    dst_point_counts[curve]+=extra_point_count_per_side[curve] * do_start;
    dst_point_counts[curve]+=extra_point_count_per_side[curve] * do_end;
    do_starts[curve]=do_start;
    do_ends[curve]=do_end;
  }
  OffsetIndices<int> dst_point_offsets = offset_indices::accumulate_counts_to_offsets(dst_point_counts.as_mutable_span());
  int target_point_count = dst_point_offsets.total_size();

  /* Make dest to source map for points. */
  Array<int> dst_to_src_point(target_point_count);
  for(const int curve : src_curves.curves_range()){
    const int point_count = points_by_curve[curve].size();
    int local_front = 0;
    MutableSpan<int> new_points = dst_to_src_point.as_mutable_span().slice(dst_point_offsets[curve]);
    if(do_starts[curve]){
      MutableSpan<int> start_points = new_points.slice(0,extra_point_count_per_side[curve]);
      start_points.fill(points_by_curve[curve].first());
      local_front = extra_point_count_per_side[curve];
    }
    if(do_ends[curve]){
      MutableSpan<int> end_points = new_points.slice(
        new_points.size()-extra_point_count_per_side[curve],extra_point_count_per_side[curve]);
      end_points.fill(points_by_curve[curve].last());
    }
    MutableSpan<int> original_points = new_points.slice(local_front, point_count);
    for(const int point_i : original_points.index_range()){
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

  const OffsetIndices<int> new_points = src_curves.points_by_curve();
  for(const int curve : dst_curves.curves_range()){
    int new_size=new_points[curve].size();
    float used_percent_length = overshoot_fac[curve];
    CLAMP(used_percent_length, 1e-4f, 1.0f);
    if (!isfinite(used_percent_length)) {
      /* #used_percent_length must always be finite, otherwise a segfault occurs.
      * Since this function should never segfault, set #used_percent_length to a safe fallback. */
      /* NOTE: This fallback is used if `gps->totpoints == 2`, see `MOD_gpencil_legacy_length.cc`. */
      used_percent_length = 0.1f;
    }
    /* Handle simple case first, straight stretching. */
    if(!follow_curvature[curve]){

      float overshoot_point_param = used_percent_length * (new_size - 1);
      float3 result;
      if (do_starts[curve]) {
        // TODO: I don't think the algorithm here is correct?
        // I think It should always be endpoint to overshoot point?
        int index1 = floor(overshoot_point_param);
        int index2 = ceil(overshoot_point_param);
        interp_v3_v3v3(result, positions[new_points[curve][index1]],
                       positions[new_points[curve][index2]],
                      fmodf(overshoot_point_param, 1.0f));
        result -= positions[new_points[curve].first()];
        if (UNLIKELY(is_zero_v3(result))) {
          result = positions[new_points[curve][1]] - positions[new_points[curve][0]];
        }
        madd_v3_v3fl(positions[new_points[curve][0]], result, -start_lengths[curve] / len_v3(result));
      }

      if (do_ends[curve]) {
        int index1 = new_size - 1 - floor(overshoot_point_param);
        int index2 = new_size - 1 - ceil(overshoot_point_param);
        interp_v3_v3v3(result,positions[new_points[curve][index1]],
                       positions[new_points[curve][index2]],
                      fmodf(overshoot_point_param, 1.0f));
        result -= positions[new_points[curve].last()];
        if (UNLIKELY(is_zero_v3(result))) {
          result = positions[new_points[curve][new_size - 2]] - positions[new_points[curve][new_size - 1]];
        }
        madd_v3_v3fl(positions[new_points[curve][new_size - 1]], result, -end_lengths[curve] / len_v3(result));
      }

    }else{ /* Now take care of stretching with curvature. */

    }
  }


  // TODO:
  /*
    Actually modify point positions to get the stroke stretch effect.
  */
  return dst_curves;
}

}  // namespace blender::geometry
