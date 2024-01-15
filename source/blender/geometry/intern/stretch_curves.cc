/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include "BLI_array_utils.hh"
#include "BLI_length_parameterize.hh"

#include "BKE_attribute.hh"
#include "BKE_attribute_math.hh"
#include "BKE_curves.hh"
#include "BKE_curves_utils.hh"
#include "BKE_geometry_set.hh"

#include "GEO_trim_curves.hh"

namespace blender::geometry {

bke::CurvesGeometry stretch_curves(const bke::CurvesGeometry &src_curves,
                                   const IndexMask &selection,
                                   const VArray<float> &start_lengths,
                                   const VArray<float> &end_lengths,
                                   const VArray<float> &dist,
                                   const VArray<float> &overshoot_fac,
                                   const VArray<bool> &follow_curvature,
                                   const VArray<int> &extra_point_count_per_side,
                                   const VArray<float> &segment_influence,
                                   const VArray<float> &max_angle,
                                   const VArray<bool> &invert_curvature,
                                   const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  const int src_curves_num = src_curves.curves_num();
  Array<bool> do_curves(src_curves_num);
  Array<bool> do_starts(src_curves_num);
  Array<bool> do_ends(src_curves_num);
  selection.to_bools(do_curves.as_mutable_span());

  const OffsetIndices<int> points_by_curve = src_curves.points_by_curve();
  Array<int> dst_point_counts(src_curves_num + 1);
  
  /* Count how many points we need. */
  for(const int curve : do_curves.index_range()){
    int point_count = points_by_curve[curve].size();
    dst_point_counts[curve]=point_count;
    /* Curve not suitable for stretching... */
    if(point_count <= 2 ){
      continue;
    }
    const int do_start = (start_lengths[curve]>0)?1:0;
    const int do_end = (end_lengths[curve]>0)?1:0;
    dst_point_counts[curve]+=extra_point_count_per_side * do_start;
    dst_point_counts[curve]+=extra_point_count_per_side * do_end;
    do_starts[curve]=do_start;
    do_ends[curve]=do_end;
  }
  offset_indices::accumulate_counts_to_offsets(dst_point_counts.as_mutable_span());
  int target_point_count = dst_point_counts.last();

  /* Make dest to source map for points. */
  Array<int> dst_to_src_point(target_point_count);
  for(const int curve : do_curves.index_range()){
    const int point_count = points_by_curve[curve].size();
    int local_front = 0;
    MutableSpan<int> new_points = dst_to_src_point.as_mutable_span().slice(
      dst_point_counts[curve], dst_point_counts[curve+1]-dst_point_counts[curve]);
    if(do_starts[curve]){
      MutableSpan<int> start_points = new_points.slice(0,extra_point_count_per_side);
      start_points.fill(points_by_curve[curve].first());
      local_front = extra_point_count_per_side;
    }
    if(do_ends[curve]){
      MutableSpan<int> end_points = new_points.slice(
        new_points.size()-extra_point_count_per_side,extra_point_count_per_side);
      end_points.fill(points_by_curve[curve].last());
    }
    MutableSpan<int> original_points = new_points.slice(local_front, point_count);
    for(const int point_i : original_points.index_range()){
      original_points[point_i] = points_by_curve[curve][point_i];
    }
  }

  /* Make dest to source map for curves, nothing changes so just [0,1,2,3,...]. */
  Array<int> dst_to_src_curve(src_curves_num);
  dst_to_src_curve.fill(1);
  dst_to_src_curve[0] = 0;
  offset_indices::accumulate_counts_to_offsets(dst_to_src_curve.as_mutable_span());

  bke::CurvesGeometry dst_curves(target_point_count, src_curves_num);

  MutableSpan<int> new_curve_offsets = dst_curves.offsets_for_write();
  array_utils::copy(dst_point_counts.as_span(), new_curve_offsets);

  const bke::AttributeAccessor src_attributes = src_curves.attributes();
  bke::MutableAttributeAccessor dst_attributes = dst_curves.attributes_for_write();

  /* Transfer curve attributes. */
  gather_attributes(
      src_attributes, bke::AttrDomain::Curve, {}, {}, dst_to_src_curve, dst_attributes);

  /* Transfer point attributes. */
  gather_attributes(
      src_attributes, bke::AttrDomain::Point, {}, {}, dst_to_src_point, dst_attributes);

  dst_curves.update_curve_types();
  dst_curves.remove_attributes_based_on_types();


  // TODO:
  /*
    Actually modify point positions to get the stroke stretch effect.
  */
  return src_curves;
}

}  // namespace blender::geometry
