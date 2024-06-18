/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute_math.hh"
#include "BKE_curves.hh"

#include "BLI_assert.h"
#include "BLI_length_parameterize.hh"
#include "BLI_math_vector.hh"
#include "BLI_offset_indices.hh"
#include "BLI_task.hh"

#include "DNA_customdata_types.h"

#include "GEO_interpolate_curves.hh"
#include "GEO_resample_curves.hh"

namespace blender::geometry {

/**
 * Return true if the attribute should be copied/interpolated to the result curves.
 * Don't output attributes that correspond to curve types that have no curves in the result.
 */
static bool interpolate_attribute_to_curves(const bke::AttributeIDRef &attribute_id,
                                            const std::array<int, CURVE_TYPES_NUM> &type_counts)
{
  if (attribute_id.is_anonymous()) {
    return true;
  }
  if (ELEM(attribute_id.name(),
           "handle_type_left",
           "handle_type_right",
           "handle_left",
           "handle_right"))
  {
    return type_counts[CURVE_TYPE_BEZIER] != 0;
  }
  if (ELEM(attribute_id.name(), "nurbs_weight")) {
    return type_counts[CURVE_TYPE_NURBS] != 0;
  }
  return true;
}

/**
 * Return true if the attribute should be copied to poly curves.
 */
static bool interpolate_attribute_to_poly_curve(const bke::AttributeIDRef &attribute_id)
{
  static const Set<StringRef> no_interpolation{{
      "handle_type_left",
      "handle_type_right",
      "handle_right",
      "handle_left",
      "nurbs_weight",
  }};
  return !no_interpolation.contains(attribute_id.name());
}

/**
 * Retrieve spans from source and result attributes.
 */
static void retrieve_attribute_spans(const Span<bke::AttributeIDRef> ids,
                                     const CurvesGeometry &src_from_curves,
                                     const CurvesGeometry &src_to_curves,
                                     CurvesGeometry &dst_curves,
                                     Vector<GSpan> &src_from,
                                     Vector<GSpan> &src_to,
                                     Vector<GMutableSpan> &dst,
                                     Vector<bke::GSpanAttributeWriter> &dst_attributes)
{
  const bke::AttributeAccessor src_from_attributes = src_from_curves.attributes();
  const bke::AttributeAccessor src_to_attributes = src_to_curves.attributes();
  for (const int i : ids.index_range()) {
    eCustomDataType data_type;

    const GVArray src_from_attribute = *src_from_attributes.lookup(ids[i], bke::AttrDomain::Point);
    if (src_from_attribute) {
      data_type = bke::cpp_type_to_custom_data_type(src_from_attribute.type());

      const GVArray src_to_attribute = *src_to_attributes.lookup(
          ids[i], bke::AttrDomain::Point, data_type);

      src_from.append(src_from_attribute.get_internal_span());
      src_to.append(src_to_attribute ? src_to_attribute.get_internal_span() : GSpan{});
    }
    else {
      const GVArray src_to_attribute = *src_to_attributes.lookup(ids[i], bke::AttrDomain::Point);
      /* Attribute should exist on at least one of the geometries. */
      BLI_assert(src_to_attribute);

      data_type = bke::cpp_type_to_custom_data_type(src_to_attribute.type());

      src_from.append(GSpan{});
      src_to.append(src_to_attribute.get_internal_span());
    }

    bke::GSpanAttributeWriter dst_attribute =
        dst_curves.attributes_for_write().lookup_or_add_for_write_only_span(
            ids[i], bke::AttrDomain::Point, data_type);
    dst.append(dst_attribute.span);
    dst_attributes.append(std::move(dst_attribute));
  }
}

struct AttributesForInterpolation : NonCopyable, NonMovable {
  Vector<GSpan> src_from;
  Vector<GSpan> src_to;
  Vector<GMutableSpan> dst;

  Vector<bke::GSpanAttributeWriter> dst_attributes;
};

/**
 * Gather a set of all generic attribute IDs to copy to the result curves.
 */
static void gather_point_attributes_to_interpolate(const CurvesGeometry &from_curves,
                                                   const CurvesGeometry &to_curves,
                                                   CurvesGeometry &dst_curves,
                                                   AttributesForInterpolation &result)
{
  VectorSet<bke::AttributeIDRef> ids;
  auto add_attribute = [&](const bke::AttributeIDRef &id, const bke::AttributeMetaData meta_data) {
    if (meta_data.domain != bke::AttrDomain::Point) {
      return true;
    }
    if (meta_data.data_type == CD_PROP_STRING) {
      return true;
    }
    if (!interpolate_attribute_to_curves(id, dst_curves.curve_type_counts())) {
      return true;
    }
    if (interpolate_attribute_to_poly_curve(id)) {
      ids.add(id);
    }
    return true;
  };

  from_curves.attributes().for_all(add_attribute);
  to_curves.attributes().for_all(add_attribute);

  /* Position is handled differently since it has non-generic interpolation for Bezier
   * curves and because the evaluated positions are cached for each evaluated point. */
  ids.remove_contained("position");

  retrieve_attribute_spans(ids,
                           from_curves,
                           to_curves,
                           dst_curves,
                           result.src_from,
                           result.src_to,
                           result.dst,
                           result.dst_attributes);
}

void interpolate_curves(const CurvesGeometry &from_curves,
                        const CurvesGeometry &to_curves,
                        const Span<int> from_curve_indices,
                        const Span<int> to_curve_indices,
                        const IndexMask &selection,
                        VArray<bool> curve_flip_direction,
                        const float mix_factor,
                        CurvesGeometry &dst_curves)
{
  BLI_assert(from_curve_indices.size() == selection.size());
  BLI_assert(to_curve_indices.size() == selection.size());

  if (from_curves.curves_range().is_empty() || to_curves.curves_range().is_empty()) {
    return;
  }

  const VArray<bool> from_curves_cyclic = from_curves.cyclic();
  const VArray<bool> to_curves_cyclic = to_curves.cyclic();
  const Span<float3> from_evaluated_positions = from_curves.evaluated_positions();
  const Span<float3> to_evaluated_positions = to_curves.evaluated_positions();

  /* All resampled curves are poly curves. */
  dst_curves.fill_curve_types(selection, CURVE_TYPE_POLY);

  MutableSpan<float3> dst_positions = dst_curves.positions_for_write();

  AttributesForInterpolation attributes;
  gather_point_attributes_to_interpolate(from_curves, to_curves, dst_curves, attributes);

  from_curves.ensure_evaluated_lengths();
  to_curves.ensure_evaluated_lengths();

  /* Sampling arbitrary attributes works by first interpolating them to the curve's standard
   * "evaluated points" and then interpolating that result with the uniform samples. This is
   * potentially wasteful when down-sampling a curve to many fewer points. There are two possible
   * solutions: only sample the necessary points for interpolation, or first sample curve
   * parameter/segment indices and evaluate the curve directly. */
  Array<int> from_sample_indices(dst_curves.points_num());
  Array<int> to_sample_indices(dst_curves.points_num());
  Array<float> from_sample_factors(dst_curves.points_num());
  Array<float> to_sample_factors(dst_curves.points_num());

  const OffsetIndices dst_points_by_curve = dst_curves.points_by_curve();

  /* Use a "for each group of curves: for each attribute: for each curve" pattern to work on
   * smaller sections of data that ideally fit into CPU cache better than simply one attribute at a
   * time or one curve at a time. */
  selection.foreach_segment(
      GrainSize(512), [&](const IndexMaskSegment selection_segment, const int64_t segment_pos) {
        const IndexRange segment_range = selection_segment.index_range().shift(
            segment_pos + selection_segment.offset());
        const Span<int> from_indices_segment = from_curve_indices.slice(segment_pos,
                                                                        selection_segment.size());
        const Span<int> to_indices_segment = to_curve_indices.slice(segment_pos,
                                                                    selection_segment.size());

        /* Gather uniform samples based on the accumulated lengths of the original curve. */
        for (const int i : selection_segment.index_range()) {
          const int i_dst_curve = selection_segment[i];
          const int i_from_curve = from_indices_segment[i];
          const int i_to_curve = to_indices_segment[i];
          const IndexRange dst_points = dst_points_by_curve[i_dst_curve];
          const Span<float> from_lengths = from_curves.evaluated_lengths_for_curve(
              i_from_curve, from_curves_cyclic[i_from_curve]);
          const Span<float> to_lengths = to_curves.evaluated_lengths_for_curve(
              i_to_curve, to_curves_cyclic[i_to_curve]);

          if (from_lengths.is_empty()) {
            /* Handle curves with only one evaluated point. */
            from_sample_indices.as_mutable_span().slice(dst_points).fill(0);
            from_sample_factors.as_mutable_span().slice(dst_points).fill(0.0f);
          }
          else {
            length_parameterize::sample_uniform(
                from_lengths,
                !from_curves_cyclic[i_from_curve],
                false,
                from_sample_indices.as_mutable_span().slice(dst_points),
                from_sample_factors.as_mutable_span().slice(dst_points));
          }
          if (to_lengths.is_empty()) {
            /* Handle curves with only one evaluated point. */
            to_sample_indices.as_mutable_span().slice(dst_points).fill(0);
            to_sample_factors.as_mutable_span().slice(dst_points).fill(0.0f);
          }
          else {
            if (curve_flip_direction[i_dst_curve]) {
              length_parameterize::sample_uniform(
                  to_lengths,
                  !to_curves_cyclic[i_to_curve],
                  true,
                  to_sample_indices.as_mutable_span().slice(dst_points),
                  to_sample_factors.as_mutable_span().slice(dst_points));
            }
            else {
              length_parameterize::sample_uniform(
                  to_lengths,
                  !to_curves_cyclic[i_to_curve],
                  false,
                  to_sample_indices.as_mutable_span().slice(dst_points),
                  to_sample_factors.as_mutable_span().slice(dst_points));
            }
          }
        }

        /* For every attribute, evaluate attributes from every curve in the range in the original
         * curve's "evaluated points", then use linear interpolation to sample to the result. */
        for (const int i_attribute : attributes.dst.index_range()) {
          if (attributes.src_from[i_attribute].is_empty()) {
            BLI_assert(!attributes.src_to[i_attribute].is_empty());
            resample_curve_attribute(to_curves,
                                     dst_curves,
                                     attributes.src_to[i_attribute],
                                     attributes.dst[i_attribute],
                                     segment_range,
                                     to_sample_indices,
                                     to_sample_factors,
                                     1.0f,
                                     false);
          }
          else if (attributes.src_to[i_attribute].is_empty()) {
            resample_curve_attribute(from_curves,
                                     dst_curves,
                                     attributes.src_from[i_attribute],
                                     attributes.dst[i_attribute],
                                     segment_range,
                                     from_sample_indices,
                                     from_sample_factors,
                                     1.0f,
                                     false);
          }
          else {
            /* Resample 'from' curves, then mix with 'to' curves. */
            resample_curve_attribute(from_curves,
                                     dst_curves,
                                     attributes.src_from[i_attribute],
                                     attributes.dst[i_attribute],
                                     segment_range,
                                     from_sample_indices,
                                     from_sample_factors,
                                     1.0f,
                                     false);
            resample_curve_attribute(to_curves,
                                     dst_curves,
                                     attributes.src_to[i_attribute],
                                     attributes.dst[i_attribute],
                                     segment_range,
                                     to_sample_indices,
                                     to_sample_factors,
                                     mix_factor,
                                     false);
          }
        }

        /* Interpolate the evaluated positions to the resampled curves. */
        resample_curve_attribute(from_curves,
                                 dst_curves,
                                 from_evaluated_positions,
                                 dst_positions,
                                 segment_range,
                                 from_sample_indices,
                                 from_sample_factors,
                                 1.0f,
                                 false);
        resample_curve_attribute(to_curves,
                                 dst_curves,
                                 to_evaluated_positions,
                                 dst_positions,
                                 segment_range,
                                 to_sample_indices,
                                 to_sample_factors,
                                 mix_factor,
                                 false);
      });

  for (bke::GSpanAttributeWriter &attribute : attributes.dst_attributes) {
    attribute.finish();
  }
}

CurvesGeometry interpolate_curves(const CurvesGeometry &from_curves,
                                  const CurvesGeometry &to_curves,
                                  Span<int> from_curve_indices,
                                  Span<int> to_curve_indices,
                                  const IndexMask &selection,
                                  VArray<int> dst_curve_counts,
                                  VArray<bool> curve_flip_direction,
                                  const float mix_factor)
{
  if (from_curves.curves_range().is_empty() || to_curves.curves_range().is_empty()) {
    return {};
  }

  const int dst_curve_num = from_curve_indices.size();
  BLI_assert(to_curve_indices.size() == dst_curve_num);
  BLI_assert(dst_curve_counts.size() == dst_curve_num);
  Array<int> dst_curve_offsets(dst_curve_num + 1);
  dst_curve_counts.materialize_to_uninitialized(dst_curve_offsets);
  offset_indices::accumulate_counts_to_offsets(dst_curve_offsets);
  const int dst_point_num = dst_curve_offsets.last();

  CurvesGeometry dst_curves(dst_point_num, dst_curve_num);
  /* Offsets are empty when there are no curves. */
  if (dst_curve_num > 0) {
    dst_curves.offsets_for_write().copy_from(dst_curve_offsets);
  }

  interpolate_curves(from_curves,
                     to_curves,
                     from_curve_indices,
                     to_curve_indices,
                     selection,
                     curve_flip_direction,
                     mix_factor,
                     dst_curves);

  return dst_curves;
}

}  // namespace blender::geometry
