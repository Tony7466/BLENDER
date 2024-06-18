/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "FN_field.hh"

#include "BKE_attribute.hh"
#include "BKE_curves.hh"

namespace blender::geometry {

using bke::CurvesGeometry;

struct ResampleCurvesOutputAttributeIDs {
  bke::AttributeIDRef tangent_id;
  bke::AttributeIDRef normal_id;
};

/**
 * Create new curves where the selected curves have been resampled with a number of uniform-length
 * samples defined by the count field. Interpolate attributes to the result, with an accuracy that
 * depends on the curve's resolution parameter.
 *
 * \note The values provided by the #count_field are clamped to 1 or greater.
 */
CurvesGeometry resample_to_count(const CurvesGeometry &src_curves,
                                 const IndexMask &selection,
                                 const VArray<int> &counts,
                                 const ResampleCurvesOutputAttributeIDs &output_ids = {});
CurvesGeometry resample_to_count(const CurvesGeometry &src_curves,
                                 const fn::FieldContext &field_context,
                                 const fn::Field<bool> &selection_field,
                                 const fn::Field<int> &count_field,
                                 const ResampleCurvesOutputAttributeIDs &output_ids = {});

/**
 * Create new curves resampled to make each segment have the length specified by the
 * #segment_length field input, rounded to make the length of each segment the same.
 * The accuracy will depend on the curve's resolution parameter.
 */
CurvesGeometry resample_to_length(const CurvesGeometry &src_curves,
                                  const IndexMask &selection,
                                  const VArray<float> &sample_lengths,
                                  const ResampleCurvesOutputAttributeIDs &output_ids = {});
CurvesGeometry resample_to_length(const CurvesGeometry &src_curves,
                                  const fn::FieldContext &field_context,
                                  const fn::Field<bool> &selection_field,
                                  const fn::Field<float> &segment_length_field,
                                  const ResampleCurvesOutputAttributeIDs &output_ids = {});

/**
 * Evaluate each selected curve to its implicit evaluated points.
 */
CurvesGeometry resample_to_evaluated(const CurvesGeometry &src_curves,
                                     const IndexMask &selection,
                                     const ResampleCurvesOutputAttributeIDs &output_ids = {});
CurvesGeometry resample_to_evaluated(const CurvesGeometry &src_curves,
                                     const fn::FieldContext &field_context,
                                     const fn::Field<bool> &selection_field,
                                     const ResampleCurvesOutputAttributeIDs &output_ids = {});

/**
 * Resample a span of attribute values from source curves to a destination buffer.
 * \param src_curves: Curves to resample.
 * \param dst_curves: Curves to write resampled data to.
 * \param src_data: Source point attribute data to resample.
 * \param dst_data: Target point attribute span to write to.
 * \param curve_selection: Selection of curves to interpolate, unselected curves are not modified.
 * \param sample_indices: Point index in source curves to sample from for each target point.
 * \param sample_indices: Interpolation factor between start and end point for each target point.
 * \param is_evaluated_data: Source data is for evaluated points.
 */
void resample_curve_attribute(const bke::CurvesGeometry &src_curves,
                              bke::CurvesGeometry &dst_curves,
                              GSpan src_data,
                              GMutableSpan dst_data,
                              const IndexMask &curve_selection,
                              Span<int> sample_indices,
                              Span<float> sample_factors,
                              bool is_evaluated_data);

}  // namespace blender::geometry
