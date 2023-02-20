/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edcurves
 */

#include "BLI_index_mask.hh"
#include "BLI_index_mask_ops.hh"

#include "BKE_curves.hh"
#include "BKE_curves_utils.hh"

#include "ED_curves.h"

namespace blender::ed::curves {

bool remove_selection(bke::CurvesGeometry &curves, const eAttrDomain selection_domain)
{
  const bke::AttributeAccessor attributes = curves.attributes();
  const VArray<bool> selection = attributes.lookup_or_default<bool>(
      ".selection", selection_domain, true);
  const int domain_size_orig = attributes.domain_size(selection_domain);
  Vector<int64_t> indices;
  const IndexMask mask = index_mask_ops::find_indices_from_virtual_array(
      selection.index_range(), selection, 4096, indices);
  switch (selection_domain) {
    case ATTR_DOMAIN_POINT:
      curves.remove_points(mask);
      break;
    case ATTR_DOMAIN_CURVE:
      curves.remove_curves(mask);
      break;
    default:
      BLI_assert_unreachable();
  }

  return attributes.domain_size(selection_domain) != domain_size_orig;
}

bke::CurvesGeometry extend(bke::CurvesGeometry &curves, eAttrDomain selection_domain)
{
  const bke::AttributeAccessor attributes = curves.attributes();
  const OffsetIndices points_by_curve = curves.points_by_curve();

  /* Get the selection and the unselected ranges. */
  Vector<int64_t> indices;
  IndexMask selection = retrieve_selected_curves(curves, indices);
  Vector<IndexRange> unselected_ranges = selection.extract_ranges_invert(curves.points_range());

  bke::CurvesGeometry dst_curves = bke::curves::copy_only_curve_domain(curves);
  /* Use the offsets as sizes and convert back to offsets later. */
  MutableSpan<int> dst_curve_sizes = dst_curves.offsets_for_write();
  bke::curves::copy_curve_sizes(points_by_curve, unselected_ranges, dst_curve_sizes);

  /* Increase the sizes of the selected curves by one. */
  threading::parallel_for(selection.index_range(), 512, [&](IndexRange range) {
    for (const int curve_i : range) {
      dst_curve_sizes[curve_i] = points_by_curve.size(curve_i) + 1;
    }
  });

  /* Convert sizes back to offsets inplace. */
  offset_indices::accumulate_counts_to_offsets(dst_curve_sizes);
  /* Resize the points to the new amount. */
  dst_curves.resize(dst_curves.offsets().last(), dst_curves.curves_num());

  bke::MutableAttributeAccessor dst_attributes = dst_curves.attributes_for_write();
  const OffsetIndices dst_points_by_curve = dst_curves.points_by_curve();
  for (auto &attribute : bke::retrieve_attributes_for_transfer(
           attributes, dst_attributes, ATTR_DOMAIN_MASK_POINT, {})) {
    /* Transfer all the attributes for unselected curves. */
    bke::curves::copy_point_data(points_by_curve,
                                 dst_points_by_curve,
                                 unselected_ranges,
                                 attribute.src,
                                 attribute.dst.span);

    /* Transfer the attribute for all the last points of the selection. */
    attribute_math::convert_to_static_type(attribute.src.type(), [&](auto dummy) {
      using T = decltype(dummy);
      const Span<T> src = attribute.src.typed<T>();
      MutableSpan<T> dst = attribute.dst.span.typed<T>();
      threading::parallel_for(selection.index_range(), 512, [&](IndexRange range) {
        for (const int curve_i : range) {
          const int src_index_end = points_by_curve[curve_i].last();
          const int dst_index_end = dst_points_by_curve[curve_i].last();
          dst[dst_index_end] = src[src_index_end];
        }
      });
    });
    attribute.dst.finish();
  }

  return dst_curves;
}

}  // namespace blender::ed::curves
