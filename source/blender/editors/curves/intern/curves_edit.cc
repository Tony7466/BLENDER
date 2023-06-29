/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edcurves
 */

#include "BKE_curves.hh"

#include "ED_curves.hh"

namespace blender::ed::curves {

template<typename T>
Span<T> gaussian_blur(const bool is_cyclic,
                      const IndexRange curve_points,
                      const int iterations,
                      const bool keep_shape,
                      const MutableSpan<T> buffer_a,
                      const MutableSpan<T> buffer_b,
                      const IndexMask &mask)
{
  /* TODO ? Unlike for the legacy algorithm,
     we don't have a smooth cap parameter */

  MutableSpan<T> dst = buffer_a;
  dst.fill(T(0));

  /* Weight Initialization */
  const int n_half = keep_shape ? (iterations * iterations) / 8 + iterations :
                                  (iterations * iterations) / 4 + 2 * iterations + 12;
  double w = keep_shape ? 2.0 : 1.0;
  double w2 = keep_shape ?
                  (1.0 / M_SQRT3) * exp((2 * iterations * iterations) / double(n_half * 3)) :
                  0.0;
  double total_w = 0.0;

  const int nb_pts = curve_points.size();

  for (const int step : IndexRange(iterations)) {
    float w_before = float(w - w2);
    float w_after = float(w - w2);

    mask.foreach_index(256, [&](const int64_t point_index) {
      /* Compute the neighboring points */
      int64_t before = point_index - step;
      int64_t after = point_index + step;
      if (is_cyclic) {
        before = (nb_pts + ((before - curve_points.first()) % nb_pts)) % nb_pts;
        after = after % nb_pts;
      }
      else {
        before = std::max(before, curve_points.first());
        after = std::min(after, curve_points.last());
      }

      /* Add the neighboring values */
      const T bval = buffer_b[before];
      const T aval = buffer_b[after];
      const T cval = buffer_b[point_index];

      dst[point_index] += (bval - cval) * w_before + (aval - cval) * w_after;
    });

    /* Update the weight values */
    total_w += w_before;
    total_w += w_after;

    w *= (n_half + step) / double(n_half + 1 - step);
    w2 *= (n_half * 3 + step) / double(n_half * 3 + 1 - step);
  }
  total_w += w - w2;

  /* Normalize the weights */
  mask.foreach_index(256, [&](const int64_t point_index) {
    dst[point_index] = buffer_b[point_index] + dst[point_index] / total_w;
  });

  return dst.as_span();
}

bool remove_selection(bke::CurvesGeometry &curves, const eAttrDomain selection_domain)
{
  const bke::AttributeAccessor attributes = curves.attributes();
  const VArray<bool> selection = *attributes.lookup_or_default<bool>(
      ".selection", selection_domain, true);
  const int domain_size_orig = attributes.domain_size(selection_domain);
  IndexMaskMemory memory;
  const IndexMask mask = IndexMask::from_bools(selection, memory);
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

}  // namespace blender::ed::curves
