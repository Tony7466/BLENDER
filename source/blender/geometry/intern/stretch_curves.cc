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
                                   const VArray<int> &extra_point_count,
                                   const VArray<float> &segment_influence,
                                   const VArray<float> &max_angle,
                                   const VArray<bool> &invert_curvature,
                                   const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  // TODO:
  /*
    We need to build a new CurvesGeometry with all the extra points added and also keep a mapping info
    so we can transfer attributes to new indicies.

    The main think I'm not sure about now is how to map the attributes from the old stroke to the new one,
    maybe we do it manually like `point_merge_by_distance.cc` or `trim_attribute_linear` in `trim_curves.cc`.

    (I think there's a specific offset map kind of thing that can do this automatically?)

    This function should not involve interpolation of the point attributes since we are adding points at the
    end of the curves and not touching anything in the middle.
  */
  return src_curves;
}

}  // namespace blender::geometry
