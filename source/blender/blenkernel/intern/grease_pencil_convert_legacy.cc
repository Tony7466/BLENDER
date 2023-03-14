/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include "BKE_curves.hh"
#include "BKE_grease_pencil.hh"

#include "BLI_listbase.h"
#include "BLI_math_vector_types.hh"
#include "BLI_vector.hh"

#include "DNA_gpencil_legacy_types.h"
#include "DNA_grease_pencil_types.h"

namespace blender::bke::gpencil::convert {

CurvesGeometry &legacy_gpencil_frame_to_curves_geometry(bGPDframe &gpf)
{
  /* Get the number of points, number of strokes and the offsets for each stroke. */
  Vector<int> offsets;
  offsets.append(0);
  int num_strokes = 0;
  int num_points = 0;
  LISTBASE_FOREACH (bGPDstroke *, gps, &gpf.strokes) {
    num_points += gps->totpoints;
    offsets.append(num_points);
    num_strokes++;
  }

  /* Create a new CurvesGeometry. */
  CurvesGeometry curves{num_points, num_strokes};
  curves.offsets_for_write().copy_from(offsets);
  OffsetIndices<int> points_by_curve = curves.points_by_curve();
  MutableAttributeAccessor attributes = curves.attributes_for_write();

  /* All strokes are poly curves. */
  curves.curve_types_for_write().fill(CURVE_TYPE_POLY);

  /* Point Attributes. */
  MutableSpan<float3> positions = curves.positions_for_write();
  MutableSpan<float> radii =
      attributes.lookup_or_add_for_write_span<float>("radius", ATTR_DOMAIN_POINT).span;
  MutableSpan<float> opacities =
      attributes.lookup_or_add_for_write_span<float>("opacity", ATTR_DOMAIN_POINT).span;
  MutableSpan<bool> selection =
      attributes.lookup_or_add_for_write_span<bool>(".selection", ATTR_DOMAIN_POINT).span;

  /* Curve Attributes. */
  MutableSpan<bool> curve_cyclic =
      attributes.lookup_or_add_for_write_span<bool>("cyclic", ATTR_DOMAIN_CURVE).span;
  MutableSpan<int> curve_materials =
      attributes.lookup_or_add_for_write_span<int>("material_index", ATTR_DOMAIN_CURVE).span;

  int i = 0;
  LISTBASE_FOREACH_INDEX (bGPDstroke *, gps, &gpf.strokes, i) {
    /* Write curve attributes. */
    curve_cyclic[i] = (gps->flag & GP_STROKE_CYCLIC) != 0;
    curve_materials[i] = gps->mat_nr;

    /* Write point attributes. */
    IndexRange curve_points = points_by_curve[i];
    Span<bGPDspoint> stroke_points{gps->points, gps->totpoints};

    MutableSpan<float3> curve_positions = positions.slice(curve_points);
    MutableSpan<float> curve_radii = radii.slice(curve_points);
    MutableSpan<float> curve_opacities = opacities.slice(curve_points);
    MutableSpan<bool> curve_selections = selection.slice(curve_points);

    for (const int j : stroke_points.index_range()) {
      const bGPDspoint &pt = stroke_points[j];
      curve_positions[j] = float3(pt.x, pt.y, pt.z);
      curve_radii[j] = pt.pressure;
      curve_opacities[j] = pt.strength;
      curve_selections[j] = (pt.flag & GP_SPOINT_SELECT) != 0;
    }
  }

  return curves;
}

}  // namespace blender::bke::gpencil::convert