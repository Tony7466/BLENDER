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

void legacy_gpencil_frame_to_curves_geometry(GreasePencilDrawing &drawing, bGPDframe &gpf)
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
  curves.fill_curve_types(CURVE_TYPE_POLY);

  /* Point Attributes. */
  MutableSpan<float3> positions = curves.positions_for_write();
  SpanAttributeWriter<float> radii = attributes.lookup_or_add_for_write_span<float>(
      "radius", ATTR_DOMAIN_POINT);
  SpanAttributeWriter<float> opacities = attributes.lookup_or_add_for_write_span<float>(
      "opacity", ATTR_DOMAIN_POINT);
  SpanAttributeWriter<bool> selection = attributes.lookup_or_add_for_write_span<bool>(
      ".selection", ATTR_DOMAIN_POINT);

  /* Curve Attributes. */
  SpanAttributeWriter<bool> curve_cyclic = attributes.lookup_or_add_for_write_span<bool>(
      "cyclic", ATTR_DOMAIN_CURVE);
  SpanAttributeWriter<int> curve_materials = attributes.lookup_or_add_for_write_span<int>(
      "material_index", ATTR_DOMAIN_CURVE);

  int i = 0;
  LISTBASE_FOREACH_INDEX (bGPDstroke *, gps, &gpf.strokes, i) {
    /* TODO: check if gps->editcurve is not nullptr and parse bezier curve instead. */

    /* Write curve attributes. */
    curve_cyclic.span[i] = (gps->flag & GP_STROKE_CYCLIC) != 0;
    curve_materials.span[i] = gps->mat_nr;

    /* Write point attributes. */
    IndexRange curve_points = points_by_curve[i];
    Span<bGPDspoint> stroke_points{gps->points, gps->totpoints};

    MutableSpan<float3> curve_positions = positions.slice(curve_points);
    MutableSpan<float> curve_radii = radii.span.slice(curve_points);
    MutableSpan<float> curve_opacities = opacities.span.slice(curve_points);
    MutableSpan<bool> curve_selections = selection.span.slice(curve_points);

    for (const int j : stroke_points.index_range()) {
      const bGPDspoint &pt = stroke_points[j];
      curve_positions[j] = float3(pt.x, pt.y, pt.z);
      /* For now, store the actual radius of the stroke (without layer adjustment). */
      curve_radii[j] = gps->thickness * pt.pressure;
      curve_opacities[j] = pt.strength;
      curve_selections[j] = (pt.flag & GP_SPOINT_SELECT) != 0;
    }
  }

  radii.finish();
  opacities.finish();
  selection.finish();

  curve_cyclic.finish();
  curve_materials.finish();

  curves.tag_topology_changed();

  drawing.geometry.wrap() = std::move(curves);
}

void legacy_gpencil_to_grease_pencil(GreasePencil &grease_pencil, bGPdata &gpd)
{
  using namespace blender::bke::gpencil;

  int num_layers = 0;
  int num_drawings = 0;
  LISTBASE_FOREACH (bGPDlayer *, gpl, &gpd.layers) {
    num_drawings += BLI_listbase_count(&gpl->frames);
    num_layers++;
  }

  grease_pencil.drawing_array_size = num_drawings;
  grease_pencil.drawing_array = reinterpret_cast<GreasePencilDrawingOrReference **>(
      MEM_cnew_array<GreasePencilDrawing *>(num_drawings, __func__));

  int i = 0;
  LISTBASE_FOREACH (bGPDlayer *, gpl, &gpd.layers) {
    Layer &new_layer = grease_pencil.root_group().add_layer(Layer(gpl->info));
    LISTBASE_FOREACH (bGPDframe *, gpf, &gpl->frames) {
      grease_pencil.drawing_array[i] = reinterpret_cast<GreasePencilDrawingOrReference *>(
          MEM_new<GreasePencilDrawing>(__func__));
      GreasePencilDrawing &drawing = *reinterpret_cast<GreasePencilDrawing *>(
          grease_pencil.drawing_array[i]);
      drawing.base.type = GREASE_PENCIL_DRAWING;
      drawing.runtime = MEM_new<bke::GreasePencilDrawingRuntime>(__func__);
      /* TODO: copy flag. */

      /* Convert the frame to a drawing. */
      legacy_gpencil_frame_to_curves_geometry(drawing, *gpf);

      GreasePencilFrame frame;
      frame.drawing_index = i;
      frame.type = gpf->key_type;
      new_layer.insert_frame(gpf->framenum, std::move(frame));
      i++;
    }
  }
}

}  // namespace blender::bke::gpencil::convert