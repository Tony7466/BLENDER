/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. */

/** \file
 * \ingroup bke
 */

#include "BKE_curves.hh"
#include "BKE_grease_pencil.hh"

#include "BLI_color.hh"
#include "BLI_listbase.h"
#include "BLI_math_vector_types.hh"
#include "BLI_vector.hh"

#include "DNA_gpencil_legacy_types.h"
#include "DNA_grease_pencil_types.h"

namespace blender::bke::greasepencil::convert {

void legacy_gpencil_frame_to_grease_pencil_drawing(GreasePencilDrawing &drawing, bGPDframe &gpf)
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
  SpanAttributeWriter<float> delta_times = attributes.lookup_or_add_for_write_span<float>(
      "delta_time", ATTR_DOMAIN_POINT);
  SpanAttributeWriter<float> rotations = attributes.lookup_or_add_for_write_span<float>(
      "rotation", ATTR_DOMAIN_POINT);
  SpanAttributeWriter<ColorGeometry4f> vertex_colors =
      attributes.lookup_or_add_for_write_span<ColorGeometry4f>("vertex_color", ATTR_DOMAIN_POINT);
  SpanAttributeWriter<bool> selection = attributes.lookup_or_add_for_write_span<bool>(
      ".selection", ATTR_DOMAIN_POINT);

  /* Curve Attributes. */
  SpanAttributeWriter<bool> stroke_cyclic = attributes.lookup_or_add_for_write_span<bool>(
      "cyclic", ATTR_DOMAIN_CURVE);
  /* TODO: This should be a `double` attribute. */
  SpanAttributeWriter<float> stroke_init_times = attributes.lookup_or_add_for_write_span<float>(
      "init_time", ATTR_DOMAIN_CURVE);
  SpanAttributeWriter<int8_t> stroke_start_caps = attributes.lookup_or_add_for_write_span<int8_t>(
      "start_cap", ATTR_DOMAIN_CURVE);
  SpanAttributeWriter<int8_t> stroke_end_caps = attributes.lookup_or_add_for_write_span<int8_t>(
      "end_cap", ATTR_DOMAIN_CURVE);
  SpanAttributeWriter<float> stroke_hardnesses = attributes.lookup_or_add_for_write_span<float>(
      "hardness", ATTR_DOMAIN_CURVE);
  SpanAttributeWriter<float> stroke_point_aspect_ratios =
      attributes.lookup_or_add_for_write_span<float>("point_aspect_ratio", ATTR_DOMAIN_CURVE);
  SpanAttributeWriter<float2> stroke_fill_translations =
      attributes.lookup_or_add_for_write_span<float2>("fill_translation", ATTR_DOMAIN_CURVE);
  SpanAttributeWriter<float> stroke_fill_rotations =
      attributes.lookup_or_add_for_write_span<float>("fill_rotation", ATTR_DOMAIN_CURVE);
  SpanAttributeWriter<float2> stroke_fill_scales = attributes.lookup_or_add_for_write_span<float2>(
      "fill_scale", ATTR_DOMAIN_CURVE);
  SpanAttributeWriter<ColorGeometry4f> stroke_fill_colors =
      attributes.lookup_or_add_for_write_span<ColorGeometry4f>("fill_color", ATTR_DOMAIN_CURVE);
  SpanAttributeWriter<int> stroke_materials = attributes.lookup_or_add_for_write_span<int>(
      "material_index", ATTR_DOMAIN_CURVE);

  int stroke_i = 0;
  LISTBASE_FOREACH_INDEX (bGPDstroke *, gps, &gpf.strokes, stroke_i) {
    /* TODO: check if gps->editcurve is not nullptr and parse bezier curve instead. */

    /* Write curve attributes. */
    stroke_cyclic.span[stroke_i] = (gps->flag & GP_STROKE_CYCLIC) != 0;
    /* TODO: This should be a `double` attribute. */
    stroke_init_times.span[stroke_i] = static_cast<float>(gps->inittime);
    stroke_start_caps.span[stroke_i] = static_cast<int8_t>(gps->caps[0]);
    stroke_end_caps.span[stroke_i] = static_cast<int8_t>(gps->caps[1]);
    stroke_hardnesses.span[stroke_i] = gps->hardeness;
    stroke_point_aspect_ratios.span[stroke_i] = gps->aspect_ratio[0] /
                                                max_ff(gps->aspect_ratio[1], 1e-8);
    stroke_fill_translations.span[stroke_i] = float2(gps->uv_translation);
    stroke_fill_rotations.span[stroke_i] = gps->uv_rotation;
    stroke_fill_scales.span[stroke_i] = float2(gps->uv_scale);
    stroke_fill_colors.span[stroke_i] = ColorGeometry4f(gps->vert_color_fill);
    stroke_materials.span[stroke_i] = gps->mat_nr;

    /* Write point attributes. */
    IndexRange stroke_points_range = points_by_curve[stroke_i];
    if (stroke_points_range.size() == 0) {
      continue;
    }

    Span<bGPDspoint> stroke_points{gps->points, gps->totpoints};
    MutableSpan<float3> stroke_positions = positions.slice(stroke_points_range);
    MutableSpan<float> stroke_radii = radii.span.slice(stroke_points_range);
    MutableSpan<float> stroke_opacities = opacities.span.slice(stroke_points_range);
    MutableSpan<float> stroke_deltatimes = delta_times.span.slice(stroke_points_range);
    MutableSpan<float> stroke_rotations = rotations.span.slice(stroke_points_range);
    MutableSpan<ColorGeometry4f> stroke_vertex_colors = vertex_colors.span.slice(
        stroke_points_range);
    MutableSpan<bool> stroke_selections = selection.span.slice(stroke_points_range);

    /* Do first point. */
    const bGPDspoint &first_pt = stroke_points.first();
    stroke_positions.first() = float3(first_pt.x, first_pt.y, first_pt.z);
    /* Store the actual radius of the stroke (without layer adjustment). */
    stroke_radii.first() = gps->thickness * first_pt.pressure;
    stroke_opacities.first() = first_pt.strength;
    stroke_deltatimes.first() = 0;
    stroke_rotations.first() = first_pt.uv_rot;
    stroke_vertex_colors.first() = ColorGeometry4f(first_pt.vert_color);
    stroke_selections.first() = (first_pt.flag & GP_SPOINT_SELECT) != 0;

    /* Do the rest of the points. */
    for (const int i : stroke_points.index_range().drop_back(1)) {
      const int point_i = i + 1;
      const bGPDspoint &pt_prev = stroke_points[point_i - 1];
      const bGPDspoint &pt = stroke_points[point_i];
      stroke_positions[point_i] = float3(pt.x, pt.y, pt.z);
      /* Store the actual radius of the stroke (without layer adjustment). */
      stroke_radii[point_i] = gps->thickness * pt.pressure;
      stroke_opacities[point_i] = pt.strength;
      stroke_deltatimes[point_i] = pt.time - pt_prev.time;
      stroke_rotations[point_i] = pt.uv_rot;
      stroke_vertex_colors[point_i] = ColorGeometry4f(pt.vert_color);
      stroke_selections[point_i] = (pt.flag & GP_SPOINT_SELECT) != 0;
    }
  }

  radii.finish();
  opacities.finish();
  delta_times.finish();
  rotations.finish();
  vertex_colors.finish();
  selection.finish();

  stroke_cyclic.finish();
  stroke_init_times.finish();
  stroke_start_caps.finish();
  stroke_end_caps.finish();
  stroke_hardnesses.finish();
  stroke_point_aspect_ratios.finish();
  stroke_fill_translations.finish();
  stroke_fill_rotations.finish();
  stroke_fill_scales.finish();
  stroke_fill_colors.finish();
  stroke_materials.finish();

  curves.tag_topology_changed();

  drawing.geometry.wrap() = std::move(curves);
#undef POINT_ATTRIBUTE
}

void legacy_gpencil_to_grease_pencil(GreasePencil &grease_pencil, bGPdata &gpd)
{
  using namespace blender::bke::greasepencil;

  int num_layers = 0;
  int num_drawings = 0;
  LISTBASE_FOREACH (bGPDlayer *, gpl, &gpd.layers) {
    num_drawings += BLI_listbase_count(&gpl->frames);
    num_layers++;
  }

  grease_pencil.drawing_array_size = num_drawings;
  grease_pencil.drawing_array = reinterpret_cast<GreasePencilDrawingOrReference **>(
      MEM_cnew_array<GreasePencilDrawing *>(num_drawings, __func__));

  int i = 0, layer_idx = 0;
  int active_layer_index = 0;
  LISTBASE_FOREACH_INDEX (bGPDlayer *, gpl, &gpd.layers, layer_idx) {
    Layer &new_layer = grease_pencil.root_group_for_write().add_layer(Layer(gpl->info));
    if ((gpl->flag & GP_LAYER_ACTIVE) != 0) {
      active_layer_index = layer_idx;
    }
    LISTBASE_FOREACH (bGPDframe *, gpf, &gpl->frames) {
      grease_pencil.drawing_array[i] = reinterpret_cast<GreasePencilDrawingOrReference *>(
          MEM_new<GreasePencilDrawing>(__func__));
      GreasePencilDrawing &drawing = *reinterpret_cast<GreasePencilDrawing *>(
          grease_pencil.drawing_array[i]);
      drawing.base.type = GREASE_PENCIL_DRAWING;
      drawing.runtime = MEM_new<bke::GreasePencilDrawingRuntime>(__func__);
      /* TODO: copy flag. */

      /* Convert the frame to a drawing. */
      legacy_gpencil_frame_to_grease_pencil_drawing(drawing, *gpf);

      GreasePencilFrame frame;
      frame.drawing_index = i;
      frame.type = gpf->key_type;
      new_layer.insert_frame(gpf->framenum, std::move(frame));
      i++;
    }
  }

  grease_pencil.runtime->set_active_layer_index(active_layer_index);
}

}  // namespace blender::bke::greasepencil::convert