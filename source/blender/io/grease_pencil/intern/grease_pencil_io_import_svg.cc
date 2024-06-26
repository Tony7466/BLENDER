/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_bounds.hh"
#include "BLI_math_euler_types.hh"
#include "BLI_math_matrix.hh"
#include "BLI_offset_indices.hh"
#include "BLI_path_util.h"
#include "BLI_task.hh"

#include "BKE_curves.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_report.hh"

#include "DNA_grease_pencil_types.h"
#include "DNA_space_types.h"
#include "DNA_windowmanager_types.h"

#include "GEO_resample_curves.hh"

#include "ED_grease_pencil.hh"

#include "grease_pencil_io.hh"

#include "nanosvg.h"

#include <fmt/core.h>
#include <fmt/format.h>

/** \file
 * \ingroup bgrease_pencil
 */

using blender::bke::greasepencil::Drawing;
using blender::bke::greasepencil::Layer;
using blender::bke::greasepencil::TreeNode;

namespace blender::io::grease_pencil {

static std::string get_layer_id(const NSVGshape &shape, const int prefix)
{
  return (shape.id_parent[0] == '\0') ? fmt::format("Layer_{:03d}", prefix) :
                                        fmt::format("{:s}", shape.id_parent);
}

// void GpencilImporterSVG::create_stroke(bGPdata *gpd,
//                                        bGPDframe *gpf,
//                                        NSVGshape *shape,
//                                        NSVGpath *path,
//                                        const int32_t mat_index,
//                                        const float matrix[4][4])
// {
//   const bool is_stroke = bool(shape->stroke.type);
//   const bool is_fill = bool(shape->fill.type);

//   const int edges = params_.resolution;
//   const float step = 1.0f / float(edges - 1);

//   const int totpoints = (path->npts / 3) * params_.resolution;

//   bGPDstroke *gps = BKE_gpencil_stroke_new(mat_index, totpoints, 1.0f);
//   BLI_addtail(&gpf->strokes, gps);

//   if (path->closed == '1') {
//     gps->flag |= GP_STROKE_CYCLIC;
//   }
//   if (is_stroke) {
//     gps->thickness = shape->strokeWidth * params_.scale;
//   }
//   /* Apply Fill vertex color. */
//   if (is_fill) {
//     NSVGpaint fill = shape->fill;
//     convert_color(fill.color, gps->vert_color_fill);
//     gps->fill_opacity_fac = gps->vert_color_fill[3];
//     gps->vert_color_fill[3] = 1.0f;
//   }

//   int start_index = 0;
//   for (int i = 0; i < path->npts - 1; i += 3) {
//     float *p = &path->pts[i * 2];
//     float a = 0.0f;
//     for (int v = 0; v < edges; v++) {
//       bGPDspoint *pt = &gps->points[start_index];
//       pt->strength = shape->opacity;
//       pt->pressure = 1.0f;
//       pt->z = 0.0f;
//       /* TODO(antoniov): Can be improved loading curve data instead of loading strokes. */
//       interp_v2_v2v2v2v2_cubic(&pt->x, &p[0], &p[2], &p[4], &p[6], a);

//       /* Scale from millimeters. */
//       mul_v3_fl(&pt->x, 0.001f);
//       mul_m4_v3(matrix, &pt->x);

//       /* Apply color to vertex color. */
//       if (is_fill) {
//         NSVGpaint fill = shape->fill;
//         convert_color(fill.color, pt->vert_color);
//       }
//       if (is_stroke) {
//         NSVGpaint stroke = shape->stroke;
//         convert_color(stroke.color, pt->vert_color);
//         gps->fill_opacity_fac = pt->vert_color[3];
//       }
//       pt->vert_color[3] = 1.0f;

//       a += step;
//       start_index++;
//     }
//   }

//   /* Cleanup and recalculate geometry. */
//   BKE_gpencil_stroke_merge_distance(gpd, gpf, gps, 0.001f, true);
//   BKE_gpencil_stroke_geometry_update(gpd, gps);
// }

/* Make room for curves and points from the SVG shape.
 * Returns the index range of newly added curves. */
static IndexRange extend_curves_geometry(bke::CurvesGeometry &curves, const NSVGshape &shape)
{
  const int old_curves_num = curves.curves_num();
  const int old_points_num = curves.points_num();
  const Span<int> old_offsets = curves.offsets();

  /* Count curves and points. */
  Vector<int> new_curve_offsets;
  for (NSVGpath *path = shape.paths; path; path = path->next) {
    if (path->npts <= 0) {
      continue;
    }
    /* nanosvg converts everything to bezier curves, points come in triplets. */
    const int point_num = path->npts / 3;
    new_curve_offsets.append(point_num);
  }
  if (new_curve_offsets.is_empty()) {
    return {};
  }
  new_curve_offsets.append(0);
  const OffsetIndices new_points_by_curve = offset_indices::accumulate_counts_to_offsets(
      new_curve_offsets, old_points_num);

  const IndexRange new_curves_range = {old_curves_num, new_points_by_curve.size()};
  const int curves_num = new_curves_range.one_after_last();
  const int points_num = new_points_by_curve.total_size();

  Array<int> new_offsets(curves_num + 1);
  if (old_curves_num > 0) {
    new_offsets.as_mutable_span().slice(0, old_curves_num).copy_from(old_offsets.drop_back(1));
  }
  new_offsets.as_mutable_span()
      .slice(old_curves_num, new_curve_offsets.size())
      .copy_from(new_curve_offsets);

  curves.resize(points_num, curves_num);
  curves.offsets_for_write().copy_from(new_offsets);

  /* nanosvg converts everything to Bezier curves. */
  curves.curve_types_for_write().slice(new_curves_range).fill(CURVE_TYPE_BEZIER);

  curves.tag_topology_changed();
  curves.update_curve_types();

  return new_curves_range;
}

static void shape_attributes_to_curves(bke::CurvesGeometry &curves,
                                       const NSVGshape &shape,
                                       const IndexRange curves_range)
{
  const OffsetIndices points_by_curve = curves.points_by_curve();
  MutableSpan<float3> positions = curves.positions_for_write();

  int curve_index = curves_range.start();
  for (NSVGpath *path = shape.paths; path; path = path->next, ++curve_index) {
    /* 2D vectors in triplets: [control point, left handle, right handle]. */
    const Span<float2> svg_path_data = Span<float>(path->pts, 2 * path->npts).cast<float2>();

    const IndexRange points = points_by_curve[curve_index];
    for (const int i : points.index_range()) {
      const int point_index = points[i];
      positions[point_index] = float3(svg_path_data[i * 3], 0.0f);
    }
  }
}

static void shift_to_bounds_center(GreasePencil &grease_pencil)
{
  const std::optional<Bounds<float3>> bounds = [&]() {
    std::optional<Bounds<float3>> bounds;
    for (GreasePencilDrawingBase *drawing_base : grease_pencil.drawings()) {
      if (drawing_base->type != GP_DRAWING) {
        continue;
      }
      Drawing &drawing = reinterpret_cast<GreasePencilDrawing *>(drawing_base)->wrap();
      bounds = bounds::merge(bounds, drawing.strokes().bounds_min_max());
    }
    return bounds;
  }();
  if (!bounds) {
    return;
  }
  const float3 offset = -bounds->center();

  for (GreasePencilDrawingBase *drawing_base : grease_pencil.drawings()) {
    if (drawing_base->type != GP_DRAWING) {
      continue;
    }
    Drawing &drawing = reinterpret_cast<GreasePencilDrawing *>(drawing_base)->wrap();

    MutableSpan<float3> positions = drawing.strokes_for_write().positions_for_write();
    threading::parallel_for(positions.index_range(), 4096, [&](const IndexRange range) {
      for (const int i : range) {
        positions[i] += offset;
      }
    });

    drawing.tag_positions_changed();
  }
}

bool SVGImporter::read(StringRefNull filepath)
{
  NSVGimage *svg_data = nullptr;
  svg_data = nsvgParseFromFile(filepath.c_str(), "mm", 96.0f);
  if (svg_data == nullptr) {
    BKE_report(context_.reports, RPT_ERROR, "Could not open SVG");
    return false;
  }

  /* Create grease pencil object. */
  char filename[FILE_MAX];
  BLI_path_split_file_part(filepath.c_str(), filename, ARRAY_SIZE(filename));
  object_ = create_object(filename);
  if (object_ == nullptr) {
    BKE_report(context_.reports, RPT_ERROR, "Unable to create new object");
    nsvgDelete(svg_data);
    return false;
  }
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object_->data);

  /* Grease pencil is rotated 90 degrees in X axis by default. */
  const float4x4 matrix = math::scale(math::from_rotation<float4x4>(math::EulerXYZ(-90, 0, 0)),
                                      float3(scale_));

  /* Loop all shapes. */
  std::string prv_id = "*";
  int prefix = 0;
  for (NSVGshape *shape = svg_data->shapes; shape; shape = shape->next) {
    std::string layer_id = get_layer_id(*shape, prefix);
    if (prv_id != layer_id) {
      prefix++;
      layer_id = get_layer_id(*shape, prefix);
      prv_id = layer_id;
    }

    /* Check if the layer exist and create if needed. */
    Layer &layer = [&]() -> Layer & {
      TreeNode *layer_node = grease_pencil.find_node_by_name(layer_id);
      if (layer_node && layer_node->is_layer()) {
        return layer_node->as_layer();
      }

      Layer &layer = grease_pencil.add_layer(layer_id);
      layer.as_node().flag |= GP_LAYER_TREE_NODE_USE_LIGHTS;
      return layer;
    }();

    /* Check frame. */
    Drawing *drawing = grease_pencil.get_drawing_at(layer, frame_number_);
    if (drawing == nullptr) {
      drawing = grease_pencil.insert_frame(layer, frame_number_);
      if (!drawing) {
        continue;
      }
    }
    bke::CurvesGeometry &curves = drawing->strokes_for_write();

    /* Create materials. */
    const bool is_fill = bool(shape->fill.type);
    const bool is_stroke = bool(shape->stroke.type) || !is_fill;
    const StringRefNull mat_name = (is_stroke ? (is_fill ? "Both" : "Stroke") : "Fill");
    const int32_t mat_index = create_material(mat_name, is_stroke, is_fill);

    const IndexRange new_curves_range = extend_curves_geometry(curves, *shape);
    if (new_curves_range.is_empty()) {
      continue;
    }
    shape_attributes_to_curves(curves, *shape, new_curves_range);

    /* Convert Bezier curves to poly curves.
     * XXX This will not be necessary once Bezier curves are fully supported in grease pencil. */
    curves = blender::geometry::resample_to_count(
        std::move(curves),
        new_curves_range,
        VArray<int>::ForSingle(resolution_, curves.curves_num()),
        {});

    drawing->strokes_for_write() = std::move(curves);
  }

  /* Free SVG memory. */
  nsvgDelete(svg_data);

  /* Calculate bounding box and move all points to new origin center. */
  if (recenter_bounds_) {
    shift_to_bounds_center(grease_pencil);
  }

  return true;
}

}  // namespace blender::io::grease_pencil
