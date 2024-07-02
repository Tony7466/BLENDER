/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute.hh"
#include "BKE_curves.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_material.h"
#include "BKE_scene.hh"

#include "DEG_depsgraph_query.hh"

#include "DNA_grease_pencil_types.h"
#include "DNA_material_types.h"
#include "DNA_scene_types.h"

#include "ED_view3d.hh"

#include "grease_pencil_io.hh"
#include "grease_pencil_io_intern.hh"

#include "hpdf.h"

#include <iostream>

/** \file
 * \ingroup bgrease_pencil
 */

namespace blender::io::grease_pencil {

class PDFExporter : public GreasePencilExporter {
 public:
  using GreasePencilExporter::GreasePencilExporter;

  HPDF_Doc pdf_;
  HPDF_Page page_;

  bool export_scene(Scene &scene, StringRefNull filepath);
  void export_grease_pencil_objects(int frame_number);
  void export_grease_pencil_layer(const Object &object,
                                  const GreasePencil &grease_pencil,
                                  const bke::greasepencil::Layer &layer,
                                  const int frame_number);

  bool create_document();
  bool add_page();

  bool write_to_file(StringRefNull filepath);
};

static bool is_selected_frame(const GreasePencil &grease_pencil, const int frame_number)
{
  for (const bke::greasepencil::Layer *layer : grease_pencil.layers()) {
    if (layer->is_visible()) {
      const GreasePencilFrame *frame = layer->frame_at(frame_number);
      if (frame->is_selected()) {
        return true;
      }
    }
  }
  return false;
}

bool PDFExporter::export_scene(Scene &scene, StringRefNull filepath)
{
  bool result = false;
  // Object *ob_eval_ = (Object *)DEG_get_evaluated_id(depsgraph, &ob->id);
  // bGPdata *gpd_eval = (bGPdata *)ob_eval_->data;
  Object &ob_eval = *DEG_get_evaluated_object(context_.depsgraph, params_.object);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(ob_eval.data);

  if (!create_document()) {
    return false;
  }

  switch (params_.frame_mode) {
    case ExportParams::FrameMode::Active: {
      this->prepare_camera_params(scene, true);
      this->add_page();
      this->export_grease_pencil_objects(scene.r.cfra);
      result = this->write_to_file(filepath);
      break;
    }
    case ExportParams::FrameMode::Selected: {
      case ExportParams::FrameMode::Scene:
        const bool only_selected = (params_.frame_mode == ExportParams::FrameMode::Selected);
        const int orig_frame = scene.r.cfra;
        for (int frame_number = scene.r.sfra; frame_number <= scene.r.efra; frame_number++) {
          if (only_selected && !is_selected_frame(grease_pencil, frame_number)) {
            continue;
          }

          scene.r.cfra = frame_number;
          BKE_scene_graph_update_for_newframe(context_.depsgraph);

          this->prepare_camera_params(scene, true);
          this->add_page();
          this->export_grease_pencil_objects(frame_number);
        }

        result = this->write_to_file(filepath);

        /* Back to original frame. */
        scene.r.cfra = orig_frame;
        BKE_scene_camera_switch_update(&scene);
        BKE_scene_graph_update_for_newframe(context_.depsgraph);
        break;
    }
    default:
      break;
  }

  return result;
}

void PDFExporter::export_grease_pencil_objects(const int frame_number)
{
  Vector<ObjectInfo> objects = retrieve_objects();

  for (const ObjectInfo &info : objects) {
    const Object *ob = info.object;

    /* Use evaluated version to get strokes with modifiers. */
    Object *ob_eval = DEG_get_evaluated_object(context_.depsgraph, const_cast<Object *>(ob));
    BLI_assert(ob_eval->type == OB_GREASE_PENCIL);
    const GreasePencil *grease_pencil_eval = static_cast<const GreasePencil *>(ob_eval->data);

    for (const bke::greasepencil::Layer *layer : grease_pencil_eval->layers()) {
      export_grease_pencil_layer(*ob_eval, *grease_pencil_eval, *layer, frame_number);
    }
  }
}

//   LISTBASE_FOREACH (bGPDlayer *, gpl, &gpd_eval->layers) {
//     if (gpl->flag & GP_LAYER_HIDE) {
//       continue;
//     }
//     prepare_layer_export_matrix(ob, gpl);

//     bGPDframe *gpf = gpl->actframe;
//     if ((gpf == nullptr) || (gpf->strokes.first == nullptr)) {
//       continue;
//     }

//     LISTBASE_FOREACH (bGPDstroke *, gps, &gpf->strokes) {
//       if (gps->totpoints < 2) {
//         continue;
//       }
//       if (!ED_gpencil_stroke_material_visible(ob, gps)) {
//         continue;
//       }
//       /* Skip invisible lines. */
//       prepare_stroke_export_colors(ob, gps);
//       const float fill_opacity = fill_color_[3] * gpl->opacity;
//       const float stroke_opacity = stroke_color_[3] * stroke_average_opacity_get() *
//                                    gpl->opacity;
//       if ((fill_opacity < GPENCIL_ALPHA_OPACITY_THRESH) &&
//           (stroke_opacity < GPENCIL_ALPHA_OPACITY_THRESH))
//       {
//         continue;
//       }

//       MaterialGPencilStyle *gp_style = BKE_gpencil_material_settings(ob, gps->mat_nr + 1);
//       const bool is_stroke = ((gp_style->flag & GP_MATERIAL_STROKE_SHOW) &&
//                               (gp_style->stroke_rgba[3] > GPENCIL_ALPHA_OPACITY_THRESH) &&
//                               (stroke_opacity > GPENCIL_ALPHA_OPACITY_THRESH));
//       const bool is_fill = ((gp_style->flag & GP_MATERIAL_FILL_SHOW) &&
//                             (gp_style->fill_rgba[3] > GPENCIL_ALPHA_OPACITY_THRESH));

//       if ((!is_stroke) && (!is_fill)) {
//         continue;
//       }

//       /* Duplicate the stroke to apply any layer thickness change. */
//       bGPDstroke *gps_duplicate = BKE_gpencil_stroke_duplicate(gps, true, false);

//       /* Apply layer thickness change. */
//       gps_duplicate->thickness += gpl->line_change;
//       /* Apply object scale to thickness. */
//       const float scalef = mat4_to_scale(ob->object_to_world().ptr());
//       gps_duplicate->thickness = ceilf(float(gps_duplicate->thickness) * scalef);
//       CLAMP_MIN(gps_duplicate->thickness, 1.0f);
//       /* Fill. */
//       if ((is_fill) && (params_.flag & GP_EXPORT_FILL)) {
//         /* Fill is exported as polygon for fill and stroke in a different shape. */
//         export_stroke_to_polyline(gpd_eval, gpl, gps_duplicate, is_stroke, true, false);
//       }

//       /* Stroke. */
//       if (is_stroke) {
//         if (is_normalized) {
//           export_stroke_to_polyline(gpd_eval, gpl, gps_duplicate, is_stroke, false, true);
//         }
//         else {
//           bGPDstroke *gps_perimeter = BKE_gpencil_stroke_perimeter_from_view(
//               rv3d_->viewmat, gpd_eval, gpl, gps_duplicate, 3, diff_mat_.ptr(), 0.0f);

//           /* Sample stroke. */
//           if (params_.stroke_sample > 0.0f) {
//             BKE_gpencil_stroke_sample(gpd_eval, gps_perimeter, params_.stroke_sample, false,
//             0);
//           }

//           export_stroke_to_polyline(gpd_eval, gpl, gps_perimeter, is_stroke, false, false);

//           BKE_gpencil_free_stroke(gps_perimeter);
//         }
//       }
//       BKE_gpencil_free_stroke(gps_duplicate);
//     }
//   }
void PDFExporter::export_grease_pencil_layer(const Object &object,
                                             const GreasePencil &grease_pencil,
                                             const bke::greasepencil::Layer &layer,
                                             const int frame_number)
{
  using bke::greasepencil::Drawing;

  if (!layer.is_visible()) {
    return;
  }
  const Drawing *drawing = grease_pencil.get_drawing_at(layer, frame_number);
  if (drawing == nullptr) {
    return;
  }

  const float4x4 layer_to_world = layer.to_world_space(object);
  const float4x4 viewmat = float4x4(context_.rv3d->viewmat);
  const float4x4 layer_to_view = viewmat * layer_to_world;

  /* Layer node. */
  // const std::string txt = "Layer: " + layer.name();
  // node.append_child(pugi::node_comment).set_value(txt.c_str());

  // pugi::xml_node layer_node = node.append_child("g");
  // layer_node.append_attribute("id").set_value(layer.name().c_str());

  const bke::CurvesGeometry &curves = drawing->strokes();
  const bke::AttributeAccessor attributes = curves.attributes();
  const OffsetIndices points_by_curve = curves.points_by_curve();
  const VArray<bool> cyclic = curves.cyclic();
  const VArraySpan<int> material_indices = *attributes.lookup_or_default<int>(
      "material_index", bke::AttrDomain::Curve, 0);
  const Span<float3> positions = curves.positions();
  const VArraySpan<float> radii = drawing->radii();
  const VArraySpan<float> opacities = drawing->opacities();
  const VArray<int8_t> start_caps = *attributes.lookup_or_default<int8_t>(
      "start_cap", bke::AttrDomain::Curve, GP_STROKE_CAP_TYPE_ROUND);
  const VArray<int8_t> end_caps = *attributes.lookup_or_default<int8_t>(
      "end_cap", bke::AttrDomain::Curve, 0);

  Array<float3> world_positions(positions.size());
  threading::parallel_for(positions.index_range(), 4096, [&](const IndexRange range) {
    for (const int i : range) {
      world_positions[i] = math::transform_point(layer_to_world, positions[i]);
    }
  });

  for (const int i_curve : curves.curves_range()) {
    const IndexRange points = points_by_curve[i_curve];
    if (points.size() < 2) {
      continue;
    }

    const bool is_cyclic = cyclic[i_curve];
    const int material_index = material_indices[i_curve];
    const Material *material = BKE_object_material_get(const_cast<Object *>(&object),
                                                       material_index + 1);
    BLI_assert(material->gp_style != nullptr);
    if (material->gp_style->flag & GP_MATERIAL_HIDE) {
      continue;
    }
    const bool is_stroke_material = material->gp_style->flag & GP_MATERIAL_STROKE_SHOW;
    const bool is_fill_material = material->gp_style->flag & GP_MATERIAL_FILL_SHOW;

    /* Fill. */
    if (is_fill_material && params_.export_fill_materials) {
      // /* Fill is always exported as polygon because the stroke of the fill is done
      //  * in a different SVG command. */
      // pugi::xml_node element_node = write_polyline(
      //     layer_node, layer_to_view, positions.slice(points), is_cyclic, std::nullopt);

      // const ColorGeometry4f fill_color = {material->gp_style->fill_rgba};
      // write_fill_color_attribute(element_node, fill_color, layer.opacity);
    }

    /* Stroke. */
    if (is_stroke_material && params_.export_stroke_materials) {
      const ColorGeometry4f stroke_color = {material->gp_style->stroke_rgba};
      const GreasePencilStrokeCapType start_cap = GreasePencilStrokeCapType(start_caps[i_curve]);
      const GreasePencilStrokeCapType end_cap = GreasePencilStrokeCapType(end_caps[i_curve]);
      const bool round_cap = start_cap == GP_STROKE_CAP_TYPE_ROUND ||
                             end_cap == GP_STROKE_CAP_TYPE_ROUND;

      /* Compute per-point stroke width based on pixel size. */
      VArray<float> widths = VArray<float>::ForFunc(points.size(), [&](const int index) {
        const float3 &pos = world_positions[points[index]];
        const float radius = radii[points[index]];
        return 2.0f * radius * ED_view3d_pixel_size(context_.rv3d, pos);
      });
      const std::optional<float> uniform_width = params_.use_uniform_width ?
                                                     try_get_constant_value(widths) :
                                                     std::nullopt;
      if (uniform_width) {
        pugi::xml_node element_node = write_polyline(
            layer_node, layer_to_view, positions.slice(points), is_cyclic, uniform_width);
        write_stroke_color_attribute(
            element_node, stroke_color, layer.opacity, round_cap, opacities);
      }
      else {
        const IndexMask single_curve_mask = IndexRange::from_single(i_curve);

        constexpr int corner_subdivisions = 3;
        constexpr float outline_radius = 0.0f;
        constexpr float outline_offset = 0.0f;
        bke::CurvesGeometry outline = ed::greasepencil::create_curves_outline(*drawing,
                                                                              single_curve_mask,
                                                                              layer_to_view,
                                                                              corner_subdivisions,
                                                                              outline_radius,
                                                                              outline_offset,
                                                                              material_index);

        /* Sample the outline stroke. */
        if (params_.outline_resample_length > 0.0f) {
          VArray<float> resample_lengths = VArray<float>::ForSingle(
              params_.outline_resample_length, curves.curves_num());
          outline = geometry::resample_to_length(outline, single_curve_mask, resample_lengths);
        }

        pugi::xml_node element_node = write_path(layer_node, layer_to_view, positions, is_cyclic);
        /* Use stroke color to fill the outline. */
        write_fill_color_attribute(element_node, stroke_color, layer.opacity);
      }
    }
  }
}

bool PDFExporter::create_document()
{
  auto hpdf_error_handler = [](HPDF_STATUS error_no, HPDF_STATUS detail_no, void * /*user_data*/) {
    printf("ERROR: error_no=%04X, detail_no=%u\n", (HPDF_UINT)error_no, (HPDF_UINT)detail_no);
  };

  pdf_ = HPDF_New(hpdf_error_handler, nullptr);
  if (!pdf_) {
    std::cout << "error: cannot create PdfDoc object\n";
    return false;
  }
  return true;
}

bool PDFExporter::add_page()
{
  page_ = HPDF_AddPage(pdf_);
  if (!pdf_) {
    std::cout << "error: cannot create PdfPage\n";
    return false;
  }

  HPDF_Page_SetWidth(page_, render_size_.x);
  HPDF_Page_SetHeight(page_, render_size_.y);

  return true;
}

bool PDFExporter::write_to_file(StringRefNull filepath)
{
  /* Support unicode character paths on Windows. */
  HPDF_STATUS result = 0;

  /* TODO: It looks `libharu` does not support unicode. */
#if 0 /* `ifdef WIN32` */
  wchar_t *filepath_16 = alloc_utf16_from_8(filepath.c_str(), 0);
  std::wstring wstr(filepath_16);
  result = HPDF_SaveToFile(pdf_, wstr.c_str());
  free(filepath_16);
#else
  result = HPDF_SaveToFile(pdf_, filepath.c_str());
#endif

  return (result == 0) ? true : false;
}

bool export_pdf(const IOContext &context,
                const ExportParams &params,
                Scene &scene,
                StringRefNull filepath)
{
  PDFExporter exporter(context, params);
  return exporter.export_scene(scene, filepath);
}

}  // namespace blender::io::grease_pencil
