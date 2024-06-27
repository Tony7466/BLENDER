/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute.hh"
#include "BKE_curves.hh"
#include "BKE_material.h"
#include "BLI_color.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_quaternion.hh"
#include "BLI_math_vector.hh"
#include "BLI_string.h"
#include "BLI_task.hh"
#include "BLI_vector.hh"

#include "BKE_context.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_layer.hh"

#include "BLI_virtual_array.hh"
#include "DNA_material_types.h"
#include "DNA_object_types.h"

#include "DEG_depsgraph_query.hh"

#include "DNA_view3d_types.h"
#include "ED_grease_pencil.hh"
#include "ED_view3d.hh"

#include "grease_pencil_io_intern.hh"

#include "pugixml.hpp"

#include <fmt/core.h>
#include <fmt/format.h>
#include <numeric>
#include <optional>

/** \file
 * \ingroup bgrease_pencil
 */

namespace blender::io::grease_pencil {

constexpr const char *svg_exporter_name = "SVG Export for Grease Pencil";
constexpr const char *svg_exporter_version = "v2.0";

static std::string rgb_to_hexstr(const float color[3])
{
  uint8_t r = color[0] * 255.0f;
  uint8_t g = color[1] * 255.0f;
  uint8_t b = color[2] * 255.0f;
  return fmt::format("#{02X}{02X}{02X}", r, g, b);
}

static float get_average(const Span<float> values)
{
  return values.is_empty() ? 0.0f :
                             std::accumulate(values.begin(), values.end(), 0.0f) / values.size();
}

static std::optional<float> try_get_constant_value(const VArray<float> values,
                                                   const float epsilon = 1e-5f)
{
  if (values.is_empty()) {
    return std::nullopt;
  }
  const float first_value = values.first();
  const std::optional<float> first_value_opt = std::make_optional(first_value);
  return threading::parallel_reduce(
      values.index_range().drop_front(1),
      4096,
      first_value_opt,
      [&](const IndexRange range, const std::optional<float> /*value*/) -> std::optional<float> {
        for (const int i : range) {
          if (math::abs(values[i] - first_value) > epsilon) {
            return std::nullopt;
          }
        }
        return first_value_opt;
      },
      [&](const std::optional<float> a, const std::optional<float> b) {
        return (a && b) ? first_value_opt : std::nullopt;
      });
}

static void write_stroke_color_attribute(pugi::xml_node node,
                                         const ColorGeometry4f &stroke_color,
                                         const float layer_opacity,
                                         const bool round_cap,
                                         const Span<float> point_opacities)
{
  const float average_stroke_opacity = get_average(point_opacities);

  ColorGeometry4f color;
  linearrgb_to_srgb_v3_v3(color, stroke_color);
  std::string stroke_hex = rgb_to_hexstr(color);

  node.append_attribute("stroke").set_value(stroke_hex.c_str());
  node.append_attribute("stroke-opacity")
      .set_value(stroke_color.a * average_stroke_opacity * layer_opacity);

  node.append_attribute("fill").set_value("none");
  node.append_attribute("stroke-linecap").set_value(round_cap ? "round" : "square");
}

static void write_fill_color_attribute(pugi::xml_node node,
                                       const ColorGeometry4f &fill_color,
                                       const float layer_opacity)
{
  ColorGeometry4f color;
  linearrgb_to_srgb_v3_v3(color, fill_color);
  std::string stroke_hex = rgb_to_hexstr(color);

  node.append_attribute("fill").set_value(stroke_hex.c_str());
  node.append_attribute("stroke").set_value("none");
  node.append_attribute("fill-opacity").set_value(fill_color.a * layer_opacity);
}

static void write_rect(pugi::xml_node node,
                       float x,
                       float y,
                       float width,
                       float height,
                       float thickness,
                       const std::string &hexcolor)
{
  pugi::xml_node rect_node = node.append_child("rect");
  rect_node.append_attribute("x").set_value(x);
  rect_node.append_attribute("y").set_value(y);
  rect_node.append_attribute("width").set_value(width);
  rect_node.append_attribute("height").set_value(height);
  rect_node.append_attribute("fill").set_value("none");
  if (thickness > 0.0f) {
    rect_node.append_attribute("stroke").set_value(hexcolor.c_str());
    rect_node.append_attribute("stroke-width").set_value(thickness);
  }
}

static void write_text(pugi::xml_node node,
                       float x,
                       float y,
                       const std::string &text,
                       const float size,
                       const std::string &hexcolor)
{
  pugi::xml_node nodetxt = node.append_child("text");

  nodetxt.append_attribute("x").set_value(x);
  nodetxt.append_attribute("y").set_value(y);
  // nodetxt.append_attribute("font-family").set_value("'system-ui'");
  nodetxt.append_attribute("font-size").set_value(size);
  nodetxt.append_attribute("fill").set_value(hexcolor.c_str());
  nodetxt.text().set(text.c_str());
}

class SVGExporter : public GreasePencilExporter {
 public:
  using GreasePencilExporter::GreasePencilExporter;

  struct ObjectInfo {
    const Object *object;
    float depth;
  };

  pugi::xml_document main_doc_;
  pugi::xml_node main_node_;
  pugi::xml_node frame_node_;

  bool write(Scene &scene, StringRefNull filepath);

  void write_document_header();
  void export_grease_pencil_objects();
  void export_grease_pencil_layer(pugi::xml_node node,
                                  const Object &object,
                                  const GreasePencil &grease_pencil,
                                  const bke::greasepencil::Layer &layer);

  void write_polygon(pugi::xml_node node, const Span<float3> view_positions);

  void write_polyline(pugi::xml_node node,
                      const bool cyclic,
                      const std::optional<float> width,
                      const Span<float3> view_positions);

  Vector<ObjectInfo> retrieve_objects() const;
};

bool SVGExporter::write(Scene &scene, StringRefNull filepath)
{
  this->prepare_camera_params(scene, false);

  this->write_document_header();
  this->export_grease_pencil_objects();
  return true;
}

void SVGExporter::write_document_header()
{
  /* Add a custom document declaration node. */
  pugi::xml_node decl = main_doc_.prepend_child(pugi::node_declaration);
  decl.append_attribute("version") = "1.0";
  decl.append_attribute("encoding") = "UTF-8";

  pugi::xml_node comment = main_doc_.append_child(pugi::node_comment);
  std::string txt = std::string(" Generator: Blender, ") + svg_exporter_name + " - " +
                    svg_exporter_version + " ";
  comment.set_value(txt.c_str());

  pugi::xml_node doctype = main_doc_.append_child(pugi::node_doctype);
  doctype.set_value(
      "svg PUBLIC \"-//W3C//DTD SVG 1.1//EN\" "
      "\"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd\"");

  main_node_ = main_doc_.append_child("svg");
  main_node_.append_attribute("version").set_value("1.0");
  main_node_.append_attribute("x").set_value("0px");
  main_node_.append_attribute("y").set_value("0px");
  main_node_.append_attribute("xmlns").set_value("http://www.w3.org/2000/svg");

  std::string width;
  std::string height;

  width = std::to_string(render_size_.x);
  height = std::to_string(render_size_.y);

  main_node_.append_attribute("width").set_value((width + "px").c_str());
  main_node_.append_attribute("height").set_value((height + "px").c_str());
  std::string viewbox = "0 0 " + width + " " + height;
  main_node_.append_attribute("viewBox").set_value(viewbox.c_str());
}

Vector<SVGExporter::ObjectInfo> SVGExporter::retrieve_objects() const
{
  using SelectMode = ExportParams::SelectMode;

  Scene &scene = *CTX_data_scene(&context_.C);
  ViewLayer *view_layer = CTX_data_view_layer(&context_.C);
  const float3 camera_z_axis = float3(context_.rv3d->viewinv[2]);

  BKE_view_layer_synced_ensure(&scene, view_layer);

  Vector<ObjectInfo> objects;
  auto add_object = [&](Object *object) {
    if (object == nullptr || object->type != OB_GREASE_PENCIL) {
      return;
    }

    const float3 position = object->object_to_world().location();

    /* Save z-depth from view to sort from back to front. */
    const bool use_ortho_depth = is_camera_ || !context_.rv3d->is_persp;
    const float depth = use_ortho_depth ? math::dot(camera_z_axis, position) :
                                          -ED_view3d_calc_zfac(context_.rv3d, position);
    objects.append_as(object, depth);
  };

  switch (params_.select_mode) {
    case SelectMode::Active:
      add_object(params_.object);
      break;
    case SelectMode::Selected:
      LISTBASE_FOREACH (Base *, base, BKE_view_layer_object_bases_get(view_layer)) {
        if (base->flag & BASE_SELECTED) {
          add_object(base->object);
        }
      }
      break;
    case SelectMode::Visible:
      LISTBASE_FOREACH (Base *, base, BKE_view_layer_object_bases_get(view_layer)) {
        add_object(base->object);
      }
      break;
  }

  /* Sort list of objects from point of view. */
  std::sort(objects.begin(), objects.end(), [](const ObjectInfo &info1, const ObjectInfo &info2) {
    return info1.depth < info2.depth;
  });

  return objects;
}

void SVGExporter::export_grease_pencil_objects()
{
  const bool is_clipping = is_camera_ && params_.use_clip_camera;

  /* If is doing a set of frames, the list of objects can change for each frame. */
  Vector<ObjectInfo> objects = retrieve_objects();

  for (const ObjectInfo &info : objects) {
    const Object *ob = info.object;

    /* Camera clipping. */
    if (is_clipping) {
      pugi::xml_node clip_node = main_node_.append_child("clipPath");
      clip_node.append_attribute("id").set_value(
          ("clip-path" + std::to_string(params_.frame)).c_str());

      write_rect(clip_node, 0, 0, render_size_.x, render_size_.y, 0.0f, "#000000");
    }

    frame_node_ = main_node_.append_child("g");
    std::string frametxt = "blender_frame_" + std::to_string(params_.frame);
    frame_node_.append_attribute("id").set_value(frametxt.c_str());

    /* Clip area. */
    if (is_clipping) {
      frame_node_.append_attribute("clip-path")
          .set_value(("url(#clip-path" + std::to_string(params_.frame) + ")").c_str());
    }

    pugi::xml_node ob_node = frame_node_.append_child("g");

    char obtxt[96];
    SNPRINTF(obtxt, "blender_object_%s", ob->id.name + 2);
    ob_node.append_attribute("id").set_value(obtxt);

    /* Use evaluated version to get strokes with modifiers. */
    Object *ob_eval = DEG_get_evaluated_object(context_.depsgraph, const_cast<Object *>(ob));
    BLI_assert(ob_eval->type == OB_GREASE_PENCIL);
    const GreasePencil *grease_pencil_eval = static_cast<const GreasePencil *>(ob_eval->data);

    for (const bke::greasepencil::Layer *layer : grease_pencil_eval->layers()) {
      export_grease_pencil_layer(ob_node, *ob_eval, *grease_pencil_eval, *layer);
    }
  }
}

void SVGExporter::export_grease_pencil_layer(pugi::xml_node node,
                                             const Object &object,
                                             const GreasePencil &grease_pencil,
                                             const bke::greasepencil::Layer &layer)
{
  using bke::greasepencil::Drawing;

  if (!layer.is_visible()) {
    return;
  }
  const Drawing *drawing = grease_pencil.get_drawing_at(layer, params_.frame);
  if (drawing == nullptr) {
    return;
  }

  const Scene &scene = *CTX_data_scene(&context_.C);
  const float4x4 layer_to_world = layer.to_world_space(object);
  const float4x4 viewmat = float4x4(context_.rv3d->viewmat);
  // const float4x4 layer_to_view = viewmat * layer_to_world;

  /* Layer node. */
  const std::string txt = "Layer: " + layer.name();
  node.append_child(pugi::node_comment).set_value(txt.c_str());

  pugi::xml_node layer_node = node.append_child("g");
  layer_node.append_attribute("id").set_value(layer.name().c_str());

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
    const Material *material = BKE_object_material_get(const_cast<Object *>(&object),
                                                       material_indices[i_curve] + 1);
    BLI_assert(material->gp_style != nullptr);
    if (material->gp_style->flag & GP_MATERIAL_HIDE) {
      continue;
    }
    const bool is_stroke_material = material->gp_style->flag & GP_MATERIAL_STROKE_SHOW;
    const bool is_fill_material = material->gp_style->flag & GP_MATERIAL_FILL_SHOW;

    /* Fill. */
    if (is_fill_material && params_.export_fill_materials) {
      /* Fill is always exported as polygon because the stroke of the fill is done
       * in a different SVG command. */
      const ColorGeometry4f fill_color = {material->gp_style->fill_rgba};
      write_fill_color_attribute(layer_node, fill_color, layer.opacity);
      write_polyline(layer_node, is_cyclic, std::nullopt, world_positions);
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
        write_stroke_color_attribute(
            layer_node, stroke_color, layer.opacity, round_cap, opacities);
        write_polyline(layer_node, is_cyclic, uniform_width, world_positions);
      }
      else {
        bke::CurvesGeometry outline = ed::greasepencil::create_curves_outline(
            *drawing,
            IndexRange::from_single(i_curve),
            viewmat,
            3,
            outline_radius,
            0.0f,
            material_index);

        bGPDstroke *gps_perimeter = BKE_gpencil_stroke_perimeter_from_view(
            rv3d_->viewmat, gpd_eval, gpl, gps_duplicate, 3, diff_mat_.ptr(), 0.0f);

        /* Sample stroke. */
        if (params_.stroke_sample > 0.0f) {
          BKE_gpencil_stroke_sample(gpd_eval, gps_perimeter, params_.stroke_sample, false, 0);
        }

        export_stroke_to_path(gpl, gps_perimeter, layer_node, false);

        BKE_gpencil_free_stroke(gps_perimeter);
      }
    }
  }
}

// void SVGExporter::export_stroke_to_path(bGPDlayer *gpl,
//                                         bGPDstroke *gps,
//                                         pugi::xml_node node_gpl,
//                                         const bool do_fill)
// {
//   pugi::xml_node node_gps = node_gpl.append_child("path");

//   float col[3];
//   std::string stroke_hex;
//   if (do_fill) {
//     node_gps.append_attribute("fill-opacity").set_value(fill_color_[3] * gpl->opacity);

//     interp_v3_v3v3(col, fill_color_, gpl->tintcolor, gpl->tintcolor[3]);
//   }
//   else {
//     node_gps.append_attribute("fill-opacity")
//         .set_value(stroke_color_[3] * stroke_average_opacity_get() * gpl->opacity);

//     interp_v3_v3v3(col, stroke_color_, gpl->tintcolor, gpl->tintcolor[3]);
//   }

//   linearrgb_to_srgb_v3_v3(col, col);
//   stroke_hex = rgb_to_hexstr(col);

//   node_gps.append_attribute("fill").set_value(stroke_hex.c_str());
//   node_gps.append_attribute("stroke").set_value("none");

//   std::string txt = "M";
//   for (const int i : IndexRange(gps->totpoints)) {
//     if (i > 0) {
//       txt.append("L");
//     }
//     bGPDspoint &pt = gps->points[i];
//     const float2 screen_co = gpencil_3D_point_to_2D(&pt.x);
//     txt.append(std::to_string(screen_co.x) + "," + std::to_string(screen_co.y));
//   }
//   /* Close patch (cyclic). */
//   if (gps->flag & GP_STROKE_CYCLIC) {
//     txt.append("z");
//   }

//   node_gps.append_attribute("d").set_value(txt.c_str());
// }

void SVGExporter::write_polygon(pugi::xml_node node, const Span<float3> world_positions)
{
  pugi::xml_node node_gps = node.append_child("polygon");

  std::string txt;
  for (const int i : view_positions.index_range()) {
    if (i > 0) {
      txt.append(" ");
    }
    const float3 &pos = view_positions[i];
    txt.append(std::to_string(pos.x) + "," + std::to_string(pos.y));
  }

  node_gps.append_attribute("points").set_value(txt.c_str());
}

void SVGExporter::write_polyline(pugi::xml_node node,
                                 const bool cyclic,
                                 const std::optional<float> width,
                                 const Span<float3> world_positions)
{
  pugi::xml_node node_gps = node.append_child(cyclic ? "polygon" : "polyline");

  if (width) {
    node_gps.append_attribute("stroke-width").set_value(*width);
  }

  std::string txt;
  for (const int i : view_positions.index_range()) {
    if (i > 0) {
      txt.append(" ");
    }
    float2 screen_co = float2(0);
    ED_view3d_project_float_ex(&region,
                               const_cast<float(*)[4]>(rv3d.winmat),
                               false,
                               view_positions[i],
                               screen_co,
                               V3D_PROJ_TEST_NOP);
    txt.append(std::to_string(screen_co.x) + "," + std::to_string(screen_co.y));
  }

  node_gps.append_attribute("points").set_value(txt.c_str());
}

bool export_svg(const IOContext &context,
                const ExportParams &params,
                Scene &scene,
                StringRefNull filepath)
{
  SVGExporter exporter(context, params);
  return exporter.write(scene, filepath);
}

}  // namespace blender::io::grease_pencil
