/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute.hh"
#include "BKE_curves.hh"
#include "BKE_material.h"
#include "BLI_color.hh"
#include "BLI_length_parameterize.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_quaternion.hh"
#include "BLI_math_vector.hh"
#include "BLI_string.h"
#include "BLI_task.hh"
#include "BLI_vector.hh"
#include "BLI_virtual_array.hh"

#include "BKE_context.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_layer.hh"

#include "DNA_material_types.h"
#include "DNA_object_types.h"

#include "DEG_depsgraph_query.hh"
#include "DNA_view3d_types.h"

#include "GEO_resample_curves.hh"

#include "ED_grease_pencil.hh"
#include "ED_view3d.hh"

#include "grease_pencil_io_intern.hh"

#include <fmt/core.h>
#include <fmt/format.h>
#include <numeric>
#include <optional>
#include <pugixml.hpp>

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
  return fmt::format("#{:02X}{:02X}{:02X}", r, g, b);
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

class SVGExporter : public GreasePencilExporter {
 public:
  using GreasePencilExporter::GreasePencilExporter;

  struct ObjectInfo {
    const Object *object;
    float depth;
  };

  pugi::xml_document main_doc_;

  bool write(Scene &scene, StringRefNull filepath);

  void write_document_header();
  pugi::xml_node write_main_node();
  void export_grease_pencil_objects(pugi::xml_node node);
  void export_grease_pencil_layer(pugi::xml_node node,
                                  const Object &object,
                                  const GreasePencil &grease_pencil,
                                  const bke::greasepencil::Layer &layer);

  pugi::xml_node write_polygon(pugi::xml_node node,
                               const float4x4 &transform,
                               Span<float3> positions);
  pugi::xml_node write_polyline(pugi::xml_node node,
                                const float4x4 &transform,
                                Span<float3> positions,
                                bool cyclic,
                                std::optional<float> width);
  pugi::xml_node write_path(pugi::xml_node node,
                            const float4x4 &transform,
                            Span<float3> positions,
                            bool cyclic);

  Vector<ObjectInfo> retrieve_objects() const;
};

bool SVGExporter::write(Scene &scene, StringRefNull filepath)
{
  this->prepare_camera_params(scene, false);

  this->write_document_header();
  pugi::xml_node main_node = this->write_main_node();
  this->export_grease_pencil_objects(main_node);

  bool result = true;
  /* Support unicode character paths on Windows. */
#ifdef WIN32
  wchar_t *filepath_16 = alloc_utf16_from_8(filepath.c_str(), 0);
  std::wstring wstr(filepath_16);
  result = main_doc_.save_file(wstr.c_str());
  free(filepath_16);
#else
  result = main_doc_.save_file(filepath.c_str());
#endif

  return result;
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
}

pugi::xml_node SVGExporter::write_main_node()
{
  pugi::xml_node main_node = main_doc_.append_child("svg");
  main_node.append_attribute("version").set_value("1.0");
  main_node.append_attribute("x").set_value("0px");
  main_node.append_attribute("y").set_value("0px");
  main_node.append_attribute("xmlns").set_value("http://www.w3.org/2000/svg");

  std::string width = std::to_string(render_size_.x);
  std::string height = std::to_string(render_size_.y);

  main_node.append_attribute("width").set_value((width + "px").c_str());
  main_node.append_attribute("height").set_value((height + "px").c_str());
  std::string viewbox = "0 0 " + width + " " + height;
  main_node.append_attribute("viewBox").set_value(viewbox.c_str());

  return main_node;
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
    objects.append({object, depth});
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

void SVGExporter::export_grease_pencil_objects(pugi::xml_node node)
{
  const bool is_clipping = is_camera_ && params_.use_clip_camera;

  /* If is doing a set of frames, the list of objects can change for each frame. */
  Vector<ObjectInfo> objects = retrieve_objects();

  for (const ObjectInfo &info : objects) {
    const Object *ob = info.object;

    /* Camera clipping. */
    if (is_clipping) {
      pugi::xml_node clip_node = node.append_child("clipPath");
      clip_node.append_attribute("id").set_value(
          ("clip-path" + std::to_string(params_.frame)).c_str());

      write_rect(clip_node, 0, 0, render_size_.x, render_size_.y, 0.0f, "#000000");
    }

    pugi::xml_node frame_node = node.append_child("g");
    std::string frametxt = "blender_frame_" + std::to_string(params_.frame);
    frame_node.append_attribute("id").set_value(frametxt.c_str());

    /* Clip area. */
    if (is_clipping) {
      frame_node.append_attribute("clip-path")
          .set_value(("url(#clip-path" + std::to_string(params_.frame) + ")").c_str());
    }

    pugi::xml_node ob_node = frame_node.append_child("g");

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

  const float4x4 layer_to_world = layer.to_world_space(object);
  const float4x4 viewmat = float4x4(context_.rv3d->viewmat);
  const float4x4 layer_to_view = viewmat * layer_to_world;

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
      /* Fill is always exported as polygon because the stroke of the fill is done
       * in a different SVG command. */
      pugi::xml_node element_node = write_polyline(
          layer_node, layer_to_view, positions.slice(points), is_cyclic, std::nullopt);

      const ColorGeometry4f fill_color = {material->gp_style->fill_rgba};
      write_fill_color_attribute(element_node, fill_color, layer.opacity);
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

pugi::xml_node SVGExporter::write_polygon(pugi::xml_node node,
                                          const float4x4 &transform,
                                          const Span<float3> positions)
{
  pugi::xml_node element_node = node.append_child("polygon");

  std::string txt;
  for (const int i : positions.index_range()) {
    if (i > 0) {
      txt.append(" ");
    }
    float2 screen_co = float2(0);
    ED_view3d_project_float_ex(context_.region,
                               const_cast<float(*)[4]>(context_.rv3d->winmat),
                               false,
                               math::transform_point(transform, positions[i]),
                               screen_co,
                               V3D_PROJ_TEST_NOP);
    txt.append(std::to_string(screen_co.x) + "," + std::to_string(screen_co.y));
  }

  element_node.append_attribute("points").set_value(txt.c_str());

  return element_node;
}

pugi::xml_node SVGExporter::write_polyline(pugi::xml_node node,
                                           const float4x4 &transform,
                                           const Span<float3> positions,
                                           const bool cyclic,
                                           const std::optional<float> width)
{
  pugi::xml_node element_node = node.append_child(cyclic ? "polygon" : "polyline");

  if (width) {
    element_node.append_attribute("stroke-width").set_value(*width);
  }

  std::string txt;
  for (const int i : positions.index_range()) {
    if (i > 0) {
      txt.append(" ");
    }
    float2 screen_co = float2(0);
    ED_view3d_project_float_ex(context_.region,
                               const_cast<float(*)[4]>(context_.rv3d->winmat),
                               false,
                               math::transform_point(transform, positions[i]),
                               screen_co,
                               V3D_PROJ_TEST_NOP);
    txt.append(std::to_string(screen_co.x) + "," + std::to_string(screen_co.y));
  }

  element_node.append_attribute("points").set_value(txt.c_str());

  return element_node;
}

pugi::xml_node SVGExporter::write_path(pugi::xml_node node,
                                       const float4x4 &transform,
                                       const Span<float3> positions,
                                       const bool cyclic)
{
  pugi::xml_node element_node = node.append_child("path");

  std::string txt = "M";
  for (const int i : positions.index_range()) {
    if (i > 0) {
      txt.append("L");
    }
    float2 screen_co = float2(0);
    ED_view3d_project_float_ex(context_.region,
                               const_cast<float(*)[4]>(context_.rv3d->winmat),
                               false,
                               math::transform_point(transform, positions[i]),
                               screen_co,
                               V3D_PROJ_TEST_NOP);
    txt.append(std::to_string(screen_co.x) + "," + std::to_string(screen_co.y));
  }
  /* Close patch (cyclic). */
  if (cyclic) {
    txt.append("z");
  }

  element_node.append_attribute("d").set_value(txt.c_str());

  return element_node;
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
