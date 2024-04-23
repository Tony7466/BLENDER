/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_matrix.hh"

#include "BKE_attribute.hh"
#include "BKE_camera.h"
#include "BKE_curves.hh"
#include "BKE_image.h"

#include "DNA_material_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_view3d_types.h"

#include "ED_grease_pencil.hh"
#include "ED_view3d.hh"

#include "IMB_imbuf.hh"
#include "IMB_imbuf_types.hh"

#include "GPU_framebuffer.hh"
#include "GPU_immediate.hh"
#include "GPU_matrix.hh"
#include "GPU_shader_shared.hh"
#include "GPU_state.hh"
#include "GPU_texture.hh"
#include "GPU_vertex_format.hh"

namespace blender::ed::greasepencil::image_render {

struct ImageRenderData {
  GPUOffScreen *offscreen;
  int2 region_winsize;
  rcti region_winrct;
};

ImageRenderData *image_render_begin(ARegion &region, const int2 &win_size)
{
  char err_out[256] = "unknown";
  GPUOffScreen *offscreen = GPU_offscreen_create(
      win_size.x, win_size.y, true, GPU_RGBA8, GPU_TEXTURE_USAGE_HOST_READ, err_out);
  if (offscreen == nullptr) {
    return nullptr;
  }

  const int2 region_winsize = {region.winx, region.winy};
  const rcti region_winrct = region.winrct;

  /* Resize region. */
  region.winrct.xmin = 0;
  region.winrct.ymin = 0;
  region.winrct.xmax = win_size.x;
  region.winrct.ymax = win_size.y;
  region.winx = short(win_size.x);
  region.winy = short(win_size.y);

  GPU_offscreen_bind(offscreen, true);

  GPU_matrix_push_projection();
  GPU_matrix_identity_projection_set();
  GPU_matrix_push();
  GPU_matrix_identity_set();

  GPU_clear_color(0.0f, 0.0f, 0.0f, 0.0f);
  GPU_clear_depth(1.0f);

  return new ImageRenderData{offscreen, region_winsize, region_winrct};
}

Image *image_render_end(Main &bmain, ARegion &region, ImageRenderData *data)
{
  const int2 win_size = {GPU_offscreen_width(data->offscreen),
                         GPU_offscreen_height(data->offscreen)};
  /* create a image to see result of template */
  const uint imb_flag = IB_rect;
  ImBuf *ibuf = IMB_allocImBuf(win_size.x, win_size.y, 32, imb_flag);
  if (ibuf->float_buffer.data) {
    GPU_offscreen_read_color(data->offscreen, GPU_DATA_FLOAT, ibuf->float_buffer.data);
  }
  else if (ibuf->byte_buffer.data) {
    GPU_offscreen_read_color(data->offscreen, GPU_DATA_UBYTE, ibuf->byte_buffer.data);
  }
  if (ibuf->float_buffer.data && ibuf->byte_buffer.data) {
    IMB_rect_from_float(ibuf);
  }

  Image *ima = BKE_image_add_from_imbuf(&bmain, ibuf, "Grease Pencil Fill");
  ima->id.tag |= LIB_TAG_DOIT;

  BKE_image_release_ibuf(ima, ibuf, nullptr);

  /* Switch back to window-system-provided frame-buffer. */
  GPU_offscreen_unbind(data->offscreen, true);
  GPU_offscreen_free(data->offscreen);

  region.winx = data->region_winsize.x;
  region.winy = data->region_winsize.y;
  region.winrct = data->region_winrct;

  delete data;

  return ima;
}

void set_viewmat(ARegion &region,
                 View3D &view3d,
                 RegionView3D &rv3d,
                 Depsgraph &depsgraph,
                 const Scene &scene,
                 const int2 &win_size,
                 const float2 &zoom,
                 const float2 &offset)
{
  // CameraParams camera_params;
  // BKE_camera_params_init(&camera_params);
  // BKE_camera_params_from_view3d(&camera_params, &depsgraph, &view3d, &rv3d);
  // BKE_camera_params_compute_viewplane(&camera_params, win_size.x, win_size.y, 1.0f, 1.0f);
  // BKE_camera_params_compute_matrix(&camera_params);

  rctf viewplane;
  float clip_start, clip_end;
  const bool is_ortho = ED_view3d_viewplane_get(&depsgraph,
                                                &view3d,
                                                &rv3d,
                                                win_size.x,
                                                win_size.y,
                                                &viewplane,
                                                &clip_start,
                                                &clip_end,
                                                nullptr);

  /* Rescale `viewplane` to fit all strokes. */
  viewplane.xmin = viewplane.xmin * zoom.x + offset.x;
  viewplane.xmax = viewplane.xmax * zoom.x + offset.x;
  viewplane.ymin = viewplane.ymin * zoom.y + offset.y;
  viewplane.ymax = viewplane.ymax * zoom.y + offset.y;

  float4x4 winmat;
  if (is_ortho) {
    orthographic_m4(winmat.ptr(),
                    viewplane.xmin,
                    viewplane.xmax,
                    viewplane.ymin,
                    viewplane.ymax,
                    -clip_end,
                    clip_end);
  }
  else {
    perspective_m4(winmat.ptr(),
                   viewplane.xmin,
                   viewplane.xmax,
                   viewplane.ymin,
                   viewplane.ymax,
                   clip_start,
                   clip_end);
  }

  ED_view3d_update_viewmat(
      &depsgraph, &scene, &view3d, &region, nullptr, winmat.ptr(), nullptr, true);
  GPU_matrix_set(rv3d.viewmat);
  GPU_matrix_projection_set(rv3d.winmat);
}

void clear_viewmat()
{
  GPU_matrix_pop_projection();
  GPU_matrix_pop();
}

void draw_dot(const float3 &position, const float point_size, const ColorGeometry4f &color)
{
  GPUVertFormat *format = immVertexFormat();
  uint attr_pos = GPU_vertformat_attr_add(format, "pos", GPU_COMP_F32, 3, GPU_FETCH_FLOAT);
  uint attr_size = GPU_vertformat_attr_add(format, "size", GPU_COMP_F32, 1, GPU_FETCH_FLOAT);
  uint attr_color = GPU_vertformat_attr_add(format, "color", GPU_COMP_F32, 4, GPU_FETCH_FLOAT);

  /* Draw mouse click position in Blue. */
  GPU_program_point_size(true);
  immBindBuiltinProgram(GPU_SHADER_3D_POINT_VARYING_SIZE_VARYING_COLOR);
  immBegin(GPU_PRIM_POINTS, 1);
  immAttr1f(attr_size, point_size * M_SQRT2);
  immAttr4fv(attr_color, color);
  immVertex3fv(attr_pos, position);
  immEnd();
  immUnbindProgram();
  GPU_program_point_size(false);
}

void draw_curve(Span<float3> positions,
                const VArray<ColorGeometry4f> &colors,
                const IndexRange indices,
                const float4x4 &layer_to_world,
                const bool cyclic,
                const float line_width)
{
  GPUVertFormat *format = immVertexFormat();
  const uint attr_pos = GPU_vertformat_attr_add(format, "pos", GPU_COMP_F32, 3, GPU_FETCH_FLOAT);
  const uint attr_color = GPU_vertformat_attr_add(
      format, "color", GPU_COMP_F32, 4, GPU_FETCH_FLOAT);
  immBindBuiltinProgram(GPU_SHADER_3D_FLAT_COLOR);

  /* draw stroke curve */
  GPU_line_width(line_width);
  /* If cyclic needs one more vertex. */
  const int cyclic_add = (cyclic && indices.size() > 2) ? 1 : 0;
  immBeginAtMost(GPU_PRIM_LINE_STRIP, indices.size() + cyclic_add);

  for (const int point_i : indices) {
    immAttr4fv(attr_color, colors[point_i]);
    immVertex3fv(attr_pos, math::transform_point(layer_to_world, positions[point_i]));
  }

  if (cyclic && indices.size() > 2) {
    const int point_i = indices[0];
    immAttr4fv(attr_color, colors[point_i]);
    immVertex3fv(attr_pos, math::transform_point(layer_to_world, positions[point_i]));
  }

  immEnd();
  immUnbindProgram();
}

static GPUUniformBuf *create_shader_ubo(const RegionView3D &rv3d,
                                        const int2 &win_size,
                                        const Object *object,
                                        const eGPDstroke_Caps caps_start,
                                        const eGPDstroke_Caps caps_end,
                                        const bool is_fill_stroke)
{

  float obj_scale = object ? math::average(float3(object->scale)) : 1.0f;

  GPencilStrokeData gpencil_stroke_data;
  copy_v2_v2(gpencil_stroke_data.viewport, float2(win_size));
  gpencil_stroke_data.pixsize = rv3d.pixsize;
  gpencil_stroke_data.objscale = obj_scale;
  /* TODO Was based on the GP_DATA_STROKE_KEEPTHICKNESS flag which does not seem to be converted atm. */
  gpencil_stroke_data.keep_size = false;
  gpencil_stroke_data.pixfactor = 1.0f;
  /* X-ray mode always to 3D space to avoid wrong Z-depth calculation (#60051). */
  gpencil_stroke_data.xraymode = GP_XRAY_3DSPACE;
  gpencil_stroke_data.caps_start = caps_start;
  gpencil_stroke_data.caps_end = caps_end;
  gpencil_stroke_data.fill_stroke = is_fill_stroke;

  return GPU_uniformbuf_create_ex(sizeof(GPencilStrokeData), &gpencil_stroke_data, __func__);
}

void draw_grease_pencil_stroke(Span<float3> positions,
                               const VArray<float> &radii,
                               const VArray<ColorGeometry4f> &colors,
                               const IndexRange indices,
                               const float4x4 &layer_to_world,
                               const bool cyclic)
{
  if (indices.is_empty()) {
    return;
  }

  GPUVertFormat *format = immVertexFormat();
  const uint attr_pos = GPU_vertformat_attr_add(format, "pos", GPU_COMP_F32, 3, GPU_FETCH_FLOAT);
  const uint attr_color = GPU_vertformat_attr_add(
      format, "color", GPU_COMP_F32, 4, GPU_FETCH_FLOAT);
  const uint attr_thickness = GPU_vertformat_attr_add(
      format, "thickness", GPU_COMP_F32, 1, GPU_FETCH_FLOAT);
  immBindBuiltinProgram(GPU_SHADER_GPENCIL_STROKE);
  constexpr const float min_thickness = 0.05f;

  /* If cyclic needs one more vertex. */
  const int cyclic_add = (cyclic && indices.size() > 2) ? 1 : 0;

  immBeginAtMost(GPU_PRIM_LINE_STRIP_ADJ, indices.size() + cyclic_add + 2);

  auto set_point = [&](const int point_i) {
    immAttr4fv(attr_color, colors[point_i]);
    immAttr1f(attr_thickness, std::max(radii[point_i], min_thickness));
    immVertex3fv(attr_pos, math::transform_point(layer_to_world, positions[point_i]));
  };

  /* first point for adjacency (not drawn) */
  if (cyclic && indices.size() > 2) {
    set_point(indices.last() - 1);
  }
  else {
    set_point(indices.first() + 1);
  }

  for (const int point_i : indices) {
    set_point(point_i);
  }

  if (cyclic && indices.size() > 2) {
    /* draw line to first point to complete the cycle */
    set_point(indices.first());
    set_point(indices.first() + 1);
  }
  /* last adjacency point (not drawn) */
  else {
    set_point(indices.last() - 1);
  }

  immEnd();
  immUnbindProgram();
}


void draw_dots(Span<float3> positions,
               const VArray<ColorGeometry4f> &colors,
               const IndexRange indices,
               const float4x4 &layer_to_world,
               const bool cyclic,
               const float line_width)
{
  /* TODO */
  return draw_curve(positions, colors, indices, layer_to_world, cyclic, line_width);
}

static auto get_stroke_mode_draw_fn(const eMaterialGPencilStyle_Mode mode)
{
  switch (mode) {
    case GP_MATERIAL_MODE_LINE:
      return draw_curve;
  case GP_MATERIAL_MODE_DOT:
  case GP_MATERIAL_MODE_SQUARE:
    return draw_dots;
  }
  return draw_curve;
}

/* draw a set of strokes */
void draw_curves_geometry(const bke::CurvesGeometry &curves,
                          const IndexMask &strokes_mask,
                          const VArray<ColorGeometry4f> &stroke_colors,
                          const float4x4 &layer_to_world,
                          const int mode,
                          const bool use_xray)
{
  GPUUniformBuf *ubo = create_shader_ubo(
      rv3d, win_size, object, caps_start, caps_end, is_fill_stroke);

  //float tcolor[4];
  //short sthickness;
  //float ink[4];
  //const bool is_unique = (tgpw->gps != nullptr);
  //const bool use_mat = (tgpw->gpd->mat != nullptr);

  GPU_program_point_size(true);

  /* Do not write to depth (avoid self-occlusion). */
  const bool prev_depth_mask = GPU_depth_mask_get();
  GPU_depth_mask(false);

  //bGPDstroke *gps_init = static_cast<bGPDstroke *>((tgpw->gps) ? tgpw->gps :
  //                                                               tgpw->t_gpf->strokes.first);

  auto stroke_draw_fn = get_stroke_mode_draw_fn(eMaterialGPencilStyle_Mode(mode));

  const OffsetIndices points_by_curve = curves.points_by_curve();
  const Span<float3> positions = curves.positions();
  const bke::AttributeAccessor attributes = curves.attributes();
  const VArray<bool> cyclic = curves.cyclic();
  const VArray<float> &radii = *attributes.lookup<float>("radius");

  strokes_mask.foreach_index([&](const int stroke_i) {
    ///* check if stroke can be drawn */
    //if (gpencil_can_draw_stroke(gps, tgpw->dflag) == false) {
    //  continue;
    //}
    ///* check if the color is visible */
    //Material *ma = (use_mat) ? tgpw->gpd->mat[gps->mat_nr] : BKE_material_default_gpencil();
    //MaterialGPencilStyle *gp_style = (ma) ? ma->gp_style : nullptr;

    //if ((gp_style == nullptr) || (gp_style->flag & GP_MATERIAL_HIDE) ||
    //    /* If onion and ghost flag do not draw. */
    //    (tgpw->onion && (gp_style->flag & GP_MATERIAL_HIDE_ONIONSKIN)))
    //{
    //  continue;
    //}

    ///* if disable fill, the colors with fill must be omitted too except fill boundary strokes */
    //if ((tgpw->disable_fill == 1) && (gp_style->fill_rgba[3] > 0.0f) &&
    //    ((gps->flag & GP_STROKE_NOFILL) == 0) && (gp_style->flag & GP_MATERIAL_FILL_SHOW))
    //{
    //  continue;
    //}

    const float stroke_radius = radii[stroke_i];
    if (stroke_radius <= 0) {
      return;
    }

    if (!use_xray) {
      GPU_depth_test(GPU_DEPTH_LESS_EQUAL);

      /* first arg is normally rv3d->dist, but this isn't
        * available here and seems to work quite well without */
      GPU_polygon_offset(1.0f, 1.0f);
    }

    /* 3D Stroke */
    ///* set color using material tint color and opacity */
    //if (!tgpw->onion) {
    //  interp_v3_v3v3(tcolor, gp_style->stroke_rgba, tgpw->tintcolor, tgpw->tintcolor[3]);
    //  tcolor[3] = gp_style->stroke_rgba[3] * tgpw->opacity;
    //  copy_v4_v4(ink, tcolor);
    //}
    //else {
    //  if (tgpw->custonion) {
    //    copy_v4_v4(ink, tgpw->tintcolor);
    //  }
    //  else {
    //    ARRAY_SET_ITEMS(tcolor, UNPACK3(gp_style->stroke_rgba), tgpw->opacity);
    //    copy_v4_v4(ink, tcolor);
    //  }
    //}

    ///* if used for fill, set opacity to 1 */
    //if (tgpw->is_fill_stroke) {
    //  if (ink[3] >= GPENCIL_ALPHA_OPACITY_THRESH) {
    //    ink[3] = 1.0f;
    //  }
    //}

    stroke_draw_fn(positions,
                   stroke_colors,
                   points_by_curve[stroke_i],
                   layer_to_world,
                   cyclic[stroke_i],
                   radii[stroke_i]);

    //if (gp_style->mode == GP_MATERIAL_MODE_DOT) {
    //  /* volumetric stroke drawing */
    //  if (tgpw->disable_fill != 1) {
    //    gpencil_draw_stroke_volumetric_3d(gps->points, gps->totpoints, sthickness, ink);
    //  }
    //}
    //else {
    //  /* 3D Lines - OpenGL primitives-based */
    //  if (gps->totpoints > 1) {
    //    tgpw->gps = gps;
    //    gpencil_draw_stroke_3d(tgpw, sthickness, ink, gps->flag & GP_STROKE_CYCLIC);
    //  }
    //}
    if (!use_xray) {
      GPU_depth_test(GPU_DEPTH_NONE);

      GPU_polygon_offset(0.0f, 0.0f);
    }
  });

  GPU_depth_mask(prev_depth_mask);
  GPU_program_point_size(false);
}

}  // namespace blender::ed::greasepencil::image_render
