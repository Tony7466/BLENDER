/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_color.hh"
#include "BLI_index_mask.hh"
#include "BLI_math_matrix.hh"
#include "BLI_rect.h"
#include "BLI_stack.hh"
#include "BLI_task.hh"

#include "BKE_attribute.hh"
#include "BKE_camera.h"
#include "BKE_context.hh"
#include "BKE_crazyspace.hh"
#include "BKE_curves.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_image.h"
#include "BKE_lib_id.hh"
#include "BKE_material.h"
#include "BKE_paint.hh"

#include "DNA_curves_types.h"
#include "DNA_material_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_view3d_types.h"

#include "DEG_depsgraph_query.hh"

#include "ED_grease_pencil.hh"
#include "ED_view3d.hh"

#include "IMB_imbuf.hh"
#include "IMB_imbuf_types.hh"

#include "GPU_framebuffer.hh"
#include "GPU_immediate.hh"
#include "GPU_matrix.hh"
#include "GPU_state.hh"
#include "GPU_texture.hh"
#include "GPU_vertex_format.hh"

#include <optional>

namespace blender::ed::greasepencil {

/* Utilities for rendering with immediate mode into an offscreen buffer. */
namespace render_utils {

/* Set up an offscreen buffer for rendering and return result as an image. */
Image *render_to_image(Main &bmain,
                       ARegion &region,
                       const int2 &win_size,
                       FunctionRef<void()> render_fn)
{
  char err_out[256] = "unknown";
  GPUOffScreen *offscreen = GPU_offscreen_create(
      win_size.x, win_size.y, true, GPU_RGBA8, GPU_TEXTURE_USAGE_HOST_READ, err_out);
  if (offscreen == nullptr) {
    return nullptr;
  }

  auto restore_region =
      [&region, winx = region.winx, winy = region.winy, winrct = region.winrct]() {
        region.winx = winx;
        region.winy = winy;
        region.winrct = winrct;
      };

  /* Resize region. */
  region.winrct.xmin = 0;
  region.winrct.ymin = 0;
  region.winrct.xmax = win_size.x;
  region.winrct.ymax = win_size.y;
  region.winx = short(win_size.x);
  region.winy = short(win_size.y);

  GPU_offscreen_bind(offscreen, true);
  const uint imb_flag = IB_rectfloat;
  ImBuf *ibuf = IMB_allocImBuf(win_size.x, win_size.y, 32, imb_flag);

  GPU_matrix_push_projection();
  GPU_matrix_identity_projection_set();
  GPU_matrix_push();
  GPU_matrix_identity_set();

  GPU_clear_color(0.0f, 0.0f, 0.0f, 0.0f);
  GPU_clear_depth(1.0f);

  /* draw strokes */
  render_fn();

  /* create a image to see result of template */
  if (ibuf->float_buffer.data) {
    GPU_offscreen_read_color(offscreen, GPU_DATA_FLOAT, ibuf->float_buffer.data);
  }
  else if (ibuf->byte_buffer.data) {
    GPU_offscreen_read_color(offscreen, GPU_DATA_UBYTE, ibuf->byte_buffer.data);
  }
  if (ibuf->float_buffer.data && ibuf->byte_buffer.data) {
    IMB_rect_from_float(ibuf);
  }

  Image *ima = BKE_image_add_from_imbuf(&bmain, ibuf, "Grease Pencil Fill");
  ima->id.tag |= LIB_TAG_DOIT;

  BKE_image_release_ibuf(ima, ibuf, nullptr);

  /* Switch back to window-system-provided frame-buffer. */
  GPU_offscreen_unbind(offscreen, true);
  GPU_offscreen_free(offscreen);

  restore_region();

  return ima;
}

static void set_viewmat(ARegion &region,
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

static void clear_viewmat()
{
  GPU_matrix_pop_projection();
  GPU_matrix_pop();
}

static void draw_dot(const float3 &position, const float point_size, const ColorGeometry4f &color)
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

/* Draw a line from points. */
static void draw_curve(Span<float3> positions,
                       const VArray<ColorGeometry4f> &colors,
                       const IndexRange indices,
                       const float4x4 &layer_to_world,
                       const bool cyclic,
                       const float line_width)
{
  GPUVertFormat *format = immVertexFormat();
  uint attr_pos = GPU_vertformat_attr_add(format, "pos", GPU_COMP_F32, 3, GPU_FETCH_FLOAT);
  uint attr_color = GPU_vertformat_attr_add(format, "color", GPU_COMP_F32, 4, GPU_FETCH_FLOAT);

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

}  // namespace render_utils

constexpr const char *attr_material_index = "material_index";
constexpr const char *attr_is_boundary = "is_boundary";

constexpr const ColorGeometry4f boundary_color = {1, 0, 0, 1};
constexpr const ColorGeometry4f fill_color = {0, 1, 0, 1};
constexpr const ColorGeometry4f seed_color = {0.5f, 0.5f, 0, 1};
constexpr const ColorGeometry4f border_color = {0, 0, 1, 1};

constexpr const ColorGeometry4f stroke_color = boundary_color;
constexpr const ColorGeometry4f extend_color = boundary_color;
constexpr const ColorGeometry4f mouse_color = seed_color;
constexpr const ColorGeometry4f helper_color = {1, 0, 0.5f, 0.5f};

bool is_pixel_boundary(const ColorGeometry4f &color)
{
  return color.r == 1.0f;
}

bool is_pixel_filled(const ColorGeometry4f &color)
{
  return color.g == 1.0f;
}

bool is_pixel_border(const ColorGeometry4f &color)
{
  return color.b == 1.0f;
}

void clear_pixel(ColorGeometry4f &color)
{
  color = ColorGeometry4f(0, 0, 0, 1);
}

void set_pixel_boundary(ColorGeometry4f &color)
{
  color.r = 1.0f;
  color.a = 1.0f;
}

void set_pixel_filled(ColorGeometry4f &color, const bool enable = true)
{
  color.g = (enable ? 1.0f : 0.0f);
  color.a = 1.0f;
}

void set_pixel_border(ColorGeometry4f &color)
{
  color.b = 1.0f;
  color.a = 1.0f;
}

static IndexMask get_boundary_curve_mask(const Object &object,
                                         const DrawingInfo &info,
                                         const bool is_boundary_layer,
                                         IndexMaskMemory &memory)
{
  const bke::CurvesGeometry &strokes = info.drawing.strokes();
  const bke::AttributeAccessor attributes = strokes.attributes();
  const VArray<int> materials = *attributes.lookup<int>(attr_material_index,
                                                        bke::AttrDomain::Curve);

  auto is_visible_curve = [&](const int curve_i) {
    /* Check if stroke can be drawn. */
    const IndexRange points = strokes.points_by_curve()[curve_i];
    if (points.size() < 2) {
      return false;
    }

    /* Check if the material is visible. */
    const Material *material = BKE_object_material_get(const_cast<Object *>(&object),
                                                       materials[curve_i] + 1);
    const MaterialGPencilStyle *gp_style = material ? material->gp_style : nullptr;
    if (gp_style == nullptr || (gp_style->flag & GP_MATERIAL_HIDE)) {
      return false;
    }

    return true;
  };

  /* On boundary layers only boundary strokes are rendered. */
  if (is_boundary_layer) {
    const VArray<bool> boundary_strokes = *attributes.lookup_or_default<bool>(
        attr_is_boundary, bke::AttrDomain::Curve, false);

    return IndexMask::from_predicate(
        strokes.curves_range(), GrainSize(512), memory, [&](const int curve_i) {
          if (!is_visible_curve(curve_i)) {
            return false;
          }
          const bool is_boundary_stroke = boundary_strokes[curve_i];
          return is_boundary_stroke;
        });
  }

  return IndexMask::from_predicate(
      strokes.curves_range(), GrainSize(512), memory, is_visible_curve);
}

static VArray<ColorGeometry4f> stroke_colors(const VArray<float> &opacities,
                                             const ColorGeometry4f &tint_color,
                                             const int64_t num_points,
                                             const float material_alpha,
                                             const float alpha_threshold,
                                             const bool brush_fill_hide)
{
  return brush_fill_hide ?
             VArray<ColorGeometry4f>::ForSingle(tint_color, num_points) :
             VArray<ColorGeometry4f>::ForFunc(
                 num_points,
                 [tint_color, opacities, material_alpha, alpha_threshold](const int64_t index) {
                   float alpha = std::clamp(material_alpha * opacities[index], 0.0f, 1.0f);
                   ColorGeometry4f color = tint_color;
                   color.a = float(alpha > alpha_threshold);
                   return color;
                 });
}

/* Draw a regular stroke. */
static void draw_basic_stroke(const Span<float3> positions,
                              const VArray<float> &opacities,
                              const IndexRange indices,
                              const float4x4 &mat,
                              const bool cyclic,
                              const float material_alpha,
                              const ColorGeometry4f tint_color,
                              const bool brush_fill_hide,
                              const float alpha_threshold,
                              const float thickness)
{
  const VArray<ColorGeometry4f> colors = stroke_colors(
      opacities, tint_color, positions.size(), material_alpha, alpha_threshold, brush_fill_hide);

  render_utils::draw_curve(positions, colors, indices, mat, cyclic, thickness);
}

/* Draw a extension stroke. */
static void draw_extension_stroke(const Span<float3> positions,
                                  const VArray<float> &opacities,
                                  const IndexRange indices,
                                  const float4x4 &mat,
                                  const bool cyclic,
                                  const float material_alpha,
                                  const bool brush_fill_hide,
                                  const float alpha_threshold,
                                  const float thickness,
                                  const bool draw_as_helper)
{
  const ColorGeometry4f &color = draw_as_helper ? helper_color : extend_color;
  const VArray<ColorGeometry4f> colors = stroke_colors(
      opacities, color, positions.size(), material_alpha, alpha_threshold, brush_fill_hide);

  render_utils::draw_curve(positions, colors, indices, mat, cyclic, thickness * 2.0f);
}

/* Draw a helper stroke (viewport only). */
static void draw_helper_stroke(Span<float3> positions,
                               const VArray<float> &opacities,
                               const IndexRange indices,
                               const float4x4 &mat,
                               const bool cyclic,
                               const float material_alpha,
                               const bool brush_fill_hide,
                               const float alpha_threshold,
                               const float thickness,
                               const bool transparent)
{
  constexpr const ColorGeometry4f helper_color_transparent = ColorGeometry4f(
      helper_color.r, helper_color.g, helper_color.b, 0.0f);
  const VArray<ColorGeometry4f> colors = transparent ?
                                             VArray<ColorGeometry4f>::ForSingle(
                                                 helper_color_transparent, positions.size()) :
                                             stroke_colors(opacities,
                                                           helper_color,
                                                           positions.size(),
                                                           material_alpha,
                                                           alpha_threshold,
                                                           brush_fill_hide);

  render_utils::draw_curve(positions, colors, indices, mat, cyclic, thickness * 2.0f);
}

/* Loop all layers to draw strokes. */
static void draw_datablock(const Object &object,
                           const GreasePencil &grease_pencil,
                           const Span<DrawingInfo> drawings,
                           const VArray<bool> &boundary_layers,
                           const ColorGeometry4f &ink,
                           const eGP_FillDrawModes /*fill_draw_mode*/,
                           const float alpha_threshold,
                           const float thickness)
{
  using bke::greasepencil::Layer;

  for (const DrawingInfo &info : drawings) {
    const Layer &layer = *grease_pencil.layers()[info.layer_index];
    if (!layer.is_visible()) {
      continue;
    }
    const float4x4 layer_to_world = layer.to_world_space(object);
    const bool is_boundary_layer = boundary_layers[info.layer_index];
    const bke::CurvesGeometry &strokes = info.drawing.strokes();
    const bke::AttributeAccessor attributes = strokes.attributes();
    const Span<float3> positions = strokes.positions();
    const VArray<float> opacities = info.drawing.opacities();
    const VArray<int> materials = *attributes.lookup<int>(attr_material_index,
                                                          bke::AttrDomain::Curve);
    const VArray<bool> cyclic = strokes.cyclic();

    IndexMaskMemory curve_mask_memory;
    const IndexMask curve_mask = get_boundary_curve_mask(
        object, info, is_boundary_layer, curve_mask_memory);

    curve_mask.foreach_index(GrainSize(512), [&](const int curve_i) {
      const IndexRange points = strokes.points_by_curve()[curve_i];
      const bool is_cyclic = cyclic[curve_i];
      const Material *material = BKE_object_material_get(const_cast<Object *>(&object),
                                                         materials[curve_i] + 1);

      const float material_alpha = material && material->gp_style ?
                                       material->gp_style->stroke_rgba[3] :
                                       1.0f;

      // tgpw.is_fill_stroke = (tgpf->fill_draw_mode == GP_FILL_DMODE_CONTROL) ? false : true;
      ///* Reduce thickness to avoid gaps. */
      // tgpw.lthick = gpl->line_change;
      // tgpw.opacity = 1.0;
      // copy_v4_v4(tgpw.tintcolor, ink);
      // tgpw.onion = true;
      // tgpw.custonion = true;

      // TODO brush flag
      const bool brush_fill_hide = false;

      draw_basic_stroke(positions,
                        opacities,
                        points,
                        layer_to_world,
                        is_cyclic,
                        material_alpha,
                        ink,
                        brush_fill_hide,
                        alpha_threshold,
                        thickness);
      /* Normal strokes. */
      // if (ELEM(fill_draw_mode, GP_FILL_DMODE_STROKE, GP_FILL_DMODE_BOTH)) {
      //   if (gpencil_stroke_is_drawable(tgpf, gps) && ((gps->flag & GP_STROKE_TAG) == 0) &&
      //       ((gps->flag & GP_STROKE_HELP) == 0))
      //   {
      //     ED_gpencil_draw_fill(&tgpw);
      //   }
      //   /* In stroke mode, still must draw the extend lines. */
      //   if (extend_lines && (tgpf->fill_draw_mode == GP_FILL_DMODE_STROKE)) {
      //     if ((gps->flag & GP_STROKE_NOFILL) && (gps->flag & GP_STROKE_TAG)) {
      //       gpencil_draw_basic_stroke(tgpf,
      //                                 gps,
      //                                 tgpw.diff_mat,
      //                                 gps->flag & GP_STROKE_CYCLIC,
      //                                 ink,
      //                                 tgpf->flag,
      //                                 tgpf->fill_threshold,
      //                                 1.0f);
      //     }
      //   }
      // }

      ///* 3D Lines with basic shapes and invisible lines. */
      // if (ELEM(tgpf->fill_draw_mode, GP_FILL_DMODE_CONTROL, GP_FILL_DMODE_BOTH)) {
      //   gpencil_draw_basic_stroke(tgpf,
      //                             gps,
      //                             tgpw.diff_mat,
      //                             gps->flag & GP_STROKE_CYCLIC,
      //                             ink,
      //                             tgpf->flag,
      //                             tgpf->fill_threshold,
      //                             1.0f);
      // }
    });
  }
}

/* Set a border to create image limits. */
static void mark_borders(Image &ima, const ColorGeometry4f &color)
{
  void *lock;
  ImBuf *ibuf = BKE_image_acquire_ibuf(&ima, nullptr, &lock);
  const int width = ibuf->x;
  const int height = ibuf->y;
  MutableSpan<ColorGeometry4f> pixels(reinterpret_cast<ColorGeometry4f *>(ibuf->float_buffer.data),
                                      width * height);

  int row_start = 0;
  /* Fill first row */
  for (const int i : IndexRange(width)) {
    pixels[row_start + i] = color;
  }
  row_start += width;
  /* Fill first and last pixel of middle rows. */
  for (const int i : IndexRange(height).drop_front(1).drop_back(1)) {
    pixels[row_start] = color;
    pixels[row_start + width - 1] = color;
    row_start += width;
  }
  /* Fill last row */
  for (const int i : IndexRange(width)) {
    pixels[row_start + i] = color;
  }

  /* release ibuf */
  BKE_image_release_ibuf(&ima, ibuf, lock);

  ima.id.tag |= LIB_TAG_DOIT;
}

enum class FillResult {
  Success,
  BorderContact,
};

FillResult fill_boundaries(Image &ima)
{
  void *lock;
  ImBuf *ibuf = BKE_image_acquire_ibuf(&ima, nullptr, &lock);
  const int width = ibuf->x;
  const int height = ibuf->y;
  MutableSpan<ColorGeometry4f> pixels(reinterpret_cast<ColorGeometry4f *>(ibuf->float_buffer.data),
                                      width * height);
  auto coord_from_index = [&](const int index) {
    const div_t d = div(index, width);
    return int2{d.rem, d.quot};
  };
  auto index_from_coord = [&](const int2 &c) { return c.x + c.y * width; };
  auto pixel_from_coord = [&](const int2 &c) { return pixels[index_from_coord(c)]; };

  blender::Stack<int> active_pixels;
  /* Initialize the stack with filled pixels (dot at mouse position). */
  for (const int i : pixels.index_range()) {
    if (math::is_equal(float4(pixels[i]), float4(seed_color), 0.01f)) {
      active_pixels.push(i);
      pixels[i] = {0, 0, 0, 1};
    }
  }

  constexpr const int filter_width = 3;
  enum FilterDirection {
    Horizontal = 1,
    Vertical = 2,
  };

  bool border_contact = false;
  while (!active_pixels.is_empty()) {
    const int index = active_pixels.pop();
    const int2 coord = coord_from_index(index);
    ColorGeometry4f pixel_value = pixels[index];

    if (is_pixel_border(pixel_value)) {
      border_contact = true;
      break;
    }
    if (is_pixel_filled(pixel_value)) {
      /* Pixel already filled. */
      continue;
    }

    if (is_pixel_boundary(pixel_value)) {
      /* Boundary pixel, ignore. */
      continue;
    }

    /* Mark as filled. */
    set_pixel_filled(pixels[index]);

    /* Directional box filtering for gap detection. */
    const IndexRange filter_x_neg = IndexRange(1, std::min(coord.x, filter_width));
    const IndexRange filter_x_pos = IndexRange(1, std::min(width - 1 - coord.x, filter_width));
    const IndexRange filter_y_neg = IndexRange(1, std::min(coord.y, filter_width));
    const IndexRange filter_y_pos = IndexRange(1, std::min(height - 1 - coord.y, filter_width));
    bool is_boundary_horizontal = false;
    bool is_boundary_vertical = false;
    for (const int filter_i : filter_y_neg) {
      is_boundary_horizontal |= is_pixel_boundary(pixel_from_coord(coord - int2(0, filter_i)));
    }
    for (const int filter_i : filter_y_pos) {
      is_boundary_horizontal |= is_pixel_boundary(pixel_from_coord(coord + int2(0, filter_i)));
    }
    for (const int filter_i : filter_x_neg) {
      is_boundary_vertical |= is_pixel_boundary(pixel_from_coord(coord - int2(filter_i, 0)));
    }
    for (const int filter_i : filter_x_pos) {
      is_boundary_vertical |= is_pixel_boundary(pixel_from_coord(coord + int2(filter_i, 0)));
    }

    /* Activate neighbors */
    if (coord.x > 0 && !is_boundary_horizontal) {
      active_pixels.push(index_from_coord(coord - int2{1, 0}));
    }
    if (coord.x < width - 1 && !is_boundary_horizontal) {
      active_pixels.push(index_from_coord(coord + int2{1, 0}));
    }
    if (coord.y > 0 && !is_boundary_vertical) {
      active_pixels.push(index_from_coord(coord - int2{0, 1}));
    }
    if (coord.y < height - 1 && !is_boundary_vertical) {
      active_pixels.push(index_from_coord(coord + int2{0, 1}));
    }
  }

  BKE_image_release_ibuf(&ima, ibuf, lock);

  return border_contact ? FillResult::BorderContact : FillResult::Success;
}

void invert_fill(Image &ima)
{
  void *lock;
  ImBuf *ibuf = BKE_image_acquire_ibuf(&ima, nullptr, &lock);
  const int width = ibuf->x;
  const int height = ibuf->y;
  MutableSpan<ColorGeometry4f> pixels(reinterpret_cast<ColorGeometry4f *>(ibuf->float_buffer.data),
                                      width * height);

  /* Initialize the stack with filled pixels (dot at mouse position). */
  for (const int i : pixels.index_range()) {
    set_pixel_filled(pixels[i], !is_pixel_filled(pixels[i]));
  }

  BKE_image_release_ibuf(&ima, ibuf, lock);
}

constexpr const int num_directions = 8;
static const int2 offset_by_direction[num_directions] = {
    {-1, -1},
    {0, -1},
    {1, -1},
    {1, 0},
    {1, 1},
    {0, 1},
    {-1, 1},
    {-1, 0},
};

/* Wrap to valid direction, must be less than 2 * num_directions. */
static int wrap_dir_2n(const int dir)
{
  return dir - num_directions * int(dir >= num_directions);
}

/* Wrap to valid direction, must be less than 3 * num_directions. */
static int wrap_dir_3n(const int dir)
{
  return dir - num_directions * (int(dir >= num_directions) + int(dir >= 2 * num_directions));
}

struct BoundaryStep {
  int2 coord;
  int direction;
};

/* Get the outline points of a shape using Moore Neighborhood algorithm
 *
 * This is a Blender customized version of the general algorithm described
 * in https://en.wikipedia.org/wiki/Moore_neighborhood
 */
static Vector<BoundaryStep> build_fill_boundary(Image &ima)
{
  void *lock;
  ImBuf *ibuf = BKE_image_acquire_ibuf(&ima, nullptr, &lock);
  const int width = ibuf->x;
  const int height = ibuf->y;
  MutableSpan<ColorGeometry4f> pixels(reinterpret_cast<ColorGeometry4f *>(ibuf->float_buffer.data),
                                      width * height);
  auto coord_from_index = [&](const int index) {
    const div_t d = div(index, width);
    return int2{d.rem, d.quot};
  };
  auto index_from_coord = [&](const int2 &c) { return c.x + c.y * width; };
  auto pixel_from_coord = [&](const int2 &c) { return pixels[index_from_coord(c)]; };
  auto is_valid_coord = [width, height](const int2 &c) -> bool {
    return c.x >= 0 && c.x < width && c.y >= 0 && c.y < height;
  };

  /* Find any filled pixel as starting point. */
  auto find_start_coordinate = [&]() -> std::optional<BoundaryStep> {
    int dir = -1;
    for (const int y : IndexRange(height)) {
      for (const int x : IndexRange(width)) {
        const int2 coord = {x, y};
        const int index = index_from_coord(coord);
        if (is_pixel_filled(pixels[index])) {
          /* Direction 3: (1, 0). Found the first filled pixel in the row. */
          return BoundaryStep{coord, 3};
        }
      }
    }
    return std::nullopt;
  };
  /* Find the next filled pixel in clockwise direction from the current. */
  auto find_next_neighbor = [&](const BoundaryStep &iter) -> std::optional<BoundaryStep> {
    for (const int i : IndexRange(num_directions).drop_front(1)) {
      /* Invert direction (add 4) and start at next direction (add 1..n).
       * This can not be greater than 3*num_directions-1, wrap accordingly. */
      const int neighbor_dir = wrap_dir_3n(iter.direction + 4 + i);
      const int2 neighbor_coord = iter.coord + offset_by_direction[neighbor_dir];
      if (!is_valid_coord(neighbor_coord)) {
        continue;
      }
      const int neighbor_index = index_from_coord(neighbor_coord);
      if (!is_pixel_filled(pixels[neighbor_index])) {
        continue;
      }

      return BoundaryStep{neighbor_coord, neighbor_dir};
    }
    return std::nullopt;
  };

  const std::optional<BoundaryStep> start_iter = find_start_coordinate();
  if (!start_iter) {
    return {};
  }

  Vector<BoundaryStep> boundary_steps = {*start_iter};
  while (true) {
    std::optional<BoundaryStep> next_iter = find_next_neighbor(boundary_steps.last());
    if (!next_iter) {
      break;
    }
    if (next_iter->coord == start_iter->coord) {
      break;
    }
    boundary_steps.append(*next_iter);
  }

  /* release ibuf */
  BKE_image_release_ibuf(&ima, ibuf, lock);

  return boundary_steps;
}

/* Create curves geometry from boundary positions. */
static bke::CurvesGeometry boundary_to_curves(const ed::greasepencil::DrawingPlacement &placement,
                                              const Span<BoundaryStep> boundary)
{
  bke::CurvesGeometry curves(boundary.size(), 1);

  MutableSpan<float3> positions = curves.positions_for_write();
  for (const int index : positions.index_range()) {
    positions[index] = placement.project(float2(boundary[index].coord));
  }

  return curves;
}

static rctf get_region_bounds(const ARegion &region)
{
  /* Init maximum boundbox size. */
  rctf region_bounds;
  BLI_rctf_init(&region_bounds, 0, region.winx, 0, region.winy);
  return region_bounds;
}

/* Helper: Calc the maximum bounding box size of strokes to get the zoom level of the viewport.
 * For each stroke, the 2D projected bounding box is calculated and using this data, the total
 * object bounding box (all strokes) is calculated. */
static rctf get_boundary_bounds(const ARegion &region,
                                const Brush &brush,
                                const Object &object,
                                const Object &object_eval,
                                const VArray<bool> &boundary_layers,
                                const Span<DrawingInfo> src_drawings)
{
  using bke::greasepencil::Drawing;
  using bke::greasepencil::Layer;

  rctf bounds;
  BLI_rctf_init_minmax(&bounds);

  if (brush.gpencil_settings->flag & GP_BRUSH_FILL_FIT_DISABLE) {
    return bounds;
  }

  BLI_assert(object.type == OB_GREASE_PENCIL);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object.data);

  BLI_assert(grease_pencil.has_active_layer());

  for (const DrawingInfo &info : src_drawings) {
    const Layer &layer = *grease_pencil.layers()[info.layer_index];
    const float4x4 layer_to_world = layer.to_world_space(object);
    const bke::crazyspace::GeometryDeformation deformation =
        bke::crazyspace::get_evaluated_grease_pencil_drawing_deformation(
            &object_eval, object, info.layer_index, info.frame_number);
    const bool only_boundary_strokes = boundary_layers[info.layer_index];
    const bke::CurvesGeometry &strokes = info.drawing.strokes();
    const bke::AttributeAccessor attributes = strokes.attributes();
    const VArray<int> materials = *attributes.lookup<int>(attr_material_index,
                                                          bke::AttrDomain::Curve);
    const VArray<bool> is_boundary_stroke = *attributes.lookup_or_default<bool>(
        "is_boundary", bke::AttrDomain::Curve, false);

    IndexMaskMemory curve_mask_memory;
    const IndexMask curve_mask = get_boundary_curve_mask(
        object, info, only_boundary_strokes, curve_mask_memory);

    curve_mask.foreach_index(GrainSize(512), [&](const int curve_i) {
      const IndexRange points = strokes.points_by_curve()[curve_i];
      /* Check if stroke can be drawn. */
      if (points.size() < 2) {
        return;
      }
      /* check if the color is visible */
      const int material_index = materials[curve_i];
      Material *mat = BKE_object_material_get(const_cast<Object *>(&object), material_index + 1);
      if (mat == 0 || (mat->gp_style->flag & GP_MATERIAL_HIDE)) {
        return;
      }

      /* If the layer must be skipped, but the stroke is not boundary, skip stroke. */
      if (only_boundary_strokes && !is_boundary_stroke[curve_i]) {
        return;
      }

      for (const int point_i : points) {
        const float3 pos_world = math::transform_point(layer_to_world,
                                                       deformation.positions[point_i]);
        float2 pos_view;
        eV3DProjStatus result = ED_view3d_project_float_global(
            &region, pos_world, pos_view, V3D_PROJ_TEST_NOP);
        if (result == V3D_PROJ_RET_OK) {
          BLI_rctf_do_minmax_v(&bounds, pos_view);
        }
      }
    });
  }

  return bounds;
}

Curves *curves_new_nomain(const int points_num, const int curves_num)
{
  BLI_assert(points_num >= 0);
  BLI_assert(curves_num >= 0);
  Curves *curves_id = static_cast<Curves *>(BKE_id_new_nomain(ID_CV, nullptr));
  bke::CurvesGeometry &curves = curves_id->geometry.wrap();
  curves.resize(points_num, curves_num);
  return curves_id;
}

bke::CurvesGeometry fill_strokes(ARegion &region,
                                 View3D &view3d,
                                 RegionView3D &rv3d,
                                 Main &bmain,
                                 const Scene &scene,
                                 Depsgraph &depsgraph,
                                 Object &object,
                                 const bke::greasepencil::Layer &layer,
                                 const VArray<bool> &boundary_layers,
                                 const Span<DrawingInfo> src_drawings,
                                 const bool invert,
                                 const float2 &fill_point,
                                 const bool keep_image)
{
  using bke::greasepencil::Layer;

  BLI_assert(object.type == OB_GREASE_PENCIL);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object.data);
  const Object &object_eval = *DEG_get_evaluated_object(&depsgraph, &object);

  // TODO based on the fill_factor (aka "Precision") setting.
  constexpr const int min_window_size = 128;
  const float pixel_scale = 1.0f;
  const int2 win_size = math::max(int2(region.winx, region.winy) * pixel_scale,
                                  int2(min_window_size));

  // TODO
  const eGP_FillDrawModes fill_draw_mode = GP_FILL_DMODE_BOTH;
  const float alpha_threshold = 0.2f;
  const float thickness = 1.0f;

  const float4x4 layer_to_world = layer.to_world_space(object);
  ed::greasepencil::DrawingPlacement placement(scene, region, view3d, object_eval, layer);
  const float3 fill_point_world = math::transform_point(layer_to_world,
                                                        placement.project(fill_point));

#if 0  // TODO doesn't quite work yet, use identity for now.
    /* Zoom and offset based on bounds, to fit all strokes within the render. */
    const rctf bounds = get_boundary_bounds(
        region, brush, object, object_eval, boundary_layers, src_drawings);
    const rctf region_bounds = get_region_bounds(region);
    const float2 bounds_max = float2(bounds.xmax, bounds.ymax);
    const float2 bounds_min = float2(bounds.xmin, bounds.ymin);
    // const float2 bounds_center = 0.5f * (bounds_min + bounds_max);
    const float2 bounds_extent = bounds_max - bounds_min;
    const float2 region_max = float2(region_bounds.xmax, region_bounds.ymax);
    const float2 region_min = float2(region_bounds.xmin, region_bounds.ymin);
    // const float2 region_center = 0.5f * (region_min + region_max);
    const float2 region_extent = region_max - region_min;
    const float2 zoom = math::safe_divide(bounds_extent, region_extent);
    const float2 offset = math::safe_divide(-bounds_min, region_extent);
    std::cout << "region " << region_min << ".." << region_max << ", bounds " << bounds_min << ".."
              << bounds_max << " zoom=" << zoom << " offset=" << offset << std::endl;
#else
  const float2 zoom = float2(1);
  const float2 offset = float2(0);
#endif

  Image *ima = render_utils::render_to_image(bmain, region, win_size, [&]() {
    GPU_blend(GPU_BLEND_ALPHA);
    GPU_depth_mask(true);
    render_utils::set_viewmat(region, view3d, rv3d, depsgraph, scene, win_size, zoom, offset);

    /* Draw blue point where click with mouse. */
    const float mouse_dot_size = 4.0f;
    render_utils::draw_dot(fill_point_world, mouse_dot_size, mouse_color);

    draw_datablock(object,
                   grease_pencil,
                   src_drawings,
                   boundary_layers,
                   stroke_color,
                   fill_draw_mode,
                   alpha_threshold,
                   thickness);

    render_utils::clear_viewmat();
    GPU_depth_mask(false);
    GPU_blend(GPU_BLEND_NONE);
  });
  if (!ima) {
    return {};
  }

  /* Set red borders to create a external limit. */
  mark_borders(*ima, border_color);

  /* Apply boundary fill */
  FillResult fill_result = fill_boundaries(*ima);
  if (fill_result != FillResult::Success) {
    return {};
  }

  if (invert) {
    invert_fill(*ima);
  }

  Vector<BoundaryStep> boundary = build_fill_boundary(*ima);
  bke::CurvesGeometry fill_curves = boundary_to_curves(placement, boundary);
  /* Delete temp image. */
  if (!keep_image) {
    BKE_id_free(&bmain, ima);
  }

  return fill_curves;
}

}  // namespace blender::ed::greasepencil
