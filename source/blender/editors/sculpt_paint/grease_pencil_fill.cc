/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_array_utils.hh"
#include "BLI_color.hh"
#include "BLI_hash.hh"
#include "BLI_index_mask.hh"
#include "BLI_linear_allocator.hh"
#include "BLI_math_matrix.hh"
#include "BLI_offset_indices.hh"
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

#include <forward_list>
#include <iostream>
#include <list>
#include <numeric>
#include <optional>

namespace blender::ed::greasepencil {

constexpr const char *attr_material_index = "material_index";
constexpr const char *attr_is_boundary = "is_boundary";

const ColorGeometry4f draw_boundary_color = {1, 0, 0, 1};
const ColorGeometry4f draw_seed_color = {0, 1, 0, 1};

const ColorGeometry4b output_boundary_color = {255, 0, 0, 255};
const ColorGeometry4b output_seed_color = {127, 127, 0, 255};
const ColorGeometry4b output_border_color = {0, 0, 255, 255};
const ColorGeometry4b output_fill_color = {127, 255, 0, 255};
const ColorGeometry4b output_extend_color = {25, 255, 0, 255};
const ColorGeometry4b output_helper_color = {255, 0, 127, 255};

enum ColorFlag {
  Border = 1 << 0,
  Stroke = 1 << 1,
  Fill = 1 << 2,
  Seed = 1 << 3,
};
ENUM_OPERATORS(ColorFlag, ColorFlag::Seed)

bool get_flag(const ColorGeometry4b &color, const ColorFlag flag)
{
  return (color.r & flag) != 0;
}

void set_flag(ColorGeometry4b &color, const ColorFlag flag, bool value)
{
  color.r = value ? (color.r | flag) : (color.r & (~flag));
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
                 [tint_color, &opacities, material_alpha, alpha_threshold](const int64_t index) {
                   float alpha = std::clamp(material_alpha * opacities[index], 0.0f, 1.0f);
                   ColorGeometry4f color = tint_color;
                   color.a = (alpha > alpha_threshold ? 255 : 0);
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

  image_render::draw_curve(positions, colors, indices, mat, cyclic, thickness);
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
  const ColorGeometry4f &color = draw_as_helper ? output_helper_color.decode() :
                                                  output_extend_color.decode();
  const VArray<ColorGeometry4f> colors = stroke_colors(
      opacities, color, positions.size(), material_alpha, alpha_threshold, brush_fill_hide);

  image_render::draw_curve(positions, colors, indices, mat, cyclic, thickness * 2.0f);
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
  const ColorGeometry4f helper_color_transparent =
      ColorGeometry4b(output_helper_color.r, output_helper_color.g, output_helper_color.b, 0.0f)
          .decode();
  const VArray<ColorGeometry4f> colors = transparent ?
                                             VArray<ColorGeometry4f>::ForSingle(
                                                 helper_color_transparent, positions.size()) :
                                             stroke_colors(opacities,
                                                           output_helper_color.decode(),
                                                           positions.size(),
                                                           material_alpha,
                                                           alpha_threshold,
                                                           brush_fill_hide);

  image_render::draw_curve(positions, colors, indices, mat, cyclic, thickness * 2.0f);
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

    /* Note: Serial loop without GrainSize, since immediate mode drawing can't happen in worker
     * threads, has to be from the main thread. */
    curve_mask.foreach_index([&](const int curve_i) {
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
/* TODO this shouldn't be necessary if drawing could accurately save flag values. */
static void convert_colors_to_flags(Image &ima)
{
  void *lock;
  ImBuf *ibuf = BKE_image_acquire_ibuf(&ima, nullptr, &lock);
  const int width = ibuf->x;
  const int height = ibuf->y;
  MutableSpan<ColorGeometry4b> pixels(reinterpret_cast<ColorGeometry4b *>(ibuf->byte_buffer.data),
                                      width * height);

  for (const int i : pixels.index_range()) {
    const ColorGeometry4b input_color = pixels[i];
    const bool is_boundary = input_color.r > 0.0f;
    const bool is_seed = input_color.g > 0.0f;
    pixels[i].r = (is_boundary ? ColorFlag::Stroke : 0) | (is_seed ? ColorFlag::Seed : 0);
    pixels[i].g = 0;
    pixels[i].b = 0;
    pixels[i].a = 0;
  }

  /* release ibuf */
  BKE_image_release_ibuf(&ima, ibuf, lock);

  ima.id.tag |= LIB_TAG_DOIT;
}

/* Set a border to create image limits. */
static void convert_flags_to_colors(Image &ima)
{
  void *lock;
  ImBuf *ibuf = BKE_image_acquire_ibuf(&ima, nullptr, &lock);
  const int width = ibuf->x;
  const int height = ibuf->y;
  MutableSpan<ColorGeometry4b> pixels(reinterpret_cast<ColorGeometry4b *>(ibuf->byte_buffer.data),
                                      width * height);

  for (const int i : pixels.index_range()) {
    const ColorGeometry4b input_color = pixels[i];
    if (input_color.r & ColorFlag::Stroke) {
      pixels[i] = output_boundary_color;
    }
    else if (input_color.r & ColorFlag::Border) {
      pixels[i] = output_border_color;
    }
    else if (input_color.r & ColorFlag::Seed) {
      pixels[i] = output_seed_color;
    }
    else if (input_color.r & ColorFlag::Fill) {
      pixels[i] = output_fill_color;
    }
    else {
      pixels[i] = ColorGeometry4b(0, 0, 0, 0);
    }
  }

  /* release ibuf */
  BKE_image_release_ibuf(&ima, ibuf, lock);

  ima.id.tag |= LIB_TAG_DOIT;
}

/* Set a border to create image limits. */
static void mark_borders(Image &ima, const ColorGeometry4b &color)
{
  void *lock;
  ImBuf *ibuf = BKE_image_acquire_ibuf(&ima, nullptr, &lock);
  const int width = ibuf->x;
  const int height = ibuf->y;
  MutableSpan<ColorGeometry4b> pixels(reinterpret_cast<ColorGeometry4b *>(ibuf->byte_buffer.data),
                                      width * height);

  int row_start = 0;
  /* Fill first row */
  for (const int i : IndexRange(width)) {
    pixels[row_start + i] = color;
  }
  row_start += width;
  /* Fill first and last pixel of middle rows. */
  for ([[maybe_unused]] const int i : IndexRange(height).drop_front(1).drop_back(1)) {
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

enum FillBorderMode {
  /* Cancel when hitting the border, fill failed. */
  Cancel,
  /* Allow border contact, continue with other pixels. */
  Ignore,
};

template<FillBorderMode border_mode>
FillResult flood_fill(Image &ima, const int leak_filter_width = 0)
{
  void *lock;
  ImBuf *ibuf = BKE_image_acquire_ibuf(&ima, nullptr, &lock);
  const int width = ibuf->x;
  const int height = ibuf->y;
  MutableSpan<ColorGeometry4b> pixels(reinterpret_cast<ColorGeometry4b *>(ibuf->byte_buffer.data),
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
    if (get_flag(pixels[i], ColorFlag::Seed)) {
      active_pixels.push(i);
    }
  }

  enum FilterDirection {
    Horizontal = 1,
    Vertical = 2,
  };

  bool border_contact = false;
  while (!active_pixels.is_empty()) {
    const int index = active_pixels.pop();
    const int2 coord = coord_from_index(index);
    ColorGeometry4b pixel_value = pixels[index];

    if constexpr (border_mode == FillBorderMode::Cancel) {
      if (get_flag(pixel_value, ColorFlag::Border)) {
        border_contact = true;
        break;
      }
    }
    else if constexpr (border_mode == FillBorderMode::Ignore) {
      if (get_flag(pixel_value, ColorFlag::Border)) {
        border_contact = true;
        continue;
      }
    }
    else {
      if (get_flag(pixel_value, ColorFlag::Border)) {
        border_contact = true;
      }
    }

    if (get_flag(pixel_value, ColorFlag::Fill)) {
      /* Pixel already filled. */
      continue;
    }

    if (get_flag(pixel_value, ColorFlag::Stroke)) {
      /* Boundary pixel, ignore. */
      continue;
    }

    /* Mark as filled. */
    set_flag(pixels[index], ColorFlag::Fill, true);

    /* Directional box filtering for gap detection. */
    const IndexRange filter_x_neg = IndexRange(1, std::min(coord.x, leak_filter_width));
    const IndexRange filter_x_pos = IndexRange(1,
                                               std::min(width - 1 - coord.x, leak_filter_width));
    const IndexRange filter_y_neg = IndexRange(1, std::min(coord.y, leak_filter_width));
    const IndexRange filter_y_pos = IndexRange(1,
                                               std::min(height - 1 - coord.y, leak_filter_width));
    bool is_boundary_horizontal = false;
    bool is_boundary_vertical = false;
    for (const int filter_i : filter_y_neg) {
      is_boundary_horizontal |= get_flag(pixel_from_coord(coord - int2(0, filter_i)),
                                         ColorFlag::Stroke);
    }
    for (const int filter_i : filter_y_pos) {
      is_boundary_horizontal |= get_flag(pixel_from_coord(coord + int2(0, filter_i)),
                                         ColorFlag::Stroke);
    }
    for (const int filter_i : filter_x_neg) {
      is_boundary_vertical |= get_flag(pixel_from_coord(coord - int2(filter_i, 0)),
                                       ColorFlag::Stroke);
    }
    for (const int filter_i : filter_x_pos) {
      is_boundary_vertical |= get_flag(pixel_from_coord(coord + int2(filter_i, 0)),
                                       ColorFlag::Stroke);
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
  MutableSpan<ColorGeometry4b> pixels(reinterpret_cast<ColorGeometry4b *>(ibuf->byte_buffer.data),
                                      width * height);

  for (const int i : pixels.index_range()) {
    const bool is_filled = get_flag(pixels[i], ColorFlag::Fill);
    set_flag(pixels[i], ColorFlag::Stroke, is_filled);
    set_flag(pixels[i], ColorFlag::Fill, !is_filled);
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

/* Wrap to valid direction, must be less than 3 * num_directions. */
static int wrap_dir_3n(const int dir)
{
  return dir - num_directions * (int(dir >= num_directions) + int(dir >= 2 * num_directions));
}

struct FillBoundary {
  /* Pixel indices making up boundary curves. */
  Vector<int> pixels;
  /* Offset index for each curve. */
  Vector<int> offset_indices;
};

/* Get the outline points of a shape using Moore Neighborhood algorithm
 *
 * This is a Blender customized version of the general algorithm described
 * in https://en.wikipedia.org/wiki/Moore_neighborhood
 */
static FillBoundary build_fill_boundary(Image &ima)
{
  void *lock;
  ImBuf *ibuf = BKE_image_acquire_ibuf(&ima, nullptr, &lock);
  const int width = ibuf->x;
  const int height = ibuf->y;
  MutableSpan<ColorGeometry4b> pixels(reinterpret_cast<ColorGeometry4b *>(ibuf->byte_buffer.data),
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

  using BoundarySection = std::list<int>;
  using BoundaryStartMap = Map<int, BoundarySection>;

  /* Find possible starting points for boundary sections.
   * Direction 3 == (1, 0) is the starting direction. */
  constexpr const uint8_t start_direction = 3;
  auto find_start_coordinates = [&]() -> BoundaryStartMap {
    BoundaryStartMap starts;
    for (const int y : IndexRange(height)) {
      /* Check for empty pixels next to filled pixels. */
      for (const int x : IndexRange(width).drop_back(1)) {
        const int index_empty = index_from_coord({x, y});
        const int index_filled = index_from_coord({x + 1, y});
        if (!get_flag(pixels[index_empty], ColorFlag::Fill) &&
            get_flag(pixels[index_filled], ColorFlag::Fill))
        {
          /* Empty index list indicates uninitialized section. */
          starts.add(index_filled, {});
        }
      }
    }
    return starts;
  };

  struct NeighborIterator {
    int index;
    int direction;
  };

  /* Find the next filled pixel in clockwise direction from the current. */
  auto find_next_neighbor = [&](NeighborIterator &iter) -> bool {
    const int2 iter_coord = coord_from_index(iter.index);
    for (const int i : IndexRange(num_directions).drop_front(1)) {
      /* Invert direction (add 4) and start at next direction (add 1..n).
       * This can not be greater than 3*num_directions-1, wrap accordingly. */
      const int neighbor_dir = wrap_dir_3n(iter.direction + 4 + i);
      const int2 neighbor_coord = iter_coord + offset_by_direction[neighbor_dir];
      if (!is_valid_coord(neighbor_coord)) {
        continue;
      }
      const int neighbor_index = index_from_coord(neighbor_coord);
      if (get_flag(pixels[neighbor_index], ColorFlag::Fill)) {
        iter.index = neighbor_index;
        iter.direction = neighbor_dir;
        return true;
      }
    }
    return false;
  };

  BoundaryStartMap boundary_starts = find_start_coordinates();

  /* Find directions and connectivity for all boundary pixels. */
  for (const int start_index : boundary_starts.keys()) {
    /* Boundary map entries may get removed, only handle active starts. */
    if (!boundary_starts.contains(start_index)) {
      continue;
    }
    BoundarySection &section = boundary_starts.lookup(start_index);
    section.push_back(start_index);
    NeighborIterator iter = {start_index, start_direction};
    while (find_next_neighbor(iter)) {
      /* Loop closed when arriving at start again. */
      if (iter.index == start_index) {
        break;
      }

      /* Join existing sections. */
      if (boundary_starts.contains(iter.index)) {
        BoundarySection &next_section = boundary_starts.lookup(iter.index);
        if (next_section.empty()) {
          /* Empty sections are only start indices, remove and continue. */
          boundary_starts.remove(iter.index);
          continue;
        }

        /* Merge existing points into the current section before removing. */
        section.splice(section.end(), next_section);
        boundary_starts.remove(iter.index);
        break;
      }

      section.push_back(iter.index);
    }
  }

  /* Construct final strokes by tracing the boundary. */
  FillBoundary final_boundary;
  for (const BoundarySection &section : boundary_starts.values()) {
    final_boundary.offset_indices.append(final_boundary.pixels.size());
    for (const int index : section) {
      final_boundary.pixels.append(index);
    }
  }
  final_boundary.offset_indices.append(final_boundary.pixels.size());

  /* release ibuf */
  BKE_image_release_ibuf(&ima, ibuf, lock);

  return final_boundary;
}

/* Create curves geometry from boundary positions. */
static bke::CurvesGeometry boundary_to_curves(const ed::greasepencil::DrawingPlacement &placement,
                                              const FillBoundary &boundary,
                                              const int2 win_size)
{
  auto coord_from_index = [&](const int index) {
    const div_t d = div(index, win_size.x);
    return int2{d.rem, d.quot};
  };

  /* Curve cannot have 0 points. */
  if (boundary.offset_indices.is_empty() || boundary.pixels.is_empty()) {
    return {};
  }

  bke::CurvesGeometry curves(boundary.pixels.size(), boundary.offset_indices.size() - 1);

  curves.offsets_for_write().copy_from(boundary.offset_indices);
  const OffsetIndices points_by_curve = curves.points_by_curve();

  MutableSpan<float3> positions = curves.positions_for_write();
  for (const int curve_i : curves.curves_range()) {
    const IndexRange points = points_by_curve[curve_i];
    for (const int point_i : points) {
      const int pixel_index = boundary.pixels[point_i];
      const int2 pixel_coord = coord_from_index(pixel_index);
      positions[point_i] = placement.project(float2(pixel_coord));
    }
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
  const int leak_filter_width = 3;

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

  image_render::ImageRenderData *data = image_render::image_render_begin(region, win_size);
  GPU_blend(GPU_BLEND_ALPHA);
  GPU_depth_mask(true);
  image_render::set_viewmat(region, view3d, rv3d, depsgraph, scene, win_size, zoom, offset);

  /* Draw blue point where click with mouse. */
  const float mouse_dot_size = 4.0f;
  image_render::draw_dot(fill_point_world, mouse_dot_size, draw_seed_color);

  draw_datablock(object,
                  grease_pencil,
                  src_drawings,
                  boundary_layers,
                  draw_boundary_color,
                  fill_draw_mode,
                  alpha_threshold,
                  thickness);

  image_render::clear_viewmat();
  GPU_depth_mask(false);
  GPU_blend(GPU_BLEND_NONE);
  Image *ima = image_render::image_render_end(bmain, region, data);

  if (!ima) {
    return {};
  }

  convert_colors_to_flags(*ima);

  /* Set red borders to create a external limit. */
  mark_borders(*ima, output_border_color);

  /* Apply boundary fill */
  if (invert) {
    /* When inverted accept border fill, image borders are valid boundaries. */
    FillResult fill_result = flood_fill<FillBorderMode::Ignore>(*ima, leak_filter_width);
    if (!ELEM(fill_result, FillResult::Success, FillResult::BorderContact)) {
      return {};
    }
    /* Make fills into boundaries and vice versa for finding exterior boundaries. */
    invert_fill(*ima);
  }
  else {
    /* Cancel when encountering a border, counts as failure. */
    FillResult fill_result = flood_fill<FillBorderMode::Cancel>(*ima, leak_filter_width);
    if (fill_result != FillResult::Success) {
      return {};
    }
  }

  FillBoundary boundary = build_fill_boundary(*ima);
  bke::CurvesGeometry fill_curves = boundary_to_curves(placement, boundary, win_size);
  /* Delete temp image. */
  if (keep_image) {
    convert_flags_to_colors(*ima);
  }
  else {
    BKE_id_free(&bmain, ima);
  }

  return fill_curves;
}

}  // namespace blender::ed::greasepencil
