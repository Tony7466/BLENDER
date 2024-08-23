/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <optional>

#include "BKE_context.hh"
#include "BKE_curves.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_paint.hh"

#include "BLI_task.hh"

#include "grease_pencil_intern.hh"

namespace blender::ed::sculpt_paint::greasepencil {

struct ColorGrid {
  /* Flat array of colors. The length of this is size^2. */
  Array<float4> colors;
  /* Size of the grid. Used as the width and height. Should be divisible by 2. */
  int size;
  /* The size of each cell in pixels (screen space). Used as the cell width and height. */
  int cell_size_px;
  /* The center position of the grid (screen space). */
  float2 center;
};

class VertexSmearOperation : public GreasePencilStrokeOperationCommon {
  using GreasePencilStrokeOperationCommon::GreasePencilStrokeOperationCommon;

 public:
  void on_stroke_begin(const bContext &C, const InputSample &start_sample) override;
  void on_stroke_extended(const bContext &C, const InputSample &extension_sample) override;
  void on_stroke_done(const bContext &C) override;

 private:
  ColorGrid color_grid_;

  void init_color_grid(const bContext &C, float2 start_position);

  float2 grid_pos_to_coords(const int2 pos, const float2 center) const
  {
    const float2 centered = float2(pos - color_grid_.size / 2) + float2(0.5f);
    return (centered * color_grid_.cell_size_px) + center;
  }

  int2 coords_to_grid_pos(const float2 coord, const float2 center) const
  {
    const int2 pos = int2(math::floor((coord - center) / float(color_grid_.cell_size_px)));
    return pos + color_grid_.size / 2;
  }

  int grid_pos_to_grid_index(const int2 pos) const
  {
    return math::clamp(
        pos.y * color_grid_.size + pos.x, 0, (color_grid_.size * color_grid_.size) - 1);
  }

  template<typename Fn>
  void foreach_point_on_grid(const IndexMask &points,
                             const Span<float2> positions,
                             const float2 offset,
                             const GrainSize grain_size,
                             Fn &&fn)
  {
    const int size = color_grid_.size;
    points.foreach_index(grain_size, [&](const int64_t point) {
      const int2 grid_pos = this->coords_to_grid_pos(positions[point], offset);
      /* Check if we intersect the grid and call the callback if we do. */
      if (grid_pos.x >= 0 && grid_pos.x < size && grid_pos.y >= 0 && grid_pos.y < size) {
        fn(point, grid_pos);
      }
    });
  }
};

void VertexSmearOperation::init_color_grid(const bContext &C, const float2 start_position)
{
  const Scene &scene = *CTX_data_scene(&C);
  Paint &paint = *BKE_paint_get_active_from_context(&C);
  const Brush &brush = *BKE_paint_brush(&paint);
  const bool is_masking = GPENCIL_ANY_VERTEX_MASK(
      eGP_vertex_SelectMaskFlag(scene.toolsettings->gpencil_selectmode_vertex));
  const float radius = brush_radius(scene, brush, 1.0f);

  /* Setup grid values. */
  color_grid_.cell_size_px = 10.0f;
  color_grid_.center = start_position;
  color_grid_.size = int(math::ceil((radius * 2.0f) / color_grid_.cell_size_px));

  /* Initialize the color array. */
  const int grid_array_length = color_grid_.size * color_grid_.size;
  color_grid_.colors.reinitialize(grid_array_length);
  color_grid_.colors.fill(float4(0.0f));

  /* Initialize grid values. */
  this->foreach_editable_drawing(C, [&](const GreasePencilStrokeParams &params) {
    IndexMaskMemory memory;
    const IndexMask point_selection = point_selection_mask(params, is_masking, memory);
    if (!point_selection.is_empty()) {
      const Array<float2> view_positions = calculate_view_positions(params, point_selection);
      const Array<float> radii = calculate_view_radii(params, point_selection);
      const VArray<ColorGeometry4f> vertex_colors = params.drawing.vertex_colors();
      /* Compute the colors in the grid by averaging the vertex colors of the points that
       * intersect each cell. */
      Array<int> points_per_cell(grid_array_length, 0);
      for (const int y : IndexRange(color_grid_.size)) {
        for (const int x : IndexRange(color_grid_.size)) {
          const int2 grid_pos = int2(x, y);
          const int cell_i = this->grid_pos_to_grid_index(grid_pos);
          const float2 cell_pos = this->grid_pos_to_coords(grid_pos, color_grid_.center);
          point_selection.foreach_index([&](const int point_i) {
            const float2 view_pos = view_positions[point_i];
            const float view_radius = radii[point_i];
            const ColorGeometry4f color = vertex_colors[point_i];
            if (math::distance_squared(cell_pos, view_pos) <= view_radius * view_radius) {
              color_grid_.colors[cell_i] += float4(color.r, color.g, color.b, 1.0f);
              points_per_cell[cell_i]++;
            }
          });
        }
      }
      /* Divide by the total to get the average color per cell. */
      for (const int cell_i : color_grid_.colors.index_range()) {
        if (points_per_cell[cell_i] > 0) {
          color_grid_.colors[cell_i] *= 1.0f / float(points_per_cell[cell_i]);
        }
      }
    }
    return false;
  });
}

void VertexSmearOperation::on_stroke_begin(const bContext &C, const InputSample &start_sample)
{
  this->init_stroke(C, start_sample);
  this->init_color_grid(C, start_sample.mouse_position);
}

void VertexSmearOperation::on_stroke_extended(const bContext &C,
                                              const InputSample &extension_sample)
{
  const Scene &scene = *CTX_data_scene(&C);
  Paint &paint = *BKE_paint_get_active_from_context(&C);
  const Brush &brush = *BKE_paint_brush(&paint);
  const float radius = brush_radius(scene, brush, extension_sample.pressure);

  const bool is_masking = GPENCIL_ANY_VERTEX_MASK(
      eGP_vertex_SelectMaskFlag(scene.toolsettings->gpencil_selectmode_vertex));

  this->foreach_editable_drawing(C, GrainSize(1), [&](const GreasePencilStrokeParams &params) {
    IndexMaskMemory memory;
    const IndexMask point_selection = point_selection_mask(params, is_masking, memory);
    if (!point_selection.is_empty()) {
      const Array<float2> view_positions = calculate_view_positions(params, point_selection);
      MutableSpan<ColorGeometry4f> vertex_colors = params.drawing.vertex_colors_for_write();
      this->foreach_point_on_grid(
          point_selection,
          view_positions,
          extension_sample.mouse_position,
          GrainSize(1024),
          [&](const int point_i, const int2 grid_pos) {
            const int cell_i = this->grid_pos_to_grid_index(grid_pos);
            if (color_grid_.colors[cell_i][3] == 0.0f) {
              return;
            }
            const ColorGeometry4f mix_color = ColorGeometry4f(color_grid_.colors[cell_i]);
            const float2 view_pos = view_positions[point_i];
            const float distance_falloff = math::clamp(
                1.0f - (math::distance(color_grid_.center, view_pos) / radius * 2), 0.0f, 1.0f);
            const float influence = brush_point_influence(scene,
                                                          brush,
                                                          view_pos,
                                                          extension_sample,
                                                          params.multi_frame_falloff) *
                                    distance_falloff;
            if (influence > 0.0f) {
              ColorGeometry4f &color = vertex_colors[point_i];
              const float alpha = color.a;
              color = math::interpolate(color, mix_color, influence);
              color.a = alpha;
            }
          });
      return true;
    }
    return false;
  });
}

void VertexSmearOperation::on_stroke_done(const bContext & /*C*/) {}

std::unique_ptr<GreasePencilStrokeOperation> new_vertex_smear_operation()
{
  return std::make_unique<VertexSmearOperation>();
}

}  // namespace blender::ed::sculpt_paint::greasepencil
