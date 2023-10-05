/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_brush.hh"
#include "BKE_colortools.h"
#include "BKE_context.h"
#include "BKE_curves.hh"
#include "BKE_grease_pencil.h"
#include "BKE_grease_pencil.hh"
#include "BKE_material.h"
#include "BKE_scene.h"

#include "BLI_length_parameterize.hh"
#include "BLI_math_geom.h"

#include "DEG_depsgraph_query.hh"

#include "ED_grease_pencil.hh"
#include "ED_view3d.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "grease_pencil_intern.hh"

namespace blender::ed::sculpt_paint::greasepencil {

static constexpr float POINT_OVERRIDE_THRESHOLD_PX = 3.0f;
static constexpr float POINT_RESAMPLE_MIN_DISTANCE_PX = 10.0f;

static constexpr int64_t STOKE_CACHE_ALLOCATION_CHUNK_SIZE = 1024;

/** Sample a bezier curve at a fixed resolution and return the sampled points in an array. */
static Array<float2> sample_curve_2d(Span<float2> curve_points, const int64_t resolution)
{
  BLI_assert(curve_points.size() % 3 == 0);
  const int64_t num_handles = curve_points.size() / 3;
  const int64_t num_segments = num_handles - 1;
  const int64_t num_points = num_segments * resolution;

  Array<float2> points(num_points);
  const Span<float2> curve_segments = curve_points.drop_front(1).drop_back(1);
  for (const int64_t segment_i : IndexRange(num_segments)) {
    IndexRange segment_range(segment_i * resolution, resolution);
    bke::curves::bezier::evaluate_segment(curve_segments[segment_i * 3 + 0],
                                          curve_segments[segment_i * 3 + 1],
                                          curve_segments[segment_i * 3 + 2],
                                          curve_segments[segment_i * 3 + 3],
                                          points.as_mutable_span().slice(segment_range));
  }
  return points;
}

/** Morph \a src onto \a target such that the points have the same spacing as in \a src and
 *  write the result to \a dst. */
static void morph_points_to_curve(Span<float2> src, Span<float2> target, MutableSpan<float2> dst)
{
  BLI_assert(src.size() == dst.size());
  Array<float> accumulated_lengths_src(src.size() - 1);
  length_parameterize::accumulate_lengths<float2>(src, false, accumulated_lengths_src);

  Array<float> accumulated_lengths_target(target.size() - 1);
  length_parameterize::accumulate_lengths<float2>(target, false, accumulated_lengths_target);

  Array<int> segment_indices(accumulated_lengths_src.size());
  Array<float> segment_factors(accumulated_lengths_src.size());
  length_parameterize::sample_at_lengths(
      accumulated_lengths_target, accumulated_lengths_src, segment_indices, segment_factors);

  length_parameterize::interpolate<float2>(
      target, segment_indices, segment_factors, dst.drop_back(1));
  dst.last() = src.last();
}

struct ScreenSpacePoint {
  float2 co;
  float radius;
  float opacity;
  float4 vertex_color;
};

class PaintOperation : public GreasePencilStrokeOperation {
 private:
  Vector<float2> screen_space_coordinates_;
  Vector<Vector<float2>> screen_space_curve_fitted_coordinates_;
  Vector<float2> screen_space_smoothed_coordinates_;
  int64_t active_smooth_index_ = 0;

  friend struct PaintOperationExecutor;

 public:
  void on_stroke_begin(const bContext &C, const InputSample &start_sample) override;
  void on_stroke_extended(const bContext &C, const InputSample &extension_sample) override;
  void on_stroke_done(const bContext &C) override;

 private:
  void simplify_stroke(bke::greasepencil::Drawing &drawing, float epsilon_px);
  void process_stroke_end(bke::greasepencil::Drawing &drawing);
};

/**
 * Utility class that actually executes the update when the stroke is updated. That's useful
 * because it avoids passing a very large number of parameters between functions.
 */
struct PaintOperationExecutor {
  ARegion *region_;
  GreasePencil *grease_pencil_;

  Brush *brush_;
  int brush_size_;
  float brush_alpha_;

  BrushGpencilSettings *settings_;
  float4 vertex_color_;

  bke::greasepencil::Drawing *drawing_;

  PaintOperationExecutor(const bContext &C)
  {
    Scene *scene = CTX_data_scene(&C);
    region_ = CTX_wm_region(&C);
    Object *object = CTX_data_active_object(&C);
    grease_pencil_ = static_cast<GreasePencil *>(object->data);

    Paint *paint = &scene->toolsettings->gp_paint->paint;
    brush_ = BKE_paint_brush(paint);
    settings_ = brush_->gpencil_settings;
    brush_size_ = BKE_brush_size_get(scene, brush_);
    brush_alpha_ = BKE_brush_alpha_get(scene, brush_);

    const bool use_vertex_color = (scene->toolsettings->gp_paint->mode ==
                                   GPPAINT_FLAG_USE_VERTEXCOLOR);
    const bool use_vertex_color_stroke = use_vertex_color && ELEM(settings_->vertex_mode,
                                                                  GPPAINT_MODE_STROKE,
                                                                  GPPAINT_MODE_BOTH);
    vertex_color_ = use_vertex_color_stroke ? float4(brush_->rgb[0],
                                                     brush_->rgb[1],
                                                     brush_->rgb[2],
                                                     settings_->vertex_factor) :
                                              float4(0.0f);

    // const bool use_vertex_color_fill = use_vertex_color && ELEM(
    //     brush->gpencil_settings->vertex_mode, GPPAINT_MODE_STROKE, GPPAINT_MODE_BOTH);

    /* The object should have an active layer. */
    BLI_assert(grease_pencil_->has_active_layer());
    bke::greasepencil::Layer &active_layer = *grease_pencil_->get_active_layer_for_write();
    const int drawing_index = active_layer.drawing_index_at(scene->r.cfra);

    /* Drawing should exist. */
    BLI_assert(drawing_index >= 0);
    drawing_ =
        &reinterpret_cast<GreasePencilDrawing *>(grease_pencil_->drawing(drawing_index))->wrap();
  }

  float3 screen_space_to_object_space(float2 co)
  {
    /* TODO: Use correct plane/projection. */
    float4 plane{0.0f, -1.0f, 0.0f, 0.0f};
    /* TODO: Use object transform. */
    float3 proj_point;
    ED_view3d_win_to_3d_on_plane(region_, plane, co, false, proj_point);
    return proj_point;
  }

  ScreenSpacePoint point_from_input_sample(const InputSample &sample)
  {
    ScreenSpacePoint point;
    point.co = sample.mouse_position;
    point.radius = brush_size_ / 2.0f;
    if (BKE_brush_use_size_pressure(brush_)) {
      point.radius *= BKE_curvemapping_evaluateF(settings_->curve_sensitivity, 0, sample.pressure);
    }
    point.opacity = brush_alpha_;
    if (BKE_brush_use_alpha_pressure(brush_)) {
      point.opacity *= BKE_curvemapping_evaluateF(settings_->curve_strength, 0, sample.pressure);
    }
    point.vertex_color = vertex_color_;
    /* TODO: Get fill vertex color. */
    return point;
  }

  void process_start_sample(PaintOperation &self,
                            const InputSample &start_sample,
                            const int material_index)
  {
    ScreenSpacePoint start_point = this->point_from_input_sample(start_sample);

    self.screen_space_coordinates_.append(start_point.co);
    self.screen_space_curve_fitted_coordinates_.append(Vector<float2>({start_point.co}));
    self.screen_space_smoothed_coordinates_.append(start_point.co);

    /* Resize the curves geometry so there is one more curve with a single point. */
    bke::CurvesGeometry &curves = drawing_->strokes_for_write();
    int num_old_points = curves.points_num();
    curves.resize(curves.points_num() + 1, curves.curves_num() + 1);
    curves.offsets_for_write().last(1) = num_old_points;

    curves.positions_for_write().last() = screen_space_to_object_space(start_point.co);
    drawing_->radii_for_write().last() = start_point.radius;
    drawing_->opacities_for_write().last() = start_point.opacity;
    drawing_->vertex_colors_for_write().last() = ColorGeometry4f(start_point.vertex_color);

    bke::MutableAttributeAccessor attributes = curves.attributes_for_write();
    bke::SpanAttributeWriter<int> materials = attributes.lookup_or_add_for_write_span<int>(
        "material_index", ATTR_DOMAIN_CURVE);
    materials.span.last() = material_index;
    materials.finish();

    curves.curve_types_for_write().last() = CURVE_TYPE_POLY;
    curves.update_curve_types();

    drawing_->tag_topology_changed();
  }

  void active_smoothing(PaintOperation &self,
                        const IndexRange points_range,
                        const IndexRange smooth_window)
  {
    Span<float2> screen_space_coords_smooth_slice = self.screen_space_coordinates_.as_span().slice(
        smooth_window);

    /* Detect corners in the current slice of coordinates. */
    const float corner_min_radius_px = 5.0f;
    const float corner_max_radius_px = 30.0f;
    const int64_t corner_max_samples = 64;
    const float corner_angle_threshold = 0.6f;
    IndexMaskMemory memory;
    IndexMask corner_mask = ed::greasepencil::polyline_detect_corners(
        screen_space_coords_smooth_slice.drop_front(1).drop_back(1),
        corner_min_radius_px,
        corner_max_radius_px,
        corner_max_samples,
        corner_angle_threshold,
        memory);

    /* Pre-blur the coordinates for the curve fitting. This generally leads to a better fit. */
    Array<float2> coords_pre_blur(smooth_window.size());
    const int pre_blur_iterations = 3;
    ed::greasepencil::gaussian_blur_1D(screen_space_coords_smooth_slice,
                                       pre_blur_iterations,
                                       1.0f,
                                       true,
                                       true,
                                       false,
                                       coords_pre_blur.as_mutable_span());

    /* Curve fitting. The output will be a set of handles (float2 triplets) in a flat array. */
    const float max_error_threshold_px = 5.0f;
    Array<float2> curve_points = ed::greasepencil::fit_curve_polyline_2d(
        coords_pre_blur, max_error_threshold_px * settings_->active_smooth, corner_mask);

    /* Sampling the curve at a fixed resolution. */
    const int64_t sample_resolution = 32;
    Array<float2> sampled_curve_points = sample_curve_2d(curve_points, sample_resolution);

    /* Morphing the coordinates onto the curve. Result is stored in a temporary array. */
    Array<float2> coords_smoothed(screen_space_coords_smooth_slice.size());
    morph_points_to_curve(screen_space_coords_smooth_slice, sampled_curve_points, coords_smoothed);

    MutableSpan<float2> smoothed_coordinates_slice =
        self.screen_space_smoothed_coordinates_.as_mutable_span().slice(smooth_window);
    MutableSpan<float3> positions_slice = drawing_->strokes_for_write()
                                              .positions_for_write()
                                              .slice(points_range)
                                              .slice(smooth_window);
    const float converging_threshold_px = 0.1f;
    bool stop_counting_converged = false;
    int num_converged = 0;
    for (const int64_t i : smooth_window.index_range()) {
      /* Record the curve fitting of this point. */
      self.screen_space_curve_fitted_coordinates_[i].append(coords_smoothed[i]);
      Span<float2> smoothed_coords_point = self.screen_space_curve_fitted_coordinates_[i];

      /* Get the sum of all the curve fittings of this point. */
      float2 sum = smoothed_coords_point[0];
      for (const float2 v : smoothed_coords_point.drop_front(1).drop_back(1)) {
        sum += v;
      }
      /* We compare the previous arithmetic mean to the current. Going from the back to the front,
       * if a point hasn't moved by a minimum threashold, it counts as converged. */
      float2 new_pos = (sum + smoothed_coords_point.last()) / smoothed_coords_point.size();
      if (!stop_counting_converged) {
        float2 prev_pos = sum / (smoothed_coords_point.size() - 1);
        if (math::distance(new_pos, prev_pos) < converging_threshold_px) {
          num_converged++;
        }
        else {
          stop_counting_converged = true;
        }
      }

      /* Update the positions in the current cache. */
      smoothed_coordinates_slice[i] = new_pos;
      positions_slice[i] = screen_space_to_object_space(new_pos);
    }

    /* Remove all the converged points from the active window and shrink the window accordingly. */
    if (num_converged > 0) {
      self.active_smooth_index_ = math::min(self.active_smooth_index_ + int64_t(num_converged),
                                            int64_t(drawing_->strokes().points_num() - 1));
      if (self.screen_space_curve_fitted_coordinates_.size() - num_converged > 0) {
        self.screen_space_curve_fitted_coordinates_.remove(0, num_converged);
      }
      else {
        self.screen_space_curve_fitted_coordinates_.clear();
      }
    }
  }

  void process_extension_sample(PaintOperation &self,
                                const InputSample &extension_sample,
                                const int curve_index)
  {
    ScreenSpacePoint point = this->point_from_input_sample(extension_sample);

    bke::CurvesGeometry &curves = drawing_->strokes_for_write();

    float2 prev_co = self.screen_space_coordinates_.last();
    float prev_radius = drawing_->radii().last();
    float prev_opacity = drawing_->opacities().last();
    ColorGeometry4f prev_vertex_color = drawing_->vertex_colors().last();

    /* Overwrite last point if it's very close. */
    if (math::distance(point.co, prev_co) < POINT_OVERRIDE_THRESHOLD_PX) {
      curves.positions_for_write().last() = screen_space_to_object_space(point.co);
      drawing_->radii_for_write().last() = math::max(point.radius, prev_radius);
      drawing_->opacities_for_write().last() = math::max(point.opacity, prev_opacity);
      return;
    }

    /* If the next sample is far away, we subdivide the segment to add more points. */
    int new_points_num = 1;
    const float distance_px = math::distance(point.co, prev_co);
    if (distance_px > POINT_RESAMPLE_MIN_DISTANCE_PX) {
      const int subdivisions = static_cast<int>(
                                   math::floor(distance_px / POINT_RESAMPLE_MIN_DISTANCE_PX)) -
                               1;
      new_points_num += subdivisions;
    }

    /* Subdivide stroke in new_range. */
    IndexRange new_range(curves.points_num(), new_points_num);
    Array<float2> new_coordinates(new_points_num);
    Array<float> new_radii(new_points_num);
    Array<float> new_opacities(new_points_num);
    Array<ColorGeometry4f> new_vertex_colors(new_points_num);
    const float step = 1.0f / static_cast<float>(new_points_num);
    float factor = step;
    for (const int64_t i : new_range.index_range()) {
      new_coordinates[i] = bke::attribute_math::mix2<float2>(factor, prev_co, point.co);
      new_radii[i] = bke::attribute_math::mix2<float>(factor, prev_radius, point.radius);
      new_opacities[i] = bke::attribute_math::mix2<float>(factor, prev_opacity, point.opacity);
      new_vertex_colors[i] = bke::attribute_math::mix2<ColorGeometry4f>(
          factor, prev_vertex_color, ColorGeometry4f(point.vertex_color));
      factor += step;
    }

    /* Update buffers with new points. */
    self.screen_space_coordinates_.extend(new_coordinates);
    self.screen_space_smoothed_coordinates_.extend(new_coordinates);
    for (float2 new_co : new_coordinates) {
      self.screen_space_curve_fitted_coordinates_.append(Vector<float2>({new_co}));
    }

    /* Resize the stroke cache. */
    curves.resize(curves.points_num() + new_points_num, curves.curves_num());
    curves.offsets_for_write().last() = curves.points_num();

    /* Write new data to the attributes. */
    drawing_->radii_for_write().slice(new_range).copy_from(new_radii);
    drawing_->opacities_for_write().slice(new_range).copy_from(new_opacities);
    drawing_->vertex_colors_for_write().slice(new_range).copy_from(new_vertex_colors);

    const int64_t min_active_smoothing_points_num = 8;
    const IndexRange points_range = curves.points_by_curve()[curve_index];
    if (points_range.size() < min_active_smoothing_points_num) {
      MutableSpan<float3> positions_slice = curves.positions_for_write().slice(new_range);
      for (const int64_t i : new_coordinates.index_range()) {
        positions_slice[i] = screen_space_to_object_space(new_coordinates[i]);
      }
      return;
    }

    const IndexRange smooth_window(self.active_smooth_index_,
                                   points_range.size() - self.active_smooth_index_);
    this->active_smoothing(self, points_range, smooth_window);
  }

  void execute(PaintOperation &self, const InputSample &extension_sample)
  {
    /* New curve was created in process_start_sample.*/
    const int curve_index = drawing_->strokes().curves_range().last();
    this->process_extension_sample(self, extension_sample, curve_index);
    drawing_->tag_topology_changed();
  }
};

void PaintOperation::on_stroke_begin(const bContext &C, const InputSample &start_sample)
{
  Scene *scene = CTX_data_scene(&C);
  Object *object = CTX_data_active_object(&C);
  GreasePencil *grease_pencil = static_cast<GreasePencil *>(object->data);

  Paint *paint = &scene->toolsettings->gp_paint->paint;
  Brush *brush = BKE_paint_brush(paint);

  BKE_curvemapping_init(brush->gpencil_settings->curve_sensitivity);
  BKE_curvemapping_init(brush->gpencil_settings->curve_strength);
  BKE_curvemapping_init(brush->gpencil_settings->curve_jitter);
  BKE_curvemapping_init(brush->gpencil_settings->curve_rand_pressure);
  BKE_curvemapping_init(brush->gpencil_settings->curve_rand_strength);
  BKE_curvemapping_init(brush->gpencil_settings->curve_rand_uv);
  BKE_curvemapping_init(brush->gpencil_settings->curve_rand_hue);
  BKE_curvemapping_init(brush->gpencil_settings->curve_rand_saturation);
  BKE_curvemapping_init(brush->gpencil_settings->curve_rand_value);

  Material *material = BKE_grease_pencil_object_material_ensure_from_active_input_brush(
      CTX_data_main(&C), object, brush);
  const int material_index = BKE_grease_pencil_object_material_index_get(object, material);

  PaintOperationExecutor executor{C};
  executor.process_start_sample(*this, start_sample, material_index);

  DEG_id_tag_update(&grease_pencil->id, ID_RECALC_GEOMETRY);
  WM_event_add_notifier(&C, NC_GEOM | ND_DATA, grease_pencil);
}

void PaintOperation::on_stroke_extended(const bContext &C, const InputSample &extension_sample)
{
  Object *object = CTX_data_active_object(&C);
  GreasePencil *grease_pencil = static_cast<GreasePencil *>(object->data);

  PaintOperationExecutor executor{C};
  executor.execute(*this, extension_sample);

  DEG_id_tag_update(&grease_pencil->id, ID_RECALC_GEOMETRY);
  WM_event_add_notifier(&C, NC_GEOM | ND_DATA, grease_pencil);
}

static float dist_to_interpolated_2d(
    float2 pos, float2 posA, float2 posB, float val, float valA, float valB)
{
  const float dist1 = math::distance_squared(posA, pos);
  const float dist2 = math::distance_squared(posB, pos);

  if (dist1 + dist2 > 1e-5f) {
    const float interpolated_val = interpf(valB, valA, dist1 / (dist1 + dist2));
    return math::distance(interpolated_val, val);
  }
  return 0.0f;
}

void PaintOperation::simplify_stroke(bke::greasepencil::Drawing &drawing, const float epsilon_px)
{
  const int stroke_index = drawing.strokes().curves_range().last();
  const IndexRange points_range = drawing.strokes().points_by_curve()[stroke_index];
  bke::CurvesGeometry &curves = drawing.strokes_for_write();
  const VArray<float> radii = drawing.radii();

  /* Distance function for `ramer_douglas_peucker_simplify`. */
  const Span<float2> positions_2d = this->screen_space_smoothed_coordinates_.as_span();
  const auto dist_function =
      [points_range, positions_2d, radii](int64_t first_index, int64_t last_index, int64_t index) {
        /* 2D coordinates are only stored for the current stroke, so offset the indices. */
        const float dist_position_px = dist_to_line_segment_v2(
            positions_2d[index - points_range.first()],
            positions_2d[first_index - points_range.first()],
            positions_2d[last_index - points_range.first()]);
        const float dist_radii_px = dist_to_interpolated_2d(
            positions_2d[index - points_range.first()],
            positions_2d[first_index - points_range.first()],
            positions_2d[last_index - points_range.first()],
            radii[index],
            radii[first_index],
            radii[last_index]);
        return math::max(dist_position_px, dist_radii_px);
      };

  Array<bool> points_to_delete(curves.points_num(), false);
  int64_t total_points_to_delete = ed::greasepencil::ramer_douglas_peucker_simplify(
      points_range, epsilon_px, dist_function, points_to_delete.as_mutable_span());

  if (total_points_to_delete > 0) {
    IndexMaskMemory memory;
    curves.remove_points(IndexMask::from_bools(points_to_delete, memory));
  }
}

void PaintOperation::process_stroke_end(bke::greasepencil::Drawing &drawing)
{
  const int stroke_index = drawing.strokes().curves_range().last();
  const IndexRange points_range = drawing.strokes().points_by_curve()[stroke_index];
  bke::CurvesGeometry &curves = drawing.strokes_for_write();
  /* Remove points at the end that have a radius close to 0. */
  int64_t points_to_remove = 0;
  for (int64_t index = points_range.last(); index >= points_range.first(); index--) {
    if (drawing.radii()[index] < 1e-5f) {
      points_to_remove++;
    }
    else {
      break;
    }
  }
  if (points_to_remove > 0) {
    curves.resize(curves.points_num() - points_to_remove, curves.curves_num());
    curves.offsets_for_write().last() = curves.points_num();
  }
}

void PaintOperation::on_stroke_done(const bContext &C)
{
  using namespace blender::bke;
  Scene *scene = CTX_data_scene(&C);
  Object *object = CTX_data_active_object(&C);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object->data);

  /* Grease Pencil should have an active layer. */
  BLI_assert(grease_pencil.has_active_layer());
  bke::greasepencil::Layer &active_layer = *grease_pencil.get_active_layer_for_write();
  const int drawing_index = active_layer.drawing_index_at(scene->r.cfra);

  /* Drawing should exist. */
  BLI_assert(drawing_index >= 0);
  bke::greasepencil::Drawing &drawing =
      reinterpret_cast<GreasePencilDrawing *>(grease_pencil.drawing(drawing_index))->wrap();

  const float simplifiy_threashold_px = 0.5f;
  this->simplify_stroke(drawing, simplifiy_threashold_px);
  this->process_stroke_end(drawing);
  drawing.tag_topology_changed();

  DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY);
  WM_main_add_notifier(NC_GEOM | ND_DATA, &grease_pencil.id);
}

std::unique_ptr<GreasePencilStrokeOperation> new_paint_operation()
{
  return std::make_unique<PaintOperation>();
}

}  // namespace blender::ed::sculpt_paint::greasepencil
