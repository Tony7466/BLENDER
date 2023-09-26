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

/** Interpolate \a dst such that the points in \a dst lie evenly distributed on \a src. */
static void interp_polyline_to_polyline(Span<float2> src, MutableSpan<float2> dst)
{
  using ParamIt = double *;
  BLI_assert(src.size() > 1);
  Array<double> normalized_parameters_src(src.size(), 0.0f);
  double total_dist_src = 0.0f;
  for (const int64_t i : src.index_range().drop_front(1)) {
    total_dist_src += math::distance(src[i], src[i - 1]);
    normalized_parameters_src[i] = total_dist_src;
  }
  if (total_dist_src < 1e-6f) {
    normalized_parameters_src.fill(0.0f);
  }
  else {
    for (const int64_t i : src.index_range()) {
      normalized_parameters_src[i] /= total_dist_src;
    }
  }

  Array<double> accumulated_lengths_dst(dst.size(), 0.0f);
  double total_dist_dst = 0.0f;
  for (const int64_t i : dst.index_range().drop_front(1)) {
    total_dist_dst += math::distance(dst[i], dst[i - 1]);
    accumulated_lengths_dst[i] = total_dist_dst;
  }

  ParamIt start_it = normalized_parameters_src.begin();
  for (const int64_t i : dst.index_range().drop_front(1)) {
    const double target = accumulated_lengths_dst[i] / total_dist_dst;
    ParamIt it = std::lower_bound(start_it, normalized_parameters_src.end(), target);
    const int64_t index = std::distance(normalized_parameters_src.begin(), it);
    start_it = it;
    if (index + 1 >= normalized_parameters_src.size()) {
      break;
    }
    if (math::distance_squared(src[index], dst[i]) < 1e-6 ||
        math::abs(normalized_parameters_src[index + 1] - normalized_parameters_src[index]) < 1e-8)
    {
      dst[i] = src[index];
      continue;
    }
    const double t = (target - normalized_parameters_src[index]) /
                     (normalized_parameters_src[index + 1] - normalized_parameters_src[index]);

    dst[i] = math::interpolate(src[index], src[index + 1], t);
  }
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

  bke::greasepencil::StrokeCache *stroke_cache_;

  friend struct PaintOperationExecutor;

 public:
  void on_stroke_begin(const bContext &C, const InputSample &start_sample) override;
  void on_stroke_extended(const bContext &C, const InputSample &extension_sample) override;
  void on_stroke_done(const bContext &C) override;

 private:
  void simplify_stroke_cache(const float epsilon_px);
  void process_stroke_end();
  void create_stroke_from_stroke_cache(bke::greasepencil::Drawing &drawing_orig);
};

/**
 * Utility class that actually executes the update when the stroke is updated. That's useful
 * because it avoids passing a very large number of parameters between functions.
 */
struct PaintOperationExecutor {
  ARegion *region_;

  Brush *brush_;
  int brush_size_;
  float brush_alpha_;

  BrushGpencilSettings *settings_;
  float4 vertex_color_;

  PaintOperationExecutor(const bContext &C)
  {
    Scene *scene = CTX_data_scene(&C);
    region_ = CTX_wm_region(&C);

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

  void process_start_sample(PaintOperation &self, const InputSample &start_sample)
  {
    ScreenSpacePoint start_point = this->point_from_input_sample(start_sample);

    self.screen_space_coordinates_.append(start_point.co);
    self.screen_space_curve_fitted_coordinates_.append(Vector<float2>({start_point.co}));
    self.screen_space_smoothed_coordinates_.append(start_point.co);

    self.stroke_cache_->resize(1);
    self.stroke_cache_->positions_for_write().last() = screen_space_to_object_space(
        start_point.co);
    self.stroke_cache_->radii_for_write().last() = start_point.radius;
    self.stroke_cache_->opacities_for_write().last() = start_point.opacity;
    self.stroke_cache_->vertex_colors_for_write().last() = ColorGeometry4f(
        start_point.vertex_color);
  }

  void active_smoothing(PaintOperation &self)
  {
    /* Active smoothing is only done on a slice of the points (from the active index to the current
     * end of the stroke). */
    const int64_t smooth_window_size = self.stroke_cache_->size() - self.active_smooth_index_;
    IndexRange smooth_window(self.active_smooth_index_, smooth_window_size);

    Span<float2> screen_space_coords_smooth_slice = self.screen_space_coordinates_.as_span().slice(
        smooth_window);

    /* Detect corners in the current slice of coordinates. */
    IndexMaskMemory memory;
    const float min_radius_px = 5.0f;
    const float max_radius_px = 30.0f;
    const int64_t max_samples = 64;
    const float angle_threshold = 0.6f;
    IndexMask corner_mask = ed::greasepencil::polyline_detect_corners(
        screen_space_coords_smooth_slice,
        min_radius_px,
        max_radius_px,
        max_samples,
        angle_threshold,
        memory);

    /* Pre-blur the coordinates for the curve fitting. This generally leads to a better fit. */
    Array<float2> coords_pre_blur(smooth_window.size());
    ed::greasepencil::gaussian_blur_1D(screen_space_coords_smooth_slice,
                                       3,
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
    Array<float2> coords_smoothed(screen_space_coords_smooth_slice);
    interp_polyline_to_polyline(sampled_curve_points, coords_smoothed.as_mutable_span());

    MutableSpan<float2> smoothed_coordinates_slice =
        self.screen_space_smoothed_coordinates_.as_mutable_span().slice(smooth_window);
    MutableSpan<float3> positions_slice = self.stroke_cache_->positions_for_write().slice(
        smooth_window);
    const float converging_threshold_px = 0.05f;
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
      self.active_smooth_index_ = math::min(self.active_smooth_index_ + num_converged,
                                            self.stroke_cache_->size() - 1);
      if (self.screen_space_curve_fitted_coordinates_.size() - num_converged > 0) {
        self.screen_space_curve_fitted_coordinates_.remove(0, num_converged);
      }
      else {
        self.screen_space_curve_fitted_coordinates_.clear();
      }
    }

#ifdef DEBUG
    /* Visualize active window. */
    self.stroke_cache_->vertex_colors_for_write().fill(ColorGeometry4f(float4(0.0f)));
    for (const int64_t i : smooth_window.index_range()) {
      self.stroke_cache_->vertex_colors_for_write().slice(smooth_window)[i] = ColorGeometry4f(
          float4(1.0f, 0.1f, 0.1f, 1.0f));
    }
#endif
  }

  void process_extension_sample(PaintOperation &self, const InputSample &extension_sample)
  {
    ScreenSpacePoint point = this->point_from_input_sample(extension_sample);

    float2 prev_co = self.screen_space_coordinates_.last();
    float prev_radius = self.stroke_cache_->radii().last();
    float prev_opacity = self.stroke_cache_->opacities().last();
    ColorGeometry4f prev_vertex_color = self.stroke_cache_->vertex_colors().last();

    /* Overwrite last point if it's very close. */
    if (math::distance(point.co, prev_co) < POINT_OVERRIDE_THRESHOLD_PX) {
      self.stroke_cache_->positions_for_write().last() = screen_space_to_object_space(point.co);
      self.stroke_cache_->radii_for_write().last() = math::max(point.radius, prev_radius);
      self.stroke_cache_->opacities_for_write().last() = math::max(point.opacity, prev_opacity);
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
    IndexRange new_range(self.stroke_cache_->size(), new_points_num);
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
    self.stroke_cache_->resize(self.stroke_cache_->size() + new_points_num);

    /* Write new data to the attributes. */
    self.stroke_cache_->radii_for_write().slice(new_range).copy_from(new_radii);
    self.stroke_cache_->opacities_for_write().slice(new_range).copy_from(new_opacities);
    self.stroke_cache_->vertex_colors_for_write().slice(new_range).copy_from(new_vertex_colors);

    const int64_t min_active_smoothing_points_num = 8;
    if (self.stroke_cache_->size() < min_active_smoothing_points_num) {
      MutableSpan<float3> positions_slice = self.stroke_cache_->positions_for_write().slice(
          new_range);
      for (const int64_t i : new_coordinates.index_range()) {
        positions_slice[i] = screen_space_to_object_space(new_coordinates[i]);
      }
      return;
    }

    this->active_smoothing(self);
  }

  void execute(PaintOperation &self, const bContext &C, const InputSample &extension_sample)
  {
    Depsgraph *depsgraph = CTX_data_depsgraph_pointer(&C);
    Object *object = CTX_data_active_object(&C);
    Object *object_eval = DEG_get_evaluated_object(depsgraph, object);
    GreasePencil *grease_pencil = static_cast<GreasePencil *>(object_eval->data);

    this->process_extension_sample(self, extension_sample);

    BKE_grease_pencil_batch_cache_dirty_tag(grease_pencil, BKE_GREASEPENCIL_BATCH_DIRTY_ALL);
  }
};

void PaintOperation::on_stroke_begin(const bContext &C, const InputSample &start_sample)
{
  Depsgraph *depsgraph = CTX_data_depsgraph_pointer(&C);
  Scene *scene = CTX_data_scene(&C);
  Object *object = CTX_data_active_object(&C);
  Object *object_eval = DEG_get_evaluated_object(depsgraph, object);
  /**
   * Note: We write to the evaluated object here, so that the additional copy from orig -> eval
   * is not needed for every update. After the stroke is done, the result is written to the
   * original object.
   */
  GreasePencil *grease_pencil = static_cast<GreasePencil *>(object_eval->data);
  stroke_cache_ = &grease_pencil->runtime->stroke_cache;

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
  stroke_cache_->set_material_index(BKE_grease_pencil_object_material_index_get(object, material));

  stroke_cache_->reserve(STOKE_CACHE_ALLOCATION_CHUNK_SIZE);

  PaintOperationExecutor executor{C};
  executor.process_start_sample(*this, start_sample);
}

void PaintOperation::on_stroke_extended(const bContext &C, const InputSample &extension_sample)
{
  PaintOperationExecutor executor{C};
  executor.execute(*this, C, extension_sample);
}

static float dist_to_interpolated_2d(
    float2 pos, float2 posA, float2 posB, float val, float valA, float valB)
{
  const float dist1 = math::distance_squared(posA, pos);
  const float dist2 = math::distance_squared(posB, pos);

  if (dist1 + dist2 > 0.0f) {
    const float interpolated_val = interpf(valB, valA, dist1 / (dist1 + dist2));
    return math::distance(interpolated_val, val);
  }
  return 0.0f;
}

void PaintOperation::simplify_stroke_cache(const float epsilon_px)
{
  const Span<float3> positions = stroke_cache_->positions();
  const Span<float> radii = stroke_cache_->radii();
  const Span<float> opacities = stroke_cache_->opacities();
  const Span<ColorGeometry4f> vertex_colors = stroke_cache_->vertex_colors();

  /* Distance function for `ramer_douglas_peucker_simplify`. */
  const Span<float2> positions_2d = this->screen_space_smoothed_coordinates_.as_span();
  const auto dist_function = [positions_2d,
                              radii](int64_t first_index, int64_t last_index, int64_t index) {
    const float dist_position_px = dist_to_line_segment_v2(
        positions_2d[index], positions_2d[first_index], positions_2d[last_index]);
    const float dist_radii_px = dist_to_interpolated_2d(positions_2d[index],
                                                        positions_2d[first_index],
                                                        positions_2d[last_index],
                                                        radii[index],
                                                        radii[first_index],
                                                        radii[last_index]);
    return math::max(dist_position_px, dist_radii_px);
  };

  Array<bool> points_to_delete(stroke_cache_->size(), false);
  int64_t total_points_to_remove = ed::greasepencil::ramer_douglas_peucker_simplify(
      IndexRange(stroke_cache_->size()),
      epsilon_px,
      dist_function,
      points_to_delete.as_mutable_span());

  int64_t new_size = stroke_cache_->size() - total_points_to_remove;
  BLI_assert(new_size > 0);

  Array<int64_t> old_indices(new_size);
  int64_t i = 0;
  for (const int64_t old_index : points_to_delete.index_range()) {
    if (points_to_delete[old_index] == false) {
      old_indices[i] = old_index;
      i++;
    }
  }

  Array<float3> new_positions(new_size);
  Array<float> new_radii(new_size);
  Array<float> new_opacities(new_size);
  Array<ColorGeometry4f> new_vertex_colors(new_size);
  for (const int64_t index : IndexRange(new_size)) {
    new_positions[index] = positions[old_indices[index]];
    new_radii[index] = radii[old_indices[index]];
    new_opacities[index] = opacities[old_indices[index]];
    new_vertex_colors[index] = vertex_colors[old_indices[index]];
  }
  stroke_cache_->resize(new_size);

  stroke_cache_->positions_for_write().copy_from(new_positions);
  stroke_cache_->radii_for_write().copy_from(new_radii);
  stroke_cache_->opacities_for_write().copy_from(new_opacities);
  stroke_cache_->vertex_colors_for_write().copy_from(new_vertex_colors);

  // stroke_cache_->triangles.clear_and_shrink();
}

void PaintOperation::process_stroke_end()
{
  /* Remove points at the end that have a radius close to 0. */
  int64_t points_to_remove = 0;
  for (int64_t index = this->stroke_cache_->size() - 1; index >= 0; index--) {
    if (this->stroke_cache_->radii()[index] < 1e-5f) {
      points_to_remove++;
    }
    else {
      break;
    }
  }
  stroke_cache_->resize(math::max(stroke_cache_->size() - points_to_remove, int64_t(0)));
}

void PaintOperation::create_stroke_from_stroke_cache(bke::greasepencil::Drawing &drawing_orig)
{
  using namespace blender::bke;
  /* Create a new stroke from the stroke buffer. */
  CurvesGeometry &curves = drawing_orig.strokes_for_write();

  const int num_old_curves = curves.curves_num();
  const int num_old_points = curves.points_num();
  curves.resize(num_old_points + stroke_cache_->size(), num_old_curves + 1);

  curves.offsets_for_write()[num_old_curves] = num_old_points;
  curves.offsets_for_write()[num_old_curves + 1] = num_old_points + stroke_cache_->size();

  const OffsetIndices<int> points_by_curve = curves.points_by_curve();
  const IndexRange new_points_range = points_by_curve[curves.curves_num() - 1];
  const IndexRange new_curves_range = IndexRange(num_old_curves, 1);

  /* Set position, radius and opacity attribute. */
  bke::MutableAttributeAccessor attributes = curves.attributes_for_write();
  MutableSpan<float3> positions = curves.positions_for_write();
  MutableSpan<float> radii = drawing_orig.radii_for_write();
  MutableSpan<float> opacities = drawing_orig.opacities_for_write();
  SpanAttributeWriter<ColorGeometry4f> vertex_colors =
      attributes.lookup_or_add_for_write_span<ColorGeometry4f>(
          "vertex_color",
          ATTR_DOMAIN_POINT,
          AttributeInitVArray(
              VArray<ColorGeometry4f>::ForSingle(ColorGeometry4f(0.0f, 0.0f, 0.0f, 0.0f),
                                                 attributes.domain_size(ATTR_DOMAIN_POINT))));
  positions.slice(new_points_range).copy_from(stroke_cache_->positions());
  radii.slice(new_points_range).copy_from(stroke_cache_->radii());
  opacities.slice(new_points_range).copy_from(stroke_cache_->opacities());
  vertex_colors.span.slice(new_points_range).copy_from(stroke_cache_->vertex_colors());

  SpanAttributeWriter<int> materials = attributes.lookup_or_add_for_write_span<int>(
      "material_index", ATTR_DOMAIN_CURVE);
  materials.span.slice(new_curves_range).fill(stroke_cache_->material_index());

  /* Set curve_type attribute. */
  curves.fill_curve_types(new_curves_range, CURVE_TYPE_POLY);

  /* Explicitly set all other attributes besides those processed above to default values. */
  Set<std::string> attributes_to_skip{
      {"position", "radius", "opacity", "vertex_color", "material_index", "curve_type"}};
  attributes.for_all(
      [&](const bke::AttributeIDRef &id, const bke::AttributeMetaData /*meta_data*/) {
        if (attributes_to_skip.contains(id.name())) {
          return true;
        }
        bke::GSpanAttributeWriter attribute = attributes.lookup_for_write_span(id);
        const CPPType &type = attribute.span.type();
        GMutableSpan new_data = attribute.span.slice(
            attribute.domain == ATTR_DOMAIN_POINT ? new_points_range : new_curves_range);
        type.fill_assign_n(type.default_value(), new_data.data(), new_data.size());
        attribute.finish();
        return true;
      });

  vertex_colors.finish();
  materials.finish();

  drawing_orig.tag_positions_changed();
  stroke_cache_->clear();
}

void PaintOperation::on_stroke_done(const bContext &C)
{
  using namespace blender::bke;
  Scene *scene = CTX_data_scene(&C);
  Object *object = CTX_data_active_object(&C);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object->data);

  /* No stroke to create, return. */
  if (stroke_cache_->size() == 0) {
    return;
  }

  const float simplifiy_threashold_px = 0.5f;
  this->simplify_stroke_cache(simplifiy_threashold_px);

  this->process_stroke_end();

  /* The object should have an active layer. */
  BLI_assert(grease_pencil.has_active_layer());
  const bke::greasepencil::Layer &active_layer_orig = *grease_pencil.get_active_layer();
  const int index_orig = active_layer_orig.drawing_index_at(scene->r.cfra);

  bke::greasepencil::Drawing &drawing_orig =
      reinterpret_cast<GreasePencilDrawing *>(grease_pencil.drawings()[index_orig])->wrap();
  this->create_stroke_from_stroke_cache(drawing_orig);

  DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY);
  WM_main_add_notifier(NC_GEOM | ND_DATA, &grease_pencil.id);
}

std::unique_ptr<GreasePencilStrokeOperation> new_paint_operation()
{
  return std::make_unique<PaintOperation>();
}

}  // namespace blender::ed::sculpt_paint::greasepencil
