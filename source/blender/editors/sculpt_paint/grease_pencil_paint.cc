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

#include "BLI_math_base.hh"

#include "DEG_depsgraph_query.h"

#include "ED_grease_pencil.hh"
#include "ED_view3d.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "grease_pencil_intern.hh"

namespace blender::ed::sculpt_paint::greasepencil {

static constexpr int64_t STOKE_CACHE_ALLOCATION_CHUNK_SIZE = 1024;

struct ScreenSpacePoint {
  float2 position;
  float radius;
  float opacity;
  float4 vertex_color;
};

class PaintOperation : public GreasePencilStrokeOperation {
 private:
  Vector<float2> screen_space_coordinates_;

  bke::greasepencil::StrokeCache *stroke_cache_;
  int64_t active_index_ = 0;

  Vector<float3> sampled_positions_;
  Vector<float> sampled_radii_;

  Vector<float3> smoothed_positions_;
  Vector<float> smoothed_radii_;

  friend struct PaintOperationExecutor;

 public:
  void on_stroke_begin(const bContext &C, const InputSample &start_sample) override;
  void on_stroke_extended(const bContext &C, const InputSample &extension_sample) override;
  void on_stroke_done(const bContext &C) override;

 private:
  void simplify_stroke_cache(const float epsilon);
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
  bool use_vertex_color_stroke_;

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
    use_vertex_color_stroke_ = use_vertex_color && ELEM(settings_->vertex_mode,
                                                        GPPAINT_MODE_STROKE,
                                                        GPPAINT_MODE_BOTH);
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

    point.position = sample.mouse_position;

    point.radius = brush_size_ / 2.0f;
    if (BKE_brush_use_size_pressure(brush_)) {
      point.radius *= BKE_curvemapping_evaluateF(settings_->curve_sensitivity, 0, sample.pressure);
    }
    point.opacity = brush_alpha_;
    if (BKE_brush_use_alpha_pressure(brush_)) {
      point.opacity *= BKE_curvemapping_evaluateF(settings_->curve_strength, 0, sample.pressure);
    }
    point.vertex_color = use_vertex_color_stroke_ ? float4(brush_->rgb[0],
                                                           brush_->rgb[1],
                                                           brush_->rgb[2],
                                                           settings_->vertex_factor) :
                                                    float4(0.0f);
    /* TODO: Get fill vertex color. */
    return point;
  }

  void update_stroke_cache(PaintOperation &self,
                           const int64_t new_points_num,
                           Span<float2> new_coordinates,
                           Span<float> new_radii,
                           Span<float> new_opacities,
                           Span<ColorGeometry4f> new_vertex_colors)
  {
    const int64_t old_size = self.stroke_cache_->size();
    self.stroke_cache_->resize(self.stroke_cache_->size() + new_points_num);

    IndexRange new_range(old_size, new_points_num);
    MutableSpan<float3> positions_slice = self.stroke_cache_->positions_for_write().slice(
        new_range);
    for (const int64_t i : new_coordinates.index_range()) {
      positions_slice[i] = screen_space_to_object_space(new_coordinates[i]);
    }
    self.stroke_cache_->radii_for_write().slice(new_range).copy_from(new_radii);
    self.stroke_cache_->opacities_for_write().slice(new_range).copy_from(new_opacities);
    self.stroke_cache_->vertex_colors_for_write().slice(new_range).copy_from(new_vertex_colors);
  }

  void process_start_sample(PaintOperation &self, const InputSample &start_sample)
  {
    ScreenSpacePoint start_point = this->point_from_input_sample(start_sample);

    self.screen_space_coordinates_.append(start_point.position);

    self.stroke_cache_->resize(1);
    self.stroke_cache_->positions_for_write().last() = screen_space_to_object_space(
        start_point.position);
    self.stroke_cache_->radii_for_write().last() = start_point.radius;
    self.stroke_cache_->opacities_for_write().last() = start_point.opacity;
    self.stroke_cache_->vertex_colors_for_write().last() = ColorGeometry4f(
        start_point.vertex_color);
  }

  void process_extension_sample(PaintOperation &self, const InputSample &extension_sample)
  {
    ScreenSpacePoint point = this->point_from_input_sample(extension_sample);

    float2 prev_co = self.screen_space_coordinates_.last();
    float prev_radius = self.stroke_cache_->radii().last();
    float prev_opacity = self.stroke_cache_->opacities().last();
    ColorGeometry4f prev_vertex_color = self.stroke_cache_->vertex_colors().last();

    int new_points_num = 1;
    const float min_distance_px = 10.0f;
    if (self.screen_space_coordinates_.size() > 1) {
      const float distance_px = math::distance(point.position, prev_co);
      if (distance_px > min_distance_px) {
        new_points_num += static_cast<int>(math::floor(distance_px / min_distance_px)) - 1;
      }
    }

    Array<float2> new_coordinates(new_points_num);
    Array<float> new_radii(new_points_num);
    Array<float> new_opacities(new_points_num);
    Array<ColorGeometry4f> new_vertex_colors(new_points_num);
    const float step = 1.0f / static_cast<float>(new_points_num);
    float factor = step;
    for (const int i : IndexRange(new_points_num)) {
      new_coordinates[i] = bke::attribute_math::mix2<float2>(factor, prev_co, point.position);
      new_radii[i] = bke::attribute_math::mix2<float>(factor, prev_radius, point.radius);
      new_opacities[i] = bke::attribute_math::mix2<float>(factor, prev_opacity, point.opacity);
      new_vertex_colors[i] = bke::attribute_math::mix2<ColorGeometry4f>(
          factor, prev_vertex_color, ColorGeometry4f(point.vertex_color));
      factor += step;
    }

    for (float2 co : new_coordinates) {
      self.screen_space_coordinates_.append(co);
    }

    this->update_stroke_cache(
        self, new_points_num, new_coordinates, new_radii, new_opacities, new_vertex_colors);

    // const float active_smooth_factor = settings_->active_smooth;

    // self.sampled_positions_.append(proj_pos);
    // self.sampled_radii_.append(radius);

    // self.smoothed_positions_.append(proj_pos);
    // self.smoothed_radii_.append(radius);

    // const int64_t smooth_window_size = 16;
    // const int64_t inverted_copy_window_size = math::max(
    //     self.stroke_cache_->size() - smooth_window_size, int64_t(0));

    // const int64_t smooth_radius = 16;

    // const int64_t inverted_smooth_window_size = math::max(
    //     self.stroke_cache_->size() - smooth_window_size - smooth_radius, int64_t(0));

    // Span<float3> src_positions = self.sampled_positions_.as_span().drop_front(
    //     inverted_smooth_window_size);
    // MutableSpan<float3> dst_positions = self.smoothed_positions_.as_mutable_span().drop_front(
    //     inverted_smooth_window_size);

    // Span<float> src_radii =
    // self.sampled_radii_.as_span().drop_front(inverted_smooth_window_size); MutableSpan<float>
    // dst_radii = self.smoothed_radii_.as_mutable_span().drop_front(
    //     inverted_smooth_window_size);

    // ed::greasepencil::gaussian_blur_1D(
    //     src_positions, smooth_radius, active_smooth_factor, false, true, false, dst_positions);
    // ed::greasepencil::gaussian_blur_1D(
    //     src_radii, smooth_radius, active_smooth_factor, false, false, false, dst_radii);

    // self.stroke_cache_->positions_for_write()
    //     .drop_front(inverted_copy_window_size)
    //     .copy_from(self.smoothed_positions_.as_span().drop_front(inverted_copy_window_size));
    // self.stroke_cache_->radii_for_write()
    //     .drop_front(inverted_copy_window_size)
    //     .copy_from(self.smoothed_radii_.as_span().drop_front(inverted_copy_window_size));

#ifdef DEBUG
    // /* Visualize active window. */
    // self.stroke_cache_->vertex_colors.fill(ColorGeometry4f(float4(0.0f)));
    // self.stroke_cache_->vertex_colors.as_mutable_span()
    //     .drop_front(inverted_active_window_size)
    //     .fill(ColorGeometry4f(float4(1.0f, 0.1f, 0.1f, 1.0f)));
#endif
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

static float dist_to_interpolated(
    float3 pos, float3 posA, float3 posB, float val, float valA, float valB)
{
  float dist1 = math::distance_squared(posA, pos);
  float dist2 = math::distance_squared(posB, pos);

  if (dist1 + dist2 > 0) {
    float interpolated_val = interpf(valB, valA, dist1 / (dist1 + dist2));
    return math::distance(interpolated_val, val);
  }
  return 0.0f;
}

void PaintOperation::simplify_stroke_cache(const float epsilon)
{
  const Span<float3> positions = stroke_cache_->positions();
  const Span<float> radii = stroke_cache_->radii();
  const Span<float> opacities = stroke_cache_->opacities();
  const Span<ColorGeometry4f> vertex_colors = stroke_cache_->vertex_colors();

  /* Distance function for `ramer_douglas_peucker_simplify`. */
  const auto dist_function = [positions,
                              radii](int64_t first_index, int64_t last_index, int64_t index) {
    const float dist_position = dist_to_line_v3(
        positions[index], positions[first_index], positions[last_index]);
    /* We devide the distance by 2000.0f to convert from "pixels" to an actual distance.
     * For some reason, grease pencil storkes the thickness of strokes in pixels rather
     * than object space distance. */
    const float dist_radii = dist_to_interpolated(positions[index],
                                                  positions[first_index],
                                                  positions[last_index],
                                                  radii[index],
                                                  radii[first_index],
                                                  radii[last_index]) /
                             2000.0f;
    return math::max(dist_position, dist_radii);
  };

  Array<bool> points_to_delete(stroke_cache_->size(), false);
  int64_t total_points_to_remove = ed::greasepencil::ramer_douglas_peucker_simplify(
      IndexRange(stroke_cache_->size()),
      epsilon,
      dist_function,
      points_to_delete.as_mutable_span());

  int64_t new_size = stroke_cache_->size() - total_points_to_remove;

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

  // this->simplify_stroke_cache(0.0005f);

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
