/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_brush.h"
#include "BKE_colortools.h"
#include "BKE_context.h"
#include "BKE_curves.hh"
#include "BKE_grease_pencil.h"
#include "BKE_grease_pencil.hh"
#include "BKE_scene.h"

#include "BLI_math_base.hh"

#include "DEG_depsgraph_query.h"

#include "ED_grease_pencil.hh"
#include "ED_view3d.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "grease_pencil_intern.hh"

namespace blender::ed::sculpt_paint::greasepencil {

class PaintOperation : public GreasePencilStrokeOperation {
 private:
  Vector<float3> sampled_positions_;
  Vector<float> sampled_radii_;

  Vector<float3> smoothed_positions_;
  Vector<float> smoothed_radii_;
  bke::greasepencil::StrokeCache *stroke_cache_;

  friend struct PaintOperationExecutor;

 public:
  void on_stroke_begin(const bContext &C, const InputSample &start_sample) override;
  void on_stroke_extended(const bContext &C, const InputSample &extension_sample) override;
  void on_stroke_done(const bContext &C) override;
};

/**
 * Utility class that actually executes the update when the stroke is updated. That's useful
 * because it avoids passing a very large number of parameters between functions.
 */
struct PaintOperationExecutor {
  PaintOperationExecutor(const bContext & /*C*/) {}

  void execute(PaintOperation &self, const bContext &C, const InputSample &extension_sample)
  {
    Depsgraph *depsgraph = CTX_data_depsgraph_pointer(&C);
    Scene *scene = CTX_data_scene(&C);
    Object *object = CTX_data_active_object(&C);
    Object *object_eval = DEG_get_evaluated_object(depsgraph, object);
    ARegion *region = CTX_wm_region(&C);

    Paint *paint = &scene->toolsettings->gp_paint->paint;
    Brush *brush = BKE_paint_brush(paint);
    int brush_size = BKE_brush_size_get(scene, brush);
    float brush_alpha = BKE_brush_alpha_get(scene, brush);

    const bool use_vertex_color = (scene->toolsettings->gp_paint->mode ==
                                   GPPAINT_FLAG_USE_VERTEXCOLOR);
    const bool use_vertex_color_stroke = use_vertex_color &&
                                         ELEM(brush->gpencil_settings->vertex_mode,
                                              GPPAINT_MODE_STROKE,
                                              GPPAINT_MODE_BOTH);
    // const bool use_vertex_color_fill = use_vertex_color && ELEM(
    //     brush->gpencil_settings->vertex_mode, GPPAINT_MODE_STROKE, GPPAINT_MODE_BOTH);

    /**
     * Note: We write to the evaluated object here, so that the additional copy from orig ->
     * eval is not needed for every update. After the stroke is done, the result is written to
     * the original object.
     */
    GreasePencil *grease_pencil = static_cast<GreasePencil *>(object_eval->data);

    float4 plane{0.0f, -1.0f, 0.0f, 0.0f};
    float3 proj_pos;
    ED_view3d_win_to_3d_on_plane(region, plane, extension_sample.mouse_position, false, proj_pos);

    float radius = brush_size / 2.0f;
    if (BKE_brush_use_size_pressure(brush)) {
      radius *= BKE_curvemapping_evaluateF(
          brush->gpencil_settings->curve_sensitivity, 0, extension_sample.pressure);
    }
    float opacity = brush_alpha;
    if (BKE_brush_use_alpha_pressure(brush)) {
      opacity *= BKE_curvemapping_evaluateF(
          brush->gpencil_settings->curve_strength, 0, extension_sample.pressure);
    }
    float4 vertex_color = use_vertex_color_stroke ?
                              float4(brush->rgb[0],
                                     brush->rgb[1],
                                     brush->rgb[2],
                                     brush->gpencil_settings->vertex_factor) :
                              float4(0.0f);
    const float active_smooth_factor = brush->gpencil_settings->active_smooth;

    self.stroke_cache_->append(proj_pos, radius, opacity, ColorGeometry4f(vertex_color));

    self.sampled_positions_.append(proj_pos);
    self.sampled_radii_.append(radius);

    self.smoothed_positions_.append(proj_pos);
    self.smoothed_radii_.append(radius);

    const int64_t smooth_window_size = 16;
    const int64_t inverted_copy_window_size = math::max(
        self.stroke_cache_->size - smooth_window_size, int64_t(0));

    const int64_t smooth_radius = 16;

    const int64_t inverted_smooth_window_size = math::max(
        self.stroke_cache_->size - smooth_window_size - smooth_radius, int64_t(0));

    Span<float3> src_positions = self.sampled_positions_.as_span().drop_front(
        inverted_smooth_window_size);
    MutableSpan<float3> dst_positions = self.smoothed_positions_.as_mutable_span().drop_front(
        inverted_smooth_window_size);

    Span<float> src_radii = self.sampled_radii_.as_span().drop_front(inverted_smooth_window_size);
    MutableSpan<float> dst_radii = self.smoothed_radii_.as_mutable_span().drop_front(
        inverted_smooth_window_size);

    ed::greasepencil::gaussian_blur_1D(
        src_positions, smooth_radius, active_smooth_factor, false, true, false, dst_positions);
    ed::greasepencil::gaussian_blur_1D(
        src_radii, smooth_radius, active_smooth_factor, false, false, false, dst_radii);

    self.stroke_cache_->positions.as_mutable_span()
        .drop_front(inverted_copy_window_size)
        .copy_from(self.smoothed_positions_.as_span().drop_front(inverted_copy_window_size));
    self.stroke_cache_->radii.as_mutable_span()
        .drop_front(inverted_copy_window_size)
        .copy_from(self.smoothed_radii_.as_span().drop_front(inverted_copy_window_size));

#ifdef DEBUG
    /* Visualize active window. */
    self.stroke_cache_->vertex_colors.fill(ColorGeometry4f(float4(0.0f)));
    self.stroke_cache_->vertex_colors.as_mutable_span()
        .drop_front(inverted_active_window_size)
        .fill(ColorGeometry4f(float4(1.0f, 0.1f, 0.1f, 1.0f)));
#endif

    BKE_grease_pencil_batch_cache_dirty_tag(grease_pencil, BKE_GREASEPENCIL_BATCH_DIRTY_ALL);
  }
};

void PaintOperation::on_stroke_begin(const bContext &C, const InputSample & /*start_sample*/)
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
}

void PaintOperation::on_stroke_extended(const bContext &C, const InputSample &extension_sample)
{
  PaintOperationExecutor executor{C};
  executor.execute(*this, C, extension_sample);
}

void PaintOperation::on_stroke_done(const bContext &C)
{
  using namespace blender::bke;
  Depsgraph *depsgraph = CTX_data_depsgraph_pointer(&C);
  Scene *scene = CTX_data_scene(&C);
  Object *object = CTX_data_active_object(&C);
  Object *object_eval = DEG_get_evaluated_object(depsgraph, object);

  GreasePencil &grease_pencil_orig = *static_cast<GreasePencil *>(object->data);
  GreasePencil &grease_pencil_eval = *static_cast<GreasePencil *>(object_eval->data);

  /* No stroke to create, return. */
  if (stroke_cache_->size == 0) {
    return;
  }

  /* The object should have an active layer. */
  BLI_assert(grease_pencil_orig.has_active_layer());

  /* Create the new stroke from the stroke buffer. */
  const bke::greasepencil::Layer &active_layer_orig = *grease_pencil_orig.get_active_layer();
  const int index_orig = active_layer_orig.drawing_index_at(scene->r.cfra);

  bke::greasepencil::Drawing &drawing_orig =
      reinterpret_cast<GreasePencilDrawing *>(grease_pencil_orig.drawings()[index_orig])->wrap();
  CurvesGeometry &curves = drawing_orig.strokes_for_write();

  const int num_old_curves = curves.curves_num();
  const int num_old_points = curves.points_num();
  curves.resize(num_old_points + stroke_cache_->size, num_old_curves + 1);

  curves.offsets_for_write()[num_old_curves] = num_old_points;
  curves.offsets_for_write()[num_old_curves + 1] = num_old_points + stroke_cache_->size;

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
  positions.slice(new_points_range).copy_from(stroke_cache_->positions);
  radii.slice(new_points_range).copy_from(stroke_cache_->radii);
  opacities.slice(new_points_range).copy_from(stroke_cache_->opacities);
  vertex_colors.span.slice(new_points_range).copy_from(stroke_cache_->vertex_colors);

  /* TODO: Set material index attribute. */
  int material_index = 0;
  SpanAttributeWriter<int> materials = attributes.lookup_or_add_for_write_span<int>(
      "material_index", ATTR_DOMAIN_CURVE);

  materials.span.slice(new_curves_range).fill(material_index);

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

  grease_pencil_eval.runtime->stroke_cache.clear();
  drawing_orig.tag_positions_changed();

  vertex_colors.finish();
  materials.finish();

  DEG_id_tag_update(&grease_pencil_orig.id, ID_RECALC_GEOMETRY);
  WM_main_add_notifier(NC_GEOM | ND_DATA, &grease_pencil_orig.id);
}

std::unique_ptr<GreasePencilStrokeOperation> new_paint_operation()
{
  return std::make_unique<PaintOperation>();
}

}  // namespace blender::ed::sculpt_paint::greasepencil
