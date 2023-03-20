/* SPDX-License-Identifier: GPL-2.0-or-later */

#include <algorithm>

#include "curves_sculpt_intern.hh"

#include "BLI_index_mask_ops.hh"
#include "BLI_kdtree.h"
#include "BLI_math_matrix_types.hh"
#include "BLI_rand.hh"
#include "BLI_vector.hh"

#include "PIL_time.h"

#include "DEG_depsgraph.h"
#include "DEG_depsgraph_query.h"

#include "BKE_attribute_math.hh"
#include "BKE_brush.h"
#include "BKE_bvhutils.h"
#include "BKE_colortools.h"
#include "BKE_context.h"
#include "BKE_crazyspace.hh"
#include "BKE_curves.hh"
#include "BKE_geometry_set.hh"
#include "BKE_mesh.hh"
#include "BKE_mesh_runtime.h"
#include "BKE_paint.h"

#include "DNA_brush_enums.h"
#include "DNA_brush_types.h"
#include "DNA_curves_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_object_types.h"
#include "DNA_screen_types.h"
#include "DNA_space_types.h"

#include "ED_screen.h"
#include "ED_view3d.h"

#include "UI_interface.h"

#include "WM_api.h"

/**
 * The code below uses a prefix naming convention to indicate the coordinate space:
 * cu: Local space of the curves object that is being edited.
 * su: Local space of the surface object.
 * wo: World space.
 * re: 2D coordinates within the region.
 */

namespace blender::ed::sculpt_paint {

using blender::bke::CurvesGeometry;
using threading::EnumerableThreadSpecific;

/**
 * Moves individual points under the brush and does a length preservation step afterwards.
 */
class CombOperation : public CurvesSculptStrokeOperation {
 private:
  /** Last mouse position. */
  float2 brush_pos_last_re_;

  /** Only used when a 3D brush is used. */
  CurvesBrush3D brush_3d_;

  /** Solver for length and collision constraints. */
  CurvesConstraintSolver constraint_solver_;

  Array<float> curve_lengths_;

  friend struct CombOperationExecutor;

 public:
  void on_stroke_extended(const bContext &C, const StrokeExtension &stroke_extension) override;
};

/**
 * Utility class that actually executes the update when the stroke is updated. That's useful
 * because it avoids passing a very large number of parameters between functions.
 */
struct CombOperationExecutor {
  CombOperation *self_ = nullptr;
  CurvesSculptCommonContext ctx_;

  const CurvesSculpt *curves_sculpt_ = nullptr;
  const Brush *brush_ = nullptr;
  float brush_radius_base_re_;
  float brush_radius_factor_;
  float brush_strength_;

  eBrushFalloffShape falloff_shape_;

  Object *curves_ob_orig_ = nullptr;
  Curves *curves_id_orig_ = nullptr;
  CurvesGeometry *curves_orig_ = nullptr;

  VArray<float> point_factors_;
  Vector<int64_t> selected_curve_indices_;
  IndexMask curve_selection_;

  float2 brush_pos_prev_re_;
  float2 brush_pos_re_;
  float2 brush_pos_diff_re_;

  CurvesSurfaceTransforms transforms_;

  CombOperationExecutor(const bContext &C) : ctx_(C)
  {
  }

  void execute(CombOperation &self, const bContext &C, const StrokeExtension &stroke_extension)
  {
    self_ = &self;

    BLI_SCOPED_DEFER([&]() { self_->brush_pos_last_re_ = stroke_extension.mouse_position; });

    curves_ob_orig_ = CTX_data_active_object(&C);
    curves_id_orig_ = static_cast<Curves *>(curves_ob_orig_->data);
    curves_orig_ = &curves_id_orig_->geometry.wrap();
    if (curves_orig_->curves_num() == 0) {
      return;
    }

    curves_sculpt_ = ctx_.scene->toolsettings->curves_sculpt;
    brush_ = BKE_paint_brush_for_read(&curves_sculpt_->paint);
    brush_radius_base_re_ = BKE_brush_size_get(ctx_.scene, brush_);
    brush_radius_factor_ = brush_radius_factor(*brush_, stroke_extension);
    brush_strength_ = brush_strength_get(*ctx_.scene, *brush_, stroke_extension);

    falloff_shape_ = static_cast<eBrushFalloffShape>(brush_->falloff_shape);

    transforms_ = CurvesSurfaceTransforms(*curves_ob_orig_, curves_id_orig_->surface);

    point_factors_ = curves_orig_->attributes().lookup_or_default<float>(
        ".selection", ATTR_DOMAIN_POINT, 1.0f);
    curve_selection_ = curves::retrieve_selected_curves(*curves_id_orig_, selected_curve_indices_);

    brush_pos_prev_re_ = self_->brush_pos_last_re_;
    brush_pos_re_ = stroke_extension.mouse_position;
    brush_pos_diff_re_ = brush_pos_re_ - brush_pos_prev_re_;

    if (stroke_extension.is_first) {
      if (falloff_shape_ == PAINT_FALLOFF_SHAPE_SPHERE) {
        this->initialize_spherical_brush_reference_point();
      }
      self_->constraint_solver_.initialize(
          *curves_orig_,
          curve_selection_,
          curves_id_orig_->flag & CV_SCULPT_COLLISION_ENABLED ?
              CurvesConstraintSolver::CollisionConstraintType::Raycast :
              CurvesConstraintSolver::CollisionConstraintType::None,
          CurvesConstraintSolver::LengthConstraintType::Symmetric,
          CurvesConstraintSolver::GoalConstraintType::Keyhole);

      self_->curve_lengths_.reinitialize(curves_orig_->curves_num());
      const Span<float> segment_lengths = self_->constraint_solver_.segment_lengths();
      const OffsetIndices points_by_curve = curves_orig_->points_by_curve();
      threading::parallel_for(curve_selection_.index_range(), 512, [&](const IndexRange range) {
        for (const int curve_i : curve_selection_.slice(range)) {
          const IndexRange points = points_by_curve[curve_i];
          const Span<float> lengths = segment_lengths.slice(points.drop_back(1));
          self_->curve_lengths_[curve_i] = std::accumulate(lengths.begin(), lengths.end(), 0.0f);
        }
      });
      /* Combing does nothing when there is no mouse movement, so return directly. */
      return;
    }

    Array<bool> changed_curves(curves_orig_->curves_num(), false);

    if (falloff_shape_ == PAINT_FALLOFF_SHAPE_TUBE) {
      this->comb_projected_with_symmetry(changed_curves);
    }
    else if (falloff_shape_ == PAINT_FALLOFF_SHAPE_SPHERE) {
      this->comb_spherical_with_symmetry(changed_curves);
    }
    else {
      BLI_assert_unreachable();
    }

    const Mesh *surface = curves_id_orig_->surface && curves_id_orig_->surface->type == OB_MESH ?
                              static_cast<Mesh *>(curves_id_orig_->surface->data) :
                              nullptr;

    Vector<int64_t> indices;
    const IndexMask changed_curves_mask = index_mask_ops::find_indices_from_array(changed_curves,
                                                                                  indices);
    self_->constraint_solver_.solve_step(
        *curves_orig_, changed_curves_mask, surface, transforms_, point_factors_, 5);

    curves_orig_->tag_positions_changed();
    DEG_id_tag_update(&curves_id_orig_->id, ID_RECALC_GEOMETRY);
    WM_main_add_notifier(NC_GEOM | ND_DATA, &curves_id_orig_->id);
    ED_region_tag_redraw(ctx_.region);
  }

  void find_or_update_goals_projected(const OffsetIndices<int> points_by_curve,
                                      const Span<float3> positions,
                                      const IndexMask curve_selection,
                                      const float2 &target_point,
                                      const float brush_radius_sq,
                                      MutableSpan<bool> r_used_curves)
  {
    MutableSpan<bool> has_goals = self_->constraint_solver_.has_goals();
    MutableSpan<float3> goals = self_->constraint_solver_.goals();
    MutableSpan<int> closest_points = self_->constraint_solver_.closest_points();
    MutableSpan<float> closest_factors = self_->constraint_solver_.closest_factors();

    BLI_assert(goals.size() == points_by_curve.ranges_num());

    float4x4 projection;
    ED_view3d_ob_project_mat_get(ctx_.rv3d, curves_ob_orig_, projection.ptr());

    threading::parallel_for(curve_selection.index_range(), 256, [&](const IndexRange range) {
      for (const int curve_i : curve_selection.slice(range)) {
        const IndexRange points = points_by_curve[curve_i].drop_back(1);
        int min_point_i = -1;
        float min_distance_sq = FLT_MAX;
        float min_lambda;
        float2 min_closest_re;
        for (const int point_i : points) {
          float2 p0_re, p1_re;
          ED_view3d_project_float_v2_m4(ctx_.region, positions[point_i], p0_re, projection.ptr());
          ED_view3d_project_float_v2_m4(ctx_.region, positions[point_i + 1], p1_re, projection.ptr());

                // TODO add brush bounding box check to speed up culling
          float2 closest_re;
          const float lambda = closest_to_line_segment_v2(closest_re, target_point, p0_re, p1_re);
          const float distance_sq = math::distance_squared(closest_re, target_point);
          if (distance_sq <= brush_radius_sq && distance_sq < min_distance_sq) {
            min_point_i = point_i;
            min_distance_sq = distance_sq;
            min_lambda = lambda;
            min_closest_re = closest_re;
          }
        }

        if (min_point_i >= 0) {
          r_used_curves[curve_i] = true;

          /* Initialize new goals */
          if (!has_goals[curve_i]) {
            float3 min_closest_cu;
            ED_view3d_win_to_3d(ctx_.v3d,
                                ctx_.region,
                                math::transform_point(transforms_.curves_to_world, positions[min_point_i]),
                                min_closest_re,
                                min_closest_cu);
            has_goals[curve_i] = true;
            goals[curve_i] = min_closest_cu;
            closest_points[curve_i] = min_point_i;
            closest_factors[curve_i] = min_lambda;
          }
        }
        else {
          r_used_curves[curve_i] = false;

          /* Drop existing goals */
          has_goals[curve_i] = false;
        }
      }
    });
  }

  void find_or_update_goals_spherical(const OffsetIndices<int> points_by_curve,
                                      const Span<float3> positions,
                                      const IndexMask curve_selection,
                                      const float3 &target_point,
                                      const float brush_radius_sq,
                                      MutableSpan<bool> r_used_curves)
  {
    MutableSpan<bool> has_goals = self_->constraint_solver_.has_goals();
    MutableSpan<float3> goals = self_->constraint_solver_.goals();
    MutableSpan<int> closest_points = self_->constraint_solver_.closest_points();
    MutableSpan<float> closest_factors = self_->constraint_solver_.closest_factors();

    BLI_assert(goals.size() == points_by_curve.ranges_num());

    threading::parallel_for(curve_selection.index_range(), 256, [&](const IndexRange range) {
      for (const int curve_i : curve_selection.slice(range)) {
        const IndexRange points = points_by_curve[curve_i].drop_back(1);
        int min_point_i = -1;
        float min_distance_sq = FLT_MAX;
        float min_lambda;
        float3 min_closest;
        for (const int point_i : points) {
          // TODO add brush bounding box check to speed up culling
          float3 closest;
          const float lambda = closest_to_line_segment_v3(
              closest, target_point, positions[point_i], positions[point_i + 1]);
          const float distance_sq = math::distance_squared(closest, target_point);
          if (distance_sq <= brush_radius_sq && distance_sq < min_distance_sq) {
            min_point_i = point_i;
            min_distance_sq = distance_sq;
            min_lambda = lambda;
            min_closest = closest;
          }
        }

        if (min_point_i >= 0) {
          r_used_curves[curve_i] = true;

          /* Initialize new goals */
          if (!has_goals[curve_i]) {
            has_goals[curve_i] = true;
            goals[curve_i] = min_closest;
            closest_points[curve_i] = min_point_i;
            closest_factors[curve_i] = min_lambda;
          }
        }
        else {
          r_used_curves[curve_i] = false;

          /* Drop existing goals */
          has_goals[curve_i] = false;
        }
      }
    });
  }

  /**
   * Do combing in screen space.
   */
  void comb_projected_with_symmetry(MutableSpan<bool> r_changed_curves)
  {
    const Vector<float4x4> symmetry_brush_transforms = get_symmetry_brush_transforms(
        eCurvesSymmetryType(curves_id_orig_->symmetry));
    for (const float4x4 &brush_transform : symmetry_brush_transforms) {
      this->comb_projected(r_changed_curves, brush_transform);
    }
  }

#if 1
  void comb_projected(MutableSpan<bool> r_changed_curves, const float4x4 &brush_transform)
  {
    const bke::crazyspace::GeometryDeformation deformation =
        bke::crazyspace::get_evaluated_curves_deformation(*ctx_.depsgraph, *curves_ob_orig_);
    const OffsetIndices points_by_curve = curves_orig_->points_by_curve();

    float4x4 projection;
    ED_view3d_ob_project_mat_get(ctx_.rv3d, curves_ob_orig_, projection.ptr());

    const float brush_radius_re = brush_radius_base_re_ * brush_radius_factor_;
    const float brush_radius_sq_re = pow2f(brush_radius_re);

    CurveMapping &curve_parameter_falloff_mapping =
        *brush_->curves_sculpt_settings->curve_parameter_falloff;
    BKE_curvemapping_init(&curve_parameter_falloff_mapping);

    find_or_update_goals_projected(points_by_curve,
                                   deformation.positions,
                                   curve_selection_,
                                   brush_pos_prev_re_,
                                   brush_radius_sq_re,
                                   r_changed_curves);

    /* Move goals to the end of the stroke segment. */
    MutableSpan<float3> goals = self_->constraint_solver_.goals();
    MutableSpan<float> goal_factors = self_->constraint_solver_.goal_factors();
    Span<int> closest_points = self_->constraint_solver_.closest_points();
    Span<float> closest_factors = self_->constraint_solver_.closest_factors();
    threading::parallel_for(curve_selection_.index_range(), 256, [&](const IndexRange range) {
      for (const int curve_i : curve_selection_.slice(range)) {
        if (r_changed_curves[curve_i]) {

          const int closest_point = closest_points[curve_i];
          const float closest_u = closest_factors[curve_i];

          float2 p0_re, p1_re;
          ED_view3d_project_float_v2_m4(ctx_.region, deformation.positions[closest_point], p0_re, projection.ptr());
          ED_view3d_project_float_v2_m4(ctx_.region, deformation.positions[closest_point + 1], p1_re, projection.ptr());
          const float2 closest_pos_re = (1.0f - closest_u) * p0_re + closest_u * p1_re;

          /* Compute distance to the brush. */
          const float distance_to_closest_sq_re = dist_squared_to_line_segment_v3(
              closest_pos_re, brush_pos_prev_re_, brush_pos_re_);
          if (distance_to_closest_sq_re > brush_radius_sq_re) {
            /* Ignore the curve because it's too far away. */
            goal_factors[curve_i] = 0.0f;
          }
          else {
            const float distance_to_closest_re = std::sqrt(distance_to_closest_sq_re);

            /* A falloff that is based on how far away the point is from the stroke. */
            const float radius_falloff = BKE_brush_curve_strength(
                brush_, distance_to_closest_re, brush_radius_re);
            /* Combine the falloff and brush strength. */
            goal_factors[curve_i] = brush_strength_ * radius_falloff;

            /* Update goal */
            float3 brush_start_cu, brush_end_cu;
            const float3 depth_point_cu = math::transform_point(transforms_.curves_to_world, goals[curve_i]);
            ED_view3d_win_to_3d(
                ctx_.v3d, ctx_.region, depth_point_cu, brush_pos_prev_re_, brush_start_cu);
            ED_view3d_win_to_3d(
                ctx_.v3d, ctx_.region, depth_point_cu, brush_pos_re_, brush_end_cu);
            goals[curve_i] += brush_end_cu - brush_start_cu;
          }
        }
      }
    });
  }
#else
  void comb_projected(MutableSpan<bool> r_changed_curves, const float4x4 &brush_transform)
  {
    const float4x4 brush_transform_inv = math::invert(brush_transform);

    MutableSpan<float3> positions_cu_orig = curves_orig_->positions_for_write();
    const bke::crazyspace::GeometryDeformation deformation =
        bke::crazyspace::get_evaluated_curves_deformation(*ctx_.depsgraph, *curves_ob_orig_);
    const OffsetIndices points_by_curve = curves_orig_->points_by_curve();

    float4x4 projection;
    ED_view3d_ob_project_mat_get(ctx_.rv3d, curves_ob_orig_, projection.ptr());

    const float brush_radius_re = brush_radius_base_re_ * brush_radius_factor_;
    const float brush_radius_sq_re = pow2f(brush_radius_re);

    CurveMapping &curve_parameter_falloff_mapping =
        *brush_->curves_sculpt_settings->curve_parameter_falloff;
    BKE_curvemapping_init(&curve_parameter_falloff_mapping);

    const Span<float> segment_lengths = self_->constraint_solver_.segment_lengths();

    threading::parallel_for(curve_selection_.index_range(), 256, [&](const IndexRange range) {
      for (const int curve_i : curve_selection_.slice(range)) {
        bool curve_changed = false;
        const IndexRange points = points_by_curve[curve_i];

        const float total_length = self_->curve_lengths_[curve_i];
        const float total_length_inv = safe_divide(1.0f, total_length);
        float current_length = 0.0f;
        for (const int point_i : points.drop_front(1)) {
          current_length += segment_lengths[point_i - 1];

          const float3 old_pos_cu = deformation.positions[point_i];
          const float3 old_symm_pos_cu = math::transform_point(brush_transform_inv, old_pos_cu);

          /* Find the position of the point in screen space. */
          float2 old_symm_pos_re;
          ED_view3d_project_float_v2_m4(
              ctx_.region, old_symm_pos_cu, old_symm_pos_re, projection.ptr());

          const float distance_to_brush_sq_re = dist_squared_to_line_segment_v2(
              old_symm_pos_re, brush_pos_prev_re_, brush_pos_re_);
          if (distance_to_brush_sq_re > brush_radius_sq_re) {
            /* Ignore the point because it's too far away. */
            continue;
          }

          const float distance_to_brush_re = std::sqrt(distance_to_brush_sq_re);
          /* A falloff that is based on how far away the point is from the stroke. */
          const float radius_falloff = BKE_brush_curve_strength(
              brush_, distance_to_brush_re, brush_radius_re);
          const float curve_parameter = current_length * total_length_inv;
          const float curve_falloff = BKE_curvemapping_evaluateF(
              &curve_parameter_falloff_mapping, 0, curve_parameter);
          /* Combine the falloff and brush strength. */
          const float weight = brush_strength_ * curve_falloff * radius_falloff *
                               point_factors_[point_i];

          /* Offset the old point position in screen space and transform it back into 3D space.
           */
          const float2 new_symm_pos_re = old_symm_pos_re + brush_pos_diff_re_ * weight;
          float3 new_symm_pos_wo;
          ED_view3d_win_to_3d(ctx_.v3d,
                              ctx_.region,
                              math::transform_point(transforms_.curves_to_world, old_symm_pos_cu),
                              new_symm_pos_re,
                              new_symm_pos_wo);
          const float3 new_pos_cu = math::transform_point(
              brush_transform,
              math::transform_point(transforms_.world_to_curves, new_symm_pos_wo));

          const float3 translation_eval = new_pos_cu - old_pos_cu;
          const float3 translation_orig = deformation.translation_from_deformed_to_original(
              point_i, translation_eval);
          positions_cu_orig[point_i] += translation_orig;

          curve_changed = true;
        }
        if (curve_changed) {
          r_changed_curves[curve_i] = true;
        }
      }
    });
  }
#endif

  /**
   * Do combing in 3D space.
   */
  void comb_spherical_with_symmetry(MutableSpan<bool> r_changed_curves)
  {
    float4x4 projection;
    ED_view3d_ob_project_mat_get(ctx_.rv3d, curves_ob_orig_, projection.ptr());

    float3 brush_start_wo, brush_end_wo;
    ED_view3d_win_to_3d(
        ctx_.v3d,
        ctx_.region,
        math::transform_point(transforms_.curves_to_world, self_->brush_3d_.position_cu),
        brush_pos_prev_re_,
        brush_start_wo);
    ED_view3d_win_to_3d(
        ctx_.v3d,
        ctx_.region,
        math::transform_point(transforms_.curves_to_world, self_->brush_3d_.position_cu),
        brush_pos_re_,
        brush_end_wo);
    const float3 brush_start_cu = math::transform_point(transforms_.world_to_curves,
                                                        brush_start_wo);
    const float3 brush_end_cu = math::transform_point(transforms_.world_to_curves, brush_end_wo);

    const float brush_radius_cu = self_->brush_3d_.radius_cu * brush_radius_factor_;

    const Vector<float4x4> symmetry_brush_transforms = get_symmetry_brush_transforms(
        eCurvesSymmetryType(curves_id_orig_->symmetry));
    for (const float4x4 &brush_transform : symmetry_brush_transforms) {
      this->comb_spherical(r_changed_curves,
                           math::transform_point(brush_transform, brush_start_cu),
                           math::transform_point(brush_transform, brush_end_cu),
                           brush_radius_cu);
    }
  }

#if 1
  void comb_spherical(MutableSpan<bool> r_changed_curves,
                      const float3 &brush_start_cu,
                      const float3 &brush_end_cu,
                      const float brush_radius_cu)
  {
    const float brush_radius_sq_cu = pow2f(brush_radius_cu);
    const float3 brush_diff_cu = brush_end_cu - brush_start_cu;

    CurveMapping &curve_parameter_falloff_mapping =
        *brush_->curves_sculpt_settings->curve_parameter_falloff;
    BKE_curvemapping_init(&curve_parameter_falloff_mapping);

    const bke::crazyspace::GeometryDeformation deformation =
        bke::crazyspace::get_evaluated_curves_deformation(*ctx_.depsgraph, *curves_ob_orig_);
    const OffsetIndices points_by_curve = curves_orig_->points_by_curve();

    find_or_update_goals_spherical(points_by_curve,
                                   deformation.positions,
                                   curve_selection_,
                                   brush_start_cu,
                                   brush_radius_sq_cu,
                                   r_changed_curves);

    /* Move goals to the end of the stroke segment. */
    MutableSpan<float3> goals = self_->constraint_solver_.goals();
    MutableSpan<float> goal_factors = self_->constraint_solver_.goal_factors();
    Span<int> closest_points = self_->constraint_solver_.closest_points();
    Span<float> closest_factors = self_->constraint_solver_.closest_factors();
    threading::parallel_for(curve_selection_.index_range(), 256, [&](const IndexRange range) {
      for (const int curve_i : curve_selection_.slice(range)) {
        if (r_changed_curves[curve_i]) {
          const int closest_point = closest_points[curve_i];
          const float closest_u = closest_factors[curve_i];
          const float3 closest = (1.0f - closest_u) * deformation.positions[closest_point] + closest_u * deformation.positions[closest_point + 1];

          /* Compute distance to the brush. */
          const float distance_to_closest_sq = dist_squared_to_line_segment_v3(
              closest, brush_start_cu, brush_end_cu);
          if (distance_to_closest_sq > brush_radius_sq_cu) {
            /* Ignore the curve because it's too far away. */
            goal_factors[curve_i] = 0.0f;
          }
          else {
            const float distance_to_closest = std::sqrt(distance_to_closest_sq);

            /* A falloff that is based on how far away the point is from the stroke. */
            const float radius_falloff = BKE_brush_curve_strength(
                brush_, distance_to_closest, brush_radius_cu);
            /* Combine the falloff and brush strength. */
            goal_factors[curve_i] = brush_strength_ * radius_falloff;

            /* Update goal */
            goals[curve_i] += brush_diff_cu;
          }
        }
      }
    });
  }
#else
  void comb_spherical(MutableSpan<bool> r_changed_curves,
                      const float3 &brush_start_cu,
                      const float3 &brush_end_cu,
                      const float brush_radius_cu)
  {
    MutableSpan<float3> positions_cu = curves_orig_->positions_for_write();
    const float brush_radius_sq_cu = pow2f(brush_radius_cu);
    const float3 brush_diff_cu = brush_end_cu - brush_start_cu;

    CurveMapping &curve_parameter_falloff_mapping =
        *brush_->curves_sculpt_settings->curve_parameter_falloff;
    BKE_curvemapping_init(&curve_parameter_falloff_mapping);

    const bke::crazyspace::GeometryDeformation deformation =
        bke::crazyspace::get_evaluated_curves_deformation(*ctx_.depsgraph, *curves_ob_orig_);
    const OffsetIndices points_by_curve = curves_orig_->points_by_curve();
    const Span<float> segment_lengths = self_->constraint_solver_.segment_lengths();

    threading::parallel_for(curve_selection_.index_range(), 256, [&](const IndexRange range) {
      for (const int curve_i : curve_selection_.slice(range)) {
        bool curve_changed = false;
        const IndexRange points = points_by_curve[curve_i];

        const float total_length = self_->curve_lengths_[curve_i];
        const float total_length_inv = safe_divide(1.0f, total_length);
        float current_length = 0.0f;
        for (const int point_i : points.drop_front(1)) {
          current_length += segment_lengths[point_i - 1];

          const float3 pos_old_cu = deformation.positions[point_i];

          /* Compute distance to the brush. */
          const float distance_to_brush_sq_cu = dist_squared_to_line_segment_v3(
              pos_old_cu, brush_start_cu, brush_end_cu);
          if (distance_to_brush_sq_cu > brush_radius_sq_cu) {
            /* Ignore the point because it's too far away. */
            continue;
          }

          const float distance_to_brush_cu = std::sqrt(distance_to_brush_sq_cu);

          /* A falloff that is based on how far away the point is from the stroke. */
          const float radius_falloff = BKE_brush_curve_strength(
              brush_, distance_to_brush_cu, brush_radius_cu);
          const float curve_parameter = current_length * total_length_inv;
          const float curve_falloff = BKE_curvemapping_evaluateF(
              &curve_parameter_falloff_mapping, 0, curve_parameter);
          /* Combine the falloff and brush strength. */
          const float weight = brush_strength_ * curve_falloff * radius_falloff *
                               point_factors_[point_i];

          const float3 translation_eval_cu = weight * brush_diff_cu;
          const float3 translation_orig_cu = deformation.translation_from_deformed_to_original(
              point_i, translation_eval_cu);

          /* Update the point position. */
          positions_cu[point_i] += translation_orig_cu;

          curve_changed = true;
        }
        if (curve_changed) {
          r_changed_curves[curve_i] = true;
        }
      }
    });
  }
#endif

  /**
   * Sample depth under mouse by looking at curves and the surface.
   */
  void initialize_spherical_brush_reference_point()
  {
    std::optional<CurvesBrush3D> brush_3d = sample_curves_3d_brush(*ctx_.depsgraph,
                                                                   *ctx_.region,
                                                                   *ctx_.v3d,
                                                                   *ctx_.rv3d,
                                                                   *curves_ob_orig_,
                                                                   brush_pos_re_,
                                                                   brush_radius_base_re_);
    if (brush_3d.has_value()) {
      self_->brush_3d_ = *brush_3d;
    }
  }
};

void CombOperation::on_stroke_extended(const bContext &C, const StrokeExtension &stroke_extension)
{
  CombOperationExecutor executor{C};
  executor.execute(*this, C, stroke_extension);
}

std::unique_ptr<CurvesSculptStrokeOperation> new_comb_operation()
{
  return std::make_unique<CombOperation>();
}

}  // namespace blender::ed::sculpt_paint
