/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BKE_brush.hh"
#include "BKE_colortools.hh"
#include "BKE_context.hh"
#include "BKE_crazyspace.hh"
#include "BKE_curves.hh"
#include "BKE_deform.hh"
#include "BKE_grease_pencil_vertex_groups.hh"
#include "BKE_modifier.hh"
#include "BKE_object_deform.h"
#include "BKE_scene.hh"

#include "DEG_depsgraph_query.hh"

#include "BLI_kdtree.h"

#include "DNA_meshdata_types.h"

#include "ED_grease_pencil.hh"
#include "ED_view3d.hh"

#include "grease_pencil_intern.hh"

namespace blender::ed::sculpt_paint::greasepencil {

static constexpr int POINT_CACHE_CHUNK = 2048;
static constexpr float FIND_NEAREST_POINT_EPSILON = 1e-6f;
static constexpr int BLUR_NEIGHBOUR_NUM = 5;
static constexpr int SMEAR_NEIGHBOUR_NUM = 8;

class WeightPaintOperation : public GreasePencilStrokeOperation {
 public:
  struct BrushPoint {
    float influence;
    int drawing_point_index;
  };

  struct DrawingWeightData {
    int active_vertex_group;
    MutableSpan<MDeformVert> deform_verts;
    VMutableArray<float> deform_weights;
    float multi_frame_falloff;

    Vector<bool> locked_vgroups;
    Vector<bool> bone_deformed_vgroups;

    Array<float2> point_positions;
    Array<int> point_index_in_kdtree;

    /* Collected points under the brush in one #on_stroke_extended action. */
    Vector<BrushPoint> points_in_brush;
  };

  struct PointsInBrushStroke {
    KDTree_2d *nearest_points;
    Vector<float2> nearest_points_positions;
    Vector<float> nearest_points_weights;
    /* Size of the KDtree. */
    int nearest_points_size;
  };

  Object *object;
  GreasePencil *grease_pencil;
  Brush *brush;
  float initial_brush_radius;
  float brush_radius;
  float brush_radius_wide;
  float initial_brush_strength;
  float brush_strength;
  float brush_weight;
  float2 mouse_position;
  float2 mouse_position_previous;
  rctf brush_bbox;

  /* Flag for Auto-normalize weights of bone deformed vertex groups. */
  bool auto_normalize;
  /* Brush mode: normal, invert or smooth. */
  BrushStrokeMode brush_mode;
  /* Add or subtract weight? */
  bool invert_brush_weight;
  /* Active vertex group in GP object. */
  bDeformGroup *object_defgroup;

  /* Collected points under the brush during the entire brush stroke. Used for finding nearest
   * points for Smear and Blur. Stored per frame group. */
  Array<PointsInBrushStroke> points_in_stroke;

  /* The number of points stored in the stroke point buffers. Per frame group.
   * Note: we can't use Array or Vector here, because it doesn't support atomic types. */
  std::vector<std::atomic<int>> points_in_stroke_num;

  /* Weight paint data per editable drawing. Stored per frame group. */
  Array<Array<DrawingWeightData>> drawing_weight_data;

  /* Set of bone-deformed vertex groups (object level). */
  Set<std::string> object_bone_deformed_defgroups;
  /* Set of locked vertex groups (object level). */
  Set<std::string> object_locked_defgroups;

  ~WeightPaintOperation() override {}

  /* Ensure there is enough space in the buffer with brush stroke points to add a new point. */
  void ensure_stroke_buffer_size(const int frame_group, std::mutex &mutex)
  {
    /* We take a little margin here, to be thread safe. */
    PointsInBrushStroke &buffer = this->points_in_stroke[frame_group];
    if (this->points_in_stroke_num[frame_group] < buffer.nearest_points_size - 32) {
      return;
    }

    std::lock_guard lock{mutex};
    BLI_kdtree_2d_free(buffer.nearest_points);
    buffer.nearest_points_size += POINT_CACHE_CHUNK;

    buffer.nearest_points_positions.resize(buffer.nearest_points_size);
    buffer.nearest_points_weights.resize(buffer.nearest_points_size);

    /* Rebuild KDtree. */
    buffer.nearest_points = BLI_kdtree_2d_new(buffer.nearest_points_size);
    for (const int i : IndexRange(this->points_in_stroke_num[frame_group])) {
      BLI_kdtree_2d_insert(buffer.nearest_points, i, buffer.nearest_points_positions[i]);
    }
  }

  /* Apply a weight to a point under the brush. */
  void apply_weight_to_point(const BrushPoint &point,
                             const float target_weight,
                             DrawingWeightData &drawing_weight)
  {
    /* Blend the current point weight with the target weight. */
    const float old_weight = drawing_weight.deform_weights[point.drawing_point_index];
    const float weight_delta = (this->invert_brush_weight ? (1.0f - target_weight) :
                                                            target_weight) -
                               old_weight;
    drawing_weight.deform_weights.set(
        point.drawing_point_index,
        math::clamp(
            old_weight + math::interpolate(0.0f, weight_delta, point.influence), 0.0f, 1.0f));
  }

  /* Get brush settings (radius, strength etc.) */
  void get_brush_settings(const bContext &C, const InputSample &start_sample)
  {
    using namespace blender::ed::greasepencil;

    const Scene *scene = CTX_data_scene(&C);
    this->object = CTX_data_active_object(&C);
    this->grease_pencil = static_cast<GreasePencil *>(this->object->data);
    Paint *paint = BKE_paint_get_active_from_context(&C);
    Brush *brush = BKE_paint_brush(paint);

    this->brush = brush;
    this->initial_brush_radius = BKE_brush_size_get(scene, brush);
    this->initial_brush_strength = BKE_brush_alpha_get(scene, brush);
    this->brush_weight = BKE_brush_weight_get(scene, brush);
    this->mouse_position_previous = start_sample.mouse_position;
    this->invert_brush_weight = false;

    BKE_curvemapping_init(brush->curve);

    /* Auto-normalize weights is only applied when the object is deformed by an armature. */
    const ToolSettings *ts = CTX_data_tool_settings(&C);
    this->auto_normalize = ts->auto_normalize &&
                           (BKE_modifiers_is_deformed_by_armature(this->object) != nullptr);
  }

  /* Get or create active vertex group in GP object. */
  void ensure_active_vertex_group_in_object()
  {
    int object_defgroup_nr = BKE_object_defgroup_active_index_get(this->object) - 1;
    if (object_defgroup_nr == -1) {
      BKE_object_defgroup_add(this->object);
      object_defgroup_nr = 0;
    }
    this->object_defgroup = static_cast<bDeformGroup *>(
        BLI_findlink(BKE_object_defgroup_list(this->object), object_defgroup_nr));
  }

  /* Get locked and bone-deformed vertex groups in GP object. */
  void get_locked_and_bone_deformed_vertex_groups()
  {
    const ListBase *defgroups = BKE_object_defgroup_list(this->object);
    LISTBASE_FOREACH (bDeformGroup *, dg, defgroups) {
      if ((dg->flag & DG_LOCK_WEIGHT) != 0) {
        this->object_locked_defgroups.add(dg->name);
      }
    }
    this->object_bone_deformed_defgroups = ed::greasepencil::get_bone_deformed_vertex_group_names(
        *this->object);
  }

  /* Init point buffer for all stroke points hit by the brush during a stroke operation. */
  void init_stroke_point_buffer(const int frame_group)
  {
    this->points_in_stroke_num[frame_group] = 0;
    PointsInBrushStroke &points_in_stroke = this->points_in_stroke[frame_group];
    points_in_stroke.nearest_points_size = POINT_CACHE_CHUNK;
    points_in_stroke.nearest_points_positions = Vector<float2>(POINT_CACHE_CHUNK);
    points_in_stroke.nearest_points_weights = Vector<float>(POINT_CACHE_CHUNK);
    points_in_stroke.nearest_points = BLI_kdtree_2d_new(POINT_CACHE_CHUNK);
  }

  /* For each drawing, retrieve pointers to the vertex weight data of the active vertex group,
   * so that we can read and write to them later. And create buffers for points under the brush
   * during one #on_stroke_extended action. */
  void init_weight_data_for_drawings(const bContext &C,
                                     const Span<ed::greasepencil::MutableDrawingInfo> &drawings,
                                     const int frame_group)
  {
    const Depsgraph *depsgraph = CTX_data_depsgraph_pointer(&C);
    const Object *ob_eval = DEG_get_evaluated_object(depsgraph, this->object);
    const RegionView3D *rv3d = CTX_wm_region_view3d(&C);
    const ARegion *region = CTX_wm_region(&C);
    const float4x4 projection = ED_view3d_ob_project_mat_get(rv3d, this->object);

    this->drawing_weight_data[frame_group] = Array<DrawingWeightData>(drawings.size());

    threading::parallel_for(drawings.index_range(), 1, [&](const IndexRange range) {
      for (const int drawing_index : range) {
        const ed::greasepencil::MutableDrawingInfo &drawing_info = drawings[drawing_index];
        bke::CurvesGeometry &curves = drawing_info.drawing.strokes_for_write();

        /* Find or create the active vertex group in the drawing. */
        DrawingWeightData &drawing_weight_data =
            this->drawing_weight_data[frame_group][drawing_index];
        drawing_weight_data.active_vertex_group = bke::greasepencil::ensure_vertex_group(
            this->object_defgroup->name, curves.vertex_group_names);

        drawing_weight_data.multi_frame_falloff = drawing_info.multi_frame_falloff;
        drawing_weight_data.deform_verts = curves.deform_verts_for_write();
        drawing_weight_data.deform_weights = bke::varray_for_mutable_deform_verts(
            drawing_weight_data.deform_verts, drawing_weight_data.active_vertex_group);

        /* Create boolean arrays indicating whether a vertex group is locked/bone deformed
         * or not. */
        if (this->auto_normalize) {
          LISTBASE_FOREACH (bDeformGroup *, dg, &curves.vertex_group_names) {
            drawing_weight_data.locked_vgroups.append(
                this->object_locked_defgroups.contains(dg->name));
            drawing_weight_data.bone_deformed_vgroups.append(
                this->object_bone_deformed_defgroups.contains(dg->name));
          }
        }

        /* Convert stroke points to screen space positions. */
        bke::crazyspace::GeometryDeformation deformation =
            bke::crazyspace::get_evaluated_grease_pencil_drawing_deformation(
                ob_eval, *this->object, drawing_info.layer_index, drawing_info.frame_number);
        drawing_weight_data.point_positions = Array<float2>(deformation.positions.size());
        drawing_weight_data.point_index_in_kdtree = Array<int>(deformation.positions.size(), -1);
        threading::parallel_for(curves.points_range(), 1024, [&](const IndexRange point_range) {
          for (const int point : point_range) {
            drawing_weight_data.point_positions[point] = ED_view3d_project_float_v2_m4(
                region, deformation.positions[point], projection);
          }
        });
      }
    });
  }

  /* Get mouse position and pressure. */
  void get_mouse_input_sample(const InputSample &input_sample,
                              const float brush_widen_factor = 1.0f)
  {
    this->mouse_position = input_sample.mouse_position;
    this->brush_radius = this->initial_brush_radius;
    if (BKE_brush_use_size_pressure(this->brush)) {
      this->brush_radius *= input_sample.pressure;
    }
    this->brush_strength = this->initial_brush_strength;
    if (BKE_brush_use_alpha_pressure(this->brush)) {
      this->brush_strength *= input_sample.pressure;
    }
    this->brush_radius_wide = this->brush_radius * brush_widen_factor;

    BLI_rctf_init(&this->brush_bbox,
                  this->mouse_position.x - this->brush_radius_wide,
                  this->mouse_position.x + this->brush_radius_wide,
                  this->mouse_position.y - this->brush_radius_wide,
                  this->mouse_position.y + this->brush_radius_wide);
  }

  /* Add a point to the brush buffer when it is within the brush radius.
   * Returns false when the point is outside the brush. */
  bool add_point_under_brush_to_brush_buffer(const float2 point_position,
                                             DrawingWeightData &drawing_weight,
                                             const int point_index)
  {
    if (!BLI_rctf_isect_pt_v(&this->brush_bbox, point_position)) {
      return false;
    }
    const float dist_point_to_brush_center = math::distance(point_position, this->mouse_position);
    if (dist_point_to_brush_center > this->brush_radius_wide) {
      return false;
    }
    if (dist_point_to_brush_center > this->brush_radius) {
      return true;
    }

    /* When the point is under the brush, add it to the brush buffer. */
    const float influence = drawing_weight.multi_frame_falloff * this->brush_strength *
                            BKE_brush_curve_strength(
                                this->brush, dist_point_to_brush_center, this->brush_radius);
    if (influence != 0.0f) {
      drawing_weight.points_in_brush.append({influence, point_index});
    }

    return true;
  }

  /* Add a point to the stroke point buffer. Returns true when the point is newly added. */
  bool add_point_to_stroke_buffer(const float2 point_position,
                                  const int frame_group,
                                  const int point_index,
                                  DrawingWeightData &drawing_weight,
                                  std::mutex &mutex)
  {
    PointsInBrushStroke &points_in_stroke = this->points_in_stroke[frame_group];

    /* Add the point only once to the buffer, avoid duplicates. */
    if (drawing_weight.point_index_in_kdtree[point_index] == -1) {
      this->ensure_stroke_buffer_size(frame_group, mutex);
      const int buffer_index = (this->points_in_stroke_num[frame_group]++);
      points_in_stroke.nearest_points_positions[buffer_index] = point_position;
      points_in_stroke.nearest_points_weights[buffer_index] =
          drawing_weight.deform_weights[point_index];
      BLI_kdtree_2d_insert(points_in_stroke.nearest_points, buffer_index, point_position);
      drawing_weight.point_index_in_kdtree[point_index] = buffer_index;

      return true;
    }

    /* When the point was already in the stroke buffer, update the weight. */
    points_in_stroke.nearest_points_weights[drawing_weight.point_index_in_kdtree[point_index]] =
        drawing_weight.deform_weights[point_index];

    return false;
  }
};

}  // namespace blender::ed::sculpt_paint::greasepencil
