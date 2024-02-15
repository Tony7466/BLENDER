/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <map>

#include "BLI_array.hh"
#include "BLI_kdtree.h"
#include "BLI_rect.h"
#include "BLI_set.hh"
#include "BLI_task.hh"

#include "BKE_brush.hh"
#include "BKE_colortools.hh"
#include "BKE_context.hh"
#include "BKE_crazyspace.hh"
#include "BKE_curves.hh"
#include "BKE_deform.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_grease_pencil_vertex_groups.hh"
#include "BKE_modifier.hh"
#include "BKE_scene.hh"

#include "DEG_depsgraph_query.hh"

#include "DNA_meshdata_types.h"

#include "ED_grease_pencil.hh"
#include "ED_view3d.hh"

#include "grease_pencil_intern.hh"

namespace blender::ed::sculpt_paint::greasepencil {

static constexpr int POINT_CACHE_CHUNK = 1024;
static constexpr float FIND_NEAREST_POINT_EPSILON = 1e-6f;
static constexpr int BLUR_NEIGHBOUR_NUM = 5;
static constexpr int SMEAR_NEIGHBOUR_NUM = 8;

struct BrushPoint {
  float influence;
  int drawing_point_index;
};

struct DrawingWeightData {
  int active_vertex_group;
  int vertex_group_num;
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

class WeightPaintOperation : public GreasePencilStrokeOperation {
 private:
  Brush *brush;
  float brush_radius;
  float brush_strength;
  float brush_weight;
  float2 mouse_position;
  float2 mouse_position_previous;
  /* Brush direction (angle) during a stroke movement. */
  float2 brush_direction;
  bool brush_direction_is_set = false;
  /* For Blur and Smear we build a KDtree for finding nearest points. */
  bool use_find_nearest;
  /* Flag for Auto-normalize weights of bone deformed vertex groups. */
  bool auto_normalize;
  /* Add or subtract weight (Draw tool). */
  bool invert_brush_weight;

  /* Collected points under the brush during the entire brush stroke. Used for finding nearest
   * points for Smear and Blur. Stored per frame group. */
  Array<PointsInBrushStroke> points_in_stroke;

  /* The number of points stored in the stroke point buffers. Per frame group.
   * Note: we can't use Array or Vector here, because it doesn't support atomic types. */
  std::vector<std::atomic<int>> points_in_stroke_num;

  /* Weight paint data per editable drawing. Stored per frame group. */
  Array<Array<DrawingWeightData>> drawing_weight_data;

  /* Mapping of drawing to index in #drawing_weight_data. */
  std::map<bke::greasepencil::Drawing *, int> mapping_of_drawing_to_weight_data;

  /** Get the direction of the brush while the mouse is moving. The direction is given as a
   * normalized XY vector. */
  bool get_brush_direction()
  {
    this->brush_direction = this->mouse_position - this->mouse_position_previous;

    /* Skip tiny changes in direction, we want the bigger movements only. */
    if (math::length_squared(this->brush_direction) < 9.0f) {
      return this->brush_direction_is_set;
    }

    this->brush_direction = math::normalize(this->brush_direction);
    this->brush_direction_is_set = true;
    this->mouse_position_previous = this->mouse_position;

    return true;
  }

  /** Ensure there is enough space in the buffer with brush stroke points to add a new point. */
  void ensure_stroke_buffer_size(const int frame_group, std::mutex &mutex)
  {
    /* We take a little margin here, to be thread safe. */
    PointsInBrushStroke &buffer = this->points_in_stroke[frame_group];
    if (this->points_in_stroke_num[frame_group] >= buffer.nearest_points_size - 32) {
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
  }

  /** Get the average weight of all points in the brush buffer. */
  float get_average_weight_in_brush_buffer(const Array<DrawingWeightData> &drawing_weights)
  {
    float average_sum = 0.0f;
    float point_num = 0;
    for (const DrawingWeightData &drawing_weight : drawing_weights) {
      for (const BrushPoint &point : drawing_weight.points_in_brush) {
        average_sum += drawing_weight.deform_weights[point.drawing_point_index];
        point_num++;
      }
    }

    if (point_num == 0) {
      return 0.0f;
    }
    return math::clamp(average_sum / point_num, 0.0f, 1.0f);
  }

  /** Apply a weight to a point under the brush. */
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

  /** Apply the Blur tool to a point under the brush. */
  void apply_blur_tool(const BrushPoint &point,
                       DrawingWeightData &drawing_weight,
                       PointsInBrushStroke &points_in_stroke)
  {
    /* Find the nearest neighbours of the to-be-blurred point. The point itself is included. */
    KDTreeNearest_2d nearest_points[BLUR_NEIGHBOUR_NUM];
    const int point_num = BLI_kdtree_2d_find_nearest_n(
        points_in_stroke.nearest_points,
        drawing_weight.point_positions[point.drawing_point_index],
        nearest_points,
        BLUR_NEIGHBOUR_NUM);

    if (point_num <= 1) {
      return;
    }

    /* Calculate the blurred weight for the point (A). For this we use a weighted average of the
     * point weights, based on the distance of the neighbour point to A. So points closer to A
     * contribute more to the average than points farther away from A. */
    float distance_sum = 0.0f;
    for (int i = 0; i < point_num; i++) {
      distance_sum += nearest_points[i].dist;
    }
    if (distance_sum == 0.0f) {
      return;
    }
    float blur_weight_sum = 0.0f;
    for (int i = 0; i < point_num; i++) {
      blur_weight_sum += (1.0f - nearest_points[i].dist / distance_sum) *
                         points_in_stroke.nearest_points_weights[nearest_points[i].index];
    }
    const float blur_weight = blur_weight_sum / point_num;

    apply_weight_to_point(point, blur_weight, drawing_weight);
  }

  /** Apply to Smear tool to a point under the brush. */
  void apply_smear_tool(const BrushPoint &point,
                        DrawingWeightData &drawing_weight,
                        PointsInBrushStroke &points_in_stroke)
  {
    /* Find the nearest neighbours of the to-be-smeared point. */
    KDTreeNearest_2d nearest_points[SMEAR_NEIGHBOUR_NUM];
    const int point_num = BLI_kdtree_2d_find_nearest_n(
        points_in_stroke.nearest_points,
        drawing_weight.point_positions[point.drawing_point_index],
        nearest_points,
        SMEAR_NEIGHBOUR_NUM);

    /* For smearing a weight to point A, we look for a point B in the trail of the mouse movement,
     * matching the last known brush angle best and with the shortest distance to A. */
    float point_dot_product[SMEAR_NEIGHBOUR_NUM];
    float min_distance = FLT_MAX, max_distance = -FLT_MAX;
    int smear_point_num = 0;
    for (int i = 0; i < point_num; i++) {
      /* Skip the point we are about to smear. */
      if (nearest_points[i].dist < FIND_NEAREST_POINT_EPSILON) {
        continue;
      }
      const float2 direction_nearest_to_point = math::normalize(
          drawing_weight.point_positions[point.drawing_point_index] -
          points_in_stroke.nearest_points_positions[nearest_points[i].index]);

      /* Match point direction with brush direction. */
      point_dot_product[i] = math::dot(direction_nearest_to_point, this->brush_direction);
      if (point_dot_product[i] <= 0.0f) {
        continue;
      }
      smear_point_num++;
      min_distance = math::min(min_distance, nearest_points[i].dist);
      max_distance = math::max(max_distance, nearest_points[i].dist);
    }
    if (smear_point_num == 0) {
      return;
    }

    /* Find best match in angle and distance. */
    int best_match = -1;
    float max_score = 0.0f;
    const float distance_normalizer = (min_distance == max_distance) ?
                                          1.0f :
                                          (0.95f / (max_distance - min_distance));
    for (int i = 0; i < point_num; i++) {
      if (point_dot_product[i] <= 0.0f) {
        continue;
      }
      const float score = point_dot_product[i] *
                          (1.0f - (nearest_points[i].dist - min_distance) * distance_normalizer);
      if (score > max_score) {
        max_score = score;
        best_match = i;
      }
    }
    if (best_match == -1) {
      return;
    }
    const float smear_weight =
        points_in_stroke.nearest_points_weights[nearest_points[best_match].index];

    apply_weight_to_point(point, smear_weight, drawing_weight);
  }

 public:
  ~WeightPaintOperation() override {}

  void on_stroke_begin(const bContext &C, const InputSample &start_sample)
  {
    using namespace blender::ed::greasepencil;

    const Scene *scene = CTX_data_scene(&C);
    Depsgraph *depsgraph = CTX_data_depsgraph_pointer(&C);
    const RegionView3D *rv3d = CTX_wm_region_view3d(&C);
    const ARegion *region = CTX_wm_region(&C);
    Object *object = CTX_data_active_object(&C);
    const Object *ob_eval = DEG_get_evaluated_object(depsgraph, object);
    float4x4 projection = ED_view3d_ob_project_mat_get(rv3d, object);

    const ToolSettings *ts = CTX_data_tool_settings(&C);
    Paint *paint = BKE_paint_get_active_from_context(&C);
    Brush *brush = BKE_paint_brush(paint);
    GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object->data);

    this->brush = brush;
    this->brush_radius = BKE_brush_size_get(scene, brush);
    this->brush_strength = BKE_brush_alpha_get(scene, brush);
    this->brush_weight = BKE_brush_weight_get(scene, brush);
    this->use_find_nearest = ELEM(brush->weightpaint_tool, WPAINT_TOOL_BLUR, WPAINT_TOOL_SMEAR);
    this->mouse_position_previous = start_sample.mouse_position;
    this->brush_direction_is_set = false;

    BKE_curvemapping_init(brush->curve);

    /* Get the add/subtract mode of the draw tool. */
    this->invert_brush_weight = false;
    if (this->brush->weightpaint_tool == WPAINT_TOOL_DRAW) {
      this->invert_brush_weight = (this->brush->flag & BRUSH_DIR_IN) != 0;
      if (this->brush_mode == BRUSH_STROKE_INVERT) {
        this->invert_brush_weight = !this->invert_brush_weight;
      }
    }

    /* Auto-normalize weights is only applied when the object is deformed by an armature. */
    this->auto_normalize = ts->auto_normalize &&
                           (BKE_modifiers_is_deformed_by_armature(object) != nullptr);

    /* Get or create active vertex group in object. */
    int object_defgroup_nr = BKE_object_defgroup_active_index_get(object) - 1;
    if (object_defgroup_nr == -1) {
      object_defgroup_nr = create_vertex_group_in_object(object);
    }
    const bDeformGroup *object_defgroup = static_cast<const bDeformGroup *>(
        BLI_findlink(BKE_object_defgroup_list(object), object_defgroup_nr));

    /* Get a set of locked and bone deformed vertex groups. These are used for auto-normalizing
     * weights. */
    Set<std::string> object_bone_deformed_defgroups;
    Set<std::string> object_locked_defgroups;
    if (this->auto_normalize) {
      const ListBase *defgroups = BKE_object_defgroup_list(object);
      LISTBASE_FOREACH (bDeformGroup *, dg, defgroups) {
        if ((dg->flag & DG_LOCK_WEIGHT) != 0) {
          object_locked_defgroups.add(dg->name);
        }
      }
      object_bone_deformed_defgroups = get_bone_deformed_vertex_groups(*object);
    }

    /* Get editable drawings grouped per frame number. When multiframe editing is disabled, this is
     * one group for the current frame. When multiframe editing is enabled, the selected keyframes
     * are grouped per frame number. */
    Vector<Vector<MutableDrawingInfo>> drawings_per_frame = retrieve_editable_drawings_per_frame(
        *scene, grease_pencil);
    this->points_in_stroke = Array<PointsInBrushStroke>(drawings_per_frame.size());
    this->points_in_stroke_num = std::vector<std::atomic<int>>(drawings_per_frame.size());
    this->drawing_weight_data = Array<Array<DrawingWeightData>>(drawings_per_frame.size());

    for (const int frame_group : drawings_per_frame.index_range()) {
      /* Create a buffer for points under the brush during the brush stroke. */
      const Vector<MutableDrawingInfo> &drawings = drawings_per_frame[frame_group];
      if (this->use_find_nearest) {
        PointsInBrushStroke &points_in_stroke = this->points_in_stroke[frame_group];
        this->points_in_stroke_num[frame_group] = 0;
        points_in_stroke.nearest_points_size = POINT_CACHE_CHUNK;
        points_in_stroke.nearest_points_positions = Vector<float2>(POINT_CACHE_CHUNK);
        points_in_stroke.nearest_points_weights = Vector<float>(POINT_CACHE_CHUNK);
        points_in_stroke.nearest_points = BLI_kdtree_2d_new(POINT_CACHE_CHUNK);
      }

      /* For each drawing, retrieve pointers to the vertex weight data of the active vertex group,
       * so that we can read and write to them later. And create buffers for points under the brush
       * during one #on_stroke_extended action. */
      this->drawing_weight_data[frame_group] = Array<DrawingWeightData>(drawings.size());

      threading::parallel_for(drawings.index_range(), 1, [&](const IndexRange range) {
        for (const int drawing_index : range) {
          const MutableDrawingInfo &drawing_info = drawings[drawing_index];
          bke::CurvesGeometry &curves = drawing_info.drawing.strokes_for_write();

          /* Find or create the active vertex group in the drawing. */
          DrawingWeightData &drawing_weight_data =
              this->drawing_weight_data[frame_group][drawing_index];
          drawing_weight_data.active_vertex_group = bke::greasepencil::ensure_vertex_group(
              object_defgroup->name, curves.vertex_group_names);

          drawing_weight_data.vertex_group_num = BLI_listbase_count(&curves.vertex_group_names);
          drawing_weight_data.multi_frame_falloff = drawing_info.multi_frame_falloff;
          drawing_weight_data.deform_verts = curves.deform_verts_for_write();
          drawing_weight_data.deform_weights = bke::varray_for_mutable_deform_verts(
              drawing_weight_data.deform_verts, drawing_weight_data.active_vertex_group);

          /* Create boolean arrays indicating whether a vertex group is locked/bone deformed
           * or not. */
          LISTBASE_FOREACH (bDeformGroup *, dg, &curves.vertex_group_names) {
            drawing_weight_data.locked_vgroups.append(object_locked_defgroups.contains(dg->name));
            drawing_weight_data.bone_deformed_vgroups.append(
                object_bone_deformed_defgroups.contains(dg->name));
          }

          /* Convert stroke points to screen space positions. */
          bke::crazyspace::GeometryDeformation deformation =
              bke::crazyspace::get_evaluated_grease_pencil_drawing_deformation(
                  ob_eval, *object, drawing_info.layer_index, drawing_info.frame_number);
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
  }

  void on_stroke_extended(const bContext &C, const InputSample &extension_sample)
  {
    using namespace blender::ed::greasepencil;
    Object *object = CTX_data_active_object(&C);

    /* Get the mouse input data. */
    this->mouse_position = extension_sample.mouse_position;
    float brush_radius = this->brush_radius;
    if (BKE_brush_use_size_pressure(this->brush)) {
      brush_radius *= extension_sample.pressure;
    }
    float brush_influence = this->brush_strength;
    if (BKE_brush_use_alpha_pressure(this->brush)) {
      brush_influence *= extension_sample.pressure;
    }

    /* For the Blur tool, look a bit wider than the brush itself,
     * because we need the weight of surrounding points to perform the blur. */
    const bool use_widened_brush = (this->brush->weightpaint_tool == WPAINT_TOOL_BLUR);
    const float brush_radius_wide = (use_widened_brush ? brush_radius : brush_radius * 1.3f);

    /* For the Smear tool, we use the direction of the brush during the stroke movement. The
     * direction is derived from the current and previous mouse position. */
    if (this->brush->weightpaint_tool == WPAINT_TOOL_SMEAR && !this->get_brush_direction()) {
      /* Abort smear when no direction is established yet. */
      return;
    }

    rctf brush_bbox;
    BLI_rctf_init(&brush_bbox,
                  this->mouse_position.x - brush_radius_wide,
                  this->mouse_position.x + brush_radius_wide,
                  this->mouse_position.y - brush_radius_wide,
                  this->mouse_position.y + brush_radius_wide);

    /* Iterate over the drawings grouped per frame number. Collect all stroke points under the
     * brush and apply the weight tool on them. */
    std::atomic<bool> changed = false;
    std::mutex mutex;
    threading::parallel_for(
        this->drawing_weight_data.index_range(), 1, [&](const IndexRange frame_group_range) {
          for (const int frame_group : frame_group_range) {
            Array<DrawingWeightData> &drawing_weights = this->drawing_weight_data[frame_group];
            std::atomic<bool> balance_kdtree = false;

            /* Collect all stroke points under the brush in a buffer. We use a buffer, because for
             * most weight tools we need the position and weight of all points under the brush to
             * perform the weight action. */
            threading::parallel_for(
                drawing_weights.index_range(), 1, [&](const IndexRange drawing_range) {
                  for (const int drawing_index : drawing_range) {
                    DrawingWeightData &drawing_weight = drawing_weights[drawing_index];
                    for (const int point_index : drawing_weight.point_positions.index_range()) {
                      const float2 &co = drawing_weight.point_positions[point_index];
                      /* For performance, do a brush bounding box check first. */
                      if (!BLI_rctf_isect_pt_v(&brush_bbox, co)) {
                        continue;
                      }
                      const float dist_point_to_brush_center = math::distance(
                          co, this->mouse_position);
                      if (dist_point_to_brush_center > brush_radius_wide) {
                        continue;
                      }

                      /* When the point is under the brush, add it to brush buffer. */
                      if (!use_widened_brush || dist_point_to_brush_center <= brush_radius) {
                        drawing_weight.points_in_brush.append(
                            {drawing_weight.multi_frame_falloff * brush_influence *
                                 BKE_brush_curve_strength(
                                     this->brush, dist_point_to_brush_center, brush_radius),
                             point_index});
                      }

                      /* When the point is under the widened brush, add it to the stroke buffer. */
                      if (this->use_find_nearest) {
                        PointsInBrushStroke &points_in_stroke =
                            this->points_in_stroke[frame_group];

                        /* Add the point only once (no duplicates). */
                        if (drawing_weight.point_index_in_kdtree[point_index] == -1) {
                          this->ensure_stroke_buffer_size(frame_group, mutex);
                          const int buffer_index = (this->points_in_stroke_num[frame_group]++);
                          points_in_stroke.nearest_points_positions[buffer_index] = co;
                          points_in_stroke.nearest_points_weights[buffer_index] =
                              drawing_weight.deform_weights[point_index];
                          BLI_kdtree_2d_insert(points_in_stroke.nearest_points, buffer_index, co);
                          drawing_weight.point_index_in_kdtree[point_index] = buffer_index;
                          balance_kdtree = true;
                        }
                        else {
                          /* When the point was already in the buffer, update the weight. */
                          points_in_stroke.nearest_points_weights
                              [drawing_weight.point_index_in_kdtree[point_index]] =
                              drawing_weight.deform_weights[point_index];
                        }
                      }
                    }
                  }
                });

            /* Buffer calculations:
             * - For the Average tool, get the average weight of the points in the brush buffer.
             * - For Blur and Smear, balance the KDtree for finding nearest points.
             */
            float average_weight_in_brush_buffer;
            if (this->brush->weightpaint_tool == WPAINT_TOOL_AVERAGE) {
              average_weight_in_brush_buffer = this->get_average_weight_in_brush_buffer(
                  drawing_weights);
            }
            if (this->use_find_nearest && balance_kdtree) {
              BLI_kdtree_2d_balance(this->points_in_stroke[frame_group].nearest_points);
            }

            /* Apply the weight tool to all points in the brush buffer. */
            threading::parallel_for(
                drawing_weights.index_range(), 1, [&](const IndexRange drawing_range) {
                  for (const int drawing_index : drawing_range) {
                    DrawingWeightData &drawing_weight = drawing_weights[drawing_index];

                    for (const BrushPoint &point : drawing_weight.points_in_brush) {
                      switch (this->brush->weightpaint_tool) {
                        case WPAINT_TOOL_DRAW:
                          this->apply_weight_to_point(point, this->brush_weight, drawing_weight);
                          break;
                        case WPAINT_TOOL_AVERAGE:
                          this->apply_weight_to_point(
                              point, average_weight_in_brush_buffer, drawing_weight);
                          break;
                        case WPAINT_TOOL_BLUR:
                          this->apply_blur_tool(
                              point, drawing_weight, this->points_in_stroke[frame_group]);
                          break;
                        case WPAINT_TOOL_SMEAR:
                          this->apply_smear_tool(
                              point, drawing_weight, this->points_in_stroke[frame_group]);
                          break;
                        default:
                          BLI_assert_unreachable();
                          break;
                      }

                      /* Normalize weights of bone-deformed vertex groups to 1.0f. */
                      if (this->auto_normalize) {
                        normalize_vertex_weights(
                            drawing_weight.deform_verts[point.drawing_point_index],
                            drawing_weight.active_vertex_group,
                            drawing_weight.locked_vgroups,
                            drawing_weight.bone_deformed_vgroups);
                      }
                    }

                    if (!drawing_weight.points_in_brush.is_empty()) {
                      changed = true;
                      drawing_weight.points_in_brush.clear();
                    }
                  }
                });
          }
        });

    if (changed) {
      GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object->data);
      DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY);
      WM_event_add_notifier(&C, NC_GEOM | ND_DATA, &grease_pencil);
    }
  }

  void on_stroke_done(const bContext & /*C*/)
  {
    /* Clean up KDtrees. */
    if (this->use_find_nearest) {
      for (const PointsInBrushStroke &point_buffer : this->points_in_stroke) {
        BLI_kdtree_2d_free(point_buffer.nearest_points);
      }
    }
  }
};

std::unique_ptr<GreasePencilStrokeOperation> new_weight_paint_operation()
{
  return std::make_unique<WeightPaintOperation>();
}

}  // namespace blender::ed::sculpt_paint::greasepencil
