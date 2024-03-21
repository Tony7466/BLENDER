/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgreasepencil
 */

#pragma once

/* DEBUG: show developer extra's in viewport and console. */
/* #define GP_FILL_DEBUG_MODE */

#include "BKE_context.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_material.h"
#include "BKE_paint.hh"
#include "BKE_report.hh"

#include "BLI_kdtree.h"
#include "BLI_threads.h"

#include "ANIM_keyframing.hh"

#include "DEG_depsgraph_query.hh"

#include "ED_curves.hh"
#include "ED_grease_pencil.hh"
#include "ED_screen.hh"
#include "ED_space_api.hh"
#include "ED_view3d.hh"

#include "GPU_framebuffer.h"
#include "GPU_immediate.h"
#include "GPU_matrix.h"
#include "GPU_state.h"

#include "WM_api.hh"

#include "grease_pencil_fill_flood.hh"
#include "grease_pencil_fill_geometry.hh"

#ifdef GP_FILL_DEBUG_MODE
#  include "BLF_api.hh"
#  include "UI_interface.hh"
#  include "UI_resources.hh"
#endif

namespace blender::ed::greasepencil::fill {

void draw_overlay(const bContext * /*C*/, ARegion *region, void *arg);

/**
 * Operator handles for flood based fill.
 */
void flood_fill_exit(const wmOperator &op);
bool flood_fill_exec(const wmOperator &op);
int flood_fill_modal(const wmOperator &op, const wmEvent &event);
bool flood_fill_invoke(bContext *C, wmOperator *op);

/**
 * Operator handles for geometry based fill.
 */
void geometry_fill_exit(const wmOperator &op);
bool geometry_fill_exec(const wmOperator &op);
int geometry_fill_modal(const wmOperator &op, const wmEvent &event);
bool geometry_fill_invoke(bContext *C, wmOperator *op);

/* Number of nearest neighbours when looking for gap closure by curve end radius. */
static constexpr unsigned int RADIUS_NEAREST_NUM = 12;
/* Number of pixels the gap closure size stands for. */
static constexpr float GAP_PIXEL_FACTOR = 25.0f;
/* Margin for intersection distances to be considered equal. */
static constexpr float DISTANCE_EPSILON = 0.001f;
static constexpr float DISTANCE_SORT_EPSILON = 0.02f;
/* Margin for vector cross product considered to be parallel. */
static constexpr float PARALLEL_EPSILON = 0.01f;

/* Base fill operation class, with data members and methods for gap closure and conversion of
 * curves to 2D screen space. */
class FillOperation {
 public:
  /* Active Grease Pencil object. */
  GreasePencil *grease_pencil;
  /* View context data.*/
  ViewContext vc;
  /* Fill brush. */
  Brush *brush;
  /* Additive drawing (from tool settings). */
  bool additive_drawing;

  /* For modal event handling: wait for key release before processing the key event again. */
  bool wait_for_release{};
  /* Wait-for-release event key. */
  short wait_event_type{};

  /* Mouse position of the fill operation click. */
  float2 mouse_pos{};

  /* Frame number to perform the fill operation on. */
  int frame_number;

  /* True when edge gaps are closed by extending the curve ends. */
  bool use_gap_close_extend{};
  /* True when edge gaps are closed by curve end radii. */
  bool use_gap_close_radius{};
  /* True when curve end extensions stop at their first intersection. */
  bool extensions_stop_at_first_intersection{};
  /* Gap closure distance in pixels. */
  float gap_distance{};
  /* True when edge gaps are closed by curve proximity. */
  bool use_gap_close_proximity{};
  /* Curve proximity distance in pixels. */
  float proximity_distance;
  float proximity_distance_squared;

  /* Colors of gap closure visual aids. */
  float gap_closure_color[3];
  float gap_closed_color[3];
  float gap_proximity_color[4];

  /* Draw handle for 3D viewport overlay. */
  void *draw_handle = nullptr;

  /* Curve points converted to viewport 2D space. */
  Curves2DSpace curves_2d{};
  /* Curve end extensions in viewport 2D space. */
  Curves2DSpace extensions_2d{};
  /* Flag for curve end extensions, true when the extension intersects a curve or other
   * end extension. */
  Array<bool> extension_has_intersection;
  /* Intersections of curve end extensions with curves or other extensions. */
  Vector<IntersectingCurve> extension_intersections;
  /* Slight normalized extension of all curve segments (epsilon), used to avoid floating point
   * precision errors when looking for intersections. */
  Array<float2> curve_segment_epsilon;
  /* Ratio between length of extension and length of curve end segment. */
  Array<float> extension_length_ratio;
  /* KD tree of curve ends in 2D space, used for gap closure by radius. */
  KDTree_2d *curve_ends = nullptr;
  /* Flag indicating a curve end is connected by radius with one or more other curve ends. */
  Array<bool> connected_by_radius;
  /* Curve end connections (by radius) with other curve ends. */
  Vector<int2> radius_connections;

  /**
   * Get the normalized distance from v1 and v3 to the intersection point of line segments v1-v2
   * and v3-v4.
   */
  float2 get_intersection_distance_normalized(const float2 &v1,
                                              const float2 &v2,
                                              const float2 &v3,
                                              const float2 &v4)
  {
    float2 distance = {0.0f, 0.0f};

    /* Get intersection point. */
    const float a1 = v2[1] - v1[1];
    const float b1 = v1[0] - v2[0];
    const float c1 = a1 * v1[0] + b1 * v1[1];

    const float a2 = v4[1] - v3[1];
    const float b2 = v3[0] - v4[0];
    const float c2 = a2 * v3[0] + b2 * v3[1];

    const float det = a1 * b2 - a2 * b1;
    BLI_assert(det != 0.0f);

    float2 isect;
    isect[0] = (b2 * c1 - b1 * c2) / det;
    isect[1] = (a1 * c2 - a2 * c1) / det;

    /* Get normalized distance from v1 and v3 to intersection point. */
    const float v1_length = math::length(v2 - v1);
    const float v3_length = math::length(v4 - v3);

    distance[0] = (v1_length == 0.0f ?
                       0.0f :
                       math::clamp(math::length(isect - v1) / v1_length, 0.0f, 1.0f));
    distance[1] = (v3_length == 0.0f ?
                       0.0f :
                       math::clamp(math::length(isect - v3) / v3_length, 0.0f, 1.0f));
    return distance;
  }

  virtual void store_overlapping_segment(
      const int, const int, const bool, const int, const int) = 0;

  /**
   * Get all intersections of a segment with other curves.
   */
  Vector<IntersectingCurve> get_intersections_of_segment_with_curves(
      const float2 &segment_a,
      const float2 &segment_b,
      const int segment_curve_index,
      const Curves2DSpace *curves_2d,
      const float2 &adj_a = {FLT_MAX, FLT_MAX},
      const float2 &adj_b = {FLT_MAX, FLT_MAX},
      const bool store_overlap = false,
      const int segment_point_a = 0,
      const int segment_point_b = 0,
      const int segment_direction = 1,
      const bool use_epsilon = false)
  {
    /* Inits. */
    Vector<IntersectingCurve> intersections;
    ThreadMutex mutex = BLI_MUTEX_INITIALIZER;

    const float2 segment_vec = segment_b - segment_a;

    float2 segment_epsilon = {0.0f, 0.0f};
    if (use_epsilon) {
      if (segment_direction == 1) {
        const int epsilon_i = curves_2d->point_offset[segment_curve_index] + segment_point_a;
        segment_epsilon = -this->curve_segment_epsilon[epsilon_i];
      }
      else {
        const int epsilon_i = curves_2d->point_offset[segment_curve_index] + segment_point_b;
        segment_epsilon = this->curve_segment_epsilon[epsilon_i];
      }
    }

    /* Loop all curves, looking for intersecting segments. */
    threading::parallel_for(
        curves_2d->point_offset.index_range(), 256, [&](const IndexRange range) {
          rctf bbox_segment, bbox_isect;

          for (const int curve_i : range) {
            /* Only process curves with at least two points. */
            if (curves_2d->point_size[curve_i] < 2) {
              continue;
            }

            /* Create a slightly extended segment to avoid floating point precision errors.
             * Otherwise an intersection can be missed when it is on the outer end of a segment. */
            float2 seg_a_extended = segment_a;
            float2 seg_b_extended = segment_b;
            if (use_epsilon && curve_i != segment_curve_index) {
              seg_a_extended += segment_epsilon;
              seg_b_extended -= segment_epsilon;
            }

            /* Create bounding box around the segment. */
            BLI_rctf_init_minmax(&bbox_segment);
            BLI_rctf_do_minmax_v(&bbox_segment, seg_a_extended);
            BLI_rctf_do_minmax_v(&bbox_segment, seg_b_extended);

            /* Do a quick bounding box check. When the bounding box of a curve doesn't
             * intersect with the segment, none of the curve segments do. */
            if (!BLI_rctf_isect(&bbox_segment, &curves_2d->curve_bbox[curve_i], nullptr)) {
              continue;
            }

            /* Get point range. */
            const int point_offset = curves_2d->point_offset[curve_i];
            const int point_size = curves_2d->point_size[curve_i] -
                                   (curves_2d->is_cyclic[curve_i] ? 0 : 1);
            const int point_last = point_offset + curves_2d->point_size[curve_i] - 1;

            /* Skip curves with identical (overlapping) segments, because they produce
             * false-positive intersections. Overlapping curves are most likely created by a
             * previous fill operation. */
            if (curve_i != segment_curve_index) {
              bool skip_curve = false;
              for (const int point_i : IndexRange(point_offset, point_size)) {
                const int point_i_next = (point_i == point_last ? point_offset : point_i + 1);
                const float2 p0 = curves_2d->points_2d[point_i];
                const float2 p1 = curves_2d->points_2d[point_i_next];

                /* Check for identical segments. */
                if (segment_a == p0 && segment_b == p1) {
                  if (store_overlap) {
                    store_overlapping_segment(
                        curve_i, point_i - point_offset, 1, segment_point_a, segment_direction);
                  }
                  skip_curve = true;
                  break;
                }
                if (segment_a == p1 && segment_b == p0) {
                  if (store_overlap) {
                    store_overlapping_segment(
                        curve_i, point_i - point_offset, -1, segment_point_a, segment_direction);
                  }
                  skip_curve = true;
                  break;
                }
              }
              if (skip_curve) {
                continue;
              }
            }

            /* Find intersecting curve segments. */
            int prev_intersection_point = -1;
            for (const int point_i : IndexRange(point_offset, point_size)) {
              const int point_i_next = (point_i == point_last ? point_offset : point_i + 1);
              const float2 p0 = curves_2d->points_2d[point_i];
              const float2 p1 = curves_2d->points_2d[point_i_next];
              float2 p0_extended = p0;
              float2 p1_extended = p1;
              if (use_epsilon && curve_i != segment_curve_index) {
                p0_extended -= this->curve_segment_epsilon[point_i];
                p1_extended += this->curve_segment_epsilon[point_i];
              }

              /* Skip when previous segment was intersecting. */
              if (prev_intersection_point == point_i) {
                continue;
              }

              /* Skip when bounding boxes don't overlap. */
              BLI_rctf_init(
                  &bbox_isect, p0_extended[0], p0_extended[0], p0_extended[1], p0_extended[1]);
              BLI_rctf_do_minmax_v(&bbox_isect, p1_extended);
              if (!BLI_rctf_isect(&bbox_segment, &bbox_isect, nullptr)) {
                continue;
              }

              /* Don't self check. */
              if (curve_i == segment_curve_index) {
                if (segment_a == p0 || segment_a == p1 || segment_b == p0 || segment_b == p1) {
                  continue;
                }
              }

              /* Skip identical adjacent segments. */
              if ((adj_a == p0 && segment_a == p1) || (segment_b == p0 && adj_b == p1) ||
                  (adj_b == p0 && segment_b == p1) || (segment_a == p0 && adj_a == p1))
              {
                continue;
              }

              /* Skip segments that exactly overlap the current one. */
              if (segment_a == p0 || segment_a == p1 || segment_b == p0 || segment_b == p1) {
                if (compare_ff(cross_v2v2(p1 - p0, segment_vec), 0.0f, PARALLEL_EPSILON)) {
                  continue;
                }
              }

              /* Check for intersection. */
              auto isect = math::isect_seg_seg(
                  seg_a_extended, seg_b_extended, p0_extended, p1_extended);
              if (ELEM(isect.kind, isect.LINE_LINE_CROSS, isect.LINE_LINE_EXACT)) {
                IntersectingCurve intersection{};
                intersection.curve_index_2d = curve_i;
                intersection.point_start = point_i - point_offset;
                intersection.point_end = point_i_next - point_offset;
                intersection.distance = get_intersection_distance_normalized(
                    segment_a, segment_b, p0, p1);
                intersection.has_stroke = curves_2d->has_stroke[curve_i];

                BLI_mutex_lock(&mutex);
                intersections.append(intersection);
                BLI_mutex_unlock(&mutex);

                prev_intersection_point = point_i_next;
              }
            }
          }
        });

    return intersections;
  }

  /**
   * Create a KD tree for the gap closure of curve ends by radius.
   */
  void init_curve_end_radii()
  {
    /* Create KD tree for all curve ends. */
    const int curve_num = this->curves_2d.point_offset.size();
    this->curve_ends = BLI_kdtree_2d_new(curve_num * 2);

    /* Add curve ends. */
    int tree_index = 0;
    for (const int curve_i : this->curves_2d.point_offset.index_range()) {
      /* Add first curve point. */
      int point_i = this->curves_2d.point_offset[curve_i];
      BLI_kdtree_2d_insert(this->curve_ends, tree_index, this->curves_2d.points_2d[point_i]);
      tree_index++;

      /* Add last curve point. */
      point_i += this->curves_2d.point_size[curve_i] - 1;
      BLI_kdtree_2d_insert(this->curve_ends, tree_index, this->curves_2d.points_2d[point_i]);
      tree_index++;
    }

    BLI_kdtree_2d_balance(this->curve_ends);

    /* Init array with curve end connection flag. */
    this->connected_by_radius = Array<bool>(curve_num * 2, false);
  }

  /**
   * Determine which curves ends are connected with each other based on radius.
   */
  void get_connected_curve_end_radii()
  {
    const float max_dist = 2 * this->gap_distance;

    /* Init connections by radius. */
    this->connected_by_radius.fill(false);
    this->radius_connections.clear();

    /* Check all curvess. */
    for (const int curve_i : this->curves_2d.point_offset.index_range()) {
      /* Skip cyclic curves. */
      if (this->curves_2d.is_cyclic[curve_i]) {
        continue;
      }

      /* Check both ends. */
      for (int side = 0; side <= 1; side++) {
        const int kdtree_index = curve_i * 2 + side;
        const int point_index = this->curves_2d.point_offset[curve_i] +
                                (side == 0 ? 0 : this->curves_2d.point_size[curve_i] - 1);

        /* Find nearest curve end points. */
        KDTreeNearest_2d nearest[RADIUS_NEAREST_NUM];
        const int nearest_num = BLI_kdtree_2d_find_nearest_n(
            this->curve_ends, this->curves_2d.points_2d[point_index], nearest, RADIUS_NEAREST_NUM);

        for (int i = 0; i < nearest_num; i++) {
          /* Skip self, already registered curve ends and curve ends out of close radius range. */
          if (nearest[i].index <= kdtree_index || nearest[i].dist > max_dist) {
            continue;
          }

          /* Skip cyclic curves. */
          const int nearest_curve = int(nearest[i].index / 2);
          if (this->curves_2d.is_cyclic[nearest_curve]) {
            continue;
          }

          /* Flag connection and append to list for drawing. */
          this->connected_by_radius[kdtree_index] = true;
          this->connected_by_radius[nearest[i].index] = true;
          this->radius_connections.append({kdtree_index, nearest[i].index});
        }
      }
    }
  }

  /**
   * Init the data structure for curve end extensions.
   */
  void init_curve_end_extensions()
  {
    const int curve_num = this->curves_2d.point_offset.size() * 2;
    const int point_num = curve_num * 2;

    this->extensions_2d.point_offset = Array<int>(curve_num);
    this->extensions_2d.point_size = Array<int>(curve_num, 2);
    this->extensions_2d.is_cyclic = Array<bool>(curve_num, false);
    this->extensions_2d.has_stroke = Array<bool>(curve_num, false);
    this->extensions_2d.drawing_index_2d = Array<int>(curve_num);
    this->extensions_2d.points_2d = Array<float2>(point_num);
    this->extensions_2d.curve_bbox = Array<rctf>(curve_num);
    this->extension_length_ratio = Array<float>(curve_num);
    this->extension_has_intersection = Array<bool>(curve_num, false);
  }

  /**
   * Add a curve end extension to the set of extensions.
   */
  void add_curve_end_extension(const int curve_i,
                               const int curve_ext,
                               const int point_i,
                               const int next_point_delta,
                               const int point_i_ext)
  {
    /* Set drawing index and point index for extension. */
    this->extensions_2d.drawing_index_2d[curve_ext] = this->curves_2d.drawing_index_2d[curve_i];
    this->extensions_2d.point_offset[curve_ext] = point_i_ext;
    this->extensions_2d.is_cyclic[curve_ext] = this->curves_2d.is_cyclic[curve_i];

    /* Use the vector between the two outer points of the curve to calculate the extension
     * coordinates. */
    const float2 co = this->curves_2d.points_2d[point_i];
    this->extensions_2d.points_2d[point_i_ext] = co;
    float2 end_vec = co - this->curves_2d.points_2d[point_i + next_point_delta];
    const float end_length = math::length(end_vec);
    end_vec = math::normalize(end_vec) * this->gap_distance;
    const float extension_length = math::length(end_vec);
    this->extensions_2d.points_2d[point_i_ext + 1] = co + end_vec;
    this->extension_length_ratio[curve_ext] = (end_length == 0.0f ? 0.0f :
                                                                    extension_length / end_length);
  }

  /**
   * Create a set of 2D curve end extensions. Check if the extensions intersect with curves or
   * other end extensions.
   */
  void get_curve_end_extensions()
  {
    /* Create extension curves. */
    for (const int curve_i : this->curves_2d.point_offset.index_range()) {
      /* Two extensions for each curve. */
      const int curve_ext = curve_i * 2;
      /* Each extension contains two points. */
      const int point_i_ext = curve_ext * 2;
      const int point_size = this->curves_2d.point_size[curve_i];

      /* Create extension for the first segment of the 2D curve. */
      int point_i = this->curves_2d.point_offset[curve_i];
      int next_point_delta = (point_size > 1 ? 1 : 0);
      this->add_curve_end_extension(curve_i, curve_ext, point_i, next_point_delta, point_i_ext);

      /* Create extension for the last segment of the 2D curve. */
      point_i += point_size - 1;
      next_point_delta = (point_size > 1 ? -1 : 0);
      this->add_curve_end_extension(
          curve_i, curve_ext + 1, point_i, next_point_delta, point_i_ext + 2);

      /* Create bounding boxes for the extensions. */
      BLI_rctf_init_minmax(&this->extensions_2d.curve_bbox[curve_ext]);
      BLI_rctf_do_minmax_v(&this->extensions_2d.curve_bbox[curve_ext],
                           this->extensions_2d.points_2d[point_i_ext]);
      BLI_rctf_do_minmax_v(&this->extensions_2d.curve_bbox[curve_ext],
                           this->extensions_2d.points_2d[point_i_ext + 1]);

      BLI_rctf_init_minmax(&this->extensions_2d.curve_bbox[curve_ext + 1]);
      BLI_rctf_do_minmax_v(&this->extensions_2d.curve_bbox[curve_ext + 1],
                           this->extensions_2d.points_2d[point_i_ext + 2]);
      BLI_rctf_do_minmax_v(&this->extensions_2d.curve_bbox[curve_ext + 1],
                           this->extensions_2d.points_2d[point_i_ext + 3]);
    }

    /* Clear intersection data. */
    this->extension_intersections.clear();
    this->extension_has_intersection.fill(false);

    /* Check intersections of extensions with curves or other end extensions. */
    for (const int extension_index : this->extensions_2d.point_offset.index_range()) {
      /* Skip end extensions of cyclic curves. */
      if (this->extensions_2d.is_cyclic[extension_index]) {
        continue;
      }

      const int point_offset = this->extensions_2d.point_offset[extension_index];
      const float2 segment_point_a = this->extensions_2d.points_2d[point_offset];
      const float2 segment_point_b = this->extensions_2d.points_2d[point_offset + 1];

      /* Get intersections with other curve end extensions. */
      Vector<IntersectingCurve> all_intersections;
      Vector<IntersectingCurve> intersections = this->get_intersections_of_segment_with_curves(
          segment_point_a, segment_point_b, extension_index, &this->extensions_2d);

      for (auto intersection : intersections) {
        intersection.with_end_extension = true;
        intersection.extension_index = extension_index;
        all_intersections.append(intersection);
      }

      /* Get intersections with other curves. */
      const int curve_i = int(extension_index / 2);
      intersections = this->get_intersections_of_segment_with_curves(
          segment_point_a, segment_point_b, curve_i, &this->curves_2d);

      for (auto intersection : intersections) {
        intersection.with_end_extension = false;
        intersection.extension_index = extension_index;
        all_intersections.append(intersection);
      }

      /* Sort intersections on distance. */
      if (!all_intersections.is_empty()) {
        std::sort(all_intersections.begin(),
                  all_intersections.end(),
                  [](const IntersectingCurve &a, const IntersectingCurve &b) {
                    return a.distance[0] < b.distance[0];
                  });

        /* Add closest result(s) to list of extension intersections. */
        const float shortest_dist = all_intersections.first().distance[0];
        for (const auto &intersection : all_intersections) {
          if (this->extensions_stop_at_first_intersection &&
              (intersection.distance[0] - shortest_dist) > DISTANCE_SORT_EPSILON)
          {
            break;
          }
          this->extension_intersections.append(intersection);
          this->extension_has_intersection[intersection.extension_index] = true;
        }
      }
    }
  }

  /**
   * Get a slight extension for each curve segment, based on the normalized vector of each
   * curve point to the next.
   */
  void get_curve_segment_epsilons()
  {
    this->curve_segment_epsilon = Array<float2>(this->curves_2d.points_2d.size());

    threading::parallel_for(
        this->curves_2d.point_offset.index_range(), 256, [&](const IndexRange range) {
          for (const int curve_i : range) {
            const int point_last = this->curves_2d.point_offset[curve_i] +
                                   this->curves_2d.point_size[curve_i] - 1;
            for (const int point_i : IndexRange(this->curves_2d.point_offset[curve_i],
                                                this->curves_2d.point_size[curve_i]))
            {
              const int point_next = (point_i == point_last ?
                                          this->curves_2d.point_offset[curve_i] :
                                          point_i + 1);
              this->curve_segment_epsilon[point_i] = math::normalize(
                                                         this->curves_2d.points_2d[point_next] -
                                                         this->curves_2d.points_2d[point_i]) *
                                                     DISTANCE_EPSILON;
            }
          }
        });
  }

#ifdef GP_FILL_DEBUG_MODE
  /**
   * Debug function: show curve indices in the viewport.
   */
  void debug_draw_curve_indices()
  {
    const int font_id = BLF_default();
    const uiStyle *style = UI_style_get();
    BLF_size(font_id, style->widget.points * UI_SCALE_FAC * 0.8);

    /* Draw point indices. */
    BLF_color4fv(font_id, float4{0.0f, 0.0f, 0.0f, 0.5f});
    for (const int curve_i : this->curves_2d.point_offset.index_range()) {
      const int point_offset = this->curves_2d.point_offset[curve_i];
      const int step = 5;
      float x_prev = FLT_MAX;
      float y_prev = FLT_MAX;
      for (int point_i = step; point_i < this->curves_2d.point_size[curve_i]; point_i += step) {
        const std::string str = std::to_string(point_i) + "-" + std::to_string(curve_i);
        const char *text = str.c_str();

        const float x = this->curves_2d.points_2d[point_offset + point_i][0] - strlen(text) * 5.4;
        const float y = this->curves_2d.points_2d[point_offset + point_i][1] + 1;

        if (((x - x_prev) * (x - x_prev) + (y - y_prev) * (y - y_prev)) > 1000) {
          BLF_position(font_id, x, y, 0);
          BLF_draw(font_id, text, strlen(text));

          x_prev = x;
          y_prev = y;
        }
      }
    }

    /* Draw curve indices. */
    BLF_size(font_id, style->widget.points * UI_SCALE_FAC * 1.2);
    BLF_color4fv(font_id, float4{1.0f, 0.0f, 0.2f, 1.0f});
    BLF_enable(font_id, BLF_BOLD);

    for (const int curve_i : this->curves_2d.point_offset.index_range()) {
      const std::string str = std::to_string(curve_i);
      const char *text = str.c_str();

      const int point_offset = this->curves_2d.point_offset[curve_i];
      const float x = this->curves_2d.points_2d[point_offset][0] - strlen(text) * 10;
      const float y = this->curves_2d.points_2d[point_offset][1] + 2;
      BLF_position(font_id, x, y, 0);

      BLF_draw(font_id, text, strlen(text));
    }

    BLF_disable(font_id, BLF_BOLD);
  }
#endif

  /**
   * Get curve point index given an end extension index.
   */
  int get_curve_point_by_end_index(const int end_index)
  {
    const int curve_i = int(end_index / 2);
    int point_i = this->curves_2d.point_offset[curve_i];
    if ((end_index & 1) == 1) {
      point_i += this->curves_2d.point_size[curve_i] - 1;
    }
    return point_i;
  }

  /**
   * Get latest tool settings before executing the actual fill.
   */
  void get_latest_toolsettings()
  {
    const bool use_gap_closing = (this->brush->gpencil_settings->fill_extend_fac > FLT_EPSILON);
    this->use_gap_close_extend = (this->brush->gpencil_settings->fill_extend_mode ==
                                  GP_FILL_EMODE_EXTEND) &&
                                 use_gap_closing;
    this->use_gap_close_radius = (this->brush->gpencil_settings->fill_extend_mode ==
                                  GP_FILL_EMODE_RADIUS) &&
                                 use_gap_closing;
    this->gap_distance = this->brush->gpencil_settings->fill_extend_fac * GAP_PIXEL_FACTOR;
    this->use_gap_close_proximity = (this->brush->gpencil_settings->fill_proximity_distance > 0);
    this->proximity_distance = float(this->brush->gpencil_settings->fill_proximity_distance);
    this->proximity_distance_squared = this->proximity_distance * this->proximity_distance;
  }

  /**
   * Update the gap closure By Proximity in the viewport when the value changed.
   */
  void update_proximity_distance(const int delta)
  {
    this->brush->gpencil_settings->fill_proximity_distance = math::max(
        0, math::min(100, this->brush->gpencil_settings->fill_proximity_distance + delta));
    this->get_latest_toolsettings();
    ED_region_tag_redraw(this->vc.region);
  }

  /**
   * Update the gap closure connections when the radius changed.
   */
  void update_gap_distance(const float delta)
  {
    this->brush->gpencil_settings->fill_extend_fac = math::max(
        0.0f, math::min(10.0f, this->brush->gpencil_settings->fill_extend_fac + delta));
    this->get_latest_toolsettings();

    if (this->use_gap_close_extend) {
      this->get_curve_end_extensions();
    }
    if (this->use_gap_close_radius) {
      this->get_connected_curve_end_radii();
    }
    ED_region_tag_redraw(this->vc.region);
  }

  /**
   * Get a list of layers used for the fill edge detection. The list is based on the 'Layers' field
   * in the tool settings.
   */
  void get_fill_edge_layers(const int frame_number,
                            Vector<const bke::greasepencil::Drawing *> &r_drawings,
                            Vector<int> &r_layer_index)
  {
    /* Find index of active layer. */
    int active_layer_index = -1;
    const bke::greasepencil::Layer *active_layer = this->grease_pencil->get_active_layer();
    Span<const bke::greasepencil::Layer *> layers = this->grease_pencil->layers();
    for (int layer_index = 0; layer_index < layers.size(); layer_index++) {
      if (layers[layer_index] == active_layer) {
        active_layer_index = layer_index;
        break;
      }
    }

    /* Select layers based on position in the layer collection. */
    for (int layer_index = 0; layer_index < layers.size(); layer_index++) {
      /* Skip invisible layers. */
      if (!layers[layer_index]->is_visible()) {
        continue;
      }

      bool add = false;
      switch (this->brush->gpencil_settings->fill_layer_mode) {
        case GP_FILL_GPLMODE_ACTIVE:
          add = (layer_index == active_layer_index);
          break;
        case GP_FILL_GPLMODE_ABOVE:
          add = (layer_index == active_layer_index + 1);
          break;
        case GP_FILL_GPLMODE_BELOW:
          add = (layer_index == active_layer_index - 1);
          break;
        case GP_FILL_GPLMODE_ALL_ABOVE:
          add = (layer_index > active_layer_index);
          break;
        case GP_FILL_GPLMODE_ALL_BELOW:
          add = (layer_index < active_layer_index);
          break;
        case GP_FILL_GPLMODE_VISIBLE:
          add = true;
          break;
      }

      if (!add) {
        continue;
      }
      if (const bke::greasepencil::Drawing *drawing = this->grease_pencil->get_editable_drawing_at(
              *layers[layer_index], frame_number))
      {
        r_drawings.append(drawing);
        r_layer_index.append(layer_index);
      }
    }
  }

  /**
   * Get drawings and gap closure data for a given frame number.
   */
  bool get_drawings_and_gap_closures_at_frame(const int frame_number)
  {
    /* Get layers according to tool settings (visible, above, below, etc.) */
    Vector<const bke::greasepencil::Drawing *> drawings;
    Vector<int> layer_indices;

    this->get_fill_edge_layers(frame_number, drawings, layer_indices);
    if (drawings.is_empty()) {
      return false;
    }

    /* Convert curves to viewport 2D space. */
    this->curves_2d = curves_in_2d_space_get(*this->vc.obact,
                                             this->vc,
                                             *this->grease_pencil,
                                             drawings,
                                             layer_indices,
                                             frame_number,
                                             true);

    /* Calculate epsilon values of all 2D curve segments, used to avoid floating point precision
     * errors. */
    this->get_curve_segment_epsilons();

    /* When using extensions of the curve ends to close gaps, build an array of those
     * two-point 'curves'. */
    if (this->use_gap_close_extend) {
      this->init_curve_end_extensions();
      this->get_curve_end_extensions();
    }

    /* When using radii to close gaps, build KD tree of curve end points. */
    if (this->use_gap_close_radius) {
      this->init_curve_end_radii();
      this->get_connected_curve_end_radii();
    }

    return true;
  }

  /**
   * Initialize the fill operator data.
   */
  bool fill_init(bContext *C)
  {
    /* Get view context. */
    Depsgraph *depsgraph = CTX_data_ensure_evaluated_depsgraph(C);
    this->vc = ED_view3d_viewcontext_init(C, depsgraph);

    /* Get active GP object. */
    this->grease_pencil = static_cast<GreasePencil *>(this->vc.obact->data);

    /* Get tool brush. */
    const ToolSettings *ts = CTX_data_tool_settings(C);
    this->brush = BKE_paint_brush(&ts->gp_paint->paint);
    this->additive_drawing = ((ts->gpencil_flags & GP_TOOL_FLAG_RETAIN_LAST) != 0);

    /* Init vector fill flags. */
    this->use_gap_close_extend = (this->brush->gpencil_settings->fill_extend_mode ==
                                  GP_FILL_EMODE_EXTEND);
    this->use_gap_close_radius = (this->brush->gpencil_settings->fill_extend_mode ==
                                  GP_FILL_EMODE_RADIUS);
    this->extensions_stop_at_first_intersection = (this->brush->gpencil_settings->flag &
                                                   GP_BRUSH_FILL_STROKE_COLLIDE) != 0;
    this->gap_distance = this->brush->gpencil_settings->fill_extend_fac * GAP_PIXEL_FACTOR;
    this->wait_for_release = true;
    this->wait_event_type = LEFTMOUSE;
    this->use_gap_close_proximity = (this->brush->gpencil_settings->fill_proximity_distance > 0);
    this->proximity_distance = float(this->brush->gpencil_settings->fill_proximity_distance);
    this->proximity_distance_squared = this->proximity_distance * this->proximity_distance;

    const float default_gap_closure_color[3] = {1.0f, 0.0f, 0.5f};
    const float default_gap_closed_color[3] = {0.0f, 1.0f, 1.0f};
    const float default_gap_proximity_color[4] = {1.0f, 0.8f, 0.0f, 0.3f};
    copy_v3_v3(this->gap_closure_color, default_gap_closure_color);
    copy_v3_v3(this->gap_closed_color, default_gap_closed_color);
    copy_v4_v4(this->gap_proximity_color, default_gap_proximity_color);

    /* Get drawings for current frame. */
    if (!this->get_drawings_and_gap_closures_at_frame(this->vc.scene->r.cfra)) {
      return false;
    }

    /* Activate 3D viewport overlay for showing gap closure visual aids. */
    if ((this->brush->gpencil_settings->flag & GP_BRUSH_FILL_SHOW_EXTENDLINES) != 0) {
      this->draw_handle = ED_region_draw_cb_activate(
          this->vc.region->type, draw_overlay, this, REGION_DRAW_POST_PIXEL);
      ED_region_tag_redraw(this->vc.region);
    }

    return true;
  }

  /**
   * Clean up the fill operator data.
   */
  void fill_exit()
  {
    /* Remove viewport overlay. */
    ED_region_draw_cb_exit(this->vc.region->type, this->draw_handle);
    ED_region_tag_redraw(this->vc.region);

    /* Remove KD tree of curve ends. */
    if (this->curve_ends) {
      BLI_kdtree_2d_free(this->curve_ends);
    }
  }

  virtual bool execute_fill() = 0;

  /**
   * Execute the fill operation (on second mouse click in the viewport).
   */
  bool fill_exec()
  {
#ifdef GP_FILL_DEBUG_MODE
    /* DEBUG: measure time. */
    auto t1 = std::chrono::high_resolution_clock::now();
#endif  // GP_FILL_DEBUG_MODE

    this->get_latest_toolsettings();

    /* Perform the fill operation for all selected keyframes. */
    const bool use_multi_frame_editing = (this->vc.scene->toolsettings->gpencil_flags &
                                          GP_USE_MULTI_FRAME_EDITING) != 0;
    const Array<int> frame_numbers = get_frame_numbers_for_layer(
        this->grease_pencil->get_active_layer()->wrap(),
        this->vc.scene->r.cfra,
        use_multi_frame_editing);
    bool get_drawings = false;
    bool success = false;

    for (const int frame_number : frame_numbers) {
      /* For the current frame (first entry in the array) we already retrieved the drawings during
       * the operator invoke, so no need to do it a second time. */
      if (get_drawings) {
        if (!this->get_drawings_and_gap_closures_at_frame(frame_number)) {
          continue;
        }
      }
      get_drawings = true;

      this->frame_number = frame_number;

      if (execute_fill()) {
        success = true;
      }
    }

#ifdef GP_FILL_DEBUG_MODE
    /* DEBUG: measure time. */
    auto t2 = std::chrono::high_resolution_clock::now();
    auto delta_t = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
    printf("Fill operator took %d ms.\n", int(delta_t.count()));
#endif  // GP_FILL_DEBUG_MODE

    return success;
  }

  /**
   * Modal handler for the fill operator:
   * - Change gap closure length/radius
   * - Perform the fill at second mouse click
   */
  int fill_modal(const wmOperator &op, const wmEvent &event)
  {
    int modal_state = OPERATOR_RUNNING_MODAL;

    /* Prevent repeating event keys, when not released yet. */
    if (this->wait_for_release && this->wait_event_type == event.type) {
      if (event.val == KM_RELEASE) {
        this->wait_for_release = false;
      }
      return modal_state;
    }

    switch (event.type) {
      case EVT_ESCKEY:
      case RIGHTMOUSE:
        modal_state = OPERATOR_CANCELLED;
        break;

      case EVT_PAGEUPKEY:
      case WHEELUPMOUSE:
        this->update_gap_distance((event.modifier & KM_SHIFT) ? 0.01f : 0.1f);
        break;
      case EVT_PAGEDOWNKEY:
      case WHEELDOWNMOUSE:
        this->update_gap_distance((event.modifier & KM_SHIFT) ? -0.01f : -0.1f);
        break;

      case EVT_LEFTBRACKETKEY:
        if (this->use_gap_close_proximity) {
          this->update_proximity_distance(-1);
        }
        break;
      case EVT_RIGHTBRACKETKEY:
        if (this->use_gap_close_proximity) {
          this->update_proximity_distance(1);
        }
        break;

      case LEFTMOUSE: {
        /* Get mouse position of second click (the 'fill' click). */
        this->mouse_pos[0] = float(event.mval[0]);
        this->mouse_pos[1] = float(event.mval[1]);

        /* Perform the fill operation. */
        if (this->fill_exec()) {
          modal_state = OPERATOR_FINISHED;
        }
        else {
          BKE_report(op.reports, RPT_INFO, "Unable to fill unclosed area");
          modal_state = OPERATOR_CANCELLED;
        }
        break;
      }
    }

    return modal_state;
  }
};

}  // namespace blender::ed::greasepencil::fill
