/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgreasepencil
 */

/* DEBUG: print segment info, show curve indices in viewport. */
#define GP_VFILL_DEBUG_VERBOSE

#include "DNA_brush_types.h"
#include "DNA_meshdata_types.h"

#include "BKE_brush.hh"
#include "BKE_context.h"
#include "BKE_grease_pencil.hh"
#include "BKE_report.h"

#include "BLI_kdtree.h"
#include "BLI_math_vector.h"
#include "BLI_math_vector.hh"

#include "DEG_depsgraph_query.h"

#include "ED_curves.hh"
#include "ED_grease_pencil.hh"
#include "ED_keyframing.hh"
#include "ED_screen.hh"
#include "ED_space_api.hh"
#include "ED_view3d.hh"

#include "GPU_framebuffer.h"
#include "GPU_immediate.h"
#include "GPU_matrix.h"
#include "GPU_state.h"

#ifdef GP_VFILL_DEBUG_VERBOSE
#  include "BLF_api.h"
#  include "UI_interface.hh"
#  include "UI_resources.hh"
#endif

#include "WM_api.hh"

/**
 * Explanation of the used algorithm for vector-based fill:
 *
 *
 *     |                         (C)      segment 1       \ (D)
 *    -|-->>----------------------x--->>------------->>----\---
 *     |                                                    \
 *     ^                          ^                          \  segment 2
 *     ^                          |                           \
 *     |                      (B) |                            \|
 *     |                          |                             |\
 *     |                    (A) mouse                           |
 *     |                        click                           |
 *     |                                                        |  segment 3
 *     |                                                        |    etc.
 *     |                                       1                |
 *     |                                       |  (E)           |
 *     ^                                  2 ---|----------<<----|-
 *     ^                   /\                  |
 *     |       (G)        /  \               3 |
 *     |    e0    e1     /    \  (F)           |
 *   ..|....x-----x-----/------\---------<<----|-
 *     |    r1         /        \
 *     r2
 *
 * When the user clicks to fill an area, a ray (B) is casted from the mouse position (A)
 * to find a first intersection point with a curve (C).
 * From point (C) a 'walk-along-egde' search is carried out to find a fully closed fill edge.
 * The walk is in clockwise direction (C -->>--).
 * The edge is made up of segments. A segment is part of a curve and ends at an intersection
 * with another curve (D) or at the end of the curve itself (G).
 * At an intersection (D), the right turn will be inspected first. Because we are inspecting
 * the edge in clockwise direction, this guarantees us that we find the 'narrowest' edge. See
 * for example at (F): by taking the right turn first, we find the smallest fill area (just as
 * we should).
 * At an intersection, there are three possible turns. When a turn is not leading to a closed edge
 * (see e.g. (E), turn 1 and 2), the next turn is inspected.
 *
 * When a segment ends without intersection, the 'gap closure' inspection starts (G). In 'extend'
 * mode, the extrapolated first/last two points of the curve (e0 and e1) are used to find
 * intersections with other curves (or curve extensions).
 * In 'radius' mode, we search for other curve ends (r2) within a radius of the end point (r1)
 * of the curve.
 *
 * When a closed edge is found, we have to check if the mouse click position (A) is inside this
 * edge. This is because edges can be self-intersecting and give a 'false-positive' close.
 *
 * All inspections are done with a 2D (viewport) representation of all curve points.
 * When a closed edge is found, the fill geometry is created with the corresponding 3D
 * coordinates of the segment points. That way we almost only use existing geometry points; at
 * intersections we create one additional curve point, that's it.
 *
 * Finding intersections can be a bit troublesome when two curves overlap (sharing exact same
 * segment points). This can easily occur when the fill tool is used twice: the original stroke
 * and the fill curve share geometry points. Therefore, when looking for intersections, priority is
 * given to curves with only 'stroke' material over curves with a 'fill' material.
 *
 * In the code:
 * #vector_fill_do()          start of the algorithm, casting a ray (B)
 * #find_closed_fill_edge()   search for the narrowest closed edge
 * #walk_along_curve()        creating an edge segment, inspecting where the segment ends
 */

namespace blender::ed::greasepencil::vectorfill {

/* ------------------------------------------------------------------------------ */
/** \name Vector Fill data structs
 * \{ */

/* Number of nearest neighbours when looking for gap closure by curve end radius. */
#define RADIUS_NEAREST_NUM 12
/* Number of pixels the gap closure size stands for. */
#define GAP_PIXEL_FACTOR 20.0f
/* Margin for intersection distances to be considered equal. */
#define DISTANCE_EPSILON 0.001f
/* Margin for angles to be considered equal. */
#define ANGLE_EPSILON 0.001f
/* Margin for vector cross product considered to be parallel. */
#define PARALLEL_EPSILON 0.01f
/* Maximum execution time of vector fill operator (in milliseconds). */
#define MAX_EXECUTION_TIME 500

static inline constexpr float less_than_2pi(float angle)
{
  return (angle >= 2 * M_PI) ? (angle - 2 * M_PI) : angle;
}

/* Fill segment and segment end flags. */
enum class vfFlag : uint8_t {
  None = 0,
  /* Segment is inspected on intersections and gap closures. */
  IsInspected = 1 << 0,
  /* When backwards, point range of segment is from high to low. */
  DirectionBackwards = 1 << 1,
  /* Segment is intersected by other curve (or curve end extension.) */
  IsIntersected = 1 << 2,
  /* Segment is part of a set (two segments of cyclic curve, or overlapping tail segments). */
  IsPartOfSet = 1 << 3,
  /* Segment ends with a gap closure to another curve. */
  IsGapClosure = 1 << 4,
  /* Segment end connects with end extension of other curve. */
  WithEndExtension = 1 << 5,
  /* For the first segment only: ignore intersections before the start distance. */
  CheckStartDistance = 1 << 6,
  /* Segment is part of the unused head of a closed fill edge. */
  IsUnused = 1 << 7,
};
ENUM_OPERATORS(vfFlag, vfFlag::IsUnused);
static inline constexpr bool operator==(vfFlag a, bool b)
{
  return (uint64_t(a) != 0) == b;
}

/* Directions of casted ray to find first edge point of fill. */
enum class vfRayDirection : uint8_t {
  Up = 0,
  Right = 1,
  Down = 2,
  Left = 3,
};

constexpr std::initializer_list<vfRayDirection> ray_directions = {
    vfRayDirection::Up, vfRayDirection::Right, vfRayDirection::Down, vfRayDirection::Left};

/* Possible turns at a curve intersection. */
enum class vfSegmentTurn : uint8_t {
  None = 0,
  FirstAngle = 1,
  StraightAhead = 2,
  SecondAngle = 3,
};

/* Intersecting segment in 2D space. */
struct IntersectingSegment2D {
  /* Curve index in #Curves2DSpace struct. */
  int curve_index_2d;
  /* Start point of the intersection (relative index: 0..<curve_point_size>). */
  int point_start;
  /* End point of the intersection (relative index: 0..<curve_point_size>). */
  int point_end;
  /* Distance of the intersection point on the intersected and intersecting segment. */
  float2 distance;
  /* Intersection with end extension. */
  bool with_end_extension = false;
  /* Index of the curve end extension. */
  int extension_index;
};

/* Edge segment ending: intersection data when a curve of a curve end extension intersects
 * the edge segment. */
struct SegmentEnd {
  /* Curve index (in #Curves2DSpace struct) of the intersecting/gap closing curve. */
  int curve_index_2d{};
  /* Curve point index of the intersection start point. */
  int point_start{};
  /* Curve point index of the intersection end point. */
  int point_end{};
  /* Curve end point index of the original curve that is intersected. */
  int ori_segment_point_end{};
  /* Normalized distance of intersection. */
  float distance{};
  /* Incoming and outgoing angle of the intersection. */
  float2 angle{};
  /* Minimum angle. */
  float angle_min;
  /* Maximum angle. */
  float angle_max;
  /* Flag if the incoming and outgoing angle is set. */
  bool angle_is_set[2];
  /* Segment flags. */
  vfFlag flag = vfFlag::None;
  /* Turn to take at intersections. */
  short turn{};

  bool first_angle_is_smallest() const
  {
    if (!this->angle_is_set[0]) {
      return false;
    }
    if (!this->angle_is_set[1]) {
      return true;
    }
    return this->angle[0] <= this->angle[1];
  }

  bool first_angle_is_largest() const
  {
    if (!this->angle_is_set[0]) {
      return false;
    }
    if (!this->angle_is_set[1]) {
      return true;
    }
    return this->angle[0] > this->angle[1];
  }
};

/* Fill edge segment. An edge segment is a part of an existing curve. A fill edge is a closed set
 * of edge segments. */
struct EdgeSegment {
  /* Curve index in #Curves2DSpace struct (with curve points in 2D space). */
  int curve_index_2d{};
  /* Curve point index range. */
  int2 point_range{};
  /* Segment flags. */
  vfFlag flag = vfFlag::None;

  /* Segment endings: intersection data or gap closure data. */
  Vector<SegmentEnd> segment_ends{};
  /* Last handled segment end. */
  int segment_ends_index{};

  void init_point_range(const int point_i)
  {
    this->point_range[0] = point_i;
    this->point_range[1] = point_i;
  }

  void set_point_range(const int point_i, const int direction)
  {
    if (direction == -1) {
      this->point_range[1] = this->point_range[0];
      this->point_range[0] = point_i;
    }
    else {
      this->point_range[1] = point_i;
    }
  }
};

/* Segment that overlaps with the last created segment. Used for closed egde inspection, when the
 * segments overlap, but have different curve indices. */
struct OverlappingSegment {
  int2 base_point_range{};
  int overlapping_curve_index{};
  int2 overlapping_point_range{};
  bool overlapping_backwards = false;
};

struct VectorFillData {
  /* For modal event handling: wait for release until processing the event key again. */
  bool wait_for_release{};
  /* Wait-for-release event key. */
  short wait_event_type{};

  /* Mouse position of the fill operation click. */
  float2 mouse_pos{};

  /* True when edge gaps are closed by extending the curve ends. */
  bool gap_close_extend{};
  /* True when edge gaps are closed by curve end radii. */
  bool gap_close_radius{};
  /* True when curve end extensions can collide with curves. */
  bool extensions_collide_with_curves{};
  /* Gap closure distance in pixels. */
  float gap_distance{};

  /* Active Grease Pencil object. */
  GreasePencil *grease_pencil;
  /* View context data.*/
  ViewContext vc;
  /* Fill brush. */
  Brush *brush;

  /* Draw handle for 3D viewport overlay. */
  void *draw_handle = nullptr;

  /* Starting time of the vector fill operator. */
  std::chrono::high_resolution_clock::time_point operator_time_start;

  /* Curve points converted to viewport 2D space. */
  Curves2DSpace curves_2d{};
  /* Slight normalized extension of all curve segments (epsilon), used to avoid floating precision
   * errors when looking for intersections. */
  Array<float2> curve_segment_epsilon;
  /* Curve end extensions in viewport 2D space. */
  Curves2DSpace extensions_2d{};
  /* Ratio between length of extension and length of curve end segment. */
  Array<float> extension_length_ratio;
  /* Flag for curve end extensions, true when the extension intersects a curve or other
   * end extension. */
  Array<bool> extension_has_intersection;
  /* Intersections of curve end extensions with curves or other extensions. */
  Vector<IntersectingSegment2D> extension_intersections;
  /* KD tree of curve ends in 2D space, used for gap closure by radius. */
  KDTree_2d *curve_ends = nullptr;
  /* Flag indicating a curve end is connected by radius with one or more other curve ends. */
  Array<bool> connected_by_radius;
  /* Curve end connections (by radius) with other curve ends. */
  Vector<int2> radius_connections;

  /* The initial curve segments from which we try to find a closed fill edge. */
  Vector<IntersectingSegment2D> starting_segments;
  /* The curve segments that will form a closed fill edge. */
  Vector<EdgeSegment> segments;
  /* Overlapping curve segments of the last created edge segment. */
  Vector<OverlappingSegment> overlapping_segments;

  /* The intersection distance of the start segment (found by the casted ray). */
  float start_distance{};
};

/** \} */

/* ------------------------------------------------------------------------------ */
/** \name Intersection functions
 * \{ */

/**
 * Get the normalized distance from v1 and v3 to the intersection point of line segments v1-v2
 * and v3-v4.
 */
static float2 get_intersection_distance_normalized(const float2 &v1,
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

/**
 * Store a segment that shares points with an inspected edge segment.
 */
static void store_overlapping_segment(const int overlapping_curve,
                                      const int overlapping_point,
                                      const bool in_opposite_dir,
                                      const int base_point,
                                      const int base_direction,
                                      VectorFillData *vf)
{
  const int overlapping_direction = (in_opposite_dir ? -base_direction : base_direction);
  const bool overlapping_backwards = (overlapping_direction == -1);
  const int overlapping_range_index = (overlapping_direction == 1 ? 0 : 1);
  const int base_range_index = (base_direction == 1 ? 0 : 1);

  /* Look for an existing overlapping segment which point range we can expand. */
  for (auto &overlap : vf->overlapping_segments) {
    if (overlap.overlapping_curve_index == overlapping_curve &&
        overlap.overlapping_backwards == overlapping_backwards &&
        overlap.overlapping_point_range[overlapping_range_index] ==
            (overlapping_point - overlapping_direction) &&
        overlap.base_point_range[base_range_index] == (base_point - base_direction))
    {
      overlap.overlapping_point_range[overlapping_range_index] = overlapping_point;
      overlap.base_point_range[base_range_index] = base_point;
      return;
    }
  }

  /* Create new overlapping segment. */
  OverlappingSegment overlap{};
  overlap.overlapping_curve_index = overlapping_curve;
  overlap.overlapping_backwards = overlapping_backwards;
  overlap.overlapping_point_range[0] = overlapping_point;
  overlap.overlapping_point_range[1] = overlapping_point;
  overlap.base_point_range[0] = base_point;
  overlap.base_point_range[1] = base_point;

  std::mutex mutex;
  std::lock_guard lock{mutex};
  vf->overlapping_segments.append(std::move(overlap));
}

/**
 * Get all intersections of a segment with other curves.
 */
static Vector<IntersectingSegment2D> get_intersections_of_segment_with_curves(
    const float2 &segment_a,
    const float2 &segment_b,
    const int segment_curve_index,
    const Curves2DSpace *curves_2d,
    const float2 &adj_a = {FLT_MAX, FLT_MAX},
    const float2 &adj_b = {FLT_MAX, FLT_MAX},
    const bool store_overlapping_segments = false,
    const int segment_point_start = 0,
    const int segment_point_next = 0,
    const int segment_direction = 1,
    const bool slight_extend = false,
    VectorFillData *vf = nullptr)
{
  /* Init result vector. */
  Vector<IntersectingSegment2D> intersections;
  std::mutex mutex;

  /* Create a slightly extended segment to avoid float precision errors. Otherwise an intersection
   * can be missed when it is on the outer end or start of a segment. */
  float2 seg_a_extended = segment_a;
  float2 seg_b_extended = segment_b;
  if (slight_extend) {
    if (segment_direction == 1) {
      const int ext_i = curves_2d->point_offset[segment_curve_index] + segment_point_start;
      seg_a_extended -= vf->curve_segment_epsilon[ext_i];
      seg_b_extended += vf->curve_segment_epsilon[ext_i];
    }
    else {
      const int ext_i = curves_2d->point_offset[segment_curve_index] + segment_point_next;
      seg_a_extended += vf->curve_segment_epsilon[ext_i];
      seg_b_extended -= vf->curve_segment_epsilon[ext_i];
    }
  }

  /* Create bounding box around the segment. */
  rctf bbox_segment, bbox_isect;
  BLI_rctf_init_minmax(&bbox_segment);
  BLI_rctf_do_minmax_v(&bbox_segment, seg_a_extended);
  BLI_rctf_do_minmax_v(&bbox_segment, seg_b_extended);

  const float2 segment_vec = segment_b - segment_a;

  /* Loop all curves, looking for intersecting segments. */
  threading::parallel_for(curves_2d->point_offset.index_range(), 256, [&](const IndexRange range) {
    for (const int curve_i : range) {
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

      /* Skip curves with identical (overlapping) segments, because they produce false-positive
       * intersections. Overlapping curves are most likely created by a previous fill operation. */
      if (curve_i != segment_curve_index) {
        bool skip_curve = false;
        for (const int point_i : IndexRange(point_offset, point_size)) {
          const int point_i_next = (point_i == point_last ? point_offset : point_i + 1);
          const float2 p0 = curves_2d->points_2d[point_i];
          const float2 p1 = curves_2d->points_2d[point_i_next];

          /* Check for identical segments. */
          if (segment_a == p0 && segment_b == p1) {
            if (store_overlapping_segments) {
              store_overlapping_segment(
                  curve_i, point_i - point_offset, 1, segment_point_start, segment_direction, vf);
            }
            skip_curve = true;
            break;
          }
          if (segment_a == p1 && segment_b == p0) {
            if (store_overlapping_segments) {
              store_overlapping_segment(
                  curve_i, point_i - point_offset, -1, segment_point_start, segment_direction, vf);
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
        if (slight_extend) {
          p0_extended -= vf->curve_segment_epsilon[point_i];
          p1_extended += vf->curve_segment_epsilon[point_i];
        }

        /* Skip when previous segment was intersecting. */
        if (prev_intersection_point == point_i) {
          continue;
        }

        /* Skip when bounding boxes don't overlap. */
        bbox_isect.xmin = bbox_isect.xmax = p0_extended[0];
        bbox_isect.ymin = bbox_isect.ymax = p0_extended[1];
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
          const float2 p_vec = p1 - p0;
          if (compare_ff(cross_v2v2(p_vec, segment_vec), 0.0f, PARALLEL_EPSILON)) {
            continue;
          }
        }

        /* Check for intersection. */
        auto isect = math::isect_seg_seg(seg_a_extended, seg_b_extended, p0_extended, p1_extended);
        if (ELEM(isect.kind, isect.LINE_LINE_CROSS, isect.LINE_LINE_EXACT)) {
          IntersectingSegment2D intersection{};
          intersection.curve_index_2d = curve_i;
          intersection.point_start = point_i - point_offset;
          intersection.point_end = point_i_next - point_offset;
          intersection.distance = get_intersection_distance_normalized(
              segment_a, segment_b, p0, p1);

          std::lock_guard lock{mutex};
          intersections.append(intersection);

          prev_intersection_point = point_i_next;
        }
      }
    }
  });

  return intersections;
}

/** \} */

/* ------------------------------------------------------------------------------ */
/** \name Vector Fill functions
 * \{ */

static bool equals_previous(const float3 &co, float3 &r_co_prev)
{
  if (equals_v3v3(co, r_co_prev)) {
    return true;
  }
  copy_v3_v3(r_co_prev, co);
  return false;
}

/**
 * Convert fill edge segments to 3D points.
 */
static Vector<float3> get_closed_fill_edge_as_3d_points(VectorFillData *vf)
{
  /* Ensure head starting point and tail end point are an exact match. */
  EdgeSegment *tail = &vf->segments.last();
  for (auto &head : vf->segments) {
    if ((head.flag & vfFlag::IsUnused) == true) {
      continue;
    }
    if ((head.flag & vfFlag::DirectionBackwards) == true) {
      if (head.point_range[1] > tail->point_range[1]) {
        head.point_range[1] = tail->point_range[1];
      }
    }
    else {
      if (head.point_range[0] < tail->point_range[0]) {
        head.point_range[0] = tail->point_range[0];
      }
    }
    break;
  }

  Vector<float3> edge_points;
  float3 co_prev = {FLT_MAX, FLT_MAX, FLT_MAX};
  int edge_point_index = 0;

  /* Convert all edge segments to 3D coordinates. */
  for (auto &segment : vf->segments) {
    if ((segment.flag & vfFlag::IsUnused) == true) {
      continue;
    }

    /* Get original 3D curve. */
    const int drawing_index = vf->curves_2d.drawing_index_2d[segment.curve_index_2d];
    const int curve_index = segment.curve_index_2d - vf->curves_2d.curve_offset[drawing_index];
    const bke::CurvesGeometry &curves = vf->curves_2d.drawings[drawing_index]->geometry.wrap();
    const Span<float3> positions = curves.positions();
    const OffsetIndices points_by_curve = curves.points_by_curve();
    const IndexRange points_3d = points_by_curve[curve_index];
    const int point_offset = points_3d.first();

    /* Append curve points of this segment to 3D edge point array. */
    const int direction = ((segment.flag & vfFlag::DirectionBackwards) == true) ? -1 : 1;
    int point_start = segment.point_range[0] + point_offset;
    int point_end = segment.point_range[1] + point_offset;
    if (direction == -1) {
      std::swap(point_start, point_end);
    }
    point_end += direction;

    for (int point_i = point_start; point_i != point_end; point_i += direction) {
      if (!equals_previous(positions[point_i], co_prev)) {
        edge_points.append(positions[point_i]);
        edge_point_index++;
      }
    }

    /* Calculate regular intersection point. */
    if ((segment.flag & vfFlag::IsIntersected) == true) {
      const SegmentEnd *segment_end = &segment.segment_ends[segment.segment_ends_index];
      const float3 isect_point = edge_points.last() +
                                 (positions[point_offset + segment_end->ori_segment_point_end] -
                                  edge_points.last()) *
                                     segment_end->distance;
      if (!equals_previous(isect_point, co_prev)) {
        edge_points.append(isect_point);
        edge_point_index++;
      }
    }

    /* Calculate intersection with curve end extension. */
    if (vf->gap_close_extend && (segment.flag & vfFlag::IsGapClosure) == true &&
        edge_point_index > 1) {
      /* Get segment end data. */
      const SegmentEnd *segment_end = &segment.segment_ends[segment.segment_ends_index];
      const int extension_index = segment.curve_index_2d * 2 + (direction == 1 ? 1 : 0);

      /* Extend end of the curve. */
      const float3 isect_point = edge_points[edge_point_index - 1] +
                                 (edge_points[edge_point_index - 1] -
                                  edge_points[edge_point_index - 2]) *
                                     vf->extension_length_ratio[extension_index] *
                                     segment_end->distance;
      if (!equals_previous(isect_point, co_prev)) {
        edge_points.append(isect_point);
        edge_point_index++;
      }
    }
  }

  /* Remove last point, since it is the same as the first point. */
  edge_points.remove_last();

  return edge_points;
}

/**
 * Create GP curve geometry from a closed fill edge.
 */
static void create_fill_geometry(VectorFillData *vf)
{
  /* Ensure active frame (autokey is on, we checked on operator invoke). */
  const bke::greasepencil::Layer *active_layer = vf->grease_pencil->get_active_layer();
  if (active_layer->drawing_index_at(vf->vc.scene->r.cfra) == -1) {
    bke::greasepencil::Layer *layer = vf->grease_pencil->get_active_layer_for_write();
    vf->grease_pencil->insert_blank_frame(*layer, vf->vc.scene->r.cfra, 0, BEZT_KEYTYPE_KEYFRAME);
  }

  /* Get the edge points in 3D space. */
  Vector<float3> fill_points = get_closed_fill_edge_as_3d_points(vf);

  /* Create geometry. */
  const int drawing_index = active_layer->drawing_index_at(vf->vc.scene->r.cfra);
  bke::greasepencil::Drawing &drawing = reinterpret_cast<GreasePencilDrawing *>(
                                            vf->grease_pencil->drawings()[drawing_index])
                                            ->wrap();
  bke::CurvesGeometry &curves = drawing.strokes_for_write();

  const int num_old_curves = curves.curves_num();
  const int num_old_points = curves.points_num();
  curves.resize(num_old_points + fill_points.size(), num_old_curves + 1);

  curves.offsets_for_write()[num_old_curves] = num_old_points;
  curves.offsets_for_write()[num_old_curves + 1] = num_old_points + fill_points.size();

  const OffsetIndices<int> points_by_curve = curves.points_by_curve();
  const IndexRange new_points_range = points_by_curve[curves.curves_num() - 1];
  const IndexRange new_curves_range = IndexRange(num_old_curves, 1);

  /* Set position, radius and opacity attribute. */
  bke::MutableAttributeAccessor attributes = curves.attributes_for_write();
  MutableSpan<float3> positions = curves.positions_for_write();
  MutableSpan<float> radii = drawing.radii_for_write();
  MutableSpan<float> opacities = drawing.opacities_for_write();
  positions.slice(new_points_range).copy_from(fill_points);
  radii.slice(new_points_range).fill(vf->brush->size);
  opacities.slice(new_points_range).fill(1.0f);

  /* Make curve cyclic. */
  curves.cyclic_for_write().slice(new_curves_range).fill(true);

  /* Set curve_type attribute. */
  curves.fill_curve_types(new_curves_range, CURVE_TYPE_POLY);

  /* Set vertex color for fill and stroke. */
  const bool use_vertex_color = (vf->vc.scene->toolsettings->gp_paint->mode ==
                                 GPPAINT_FLAG_USE_VERTEXCOLOR);
  const bool use_vertex_color_stroke = use_vertex_color &&
                                       ELEM(vf->brush->gpencil_settings->vertex_mode,
                                            GPPAINT_MODE_STROKE,
                                            GPPAINT_MODE_BOTH);
  const bool use_vertex_color_fill = use_vertex_color &&
                                     ELEM(vf->brush->gpencil_settings->vertex_mode,
                                          GPPAINT_MODE_FILL,
                                          GPPAINT_MODE_BOTH);
  const ColorGeometry4f vertex_color_stroke = use_vertex_color_stroke ?
                                                  ColorGeometry4f(
                                                      vf->brush->rgb[0],
                                                      vf->brush->rgb[1],
                                                      vf->brush->rgb[2],
                                                      vf->brush->gpencil_settings->vertex_factor) :
                                                  ColorGeometry4f(0.0f, 0.0f, 0.0f, 0.0f);
  const ColorGeometry4f vertex_color_fill = use_vertex_color_fill ?
                                                ColorGeometry4f(
                                                    vf->brush->rgb[0],
                                                    vf->brush->rgb[1],
                                                    vf->brush->rgb[2],
                                                    vf->brush->gpencil_settings->vertex_factor) :
                                                ColorGeometry4f(0.0f, 0.0f, 0.0f, 0.0f);

  bke::SpanAttributeWriter<ColorGeometry4f> vertex_colors =
      attributes.lookup_or_add_for_write_span<ColorGeometry4f>("vertex_color", ATTR_DOMAIN_POINT);
  vertex_colors.span.slice(new_points_range).fill(vertex_color_stroke);
  vertex_colors.finish();
  bke::SpanAttributeWriter<ColorGeometry4f> fill_colors =
      attributes.lookup_or_add_for_write_span<ColorGeometry4f>("fill_color", ATTR_DOMAIN_CURVE);
  fill_colors.span.slice(new_curves_range).fill(vertex_color_fill);
  fill_colors.finish();

  /* Set material. */
  Material *material = BKE_grease_pencil_object_material_ensure_from_active_input_brush(
      vf->vc.bmain, vf->vc.obact, vf->brush);
  const int material_index = BKE_grease_pencil_object_material_index_get(vf->vc.obact, material);

  bke::SpanAttributeWriter<int> materials = attributes.lookup_or_add_for_write_span<int>(
      "material_index", ATTR_DOMAIN_CURVE);
  materials.span.slice(new_curves_range).fill(material_index);
  materials.finish();

  /* Explicitly set all other attributes besides those processed above to default values. */
  Set<std::string> attributes_to_skip{{"position",
                                       "radius",
                                       "opacity",
                                       "curve_type",
                                       "cyclic",
                                       "vertex_color",
                                       "fill_color",
                                       "material_index"}};
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

  /* Set notifiers. */
  drawing.tag_positions_changed();
  DEG_id_tag_update(&vf->grease_pencil->id, ID_RECALC_GEOMETRY);
  WM_main_add_notifier(NC_GEOM | ND_DATA, &vf->grease_pencil->id);
}

/**
 * Check if the mouse coordinates are inside a closed fill edge.
 */
static bool mouse_pos_is_inside_polygon(float2 &mouse_pos, Vector<float2> &polygon)
{
  /* Rare edge case: polygon without area. */
  if (polygon.size() < 3) {
    return false;
  }

  /* Algorithm: draw a long horizontal line from the mouse position and count the number of
   * intersections with the polygon. An odd number of intersections means the mouse position is
   * inside the polygon. */
  const float2 line_p1 = mouse_pos;
  const float2 line_p2 = {line_p1[0] + 20000.0f, line_p1[1]};
  int count = 0;
  for (const int i : polygon.index_range().drop_back(1)) {
    auto isect = math::isect_seg_seg(polygon[i], polygon[i + 1], line_p1, line_p2);
    if (ELEM(isect.kind, isect.LINE_LINE_CROSS, isect.LINE_LINE_EXACT)) {
      count++;
    }
  }

  return (count & 1) == 1;
}

/**
 * Convert fill edge segments to 2D points.
 */
static Vector<float2> get_closed_fill_edge_as_2d_polygon(VectorFillData *vf)
{
  Vector<float2> points;

  /* We cut some corners here, literally, because we don't calculate the exact intersection point
   * of intersections. Since the 2D polygon is only used for a mouse-position-inside-polygon
   * check, we can afford to be a little sloppy. */
  for (auto &segment : vf->segments) {
    if ((segment.flag & vfFlag::IsUnused) == true) {
      continue;
    }

    const int point_offset = vf->curves_2d.point_offset[segment.curve_index_2d];
    const int direction = ((segment.flag & vfFlag::DirectionBackwards) == true) ? -1 : 1;
    int point_start = segment.point_range[0];
    int point_end = segment.point_range[1];
    if (direction == -1) {
      std::swap(point_start, point_end);
    }
    point_end += direction;
    for (int point_i = point_start; point_i != point_end; point_i += direction) {
      points.append(vf->curves_2d.points_2d[point_offset + point_i]);
    }
  }

  return points;
}

static bool mouse_pos_is_inside_closed_edge(VectorFillData *vf)
{
  Vector<float2> closed_edge = get_closed_fill_edge_as_2d_polygon(vf);

  return mouse_pos_is_inside_polygon(vf->mouse_pos, closed_edge);
}

/**
 * Split an edge segment into its original and overlapping one.
 */
static void split_segment_with_overlap(EdgeSegment *segment,
                                       const OverlappingSegment &overlap,
                                       VectorFillData *vf)
{
  const bool segment_backwards = ((segment->flag & vfFlag::DirectionBackwards) == true);

  /* Check if we can replace the entire segment by its overlapping companion. */
  if ((!segment_backwards && segment->point_range[0] == overlap.base_point_range[0]) ||
      (segment_backwards && segment->point_range[1] == overlap.base_point_range[1]))
  {
    segment->curve_index_2d = overlap.overlapping_curve_index;
    segment->point_range = overlap.overlapping_point_range;
    segment->flag &= ~vfFlag::DirectionBackwards;
    if (overlap.overlapping_backwards) {
      segment->flag |= vfFlag::DirectionBackwards;
    }
    return;
  }

  /* Shrink the segment and add another one for the overlapping part. */
  if (segment_backwards) {
    segment->point_range[0] = overlap.base_point_range[1] + 1;
  }
  else {
    segment->point_range[1] = overlap.base_point_range[0] - 1;
  }
  segment->flag &= ~vfFlag::IsIntersected;

  EdgeSegment new_seg{};
  new_seg.curve_index_2d = overlap.overlapping_curve_index;
  new_seg.point_range = overlap.overlapping_point_range;
  new_seg.flag |= vfFlag::IsPartOfSet;
  if (overlap.overlapping_backwards) {
    new_seg.flag |= vfFlag::DirectionBackwards;
  }
  vf->segments.append(std::move(new_seg));
}

/**
 * Remove redundant tail points from a closed fill edge.
 */
static void remove_overlapping_edge_tail_points(const EdgeSegment *head,
                                                EdgeSegment *tail,
                                                const int2 &range_head,
                                                int2 &range_tail)
{
  /* Get head starting point. */
  const int head_start = ((head->flag & vfFlag::DirectionBackwards) == true) ? range_head[1] :
                                                                               range_head[0];

  /* Limit tail to head starting point. */
  if ((tail->flag & vfFlag::DirectionBackwards) == true) {
    if (head_start <= range_tail[1]) {
      range_tail[0] = head_start;
    }
    else {
      range_tail[0] = range_tail[1];
    }
  }
  else {
    if (head_start >= range_tail[0]) {
      range_tail[1] = head_start;
    }
    else {
      range_tail[1] = range_tail[0];
    }
  }

  /* Clear tail end flags. */
  tail->flag &= ~vfFlag::IsGapClosure;
  tail->flag &= ~vfFlag::IsIntersected;
}

/**
 * Check if edge segments overlap. This includes a check of alternative segments,
 * that share points with the original one.
 */
static bool segments_have_overlap(EdgeSegment *head, EdgeSegment *tail, VectorFillData *vf)
{
  /* Check direct overlap of head and tail.*/
  if (head->curve_index_2d == tail->curve_index_2d) {
    /* Check overlap in curve points. */
    if (head->point_range[0] <= tail->point_range[1] &&
        head->point_range[1] >= tail->point_range[0]) {
      remove_overlapping_edge_tail_points(head, tail, head->point_range, tail->point_range);
      return true;
    }
    return false;
  }

  /* Check alternative segments that have an overlap with the tail segment. */
  for (const auto &overlap : vf->overlapping_segments) {
    if (head->curve_index_2d == overlap.overlapping_curve_index &&
        head->point_range[0] <= overlap.overlapping_point_range[1] &&
        head->point_range[1] >= overlap.overlapping_point_range[0])
    {
      split_segment_with_overlap(tail, overlap, vf);
      tail = &vf->segments.last();
      remove_overlapping_edge_tail_points(head, tail, head->point_range, tail->point_range);
      return true;
    }
  }

  return false;
}

/**
 * Check if a set of edge segments form a closed loop.
 */
static bool is_closed_fill_edge(VectorFillData *vf)
{
  /* Init: by default all segments are used in the edge. */
  for (auto &segment : vf->segments) {
    segment.flag &= ~vfFlag::IsUnused;
  }

  /* Loop through the edge segments and see if there is overlap with the last one. */
  EdgeSegment *tail = &vf->segments.last();
  for (const int segment_i : vf->segments.index_range().drop_back(1)) {
    if (segments_have_overlap(&vf->segments[segment_i], tail, vf)) {
      return true;
    }
    vf->segments[segment_i].flag |= vfFlag::IsUnused;
  }

  return false;
}

/**
 * Get the next curve point index. Handles cyclic curves.
 */
static int get_next_curve_point(const int point_i,
                                const int direction,
                                const int point_size,
                                const bool is_cyclic)
{
  int point_next = point_i + direction;

  if (is_cyclic) {
    if (point_next < 0) {
      point_next = point_size - 1;
    }
    if (point_next >= point_size) {
      point_next = 0;
    }
    return point_next;
  }

  if (point_next >= point_size) {
    point_next = -1;
  }

  return point_next;
}

/**
 * Add 'end' information to an edge segment:
 * - Does the segment end by curve intersection or curve extension (gap closure)?
 * - Determine the 'right turn' for the segment end
 */
static void add_segment_end(const Curves2DSpace *curves_2d,
                            EdgeSegment *segment,
                            const float2 &segment_point_a,
                            const float2 &segment_point_b,
                            const int segment_point_b_index,
                            const Vector<IntersectingSegment2D> &intersections,
                            const bool is_end_extension,
                            const VectorFillData *vf)
{
  const float2 segment_vec = segment_point_b - segment_point_a;

  for (auto &intersection : intersections) {
    /* Skip end extensions of cyclic curves. */
    if (is_end_extension && curves_2d->is_cyclic[intersection.curve_index_2d]) {
      continue;
    }
    /* Skip intersections before the start segment distance. */
    if ((segment->flag & vfFlag::CheckStartDistance) == true &&
        intersection.distance[0] < vf->start_distance)
    {
      continue;
    }

    /* Copy intersection data. */
    SegmentEnd segment_end{};
    segment_end.curve_index_2d = intersection.curve_index_2d;
    segment_end.point_start = intersection.point_start;
    segment_end.point_end = intersection.point_end;
    segment_end.ori_segment_point_end = segment_point_b_index;
    segment_end.distance = intersection.distance[0];

    /* Determine incoming and outgoing angle of the intersection.
     * The most common case is where the intersecting segment cuts through segment a-b:
     *
     *            b                              b
     *            |                              |
     *   p1       |       p2            p2       |       p1
     *    x-->>-----------x      or      x-----------<<--x
     *        a1  |  a0                      a0  |  a1
     *            |                              |
     *            |                              |
     *            a                              a
     *
     * (a0 = angle0, a1 = angle1)
     */
    segment_end.angle_is_set[0] = true;
    segment_end.angle_is_set[1] = true;
    const int curve_i = intersection.curve_index_2d;
    const int point_offset = curves_2d->point_offset[curve_i];
    float2 p1 = curves_2d->points_2d[point_offset + intersection.point_start];
    float2 p2 = curves_2d->points_2d[point_offset + intersection.point_end];

    float angle0 = less_than_2pi(M_PI - angle_signed_v2v2(segment_vec, p2 - p1));
    float angle1 = less_than_2pi(angle0 + M_PI);
    bool dir_backwards = (angle0 > M_PI);

    /* Handle the edge case where the intersecting segment starts at segment a-b. E.g.:
     *
     *            b                         b
     *            |                         |
     *         p1 |       p2             p1 |  a0   p2
     *            x-->>---x      or         x-->>---x
     *          / |  a0                     |\
     *      a1 /  |                         | \ a1
     *        /   |                         |  \
     *    p0 x    |                         |   x p0
     *            a                         a
     */
    if (!is_end_extension && compare_ff(intersection.distance[1], 0.0f, DISTANCE_EPSILON)) {
      dir_backwards = false;
      const int point_prev = get_next_curve_point(intersection.point_start,
                                                  -1,
                                                  curves_2d->point_size[curve_i],
                                                  curves_2d->is_cyclic[curve_i]);
      if (point_prev == -1) {
        segment_end.angle_is_set[1] = false;
      }
      else {
        float2 p0 = curves_2d->points_2d[point_offset + point_prev];
        angle1 = less_than_2pi(M_PI - angle_signed_v2v2(segment_vec, p0 - p1));
        dir_backwards = (angle1 < angle0);
      }
    }

    /* Handle the edge case where the intersecting segment ends at segment a-b. E.g.:
     *
     *            b                            b
     *            |                            |
     *   p1       | p2                p1   a0  | p2
     *    x-->>---x            or      x-->>---x
     *        a0  |\                          /|
     *            | \ a1                  a1 / |
     *            |  \                      /  |
     *            |   x p3              p3 x   |
     *            a                            a
     */
    if (!is_end_extension && compare_ff(intersection.distance[1], 1.0f, DISTANCE_EPSILON)) {
      dir_backwards = true;
      angle0 = angle1;
      const int point_next = get_next_curve_point(intersection.point_end,
                                                  1,
                                                  curves_2d->point_size[curve_i],
                                                  curves_2d->is_cyclic[curve_i]);
      if (point_next == -1) {
        segment_end.angle_is_set[1] = false;
      }
      else {
        float2 p3 = curves_2d->points_2d[point_offset + point_next];
        angle1 = less_than_2pi(M_PI - angle_signed_v2v2(segment_vec, p3 - p2));
        dir_backwards = (angle1 > angle0);
      }
    }

    /* Set the curve direction on the (first) right turn. */
    if (dir_backwards) {
      segment_end.flag |= vfFlag::DirectionBackwards;
    }

    /* Handle intersection by end extension. */
    if (is_end_extension) {
      segment_end.curve_index_2d = int(intersection.curve_index_2d / 2);
      segment_end.flag = vfFlag::WithEndExtension;
      angle0 = angle1;

      /* Set curve direction: backwards for extension at end of curve. */
      if ((intersection.curve_index_2d & 1) == 1) {
        segment_end.flag |= vfFlag::DirectionBackwards;
        segment_end.point_start = vf->curves_2d.point_size[segment_end.curve_index_2d] - 1;
      }
      segment_end.point_end = intersection.point_start;
    }

    /* Clear angle when parallel to segment a-b. */
    if (compare_ff(angle0, 0.0f, ANGLE_EPSILON)) {
      segment_end.angle_is_set[0] = false;
    }
    if (compare_ff(angle1, 0.0f, ANGLE_EPSILON) || is_end_extension) {
      segment_end.angle_is_set[1] = false;
    }
    if (!(segment_end.angle_is_set[0] || segment_end.angle_is_set[1])) {
      continue;
    }

    /* Store angles and determine minimum/maximum. */
    segment_end.angle[0] = angle0;
    segment_end.angle[1] = angle1;
    segment_end.angle_min = (segment_end.first_angle_is_smallest() ? angle0 : angle1);
    segment_end.angle_max = (segment_end.first_angle_is_largest() ? angle0 : angle1);

    /* Append to segment end list. */
    segment->segment_ends.append(segment_end);
  }
}

/**
 * Expand an edge segment by walking along the curve. An edge segment ends when an intersection is
 * found or when the end of the curve is reached.
 * At the end of a curve, gap closure inspection is performed.
 */
static bool walk_along_curve(VectorFillData *vf, EdgeSegment *segment, const int started_at = -1)
{
  /* Skip when segment is already inspected. */
  if ((segment->flag & vfFlag::IsInspected) == true) {
    return false;
  }
  segment->flag |= vfFlag::IsInspected;

  /* Get curve point start, range etc. */
  const int curve_i = segment->curve_index_2d;
  const bool is_cyclic = vf->curves_2d.is_cyclic[curve_i];
  const int direction = ((segment->flag & vfFlag::DirectionBackwards) == true) ? -1 : 1;
  const int point_size = vf->curves_2d.point_size[curve_i];
  const int point_offset = vf->curves_2d.point_offset[curve_i];
  int point_i = segment->point_range[0];
  const int point_started = (started_at == -1 ? point_i : started_at);

  float2 segment_point_a = {FLT_MAX, FLT_MAX};
  const int point_prev = get_next_curve_point(point_i, -direction, point_size, is_cyclic);
  if (point_prev != -1) {
    segment_point_a = vf->curves_2d.points_2d[point_offset + point_prev];
  }

  vf->overlapping_segments.clear();

  bool check_gaps = false;

  /* Walk along the curve until an intersection or the end of the curve is reached. */
  do {
    /* Check curve bounds. */
    if (point_i < 0 || point_i >= point_size) {
      point_i = std::min(point_size - 1, std::max(0, point_i));
      if (!is_cyclic) {
        check_gaps = true;
        break;
      }

      /* Add new segment for second point range of cyclic curve. */
      segment->set_point_range(point_i, direction);
      segment->flag &= ~vfFlag::CheckStartDistance;

      point_i = (direction == -1) ? (point_size - 1) : 0;

      EdgeSegment new_segment{};
      new_segment.curve_index_2d = segment->curve_index_2d;
      new_segment.init_point_range(point_i);
      new_segment.flag = segment->flag;
      new_segment.flag |= vfFlag::IsPartOfSet;
      if (point_i != point_started) {
        new_segment.flag &= ~vfFlag::IsInspected;
      }
      vf->segments.append(new_segment);

      if (point_i == point_started) {
        return true;
      }

      /* Walk along second part of cyclic curve. */
      walk_along_curve(vf, &vf->segments.last(), point_started);
      return true;
    }

    /* Get next point on curve. */
    const int point_next = get_next_curve_point(point_i, direction, point_size, is_cyclic);
    if (point_next == -1) {
      check_gaps = true;
      break;
    }

    /* Get surrounding points. */
    const float2 point_a_prev = segment_point_a;
    segment_point_a = vf->curves_2d.points_2d[point_offset + point_i];
    const float2 segment_point_b = vf->curves_2d.points_2d[point_offset + point_next];
    float2 point_b_next = {FLT_MAX, FLT_MAX};
    const int point_next2 = get_next_curve_point(point_next, direction, point_size, is_cyclic);
    if (point_next2 != -1) {
      point_b_next = vf->curves_2d.points_2d[point_offset + point_next2];
    }

    /* Get intersections with other curves. */
    Vector<IntersectingSegment2D> intersections = get_intersections_of_segment_with_curves(
        segment_point_a,
        segment_point_b,
        curve_i,
        &vf->curves_2d,
        point_a_prev,
        point_b_next,
        true,
        point_i,
        point_next,
        direction,
        true,
        vf);
    add_segment_end(&vf->curves_2d,
                    segment,
                    segment_point_a,
                    segment_point_b,
                    point_next,
                    intersections,
                    false,
                    vf);

    /* Get intersections with curve end extensions. */
    if (vf->gap_close_extend && vf->extensions_collide_with_curves) {
      intersections.clear();

      /* Note: we deliberately create a copy of the intersection, because we change the values. */
      for (auto intersection : vf->extension_intersections) {
        if (intersection.with_end_extension) {
          continue;
        }

        /* Look for intersections with the current curve segment. */
        if (intersection.curve_index_2d == curve_i &&
            ((intersection.point_start == point_i && intersection.point_end == point_next) ||
             (intersection.point_start == point_next && intersection.point_end == point_i)))
        {
          intersection.curve_index_2d = intersection.extension_index;
          intersection.point_start = 0;
          intersection.point_end = 1;
          std::swap(intersection.distance[0], intersection.distance[1]);
          intersections.append(intersection);
        }
      }

      add_segment_end(&vf->extensions_2d,
                      segment,
                      segment_point_a,
                      segment_point_b,
                      point_next,
                      intersections,
                      true,
                      vf);
    }

    /* When one or more intersections are found, we can stop walking along the curve. */
    if (!segment->segment_ends.is_empty()) {
      /* Sort intersections on distance and angle. This is important to find the narrowest edge. */
      std::sort(segment->segment_ends.begin(),
                segment->segment_ends.end(),
                [](const SegmentEnd &a, const SegmentEnd &b) {
                  if (compare_ff(a.distance, b.distance, DISTANCE_EPSILON)) {
                    if (compare_ff(a.angle_min, b.angle_min, ANGLE_EPSILON)) {
                      return a.angle_max < b.angle_max;
                    }
                    return a.angle_min < b.angle_min;
                  }
                  return a.distance < b.distance;
                });

      /* Set intersection flag. */
      segment->flag |= vfFlag::IsIntersected;

      break;
    }

    /* Walk to next point on curve. */
    segment->flag &= ~vfFlag::CheckStartDistance;
    point_i += direction;

    /* Check closed cycle. */
    if (point_i == point_started) {
      break;
    }
  } while (true);

  /* Set walked point range. */
  segment->flag &= ~vfFlag::CheckStartDistance;
  segment->set_point_range(point_i, direction);

  /* When the edge segment is ended with an intersection, we are done now.
   * If not, we have reached the end of the curve and we will check for gap closures. */
  if (!check_gaps) {
    return true;
  }

  /* Check for gap closure by extending the end of the curve. */
  if (vf->gap_close_extend) {
    /* Get the coordinates of start/end extension. */
    const int extension_index = curve_i * 2 + (direction == 1 ? 1 : 0);
    const int point_offset = vf->extensions_2d.point_offset[extension_index];
    const float2 segment_point_a = vf->extensions_2d.points_2d[point_offset];
    const float2 segment_point_b = vf->extensions_2d.points_2d[point_offset + 1];

    /* Abort when the end extension doesn't intersect with anything. */
    if (!vf->extension_has_intersection[extension_index]) {
      return true;
    }

    /* Find the first intersection of the end extension. The intersection data is already
     * calculated during the interactive step of the modal operator. */
    IntersectingSegment2D intersection;
    for (const auto &isect : vf->extension_intersections) {
      if (isect.extension_index == extension_index) {
        intersection = isect;
        break;
      }
    }

    /* Add intersection to the segment. */
    Vector<IntersectingSegment2D> intersections;
    intersections.append(intersection);

    if (intersection.with_end_extension) {
      /* Intersection with other curve end extension. */
      add_segment_end(&vf->extensions_2d,
                      segment,
                      segment_point_a,
                      segment_point_b,
                      -1,
                      intersections,
                      true,
                      vf);
    }
    else {
      /* Intersection with other curve. */
      add_segment_end(
          &vf->curves_2d, segment, segment_point_a, segment_point_b, -1, intersections, false, vf);
    }

    if (!segment->segment_ends.is_empty()) {
      /* Set gap closure flag. */
      segment->flag |= vfFlag::IsGapClosure;
    }
  }

  /* Check for gap closure with radii at the ends of curves. */
  if (vf->gap_close_radius) {
    /* Get coordinates of curve start/end point. */
    const int curve_point_end = point_offset + point_i;
    const float2 curve_point = vf->curves_2d.points_2d[curve_point_end];
    const bool at_start_of_curve = (point_i == 0);

    /* Get the segment coordinates at the start/end of the curve. */
    const int curve_point_end_prev = curve_point_end +
                                     math::min(point_size - 1, 1) * (at_start_of_curve ? 1 : -1);
    const float2 curve_point_prev = vf->curves_2d.points_2d[curve_point_end_prev];
    const float2 curve_end_vec = curve_point - curve_point_prev;
    const float max_dist = 2 * vf->gap_distance;

    /* Find nearest end points.
     * Note: the number of points is a bit arbitrary, but the defined number will suffice for
     * normal cases. */
    KDTreeNearest_2d nearest[RADIUS_NEAREST_NUM];
    const int nearest_num = BLI_kdtree_2d_find_nearest_n(
        vf->curve_ends, curve_point, nearest, RADIUS_NEAREST_NUM);

    for (int i = 0; i < nearest_num; i++) {
      /* Skip nearest points outside the user-defined closing radius. */
      if (nearest[i].dist > max_dist) {
        continue;
      }

      /* There are (number of curves * 2) points in the KD tree: the first and last point of each
       * curve. So an even KD tree index means a 'first' point, an odd index means a 'last'
       * point. */
      const int nearest_curve = int(nearest[i].index / 2);
      const bool nearest_at_start_of_curve = (nearest[i].index & 1) == 0;
      const int nearest_point_offset = vf->curves_2d.point_offset[nearest_curve];
      const int nearest_point_i = (nearest_at_start_of_curve ?
                                       0 :
                                       vf->curves_2d.point_size[nearest_curve] - 1);

      /* Skip self (the curve end we are gap-closing). */
      if (nearest_curve == curve_i && nearest_at_start_of_curve == at_start_of_curve) {
        continue;
      }
      /* Skip cyclic curves. */
      if (vf->curves_2d.is_cyclic[nearest_curve]) {
        continue;
      }

      /* Set segment end values. */
      SegmentEnd segment_end{};
      segment_end.flag = (nearest_at_start_of_curve ? vfFlag::None : vfFlag::DirectionBackwards);
      segment_end.curve_index_2d = nearest_curve;
      segment_end.point_start = nearest_point_i;

      /* Determine the angle between the curve start/end and the nearest point. For finding the
       * narrowest fill edge, we explore the smallest angle first (= most right turn). */
      const float2 nearest_vec = vf->curves_2d.points_2d[nearest_point_offset + nearest_point_i] -
                                 curve_point;
      segment_end.angle[0] = M_PI - angle_signed_v2v2(curve_end_vec, nearest_vec);
      segment_end.angle_is_set[0] = true;
      segment_end.angle_is_set[1] = false;

      /* Append to segment end list. */
      segment->segment_ends.append(segment_end);
    }

    if (!segment->segment_ends.is_empty()) {
      /* Sort nearest curves on angle. */
      std::sort(segment->segment_ends.begin(),
                segment->segment_ends.end(),
                [](const SegmentEnd &a, const SegmentEnd &b) { return a.angle[0] < b.angle[0]; });

      /* Set gap closure flag. */
      segment->flag |= vfFlag::IsGapClosure;
    }
  }

  return true;
}

/**
 * Get the next turn on an end extension intersection: right or left.
 */
static void take_next_turn_on_extension_intersection(VectorFillData *vf, SegmentEnd *segment_end)
{
  int point_start;
  vfFlag flag = vfFlag::None;

  switch (segment_end->turn) {
    /* Right turn. */
    case 0: {
      point_start = segment_end->point_end;
      if ((segment_end->flag & vfFlag::DirectionBackwards) == true) {
        point_start = segment_end->point_start;
        flag = vfFlag::DirectionBackwards;
      }
      break;
    }
    /* Left turn. */
    case 1: {
      point_start = segment_end->point_start;
      flag = vfFlag::DirectionBackwards;
      if ((segment_end->flag & vfFlag::DirectionBackwards) == true) {
        point_start = segment_end->point_end;
        flag = vfFlag::None;
      }
      break;
    }
    default:
      break;
  }

  /* Add new segment to the fill edge. */
  EdgeSegment segment{};
  segment.curve_index_2d = segment_end->curve_index_2d;
  segment.init_point_range(point_start);
  segment.flag = flag;

  vf->segments.append(segment);
}

/**
 * Get the next turn on a curve intersection.
 */
static void take_next_turn_on_intersection(VectorFillData *vf,
                                           SegmentEnd *segment_end,
                                           const EdgeSegment *ori_segment)
{
  vfSegmentTurn turn[3] = {vfSegmentTurn::None};
  int first_angle_index, second_angle_index, straight_ahead_index;

  /* The most common case is a three-way intersection: right turn, straight ahead and left turn.
   * But we have to handle the edge cases: the intersecting curve on one side only, skipping
   * redundant turns etc. */
  if (segment_end->angle_min > M_PI) {
    /* Intersecting curve entirely on the left side. */
    straight_ahead_index = 0;
    first_angle_index = 1;
    second_angle_index = 2;
  }
  else if (segment_end->angle_max <= M_PI) {
    /* Intersecting curve entirely on the right side. */
    first_angle_index = 0;
    second_angle_index = 1;
    straight_ahead_index = 2;
  }
  else {
    /* Intersecting curve on right and left side. */
    first_angle_index = 0;
    straight_ahead_index = 1;
    second_angle_index = 2;
  }

  /* Set the turns. */
  turn[first_angle_index] = vfSegmentTurn::FirstAngle;
  turn[straight_ahead_index] = vfSegmentTurn::StraightAhead;
  turn[second_angle_index] = vfSegmentTurn::SecondAngle;

  /* Clear the turn when the angle is undefined. */
  vfSegmentTurn current_turn = turn[segment_end->turn];
  if (current_turn == vfSegmentTurn::FirstAngle && !segment_end->angle_is_set[0]) {
    current_turn = vfSegmentTurn::None;
  }
  if (current_turn == vfSegmentTurn::SecondAngle && !segment_end->angle_is_set[1]) {
    current_turn = vfSegmentTurn::None;
  }

  /* Abort when turn is undefined. */
  if (current_turn == vfSegmentTurn::None) {
    return;
  }

  /* Take the turn. */
  int curve_index_2d, point_start;
  vfFlag flag = vfFlag::None;

  switch (current_turn) {
    /* Right turn (cq. smallest angle). */
    case vfSegmentTurn::FirstAngle: {
      curve_index_2d = segment_end->curve_index_2d;
      point_start = segment_end->point_end;
      if ((segment_end->flag & vfFlag::DirectionBackwards) == true) {
        point_start = segment_end->point_start;
        flag = vfFlag::DirectionBackwards;
      }
      break;
    }
    /* Straight ahead, on same curve as we were. */
    case vfSegmentTurn::StraightAhead: {
      curve_index_2d = ori_segment->curve_index_2d;
      point_start = segment_end->ori_segment_point_end;
      if ((ori_segment->flag & vfFlag::DirectionBackwards) == true) {
        flag = vfFlag::DirectionBackwards;
      }
      break;
    }
    /* Left turn (cq. largest angle). */
    case vfSegmentTurn::SecondAngle: {
      curve_index_2d = segment_end->curve_index_2d;
      point_start = segment_end->point_start;
      flag = vfFlag::DirectionBackwards;
      if ((segment_end->flag & vfFlag::DirectionBackwards) == true) {
        point_start = segment_end->point_end;
        flag = vfFlag::None;
      }
      break;
    }
    default:
      break;
  }

  /* Add new segment to the fill edge. */
  EdgeSegment segment{};
  segment.curve_index_2d = curve_index_2d;
  segment.init_point_range(point_start);
  segment.flag = flag;

  vf->segments.append(segment);
}

/**
 * Get the next curve found by gap closure.
 */
static void take_next_gap_closure_curve(VectorFillData *vf, SegmentEnd *segment_end)
{
  /* Add new segment to the fill edge. */
  EdgeSegment segment{};
  segment.curve_index_2d = segment_end->curve_index_2d;
  segment.init_point_range(segment_end->point_start);
  if ((segment_end->flag & vfFlag::DirectionBackwards) == true) {
    segment.flag = vfFlag::DirectionBackwards;
  }

  vf->segments.append(segment);
}

/**
 * Remove the last edge segment. Handles segment sets.
 */
static void remove_last_segment(VectorFillData *vf)
{
  /* Remove segment. */
  EdgeSegment *segment = &vf->segments.last();
  const bool is_part_of_set = ((segment->flag & vfFlag::IsPartOfSet) == true);
  vf->segments.remove_last();

  /* Repeat when segment is part of a set. */
  if (is_part_of_set) {
    remove_last_segment(vf);
  }
}

#ifdef GP_VFILL_DEBUG_VERBOSE
/**
 * Debug function: print edge segment data to the console.
 */
static void debug_print_segment_data(VectorFillData *vf, EdgeSegment *segment)
{
  printf("Segments: %lld, walked along: curve %d (%d) %d-%d, flag %d,%s end segments %lld \n",
         vf->segments.size(),
         segment->curve_index_2d,
         vf->curves_2d.point_size[segment->curve_index_2d],
         segment->point_range[0],
         segment->point_range[1],
         segment->flag,
         (segment->flag & vfFlag::DirectionBackwards) == true ? " backwards," : "",
         segment->segment_ends.size());

  for (auto seg_end : segment->segment_ends) {
    printf("   - segment end: curve %d %d-%d, distance %.3f, angles %.2f %.2f, %s \n",
           seg_end.curve_index_2d,
           seg_end.point_start,
           seg_end.point_end,
           seg_end.distance,
           seg_end.angle_is_set[0] ? seg_end.angle[0] : 9.99f,
           seg_end.angle_is_set[1] ? seg_end.angle[1] : 9.99f,
           (seg_end.flag & vfFlag::DirectionBackwards) == true ? "backwards" : "");
  }
}
#endif

/**
 * Find a closed fill edge, by creating a chain of edge segments (parts of curves).
 * The edge is build in clockwise direction. The narrowest edge is created by taking
 * a right turn first at curve intersections.
 */
static bool find_closed_fill_edge(VectorFillData *vf)
{
#ifdef GP_VFILL_DEBUG_VERBOSE
  int iteration = 0;
#endif

  while (!vf->segments.is_empty()) {

#ifdef GP_VFILL_DEBUG_VERBOSE
    /* Limit number of iterations in debug mode. */
    iteration++;
    if (iteration > 30) {
      return false;
    }
#else
    /* Emergency break: when the operator is taking too much time, jump to the conclusion that no
     * closed fill edge can be found. */
    auto t2 = std::chrono::high_resolution_clock::now();
    auto delta_t = std::chrono::duration_cast<std::chrono::milliseconds>(t2 -
                                                                         vf->operator_time_start);
    if (delta_t.count() > MAX_EXECUTION_TIME) {
      return false;
    }
#endif

    /* Explore the last edge segment in the array. */
    EdgeSegment *segment = &vf->segments.last();

    /* Walk along the curve until an intersection or the end of the curve is reached. */
    bool changed = walk_along_curve(vf, segment);
    segment = &vf->segments.last();

#ifdef GP_VFILL_DEBUG_VERBOSE
    if (changed) {
      debug_print_segment_data(vf, segment);
    }
#endif

    /* Check if the edge is closed. */
    if (changed) {
      if (is_closed_fill_edge(vf)) {
        /* Check if the mouse click position is inside the loop. */
        if (mouse_pos_is_inside_closed_edge(vf)) {
          return true;
        }

        /* Remove the closing segment and continue searching. */
        remove_last_segment(vf);
        continue;
      }
    }

    /* Inpsect the segment ends. */
    if (segment->segment_ends_index < segment->segment_ends.size()) {
      SegmentEnd *segment_end = &segment->segment_ends[segment->segment_ends_index];

      /* Handle segment that ends at an intersection. */
      if ((segment->flag & vfFlag::IsIntersected) == true) {
        /* When all intersection turns are explored, continue with the
         * next intersection (if any). */
        if (segment_end->turn >= 3) {
          segment->segment_ends_index++;
          continue;
        }

        /* Take the next turn on the intersection: right, straight ahead, left. */
        take_next_turn_on_intersection(vf, segment_end, segment);

        segment_end->turn++;
      }
      /* Handle gap closures. */
      else {
        /* End extension intersects with another curve. */
        if (vf->gap_close_extend && (segment_end->flag & vfFlag::WithEndExtension) == false) {
          /* When an end extension intersects with another curve, we have two turns to explore:
           * right and left. */
          if (segment_end->turn >= 2) {
            segment->segment_ends_index++;
            continue;
          }

          take_next_turn_on_extension_intersection(vf, segment_end);
          segment_end->turn++;
        }
        else {
          /* Gap closure with radius or with another curve end extension: jump onto the next
           * gap-closed curve. */
          if (segment_end->turn >= 1) {
            segment->segment_ends_index++;
            continue;
          }

          take_next_gap_closure_curve(vf, segment_end);
          segment_end->turn++;
        }
      }
    }
    else {
      /* We reached a dead end, delete the segment and continue with the previous. */
      remove_last_segment(vf);
    }
  }

  return false;
}

/**
 * Execute the fill operation, at the second mouse click. (The first mouse click is for the
 * interactive gap closure phase.)
 */
static bool vector_fill_do(VectorFillData *vf)
{
  /* Find a first (arbitrary) edge point of the fill area by casting a ray from the mouse click
   * position in one of four directions: up, right, down, left. When this ray crosses a curve,
   * we use the intersection point as the starting point of the fill edge.
   * We check in four directions, because the user can click near a gap in the fill edge. */
  float2 ray_vec;
  vfRayDirection ray_direction;
  vf->operator_time_start = std::chrono::high_resolution_clock::now();
  const float ray_delta = 20000.0f;
  bool found = false;

  for (auto ray_dir : ray_directions) {
    ray_direction = ray_dir;
    ray_vec = vf->mouse_pos;

    switch (ray_dir) {
      case vfRayDirection::Up:
        ray_vec[1] += ray_delta;
        break;
      case vfRayDirection::Right:
        ray_vec[0] += ray_delta;
        break;
      case vfRayDirection::Down:
        ray_vec[1] -= ray_delta;
        break;
      case vfRayDirection::Left:
        ray_vec[0] -= ray_delta;
        break;
      default:
        break;
    }

    vf->starting_segments = get_intersections_of_segment_with_curves(
        vf->mouse_pos, ray_vec, -1, &vf->curves_2d);

    if (vf->starting_segments.size() == 0) {
      continue;
    }

    /* Sort starting segments on distance (closest first). */
    std::sort(vf->starting_segments.begin(),
              vf->starting_segments.end(),
              [](const IntersectingSegment2D &a, const IntersectingSegment2D &b) {
                return a.distance[0] < b.distance[0];
              });

    /* From the first edge point we try to follow the fill edge clockwise, until we find
     * a closed loop. */
    IntersectingSegment2D *start_segment = &vf->starting_segments.first();

    /* Start with an empty edge segment list. */
    vf->segments.clear();

    /* Add first edge segment. */
    EdgeSegment segment{};
    segment.curve_index_2d = start_segment->curve_index_2d;
    segment.init_point_range(start_segment->point_start);

    /* Set the minimum distance for checking intersections on this first segment. */
    segment.flag = vfFlag::CheckStartDistance;
    vf->start_distance = start_segment->distance[1];

    /* We want to follow the fill edge clockwise. Determine in which direction we have to follow
     * the curve for that. */
    const int point_offset = vf->curves_2d.point_offset[segment.curve_index_2d];
    const float2 segment_a = vf->curves_2d.points_2d[point_offset + start_segment->point_start];
    const float2 segment_b = vf->curves_2d.points_2d[point_offset + start_segment->point_end];
    const float delta_x = segment_b[0] - segment_a[0];
    const float delta_y = segment_b[1] - segment_a[1];
    switch (ray_direction) {
      case vfRayDirection::Up:
        if (delta_x < 0) {
          segment.flag |= vfFlag::DirectionBackwards;
        }
        break;
      case vfRayDirection::Right:
        if (delta_y > 0) {
          segment.flag |= vfFlag::DirectionBackwards;
        }
        break;
      case vfRayDirection::Down:
        if (delta_x > 0) {
          segment.flag |= vfFlag::DirectionBackwards;
        }
        break;
      case vfRayDirection::Left:
        if (delta_y < 0) {
          segment.flag |= vfFlag::DirectionBackwards;
        }
        break;
      default:
        break;
    }
    if ((segment.flag & vfFlag::DirectionBackwards) == true) {
      segment.point_range[0] = start_segment->point_end;
      vf->start_distance = 1.0f - vf->start_distance;
    }

    vf->segments.append(segment);

    /* See if we can expand this first edge segment to a closed edge. */
    found = find_closed_fill_edge(vf);
    if (found) {
      break;
    }
  }

  if (found) {
    /* Create the 3D fill curve geometry. */
    create_fill_geometry(vf);
  }

  return found;
}

/**
 * Create a KD tree for the gap closure of curve ends by radius.
 */
static void init_curve_end_radii(VectorFillData *vf)
{
  /* Create KD tree for all curve ends. */
  const int curve_num = vf->curves_2d.point_offset.size();
  vf->curve_ends = BLI_kdtree_2d_new(curve_num * 2);

  /* Add curve ends. */
  int tree_index = 0;
  for (const int curve_i : vf->curves_2d.point_offset.index_range()) {
    /* Add first curve point. */
    int point_i = vf->curves_2d.point_offset[curve_i];
    BLI_kdtree_2d_insert(vf->curve_ends, tree_index, vf->curves_2d.points_2d[point_i]);
    tree_index++;

    /* Add last curve point. */
    point_i += vf->curves_2d.point_size[curve_i] - 1;
    BLI_kdtree_2d_insert(vf->curve_ends, tree_index, vf->curves_2d.points_2d[point_i]);
    tree_index++;
  }

  BLI_kdtree_2d_balance(vf->curve_ends);

  /* Init array with curve end connection flag. */
  vf->connected_by_radius = Array<bool>(curve_num * 2, false);
}

/**
 * Determine which curves ends are connected with each other based on radius.
 */
static void get_connected_curve_end_radii(VectorFillData *vf)
{
  const float max_dist = 2 * vf->gap_distance;

  /* Init connections by radius. */
  vf->connected_by_radius.fill(false);
  vf->radius_connections.clear();

  /* Check all curvess. */
  for (const int curve_i : vf->curves_2d.point_offset.index_range()) {
    /* Skip cyclic curves. */
    if (vf->curves_2d.is_cyclic[curve_i]) {
      continue;
    }

    /* Check both ends. */
    for (int side = 0; side <= 1; side++) {
      const int kdtree_index = curve_i * 2 + side;
      const int point_index = vf->curves_2d.point_offset[curve_i] +
                              (side == 0 ? 0 : vf->curves_2d.point_size[curve_i] - 1);

      /* Find nearest curve end points. */
      KDTreeNearest_2d nearest[RADIUS_NEAREST_NUM];
      const int nearest_num = BLI_kdtree_2d_find_nearest_n(
          vf->curve_ends, vf->curves_2d.points_2d[point_index], nearest, RADIUS_NEAREST_NUM);

      for (int i = 0; i < nearest_num; i++) {
        /* Skip self, already registered curve ends and curve ends out of close radius range. */
        if (nearest[i].index <= kdtree_index || nearest[i].dist > max_dist) {
          continue;
        }

        /* Skip cyclic curves. */
        const int nearest_curve = int(nearest[i].index / 2);
        if (vf->curves_2d.is_cyclic[nearest_curve]) {
          continue;
        }

        /* Flag connection and append to list for drawing. */
        vf->connected_by_radius[kdtree_index] = true;
        vf->connected_by_radius[nearest[i].index] = true;
        vf->radius_connections.append({kdtree_index, nearest[i].index});
      }
    }
  }
}

/**
 * Init the data structure for curve end extensions.
 */
static void init_curve_end_extensions(VectorFillData *vf)
{
  const int curve_num = vf->curves_2d.point_offset.size() * 2;
  const int point_num = curve_num * 2;

  vf->extensions_2d.point_offset = Array<int>(curve_num);
  vf->extensions_2d.point_size = Array<int>(curve_num, 2);
  vf->extensions_2d.is_cyclic = Array<bool>(curve_num, false);
  vf->extensions_2d.drawing_index_2d = Array<int>(curve_num);
  vf->extensions_2d.points_2d = Array<float2>(point_num);
  vf->extensions_2d.curve_bbox = Array<rctf>(curve_num);
  vf->extension_length_ratio = Array<float>(curve_num);
  vf->extension_has_intersection = Array<bool>(curve_num, false);
}

/**
 * Add a curve end extension to the set of extensions.
 */
static void add_curve_end_extension(VectorFillData *vf,
                                    const int curve_i,
                                    const int curve_ext,
                                    const int point_i,
                                    const int next_point_delta,
                                    const int point_i_ext)
{
  /* Set drawing index and point index for extension. */
  vf->extensions_2d.drawing_index_2d[curve_ext] = vf->curves_2d.drawing_index_2d[curve_i];
  vf->extensions_2d.point_offset[curve_ext] = point_i_ext;
  vf->extensions_2d.is_cyclic[curve_ext] = vf->curves_2d.is_cyclic[curve_i];

  /* Use the vector between the two outer points of the curve to calculate the extension
   * coordinates. */
  const float2 co = vf->curves_2d.points_2d[point_i];
  vf->extensions_2d.points_2d[point_i_ext] = co;
  float2 end_vec = co - vf->curves_2d.points_2d[point_i + next_point_delta];
  const float end_length = math::length(end_vec);
  end_vec = math::normalize(end_vec) * vf->gap_distance;
  const float extension_length = math::length(end_vec);
  vf->extensions_2d.points_2d[point_i_ext + 1] = co + end_vec;
  vf->extension_length_ratio[curve_ext] = (end_length == 0.0f ? 0.0f :
                                                                extension_length / end_length);
}

/**
 * Create a set of 2D curve end extensions. Check if the extensions intersect with curves or other
 * end extensions.
 */
static void get_curve_end_extensions(VectorFillData *vf)
{
  /* Create extension curves. */
  for (const int curve_i : vf->curves_2d.point_offset.index_range()) {
    /* Two extensions for each curve. */
    const int curve_ext = curve_i * 2;
    /* Each extension contains two points. */
    const int point_i_ext = curve_ext * 2;
    const int point_size = vf->curves_2d.point_size[curve_i];

    /* Create extension for the first segment of the 2D curve. */
    int point_i = vf->curves_2d.point_offset[curve_i];
    int next_point_delta = (point_size > 1 ? 1 : 0);
    add_curve_end_extension(vf, curve_i, curve_ext, point_i, next_point_delta, point_i_ext);

    /* Create extension for the last segment of the 2D curve. */
    point_i += point_size - 1;
    next_point_delta = (point_size > 1 ? -1 : 0);
    add_curve_end_extension(
        vf, curve_i, curve_ext + 1, point_i, next_point_delta, point_i_ext + 2);

    /* Create bounding boxes for the extensions. */
    BLI_rctf_init_minmax(&vf->extensions_2d.curve_bbox[curve_ext]);
    BLI_rctf_do_minmax_v(&vf->extensions_2d.curve_bbox[curve_ext],
                         vf->extensions_2d.points_2d[point_i_ext]);
    BLI_rctf_do_minmax_v(&vf->extensions_2d.curve_bbox[curve_ext],
                         vf->extensions_2d.points_2d[point_i_ext + 1]);

    BLI_rctf_init_minmax(&vf->extensions_2d.curve_bbox[curve_ext + 1]);
    BLI_rctf_do_minmax_v(&vf->extensions_2d.curve_bbox[curve_ext + 1],
                         vf->extensions_2d.points_2d[point_i_ext + 2]);
    BLI_rctf_do_minmax_v(&vf->extensions_2d.curve_bbox[curve_ext + 1],
                         vf->extensions_2d.points_2d[point_i_ext + 3]);
  }

  /* Clear intersection data. */
  vf->extension_intersections.clear();
  vf->extension_has_intersection.fill(false);

  /* Check intersections of extensions with curves or other end extensions. */
  for (const int extension_index : vf->extensions_2d.point_offset.index_range()) {
    /* Skip end extensions of cyclic curves. */
    if (vf->extensions_2d.is_cyclic[extension_index]) {
      continue;
    }

    const int point_offset = vf->extensions_2d.point_offset[extension_index];
    const float2 segment_point_a = vf->extensions_2d.points_2d[point_offset];
    const float2 segment_point_b = vf->extensions_2d.points_2d[point_offset + 1];

    /* Get intersections with other curve end extensions. */
    Vector<IntersectingSegment2D> all_intersections;
    Vector<IntersectingSegment2D> intersections = get_intersections_of_segment_with_curves(
        segment_point_a, segment_point_b, extension_index, &vf->extensions_2d);

    for (auto intersection : intersections) {
      intersection.with_end_extension = true;
      intersection.extension_index = extension_index;
      all_intersections.append(intersection);
    }

    /* Get intersections with other curves. */
    if (vf->extensions_collide_with_curves) {
      const int curve_i = int(extension_index / 2);
      intersections = get_intersections_of_segment_with_curves(
          segment_point_a, segment_point_b, curve_i, &vf->curves_2d);
      for (auto intersection : intersections) {
        intersection.with_end_extension = false;
        intersection.extension_index = extension_index;
        all_intersections.append(intersection);
      }
    }

    /* Sort intersections on distance. */
    if (!all_intersections.is_empty()) {
      std::sort(all_intersections.begin(),
                all_intersections.end(),
                [](const IntersectingSegment2D &a, const IntersectingSegment2D &b) {
                  return a.distance[0] < b.distance[0];
                });

      /* Add closest result(s) to list of extension intersections. */
      const float shortest_dist = all_intersections.first().distance[0];
      for (const auto &intersection : all_intersections) {
        if ((intersection.distance[0] - shortest_dist) > DISTANCE_EPSILON) {
          break;
        }
        vf->extension_intersections.append(intersection);
        vf->extension_has_intersection[intersection.extension_index] = true;
      }
    }
  }
}

/**
 * Get a slight extension for each curve segment, based on the normalized vector of each
 * curve point to the next.
 */
static void get_curve_segment_epsilons(VectorFillData *vf)
{
  vf->curve_segment_epsilon = Array<float2>(vf->curves_2d.points_2d.size());

  threading::parallel_for(
      vf->curves_2d.point_offset.index_range(), 256, [&](const IndexRange range) {
        for (const int curve_i : range) {
          const int point_last = vf->curves_2d.point_offset[curve_i] +
                                 vf->curves_2d.point_size[curve_i] - 1;
          for (const int point_i :
               IndexRange(vf->curves_2d.point_offset[curve_i], vf->curves_2d.point_size[curve_i]))
          {
            const int point_next = (point_i == point_last ? vf->curves_2d.point_offset[curve_i] :
                                                            point_i + 1);
            vf->curve_segment_epsilon[point_i] = math::normalize(
                                                     vf->curves_2d.points_2d[point_next] -
                                                     vf->curves_2d.points_2d[point_i]) *
                                                 DISTANCE_EPSILON;
          }
        }
      });
}

#ifdef GP_VFILL_DEBUG_VERBOSE
/**
 * Debug function: show curve indices in the viewport.
 */
static void debug_draw_curve_indices(VectorFillData *vf)
{
  const int font_id = BLF_default();
  const uiStyle *style = UI_style_get();
  BLF_size(font_id, style->widget.points * UI_SCALE_FAC * 0.8);

  /* Draw point indices. */
  BLF_color4fv(font_id, float4{0.0f, 0.0f, 0.0f, 0.5f});
  for (const int curve_i : vf->curves_2d.point_offset.index_range()) {
    const int point_offset = vf->curves_2d.point_offset[curve_i];
    const int step = 5;
    float x_prev = FLT_MAX;
    float y_prev = FLT_MAX;
    for (int point_i = step; point_i < vf->curves_2d.point_size[curve_i]; point_i += step) {
      const std::string str = std::to_string(point_i) + "-" + std::to_string(curve_i);
      const char *text = str.c_str();

      const float x = vf->curves_2d.points_2d[point_offset + point_i][0] - strlen(text) * 5.4;
      const float y = vf->curves_2d.points_2d[point_offset + point_i][1] + 1;

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

  for (const int curve_i : vf->curves_2d.point_offset.index_range()) {
    const std::string str = std::to_string(curve_i);
    const char *text = str.c_str();

    const int point_offset = vf->curves_2d.point_offset[curve_i];
    const float x = vf->curves_2d.points_2d[point_offset][0] - strlen(text) * 10;
    const float y = vf->curves_2d.points_2d[point_offset][1] + 2;
    BLF_position(font_id, x, y, 0);

    BLF_draw(font_id, text, strlen(text));
  }

  BLF_disable(font_id, BLF_BOLD);
}
#endif

/**
 * Get curve point index given an end extension index.
 */
static int get_curve_point_by_end_index(VectorFillData *vf, const int end_index)
{
  const int curve_i = int(end_index / 2);
  int point_i = vf->curves_2d.point_offset[curve_i];
  if ((end_index & 1) == 1) {
    point_i += vf->curves_2d.point_size[curve_i] - 1;
  }
  return point_i;
}

/**
 * Draw gap closure lines on an overlay in the 3D viewport.
 */
static void draw_overlay(const bContext * /*C*/, ARegion *region, void *arg)
{
  VectorFillData *vf = static_cast<VectorFillData *>(arg);

  /* Draw only in the region that originated the operator. */
  if (region != vf->vc.region) {
    return;
  }

  /* Anything to draw? */
  if (!(vf->gap_close_extend || vf->gap_close_radius)) {
    return;
  }

  const uint shdr_pos = GPU_vertformat_attr_add(
      immVertexFormat(), "pos", GPU_COMP_F32, 2, GPU_FETCH_FLOAT);
  immBindBuiltinProgram(GPU_SHADER_3D_UNIFORM_COLOR);

  GPU_line_width(1.5f);
  GPU_blend(GPU_BLEND_ALPHA);
  GPU_line_smooth(true);

  /* Draw curve end extensions. */
  if (vf->gap_close_extend) {
    /* Draw extensions that don't intersect anything. */
    immUniformColor4f(1.0f, 0.0f, 0.5f, 1.0f);
    for (const int ext_i : vf->extensions_2d.point_offset.index_range()) {
      /* Skip intersected extensions or extensions of cyclic curves. */
      if (vf->extension_has_intersection[ext_i] || vf->extensions_2d.is_cyclic[ext_i]) {
        continue;
      }

      const int point_i = vf->extensions_2d.point_offset[ext_i];

      immBegin(GPU_PRIM_LINES, 2);
      immVertex2fv(shdr_pos, vf->extensions_2d.points_2d[point_i]);
      immVertex2fv(shdr_pos, vf->extensions_2d.points_2d[point_i + 1]);
      immEnd();
    }

    /* Draw extensions that intersect a curve or other extension. */
    immUniformColor4f(0.0f, 1.0f, 1.0f, 1.0f);
    for (const auto &intersection : vf->extension_intersections) {
      const int ext_i = intersection.extension_index;
      const int point_i = vf->extensions_2d.point_offset[ext_i];

      /* Limit the end extension when it is intersecting something else. */
      const float distance = intersection.distance[0];
      const float2 p0 = vf->extensions_2d.points_2d[point_i];
      const float2 p1 = vf->extensions_2d.points_2d[point_i + 1];
      const float2 p_vec = p1 - p0;

      immBegin(GPU_PRIM_LINES, 2);
      immVertex2fv(shdr_pos, p0);
      immVertex2fv(shdr_pos, p0 + p_vec * distance);
      immEnd();
    }
  }

  /* Draw curve end radii. */
  if (vf->gap_close_radius) {
    /* TODO: use color from UI setting. */
    immUniformColor4f(0.0f, 1.0f, 1.0f, 1.0f);

    /* Draw connected curve ends. */
    for (const int2 &connection : vf->radius_connections) {
      const int point_i0 = get_curve_point_by_end_index(vf, connection[0]);
      const int point_i1 = get_curve_point_by_end_index(vf, connection[1]);
      immBegin(GPU_PRIM_LINES, 2);
      immVertex2fv(shdr_pos, vf->curves_2d.points_2d[point_i0]);
      immVertex2fv(shdr_pos, vf->curves_2d.points_2d[point_i1]);
      immEnd();
    }

    /* TODO: use color from UI setting. */
    immUniformColor4f(1.0f, 0.0f, 0.0f, 1.0f);
    GPU_line_width(2.0f);

    /* Draw unconnected curve end radii. */
    int curve_end_index = -1;
    for (const bool connected : vf->connected_by_radius) {
      curve_end_index++;
      if (connected) {
        continue;
      }

      /* Skip ends of cyclic curves. */
      const int curve_i = int(curve_end_index / 2);
      if (vf->curves_2d.is_cyclic[curve_i]) {
        continue;
      }

      /* Draw radius. */
      const int point_i = get_curve_point_by_end_index(vf, curve_end_index);
      imm_draw_circle_wire_2d(shdr_pos,
                              vf->curves_2d.points_2d[point_i][0],
                              vf->curves_2d.points_2d[point_i][1],
                              vf->gap_distance,
                              40);
    }
  }

  immUnbindProgram();

  GPU_line_width(1.0f);
  GPU_line_smooth(false);
  GPU_blend(GPU_BLEND_NONE);

#ifdef GP_VFILL_DEBUG_VERBOSE
  debug_draw_curve_indices(vf);
#endif
}

/**
 * Update the gap closure connections when the radius changed.
 */
static void update_gap_distance(VectorFillData *vf, const float delta)
{
  vf->brush->gpencil_settings->fill_extend_fac += delta;
  vf->brush->gpencil_settings->fill_extend_fac = math::max(
      0.0f, math::min(10.0f, vf->brush->gpencil_settings->fill_extend_fac));
  vf->gap_distance = vf->brush->gpencil_settings->fill_extend_fac * GAP_PIXEL_FACTOR;
  if (vf->gap_close_extend) {
    get_curve_end_extensions(vf);
  }
  if (vf->gap_close_radius) {
    get_connected_curve_end_radii(vf);
  }
  ED_region_tag_redraw(vf->vc.region);
}

/**
 * Get latest tool settings before executing the actual fill.
 */
static void get_latest_toolsettings(VectorFillData *vf)
{
  const bool use_gap_closing = (vf->brush->gpencil_settings->fill_extend_fac > FLT_EPSILON);
  vf->gap_close_extend = (vf->brush->gpencil_settings->fill_extend_mode == GP_FILL_EMODE_EXTEND) &&
                         use_gap_closing;
  vf->gap_close_radius = (vf->brush->gpencil_settings->fill_extend_mode == GP_FILL_EMODE_RADIUS) &&
                         use_gap_closing;
  vf->gap_distance = vf->brush->gpencil_settings->fill_extend_fac * GAP_PIXEL_FACTOR;
}

/**
 * Get a list of layers used for the fill edge detection. The list is based on the 'Layers' field
 * in the tool settings.
 */
static void get_fill_edge_layers(VectorFillData *vf,
                                 Vector<GreasePencilDrawing *> &r_drawings,
                                 Vector<int> &r_drawing_indices)
{
  using namespace bke::greasepencil;

  /* Find index of active layer. */
  int active_layer_index = -1;
  const Layer *active_layer = vf->grease_pencil->get_active_layer();
  Span<const Layer *> layers = vf->grease_pencil->layers();
  for (int index = 0; index < layers.size(); index++) {
    if (layers[index] == active_layer) {
      active_layer_index = index;
      break;
    }
  }

  /* Select layers based on position in the layer collection. */
  Span<GreasePencilDrawingBase *> drawings = vf->grease_pencil->drawings();
  for (int index = 0; index < layers.size(); index++) {
    /* Skip invisible layers. */
    if (!layers[index]->is_visible()) {
      continue;
    }

    bool add = false;
    switch (vf->brush->gpencil_settings->fill_layer_mode) {
      case GP_FILL_GPLMODE_ACTIVE:
        add = (index == active_layer_index);
        break;
      case GP_FILL_GPLMODE_ABOVE:
        add = (index == active_layer_index + 1);
        break;
      case GP_FILL_GPLMODE_BELOW:
        add = (index == active_layer_index - 1);
        break;
      case GP_FILL_GPLMODE_ALL_ABOVE:
        add = (index > active_layer_index);
        break;
      case GP_FILL_GPLMODE_ALL_BELOW:
        add = (index < active_layer_index);
        break;
      case GP_FILL_GPLMODE_VISIBLE:
        add = true;
      default:
        break;
    }

    if (add) {
      const int drawing_index = layers[index]->drawing_index_at(vf->vc.scene->r.cfra);
      if (drawing_index == -1) {
        continue;
      }
      GreasePencilDrawingBase *drawing_base = drawings[drawing_index];
      if (drawing_base->type == GP_DRAWING) {
        GreasePencilDrawing *drawing = reinterpret_cast<GreasePencilDrawing *>(drawing_base);
        r_drawings.append(drawing);
        r_drawing_indices.append(drawing_index);
      }
    }
  }
}

/**
 * Initialize the fill operator data.
 */
static bool init(bContext *C, wmOperator *op)
{
  VectorFillData *vf = static_cast<VectorFillData *>(op->customdata);

  /* Get view context. */
  Depsgraph *depsgraph = CTX_data_ensure_evaluated_depsgraph(C);
  ED_view3d_viewcontext_init(C, &vf->vc, depsgraph);

  /* Get active GP object. */
  vf->grease_pencil = static_cast<GreasePencil *>(vf->vc.obact->data);

  /* Get tool brush. */
  ToolSettings *ts = CTX_data_tool_settings(C);
  vf->brush = BKE_paint_brush(&ts->gp_paint->paint);

  /* Init vector fill flags. */
  vf->gap_close_extend = (vf->brush->gpencil_settings->fill_extend_mode == GP_FILL_EMODE_EXTEND);
  vf->gap_close_radius = (vf->brush->gpencil_settings->fill_extend_mode == GP_FILL_EMODE_RADIUS);
  vf->extensions_collide_with_curves = (vf->brush->gpencil_settings->flag &
                                        GP_BRUSH_FILL_STROKE_COLLIDE) != 0;
  vf->gap_distance = vf->brush->gpencil_settings->fill_extend_fac * GAP_PIXEL_FACTOR;
  vf->wait_for_release = true;
  vf->wait_event_type = LEFTMOUSE;

  /* Get layers according to tool settings (visible, above, below, etc.) */
  Vector<GreasePencilDrawing *> drawings;
  Vector<int> drawing_indices;

  get_fill_edge_layers(vf, drawings, drawing_indices);
  if (drawings.is_empty()) {
    return false;
  }

  /* Convert curves to viewport 2D space. */
  vf->curves_2d = curves_in_2d_space_get(&vf->vc, vf->vc.obact, drawings, drawing_indices);

  /* Calculate epsilon values of all 2D curve segments, used to avoid floating point precision
   * errors. */
  get_curve_segment_epsilons(vf);

  /* When using extensions of the curve ends to close gaps, build an array of those
   * two-point 'curves'. */
  if (vf->gap_close_extend) {
    init_curve_end_extensions(vf);
    get_curve_end_extensions(vf);
  }

  /* When using radii to close gaps, build KD tree of curve end points. */
  if (vf->gap_close_radius) {
    init_curve_end_radii(vf);
    get_connected_curve_end_radii(vf);
  }

  /* Activate 3D viewport overlay for showing gap closure visual aids. */
  if ((vf->brush->gpencil_settings->flag & GP_BRUSH_FILL_SHOW_EXTENDLINES) != 0) {
    vf->draw_handle = ED_region_draw_cb_activate(
        vf->vc.region->type, draw_overlay, vf, REGION_DRAW_POST_PIXEL);
    ED_region_tag_redraw(vf->vc.region);
  }

  return true;
}

/**
 * Clean up the fill operator data.
 */
static void exit(bContext *C, wmOperator *op)
{
  WM_cursor_modal_restore(CTX_wm_window(C));

  if (op->customdata != nullptr) {
    VectorFillData *vf = static_cast<VectorFillData *>(op->customdata);
    if (vf->draw_handle) {
      ED_region_draw_cb_exit(vf->vc.region->type, vf->draw_handle);
      ED_region_tag_redraw(vf->vc.region);
    }
    if (vf->curve_ends) {
      BLI_kdtree_2d_free(vf->curve_ends);
    }
    MEM_delete(vf);
    op->customdata = nullptr;
  }
}

/**
 * Modal handler for the fill operator:
 * - Change gap closure radius
 * - Perform the fill at second mouse click
 */
static int modal(bContext *C, wmOperator *op, const wmEvent *event)
{
  VectorFillData *vf = static_cast<VectorFillData *>(op->customdata);
  int modal_state = OPERATOR_RUNNING_MODAL;

  /* Prevent repeating event keys, when not released yet. */
  if (vf->wait_for_release && vf->wait_event_type == event->type) {
    if (event->val == KM_RELEASE) {
      vf->wait_for_release = false;
    }
    return modal_state;
  }

  switch (event->type) {
    case EVT_ESCKEY:
    case RIGHTMOUSE:
      modal_state = OPERATOR_CANCELLED;
      break;

    case EVT_PAGEUPKEY:
    case WHEELUPMOUSE:
      update_gap_distance(vf, (event->modifier & KM_SHIFT) ? 0.01f : 0.1f);
      break;
    case EVT_PAGEDOWNKEY:
    case WHEELDOWNMOUSE:
      update_gap_distance(vf, (event->modifier & KM_SHIFT) ? -0.01f : -0.1f);
      break;

    case LEFTMOUSE: {
      /* Get mouse position of second click (the 'fill' click). */
      vf->mouse_pos[0] = float(event->mval[0]);
      vf->mouse_pos[1] = float(event->mval[1]);

      /* DEBUG: measure time. */
      auto t1 = std::chrono::high_resolution_clock::now();

      /* Get latest toolsetting values. */
      get_latest_toolsettings(vf);

      /* Perform the fill operation. */
      const bool succes = vector_fill_do(vf);

      /* DEBUG: measure time. */
      auto t2 = std::chrono::high_resolution_clock::now();
      auto delta_t = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
      printf("Vector fill took %d ms.\n", int(delta_t.count()));

      if (succes) {
        modal_state = OPERATOR_FINISHED;
      }
      else {
        BKE_report(op->reports, RPT_INFO, "Unable to fill unclosed area");
        modal_state = OPERATOR_CANCELLED;
      }
      break;
    }
    default:
      break;
  }

  switch (modal_state) {
    case OPERATOR_FINISHED:
      exit(C, op);
      WM_event_add_notifier(C, NC_GPENCIL | NA_EDITED, nullptr);
      break;

    case OPERATOR_CANCELLED:
      exit(C, op);
      break;

    default:
      break;
  }

  return modal_state;
}

static void cancel(bContext *C, wmOperator *op)
{
  exit(C, op);
}

/**
 * Invoke the fill operator at first mouse click in the viewport.
 */
static int invoke(bContext *C, wmOperator *op, const wmEvent * /*event*/)
{
  const Scene *scene = CTX_data_scene(C);
  const Object *object = CTX_data_active_object(C);
  const GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object->data);

  /* Check active layer. */
  if (!grease_pencil.has_active_layer()) {
    BKE_report(op->reports, RPT_ERROR, "No active Grease Pencil layer");
    return OPERATOR_CANCELLED;
  }

  /* Check if layer is editable. */
  const bke::greasepencil::Layer *active_layer = grease_pencil.get_active_layer();
  if (!active_layer->is_editable()) {
    BKE_report(op->reports, RPT_ERROR, "Grease Pencil layer is not editable");
    return OPERATOR_CANCELLED;
  }

  /* Check active frame. */
  if (active_layer->drawing_index_at(scene->r.cfra) == -1) {
    if (!IS_AUTOKEY_ON(scene)) {
      BKE_report(op->reports, RPT_ERROR, "No Grease Pencil frame to draw on");
      return OPERATOR_CANCELLED;
    }
  }

  /* Init tool data. */
  VectorFillData *vf = MEM_new<VectorFillData>(__func__);
  op->customdata = vf;
  if (!init(C, op)) {
    exit(C, op);
    BKE_report(
        op->reports,
        RPT_ERROR,
        "No Grease Pencil layers with edge strokes found, see 'Layers' in Advanced options");
    return OPERATOR_CANCELLED;
  }

  /* Add modal handler. */
  WM_event_add_modal_handler(C, op);

  /* Set cursor. */
  WM_cursor_modal_set(CTX_wm_window(C), WM_CURSOR_PAINT_BRUSH);

  return OPERATOR_RUNNING_MODAL;
}

/**
 * Definition of the vector fill operator.
 */
static void GREASE_PENCIL_OT_vector_based_fill(wmOperatorType *ot)
{
  ot->name = "Fill (vector based)";
  ot->idname = __func__;
  ot->description = "Fill a shape formed by strokes";

  ot->poll = active_grease_pencil_poll;
  ot->invoke = invoke;
  ot->modal = modal;
  ot->cancel = cancel;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

/** \} */

}  // namespace blender::ed::greasepencil::vectorfill

void ED_operatortypes_grease_pencil_fill()
{
  using namespace blender::ed::greasepencil::vectorfill;

  WM_operatortype_append(GREASE_PENCIL_OT_vector_based_fill);
}
