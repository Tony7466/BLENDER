/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgreasepencil
 */

#pragma once

#include "BLI_math_vector_types.hh"
#include "BLI_sys_types.h"

#include "BKE_customdata.hh"

namespace blender::ed::greasepencil::fill {

/* Margin for angles to be considered equal. */
static constexpr float ANGLE_EPSILON = 0.005f;
/* Maximum execution time of vector fill operator (in milliseconds). */
static constexpr unsigned int MAX_EXECUTION_TIME = 500;

/* Fill segment and segment end flags. */
enum class vfFlag : uint16_t {
  None = 0,
  /* Segment is inspected on intersections and gap closures. */
  IsInspected = 1 << 0,
  /* When backwards, point range of segment is from high to low. */
  DirectionBackwards = 1 << 1,
  /* Segment is intersected by other curve (or curve end extension.) */
  IsIntersected = 1 << 2,
  /* Segment is part of a set (two segments of cyclic curve, or overlapping tail segments). */
  IsPartOfSet = 1 << 3,
  /* Segment ends with a gap closure extension to another curve. */
  GapClosureByExtension = 1 << 4,
  /* Segment end with a gap closure by proximity to another curve. */
  GapClosureByProximity = 1 << 5,
  /* Segment end connects with end extension of other curve. */
  WithEndExtension = 1 << 6,
  /* For the first segment only: ignore intersections before the start distance. */
  CheckStartDistance = 1 << 7,
  /* Segment curve has stroke material. */
  HasStroke = 1 << 8,
  /* Segment is inspected on proximity of curves. */
  ProximityInspected = 1 << 9,
  /* Segment is part of the unused head of a closed fill edge. */
  IsUnused = 1 << 10,
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

/* Proximity curve. Created when searching for curves-in-proximity of fill edge segments. */
struct ProximityCurve {
  /* Curve index in #Curves2DSpace struct. */
  int curve_index_2d;
  /* Start point of the curve in proximity (relative index: 0..<curve_point_size>). */
  int curve_point_start;
  /* End point of the curve in proximity (relative index: 0..<curve_point_size>). */
  int curve_point_end;
  /* Start point in the segment. */
  int segment_point_start;
  /* End point in the segment. */
  int segment_point_end;
  /* Flag for connecting the segment end point to the proximity start point in the fill edge.*/
  bool use_segment_point_end;
  /* Flag when the start point of the proximity curve is nearer to the segment than the
   * end point. */
  bool curve_point_start_is_nearest;
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
  /* Start point index of the original segment that is intersected. */
  int ori_segment_point_start{};
  /* End point index of the original segment that is intersected. */
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

/* Fill edge segment. An edge segment is a set of points on an existing curve. A fill edge is a
 * closed set of edge segments. */
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

/* Intersecting curve. */
struct IntersectingCurve {
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
  /* Curve has stroke material. */
  bool has_stroke = true;
};

/* Curve segment that overlaps with another curve segment. An overlap is a series of consecutive
 * points in a curve that have exactly the same position as the points in the other curve. */
struct OverlappingSegment {
  int2 base_point_range{};
  int overlapping_curve_index{};
  int2 overlapping_point_range{};
  bool overlapping_backwards = false;
};

/* Runtime vector fill data. */
struct VectorFillData {
  /* Starting time of the vector fill operator. */
  std::chrono::high_resolution_clock::time_point operator_time_start;

  /* The initial curve segments from which we try to find a closed fill edge. */
  Vector<IntersectingCurve> starting_segments;
  /* The curve segments that will form a closed fill edge. */
  Vector<EdgeSegment> segments;
  /* Overlapping curve segments of the last created edge segment. */
  Vector<OverlappingSegment> overlapping_segments;

  /* The intersection distance of the start segment (found by the casted ray). */
  float start_distance{};
};

}  // namespace blender::ed::greasepencil::fill
