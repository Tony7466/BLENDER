/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgreasepencil
 */

#include "DNA_brush_types.h"
#include "DNA_meshdata_types.h"

#include "BKE_brush.h"
#include "BKE_context.h"
#include "BKE_grease_pencil.hh"
#include "BKE_report.h"

#include "BLI_kdtree.h"
#include "BLI_math_vector.h"
#include "BLI_math_vector.hh"

#include "ED_curves.hh"
#include "ED_grease_pencil.h"
#include "ED_screen.h"
#include "ED_view3d.h"

#include "WM_api.h"

namespace blender::ed::greasepencil::vectorfill {

/*****************************************************************************************************
 * Vector fill data structure.
 *****************************************************************************************************/

/* Number of nearest neighbours when looking for gap closure by curve end radius. */
#define GP_VFILL_RADIUS_NEAREST_NUM 12

enum class vfFlag : uint8_t {
  None = 0,
  IsInspected = 1 << 0,
  DirectionBackwards = 1 << 1,
  IsCyclic = 1 << 2,
  IsIntersected = 1 << 3,
  IsGapClosure = 1 << 4,
  IsLastOfCycle = 1 << 5,
  IsUnused = 1 << 6,
};
ENUM_OPERATORS(vfFlag, vfFlag::IsUnused);
inline constexpr bool operator==(vfFlag a, bool b)
{
  return (uint64_t(a) != 0) == b;
}

struct SegmentEnd {
  /* Curve index (in `Curves2DSpace` struct) of the intersecting/gap closing curve. */
  int curve_index_2d{};
  /* Curve point index of the intersection start point. */
  int point_start{};
  /* Curve point index of the intersection end point. */
  int point_end{};
  /* Curve end point index of the original curve that is intersected. */
  int ori_segment_point_end{};
  /* Distance of intersection (also used for angle of radius gap closure). */
  float distance{};
  /* Segment flags. */
  vfFlag flag = vfFlag::None;
  /* Turn to take at intersections: 0 = right turn, 1 = straight ahead, 2 = left turn. */
  short turn{};
};

struct FillSegment {
  /* Curve index in the `Curves2DSpace` struct (with curve points in 2D space). */
  int curve_index_2d{};
  /* Curve point index range. */
  int2 point_range{};
  /* Segment flags. */
  vfFlag flag = vfFlag::None;

  /* Segment endings: intersection data or gap closure data. */
  Vector<SegmentEnd> segment_ends{};
  /* Last handled segment end. */
  int segment_ends_index{};

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

struct VectorFillData {
  /* Count fill mouse clicks: first click is for showing gap closures, second is for the fill
   * operation itself. */
  bool is_first_click{};
  /* True if the 3D viewport overlay for showing gap closures is initialized. */
  bool overlay_initialized{};

  /* Mouse position of the fill operation click. */
  float2 mouse_pos{};
  /* True when edge gaps are closed by extending the curve ends. */
  bool gap_close_extend{};
  /* True when edge gaps are closed by curve end radii. */
  bool gap_close_radius{};
  /* The distance for gap closure. */
  float gap_distance{};
  /* Radius of the fill stroke. */
  float stroke_radius{};

  GreasePencil *grease_pencil;
  ViewContext vc;

  /* Curve points converted to viewport 2D space. */
  Curves2DSpace curves_2d{};
  /* Curve end extensions in viewport 2D space. */
  Curves2DSpace extensions_2d{};
  /* KD tree of curve ends in 2D space, used for gap closure by radius. */
  KDTree_2d *curve_ends;

  /* The initial curve segments from which we try to find a closed fill edge. */
  Vector<IntersectingSegment2D> starting_segments;
  /* The curve segments that will form a closed fill edge. */
  Vector<FillSegment> segments;
};

/*****************************************************************************************************
 * Vector fill functions.
 *****************************************************************************************************/

Vector<float3> get_closed_fill_edge_as_3d_points(VectorFillData *vf)
{
  Vector<float3> edge_points;
  Vector<int> intersection_indices;
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
    const int range_index_first = (direction == 1 ? 0 : 1);
    const int range_index_last = 1 - range_index_first;

    const int point_last = segment.point_range[range_index_last] + direction;
    for (int point_i = segment.point_range[range_index_first]; point_i != point_last;
         point_i += direction)
    {
      edge_points.append(positions[point_offset + point_i]);
      edge_point_index++;
    }

    /* Calculate regular intersection point. */
    if ((segment.flag & vfFlag::IsIntersected) == true) {
      float3 isect_point{0.0f};
      SegmentEnd *segment_end = &segment.segment_ends[segment.segment_ends_index];
      isect_point = edge_points.last() +
                    (positions[point_offset + segment_end->ori_segment_point_end] -
                     edge_points.last()) *
                        segment_end->distance;
      edge_points.append(isect_point);
      edge_point_index++;
    }

    /* For intersections with curve end extensions it's easier to calculate them afterwards,
     * because we have all the relevant 3D coordinates at hand then. For now we reserve an empty
     * entry.*/
    if (vf->gap_close_extend && (segment.flag & vfFlag::IsGapClosure) == true) {
      intersection_indices.append(edge_point_index);
      edge_points.append(float3(0.0f));
      edge_point_index++;
    }
  }

  /* Calculate intersection points for curve extensions. */
  if (intersection_indices.size() > 0) {
    int isect_index = 0;
    for (auto &segment : vf->segments) {
      if ((segment.flag & vfFlag::IsUnused) == true) {
        continue;
      }

      if ((segment.flag & vfFlag::IsGapClosure) == true) {
        /* Extend end of the curve. */
        SegmentEnd *segment_end = &segment.segment_ends[segment.segment_ends_index];
        const int point_index = intersection_indices[isect_index];
        isect_index++;

        float3 isect_point = edge_points[point_index] +
                             (edge_points[point_index] - edge_points[point_index - 1]) *
                                 segment_end->distance;
        edge_points[point_index + 1] = isect_point;
      }
    }
  }

  return edge_points;
}

static void create_fill_geometry(VectorFillData *vf)
{
  /* Get the edge points in 3D space. */
  Vector<float3> fill_points = get_closed_fill_edge_as_3d_points(vf);

  /* TODO: create geometry. */
}

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
    if (isect.kind == isect.LINE_LINE_CROSS) {
      count++;
    }
  }

  return (count & 1) != 0;
}

Vector<float2> get_closed_fill_edge_as_2d_polygon(VectorFillData *vf)
{
  Vector<float2> points;

  /* We cut some corners here, literally, because we don't calculate the exact intersection point
   * of intersections. Since the 2D polygon is only used for a mouse-position-inside-polygon check,
   * we can afford to be a little sloppy. */
  for (auto &segment : vf->segments) {
    if ((segment.flag & vfFlag::IsUnused) == true) {
      continue;
    }

    const int point_offset = vf->curves_2d.point_offset[segment.curve_index_2d];
    const int direction = ((segment.flag & vfFlag::DirectionBackwards) == true) ? -1 : 1;
    const int range_index_first = (direction == 1 ? 0 : 1);
    const int range_index_last = 1 - range_index_first;

    const int point_last = segment.point_range[range_index_last] + direction;
    for (int point_i = segment.point_range[range_index_first]; point_i != point_last;
         point_i += direction)
    {
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

static void remove_overlapping_edge_tail_points(const FillSegment *head,
                                                FillSegment *tail,
                                                const int2 &range_head,
                                                int2 &range_tail)
{
  /* Get head starting point. */
  const int head_start = ((head->flag & vfFlag::DirectionBackwards) == true) ? range_head[1] :
                                                                               range_head[0];

  /* Limit tail to head starting point. */
  if ((tail->flag & vfFlag::DirectionBackwards) == true) {
    range_tail[0] = head_start;
  }
  else {
    range_tail[1] = head_start;
  }
}

static bool segments_have_overlap(FillSegment *segment, FillSegment *tail)
{
  /* Same curve? */
  if (segment->curve_index_2d != tail->curve_index_2d) {
    return false;
  }

  /* Check overlap in curve points. */
  if (segment->point_range[0] <= tail->point_range[1] &&
      segment->point_range[1] >= tail->point_range[0])
  {
    remove_overlapping_edge_tail_points(segment, tail, segment->point_range, tail->point_range);
    return true;
  }

  return false;
}

static bool is_closed_fill_edge(VectorFillData *vf)
{
  /* Init: by default all segments are used in the edge. */
  for (auto &segment : vf->segments) {
    segment.flag &= ~vfFlag::IsUnused;
  }

  /* Edge case: when there are one or two segment with a closed, cyclic loop, the fill edge is
   * closed. */
  FillSegment *last_segment = &vf->segments.last();
  if (vf->segments.size() == 1 && (last_segment->flag & vfFlag::IsCyclic) == true) {
    return true;
  }
  if (vf->segments.size() == 2 && (last_segment->flag & vfFlag::IsCyclic) == true &&
      (last_segment->flag & vfFlag::IsLastOfCycle) == true)
  {
    return true;
  }

  /* Loop through the edge segments and see if there is overlap with the last one. */
  for (const int segment_i : vf->segments.index_range().drop_back(1)) {
    if (segments_have_overlap(&vf->segments[segment_i], last_segment)) {
      return true;
    }
    vf->segments[segment_i].flag |= vfFlag::IsUnused;
  }

  return false;
}

static void add_segment_end(const Curves2DSpace *curves_2d,
                            FillSegment *segment,
                            const float2 &segment_point_a,
                            const float2 &segment_point_b,
                            const int segment_point_b_index,
                            const Vector<IntersectingSegment2D> &intersections)
{
  const float2 segment_vec = segment_point_b - segment_point_a;

  for (auto &intersection : intersections) {
    /* Copy intersection data. */
    SegmentEnd segment_end{};
    segment_end.curve_index_2d = intersection.curve_index_2d;
    segment_end.point_start = intersection.point_start;
    segment_end.point_end = intersection.point_end;
    segment_end.ori_segment_point_end = segment_point_b_index;
    segment_end.distance = intersection.distance;

    /* Since we follow the fill edge clockwise, at each intersection we want to take a right turn
     * first (for finding the shortest path).
     * So we have to determine the direction of the intersecting curve, because then we know what
     * the right turn is. */
    const int isect_offset = curves_2d->curve_offset[segment_end.curve_index_2d];
    const float2 isect_point_start = curves_2d->points_2d[isect_offset + segment_end.point_start];
    const bool point_start_is_on_right = cross_v2v2(segment_vec,
                                                    isect_point_start - segment_point_a) < 0.0f;
    if (point_start_is_on_right) {
      segment_end.flag |= vfFlag::DirectionBackwards;
    }

    /* Append to segment end list. */
    segment->segment_ends.append(segment_end);
  }
}

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

static bool walk_along_curve(VectorFillData *vf, FillSegment *r_segment)
{
  if ((r_segment->flag & vfFlag::IsInspected) == true) {
    return false;
  }
  r_segment->flag |= vfFlag::IsInspected;

  /* Walk along the curve until an intersection or the end of the curve is reached. */
  const int curve_i = r_segment->curve_index_2d;
  const bool is_cyclic = vf->curves_2d.is_cyclic[curve_i];
  const int direction = ((r_segment->flag & vfFlag::DirectionBackwards) == true) ? -1 : 1;
  const int point_size = vf->curves_2d.point_size[curve_i];
  const int point_offset = vf->curves_2d.point_offset[curve_i];
  int point_i = r_segment->point_range[0];
  const int point_start = point_i;
  bool check_gaps = false;

  do {
    /* Check curve bounds. */
    if (point_i < 0 || point_i >= point_size) {
      point_i = std::min(point_size - 1, std::max(0, point_i));
      if (!is_cyclic) {
        check_gaps = true;
        break;
      }

      /* Add new segment for second point range of cyclic curve. */
      r_segment->set_point_range(point_i, direction);

      FillSegment new_segment;
      new_segment.curve_index_2d = r_segment->curve_index_2d;
      new_segment.point_range[0] = (direction == -1) ? (point_size - 1) : 0;
      new_segment.flag = r_segment->flag;
      new_segment.flag |= vfFlag::IsLastOfCycle;
      vf->segments.append(new_segment);
      r_segment = &vf->segments.last();

      point_i = new_segment.point_range[0];
    }

    /* Check for intersections on current curve segment. */
    const int point_next = get_next_curve_point(point_i, direction, point_size, is_cyclic);
    if (point_next == -1) {
      check_gaps = true;
      break;
    }

    /* Check closed cycle. */
    if (point_next == point_start) {
      r_segment->flag |= vfFlag::IsCyclic;
      break;
    }

    /* Get intersections with other curves. */
    const float2 segment_point_a = vf->curves_2d.points_2d[point_offset + point_i];
    const float2 segment_point_b = vf->curves_2d.points_2d[point_offset + point_next];
    Vector<IntersectingSegment2D> intersections = intersections_segment_strokes_2d(
        segment_point_a, segment_point_b, curve_i, &vf->curves_2d);
    add_segment_end(
        &vf->curves_2d, r_segment, segment_point_a, segment_point_b, point_next, intersections);

    /* Get intersections with curve end extensions. */
    if (vf->gap_close_extend) {
      Vector<IntersectingSegment2D> intersections = intersections_segment_strokes_2d(
          segment_point_a, segment_point_b, -1, &vf->extensions_2d);
      add_segment_end(&vf->extensions_2d,
                      r_segment,
                      segment_point_a,
                      segment_point_b,
                      point_next,
                      intersections);
    }

    /* When one or more intersections are found, we can stop walking along the curve. */
    if (!r_segment->segment_ends.is_empty()) {
      /* Sort intersections on distance. This is important to find the shortest edge. */
      std::sort(r_segment->segment_ends.begin(),
                r_segment->segment_ends.end(),
                [](const SegmentEnd &a, const SegmentEnd &b) { return a.distance < b.distance; });

      /* Set intersection flag. */
      r_segment->flag |= vfFlag::IsIntersected;

      break;
    }

    /* Walk to next point on curve. */
    point_i += direction;
  } while (true);

  /* Set walked point range. */
  r_segment->set_point_range(point_i, direction);

  if (!check_gaps) {
    return true;
  }

  /* We've reached the end of a curve without finding an intersection. Now we check for
   * gap closures. */
  const bool at_start_of_curve = (point_i == 0);

  /* Check for gap closure extending the end of the curve. */
  if (vf->gap_close_extend) {
    /* Get curve index and coordinates of start/end extension. */
    const int curve_ext = curve_i * 2 + (at_start_of_curve ? 0 : 1);
    const int point_offset = vf->extensions_2d.point_offset[curve_ext];
    const float2 segment_point_a = vf->extensions_2d.points_2d[point_offset + point_i];
    const float2 segment_point_b = vf->extensions_2d.points_2d[point_offset + point_i + 1];

    /* Get intersections with other curves. */
    Vector<IntersectingSegment2D> intersections = intersections_segment_strokes_2d(
        segment_point_a, segment_point_b, -1, &vf->curves_2d);
    add_segment_end(
        &vf->curves_2d, r_segment, segment_point_a, segment_point_b, -1, intersections);

    /* Get intersections with other curve end extensions. */
    intersections = intersections_segment_strokes_2d(
        segment_point_a, segment_point_b, curve_ext, &vf->extensions_2d);
    add_segment_end(
        &vf->extensions_2d, r_segment, segment_point_a, segment_point_b, -1, intersections);

    if (!r_segment->segment_ends.is_empty()) {
      /* Sort intersections on distance. We want to start with the smallest distance to find the
       * shortest fill edge. */
      std::sort(r_segment->segment_ends.begin(),
                r_segment->segment_ends.end(),
                [](const SegmentEnd &a, const SegmentEnd &b) { return a.distance < b.distance; });

      /* Set gap closure flags. */
      r_segment->flag |= vfFlag::IsGapClosure;
    }
  }

  /* Check for gap closure with radii at the ends of curves. */
  if (vf->gap_close_radius) {
    /* Get coordinates of curve start/end point. */
    const int curve_point_end = point_offset + point_i;
    const float2 curve_point = vf->curves_2d.points_2d[curve_point_end];

    /* Get the segment coordinates at the start/end of the curve. */
    const int curve_point_end_prev = curve_point_end +
                                     math::min(point_size - 1, 1) * (at_start_of_curve ? 1 : -1);
    const float2 curve_point_prev = vf->curves_2d.points_2d[curve_point_end_prev];
    const float curve_angle = math::atan2(curve_point_prev[1] - curve_point[1],
                                          curve_point_prev[0] - curve_point[0]);

    const float two_pi = M_PI * 2;
    const float max_dist = vf->gap_distance * 2;

    /* Find nearest end points.
     * Note: the number of points is a bit arbitrary, but the defined number will suffice for
     * normal cases. */
    KDTreeNearest_2d nearest[GP_VFILL_RADIUS_NEAREST_NUM];
    const int nearest_num = BLI_kdtree_2d_find_nearest_n(
        vf->curve_ends, curve_point, nearest, GP_VFILL_RADIUS_NEAREST_NUM);

    for (int i = 0; i < nearest_num; i++) {
      /* Skip nearest points outside the user-defined closing radius. */
      if (nearest[i].dist > max_dist) {
        continue;
      }

      /* There are (number of curves * 2) points in the KD tree: the first and last point of each
       * curve. So an even KD tree index means a 'first' point, an odd index means a 'last'
       * point. */
      const int nearest_curve = int(nearest[i].index / 2);
      const bool nearest_at_start_of_curve = (nearest[i].index % 2) == 0;
      int nearest_point_i = vf->curves_2d.point_offset[nearest_curve];
      if (!nearest_at_start_of_curve) {
        nearest_point_i += vf->curves_2d.point_size[nearest_curve];
      }

      /* Set segment end values. */
      SegmentEnd segment_end{};
      segment_end.flag = (nearest_at_start_of_curve ? vfFlag::None : vfFlag::DirectionBackwards);
      segment_end.curve_index_2d = nearest_curve;
      segment_end.point_start = nearest_point_i;

      /* Determine the angle between the curve start/end and the nearest point. For finding the
       * shortest fill edge, we explore the smallest angle first (= most right turn).
       * Note: we use the `distance` field to store the angle. */
      const float2 co = vf->curves_2d.points_2d[nearest_point_i];
      segment_end.distance = math::atan2(co[1] - curve_point[1], co[0] - curve_point[0]) -
                             curve_angle;
      if (segment_end.distance < 0.0f) {
        segment_end.distance += two_pi;
      }

      /* Append to segment end list. */
      r_segment->segment_ends.append(segment_end);
    }

    if (!r_segment->segment_ends.is_empty()) {
      /* Sort nearest curves on angle. */
      std::sort(r_segment->segment_ends.begin(),
                r_segment->segment_ends.end(),
                [](const SegmentEnd &a, const SegmentEnd &b) { return a.distance < b.distance; });

      /* Set gap closure flags. */
      r_segment->flag |= vfFlag::IsGapClosure;
    }
  }

  return true;
}

static void take_next_turn_on_intersection(VectorFillData *vf,
                                           SegmentEnd *segment_end,
                                           const FillSegment *ori_segment)
{
  int curve_index_2d, point_start;
  vfFlag flag = vfFlag::None;

  switch (segment_end->turn) {
    /* Right turn. */
    case 0: {
      curve_index_2d = segment_end->curve_index_2d;
      point_start = segment_end->point_end;
      if ((segment_end->flag & vfFlag::DirectionBackwards) == true) {
        point_start = segment_end->point_start;
        flag = vfFlag::DirectionBackwards;
      }
      break;
    }
    /* Straight ahead, on same curve as we were. */
    case 1: {
      curve_index_2d = ori_segment->curve_index_2d;
      point_start = segment_end->ori_segment_point_end;
      if ((ori_segment->flag & vfFlag::DirectionBackwards) == true) {
        flag = vfFlag::DirectionBackwards;
      }
      break;
    }
    /* Left turn. */
    case 2: {
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
  FillSegment segment{};
  segment.curve_index_2d = curve_index_2d;
  segment.point_range[0] = point_start;
  segment.flag = flag;

  vf->segments.append(segment);
}

static void take_next_gap_closure_curve(VectorFillData *vf, SegmentEnd *segment_end)
{
  /* Add new segment to the fill edge. */
  FillSegment segment{};
  segment.curve_index_2d = segment_end->curve_index_2d;
  segment.point_range[0] = segment_end->point_start;
  if ((segment_end->flag & vfFlag::DirectionBackwards) == true) {
    segment.flag = vfFlag::DirectionBackwards;
  }

  vf->segments.append(segment);
}

static bool find_closed_fill_edge(VectorFillData *vf)
{
  while (!vf->segments.is_empty()) {
    /* Explore the last edge segment in the array. */
    FillSegment *segment = &vf->segments.last();

    /* Walk along the curve until an intersection or the end of the curve is reached. */
    bool changed = walk_along_curve(vf, segment);

    /* Check if the edge is closed. */
    if (changed) {
      if (is_closed_fill_edge(vf)) {
        /* Check if the mouse click position is inside the loop. */
        if (mouse_pos_is_inside_closed_edge(vf)) {
          return true;
        }

        /* Remove the overlapping segment and continue searching. */
        const bool is_cyclic_segment = (segment->flag & vfFlag::IsLastOfCycle) == true;
        vf->segments.remove_last();
        if (is_cyclic_segment) {
          vf->segments.remove_last();
        }
        continue;
      }
    }

    /* Inpsect the segment ends. */
    if (segment->segment_ends_index < segment->segment_ends.size()) {
      SegmentEnd segment_end = segment->segment_ends[segment->segment_ends_index];

      /* Handle segment that ends at an intersection. */
      if ((segment->flag & vfFlag::IsIntersected) == true) {
        /* When all intersection turns are explored, continue with the
         * next intersection (if any). */
        if (segment_end.turn >= 3) {
          segment->segment_ends_index++;
          continue;
        }

        /* Take the next turn on the intersection:
         * 0 = right turn, 1 = straight ahead, 2 = left turn. */
        take_next_turn_on_intersection(vf, &segment_end, segment);

        segment_end.turn++;
      }
      else {
        /* Segment that ends with a gap closure: explore the next gap-closed curve. */
        take_next_gap_closure_curve(vf, &segment_end);

        segment->segment_ends_index++;
      }
    }
    else {
      /* We reached a dead end, delete the segment and continue with the previous. */
      const bool is_cyclic_segment = (segment->flag & vfFlag::IsLastOfCycle) == true;
      vf->segments.remove_last();
      if (is_cyclic_segment) {
        vf->segments.remove_last();
      }
    }
  }

  return false;
}

static bool vector_fill_do(VectorFillData *vf)
{
  /* Find a first arbitrary edge point of the fill area by creating a large vector straight
   * upwards from the mouse position. Check curve intersections with this large vector. */
  bool found = false;
  Array<float> deltas_x = {0.0f, -4000.0f, 4000.0f};
  for (const float delta_x : deltas_x) {
    float2 upwards;
    upwards[0] = vf->mouse_pos[0] + delta_x;
    upwards[1] = vf->mouse_pos[1] + 20000.0f;

    vf->starting_segments = intersections_segment_strokes_2d(
        vf->mouse_pos, upwards, -1, &vf->curves_2d);
    if (vf->starting_segments.size() > 0) {
      found = true;
      break;
    }
  }
  if (!found) {
    /* No initial edge point found, so the fill operation fails. */
    return false;
  }

  /* Sort starting segments on distance (closest first). */
  std::sort(vf->starting_segments.begin(),
            vf->starting_segments.end(),
            [](const IntersectingSegment2D &a, const IntersectingSegment2D &b) {
              return a.distance < b.distance;
            });

  /* From the first edge point we try to follow the fill edge clockwise, until we find a closed
   * loop. */
  found = false;
  for (auto &start_segment : vf->starting_segments) {
    /* Start with an empty edge segment list. */
    vf->segments.clear();

    /* Add first edge segment. */
    FillSegment segment{};
    segment.curve_index_2d = start_segment.curve_index_2d;
    segment.point_range[0] = start_segment.point_start;

    /* We want to follow the edge clockwise, so look for a positive x delta. */
    const int point_offset = vf->curves_2d.point_offset[segment.curve_index_2d];
    const int point_next = (segment.point_range[0] + 1) %
                           vf->curves_2d.point_size[segment.curve_index_2d];
    const float delta_x = vf->curves_2d.points_2d[point_next + point_offset][0] -
                          vf->curves_2d.points_2d[segment.point_range[0] + point_offset][0];
    if (delta_x < 0) {
      segment.flag |= vfFlag::DirectionBackwards;
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

static void get_curve_end_radii(VectorFillData *vf)
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
}

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

  /* Use the vector between the two outer points of the curve to calculate the extension
   * coordinates. */
  float2 co = vf->curves_2d.points_2d[point_i];
  vf->extensions_2d.points_2d[point_i_ext] = co;
  float2 vec = co - vf->curves_2d.points_2d[point_i + next_point_delta];
  vf->extensions_2d.points_2d[point_i_ext + 1] = co + math::normalize(vec) * vf->gap_distance;
}

static void get_curve_end_extensions(VectorFillData *vf)
{
  const int curve_num = vf->curves_2d.point_offset.size() * 2;
  const int point_num = curve_num * 2;

  vf->extensions_2d.point_offset = Array<int>(curve_num);
  vf->extensions_2d.point_size = Array<int>(curve_num, 2);
  vf->extensions_2d.is_cyclic = Array<bool>(curve_num, false);
  vf->extensions_2d.drawing_index_2d = Array<int>(curve_num);
  vf->extensions_2d.points_2d = Array<float2>(point_num);
  vf->extensions_2d.curve_bbox = Array<rctf>(curve_num);

  threading::parallel_for(
      vf->curves_2d.point_offset.index_range(), 256, [&](const IndexRange range) {
        for (const int curve_i : range) {
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
      });
}

static void init(bContext *C, wmOperator *op)
{
  VectorFillData *vf = static_cast<VectorFillData *>(op->customdata);
  Depsgraph *depsgraph = CTX_data_ensure_evaluated_depsgraph(C);
  ED_view3d_viewcontext_init(C, &vf->vc, depsgraph);

  vf->grease_pencil = static_cast<GreasePencil *>(vf->vc.obact->data);

  /* Get tool settings. */
  ToolSettings *ts = CTX_data_tool_settings(C);
  Brush *brush = BKE_paint_brush(&ts->gp_paint->paint);
  BrushGpencilSettings *brush_settings = brush->gpencil_settings;

  vf->gap_distance = brush_settings->fill_extend_fac;
  const bool use_gap_close = (vf->gap_distance > FLT_EPSILON);
  vf->gap_close_extend = (brush_settings->fill_extend_mode == GP_FILL_EMODE_EXTEND) &&
                         use_gap_close;
  vf->gap_close_radius = (brush_settings->fill_extend_mode == GP_FILL_EMODE_RADIUS) &&
                         use_gap_close;
  vf->is_first_click = true;
  vf->overlay_initialized = false;
  vf->stroke_radius = float(brush->size);

  /* Convert curves to viewport 2D space. */
  /* TODO: set layer options (visible, above, all etc.) */
  vf->curves_2d = editable_strokes_in_2d_space_get(&vf->vc, vf->vc.obact);

  /* When using extensions of the curve ends to close gaps, build an array of those two-point
   * 'curves'. */
  if (vf->gap_close_extend) {
    get_curve_end_extensions(vf);
  }

  /* When using radii to close gaps, build KD tree of curve end points. */
  if (vf->gap_close_radius) {
    get_curve_end_radii(vf);
  }
}

static void exit(bContext *C, wmOperator *op)
{
  WM_cursor_modal_restore(CTX_wm_window(C));

  if (op->customdata != nullptr) {
    VectorFillData *vf = static_cast<VectorFillData *>(op->customdata);
    if (vf->gap_close_radius) {
      BLI_kdtree_2d_free(vf->curve_ends);
    }
    MEM_delete(vf);
    op->customdata = nullptr;
  }
}

static int modal(bContext *C, wmOperator *op, const wmEvent *event)
{
  int modal_state = OPERATOR_RUNNING_MODAL;
  VectorFillData *vf = static_cast<VectorFillData *>(op->customdata);

  switch (event->type) {
    case EVT_ESCKEY:
    case RIGHTMOUSE:
      modal_state = OPERATOR_CANCELLED;
      break;

    case LEFTMOUSE: {
      /* First click: show overlay with gap closure */
      if (vf->is_first_click) {
        /* TODO: show overlay. */
        if (!vf->overlay_initialized) {
          WM_cursor_modal_set(CTX_wm_window(C), WM_CURSOR_PAINT_BRUSH);
          vf->overlay_initialized = true;
        }

        if (event->val == KM_RELEASE) {
          vf->is_first_click = false;
        }
      }
      else {
        /* Second click: perform the actual fill. */
        vf->mouse_pos[0] = float(event->mval[0]);
        vf->mouse_pos[1] = float(event->mval[1]);
        const bool succes = vector_fill_do(vf);
        if (succes) {
          modal_state = OPERATOR_FINISHED;
        }
        else {
          BKE_report(op->reports, RPT_INFO, "No closed area found");
          modal_state = OPERATOR_CANCELLED;
        }
      }
      break;
    }
    default:
      break;
  }

  switch (modal_state) {
    case OPERATOR_FINISHED:
      exit(C, op);
      WM_event_add_notifier(C, NC_GPENCIL | NA_EDITED, NULL);
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

static int invoke(bContext *C, wmOperator *op, const wmEvent * /*event*/)
{
  VectorFillData *op_data = MEM_new<VectorFillData>(__func__);
  op->customdata = op_data;

  /* TODO: check active material. */

  /* Init tool data. */
  init(C, op);

  /* Add modal handler. */
  WM_event_add_modal_handler(C, op);

  return OPERATOR_RUNNING_MODAL;
}

static void GREASE_PENCIL_OT_vector_fill_based(wmOperatorType *ot)
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

}  // namespace blender::ed::greasepencil::vectorfill

void ED_operatortypes_grease_pencil_fill()
{
  using namespace blender::ed::greasepencil::vectorfill;

  WM_operatortype_append(GREASE_PENCIL_OT_vector_fill_based);
}
