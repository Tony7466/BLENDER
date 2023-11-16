/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgreasepencil
 */

#include "grease_pencil_fill.hh"

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
 * The edge is made up of segments. A segment is a point range in a curve. A segment ends at an
 * intersection with another curve (D) or at the end of the curve itself (G).
 * At an intersection (D), the right turn will be inspected first. Because we are inspecting the
 * edge in clockwise direction, this guarantees us that we find the 'narrowest' edge. See for
 * example at (F): by taking the right turn first, we find the smallest fill area (just as we
 * should). At an intersection, there are three possible turns. When a turn is not leading to a
 * closed edge (see e.g. (E), turn 1 and 2), the next turn is inspected.
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
 * curve points). This can easily occur when the fill tool is used twice: the original stroke
 * and the fill curve share geometry points. Therefore, when looking for intersections, a check on
 * overlapping segments is performed. And curves with a stroke material get priority over curves
 * with only a fill material, since that is the most common situation: the outlines of the fill
 * area are drawn with 'stroke' curves.
 *
 * In the code:
 * #vector_fill_do()          start of the algorithm, casting a ray (B)
 * #find_closed_fill_edge()   search for the narrowest closed edge
 * #walk_along_curve()        creating an edge segment, inspecting where the segment ends
 */

namespace blender::ed::greasepencil::fill {

/* ------------------------------------------------------------------------------ */
/** \name Vector Fill data structs
 * \{ */

/** \} */

/* ------------------------------------------------------------------------------ */
/** \name Intersection functions
 * \{ */

/**
 * Store a segment that shares points with an inspected edge segment.
 */
static void store_overlapping_segment(const int overlapping_curve,
                                      const int overlapping_point,
                                      const bool in_opposite_dir,
                                      const int base_point,
                                      const int base_direction,
                                      FillData *fd)
{
  const int overlapping_direction = (in_opposite_dir ? -base_direction : base_direction);
  const bool overlapping_backwards = (overlapping_direction == -1);
  const int overlapping_range_index = (overlapping_direction == 1 ? 0 : 1);
  const int base_range_index = (base_direction == 1 ? 0 : 1);

  /* Look for an existing overlapping segment which point range we can expand. */
  for (auto &overlap : fd->vf.overlapping_segments) {
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
  fd->vf.overlapping_segments.append(std::move(overlap));
}

/** \} */

/* ------------------------------------------------------------------------------ */
/** \name Curve proximity functions
 * \{ */

/**
 * Check if the shortest distance between two line segments is within a given proximity distance.
 * Note: this computation doesn't handle intersecting lines, but that is okay, since we have
 * handled intersections elsewhere already.
 */
static std::tuple<bool, bool, bool> line_segments_are_in_proximity(
    const float2 &segment_a,
    const float2 &segment_b,
    const float2 &curve_p0,
    const float2 &curve_p1,
    const float proximity_distance_squared)
{
#define DET_EPSILON 0.000001f

  /* Calculate the shortest distance between the two line segments. */
  const float2 vec_segment = segment_a - segment_b;
  const float2 vec_curve = curve_p0 - curve_p1;
  const float2 vec_segment_curve = segment_b - curve_p1;

  const float dot_seg = math::dot(vec_segment, vec_segment);
  const float dot_seg_cur = math::dot(vec_segment, vec_curve);
  const float dot_cur = math::dot(vec_curve, vec_curve);
  const float dot_seg_seg_cur = math::dot(vec_segment, vec_segment_curve);
  const float dot_cur_seg_cur = math::dot(vec_curve, vec_segment_curve);

  const float det = dot_seg * dot_cur - dot_seg_cur * dot_seg_cur;
  float d_seg = det;
  float d_cur = det;
  float p_on_seg, p_on_cur;

  /* Abort when one of the segments has equal points. */
  if (dot_seg == 0.0f || dot_cur == 0.0f) {
    return std::tuple<bool, bool, bool>(false, false, false);
  }

  if (det < DET_EPSILON) {
    /* Lines are almost parallel, force using point p0 on segment (to prevent division by zero
     * later). */
    p_on_seg = 0.0f;
    d_seg = 1.0f;
    p_on_cur = dot_cur_seg_cur;
    d_cur = dot_cur;
  }
  else {
    /* Calculate points on the infinite lines. */
    p_on_seg = dot_seg_cur * dot_cur_seg_cur - dot_cur * dot_seg_seg_cur;
    p_on_cur = dot_seg * dot_cur_seg_cur - dot_seg_cur * dot_seg_seg_cur;

    /* Limit to edges of segment line. */
    if (p_on_seg < 0.0f) {
      p_on_seg = 0.0f;
      p_on_cur = dot_cur_seg_cur;
      d_cur = dot_cur;
    }
    else if (p_on_seg > d_seg) {
      p_on_seg = d_seg;
      p_on_cur = dot_cur_seg_cur + dot_seg_cur;
      d_cur = dot_cur;
    }
  }

  /* Limit to edges of curve line. */
  if (p_on_cur < 0.0f) {
    p_on_cur = 0.0f;
    const float inv = -dot_seg_seg_cur;
    if (inv < 0.0f) {
      p_on_seg = 0.0f;
    }
    else if (inv > dot_seg) {
      p_on_seg = d_seg;
    }
    else {
      p_on_seg = inv;
      d_seg = dot_seg;
    }
  }
  else if (p_on_cur > d_cur) {
    p_on_cur = d_cur;
    const float inv = -dot_seg_seg_cur + dot_seg_cur;
    if (inv < 0.0f) {
      p_on_seg = 0.0f;
    }
    else if (inv > dot_seg) {
      p_on_seg = d_seg;
    }
    else {
      p_on_seg = inv;
      d_seg = dot_seg;
    }
  }

  /* Get the difference between the two closest points. */
  const float c_seg = (math::abs(p_on_seg) < DET_EPSILON) ? 0.0f : p_on_seg / d_seg;
  const float c_cur = (math::abs(p_on_cur) < DET_EPSILON) ? 0.0f : p_on_cur / d_cur;
  const float dist_sq = math::length_squared(vec_segment_curve + c_seg * vec_segment -
                                             c_cur * vec_curve);

  /* When the shortest distance between the line segments isn't within the proximity range,
   * return false. */
  if (dist_sq > proximity_distance_squared) {
    return std::tuple<bool, bool, bool>(false, false, false);
  }

  /* When the nearest curve point is beyond segment point b, we want to use segment point b for
   * the fill edge. */
  const bool use_segment_b = (c_seg == 0.0f);

  /* Establish which curve point is nearest to the segment start point. */
  const float dist_p0 = math::length_squared(curve_p0 - segment_a);
  const float dist_p1 = math::length_squared(curve_p1 - segment_a);
  const bool curve_p0_is_nearest = dist_p0 < dist_p1;

  return std::tuple<bool, bool, bool>(true, use_segment_b, curve_p0_is_nearest);

#undef DET_EPSILON
}

static ProximityCurve get_segment_end_proximity(EdgeSegment *segment,
                                                const int segment_direction,
                                                const int curve_index,
                                                const FillData *fd)
{
  const float proxim_distance_squared = fd->proximity_distance * fd->proximity_distance;

  /* Get segment end points. */
  const int segment_curve_index = segment->curve_index_2d;
  const int segment_point_offset = fd->curves_2d.point_offset[segment_curve_index];
  const int segment_point_end = (segment_direction == 1) ? segment->point_range[1] :
                                                           segment->point_range[0];
  const float2 segment_a =
      fd->curves_2d.points_2d[segment_point_offset + segment_point_end - segment_direction];
  const float2 segment_b = fd->curves_2d.points_2d[segment_point_offset + segment_point_end];

  /* Get curve head and tail points. */
  const int curve_point_offset = fd->curves_2d.point_offset[curve_index];
  const int curve_point_head = 0;
  const float2 curve_head_p0 = fd->curves_2d.points_2d[curve_point_offset];
  const float2 curve_head_p1 = fd->curves_2d.points_2d[curve_point_offset + 1];
  const int curve_point_tail = fd->curves_2d.point_size[curve_index] - 2;
  const float2 curve_tail_p0 = fd->curves_2d.points_2d[curve_point_offset + curve_point_tail];
  const float2 curve_tail_p1 = fd->curves_2d.points_2d[curve_point_offset + curve_point_tail + 1];

  /* When the segment curve and the proximity curve are the same (self-closing curve), take the
   * opposite point of the segment end. */
  float2 curve_p0, curve_p1;
  int curve_point_index;
  bool take_tail;
  if (segment_curve_index == curve_index) {
    take_tail = (segment_point_end == 0);
  }
  else {
    /* Otherwise, take the curve point closest to the segment end. */
    take_tail = (math::length_squared(segment_b - curve_tail_p1) <
                 math::length_squared(segment_b - curve_head_p0));
  }
  if (take_tail) {
    copy_v2_v2(curve_p0, curve_tail_p0);
    copy_v2_v2(curve_p1, curve_tail_p1);
    curve_point_index = curve_point_tail;
  }
  else {
    copy_v2_v2(curve_p0, curve_head_p0);
    copy_v2_v2(curve_p1, curve_head_p1);
    curve_point_index = curve_point_head;
  }

  /* Check proximity. */
  ProximityCurve proxim{};
  bool in_proximity;
  std::tie(in_proximity, proxim.use_segment_point_end, proxim.curve_point_start_is_nearest) =
      line_segments_are_in_proximity(
          segment_a, segment_b, curve_p0, curve_p1, proxim_distance_squared);

  /* Create proximity data. */
  if (!in_proximity) {
    proxim.curve_index_2d = -1;
  }
  else {
    proxim.curve_index_2d = curve_index;
    proxim.curve_point_start = curve_point_index;
    proxim.curve_point_end = curve_point_index + 1;
    proxim.segment_point_start = segment_point_end - segment_direction;
    proxim.segment_point_end = segment_point_end;
  }

  return proxim;
}

/**
 * Get all curves in proximity of an fill edge segment.
 */
static Vector<ProximityCurve> get_curves_in_proximity_of_segment(EdgeSegment *segment,
                                                                 const int segment_direction,
                                                                 const FillData *fd)
{
  /* Inits. */
  Vector<ProximityCurve> proximities;
  std::mutex mutex;
  rctf bbox_segment;

  const int segment_curve_index = segment->curve_index_2d;
  const int segment_point_offset = fd->curves_2d.point_offset[segment_curve_index];

  const float proxim_distance_squared = fd->proximity_distance * fd->proximity_distance;

  const bool at_end_of_curve = ((segment_direction == -1 && segment->point_range[0] == 0) ||
                                (segment_direction == 1 &&
                                 segment->point_range[1] ==
                                     fd->curves_2d.point_size[segment->curve_index_2d] - 1));

  EdgeSegment previous_segment;
  bool has_previous_segment = false;
  if (fd->vf.segments.size() > 1) {
    previous_segment = fd->vf.segments[fd->vf.segments.size() - 2];
    has_previous_segment = true;
  }
  const int prev_segment_curve_index = (has_previous_segment ? previous_segment.curve_index_2d :
                                                               -1);

  BLI_rctf_init_minmax(&bbox_segment);
  BLI_rctf_do_minmax_v(&bbox_segment,
                       fd->curves_2d.points_2d[segment_point_offset + segment->point_range[0]]);
  BLI_rctf_do_minmax_v(&bbox_segment,
                       fd->curves_2d.points_2d[segment_point_offset + segment->point_range[1]]);
  BLI_rctf_pad(&bbox_segment, fd->proximity_distance, fd->proximity_distance);

  /* Check all curves for promity. */
  threading::parallel_for(
      fd->curves_2d.point_offset.index_range(), 256, [&](const IndexRange range) {
        for (const int curve_i : range) {
          /* Only check curves with at least two points. */
          if (fd->curves_2d.point_size[curve_i] < 2) {
            continue;
          }

          /* Skip previous segment curve and current segment curve, to avoid local looping. But
           * we don't want to exclude simple one- or two-stroke fills, so we do a
           * head-tail-proximity check when we are at the end of a curve. */
          if (curve_i == segment_curve_index || curve_i == prev_segment_curve_index) {
            if (at_end_of_curve) {
              ProximityCurve proxim = get_segment_end_proximity(
                  segment, segment_direction, curve_i, fd);
              if (proxim.curve_index_2d != -1) {
                std::lock_guard lock{mutex};
                proximities.append(proxim);
              }
            }
            continue;
          }

          /* Skip curves with only a fill material. */
          if (!fd->curves_2d.has_stroke[curve_i]) {
            continue;
          }

          /* Skip curve when it is not overlapping the segment bounding box. */
          rctf bbox_proxim;
          BLI_rctf_init(&bbox_proxim,
                        fd->curves_2d.curve_bbox[curve_i].xmin,
                        fd->curves_2d.curve_bbox[curve_i].xmax,
                        fd->curves_2d.curve_bbox[curve_i].ymin,
                        fd->curves_2d.curve_bbox[curve_i].ymax);
          BLI_rctf_pad(&bbox_proxim, fd->proximity_distance, fd->proximity_distance);
          if (!BLI_rctf_isect(&bbox_segment, &bbox_proxim, nullptr)) {
            continue;
          }

          /* Skip curves that intersect the segment (no need to find them again). */
          bool skip_curve = false;
          for (const auto &segment_end : segment->segment_ends) {
            if ((segment_end.flag & vfFlag::WithEndExtension) == false &&
                segment_end.curve_index_2d == curve_i)
            {
              skip_curve = true;
              break;
            }
          }
          if (skip_curve) {
            continue;
          }

          /* Skip curves that intersect the previous segment (no need to find them again). */
          if (has_previous_segment) {
            for (const auto &segment_end : previous_segment.segment_ends) {
              if ((segment_end.flag & vfFlag::WithEndExtension) == false &&
                  segment_end.curve_index_2d == curve_i)
              {
                skip_curve = true;
                break;
              }
            }
            if (skip_curve) {
              continue;
            }
          }

          /* Skip curves that have exact overlap with curves intersecting the segment. */
          for (const auto &overlap : fd->vf.overlapping_segments) {
            if (overlap.overlapping_curve_index == curve_i) {
              skip_curve = true;
              break;
            }
          }
          if (skip_curve) {
            continue;
          }

          /* Loop over curve points. */
          bool proximity_found = false;
          const int curve_point_offset = fd->curves_2d.point_offset[curve_i];
          for (const int point_i :
               IndexRange(curve_point_offset, fd->curves_2d.point_size[curve_i]).drop_back(1))
          {
            const float2 curve_p0 = fd->curves_2d.points_2d[point_i];
            const float2 curve_p1 = fd->curves_2d.points_2d[point_i + 1];

            /* Skip when bounding box doesn't overlap with segment bounding box. */
            BLI_rctf_init(&bbox_proxim, curve_p0[0], curve_p0[0], curve_p0[1], curve_p0[1]);
            BLI_rctf_do_minmax_v(&bbox_proxim, curve_p1);
            BLI_rctf_pad(&bbox_proxim, fd->proximity_distance, fd->proximity_distance);
            if (!BLI_rctf_isect(&bbox_proxim, &bbox_segment, nullptr)) {
              continue;
            }

            /* Loop over all segment point-pairs. */
            const int segment_start = segment_point_offset + ((segment_direction == 1) ?
                                                                  segment->point_range[0] :
                                                                  segment->point_range[1]);
            const int segment_end = segment_point_offset + ((segment_direction == 1) ?
                                                                segment->point_range[1] :
                                                                segment->point_range[0]);

            for (int segment_point_i = segment_start; segment_point_i != segment_end;
                 segment_point_i += segment_direction)
            {
              const float2 segment_a = fd->curves_2d.points_2d[segment_point_i];
              const float2 segment_b =
                  fd->curves_2d.points_2d[segment_point_i + segment_direction];

              /* Skip when line segments don't overlap. */
              rctf bbox_seg;
              BLI_rctf_init(&bbox_seg, segment_a[0], segment_a[0], segment_a[1], segment_a[1]);
              BLI_rctf_do_minmax_v(&bbox_seg, segment_b);
              BLI_rctf_pad(&bbox_seg, fd->proximity_distance, fd->proximity_distance);
              if (!BLI_rctf_isect(&bbox_proxim, &bbox_seg, nullptr)) {
                continue;
              }

              /* Check if the line segments are in proximity of each other. */
              bool in_proximity, use_segment_point_end, curve_point_start_nearest;
              std::tie(in_proximity, use_segment_point_end, curve_point_start_nearest) =
                  line_segments_are_in_proximity(
                      segment_a, segment_b, curve_p0, curve_p1, proxim_distance_squared);

              /* When the line segments are in proximity, add the data to the proximity list. */
              if (in_proximity) {
                ProximityCurve proxim{};
                proxim.curve_index_2d = curve_i;
                proxim.curve_point_start = point_i - curve_point_offset;
                proxim.curve_point_end = proxim.curve_point_start + 1;
                proxim.segment_point_start = segment_point_i - segment_point_offset;
                proxim.segment_point_end = proxim.segment_point_start + segment_direction;
                proxim.use_segment_point_end = use_segment_point_end;
                proxim.curve_point_start_is_nearest = curve_point_start_nearest;

                std::lock_guard lock{mutex};
                proximities.append(proxim);

                proximity_found = true;
                break;
              }
            }

            /* Stop checking curve points when proximity to the segment is found. */
            if (proximity_found) {
              break;
            }
          }
        }
      });

  return proximities;
}

/** \} */

/* ------------------------------------------------------------------------------ */
/** \name Vector Fill functions
 * \{ */

static bool position_equals_previous(const float3 &co, float3 &r_co_prev)
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
static Vector<float3> get_closed_fill_edge_as_3d_points(FillData *fd)
{
  /* Ensure head starting point and tail end point are an exact match. */
  EdgeSegment *tail = &fd->vf.segments.last();
  for (auto &head : fd->vf.segments) {
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
  for (auto &segment : fd->vf.segments) {
    if ((segment.flag & vfFlag::IsUnused) == true) {
      continue;
    }

    /* Get original 3D curve. */
    const int drawing_index = fd->curves_2d.drawing_index_2d[segment.curve_index_2d];
    const int curve_index = segment.curve_index_2d - fd->curves_2d.curve_offset[drawing_index];
    const bke::CurvesGeometry &curves = fd->curves_2d.drawings[drawing_index]->geometry.wrap();
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
      if (!position_equals_previous(positions[point_i], co_prev)) {
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
      if (!position_equals_previous(isect_point, co_prev)) {
        edge_points.append(isect_point);
        edge_point_index++;
      }
    }

    /* Calculate intersection with curve end extension. */
    if ((segment.flag & vfFlag::GapClosureByExtension) == true && edge_point_index > 1) {
      /* Get segment end data. */
      const SegmentEnd *segment_end = &segment.segment_ends[segment.segment_ends_index];
      const int extension_index = segment.curve_index_2d * 2 + (direction == 1 ? 1 : 0);

      /* Extend end of the curve. */
      const float3 isect_point = edge_points[edge_point_index - 1] +
                                 (edge_points[edge_point_index - 1] -
                                  edge_points[edge_point_index - 2]) *
                                     fd->extension_length_ratio[extension_index] *
                                     segment_end->distance;
      if (!position_equals_previous(isect_point, co_prev)) {
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
static void create_fill_geometry(FillData *fd)
{
  /* Ensure active frame (autokey is on, we checked on operator invoke). */
  const bke::greasepencil::Layer *active_layer = fd->grease_pencil->get_active_layer();
  const int current_frame = fd->vc.scene->r.cfra;
  if (!fd->grease_pencil->get_active_layer()->frames().contains(current_frame)) {
    bke::greasepencil::Layer &active_layer = *fd->grease_pencil->get_active_layer_for_write();
    if (fd->additive_drawing) {
      /* For additive drawing, we duplicate the frame that's currently visible and insert it at the
       * current frame. */
      fd->grease_pencil->insert_duplicate_frame(
          active_layer, active_layer.frame_key_at(current_frame), current_frame, false);
    }
    else {
      /* Otherwise we just insert a blank keyframe. */
      fd->grease_pencil->insert_blank_frame(active_layer, current_frame, 0, BEZT_KEYTYPE_KEYFRAME);
    }
  }

  /* Get the edge points in 3D space. */
  Vector<float3> fill_points = get_closed_fill_edge_as_3d_points(fd);
  if (fill_points.size() <= 2) {
    return;
  }

  /* Create geometry. */
  const int drawing_index = active_layer->drawing_index_at(fd->vc.scene->r.cfra);
  bke::greasepencil::Drawing &drawing = reinterpret_cast<GreasePencilDrawing *>(
                                            fd->grease_pencil->drawings()[drawing_index])
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
  radii.slice(new_points_range).fill(fd->brush->size);
  opacities.slice(new_points_range).fill(1.0f);

  /* Make curve cyclic. */
  curves.cyclic_for_write().slice(new_curves_range).fill(true);

  /* Set curve_type attribute. */
  curves.fill_curve_types(new_curves_range, CURVE_TYPE_POLY);

  /* Set vertex color for fill and stroke. */
  const bool use_vertex_color = (fd->vc.scene->toolsettings->gp_paint->mode ==
                                 GPPAINT_FLAG_USE_VERTEXCOLOR);
  const bool use_vertex_color_stroke = use_vertex_color &&
                                       ELEM(fd->brush->gpencil_settings->vertex_mode,
                                            GPPAINT_MODE_STROKE,
                                            GPPAINT_MODE_BOTH);
  const bool use_vertex_color_fill = use_vertex_color &&
                                     ELEM(fd->brush->gpencil_settings->vertex_mode,
                                          GPPAINT_MODE_FILL,
                                          GPPAINT_MODE_BOTH);
  const ColorGeometry4f vertex_color_stroke = use_vertex_color_stroke ?
                                                  ColorGeometry4f(
                                                      fd->brush->rgb[0],
                                                      fd->brush->rgb[1],
                                                      fd->brush->rgb[2],
                                                      fd->brush->gpencil_settings->vertex_factor) :
                                                  ColorGeometry4f(0.0f, 0.0f, 0.0f, 0.0f);
  const ColorGeometry4f vertex_color_fill = use_vertex_color_fill ?
                                                ColorGeometry4f(
                                                    fd->brush->rgb[0],
                                                    fd->brush->rgb[1],
                                                    fd->brush->rgb[2],
                                                    fd->brush->gpencil_settings->vertex_factor) :
                                                ColorGeometry4f(0.0f, 0.0f, 0.0f, 0.0f);

  drawing.vertex_colors_for_write().slice(new_points_range).fill(vertex_color_stroke);
  bke::SpanAttributeWriter<ColorGeometry4f> fill_colors =
      attributes.lookup_or_add_for_write_span<ColorGeometry4f>("fill_color", ATTR_DOMAIN_CURVE);
  fill_colors.span.slice(new_curves_range).fill(vertex_color_fill);
  fill_colors.finish();

  /* Set material. */
  Material *material = BKE_grease_pencil_object_material_ensure_from_active_input_brush(
      fd->vc.bmain, fd->vc.obact, fd->brush);
  const int material_index = BKE_grease_pencil_object_material_index_get(fd->vc.obact, material);

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

  curves.curve_types_for_write().last() = CURVE_TYPE_POLY;
  curves.update_curve_types();

  /* Set notifiers. */
  drawing.tag_topology_changed();
  DEG_id_tag_update(&fd->grease_pencil->id, ID_RECALC_GEOMETRY);
  WM_main_add_notifier(NC_GEOM | ND_DATA, &fd->grease_pencil->id);
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
static Vector<float2> get_closed_fill_edge_as_2d_polygon(FillData *fd)
{
  Vector<float2> points;

  /* We cut some corners here, literally, because we don't calculate the exact intersection point
   * of intersections. Since the 2D polygon is only used for a mouse-position-inside-polygon
   * check, we can afford to be a little sloppy. */
  for (auto &segment : fd->vf.segments) {
    if ((segment.flag & vfFlag::IsUnused) == true) {
      continue;
    }

    const int point_offset = fd->curves_2d.point_offset[segment.curve_index_2d];
    const int direction = ((segment.flag & vfFlag::DirectionBackwards) == true) ? -1 : 1;
    int point_start = segment.point_range[0];
    int point_end = segment.point_range[1];
    if (direction == -1) {
      std::swap(point_start, point_end);
    }
    point_end += direction;
    for (int point_i = point_start; point_i != point_end; point_i += direction) {
      points.append(fd->curves_2d.points_2d[point_offset + point_i]);
    }
  }

  return points;
}

static bool mouse_pos_is_inside_closed_edge(FillData *fd)
{
  Vector<float2> closed_edge = get_closed_fill_edge_as_2d_polygon(fd);

  return mouse_pos_is_inside_polygon(fd->mouse_pos, closed_edge);
}

/**
 * Split an edge segment into its original and overlapping one.
 */
static void split_segment_with_overlap(EdgeSegment *segment,
                                       const OverlappingSegment &overlap,
                                       FillData *fd)
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
    segment->point_range[0] = overlap.base_point_range[1];
  }
  else {
    segment->point_range[1] = overlap.base_point_range[0];
  }
  segment->flag &= ~vfFlag::IsIntersected;

  EdgeSegment new_seg{};
  new_seg.curve_index_2d = overlap.overlapping_curve_index;
  new_seg.point_range = overlap.overlapping_point_range;
  new_seg.flag |= vfFlag::IsPartOfSet;
  if (overlap.overlapping_backwards) {
    new_seg.flag |= vfFlag::DirectionBackwards;
  }
  fd->vf.segments.append(std::move(new_seg));
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
  tail->flag &= ~vfFlag::GapClosureByExtension;
  tail->flag &= ~vfFlag::GapClosureByProximity;
  tail->flag &= ~vfFlag::IsIntersected;
}

/**
 * Check if edge segments overlap. This includes a check of alternative segments,
 * that share points with the original one.
 */
static bool segments_have_overlap(EdgeSegment *head, EdgeSegment *tail, FillData *fd)
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
  for (const auto &overlap : fd->vf.overlapping_segments) {
    if (head->curve_index_2d == overlap.overlapping_curve_index &&
        head->point_range[0] <= overlap.overlapping_point_range[1] &&
        head->point_range[1] >= overlap.overlapping_point_range[0])
    {
      split_segment_with_overlap(tail, overlap, fd);
      tail = &fd->vf.segments.last();
      remove_overlapping_edge_tail_points(head, tail, head->point_range, tail->point_range);
      return true;
    }
  }

  return false;
}

/**
 * Check if a set of edge segments form a closed loop.
 */
static bool is_closed_fill_edge(FillData *fd)
{
  /* Init: by default all segments are used in the edge. */
  for (auto &segment : fd->vf.segments) {
    segment.flag &= ~vfFlag::IsUnused;
  }

  /* Loop through the edge segments and see if there is overlap with the last one. */
  EdgeSegment *tail = &fd->vf.segments.last();
  for (const int segment_i : fd->vf.segments.index_range().drop_back(1)) {
    if (segments_have_overlap(&fd->vf.segments[segment_i], tail, fd)) {
      return true;
    }
    fd->vf.segments[segment_i].flag |= vfFlag::IsUnused;
  }

  return false;
}

/**
 * Apply epsilon to an angle around PI and 2 * PI.
 */
static float get_rounded_angle(const float angle)
{
  float rounded = angle;

  if (rounded > M_PI - ANGLE_EPSILON && rounded <= M_PI) {
    rounded = M_PI;
  }

  if (rounded >= (2 * M_PI - ANGLE_EPSILON)) {
    rounded -= 2 * M_PI;
    if (rounded < 0.0f) {
      rounded = 0.0f;
    }
  }

  return rounded;
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
 * Add proximity 'end' information to an edge segment:
 * - Curve index, curve point index of curves in proximity.
 * - Determine the turns (angles) for the proximity curves.
 */
static void add_curves_in_proximity_of_segment(EdgeSegment *segment, const FillData *fd)
{
  /* Get curves in proximity of segment. */
  const int segment_direction = ((segment->flag & vfFlag::DirectionBackwards) == true ? -1 : 1);
  Vector<ProximityCurve> proximities = get_curves_in_proximity_of_segment(
      segment, segment_direction, fd);

  /* Process the proximity curves. */
  for (const auto &proxim : proximities) {
    /* Copy proximity data. */
    SegmentEnd segment_end{};
    segment_end.curve_index_2d = proxim.curve_index_2d;
    segment_end.ori_segment_point_start = proxim.segment_point_start;
    segment_end.ori_segment_point_end = proxim.segment_point_end;
    segment_end.flag = vfFlag::GapClosureByProximity;

    /* Get the segment and proximity curve points. */
    const int segment_point_offset = fd->curves_2d.point_offset[segment->curve_index_2d];
    const int curve_i = segment_end.curve_index_2d;
    const int curve_point_offset = fd->curves_2d.point_offset[curve_i];
    const float2 segment_a =
        fd->curves_2d.points_2d[segment_point_offset + segment_end.ori_segment_point_start];
    const float2 segment_b =
        fd->curves_2d.points_2d[segment_point_offset + segment_end.ori_segment_point_end];
    const float2 curve_p1 = fd->curves_2d.points_2d[curve_point_offset + proxim.curve_point_start];
    const float2 curve_p2 = fd->curves_2d.points_2d[curve_point_offset + proxim.curve_point_end];

    /* Determine the angle between the segment and the proximity curve. */
    segment_end.angle_is_set[0] = true;
    segment_end.angle_is_set[1] = true;
    const float2 segment_vec = segment_b - segment_a;
    bool dir_backwards = false;
    float angle0 = 0.0f, angle1 = 0.0f;

    if (proxim.curve_point_start_is_nearest) {
      segment_end.point_end = proxim.curve_point_start;
      angle0 = get_rounded_angle(M_PI - angle_signed_v2v2(segment_vec, curve_p2 - segment_a));

      const int curve_point_prev = get_next_curve_point(proxim.curve_point_start,
                                                        -1,
                                                        fd->curves_2d.point_size[curve_i],
                                                        fd->curves_2d.is_cyclic[curve_i]);
      if (curve_point_prev == -1) {
        segment_end.angle_is_set[1] = false;
        angle1 = 0.0f;
        segment_end.point_start = proxim.curve_point_start;
      }
      else {
        const float2 curve_p0 = fd->curves_2d.points_2d[curve_point_offset + curve_point_prev];
        angle1 = get_rounded_angle(M_PI - angle_signed_v2v2(segment_vec, curve_p0 - segment_a));

        if (angle0 < angle1) {
          segment_end.point_start = proxim.curve_point_end;
        }
        else {
          dir_backwards = true;
          segment_end.point_start = proxim.curve_point_start;
          segment_end.point_end = curve_point_prev;
        }
      }
    }
    else {
      segment_end.point_end = proxim.curve_point_end;
      angle0 = get_rounded_angle(M_PI - angle_signed_v2v2(segment_vec, curve_p1 - segment_a));

      const int curve_point_next = get_next_curve_point(proxim.curve_point_end,
                                                        1,
                                                        fd->curves_2d.point_size[curve_i],
                                                        fd->curves_2d.is_cyclic[curve_i]);
      if (curve_point_next == -1) {
        segment_end.angle_is_set[1] = false;
        angle1 = 0.0f;
        segment_end.point_start = proxim.curve_point_end;
        dir_backwards = true;
      }
      else {
        const float2 curve_p3 = fd->curves_2d.points_2d[curve_point_offset + curve_point_next];
        angle1 = get_rounded_angle(M_PI - angle_signed_v2v2(segment_vec, curve_p3 - segment_a));

        if (angle0 < angle1) {
          dir_backwards = true;
          segment_end.point_start = proxim.curve_point_start;
        }
        else {
          segment_end.point_start = proxim.curve_point_end;
          segment_end.point_end = curve_point_next;
        }
      }
    }

    segment_end.angle[0] = angle0;
    segment_end.angle[1] = angle1;
    segment_end.angle_min = (segment_end.first_angle_is_smallest() ? angle0 : angle1);
    segment_end.angle_max = (segment_end.first_angle_is_largest() ? angle0 : angle1);
    if (dir_backwards) {
      segment_end.flag |= vfFlag::DirectionBackwards;
    }

    /* When using the end point of the segment, shift the point range of the segment
     * by one. */
    if (proxim.use_segment_point_end) {
      segment_end.ori_segment_point_start = segment_end.ori_segment_point_end;
      segment_end.ori_segment_point_end += segment_direction;

      if (segment_end.ori_segment_point_end < segment->point_range[0] ||
          segment_end.ori_segment_point_end > segment->point_range[1])
      {
        segment_end.ori_segment_point_end = -1;
      }
    }

    /* Append to segment end list. */
    segment->segment_ends.append(segment_end);
  }

  /* Sort proximity segment ends on segment start point and angle. */
  std::sort(segment->segment_ends.begin(),
            segment->segment_ends.end(),
            [segment_direction](const SegmentEnd &a, const SegmentEnd &b) {
              if ((a.flag & vfFlag::GapClosureByProximity) !=
                  (b.flag & vfFlag::GapClosureByProximity)) {
                return (a.flag & vfFlag::GapClosureByProximity) == false;
              }
              if (a.ori_segment_point_start == b.ori_segment_point_start) {
                if (compare_ff(a.angle_min, b.angle_min, ANGLE_EPSILON)) {
                  return a.angle_max < b.angle_max;
                }
                return a.angle_min < b.angle_min;
              }
              if (segment_direction == 1) {
                return a.ori_segment_point_start < b.ori_segment_point_start;
              }
              return a.ori_segment_point_start > b.ori_segment_point_start;
            });

  /* Set segment flags. */
  segment->flag &= ~vfFlag::IsIntersected;
  segment->flag &= ~vfFlag::GapClosureByExtension;
  segment->flag |= vfFlag::GapClosureByProximity;
}

/**
 * Add intersection 'end' information to an edge segment:
 * - Does the segment end by curve intersection or curve extension (gap closure)?
 * - Determine the turns (intersection angles) for the segment ends.
 */
static void add_segment_ends_intersection(const Curves2DSpace *curves_2d,
                                          EdgeSegment *segment,
                                          const float2 &segment_point_a,
                                          const float2 &segment_point_b,
                                          const int segment_point_b_index,
                                          const Vector<IntersectingCurve> &intersections,
                                          const bool is_end_extension,
                                          const FillData *fd)
{
  const float2 segment_vec = segment_point_b - segment_point_a;

  for (const auto &intersection : intersections) {
    /* Skip intersections before the start segment distance. */
    if ((segment->flag & vfFlag::CheckStartDistance) == true &&
        intersection.distance[0] < fd->vf.start_distance)
    {
      continue;
    }
    /* Skip end extensions of cyclic curves. */
    if (is_end_extension && curves_2d->is_cyclic[intersection.curve_index_2d]) {
      continue;
    }
    /* Skip end extensions at distance 0. */
    if (is_end_extension && intersection.distance[0] < DISTANCE_EPSILON) {
      continue;
    }

    /* Copy intersection data. */
    SegmentEnd segment_end{};
    segment_end.curve_index_2d = intersection.curve_index_2d;
    segment_end.point_start = intersection.point_start;
    segment_end.point_end = intersection.point_end;
    segment_end.ori_segment_point_end = segment_point_b_index;
    segment_end.distance = intersection.distance[0];
    if (intersection.has_stroke) {
      segment_end.flag |= vfFlag::HasStroke;
    }

    /* Inverse the distance when walking backwards on a curve and an intersection with an end
     * extension is found. This is because the intersection distances of end extension are
     * precalculated and always determined in ascending curve point order. */
    if (is_end_extension && segment_point_b_index != -1 &&
        (segment->flag & vfFlag::DirectionBackwards) == true)
    {
      segment_end.distance = 1.0f - intersection.distance[0];
    }

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

    float angle0 = get_rounded_angle(M_PI - angle_signed_v2v2(segment_vec, p2 - p1));
    float angle1 = get_rounded_angle(angle0 + M_PI);
    bool dir_backwards = (angle0 >= M_PI);

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
        angle1 = get_rounded_angle(M_PI - angle_signed_v2v2(segment_vec, p0 - p1));
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
        angle1 = get_rounded_angle(M_PI - angle_signed_v2v2(segment_vec, p3 - p2));
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
        segment_end.point_start = fd->curves_2d.point_size[segment_end.curve_index_2d] - 1;
      }
      segment_end.point_end = intersection.point_start;

      /* Set stroke material flag. */
      if (fd->curves_2d.has_stroke[segment_end.curve_index_2d]) {
        segment_end.flag |= vfFlag::HasStroke;
      }
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
 * Expand an edge segment by walking along the curve. An edge segment ends when an intersection
 * is found or when the end of the curve is reached. At the end of a curve, gap closure
 * inspection is performed.
 */
static bool walk_along_curve(FillData *fd, EdgeSegment *segment, const int started_at = -1)
{
  /* Skip when segment is already inspected. */
  if ((segment->flag & vfFlag::IsInspected) == true) {
    return false;
  }
  segment->flag |= vfFlag::IsInspected;

  /* Get curve point start, range etc. */
  const int curve_i = segment->curve_index_2d;
  const bool is_cyclic = fd->curves_2d.is_cyclic[curve_i];
  const int direction = ((segment->flag & vfFlag::DirectionBackwards) == true) ? -1 : 1;
  const int point_size = fd->curves_2d.point_size[curve_i];
  const int point_offset = fd->curves_2d.point_offset[curve_i];
  int point_i = segment->point_range[0];
  const int point_started = (started_at == -1 ? point_i : started_at);

  float2 segment_point_a = {FLT_MAX, FLT_MAX};
  const int point_prev = get_next_curve_point(point_i, -direction, point_size, is_cyclic);
  if (point_prev != -1) {
    segment_point_a = fd->curves_2d.points_2d[point_offset + point_prev];
  }

  fd->vf.overlapping_segments.clear();

  bool check_gaps = false;

  /* Walk along the curve until an intersection or the end of the curve is reached. */
  do {
    /* Check curve bounds. */
    if (point_i < 0 || point_i >= point_size) {
      point_i = std::clamp(point_i, 0, point_size - 1);
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
      fd->vf.segments.append(new_segment);

      if (point_i == point_started) {
        return true;
      }

      /* Walk along second part of cyclic curve. */
      walk_along_curve(fd, &fd->vf.segments.last(), point_started);
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
    segment_point_a = fd->curves_2d.points_2d[point_offset + point_i];
    const float2 segment_point_b = fd->curves_2d.points_2d[point_offset + point_next];
    float2 point_b_next = {FLT_MAX, FLT_MAX};
    const int point_next2 = get_next_curve_point(point_next, direction, point_size, is_cyclic);
    if (point_next2 != -1) {
      point_b_next = fd->curves_2d.points_2d[point_offset + point_next2];
    }

    /* Get intersections with other curves. */
    Vector<IntersectingCurve> intersections = get_intersections_of_segment_with_curves(
        segment_point_a,
        segment_point_b,
        curve_i,
        &fd->curves_2d,
        point_a_prev,
        point_b_next,
        store_overlapping_segment,
        point_i,
        point_next,
        direction,
        true,
        fd);
    add_segment_ends_intersection(&fd->curves_2d,
                                  segment,
                                  segment_point_a,
                                  segment_point_b,
                                  point_next,
                                  intersections,
                                  false,
                                  fd);

    /* Get intersections with curve end extensions. */
    if (fd->use_gap_close_extend) {
      intersections.clear();

      /* Find extension intersection by serial lookup (they have already been calculated in the
       * interactive step of the modal operator).
       * Note: we deliberately create a copy of the intersection, because we change the values.
       */
      for (auto intersection : fd->extension_intersections) {
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

      add_segment_ends_intersection(&fd->extensions_2d,
                                    segment,
                                    segment_point_a,
                                    segment_point_b,
                                    point_next,
                                    intersections,
                                    true,
                                    fd);
    }

    /* When one or more intersections are found, we can stop walking along the curve. */
    if (!segment->segment_ends.is_empty()) {
      /* Sort intersections on distance and angle. This is important to find the narrowest edge.
       * At equal distance, give priority to a curve with a stroke material. */
      std::sort(segment->segment_ends.begin(),
                segment->segment_ends.end(),
                [](const SegmentEnd &a, const SegmentEnd &b) {
                  if (compare_ff(a.distance, b.distance, DISTANCE_SORT_EPSILON)) {
                    if ((a.flag & vfFlag::HasStroke) != (b.flag & vfFlag::HasStroke)) {
                      return (a.flag & vfFlag::HasStroke) == true;
                    }
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
  if (fd->use_gap_close_extend) {
    /* Get the coordinates of start/end extension. */
    const int extension_index = curve_i * 2 + (direction == 1 ? 1 : 0);
    const int point_offset = fd->extensions_2d.point_offset[extension_index];
    const float2 segment_point_a = fd->extensions_2d.points_2d[point_offset];
    const float2 segment_point_b = fd->extensions_2d.points_2d[point_offset + 1];

    /* Abort when the end extension doesn't intersect with anything. */
    if (!fd->extension_has_intersection[extension_index]) {
      return true;
    }

    /* Find the intersections of the end extension by serial lookup. The intersection data is
     * already calculated during the interactive step of the modal operator. */
    Vector<IntersectingCurve> intersections_curves;
    Vector<IntersectingCurve> intersections_extensions;
    for (const auto &intersection : fd->extension_intersections) {
      if (intersection.extension_index != extension_index) {
        continue;
      }

      if (intersection.with_end_extension) {
        intersections_extensions.append(intersection);
      }
      else {
        intersections_curves.append(intersection);
      }
    }

    /* Add intersections with other curve. */
    add_segment_ends_intersection(&fd->curves_2d,
                                  segment,
                                  segment_point_a,
                                  segment_point_b,
                                  -1,
                                  intersections_curves,
                                  false,
                                  fd);

    /* Add intersections with other curve end extension. */
    add_segment_ends_intersection(&fd->extensions_2d,
                                  segment,
                                  segment_point_a,
                                  segment_point_b,
                                  -1,
                                  intersections_extensions,
                                  true,
                                  fd);

    if (!segment->segment_ends.is_empty()) {
      /* Sort intersections on distance and angle. This is important to find the narrowest edge. */
      std::sort(segment->segment_ends.begin(),
                segment->segment_ends.end(),
                [](const SegmentEnd &a, const SegmentEnd &b) {
                  if (compare_ff(a.distance, b.distance, DISTANCE_SORT_EPSILON)) {
                    if ((a.flag & vfFlag::HasStroke) != (b.flag & vfFlag::HasStroke)) {
                      return (a.flag & vfFlag::HasStroke) == true;
                    }
                    if (compare_ff(a.angle_min, b.angle_min, ANGLE_EPSILON)) {
                      return a.angle_max < b.angle_max;
                    }
                    return a.angle_min < b.angle_min;
                  }
                  return a.distance < b.distance;
                });

      /* Set gap closure flag. */
      segment->flag |= vfFlag::GapClosureByExtension;
    }
  }

  /* Check for gap closure with radii at the ends of curves. */
  if (fd->use_gap_close_radius) {
    /* Get coordinates of curve start/end point. */
    const int curve_point_end = point_offset + point_i;
    const float2 curve_point = fd->curves_2d.points_2d[curve_point_end];
    const bool at_start_of_curve = (point_i == 0);

    /* Get the segment coordinates at the start/end of the curve. */
    const int curve_point_end_prev = curve_point_end +
                                     math::min(point_size - 1, 1) * (at_start_of_curve ? 1 : -1);
    const float2 curve_point_prev = fd->curves_2d.points_2d[curve_point_end_prev];
    const float2 curve_end_vec = curve_point - curve_point_prev;
    const float max_dist = 2 * fd->gap_distance;

    /* Find nearest end points.
     * Note: the number of points is a bit arbitrary, but the defined number will suffice for
     * normal cases. */
    KDTreeNearest_2d nearest[RADIUS_NEAREST_NUM];
    const int nearest_num = BLI_kdtree_2d_find_nearest_n(
        fd->curve_ends, curve_point, nearest, RADIUS_NEAREST_NUM);

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
      const int nearest_point_offset = fd->curves_2d.point_offset[nearest_curve];
      const int nearest_point_i = (nearest_at_start_of_curve ?
                                       0 :
                                       fd->curves_2d.point_size[nearest_curve] - 1);

      /* Skip self (the curve end we are gap-closing). */
      if (nearest_curve == curve_i && nearest_at_start_of_curve == at_start_of_curve) {
        continue;
      }
      /* Skip cyclic curves. */
      if (fd->curves_2d.is_cyclic[nearest_curve]) {
        continue;
      }

      /* Set segment end values. */
      SegmentEnd segment_end{};
      segment_end.flag = (nearest_at_start_of_curve ? vfFlag::None : vfFlag::DirectionBackwards);
      segment_end.curve_index_2d = nearest_curve;
      segment_end.point_start = nearest_point_i;

      /* Determine the angle between the curve start/end and the nearest point. For finding the
       * narrowest fill edge, we explore the smallest angle first (= most right turn). */
      const float2 nearest_vec = fd->curves_2d.points_2d[nearest_point_offset + nearest_point_i] -
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
    }
  }

  return true;
}

/**
 * Get the next turn on a curve intersection or gap closure.
 */
static void take_next_turn_on_intersection(FillData *fd,
                                           SegmentEnd *segment_end,
                                           const EdgeSegment *ori_segment)
{
  vfSegmentTurn turn[3] = {vfSegmentTurn::None};
  int first_angle_index, second_angle_index, straight_ahead_index;
  const bool at_end_extension = ((ori_segment->flag & vfFlag::IsIntersected) == false) &&
                                ((segment_end->flag & vfFlag::GapClosureByProximity) == false);

  /* The most common case is a three-way intersection: right turn, straight ahead and left turn.
   * But we have to handle the edge cases: the intersecting curve on one side only, skipping
   * redundant turns etc. */
  if (segment_end->angle_min >= M_PI) {
    /* Intersecting curve entirely on the left side. */
    straight_ahead_index = 0;
    first_angle_index = 1;
    second_angle_index = 2;
  }
  else if (segment_end->angle_max < M_PI) {
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

  /* Clear the straight ahead turn when at an end extension. */
  if (at_end_extension && current_turn == vfSegmentTurn::StraightAhead) {
    current_turn = vfSegmentTurn::None;
  }

  /* Clear the straight ahead turn when there is no segment point left to follow.*/
  if (current_turn == vfSegmentTurn::StraightAhead && segment_end->ori_segment_point_end == -1) {
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

  fd->vf.segments.append(segment);
}

/**
 * Get the next curve found by gap closure.
 */
static void take_next_gap_closure_curve(FillData *fd, SegmentEnd *segment_end)
{
  /* Add new segment to the fill edge. */
  EdgeSegment segment{};
  segment.curve_index_2d = segment_end->curve_index_2d;
  segment.init_point_range(segment_end->point_start);
  if ((segment_end->flag & vfFlag::DirectionBackwards) == true) {
    segment.flag = vfFlag::DirectionBackwards;
  }

  fd->vf.segments.append(segment);
}

/**
 * Remove the last edge segment. Handles segment sets.
 */
static void remove_last_segment(FillData *fd)
{
  /* Remove segment. */
  EdgeSegment *segment = &fd->vf.segments.last();
  const bool is_part_of_set = ((segment->flag & vfFlag::IsPartOfSet) == true);
  fd->vf.segments.remove_last();

  /* Repeat when segment is part of a set. */
  if (is_part_of_set) {
    remove_last_segment(fd);
  }
}

#ifdef GP_FILL_DEBUG_MODE
/**
 * Debug function: print edge segment data to the console.
 */
static void debug_print_segment_data(FillData *fd, EdgeSegment *segment, const bool proximity_only)
{
  if (!proximity_only) {
    printf("Segments: %lld, walked along: curve %d (%d) %d-%d, flag %d,%s end segments %lld \n",
           fd->vf.segments.size(),
           segment->curve_index_2d,
           fd->curves_2d.point_size[segment->curve_index_2d],
           segment->point_range[0],
           segment->point_range[1],
           segment->flag,
           (segment->flag & vfFlag::DirectionBackwards) == true ? " backwards," : "",
           segment->segment_ends.size());
  }

  for (auto seg_end : segment->segment_ends) {
    if (!proximity_only || (seg_end.flag & vfFlag::GapClosureByProximity) == true) {
      printf("   - segment end: curve %d %d-%d, distance %.3f, angles %.2f %.2f%s%s%s",
             seg_end.curve_index_2d,
             seg_end.point_start,
             seg_end.point_end,
             seg_end.distance,
             seg_end.angle_is_set[0] ? seg_end.angle[0] : 9.99f,
             seg_end.angle_is_set[1] ? seg_end.angle[1] : 9.99f,
             (seg_end.flag & vfFlag::DirectionBackwards) == true ? ", backwards" : "",
             (seg_end.flag & vfFlag::WithEndExtension) == true ? ", end extension" : "",
             (seg_end.flag & vfFlag::GapClosureByProximity) == true ? ", in proximity" : "");
      if ((seg_end.flag & vfFlag::WithEndExtension) == false) {
        const int offset = fd->curves_2d.point_offset[seg_end.curve_index_2d];
        printf(", %.1f %.1f - %.1f %.1f",
               fd->curves_2d.points_2d[offset + seg_end.point_start][0],
               fd->curves_2d.points_2d[offset + seg_end.point_start][1],
               fd->curves_2d.points_2d[offset + seg_end.point_end][0],
               fd->curves_2d.points_2d[offset + seg_end.point_end][1]);
      }
      printf("\n");
    }
  }
}
#endif

/**
 * Find a closed fill edge, by creating a chain of edge segments (parts of curves).
 * The edge is build in clockwise direction. The narrowest edge is created by taking
 * the most right turn first at curve intersections.
 */
static bool find_closed_fill_edge(FillData *fd)
{
#ifdef GP_FILL_DEBUG_MODE
  int iteration = 0;
#endif

  while (!fd->vf.segments.is_empty()) {

#ifdef GP_FILL_DEBUG_MODE
    /* Limit number of iterations in debug mode. */
    if (iteration++ > 200) {
      printf("Aborted, more than 200 iterations...\n");
      return false;
    }
#else
    /* Emergency break: when the operator is taking too much time, jump to the conclusion that no
     * closed fill edge can be found. */
    auto t2 = std::chrono::high_resolution_clock::now();
    auto delta_t = std::chrono::duration_cast<std::chrono::milliseconds>(
        t2 - fd->vf.operator_time_start);
    if (delta_t.count() > MAX_EXECUTION_TIME) {
      return false;
    }
#endif

    /* Explore the last edge segment in the array. */
    EdgeSegment *segment = &fd->vf.segments.last();

    /* Walk along the curve until an intersection or the end of the curve is reached. */
    bool changed = walk_along_curve(fd, segment);
    segment = &fd->vf.segments.last();

#ifdef GP_FILL_DEBUG_MODE
    if (changed) {
      debug_print_segment_data(fd, segment, false);
    }
#endif

    /* Check if the edge is closed. */
    if (changed) {
      if (is_closed_fill_edge(fd)) {
        /* Check if the mouse click position is inside the loop. */
        if (mouse_pos_is_inside_closed_edge(fd)) {
          return true;
        }

        /* Remove the closing segment and continue searching. */
        remove_last_segment(fd);
        continue;
      }
    }

    /* Inpsect the segment ends. */
    if (segment->segment_ends_index < segment->segment_ends.size()) {
      SegmentEnd *segment_end = &segment->segment_ends[segment->segment_ends_index];

      /* When jumping to a curve in proximity, adjust the point range of the segment according to
       * the location of the curve. */
      if ((segment_end->flag & vfFlag::GapClosureByProximity) == true) {
        if ((segment->flag & vfFlag::DirectionBackwards) == true) {
          segment->point_range[0] = std::min(segment_end->ori_segment_point_start,
                                             segment_end->ori_segment_point_end);
        }
        else {
          segment->point_range[1] = std::max(segment_end->ori_segment_point_start,
                                             segment_end->ori_segment_point_end);
        }
      }

      /* Handle segment that ends at an intersection or gap closure with several turns. */
      if ((segment->flag & vfFlag::IsIntersected) == true ||
          (segment->flag & vfFlag::GapClosureByProximity) == true ||
          ((segment->flag & vfFlag::GapClosureByExtension) == true &&
           (segment_end->flag & vfFlag::WithEndExtension) == false))
      {
        /* When all intersection turns are explored, continue with the
         * next intersection (if any). */
        if (segment_end->turn >= 3) {
          segment->segment_ends_index++;
          continue;
        }

        /* Take the next turn on the intersection: right, straight ahead, left. */
        take_next_turn_on_intersection(fd, segment_end, segment);

        segment_end->turn++;
      }
      else {
        /* Gap closure with radius or with another curve end extension: jump onto the next
         * gap-closed curve. */
        if (segment_end->turn >= 1) {
          segment->segment_ends_index++;
          continue;
        }

        take_next_gap_closure_curve(fd, segment_end);
        segment_end->turn++;
      }
    }
    else if (fd->use_gap_close_proximity &&
             (segment->flag & vfFlag::ProximityInspected) == false &&
             segment->point_range[0] != segment->point_range[1])
    {
      /* Gap closure by proximity: check for curves in proximity of this segment. */
      segment->flag |= vfFlag::ProximityInspected;
      add_curves_in_proximity_of_segment(segment, fd);

#ifdef GP_FILL_DEBUG_MODE
      debug_print_segment_data(fd, segment, true);
#endif
    }
    else {
      /* We reached a dead end, delete the segment and continue with the previous. */
      remove_last_segment(fd);
    }
  }

  return false;
}

/**
 * Execute the fill operation, at the second mouse click. (The first mouse click is for the
 * interactive gap closure phase.)
 */
bool vector_fill_do(FillData *fd)
{
  /* Find a first (arbitrary) edge point of the fill area by casting a ray from the mouse click
   * position in one of four directions: up, right, down, left. When this ray crosses a curve,
   * we use the intersection point as the starting point of the fill edge.
   * We check in four directions, because the user can click near a gap in the fill edge. */
  float2 ray_vec;
  vfRayDirection ray_direction;
  fd->vf.operator_time_start = std::chrono::high_resolution_clock::now();
  const float ray_delta = 20000.0f;
  bool found = false;

  for (auto ray_dir : ray_directions) {
    ray_direction = ray_dir;
    ray_vec = fd->mouse_pos;

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

    fd->vf.starting_segments = get_intersections_of_segment_with_curves(
        fd->mouse_pos, ray_vec, -1, &fd->curves_2d);

    if (fd->vf.starting_segments.size() == 0) {
      continue;
    }

    /* Sort starting segments on distance (closest first). At equal distance, give priority to a
     * curve with stroke material. */
    std::sort(fd->vf.starting_segments.begin(),
              fd->vf.starting_segments.end(),
              [](const IntersectingCurve &a, const IntersectingCurve &b) {
                if (compare_ff(a.distance[0], b.distance[0], DISTANCE_SORT_EPSILON)) {
                  if (a.has_stroke != b.has_stroke) {
                    return a.has_stroke;
                  }
                }
                return a.distance[0] < b.distance[0];
              });

    /* From the first edge point we try to follow the fill edge clockwise, until we find
     * a closed loop. */
    IntersectingCurve *start_segment = &fd->vf.starting_segments.first();

    /* Start with an empty edge segment list. */
    fd->vf.segments.clear();

    /* Add first edge segment. */
    EdgeSegment segment{};
    segment.curve_index_2d = start_segment->curve_index_2d;
    segment.init_point_range(start_segment->point_start);

    /* Set the minimum distance for checking intersections on this first segment. */
    segment.flag = vfFlag::CheckStartDistance;
    fd->vf.start_distance = start_segment->distance[1];

    /* We want to follow the fill edge clockwise. Determine in which direction we have to follow
     * the curve for that. */
    const int point_offset = fd->curves_2d.point_offset[segment.curve_index_2d];
    const float2 segment_a = fd->curves_2d.points_2d[point_offset + start_segment->point_start];
    const float2 segment_b = fd->curves_2d.points_2d[point_offset + start_segment->point_end];
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
      fd->vf.start_distance = 1.0f - fd->vf.start_distance;
    }

    fd->vf.segments.append(segment);

    /* See if we can expand this first edge segment to a closed edge. */
    found = find_closed_fill_edge(fd);
    if (found) {
      break;
    }
  }

  if (found) {
    /* Create the 3D fill curve geometry. */
    create_fill_geometry(fd);
  }

  return found;
}

/** \} */

}  // namespace blender::ed::greasepencil::fill
