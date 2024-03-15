/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgreasepencil
 */

#include "BLI_array.hh"
#include "BLI_lasso_2d.hh"
#include "BLI_math_geom.h"
#include "BLI_rect.h"
#include "BLI_task.hh"

#include "BKE_context.hh"
#include "BKE_crazyspace.hh"
#include "BKE_curves.hh"
#include "BKE_report.hh"

#include "DEG_depsgraph_query.hh"

#include "ED_grease_pencil.hh"
#include "ED_view3d.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"

#include "WM_api.hh"

namespace blender::ed::greasepencil {

/**
 * Structure used by the cutter tool, describing a curve segment (a point range in a curve)
 * that needs to be removed from the curve.
 */
struct CutterSegment {
  /* Curve index. */
  int curve;
  /* Point range of the segment: starting point and end point. Matches the point offsets
   * in a CurvesGeometry. */
  int point_range[2];
  /* Intersection factor (0..1) of the intersection between:
   * - point start - 1 and point start
   * - point end and point end + 1
   */
  float intersection_factor[2];
  /* Intersection flag: true if point start/end is the result of an intersection
   * and false if the point is the outer end of a curve. */
  bool is_intersected[2] = {false};
};

/**
 * Structure used by the cutter tool, describing:
 * - a collection of cutter segments
 * - a flag for all curves affected by the lasso tool
 */
struct CutterSegments {
  /* Collection of cutter segments: parts of curves between other curves, to be removed from the
   * geometry. */
  Vector<CutterSegment> segments;
  /* Flag for curves: true if a curve is (partially) inside a lasso area. */
  Array<bool> curve_in_lasso_area;

  /* Check if a curve point is already stored in a curve segment. */
  bool point_is_in_segment(const int curve, const int point)
  {
    if (!this->curve_in_lasso_area[curve]) {
      return false;
    }
    for (auto &segment : this->segments) {
      if (segment.curve == curve && point >= segment.point_range[0] &&
          point <= segment.point_range[1])
      {
        return true;
      }
    }
    return false;
  }

  /* Create a cutter segment with a point range of one point. */
  CutterSegment *create_segment(const int curve, const int point)
  {
    this->curve_in_lasso_area[curve] = true;

    CutterSegment segment{};
    segment.curve = curve;
    segment.point_range[0] = point;
    segment.point_range[1] = point;

    this->segments.append(std::move(segment));

    return &this->segments.last();
  }

  /* Merge cutter segments that are next to each other. */
  void merge_adjacent_segments()
  {
    Vector<CutterSegment> merged_segments;

    /* Note on performance: we deal with small numbers here, so we can afford the double loop. */
    while (!this->segments.is_empty()) {
      CutterSegment a = this->segments.pop_last();

      bool merged = false;
      for (auto &b : merged_segments) {
        if (a.curve != b.curve) {
          continue;
        }
        /* The segments overlap when the points ranges have overlap or are exactly adjacent. */
        if ((a.point_range[0] <= b.point_range[1] && a.point_range[1] >= b.point_range[0]) ||
            (a.point_range[1] == b.point_range[0] - 1) ||
            (b.point_range[1] == a.point_range[0] - 1))
        {
          /* Merge the point ranges and related intersection data. */
          const bool take_start_a = a.point_range[0] < b.point_range[0];
          const bool take_end_a = a.point_range[1] > b.point_range[1];
          b.point_range[0] = take_start_a ? a.point_range[0] : b.point_range[0];
          b.point_range[1] = take_end_a ? a.point_range[1] : b.point_range[1];
          b.is_intersected[0] = take_start_a ? a.is_intersected[0] : b.is_intersected[0];
          b.is_intersected[1] = take_end_a ? a.is_intersected[1] : b.is_intersected[1];
          b.intersection_factor[0] = take_start_a ? a.intersection_factor[0] :
                                                    b.intersection_factor[0];
          b.intersection_factor[1] = take_end_a ? a.intersection_factor[1] :
                                                  b.intersection_factor[1];
          merged = true;
          break;
        }
      }
      if (!merged) {
        merged_segments.append(std::move(a));
      }
    }

    this->segments = merged_segments;
  }
};

/* When looking for intersections, we need a little padding, otherwise we could miss curves
 * that intersect for the eye, but not in hard numbers. */
static constexpr int INTERSECTION_PADDING = 1;
static constexpr int FLOAT_TO_INT_PADDING = 1;
static constexpr int RCTI_PADDING = INTERSECTION_PADDING + FLOAT_TO_INT_PADDING;

/* When creating new intersection points, we don't want them too close to their neighbour,
 * because that clutters the geometry. This threshold defines what 'too close' is. */
static constexpr float DISTANCE_FACTOR_THRESHOLD = 0.01f;

/**
 * Get the intersection distance of two line segments a-b and c-d.
 * The intersection distance is defined as the normalized distance (0..1)
 * from point a to the intersection point of a-b and c-d.
 */
static float get_intersection_distance_of_segments(const float2 &co_a,
                                                   const float2 &co_b,
                                                   const float2 &co_c,
                                                   const float2 &co_d)
{
  /* Get intersection point. */
  const float a1 = co_b[1] - co_a[1];
  const float b1 = co_a[0] - co_b[0];
  const float c1 = a1 * co_a[0] + b1 * co_a[1];

  const float a2 = co_d[1] - co_c[1];
  const float b2 = co_c[0] - co_d[0];
  const float c2 = a2 * co_c[0] + b2 * co_c[1];

  const float det = float(a1 * b2 - a2 * b1);
  BLI_assert(det != 0.0f);

  float2 isect;
  isect[0] = (b2 * c1 - b1 * c2) / det;
  isect[1] = (a1 * c2 - a2 * c1) / det;

  /* Get normalized distance from point a to intersection point. */
  const float length_ab = math::length(co_b - co_a);
  float distance = (length_ab == 0.0f ?
                        0.0f :
                        math::clamp(math::length(isect - co_a) / length_ab, 0.0f, 1.0f));

  return distance;
}

/**
 * Find all curves intersecting with a line segment a-b.
 *
 * \param r_distance_min: (output) the minimum intersection distance on segment a-b
 * \param r_distance_max: (output) the maximum intersection distance on segment a-b
 *
 * \returns true when intersection(s) are found.
 */
static bool get_intersections_of_segment_with_curves(const int point_a,
                                                     const int point_b,
                                                     const int segment_curve_index,
                                                     const bke::CurvesGeometry &src,
                                                     const Span<float2> screen_space_positions,
                                                     const Span<rcti> screen_space_bbox,
                                                     float *r_distance_min,
                                                     float *r_distance_max)
{
  std::mutex mutex;
  const OffsetIndices<int> points_by_curve = src.points_by_curve();
  const VArray<bool> is_cyclic = src.cyclic();
  bool intersected = false;

  /* Get coordinates of segment a-b. */
  const float2 co_a = screen_space_positions[point_a];
  const float2 co_b = screen_space_positions[point_b];
  rcti bbox_ab;
  BLI_rcti_init_minmax(&bbox_ab);
  BLI_rcti_do_minmax_v(&bbox_ab, int2(co_a));
  BLI_rcti_do_minmax_v(&bbox_ab, int2(co_b));
  BLI_rcti_pad(&bbox_ab, RCTI_PADDING, RCTI_PADDING);

  /* Loop all curves, looking for intersecting segments. */
  threading::parallel_for(src.curves_range(), 512, [&](const IndexRange curves) {
    for (const int curve : curves) {
      /* Only process curves with at least two points. */
      const IndexRange points = points_by_curve[curve];
      if (points.size() < 2) {
        continue;
      }

      /* Bounding box check: skip curves that don't overlap segment a-b. */
      if (!BLI_rcti_isect(&bbox_ab, &screen_space_bbox[curve], nullptr)) {
        continue;
      }

      /* Find intersecting curve segments. */
      for (const int point_c : points) {
        if (!is_cyclic[curve] && point_c == points.last()) {
          break;
        }

        const int point_d = (point_c == points.last()) ? points.first() : (point_c + 1);

        /* Don't self check. */
        if (curve == segment_curve_index &&
            (point_a == point_c || point_a == point_d || point_b == point_c || point_b == point_d))
        {
          continue;
        }

        /* Skip when bounding boxes of a-b and c-d don't overlap. */
        const float2 co_c = screen_space_positions[point_c];
        const float2 co_d = screen_space_positions[point_d];

        rcti bbox_cd;
        BLI_rcti_init_minmax(&bbox_cd);
        BLI_rcti_do_minmax_v(&bbox_cd, int2(co_c));
        BLI_rcti_do_minmax_v(&bbox_cd, int2(co_d));
        BLI_rcti_pad(&bbox_cd, RCTI_PADDING, RCTI_PADDING);
        if (!BLI_rcti_isect(&bbox_ab, &bbox_cd, nullptr)) {
          continue;
        }

        /* Add some padding to the line segment c-d, otherwise we could just miss an
         * intersection. */
        const float2 padding_cd = math::normalize(co_d - co_c) * INTERSECTION_PADDING;
        const float2 padded_c = co_c - padding_cd;
        const float2 padded_d = co_d + padding_cd;

        /* Check for intersection. */
        const auto isect = math::isect_seg_seg(co_a, co_b, padded_c, padded_d);
        if (ELEM(isect.kind, isect.LINE_LINE_CROSS, isect.LINE_LINE_EXACT)) {
          /* We found an intersection, set the intersection flag. */
          intersected = true;

          /* Calculate the intersection factor. This is the normalized distance (0..1) of the
           * intersection point on line segment a-b, measured from point a. */
          const float distance = get_intersection_distance_of_segments(co_a, co_b, co_c, co_d);

          /* In this case, only the intersection factor has to be stored. The curve point data
           * itself is irrelevant. */
          std::lock_guard lock{mutex};
          *r_distance_min = math::min(distance, *r_distance_min);
          *r_distance_max = math::max(distance, *r_distance_max);
        }
      }
    }
  });

  return intersected;
}

/**
 * Expand a cutter segment of one point by walking along the curve in both directions.
 * A cutter segments ends at an intersection with another curve, or at the end of the curve.
 */
static void expand_cutter_segment(CutterSegment *segment,
                                  const bke::CurvesGeometry &src,
                                  const Span<float2> screen_space_positions,
                                  const Span<rcti> screen_space_bbox,
                                  const bool check_cyclic,
                                  CutterSegments *cutter_segments)
{
  const OffsetIndices<int> points_by_curve = src.points_by_curve();
  const VArray<bool> is_cyclic = src.cyclic();
  const int curve = segment->curve;
  const int point_first = points_by_curve[curve].first();
  const int point_last = points_by_curve[curve].last();
  const int8_t directions[2] = {-1, 1};

  /* Walk along curve in both directions. */
  for (const int8_t direction : directions) {
    const int8_t point_range_index = (direction == 1) ? 1 : 0;
    int point_a = segment->point_range[point_range_index];

    bool intersected = false;
    segment->is_intersected[point_range_index] = false;

    /* Walk along curve points. */
    while ((direction == 1 && point_a < point_last) || (direction == -1 && point_a > point_first))
    {

      const int point_b = point_a + direction;
      const bool at_end_of_curve = (direction == -1 && point_b == point_first) ||
                                   (direction == 1 && point_b == point_last);

      /* Expand segment point range. */
      segment->point_range[point_range_index] = point_a;

      /* Check for intersections with other curves. For consistency in the intersection factor,
       * we always inspect the line segment a-b in ascending point order. */
      float distance_min = FLT_MAX, distance_max = -FLT_MAX;
      intersected = get_intersections_of_segment_with_curves((direction == 1 ? point_a : point_b),
                                                             (direction == 1 ? point_b : point_a),
                                                             segment->curve,
                                                             src,
                                                             screen_space_positions,
                                                             screen_space_bbox,
                                                             &distance_min,
                                                             &distance_max);

      /* Avoid orphant points at the end of a curve. */
      if (at_end_of_curve &&
          ((direction == -1 && distance_max < DISTANCE_FACTOR_THRESHOLD) ||
           (direction == 1 && distance_min > (1.0f - DISTANCE_FACTOR_THRESHOLD))))
      {
        intersected = false;
        break;
      }

      /* When we hit an intersection, store the intersection factor. Potentially, line segment
       * a-b can be intersected by multiple curves, so we want to fetch the first intersection
       * point we bumped into. In forward direction this is the minimum distance factor, in
       * backward direction the maximum. */
      if (intersected) {
        segment->is_intersected[point_range_index] = true;
        segment->intersection_factor[point_range_index] = (direction == 1) ? distance_min :
                                                                             distance_max;
        break;
      }

      /* Keep walking along curve. */
      point_a += direction;
    }

    /* Adjust point range at curve ends. */
    if (!intersected) {
      if (direction == -1) {
        segment->point_range[0] = point_first;
      }
      else {
        segment->point_range[1] = point_last;
      }
    }
  }

  /* When a curve end is reached and the curve is cyclic, we add an extra cutter segment for the
   * cyclic second part. */
  if (check_cyclic && is_cyclic[curve] &&
      (!segment->is_intersected[0] || !segment->is_intersected[1]) &&
      !(!segment->is_intersected[0] && !segment->is_intersected[1]))
  {
    /* Create extra cutter segment. */
    CutterSegment *new_segment;
    if (!segment->is_intersected[0]) {
      new_segment = cutter_segments->create_segment(curve, point_last);
    }
    else {
      new_segment = cutter_segments->create_segment(curve, point_first);
    }

    /* And expand this extra segment. */
    expand_cutter_segment(
        new_segment, src, screen_space_positions, screen_space_bbox, false, cutter_segments);
  }
}

/**
 * Find curve points within the lasso area, expand them to segments between other curves and
 * delete them from the geometry.
 */
static std::optional<bke::CurvesGeometry> stroke_cutter_find_and_remove_segments(
    const bke::CurvesGeometry &src,
    const Span<int2> mcoords,
    const Span<float2> screen_space_positions,
    const Span<rcti> screen_space_bbox,
    const bool keep_caps)
{
  const int src_curves_num = src.curves_num();
  const int src_points_num = src.points_num();
  const OffsetIndices<int> src_points_by_curve = src.points_by_curve();

  CutterSegments cutter_segments;
  cutter_segments.curve_in_lasso_area = Array<bool>(src_curves_num, false);

  rcti bbox_lasso;
  BLI_lasso_boundbox(&bbox_lasso, mcoords);

  /* Look for curves that intersect the lasso area. */
  for (const int src_curve : src.curves_range()) {
    /* To speed things up: do a bounding box check on the curve and the lasso area. */
    if (!BLI_rcti_isect(&bbox_lasso, &screen_space_bbox[src_curve], nullptr)) {
      continue;
    }
    cutter_segments.curve_in_lasso_area[src_curve] = true;

    /* Look for curve points inside the lasso area. */
    const IndexRange src_points = src_points_by_curve[src_curve];
    for (const int src_point : src_points) {
      /* Skip point when it is already part of a cutter segment. */
      if (cutter_segments.point_is_in_segment(src_curve, src_point)) {
        continue;
      }

      /* Check if point is inside the lasso area. */
      if (BLI_rcti_isect_pt_v(&bbox_lasso, int2(screen_space_positions[src_point])) &&
          BLI_lasso_is_point_inside(mcoords,
                                    int(screen_space_positions[src_point].x),
                                    int(screen_space_positions[src_point].y),
                                    IS_CLIPPED))
      {
        /* Create new cutter segment. */
        CutterSegment *segment = cutter_segments.create_segment(src_curve, src_point);

        /* Expand the cutter segment in both directions until an intersection is found or the end
         * of the curve is reached. */
        expand_cutter_segment(
            segment, src, screen_space_positions, screen_space_bbox, true, &cutter_segments);
      }
    }
  }

  /* Abort when no cutter segments are found in the lasso area. */
  if (cutter_segments.segments.is_empty()) {
    return std::nullopt;
  }

  /* Merge adjacent cutter segments. E.g. two point ranges of 0-10 and 11-20 will be merged
   * to one range of 0-20. */
  cutter_segments.merge_adjacent_segments();

  /* Create the point transfer data, for converting the source geometry into the new geometry.
   * First, add all curve points not affected by the cutter tool. */
  Array<Vector<PointTransferData>> src_to_dst_points(src_points_num);
  for (const int src_curve : src.curves_range()) {
    const IndexRange src_points = src_points_by_curve[src_curve];
    for (const int src_point : src_points) {
      Vector<PointTransferData> &dst_points = src_to_dst_points[src_point];
      const int src_next_point = (src_point == src_points.last()) ? src_points.first() :
                                                                    (src_point + 1);

      /* Add the source point only if it does not lie inside a cutter segment. */
      if (!cutter_segments.point_is_in_segment(src_curve, src_point)) {
        dst_points.append({src_point, src_next_point, 0.0f, true, false});
      }
    }
  }

  /* Add new curve points at the intersection points of the cutter segments.
   *
   *                               a                 b
   *  source curve    o--------o---*---o--------o----*---o--------o
   *                               ^                 ^
   *  cutter segment               |-----------------|
   *
   *  o = existing curve point
   *  * = newly created curve point
   *
   *  The curve points between *a and *b will be deleted.
   *  The source curve will be cut in two:
   *  - the first curve ends at *a
   *  - the second curve starts at *b
   *
   * We avoid inserting a new point very close to the adjacent one, because that's just adding
   * clutter to the geometry.
   */
  for (const auto &cutter_segment : cutter_segments.segments) {
    /* Intersection at cutter segment start. */
    if (cutter_segment.is_intersected[0] &&
        cutter_segment.intersection_factor[0] > DISTANCE_FACTOR_THRESHOLD)
    {
      const int src_point = cutter_segment.point_range[0] - 1;
      Vector<PointTransferData> &dst_points = src_to_dst_points[src_point];
      dst_points.append(
          {src_point, src_point + 1, cutter_segment.intersection_factor[0], false, false});
    }
    /* Intersection at cutter segment end. */
    if (cutter_segment.is_intersected[1]) {
      const int src_point = cutter_segment.point_range[1];
      if (cutter_segment.intersection_factor[1] < (1.0f - DISTANCE_FACTOR_THRESHOLD)) {
        Vector<PointTransferData> &dst_points = src_to_dst_points[src_point];
        dst_points.append(
            {src_point, src_point + 1, cutter_segment.intersection_factor[1], false, true});
      }
      else {
        /* Mark the 'is_cut' flag on the next point, because a new curve is starting here after
         * the removed cutter segment. */
        Vector<PointTransferData> &dst_points = src_to_dst_points[src_point + 1];
        for (auto &dst_point : dst_points) {
          if (dst_point.is_src_point) {
            dst_point.is_cut = true;
          }
        }
      }
    }
  }

  /* Create the new curves geometry. */
  bke::CurvesGeometry dst;
  compute_topology_change(src, dst, src_to_dst_points, keep_caps);

  return dst;
}

/**
 * Apply the stroke cutter to a drawing.
 */
static bool execute_cutter_on_drawing(const int layer_index,
                                      const int frame_number,
                                      const Object &ob_eval,
                                      const Object &obact,
                                      const ARegion &region,
                                      const float4x4 &projection,
                                      const Span<int2> mcoords,
                                      const bool keep_caps,
                                      bke::greasepencil::Drawing &drawing)
{
  const bke::CurvesGeometry &src = drawing.strokes();

  /* Get evaluated geometry. */
  bke::crazyspace::GeometryDeformation deformation =
      bke::crazyspace::get_evaluated_grease_pencil_drawing_deformation(
          &ob_eval, obact, layer_index, frame_number);

  /* Compute screen space positions. */
  Array<float2> screen_space_positions(src.points_num());
  threading::parallel_for(src.points_range(), 4096, [&](const IndexRange src_points) {
    for (const int src_point : src_points) {
      screen_space_positions[src_point] = ED_view3d_project_float_v2_m4(
          &region, deformation.positions[src_point], projection);
    }
  });

  /* Compute bounding boxes of curves in screen space. The bounding boxes are used to speed
   * up the search for intersecting curves. */
  Array<rcti> screen_space_bbox(src.curves_num());
  const OffsetIndices<int> src_points_by_curve = src.points_by_curve();
  threading::parallel_for(src.curves_range(), 512, [&](const IndexRange src_curves) {
    for (const int src_curve : src_curves) {
      rcti *bbox = &screen_space_bbox[src_curve];
      BLI_rcti_init_minmax(bbox);

      const IndexRange src_points = src_points_by_curve[src_curve];
      for (const int src_point : src_points) {
        BLI_rcti_do_minmax_v(bbox, int2(screen_space_positions[src_point]));
      }

      /* Add some padding, otherwise we could just miss intersections. */
      BLI_rcti_pad(bbox, RCTI_PADDING, RCTI_PADDING);
    }
  });

  /* Apply cutter. */
  std::optional<bke::CurvesGeometry> cutted_strokes = stroke_cutter_find_and_remove_segments(
      src, mcoords, screen_space_positions, screen_space_bbox, keep_caps);

  if (cutted_strokes.has_value()) {
    /* Set the new geometry. */
    drawing.geometry.wrap() = std::move(cutted_strokes.value());
    drawing.tag_topology_changed();
  }

  return cutted_strokes.has_value();
}

/**
 * Apply the stroke cutter to all layers.
 */
static int stroke_cutter_execute(wmOperator *op, const bContext *C, const Span<int2> mcoords)
{
  const Scene *scene = CTX_data_scene(C);
  const ARegion *region = CTX_wm_region(C);
  const RegionView3D *rv3d = CTX_wm_region_view3d(C);
  const Depsgraph *depsgraph = CTX_data_depsgraph_pointer(C);
  Object *obact = CTX_data_active_object(C);
  Object *ob_eval = DEG_get_evaluated_object(depsgraph, obact);

  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(obact->data);

  const bool keep_caps = !RNA_boolean_get(op->ptr, "flat_caps");
  const bool active_layer_only = RNA_boolean_get(op->ptr, "active_layer");
  std::atomic<bool> changed = false;

  if (active_layer_only) {
    /* Apply cutter on drawings of active layer. */
    if (!grease_pencil.has_active_layer()) {
      return OPERATOR_CANCELLED;
    }
    const bke::greasepencil::Layer &layer = *grease_pencil.get_active_layer();
    const float4x4 layer_to_world = layer.to_world_space(*ob_eval);
    const float4x4 projection = ED_view3d_ob_project_mat_get_from_obmat(rv3d, layer_to_world);
    const Vector<ed::greasepencil::MutableDrawingInfo> drawings =
        ed::greasepencil::retrieve_editable_drawings_from_layer(*scene, grease_pencil, layer);
    threading::parallel_for_each(drawings, [&](const ed::greasepencil::MutableDrawingInfo &info) {
      if (execute_cutter_on_drawing(info.layer_index,
                                    info.frame_number,
                                    *ob_eval,
                                    *obact,
                                    *region,
                                    projection,
                                    mcoords,
                                    keep_caps,
                                    info.drawing))
      {
        changed = true;
      }
    });
  }
  else {
    /* Apply cutter on every editable drawing. */
    const Vector<ed::greasepencil::MutableDrawingInfo> drawings =
        ed::greasepencil::retrieve_editable_drawings(*scene, grease_pencil);
    threading::parallel_for_each(drawings, [&](const ed::greasepencil::MutableDrawingInfo &info) {
      const bke::greasepencil::Layer &layer = *grease_pencil.layers()[info.layer_index];
      const float4x4 layer_to_world = layer.to_world_space(*ob_eval);
      const float4x4 projection = ED_view3d_ob_project_mat_get_from_obmat(rv3d, layer_to_world);
      if (execute_cutter_on_drawing(info.layer_index,
                                    info.frame_number,
                                    *ob_eval,
                                    *obact,
                                    *region,
                                    projection,
                                    mcoords,
                                    keep_caps,
                                    info.drawing))
      {
        changed = true;
      }
    });
  }

  if (changed) {
    DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY);
    WM_event_add_notifier(C, NC_GEOM | ND_DATA, &grease_pencil);
  }

  return (changed ? OPERATOR_FINISHED : OPERATOR_CANCELLED);
}

static int grease_pencil_stroke_cutter(bContext *C, wmOperator *op)
{
  const Array<int2> mcoords = WM_gesture_lasso_path_to_array(C, op);

  if (mcoords.is_empty()) {
    return OPERATOR_PASS_THROUGH;
  }

  const int result = stroke_cutter_execute(op, C, mcoords);

  return result;
}

}  // namespace blender::ed::greasepencil

void GREASE_PENCIL_OT_stroke_cutter(wmOperatorType *ot)
{
  using namespace blender::ed::greasepencil;

  PropertyRNA *prop;

  ot->name = "Grease Pencil Cutter";
  ot->idname = "GREASE_PENCIL_OT_stroke_cutter";
  ot->description = "Delete stroke points in between intersecting strokes";

  ot->invoke = WM_gesture_lasso_invoke;
  ot->modal = WM_gesture_lasso_modal;
  ot->exec = grease_pencil_stroke_cutter;
  ot->poll = grease_pencil_painting_poll;
  ot->cancel = WM_gesture_lasso_cancel;

  ot->flag = OPTYPE_UNDO | OPTYPE_REGISTER;

  WM_operator_properties_gesture_lasso(ot);

  prop = RNA_def_boolean(ot->srna, "flat_caps", false, "Flat Caps", "");
  RNA_def_property_flag(prop, PROP_HIDDEN);
  prop = RNA_def_boolean(
      ot->srna, "active_layer", false, "Active Layer", "Only edit the active layer of the object");
  RNA_def_property_flag(prop, PROP_HIDDEN);
}
