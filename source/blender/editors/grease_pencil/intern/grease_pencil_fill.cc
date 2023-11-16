/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgreasepencil
 */

#include "grease_pencil_fill.hh"

namespace blender::ed::greasepencil::fill {

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
 * Get all intersections of a segment with other curves.
 */
Vector<IntersectingCurve> get_intersections_of_segment_with_curves(
    const float2 &segment_a,
    const float2 &segment_b,
    const int segment_curve_index,
    const Curves2DSpace *curves_2d,
    const float2 &adj_a,
    const float2 &adj_b,
    void (*store_overlapping_segment)(int, int, bool, int, int, FillData *),
    const int segment_point_a,
    const int segment_point_b,
    const int segment_direction,
    const bool use_epsilon,
    FillData *fd)
{
  /* Inits. */
  Vector<IntersectingCurve> intersections;
  std::mutex mutex;

  const float2 segment_vec = segment_b - segment_a;

  float2 segment_epsilon = {0.0f, 0.0f};
  if (use_epsilon) {
    if (segment_direction == 1) {
      const int epsilon_i = curves_2d->point_offset[segment_curve_index] + segment_point_a;
      segment_epsilon = -fd->curve_segment_epsilon[epsilon_i];
    }
    else {
      const int epsilon_i = curves_2d->point_offset[segment_curve_index] + segment_point_b;
      segment_epsilon = fd->curve_segment_epsilon[epsilon_i];
    }
  }

  /* Loop all curves, looking for intersecting segments. */
  threading::parallel_for(curves_2d->point_offset.index_range(), 256, [&](const IndexRange range) {
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
            if (store_overlapping_segment != nullptr) {
              store_overlapping_segment(
                  curve_i, point_i - point_offset, 1, segment_point_a, segment_direction, fd);
            }
            skip_curve = true;
            break;
          }
          if (segment_a == p1 && segment_b == p0) {
            if (store_overlapping_segment) {
              store_overlapping_segment(
                  curve_i, point_i - point_offset, -1, segment_point_a, segment_direction, fd);
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
          p0_extended -= fd->curve_segment_epsilon[point_i];
          p1_extended += fd->curve_segment_epsilon[point_i];
        }

        /* Skip when previous segment was intersecting. */
        if (prev_intersection_point == point_i) {
          continue;
        }

        /* Skip when bounding boxes don't overlap. */
        BLI_rctf_init(&bbox_isect, p0_extended[0], p0_extended[0], p0_extended[1], p0_extended[1]);
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
        auto isect = math::isect_seg_seg(seg_a_extended, seg_b_extended, p0_extended, p1_extended);
        if (ELEM(isect.kind, isect.LINE_LINE_CROSS, isect.LINE_LINE_EXACT)) {
          IntersectingCurve intersection{};
          intersection.curve_index_2d = curve_i;
          intersection.point_start = point_i - point_offset;
          intersection.point_end = point_i_next - point_offset;
          intersection.distance = get_intersection_distance_normalized(
              segment_a, segment_b, p0, p1);
          intersection.has_stroke = curves_2d->has_stroke[curve_i];

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

/**
 * Create a KD tree for the gap closure of curve ends by radius.
 */
static void init_curve_end_radii(FillData *fd)
{
  /* Create KD tree for all curve ends. */
  const int curve_num = fd->curves_2d.point_offset.size();
  fd->curve_ends = BLI_kdtree_2d_new(curve_num * 2);

  /* Add curve ends. */
  int tree_index = 0;
  for (const int curve_i : fd->curves_2d.point_offset.index_range()) {
    /* Add first curve point. */
    int point_i = fd->curves_2d.point_offset[curve_i];
    BLI_kdtree_2d_insert(fd->curve_ends, tree_index, fd->curves_2d.points_2d[point_i]);
    tree_index++;

    /* Add last curve point. */
    point_i += fd->curves_2d.point_size[curve_i] - 1;
    BLI_kdtree_2d_insert(fd->curve_ends, tree_index, fd->curves_2d.points_2d[point_i]);
    tree_index++;
  }

  BLI_kdtree_2d_balance(fd->curve_ends);

  /* Init array with curve end connection flag. */
  fd->connected_by_radius = Array<bool>(curve_num * 2, false);
}

/**
 * Determine which curves ends are connected with each other based on radius.
 */
static void get_connected_curve_end_radii(FillData *fd)
{
  const float max_dist = 2 * fd->gap_distance;

  /* Init connections by radius. */
  fd->connected_by_radius.fill(false);
  fd->radius_connections.clear();

  /* Check all curvess. */
  for (const int curve_i : fd->curves_2d.point_offset.index_range()) {
    /* Skip cyclic curves. */
    if (fd->curves_2d.is_cyclic[curve_i]) {
      continue;
    }

    /* Check both ends. */
    for (int side = 0; side <= 1; side++) {
      const int kdtree_index = curve_i * 2 + side;
      const int point_index = fd->curves_2d.point_offset[curve_i] +
                              (side == 0 ? 0 : fd->curves_2d.point_size[curve_i] - 1);

      /* Find nearest curve end points. */
      KDTreeNearest_2d nearest[RADIUS_NEAREST_NUM];
      const int nearest_num = BLI_kdtree_2d_find_nearest_n(
          fd->curve_ends, fd->curves_2d.points_2d[point_index], nearest, RADIUS_NEAREST_NUM);

      for (int i = 0; i < nearest_num; i++) {
        /* Skip self, already registered curve ends and curve ends out of close radius range. */
        if (nearest[i].index <= kdtree_index || nearest[i].dist > max_dist) {
          continue;
        }

        /* Skip cyclic curves. */
        const int nearest_curve = int(nearest[i].index / 2);
        if (fd->curves_2d.is_cyclic[nearest_curve]) {
          continue;
        }

        /* Flag connection and append to list for drawing. */
        fd->connected_by_radius[kdtree_index] = true;
        fd->connected_by_radius[nearest[i].index] = true;
        fd->radius_connections.append({kdtree_index, nearest[i].index});
      }
    }
  }
}

/**
 * Init the data structure for curve end extensions.
 */
static void init_curve_end_extensions(FillData *fd)
{
  const int curve_num = fd->curves_2d.point_offset.size() * 2;
  const int point_num = curve_num * 2;

  fd->extensions_2d.point_offset = Array<int>(curve_num);
  fd->extensions_2d.point_size = Array<int>(curve_num, 2);
  fd->extensions_2d.is_cyclic = Array<bool>(curve_num, false);
  fd->extensions_2d.has_stroke = Array<bool>(curve_num, false);
  fd->extensions_2d.drawing_index_2d = Array<int>(curve_num);
  fd->extensions_2d.points_2d = Array<float2>(point_num);
  fd->extensions_2d.curve_bbox = Array<rctf>(curve_num);
  fd->extension_length_ratio = Array<float>(curve_num);
  fd->extension_has_intersection = Array<bool>(curve_num, false);
}

/**
 * Add a curve end extension to the set of extensions.
 */
static void add_curve_end_extension(FillData *fd,
                                    const int curve_i,
                                    const int curve_ext,
                                    const int point_i,
                                    const int next_point_delta,
                                    const int point_i_ext)
{
  /* Set drawing index and point index for extension. */
  fd->extensions_2d.drawing_index_2d[curve_ext] = fd->curves_2d.drawing_index_2d[curve_i];
  fd->extensions_2d.point_offset[curve_ext] = point_i_ext;
  fd->extensions_2d.is_cyclic[curve_ext] = fd->curves_2d.is_cyclic[curve_i];

  /* Use the vector between the two outer points of the curve to calculate the extension
   * coordinates. */
  const float2 co = fd->curves_2d.points_2d[point_i];
  fd->extensions_2d.points_2d[point_i_ext] = co;
  float2 end_vec = co - fd->curves_2d.points_2d[point_i + next_point_delta];
  const float end_length = math::length(end_vec);
  end_vec = math::normalize(end_vec) * fd->gap_distance;
  const float extension_length = math::length(end_vec);
  fd->extensions_2d.points_2d[point_i_ext + 1] = co + end_vec;
  fd->extension_length_ratio[curve_ext] = (end_length == 0.0f ? 0.0f :
                                                                extension_length / end_length);
}

/**
 * Create a set of 2D curve end extensions. Check if the extensions intersect with curves or
 * other end extensions.
 */
static void get_curve_end_extensions(FillData *fd)
{
  /* Create extension curves. */
  for (const int curve_i : fd->curves_2d.point_offset.index_range()) {
    /* Two extensions for each curve. */
    const int curve_ext = curve_i * 2;
    /* Each extension contains two points. */
    const int point_i_ext = curve_ext * 2;
    const int point_size = fd->curves_2d.point_size[curve_i];

    /* Create extension for the first segment of the 2D curve. */
    int point_i = fd->curves_2d.point_offset[curve_i];
    int next_point_delta = (point_size > 1 ? 1 : 0);
    add_curve_end_extension(fd, curve_i, curve_ext, point_i, next_point_delta, point_i_ext);

    /* Create extension for the last segment of the 2D curve. */
    point_i += point_size - 1;
    next_point_delta = (point_size > 1 ? -1 : 0);
    add_curve_end_extension(
        fd, curve_i, curve_ext + 1, point_i, next_point_delta, point_i_ext + 2);

    /* Create bounding boxes for the extensions. */
    BLI_rctf_init_minmax(&fd->extensions_2d.curve_bbox[curve_ext]);
    BLI_rctf_do_minmax_v(&fd->extensions_2d.curve_bbox[curve_ext],
                         fd->extensions_2d.points_2d[point_i_ext]);
    BLI_rctf_do_minmax_v(&fd->extensions_2d.curve_bbox[curve_ext],
                         fd->extensions_2d.points_2d[point_i_ext + 1]);

    BLI_rctf_init_minmax(&fd->extensions_2d.curve_bbox[curve_ext + 1]);
    BLI_rctf_do_minmax_v(&fd->extensions_2d.curve_bbox[curve_ext + 1],
                         fd->extensions_2d.points_2d[point_i_ext + 2]);
    BLI_rctf_do_minmax_v(&fd->extensions_2d.curve_bbox[curve_ext + 1],
                         fd->extensions_2d.points_2d[point_i_ext + 3]);
  }

  /* Clear intersection data. */
  fd->extension_intersections.clear();
  fd->extension_has_intersection.fill(false);

  /* Check intersections of extensions with curves or other end extensions. */
  for (const int extension_index : fd->extensions_2d.point_offset.index_range()) {
    /* Skip end extensions of cyclic curves. */
    if (fd->extensions_2d.is_cyclic[extension_index]) {
      continue;
    }

    const int point_offset = fd->extensions_2d.point_offset[extension_index];
    const float2 segment_point_a = fd->extensions_2d.points_2d[point_offset];
    const float2 segment_point_b = fd->extensions_2d.points_2d[point_offset + 1];

    /* Get intersections with other curve end extensions. */
    Vector<IntersectingCurve> all_intersections;
    Vector<IntersectingCurve> intersections = get_intersections_of_segment_with_curves(
        segment_point_a, segment_point_b, extension_index, &fd->extensions_2d);

    for (auto intersection : intersections) {
      intersection.with_end_extension = true;
      intersection.extension_index = extension_index;
      all_intersections.append(intersection);
    }

    /* Get intersections with other curves. */
    const int curve_i = int(extension_index / 2);
    intersections = get_intersections_of_segment_with_curves(
        segment_point_a, segment_point_b, curve_i, &fd->curves_2d);

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
        if (fd->extensions_stop_at_first_intersection &&
            (intersection.distance[0] - shortest_dist) > DISTANCE_SORT_EPSILON)
        {
          break;
        }
        fd->extension_intersections.append(intersection);
        fd->extension_has_intersection[intersection.extension_index] = true;
      }
    }
  }
}

/**
 * Get a slight extension for each curve segment, based on the normalized vector of each
 * curve point to the next.
 */
static void get_curve_segment_epsilons(FillData *fd)
{
  fd->curve_segment_epsilon = Array<float2>(fd->curves_2d.points_2d.size());

  threading::parallel_for(
      fd->curves_2d.point_offset.index_range(), 256, [&](const IndexRange range) {
        for (const int curve_i : range) {
          const int point_last = fd->curves_2d.point_offset[curve_i] +
                                 fd->curves_2d.point_size[curve_i] - 1;
          for (const int point_i :
               IndexRange(fd->curves_2d.point_offset[curve_i], fd->curves_2d.point_size[curve_i]))
          {
            const int point_next = (point_i == point_last ? fd->curves_2d.point_offset[curve_i] :
                                                            point_i + 1);
            fd->curve_segment_epsilon[point_i] = math::normalize(
                                                     fd->curves_2d.points_2d[point_next] -
                                                     fd->curves_2d.points_2d[point_i]) *
                                                 DISTANCE_EPSILON;
          }
        }
      });
}

#ifdef GP_FILL_DEBUG_MODE
/**
 * Debug function: show curve indices in the viewport.
 */
static void debug_draw_curve_indices(FillData *fd)
{
  const int font_id = BLF_default();
  const uiStyle *style = UI_style_get();
  BLF_size(font_id, style->widget.points * UI_SCALE_FAC * 0.8);

  /* Draw point indices. */
  BLF_color4fv(font_id, float4{0.0f, 0.0f, 0.0f, 0.5f});
  for (const int curve_i : fd->curves_2d.point_offset.index_range()) {
    const int point_offset = fd->curves_2d.point_offset[curve_i];
    const int step = 5;
    float x_prev = FLT_MAX;
    float y_prev = FLT_MAX;
    for (int point_i = step; point_i < fd->curves_2d.point_size[curve_i]; point_i += step) {
      const std::string str = std::to_string(point_i) + "-" + std::to_string(curve_i);
      const char *text = str.c_str();

      const float x = fd->curves_2d.points_2d[point_offset + point_i][0] - strlen(text) * 5.4;
      const float y = fd->curves_2d.points_2d[point_offset + point_i][1] + 1;

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

  for (const int curve_i : fd->curves_2d.point_offset.index_range()) {
    const std::string str = std::to_string(curve_i);
    const char *text = str.c_str();

    const int point_offset = fd->curves_2d.point_offset[curve_i];
    const float x = fd->curves_2d.points_2d[point_offset][0] - strlen(text) * 10;
    const float y = fd->curves_2d.points_2d[point_offset][1] + 2;
    BLF_position(font_id, x, y, 0);

    BLF_draw(font_id, text, strlen(text));
  }

  BLF_disable(font_id, BLF_BOLD);
}
#endif

/**
 * Get curve point index given an end extension index.
 */
static int get_curve_point_by_end_index(FillData *fd, const int end_index)
{
  const int curve_i = int(end_index / 2);
  int point_i = fd->curves_2d.point_offset[curve_i];
  if ((end_index & 1) == 1) {
    point_i += fd->curves_2d.point_size[curve_i] - 1;
  }
  return point_i;
}

/**
 * Draw gap closure lines on an overlay in the 3D viewport.
 */
static void draw_overlay(const bContext * /*C*/, ARegion *region, void *arg)
{
  FillData *fd = static_cast<FillData *>(arg);

  /* Draw only in the region that originated the operator. */
  if (region != fd->vc.region) {
    return;
  }

  /* Anything to draw? */
  if (!(fd->use_gap_close_extend || fd->use_gap_close_radius || fd->use_gap_close_proximity)) {
    return;
  }

  const uint shdr_pos = GPU_vertformat_attr_add(
      immVertexFormat(), "pos", GPU_COMP_F32, 2, GPU_FETCH_FLOAT);
  immBindBuiltinProgram(GPU_SHADER_3D_UNIFORM_COLOR);

  GPU_line_width(1.5f);
  GPU_blend(GPU_BLEND_ALPHA);
  GPU_line_smooth(true);

  /* Draw indications of curve proximities. */
  if (fd->use_gap_close_proximity) {
    /* Draw filled circle at start, middle and end point of each curve. */
    int point_indices[3];
    immUniformColor4fv(fd->gap_proximity_color);
    for (const int curve_i : fd->curves_2d.point_offset.index_range()) {
      if (fd->curves_2d.point_size[curve_i] < 2) {
        continue;
      }

      point_indices[0] = 0;
      point_indices[1] = fd->curves_2d.point_size[curve_i] - 1;
      point_indices[2] = int(point_indices[1] / 2);
      const int point_count = (point_indices[1] >= 2) ? 3 : 2;
      const int point_offset = fd->curves_2d.point_offset[curve_i];

      for (int point = 0; point < point_count; point++) {
        const int point_i = point_offset + point_indices[point];
        imm_draw_circle_fill_2d(shdr_pos,
                                fd->curves_2d.points_2d[point_i][0],
                                fd->curves_2d.points_2d[point_i][1],
                                fd->proximity_distance,
                                40);
      }
    }
  }

  /* Draw curve end extensions. */
  if (fd->use_gap_close_extend) {
    /* Draw extensions of full length. */
    for (const int ext_i : fd->extensions_2d.point_offset.index_range()) {
      /* Skip intersected extensions or extensions of cyclic curves. */
      if ((fd->extensions_stop_at_first_intersection && fd->extension_has_intersection[ext_i]) ||
          fd->extensions_2d.is_cyclic[ext_i])
      {
        continue;
      }

      if (fd->extension_has_intersection[ext_i]) {
        immUniformColor3fv(fd->gap_closed_color);
      }
      else {
        immUniformColor3fv(fd->gap_closure_color);
      }

      const int point_i = fd->extensions_2d.point_offset[ext_i];
      immBegin(GPU_PRIM_LINES, 2);
      immVertex2fv(shdr_pos, fd->extensions_2d.points_2d[point_i]);
      immVertex2fv(shdr_pos, fd->extensions_2d.points_2d[point_i + 1]);
      immEnd();
    }

    /* Draw shortened extensions (that intersect a curve or other extension). */
    if (fd->extensions_stop_at_first_intersection) {
      immUniformColor3fv(fd->gap_closed_color);
      int ext_prev = -1;
      for (const auto &intersection : fd->extension_intersections) {
        const int ext_i = intersection.extension_index;
        const int point_i = fd->extensions_2d.point_offset[ext_i];
        if (ext_i == ext_prev) {
          continue;
        }
        ext_prev = ext_i;

        /* Limit the end extension when it is intersecting something else. */
        const float distance = intersection.distance[0];
        const float2 p0 = fd->extensions_2d.points_2d[point_i];
        const float2 p1 = fd->extensions_2d.points_2d[point_i + 1];
        const float2 p_vec = p1 - p0;

        immBegin(GPU_PRIM_LINES, 2);
        immVertex2fv(shdr_pos, p0);
        immVertex2fv(shdr_pos, p0 + p_vec * distance);
        immEnd();
      }
    }
  }

  /* Draw curve end radii. */
  if (fd->use_gap_close_radius) {

    /* Draw connected curve ends. */
    immUniformColor3fv(fd->gap_closed_color);
    for (const int2 &connection : fd->radius_connections) {
      const int point_i0 = get_curve_point_by_end_index(fd, connection[0]);
      const int point_i1 = get_curve_point_by_end_index(fd, connection[1]);
      immBegin(GPU_PRIM_LINES, 2);
      immVertex2fv(shdr_pos, fd->curves_2d.points_2d[point_i0]);
      immVertex2fv(shdr_pos, fd->curves_2d.points_2d[point_i1]);
      immEnd();
    }

    /* Draw unconnected curve end radii. */
    immUniformColor3fv(fd->gap_closure_color);
    GPU_line_width(2.0f);

    int curve_end_index = -1;
    for (const bool connected : fd->connected_by_radius) {
      curve_end_index++;
      if (connected) {
        continue;
      }

      /* Skip ends of cyclic curves. */
      const int curve_i = int(curve_end_index / 2);
      if (fd->curves_2d.is_cyclic[curve_i]) {
        continue;
      }

      /* Draw radius. */
      const int point_i = get_curve_point_by_end_index(fd, curve_end_index);
      imm_draw_circle_wire_2d(shdr_pos,
                              fd->curves_2d.points_2d[point_i][0],
                              fd->curves_2d.points_2d[point_i][1],
                              fd->gap_distance,
                              40);
    }
  }

  immUnbindProgram();

  GPU_line_width(1.0f);
  GPU_line_smooth(false);
  GPU_blend(GPU_BLEND_NONE);

#ifdef GP_FILL_DEBUG_MODE
  debug_draw_curve_indices(fd);
#endif
}

/**
 * Get latest tool settings before executing the actual fill.
 */
static void get_latest_toolsettings(FillData *fd)
{
  const bool use_gap_closing = (fd->brush->gpencil_settings->fill_extend_fac > FLT_EPSILON);
  fd->use_gap_close_extend = (fd->brush->gpencil_settings->fill_extend_mode ==
                              GP_FILL_EMODE_EXTEND) &&
                             use_gap_closing;
  fd->use_gap_close_radius = (fd->brush->gpencil_settings->fill_extend_mode ==
                              GP_FILL_EMODE_RADIUS) &&
                             use_gap_closing;
  fd->gap_distance = fd->brush->gpencil_settings->fill_extend_fac * GAP_PIXEL_FACTOR;
  fd->use_gap_close_proximity = (fd->brush->gpencil_settings->fill_proximity_distance > 0);
  fd->proximity_distance = float(fd->brush->gpencil_settings->fill_proximity_distance);
}

/**
 * Update the gap closure By Proximity in the viewport when the value changed.
 */
static void update_proximity_distance(FillData *fd, const int delta)
{
  fd->brush->gpencil_settings->fill_proximity_distance = math::max(
      0, math::min(100, fd->brush->gpencil_settings->fill_proximity_distance + delta));
  get_latest_toolsettings(fd);
  ED_region_tag_redraw(fd->vc.region);
}

/**
 * Update the gap closure connections when the radius changed.
 */
static void update_gap_distance(FillData *fd, const float delta)
{
  fd->brush->gpencil_settings->fill_extend_fac = math::max(
      0.0f, math::min(10.0f, fd->brush->gpencil_settings->fill_extend_fac + delta));
  get_latest_toolsettings(fd);

  if (fd->use_gap_close_extend) {
    get_curve_end_extensions(fd);
  }
  if (fd->use_gap_close_radius) {
    get_connected_curve_end_radii(fd);
  }
  ED_region_tag_redraw(fd->vc.region);
}

/**
 * Get a list of layers used for the fill edge detection. The list is based on the 'Layers' field
 * in the tool settings.
 */
static void get_fill_edge_layers(FillData *fd,
                                 Vector<const bke::greasepencil::Drawing *> &r_drawings,
                                 Vector<int> &r_layer_index,
                                 const int frame_number)
{
  using namespace bke::greasepencil;

  /* Find index of active layer. */
  int active_layer_index = -1;
  const Layer *active_layer = fd->grease_pencil->get_active_layer();
  Span<const Layer *> layers = fd->grease_pencil->layers();
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
    switch (fd->brush->gpencil_settings->fill_layer_mode) {
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
      default:
        break;
    }

    if (add) {
      if (const Drawing *drawing = fd->grease_pencil->get_editable_drawing_at(layers[layer_index],
                                                                              frame_number))
      {
        r_drawings.append(drawing);
        r_layer_index.append(layer_index);
      }
    }
  }
}

/**
 * Get drawings and gap closure data for a given frame number.
 */
static bool get_drawings_and_gap_closures_at_frame(FillData *fd, const int frame_number)
{
  /* Get layers according to tool settings (visible, above, below, etc.) */
  Vector<const bke::greasepencil::Drawing *> drawings;
  Vector<int> layer_indices;

  get_fill_edge_layers(fd, drawings, layer_indices, frame_number);
  if (drawings.is_empty()) {
    return false;
  }

  /* Convert curves to viewport 2D space. */
  fd->curves_2d = curves_in_2d_space_get(
      &fd->vc, fd->vc.obact, drawings, layer_indices, frame_number, true);

  /* Calculate epsilon values of all 2D curve segments, used to avoid floating point precision
   * errors. */
  get_curve_segment_epsilons(fd);

  /* When using extensions of the curve ends to close gaps, build an array of those
   * two-point 'curves'. */
  if (fd->use_gap_close_extend) {
    init_curve_end_extensions(fd);
    get_curve_end_extensions(fd);
  }

  /* When using radii to close gaps, build KD tree of curve end points. */
  if (fd->use_gap_close_radius) {
    init_curve_end_radii(fd);
    get_connected_curve_end_radii(fd);
  }

  return true;
}

/**
 * Initialize the fill operator data.
 */
static bool operator_init(bContext *C, wmOperator *op)
{
  FillData *fd = static_cast<FillData *>(op->customdata);

  /* Get view context. */
  Depsgraph *depsgraph = CTX_data_ensure_evaluated_depsgraph(C);
  fd->vc = ED_view3d_viewcontext_init(C, depsgraph);

  /* Get active GP object. */
  fd->grease_pencil = static_cast<GreasePencil *>(fd->vc.obact->data);

  /* Get tool brush. */
  ToolSettings *ts = CTX_data_tool_settings(C);
  fd->brush = BKE_paint_brush(&ts->gp_paint->paint);
  fd->additive_drawing = ((ts->gpencil_flags & GP_TOOL_FLAG_RETAIN_LAST) != 0);

  /* Init vector fill flags. */
  fd->use_gap_close_extend = (fd->brush->gpencil_settings->fill_extend_mode ==
                              GP_FILL_EMODE_EXTEND);
  fd->use_gap_close_radius = (fd->brush->gpencil_settings->fill_extend_mode ==
                              GP_FILL_EMODE_RADIUS);
  fd->extensions_stop_at_first_intersection = (fd->brush->gpencil_settings->flag &
                                               GP_BRUSH_FILL_STROKE_COLLIDE) != 0;
  fd->gap_distance = fd->brush->gpencil_settings->fill_extend_fac * GAP_PIXEL_FACTOR;
  fd->wait_for_release = true;
  fd->wait_event_type = LEFTMOUSE;
  fd->use_gap_close_proximity = (fd->brush->gpencil_settings->fill_proximity_distance > 0);
  fd->proximity_distance = float(fd->brush->gpencil_settings->fill_proximity_distance);

  const float default_gap_closure_color[3] = {1.0f, 0.0f, 0.5f};
  const float default_gap_closed_color[3] = {0.0f, 1.0f, 1.0f};
  const float default_gap_proximity_color[4] = {1.0f, 0.8f, 0.0f, 0.3f};
  copy_v3_v3(fd->gap_closure_color, default_gap_closure_color);
  copy_v3_v3(fd->gap_closed_color, default_gap_closed_color);
  copy_v4_v4(fd->gap_proximity_color, default_gap_proximity_color);

  /* Get drawings for current frame. */
  if (!get_drawings_and_gap_closures_at_frame(fd, fd->vc.scene->r.cfra)) {
    return false;
  }

  /* Activate 3D viewport overlay for showing gap closure visual aids. */
  if ((fd->brush->gpencil_settings->flag & GP_BRUSH_FILL_SHOW_EXTENDLINES) != 0) {
    fd->draw_handle = ED_region_draw_cb_activate(
        fd->vc.region->type, draw_overlay, fd, REGION_DRAW_POST_PIXEL);
    ED_region_tag_redraw(fd->vc.region);
  }

  return true;
}

/**
 * Clean up the fill operator data.
 */
static void operator_exit(bContext *C, wmOperator *op)
{
  WM_cursor_modal_restore(CTX_wm_window(C));

  if (op->customdata != nullptr) {
    FillData *fd = static_cast<FillData *>(op->customdata);
    if (fd->draw_handle) {
      ED_region_draw_cb_exit(fd->vc.region->type, fd->draw_handle);
      ED_region_tag_redraw(fd->vc.region);
    }

    /* Vector fill: remove KD tree of curve ends. */
    if (fd->curve_ends) {
      BLI_kdtree_2d_free(fd->curve_ends);
    }

    MEM_delete(fd);
    op->customdata = nullptr;
  }
}

static bool fill_do(FillData *fd)
{
  /* DEBUG: measure time. */
  auto t1 = std::chrono::high_resolution_clock::now();

  /* Get latest toolsetting values. */
  get_latest_toolsettings(fd);

  /* Get the fill method. */
  bool (*perform_fill)(FillData *) = nullptr;
  if (fd->brush->gpencil_settings->fill_mode == GP_FILL_MODE_FLOOD) {
    perform_fill = &flood_fill_do;
  }
  else if (fd->brush->gpencil_settings->fill_mode == GP_FILL_MODE_GEOMETRY) {
    perform_fill = &vector_fill_do;
  }
  if (perform_fill == nullptr) {
    return false;
  }

  /* Perform the fill operation for all selected frames. */
  const bool use_multi_frame_editing = (fd->vc.scene->toolsettings->gpencil_flags &
                                        GP_USE_MULTI_FRAME_EDITING) != 0;
  const Array<int> frame_numbers = get_frame_numbers_for_layer(
      fd->grease_pencil->get_active_layer()->wrap(),
      fd->vc.scene->r.cfra,
      use_multi_frame_editing);
  bool get_drawings = false;
  bool success = false;

  for (const int frame_number : frame_numbers) {
    /* For the current frame (first entry in the array) we already retrieved the drawings during
     * the operator invoke, so no need to do it a second time. */
    if (get_drawings) {
      if (!get_drawings_and_gap_closures_at_frame(fd, frame_number)) {
        continue;
      }
    }
    get_drawings = true;

    fd->frame_number = frame_number;

    if (perform_fill(fd)) {
      success = true;
    }
  }

  /* DEBUG: measure time. */
  auto t2 = std::chrono::high_resolution_clock::now();
  auto delta_t = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
  printf("Fill operator took %d ms.\n", int(delta_t.count()));

  return success;
}

/**
 * Modal handler for the fill operator:
 * - Change gap closure radius
 * - Perform the fill at second mouse click
 */
static int operator_modal(bContext *C, wmOperator *op, const wmEvent *event)
{
  FillData *fd = static_cast<FillData *>(op->customdata);
  int modal_state = OPERATOR_RUNNING_MODAL;

  /* Prevent repeating event keys, when not released yet. */
  if (fd->wait_for_release && fd->wait_event_type == event->type) {
    if (event->val == KM_RELEASE) {
      fd->wait_for_release = false;
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
      update_gap_distance(fd, (event->modifier & KM_SHIFT) ? 0.01f : 0.1f);
      break;
    case EVT_PAGEDOWNKEY:
    case WHEELDOWNMOUSE:
      update_gap_distance(fd, (event->modifier & KM_SHIFT) ? -0.01f : -0.1f);
      break;

    case EVT_LEFTBRACKETKEY:
      update_proximity_distance(fd, -1);
      break;
    case EVT_RIGHTBRACKETKEY:
      update_proximity_distance(fd, 1);
      break;

    case LEFTMOUSE: {
      /* Get mouse position of second click (the 'fill' click). */
      fd->mouse_pos[0] = float(event->mval[0]);
      fd->mouse_pos[1] = float(event->mval[1]);

      /* Perform the fill operation. */
      if (fill_do(fd)) {
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
      operator_exit(C, op);
      WM_event_add_notifier(C, NC_GPENCIL | NA_EDITED, nullptr);
      break;

    case OPERATOR_CANCELLED:
      operator_exit(C, op);
      break;

    default:
      break;
  }

  return modal_state;
}

static void operator_cancel(bContext *C, wmOperator *op)
{
  operator_exit(C, op);
}

/**
 * Invoke the fill operator at first mouse click in the viewport.
 */
static int operator_invoke(bContext *C, wmOperator *op, const wmEvent * /*event*/)
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
  if (!grease_pencil.get_active_layer()->frames().contains(scene->r.cfra)) {
    if (!blender::animrig::is_autokey_on(scene)) {
      BKE_report(op->reports, RPT_ERROR, "No Grease Pencil frame to draw on");
      return OPERATOR_CANCELLED;
    }
  }

  /* Init tool data. */
  FillData *fd = MEM_new<FillData>(__func__);
  op->customdata = fd;
  if (!operator_init(C, op)) {
    operator_exit(C, op);
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
 * Definition of the fill operator.
 */
static void GREASE_PENCIL_OT_fill(wmOperatorType *ot)
{
  ot->name = "Fill";
  ot->idname = __func__;
  ot->description = "Fill a shape formed by strokes";

  ot->poll = active_grease_pencil_poll;
  ot->invoke = operator_invoke;
  ot->modal = operator_modal;
  ot->cancel = operator_cancel;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

/** \} */

}  // namespace blender::ed::greasepencil::fill

void ED_operatortypes_grease_pencil_fill()
{
  using namespace blender::ed::greasepencil::fill;

  WM_operatortype_append(GREASE_PENCIL_OT_fill);
}
