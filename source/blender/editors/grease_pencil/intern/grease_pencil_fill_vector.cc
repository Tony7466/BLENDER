/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgreasepencil
 */

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
#include "ED_screen.hh"
#include "ED_space_api.hh"
#include "ED_view3d.hh"

#include "GPU_framebuffer.h"
#include "GPU_immediate.h"
#include "GPU_matrix.h"
#include "GPU_state.h"

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
 * with an other curve (D) or at the end of the curve itself (G).
 * At an intersection (D), the right turn will be inspected first. Because we are inspecting
 * the edge in clockwise direction, this garantuees us that we find the 'narrowest' edge. See
 * at (F), for example: by taking the right turn first, we find the smallest fill area (just as
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
 * segment points). This can easily occur when the fill tool is used before: the original stroke
 * and the fill curve share geometry points. Therefore, priority is given to curve with only
 * 'stroke' material over curves with 'fill' material.
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
#define GP_VFILL_RADIUS_NEAREST_NUM 12
/* Number of pixels the gap closure size stands for. */
#define GP_VFILL_GAP_PIXEL_FACTOR 20.0f
/* Maximum execution time of vector fill operator (in milliseconds). */
#define GP_VFILL_MAX_EXECUTION_TIME 500

/* Fill segment and segment end flags. */
enum class vfFlag : uint16_t {
  None = 0,
  /* Segment is inspected on intersections and gap closures. */
  IsInspected = 1 << 0,
  /* When backwards, point range of segment is from high to low. */
  DirectionBackwards = 1 << 1,
  /* Segment is the second one of a cyclic segment pair. */
  IsLastOfCycle = 1 << 2,
  /* Segment is intersected by other curve (or curve end extension.) */
  IsIntersected = 1 << 3,
  /* Segment ends with a gap closure to an other curve. */
  IsGapClosure = 1 << 4,
  /* Segment end connects with end extension of other curve. */
  WithEndExtension = 1 << 5,
  /* The end extension of the next curve intersects at the third turn. */
  ExtensionAtThirdTurn = 1 << 6,
  /* For the first segment only: ignore intersections before the start distance. */
  CheckStartDistance = 1 << 7,
  /* Segment curve has fill material. */
  IsFilled = 1 << 8,
  /* Segment is part of the unused head of a closed fill edge. */
  IsUnused = 1 << 9,
};
ENUM_OPERATORS(vfFlag, vfFlag::IsUnused);
inline constexpr bool operator==(vfFlag a, bool b)
{
  return (uint64_t(a) != 0) == b;
}

enum class vfRayDirection : uint8_t {
  Up = 0,
  Right = 1,
  Down = 2,
  Left = 3,
};

constexpr std::initializer_list<vfRayDirection> ray_directions = {
    vfRayDirection::Up, vfRayDirection::Right, vfRayDirection::Down, vfRayDirection::Left};

/* Intersecting segment in 2D space. */
struct IntersectingSegment2D {
  /* Curve index in `Curves2DSpace` struct. */
  int curve_index_2d;
  /* Start point of the intersection (relative index: 0..<curve_point_size>). */
  int point_start;
  /* End point of the intersection (relative index: 0..<curve_point_size>). */
  int point_end;
  /* Distance of the intersection on the intersected segment. */
  float distance;
  /* Curve is filled. */
  bool is_filled;
};

/* Edge segment ending data: segment ends by intersection, gap closure etc. */
struct SegmentEnd {
  /* Curve index (in #Curves2DSpace struct) of the intersecting/gap closing curve. */
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
  /* For modal event handling: wait for release until processing the event key again. */
  bool wait_for_release{};
  /* Wait-for-release event key. */
  short wait_event_type;

  /* Mouse position of the fill operation click. */
  float2 mouse_pos{};

  /* True when edge gaps are closed by extending the curve ends. */
  bool gap_close_extend{};
  /* True when edge gaps are closed by curve end radii. */
  bool gap_close_radius{};
  /* True when curve end extensions can collide with curves. */
  bool extensions_collide_with_curves;
  /* Gap closure distance in pixels. */
  float gap_distance;

  /* Active Grease Pencil object. */
  GreasePencil *grease_pencil;
  /* View context data.*/
  ViewContext vc;
  /* Fill brush. */
  Brush *brush;

  /* Draw handle for 3D viewport overlay. */
  void *draw_handle;

  /* Starting time of the vector fill operator. */
  std::chrono::high_resolution_clock::time_point operator_time_start;

  /* Curve points converted to viewport 2D space. */
  Curves2DSpace curves_2d{};
  /* Curve end extensions in viewport 2D space. */
  Curves2DSpace extensions_2d{};
  /* Ratio between length of extension and length of curve end segment. */
  Array<float> extension_length_ratio;
  /* KD tree of curve ends in 2D space, used for gap closure by radius. */
  KDTree_2d *curve_ends;
  /* Flag indicating a curve end is connected by radius with one or more other curve ends. */
  Array<bool> connected_by_radius;
  /* Curve end connections (by radius) with other curve ends. */
  Vector<int2> radius_connections;

  /* The initial curve segments from which we try to find a closed fill edge. */
  Vector<IntersectingSegment2D> starting_segments;
  /* The curve segments that will form a closed fill edge. */
  Vector<EdgeSegment> segments;

  /* The intersection distance of the start segment (found by the casted ray). */
  float start_distance;
};

/** \} */

/* ------------------------------------------------------------------------------ */
/** \name Intersection functions
 * \{ */

static bool are_equal(const float a, const float b)
{
  return fabsf(a - b) < FLT_EPSILON;
}

static float get_intersection_distance(const float2 &v1,
                                       const float2 &v2,
                                       const float2 &v3,
                                       const float2 &v4)
{
  /* Check for zero length vector. */
  const float vec_length = math::length(v2 - v1);
  if (vec_length == 0.0f) {
    return 0.0f;
  }

  const float a1 = v2[1] - v1[1];
  const float b1 = v1[0] - v2[0];
  const float c1 = a1 * v1[0] + b1 * v1[1];

  const float a2 = v4[1] - v3[1];
  const float b2 = v3[0] - v4[0];
  const float c2 = a2 * v3[0] + b2 * v3[1];

  const float det = a1 * b2 - a2 * b1;

  float2 isect;
  isect[0] = (b2 * c1 - b1 * c2) / det;
  isect[1] = (a1 * c2 - a2 * c1) / det;

  return math::max(0.0f, math::min(1.0f, math::length(isect - v1) / vec_length));
}

Vector<IntersectingSegment2D> get_intersections_of_segment_with_curves(
    const float2 &segment_a,
    const float2 &segment_b,
    const int segment_curve_index,
    const float2 &adj_a,
    const float2 &adj_b,
    const Curves2DSpace *curves_2d)
{
  /* Init result vector. */
  Vector<IntersectingSegment2D> intersections;
  std::mutex mutex;

  /* Create bounding box around the segment. */
  rctf bbox_sel;
  BLI_rctf_init_minmax(&bbox_sel);
  BLI_rctf_do_minmax_v(&bbox_sel, segment_a);
  BLI_rctf_do_minmax_v(&bbox_sel, segment_b);

  const float2 segment_vec = segment_b - segment_a;

  /* Loop all strokes, looking for intersecting segments. */
  threading::parallel_for(curves_2d->point_offset.index_range(), 256, [&](const IndexRange range) {
    for (const int curve_i : range) {
      /* Do a quick bounding box check first. When the bounding box of a stroke doesn't
       * intersect with the segment, none of the stroke segments do. */
      if (!BLI_rctf_isect(&bbox_sel, &curves_2d->curve_bbox[curve_i], nullptr)) {
        continue;
      }

      /* Get point range. */
      const int point_offset = curves_2d->point_offset[curve_i];
      const int point_size = curves_2d->point_size[curve_i] -
                             (curves_2d->is_cyclic[curve_i] ? 0 : 1);
      const int point_last = point_offset + curves_2d->point_size[curve_i] - 1;

      /* Skip curves with identical (overlapping) segments, because they produce false-positive
       * intersections. Overlapping curves are most likely created by a previous fill operation. */
      bool skip_curve = false;
      for (const int point_i : IndexRange(point_offset, point_size)) {
        const int point_i_next = (point_i == point_last ? point_offset : point_i + 1);
        const float2 p0 = curves_2d->points_2d[point_i];
        const float2 p1 = curves_2d->points_2d[point_i_next];

        /* Check for identical segments. */
        if ((adj_a == p0 && segment_a == p1) || (segment_a == p0 && segment_b == p1) ||
            (segment_b == p0 && adj_b == p1) || (adj_b == p0 && segment_b == p1) ||
            (segment_b == p0 && segment_a == p1) || (segment_a == p0 && adj_a == p1))
        {
          skip_curve = true;
          break;
        }

        /* Check for adjacent segments, exactly parallel to the current. */
        if (segment_a == p0 || segment_a == p1 || segment_b == p0 || segment_b == p1) {
          const float2 p_vec = p1 - p0;
          if (fabsf(cross_v2v2(p_vec, segment_vec) < FLT_EPSILON)) {
            skip_curve = true;
            break;
          }
        }
      }
      if (skip_curve) {
        continue;
      }

      /* Find intersecting stroke segments. */
      for (const int point_i : IndexRange(point_offset, point_size)) {
        const int point_i_next = (point_i == point_last ? point_offset : point_i + 1);
        const float2 p0 = curves_2d->points_2d[point_i];
        const float2 p1 = curves_2d->points_2d[point_i_next];

        /* Don't self check. */
        if (curve_i == segment_curve_index) {
          if (segment_a == p0 || segment_a == p1 || segment_b == p0 || segment_b == p1) {
            continue;
          }
        }

        auto isect = math::isect_seg_seg(segment_a, segment_b, p0, p1);
        if (ELEM(isect.kind, isect.LINE_LINE_CROSS, isect.LINE_LINE_EXACT)) {
          IntersectingSegment2D intersection{};
          intersection.curve_index_2d = curve_i;
          intersection.point_start = point_i - point_offset;
          intersection.point_end = point_i_next - point_offset;
          intersection.distance = get_intersection_distance(segment_a, segment_b, p0, p1);
          intersection.is_filled = curves_2d->is_filled[curve_i];

          std::lock_guard lock{mutex};
          intersections.append(intersection);
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

Vector<float3> get_closed_fill_edge_as_3d_points(VectorFillData *vf)
{
  Vector<float3> edge_points;
  int edge_point_index = 0;

  /* Ensure head starting point and tail end point are an exact match. */
  EdgeSegment *tail = &vf->segments.last();
  for (auto &head : vf->segments) {
    if ((head.flag & vfFlag::IsUnused) == true) {
      continue;
    }
    if ((head.flag & vfFlag::DirectionBackwards) == true) {
      head.point_range[1] = tail->point_range[1];
    }
    else {
      head.point_range[0] = tail->point_range[0];
    }
    break;
  }

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
    int point_start = segment.point_range[0];
    int point_end = segment.point_range[1];
    if (direction == -1) {
      std::swap(point_start, point_end);
    }
    point_end += direction;
    for (int point_i = point_start; point_i != point_end; point_i += direction) {
      edge_points.append(positions[point_offset + point_i]);
      edge_point_index++;
    }

    /* Calculate regular intersection point. */
    if ((segment.flag & vfFlag::IsIntersected) == true) {
      const SegmentEnd *segment_end = &segment.segment_ends[segment.segment_ends_index];
      const float3 isect_point = edge_points.last() +
                                 (positions[point_offset + segment_end->ori_segment_point_end] -
                                  edge_points.last()) *
                                     segment_end->distance;
      edge_points.append(isect_point);
      edge_point_index++;
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
      edge_points.append(isect_point);
      edge_point_index++;
    }
  }

  /* Remove last point, since it is the same as the first point. */
  edge_points.remove_last();

  return edge_points;
}

static void create_fill_geometry(VectorFillData *vf)
{
  /* Get the edge points in 3D space. */
  Vector<float3> fill_points = get_closed_fill_edge_as_3d_points(vf);

  /* Create geometry. */
  BLI_assert(vf->grease_pencil->has_active_layer());
  const bke::greasepencil::Layer *active_layer = vf->grease_pencil->get_active_layer();
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
}

static bool segments_have_overlap(EdgeSegment *segment, EdgeSegment *tail)
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

  /* Loop through the edge segments and see if there is overlap with the last one. */
  EdgeSegment *last_segment = &vf->segments.last();
  for (const int segment_i : vf->segments.index_range().drop_back(1)) {
    if (segments_have_overlap(&vf->segments[segment_i], last_segment)) {
      return true;
    }
    vf->segments[segment_i].flag |= vfFlag::IsUnused;
  }

  return false;
}

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
        intersection.distance < vf->start_distance)
    {
      continue;
    }

    /* Copy intersection data. */
    SegmentEnd segment_end{};
    segment_end.curve_index_2d = intersection.curve_index_2d;
    segment_end.point_start = intersection.point_start;
    segment_end.point_end = intersection.point_end;
    segment_end.ori_segment_point_end = segment_point_b_index;
    segment_end.distance = intersection.distance;
    if (intersection.is_filled) {
      segment_end.flag |= vfFlag::IsFilled;
    }

    /* Since we follow the fill edge clockwise, at each intersection we want to take a right turn
     * first (for finding the narrowest path).
     * So we have to determine the direction of the intersecting curve, because then we know what
     * the right turn is. */
    const int point_offset = curves_2d->point_offset[intersection.curve_index_2d];
    const float2 isect_point_start = curves_2d->points_2d[point_offset + intersection.point_start];
    const bool point_start_is_on_right = cross_v2v2(segment_vec,
                                                    isect_point_start - segment_point_a) < 0.0f;
    if (point_start_is_on_right) {
      segment_end.flag |= vfFlag::DirectionBackwards;
    }

    /* Handle intersection by end extension. */
    if (is_end_extension) {
      segment_end.curve_index_2d = int(intersection.curve_index_2d / 2);
      segment_end.flag = vfFlag::WithEndExtension;

      /* Set curve direction: backwards for extension at end of curve. */
      if ((intersection.curve_index_2d % 2) == 1) {
        segment_end.flag |= vfFlag::DirectionBackwards;
        segment_end.point_start = vf->curves_2d.point_size[segment_end.curve_index_2d] - 1;
      }
      segment_end.point_end = intersection.point_start;

      /* Set right turn for extension. */
      if (!point_start_is_on_right) {
        segment_end.flag |= vfFlag::ExtensionAtThirdTurn;
      }

      /* Set fill flag. */
      if (vf->curves_2d.is_filled[segment_end.curve_index_2d]) {
        segment_end.flag |= vfFlag::IsFilled;
      }
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

static bool walk_along_curve(VectorFillData *vf, EdgeSegment *segment, const int started_at = -1)
{
  if ((segment->flag & vfFlag::IsInspected) == true) {
    return false;
  }
  segment->flag |= vfFlag::IsInspected;

  /* Walk along the curve until an intersection or the end of the curve is reached. */
  const int curve_i = segment->curve_index_2d;
  const bool is_cyclic = vf->curves_2d.is_cyclic[curve_i];
  const int direction = ((segment->flag & vfFlag::DirectionBackwards) == true) ? -1 : 1;
  const int point_size = vf->curves_2d.point_size[curve_i];
  const int point_offset = vf->curves_2d.point_offset[curve_i];
  int point_i = segment->point_range[0];
  const int point_started = (started_at == -1 ? point_i : started_at);
  bool check_gaps = false;

  float2 segment_point_a = {FLT_MAX, FLT_MAX};
  const int point_prev = get_next_curve_point(point_i, -direction, point_size, is_cyclic);
  if (point_prev != -1) {
    segment_point_a = vf->curves_2d.points_2d[point_offset + point_prev];
  }

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
      new_segment.point_range[0] = point_i;
      new_segment.flag = segment->flag;
      new_segment.flag |= vfFlag::IsLastOfCycle;
      if (point_i == point_started) {
        new_segment.point_range[1] = point_i;
      }
      else {
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
        segment_point_a, segment_point_b, curve_i, point_a_prev, point_b_next, &vf->curves_2d);
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
      const int extension_index = curve_i * 2 + (direction == 1 ? 1 : 0);
      intersections = get_intersections_of_segment_with_curves(segment_point_a,
                                                               segment_point_b,
                                                               extension_index,
                                                               point_a_prev,
                                                               point_b_next,
                                                               &vf->extensions_2d);
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
      /* Sort intersections on distance. This is important to find the narrowest edge.
       * Give priority to curves without fill material. */
      std::sort(segment->segment_ends.begin(),
                segment->segment_ends.end(),
                [](const SegmentEnd &a, const SegmentEnd &b) {
                  if (are_equal(a.distance, b.distance)) {
                    return ((a.flag & vfFlag::IsFilled) == false);
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

  /* Check for gap closure extending the end of the curve. */
  if (vf->gap_close_extend) {
    /* Get the coordinates of start/end extension. */
    const int extension_index = curve_i * 2 + (direction == 1 ? 1 : 0);
    const int point_offset = vf->extensions_2d.point_offset[extension_index];
    const float2 segment_point_a = vf->extensions_2d.points_2d[point_offset];
    const float2 segment_point_b = vf->extensions_2d.points_2d[point_offset + 1];

    /* Get intersections with other curve end extensions. */
    Vector<IntersectingSegment2D> intersections = get_intersections_of_segment_with_curves(
        segment_point_a,
        segment_point_b,
        extension_index,
        {FLT_MAX, FLT_MAX},
        {FLT_MAX, FLT_MAX},
        &vf->extensions_2d);
    add_segment_end(&vf->extensions_2d,
                    segment,
                    segment_point_a,
                    segment_point_b,
                    -1,
                    intersections,
                    true,
                    vf);

    /* Get intersections with other curves. */
    if (vf->extensions_collide_with_curves) {
      intersections = get_intersections_of_segment_with_curves(segment_point_a,
                                                               segment_point_b,
                                                               curve_i,
                                                               {FLT_MAX, FLT_MAX},
                                                               {FLT_MAX, FLT_MAX},
                                                               &vf->curves_2d);
      add_segment_end(
          &vf->curves_2d, segment, segment_point_a, segment_point_b, -1, intersections, false, vf);
    }

    if (!segment->segment_ends.is_empty()) {
      /* Sort intersections on distance. We want to start with the smallest distance to find the
       * narrowest fill edge. */
      std::sort(segment->segment_ends.begin(),
                segment->segment_ends.end(),
                [](const SegmentEnd &a, const SegmentEnd &b) {
                  if (are_equal(a.distance, b.distance)) {
                    return ((a.flag & vfFlag::IsFilled) == false);
                  }
                  return a.distance < b.distance;
                });

      /* Set gap closure flags. */
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
    const float curve_angle = math::atan2(curve_point_prev[1] - curve_point[1],
                                          curve_point_prev[0] - curve_point[0]);

    const float two_pi = M_PI * 2;
    const float max_dist = 2 * vf->gap_distance;

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
       * shortest fill edge, we explore the smallest angle first (= most right turn).
       * Note: we use the `distance` field to store the angle. */
      const float2 co = vf->curves_2d.points_2d[nearest_point_offset + nearest_point_i];
      segment_end.distance = math::atan2(co[1] - curve_point[1], co[0] - curve_point[0]) -
                             curve_angle;
      if (segment_end.distance < 0.0f) {
        segment_end.distance += two_pi;
      }

      /* Append to segment end list. */
      segment->segment_ends.append(segment_end);
    }

    if (!segment->segment_ends.is_empty()) {
      /* Sort nearest curves on angle. */
      std::sort(segment->segment_ends.begin(),
                segment->segment_ends.end(),
                [](const SegmentEnd &a, const SegmentEnd &b) {
                  if (are_equal(a.distance, b.distance)) {
                    return ((a.flag & vfFlag::IsFilled) == false);
                  }
                  return a.distance < b.distance;
                });

      /* Set gap closure flags. */
      segment->flag |= vfFlag::IsGapClosure;
    }
  }

  return true;
}

static void take_next_turn_on_extension_intersection(VectorFillData *vf, SegmentEnd *segment_end)
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
    /* Left turn. */
    case 1: {
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
  segment.point_range[0] = point_start;
  segment.flag = flag;

  vf->segments.append(segment);
}

static void take_next_turn_on_intersection(VectorFillData *vf,
                                           SegmentEnd *segment_end,
                                           const EdgeSegment *ori_segment)
{
  int curve_index_2d, point_start;
  vfFlag flag = vfFlag::None;

  /* Intersections with end extensions are a bit different: only two turns matter. */
  if ((segment_end->flag & vfFlag::WithEndExtension) == true) {
    if (segment_end->turn == 0 && (segment_end->flag & vfFlag::ExtensionAtThirdTurn) == true) {
      /* Skip right turn. */
      segment_end->turn++;
    }
    if (segment_end->turn == 2 && (segment_end->flag & vfFlag::ExtensionAtThirdTurn) == false) {
      /* Skip left turn. */
      return;
    }
  }

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
  EdgeSegment segment{};
  segment.curve_index_2d = curve_index_2d;
  segment.point_range[0] = point_start;
  segment.flag = flag;

  vf->segments.append(segment);
}

static void take_next_gap_closure_curve(VectorFillData *vf, SegmentEnd *segment_end)
{
  /* Add new segment to the fill edge. */
  EdgeSegment segment{};
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
    /* Emergency break: when the operator is taking too much time, jump to the conclusion that no
     * closed fill edge can be found. */
    auto t2 = std::chrono::high_resolution_clock::now();
    auto delta_t = std::chrono::duration_cast<std::chrono::milliseconds>(t2 -
                                                                         vf->operator_time_start);
    if (delta_t.count() > GP_VFILL_MAX_EXECUTION_TIME) {
      return false;
    }

    /* Explore the last edge segment in the array. */
    EdgeSegment *segment = &vf->segments.last();

    /* Walk along the curve until an intersection or the end of the curve is reached. */
    bool changed = walk_along_curve(vf, segment);
    segment = &vf->segments.last();

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
      SegmentEnd *segment_end = &segment->segment_ends[segment->segment_ends_index];

      /* Handle segment that ends at an intersection. */
      if ((segment->flag & vfFlag::IsIntersected) == true) {
        /* When all intersection turns are explored, continue with the
         * next intersection (if any). */
        if (segment_end->turn >= 3) {
          segment->segment_ends_index++;
          continue;
        }

        /* Take the next turn on the intersection:
         * 0 = right turn, 1 = straight ahead, 2 = left turn. */
        take_next_turn_on_intersection(vf, segment_end, segment);

        segment_end->turn++;
      }
      /* Handle gap closures. */
      else {
        /* End extension intersects with an other curve. */
        if (vf->gap_close_extend && (segment_end->flag & vfFlag::WithEndExtension) == false) {
          /* When an end extension intersects with an other curve, we have two turns to explore:
           * right and left. */
          if (segment_end->turn >= 2) {
            segment->segment_ends_index++;
            continue;
          }

          take_next_turn_on_extension_intersection(vf, segment_end);
          segment_end->turn++;
        }
        else {
          /* Gap closure with radius or with an other curve end extension: jump onto the next
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
  /* Find a first (arbitrary) edge point of the fill area by casting a ray from the mouse click
   * position in one of four directions: up, right, down, left. When this ray crosses a curve,
   * we use the intersection point as the starting point of the fill edge.
   * Note: we check in four directions, because the user can click near a gap in the fill edge. */
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
        vf->mouse_pos, ray_vec, -1, {FLT_MAX, FLT_MAX}, {FLT_MAX, FLT_MAX}, &vf->curves_2d);

    if (vf->starting_segments.size() == 0) {
      continue;
    }

    /* Sort starting segments on distance (closest first). Give priority to curves with only a
     * stroke material, to avoid interference with overlapping filled curves at exact the same
     * position. */
    std::sort(vf->starting_segments.begin(),
              vf->starting_segments.end(),
              [](const IntersectingSegment2D &a, const IntersectingSegment2D &b) {
                if (are_equal(a.distance, b.distance)) {
                  return (!a.is_filled);
                }
                return a.distance < b.distance;
              });

    /* From the first edge point we try to follow the fill edge clockwise, until we find
     * a closed loop. */
    IntersectingSegment2D *start_segment = &vf->starting_segments.first();

    /* Start with an empty edge segment list. */
    vf->segments.clear();

    /* Add first edge segment. */
    EdgeSegment segment{};
    segment.curve_index_2d = start_segment->curve_index_2d;
    segment.point_range[0] = start_segment->point_start;

    /* Set the minimum distance for checking intersections on this first segment. */
    segment.flag = vfFlag::CheckStartDistance;
    const int point_offset = vf->curves_2d.point_offset[segment.curve_index_2d];
    const float2 segment_a = vf->curves_2d.points_2d[point_offset + start_segment->point_start];
    const float2 segment_b = vf->curves_2d.points_2d[point_offset + start_segment->point_end];
    vf->start_distance = get_intersection_distance(segment_a, segment_b, vf->mouse_pos, ray_vec);

    /* We want to follow the fill edge clockwise. Determine in which direction we have to follow
     * the curve for that. */
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
      KDTreeNearest_2d nearest[GP_VFILL_RADIUS_NEAREST_NUM];
      const int nearest_num = BLI_kdtree_2d_find_nearest_n(vf->curve_ends,
                                                           vf->curves_2d.points_2d[point_index],
                                                           nearest,
                                                           GP_VFILL_RADIUS_NEAREST_NUM);

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

static void init_curve_end_extensions(VectorFillData *vf)
{
  const int curve_num = vf->curves_2d.point_offset.size() * 2;
  const int point_num = curve_num * 2;

  vf->extensions_2d.point_offset = Array<int>(curve_num);
  vf->extensions_2d.point_size = Array<int>(curve_num, 2);
  vf->extensions_2d.is_cyclic = Array<bool>(curve_num, false);
  vf->extensions_2d.is_filled = Array<bool>(curve_num, false);
  vf->extensions_2d.drawing_index_2d = Array<int>(curve_num);
  vf->extensions_2d.points_2d = Array<float2>(point_num);
  vf->extensions_2d.curve_bbox = Array<rctf>(curve_num);
  vf->extension_length_ratio = Array<float>(curve_num);
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

static void get_curve_end_extensions(VectorFillData *vf)
{
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

static int get_curve_point_by_end_index(VectorFillData *vf, const int end_index)
{
  const int curve_i = int(end_index / 2);
  int point_i = vf->curves_2d.point_offset[curve_i];
  if ((end_index % 2) == 1) {
    point_i += vf->curves_2d.point_size[curve_i] - 1;
  }
  return point_i;
}

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
    /* TODO: use color from UI setting. */
    immUniformColor4f(0.0f, 1.0f, 1.0f, 1.0f);

    for (const int curve_i : vf->extensions_2d.point_offset.index_range()) {
      /* Skip cyclic curves. */
      if (vf->extensions_2d.is_cyclic[curve_i]) {
        continue;
      }
      const int point_i = vf->extensions_2d.point_offset[curve_i];

      immBegin(GPU_PRIM_LINES, 2);
      immVertex2fv(shdr_pos, vf->extensions_2d.points_2d[point_i]);
      immVertex2fv(shdr_pos, vf->extensions_2d.points_2d[point_i + 1]);
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
}

static void update_gap_distance(VectorFillData *vf, const float delta)
{
  vf->brush->gpencil_settings->fill_extend_fac += delta;
  vf->brush->gpencil_settings->fill_extend_fac = math::max(
      0.0f, math::min(10.0f, vf->brush->gpencil_settings->fill_extend_fac));
  vf->gap_distance = vf->brush->gpencil_settings->fill_extend_fac * GP_VFILL_GAP_PIXEL_FACTOR;
  if (vf->gap_close_extend) {
    get_curve_end_extensions(vf);
  }
  if (vf->gap_close_radius) {
    get_connected_curve_end_radii(vf);
  }
  ED_region_tag_redraw(vf->vc.region);
}

static void get_latest_toolsettings(VectorFillData *vf)
{
  const bool use_gap_closing = (vf->brush->gpencil_settings->fill_extend_fac > FLT_EPSILON);
  vf->gap_close_extend = (vf->brush->gpencil_settings->fill_extend_mode == GP_FILL_EMODE_EXTEND) &&
                         use_gap_closing;
  vf->gap_close_radius = (vf->brush->gpencil_settings->fill_extend_mode == GP_FILL_EMODE_RADIUS) &&
                         use_gap_closing;
  vf->gap_distance = vf->brush->gpencil_settings->fill_extend_fac * GP_VFILL_GAP_PIXEL_FACTOR;
}

static void init(bContext *C, wmOperator *op)
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
  vf->gap_distance = vf->brush->gpencil_settings->fill_extend_fac * GP_VFILL_GAP_PIXEL_FACTOR;
  vf->wait_for_release = true;
  vf->wait_event_type = LEFTMOUSE;

  /* Convert curves to viewport 2D space. */
  /* TODO: set layer options (visible, above, all etc.) */
  vf->curves_2d = editable_strokes_in_2d_space_get(&vf->vc, vf->vc.obact, true);

  /* When using extensions of the curve ends to close gaps, build an array of those two-point
   * 'curves'. */
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
}

static void exit(bContext *C, wmOperator *op)
{
  WM_cursor_modal_restore(CTX_wm_window(C));

  if (op->customdata != nullptr) {
    VectorFillData *vf = static_cast<VectorFillData *>(op->customdata);
    if (vf->draw_handle) {
      ED_region_draw_cb_exit(vf->vc.region->type, vf->draw_handle);
      ED_region_tag_redraw(vf->vc.region);
    }
    if (vf->gap_close_radius) {
      BLI_kdtree_2d_free(vf->curve_ends);
    }
    MEM_delete(vf);
    op->customdata = nullptr;
  }
}

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
      /* Second click: perform the actual fill. */
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
      printf("Vector fill took %lld ms.\n", delta_t.count());

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

  /* TODO: check active layer. */
  /* TODO: check active frame. */
  /* TODO: check active material. */

  /* Init tool data. */
  init(C, op);

  /* Add modal handler. */
  WM_event_add_modal_handler(C, op);

  /* Set cursor. */
  WM_cursor_modal_set(CTX_wm_window(C), WM_CURSOR_PAINT_BRUSH);

  return OPERATOR_RUNNING_MODAL;
}

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
