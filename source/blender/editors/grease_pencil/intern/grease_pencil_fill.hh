/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgreasepencil
 */

#pragma once

/* DEBUG: show developer extra's in viewport and console. */
/* #define GP_FILL_DEBUG_MODE */

#include "BKE_brush.hh"
#include "BKE_context.h"
#include "BKE_grease_pencil.hh"
#include "BKE_report.h"

#include "BLI_kdtree.h"

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
#include "grease_pencil_fill_vector.hh"

#ifdef GP_FILL_DEBUG_MODE
#  include "BLF_api.h"
#  include "UI_interface.hh"
#  include "UI_resources.hh"
#endif

namespace blender::ed::greasepencil::fill {

/* Number of nearest neighbours when looking for gap closure by curve end radius. */
static constexpr unsigned int RADIUS_NEAREST_NUM = 12;
/* Number of pixels the gap closure size stands for. */
static constexpr float GAP_PIXEL_FACTOR = 25.0f;
/* Margin for intersection distances to be considered equal. */
static constexpr float DISTANCE_EPSILON = 0.001f;
static constexpr float DISTANCE_SORT_EPSILON = 0.02f;
/* Margin for vector cross product considered to be parallel. */
static constexpr float PARALLEL_EPSILON = 0.01f;

/* Runtime fill data. */
struct FillData {
 public:
  /* For modal event handling: wait for release until processing the event key again. */
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

  /* Colors of gap closure visual aids. */
  float gap_closure_color[3];
  float gap_closed_color[3];
  float gap_proximity_color[4];

  /* Active Grease Pencil object. */
  GreasePencil *grease_pencil;
  /* View context data.*/
  ViewContext vc;
  /* Fill brush. */
  Brush *brush;
  /* Additive drawing (from tool settings). */
  bool additive_drawing;

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

  /* Flood fill runtime data. */
  FloodFillData ff;

  /* Vector fill runtime data. */
  VectorFillData vf;
};

/**
 * Perform a vector fill.
 */
bool vector_fill_do(FillData *fd);
/**
 * Perform a flood fill.
 */
bool flood_fill_do(FillData *fd);
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
    void (*store_overlapping_segment)(int, int, bool, int, int, FillData *) = nullptr,
    const int segment_point_a = 0,
    const int segment_point_b = 0,
    const int segment_direction = 1,
    const bool use_epsilon = false,
    FillData *fd = nullptr);

}  // namespace blender::ed::greasepencil::fill
