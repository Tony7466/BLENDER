/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup editors
 */

#pragma once

#include "BKE_attribute.h"
#include "BLI_rect.h"

struct bContext;

struct GreasePencil;
struct GreasePencilDrawing;
struct Main;
struct Object;
struct ViewContext;

struct wmKeyConfig;

#ifdef __cplusplus
extern "C" {
#endif

enum {
  LAYER_REORDER_ABOVE,
  LAYER_REORDER_BELOW,
};

/* -------------------------------------------------------------------- */
/** \name C Wrappers
 * \{ */

void ED_operatortypes_grease_pencil(void);
void ED_operatortypes_grease_pencil_draw(void);
void ED_operatortypes_grease_pencil_frames(void);
void ED_operatortypes_grease_pencil_layers(void);
void ED_operatortypes_grease_pencil_select(void);
void ED_operatortypes_grease_pencil_edit(void);
void ED_operatortypes_grease_pencil_fill(void);
void ED_keymap_grease_pencil(struct wmKeyConfig *keyconf);
/**
 * Get the selection mode for Grease Pencil selection operators: point, stroke, segment.
 */
eAttrDomain ED_grease_pencil_selection_domain_get(struct bContext *C);

/** \} */

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#  include "BKE_grease_pencil.hh"

#  include "BLI_generic_span.hh"
#  include "BLI_math_matrix_types.hh"

namespace blender::ed::greasepencil {

void select_layer_channel(GreasePencil *grease_pencil, bke::greasepencil::Layer *layer);

/**
 * Sets the selection flag, according to \a selection_mode to the frame at \a frame_number in the
 * \a layer if such frame exists. Returns false if no such frame exists.
 */
bool select_frame_at(bke::greasepencil::Layer *layer,
                     const int frame_number,
                     const short select_mode);

void select_all_frames(bke::greasepencil::Layer *layer, const short select_mode);

/**
 * Returns true if any frame of the \a layer is selected.
 */
bool layer_has_any_frame_selected(const bke::greasepencil::Layer *layer);

bool active_grease_pencil_poll(bContext *C);
bool editable_grease_pencil_poll(bContext *C);
bool editable_grease_pencil_point_selection_poll(bContext *C);
bool grease_pencil_painting_poll(bContext *C);
bool grease_pencil_painting_fill_poll(bContext *C);

void create_blank(Main &bmain, Object &object, int frame_number);
void create_stroke(Main &bmain, Object &object, float4x4 matrix, int frame_number);
void create_suzanne(Main &bmain, Object &object, float4x4 matrix, const int frame_number);

void gaussian_blur_1D(const GSpan src,
                      int64_t iterations,
                      float influence,
                      bool smooth_ends,
                      bool keep_shape,
                      bool is_cyclic,
                      GMutableSpan dst);

int64_t ramer_douglas_peucker_simplify(IndexRange range,
                                       float epsilon,
                                       FunctionRef<float(IndexRange, int64_t)> dist_function,
                                       MutableSpan<bool> dst);

/**
 * Structure for curves converted to viewport 2D space.
 */
struct Curves2DSpace {
  Vector<GreasePencilDrawing *> drawings;
  /* Curve index offset for each GP drawing. */
  Vector<int> curve_offset;
  /* Point index offset for each curve. */
  Array<int> point_offset;
  /* Number of points in each curve. */
  Array<int> point_size;
  /* Cyclic flag for each curve. */
  Array<bool> is_cyclic;
  /* Index reference to `drawings` for each curve. */
  Array<int> drawing_index_2d;
  /* Contiguous array with all point positions in 2D space. */
  Array<float2> points_2d;
  /* Bounding box of each curve in 2D space. */
  Array<rctf> curve_bbox;
};

/**
 * Intersecting segment in 2D space.
 */
struct IntersectingSegment2D {
  /* Curve index in `Curves2DSpace` struct. */
  int curve_index_2d;
  /* Start point of the intersection (relative index: 0..<curve_point_size>). */
  int point_start;
  /* End point of the intersection (relative index: 0..<curve_point_size>). */
  int point_end;
  /* Distance of the intersection on the intersected segment. */
  float distance;
};

/**
 * Collect all editable strokes in a GP object and convert them to
 * viewport 2D space.
 *
 * \return A struct with the 2D representation of all editable strokes.
 */
Curves2DSpace editable_strokes_in_2d_space_get(ViewContext *vc, Object *ob);

/**
 * Get the intersections of a two point segment with all given 2D strokes.
 *
 * \param segment_start: Start coordinates of the segment.
 * \param segment_end: End coordinates of the segment.
 * \param segment_curve_index: Index of the segment curve in the `curves_2d` struct.
 * Used to avoid a false-positive self-intersecting result.
 * \param curves_2d: A struct with the 2D representation of all editable strokes.
 * Obtained by #editable_strokes_in_2d_space_get.
 * \return: Vector of all intersections.
 */
Vector<IntersectingSegment2D> intersections_segment_strokes_2d(const float2 segment_start,
                                                               const float2 segment_end,
                                                               const int segment_curve_index,
                                                               const Curves2DSpace *curves_2d);

}  // namespace blender::ed::greasepencil
#endif
