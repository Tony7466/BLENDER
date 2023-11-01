/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup editors
 */

#pragma once

#include "BKE_attribute.h"
#include "BKE_grease_pencil.hh"

#include "BLI_generic_span.hh"
#include "BLI_index_mask.hh"
#include "BLI_math_matrix_types.hh"
#include "BLI_rect.h"

#include "ED_keyframes_edit.hh"

struct bContext;
struct Main;
struct Object;
struct KeyframeEditData;
struct wmKeyConfig;
struct ToolSettings;
struct ViewContext;

enum {
  LAYER_REORDER_ABOVE,
  LAYER_REORDER_BELOW,
};

/* -------------------------------------------------------------------- */
/** \name C Wrappers
 * \{ */

void ED_operatortypes_grease_pencil();
void ED_operatortypes_grease_pencil_draw();
void ED_operatortypes_grease_pencil_frames();
void ED_operatortypes_grease_pencil_layers();
void ED_operatortypes_grease_pencil_select();
void ED_operatortypes_grease_pencil_edit();
void ED_keymap_grease_pencil(wmKeyConfig *keyconf);
/**
 * Get the selection mode for Grease Pencil selection operators: point, stroke, segment.
 */
eAttrDomain ED_grease_pencil_selection_domain_get(const ToolSettings *tool_settings);
/**
 * Check if the selection mode for Grease Pencil selection operators is 'segment'.
 * \return True when the selection mode is 'segment'.
 */
bool ED_grease_pencil_segment_selection_mode(const ToolSettings *tool_settings);

/** \} */

namespace blender::ed::greasepencil {

void set_selected_frames_type(bke::greasepencil::Layer &layer,
                              const eBezTriple_KeyframeType key_type);

bool snap_selected_frames(GreasePencil &grease_pencil,
                          bke::greasepencil::Layer &layer,
                          Scene &scene,
                          const eEditKeyframes_Snap mode);

bool mirror_selected_frames(GreasePencil &grease_pencil,
                            bke::greasepencil::Layer &layer,
                            Scene &scene,
                            const eEditKeyframes_Mirror mode);

/* Creates duplicate frames for each selected frame in the layer. The duplicates are stored in the
 * LayerTransformData structure of the layer runtime data. This function also deselects the
 * selected frames, while keeping the duplicates selected. */
bool duplicate_selected_frames(GreasePencil &grease_pencil, bke::greasepencil::Layer &layer);

bool remove_all_selected_frames(GreasePencil &grease_pencil, bke::greasepencil::Layer &layer);

void select_layer_channel(GreasePencil &grease_pencil, bke::greasepencil::Layer *layer);

/**
 * Sets the selection flag, according to \a selection_mode to the frame at \a frame_number in the
 * \a layer if such frame exists. Returns false if no such frame exists.
 */
bool select_frame_at(bke::greasepencil::Layer &layer,
                     const int frame_number,
                     const short select_mode);

void select_frames_at(bke::greasepencil::LayerGroup &layer_group,
                      const int frame_number,
                      const short select_mode);

void select_all_frames(bke::greasepencil::Layer &layer, const short select_mode);

void select_frames_region(KeyframeEditData *ked,
                          bke::greasepencil::TreeNode &node,
                          const short tool,
                          const short select_mode);

void select_frames_range(bke::greasepencil::TreeNode &node,
                         const float min,
                         const float max,
                         const short select_mode);

/**
 * Returns true if any frame of the \a layer is selected.
 */
bool has_any_frame_selected(const bke::greasepencil::Layer &layer);

void create_keyframe_edit_data_selected_frames_list(KeyframeEditData *ked,
                                                    const bke::greasepencil::Layer &layer);

float brush_radius_world_space(bContext &C, int x, int y);

bool active_grease_pencil_poll(bContext *C);
bool editable_grease_pencil_poll(bContext *C);
bool editable_grease_pencil_point_selection_poll(bContext *C);
bool grease_pencil_painting_poll(bContext *C);

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
                                       FunctionRef<float(int64_t, int64_t, int64_t)> dist_function,
                                       MutableSpan<bool> dst);

Array<float2> polyline_fit_curve(Span<float2> points,
                                 float error_threshold,
                                 const IndexMask &corner_mask);

IndexMask polyline_detect_corners(Span<float2> points,
                                  float radius_min,
                                  float radius_max,
                                  int samples_max,
                                  float angle_threshold,
                                  IndexMaskMemory &memory);

/**
 * Structure for curves converted to viewport 2D space.
 */
struct Curves2DSpace {
  /* Curve index offset for each drawing (= layer). */
  Vector<int> curve_offset;
  /* Point index offset for each curve. */
  Array<int> point_offset;
  /* Number of points in each curve. */
  Array<int> point_size;
  /* Contiguous array with all point positions in 2D space. */
  Array<float2> points_2d;
  /* Bounding box of each curve in 2D space. */
  Array<rctf> curve_bbox;
};

/**
 * Collect all editable strokes in a GP object and convert them to
 * viewport 2D space.
 *
 * \return A struct with the 2D representation of all editable strokes.
 */
Curves2DSpace editable_strokes_in_2d_space_get(ViewContext *vc, GreasePencil *grease_pencil);
/**
 * Checks if a segment of 2 points is intersected by any of the given 2D strokes.
 *
 * \param segment_start: Start coordinates of the segment.
 * \param segment_end: End coordinates of the segment.
 * \param segment_curve_index: Index of the segment curve in the `curves_2d` struct.
 * Used to avoid a false-positive self-intersecting result.
 * \param curves_2d: A struct with the 2D representation of all editable strokes.
 * Obtained by #editable_strokes_in_2d_space_get.
 * \return: True if the segment is intersected by a stroke, else false.
 */
bool intersect_segment_strokes_2d(const float2 segment_start,
                                  const float2 segment_end,
                                  const int segment_curve_index,
                                  const Curves2DSpace *curves_2d);
/**
 * Get the selection state of all points in a Grease Pencil drawing.
 *
 * \return An array with the selection state of all points in curves.
 */
Array<bool> point_selection_get(const GreasePencilDrawing *drawing);
/**
 * Expand the point selection in a Grease Pencil drawing to stroke segments.
 * A segment is the part of a stroke between other, intersecting strokes.
 *
 * \param old_selection: The selection state of points before the selection change.
 * Obtained by #point_selection_get.
 * \param curves: The curves in a Grease Pencil drawing.
 * \param layer_index_2d: The index of the drawing in the `curves_2d` struct.
 * \param curves_2d: A struct with the 2D representation of all editable strokes.
 * Obtained by #editable_strokes_in_2d_space_get.
 */
void expand_changed_selection_to_segments(Array<bool> &old_selection,
                                          bke::CurvesGeometry &curves,
                                          const int layer_index_2d,
                                          const Curves2DSpace *curves_2d);
/**
 * Expand a random point selection in a Grease Pencil drawing to stroke segments.
 * A segment is the part of a stroke between other, intersecting strokes.
 *
 * \param curves: The curves in a Grease Pencil drawing.
 * \param layer_index_2d: The index of the drawing in the `curves_2d` struct.
 * \param curves_2d: A struct with the 2D representation of all editable strokes.
 * Obtained by #editable_strokes_in_2d_space_get.
 * \param random_seed: Seed for the random generator.
 * \param probability: Higher probability means more segments are selected.
 */
void expand_random_selection_to_segments(bke::CurvesGeometry &curves,
                                         const int layer_index_2d,
                                         const Curves2DSpace *curves_2d,
                                         const uint32_t random_seed,
                                         const float probability);

}  // namespace blender::ed::greasepencil
