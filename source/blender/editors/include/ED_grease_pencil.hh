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
struct GreasePencilDrawing;
struct Main;
struct Object;
struct KeyframeEditData;
struct ViewContext;
struct wmKeyConfig;
struct ToolSettings;

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
void ED_operatortypes_grease_pencil_fill();
void ED_keymap_grease_pencil(wmKeyConfig *keyconf);
/**
 * Get the selection mode for Grease Pencil selection operators: point, stroke, segment.
 */
eAttrDomain ED_grease_pencil_selection_domain_get(const ToolSettings *tool_settings);

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

struct DrawingInfo {
  const bke::greasepencil::Drawing &drawing;
  const int layer_index;
  const int frame_number;
};
struct MutableDrawingInfo {
  bke::greasepencil::Drawing &drawing;
  const int layer_index;
  const int frame_number;
};
Array<int> get_frame_numbers_for_layer(const bke::greasepencil::Layer &layer,
                                       const int current_frame,
                                       const bool use_multi_frame_editing);
Array<MutableDrawingInfo> retrieve_editable_drawings(const Scene &scene,
                                                     GreasePencil &grease_pencil);
Array<DrawingInfo> retrieve_visible_drawings(const Scene &scene,
                                             const GreasePencil &grease_pencil);

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
 * Structure for the accumulated curves of a set of drawings (layers) converted to
 * viewport 2D space.
 * All points of all curves of all drawings are put in one contiguous array. Curve points are
 * accessible by offset indices. E.g. curve no. 7 of drawing no. 3 can be accessed by:
 * curve_contiguous = curve_offset[3 (= drawing index)] + 7 (= curve index)
 * point_contiguous = point_offset[curve_contiguous]
 * first_point_of_curve = points_2d[point_contiguous]
 */
struct Curves2DSpace {
  Vector<const bke::greasepencil::Drawing *> drawings;
  /* Curve offset for each drawing (layer). So when there are three drawings with 12, 15 and 8
   * curves, the curve offsets will be [0, 12, 27 (= 12 + 15)]. */
  Vector<int> curve_offset;
  /* Point index offset for each curve. */
  Array<int> point_offset;
  /* Number of points in each curve. */
  Array<int> point_size;
  /* Cyclic flag for each curve. */
  Array<bool> is_cyclic;
  /* Material stroke flag for each curve. */
  Array<bool> has_stroke;
  /* Index reference to `drawings` for each curve. */
  Array<int> drawing_index_2d;
  /* Contiguous array with all point positions in 2D space. */
  Array<float2> points_2d;
  /* Bounding box of each curve in 2D space. */
  Array<rctf> curve_bbox;
};

/**
 * Convert all given Grease Pencil drawings to viewport 2D space.
 *
 * \return A struct with the 2D representation of all editable strokes.
 */
Curves2DSpace curves_in_2d_space_get(ViewContext *vc,
                                     Object *ob,
                                     Vector<const bke::greasepencil::Drawing *> &drawings,
                                     Vector<int> &layer_index,
                                     const int frame_number,
                                     const bool get_stroke_flag = false);

}  // namespace blender::ed::greasepencil
