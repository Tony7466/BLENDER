/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup editors
 */

#pragma once

#include "BKE_grease_pencil.hh"

#include "BLI_generic_span.hh"
#include "BLI_index_mask.hh"
#include "BLI_math_matrix_types.hh"
#include "BLI_rect.h"

#include "ED_keyframes_edit.hh"

#include "WM_api.hh"

struct bContext;
struct GreasePencilDrawing;
struct Main;
struct Object;
struct KeyframeEditData;
struct wmKeyConfig;
struct wmOperator;
struct ToolSettings;
struct Scene;
struct UndoType;
struct ViewContext;
struct ViewDepths;
struct View3D;
namespace blender {
namespace bke {
enum class AttrDomain : int8_t;
}
}  // namespace blender

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
void ED_operatortypes_grease_pencil_material();
void ED_operatortypes_grease_pencil_fill();
void ED_operatormacros_grease_pencil();
void ED_keymap_grease_pencil(wmKeyConfig *keyconf);

void ED_undosys_type_grease_pencil(UndoType *undo_type);
/**
 * Get the selection mode for Grease Pencil selection operators: point, stroke, segment.
 */
blender::bke::AttrDomain ED_grease_pencil_selection_domain_get(const ToolSettings *tool_settings);

/** \} */

namespace blender::ed::greasepencil {

enum class DrawingPlacementDepth { ObjectOrigin, Cursor, Surface, NearestStroke };

enum class DrawingPlacementPlane { View, Front, Side, Top, Cursor };

class DrawingPlacement {
  const ARegion *region_;
  const View3D *view3d_;

  DrawingPlacementDepth depth_;
  DrawingPlacementPlane plane_;
  ViewDepths *depth_cache_ = nullptr;
  float surface_offset_;

  float3 placement_loc_;
  float3 placement_normal_;
  float4 placement_plane_;

  float4x4 layer_space_to_world_space_;
  float4x4 world_space_to_layer_space_;

 public:
  DrawingPlacement() = default;
  DrawingPlacement(const Scene &scene,
                   const ARegion &region,
                   const View3D &view3d,
                   const Object &eval_object,
                   const bke::greasepencil::Layer &layer);
  ~DrawingPlacement();

 public:
  bool use_project_to_surface() const;
  bool use_project_to_nearest_stroke() const;

  void cache_viewport_depths(Depsgraph *depsgraph, ARegion *region, View3D *view3d);
  void set_origin_to_nearest_stroke(float2 co);

  /**
   * Projects a screen space coordinate to the local drawing space.
   */
  float3 project(float2 co) const;
  void project(Span<float2> src, MutableSpan<float3> dst) const;
};

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

/**
 * Check for an active keyframe at the current scene time. When there is not,
 * create one when auto-key is on (taking additive drawing setting into account).
 * \return false when no keyframe could be found or created.
 */
bool ensure_active_keyframe(const Scene &scene, GreasePencil &grease_pencil);

void create_keyframe_edit_data_selected_frames_list(KeyframeEditData *ked,
                                                    const bke::greasepencil::Layer &layer);

bool active_grease_pencil_poll(bContext *C);
bool editable_grease_pencil_poll(bContext *C);
bool active_grease_pencil_layer_poll(bContext *C);
bool editable_grease_pencil_point_selection_poll(bContext *C);
bool grease_pencil_painting_poll(bContext *C);
bool grease_pencil_painting_fill_poll(bContext *C);

struct DrawingInfo {
  const bke::greasepencil::Drawing &drawing;
  const int layer_index;
  const int frame_number;
};
struct MutableDrawingInfo {
  bke::greasepencil::Drawing &drawing;
  const int layer_index;
  const int frame_number;
  const float multi_frame_falloff;
};
Array<int> get_frame_numbers_for_layer(const bke::greasepencil::Layer &layer,
                                       const int current_frame,
                                       const bool use_multi_frame_editing);
Vector<MutableDrawingInfo> retrieve_editable_drawings(const Scene &scene,
                                                      GreasePencil &grease_pencil);
Vector<MutableDrawingInfo> retrieve_editable_drawings_with_falloff(const Scene &scene,
                                                                   GreasePencil &grease_pencil);
Vector<MutableDrawingInfo> retrieve_editable_drawings_from_layer(
    const Scene &scene, GreasePencil &grease_pencil, const bke::greasepencil::Layer &layer);
Vector<DrawingInfo> retrieve_visible_drawings(const Scene &scene,
                                              const GreasePencil &grease_pencil);

IndexMask retrieve_editable_strokes(Object &grease_pencil_object,
                                    const bke::greasepencil::Drawing &drawing,
                                    IndexMaskMemory &memory);
IndexMask retrieve_editable_strokes_by_material(Object &object,
                                                const bke::greasepencil::Drawing &drawing,
                                                const int mat_i,
                                                IndexMaskMemory &memory);
IndexMask retrieve_editable_points(Object &object,
                                   const bke::greasepencil::Drawing &drawing,
                                   IndexMaskMemory &memory);
IndexMask retrieve_editable_elements(Object &object,
                                     const bke::greasepencil::Drawing &drawing,
                                     bke::AttrDomain selection_domain,
                                     IndexMaskMemory &memory);

IndexMask retrieve_visible_strokes(Object &grease_pencil_object,
                                   const bke::greasepencil::Drawing &drawing,
                                   IndexMaskMemory &memory);

IndexMask retrieve_editable_and_selected_strokes(Object &grease_pencil_object,
                                                 const bke::greasepencil::Drawing &drawing,
                                                 IndexMaskMemory &memory);
IndexMask retrieve_editable_and_selected_points(Object &object,
                                                const bke::greasepencil::Drawing &drawing,
                                                IndexMaskMemory &memory);
IndexMask retrieve_editable_and_selected_elements(Object &object,
                                                  const bke::greasepencil::Drawing &drawing,
                                                  bke::AttrDomain selection_domain,
                                                  IndexMaskMemory &memory);

void create_blank(Main &bmain, Object &object, int frame_number);
void create_stroke(Main &bmain, Object &object, const float4x4 &matrix, int frame_number);
void create_suzanne(Main &bmain, Object &object, const float4x4 &matrix, int frame_number);

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
Curves2DSpace curves_in_2d_space_get(Object &object,
                                     const ViewContext &vc,
                                     const GreasePencil &grease_pencil,
                                     const Span<const bke::greasepencil::Drawing *> &drawings,
                                     const Span<int> &layer_index,
                                     const int frame_number,
                                     const bool get_stroke_flag = false);

}  // namespace blender::ed::greasepencil
