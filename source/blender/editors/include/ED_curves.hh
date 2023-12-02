/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup editors
 */

#pragma once

#include "DNA_grease_pencil_types.h"
#include "DNA_scene_types.h"

#include "BKE_attribute.hh"
#include "BKE_crazyspace.hh"
#include "BKE_curves.hh"
#include "BKE_grease_pencil.hh"

#include "BLI_math_color.h"

#include "BLI_index_mask.hh"
#include "BLI_vector.hh"
#include "BLI_vector_set.hh"

#include "ED_select_utils.hh"
#include "ED_grease_pencil.hh"

struct bContext;
struct Curves;
struct UndoType;
struct SelectPick_Params;
struct ViewContext;
struct rcti;
struct TransVertStore;
struct wmKeyConfig;

/* -------------------------------------------------------------------- */
/** \name C Wrappers
 * \{ */

void ED_operatortypes_curves();
void ED_curves_undosys_type(UndoType *ut);
void ED_keymap_curves(wmKeyConfig *keyconf);

/**
 * Return an owning pointer to an array of point normals the same size as the number of control
 * points. The normals depend on the normal mode for each curve and the "tilt" attribute and may be
 * calculated for the evaluated points and sampled back to the control points.
 */
float (*ED_curves_point_normals_array_create(const Curves *curves_id))[3];

/** \} */

namespace blender::ed::curves {

bool object_has_editable_curves(const Main &bmain, const Object &object);
bke::CurvesGeometry primitive_random_sphere(int curves_size, int points_per_curve);
VectorSet<Curves *> get_unique_editable_curves(const bContext &C);
void ensure_surface_deformation_node_exists(bContext &C, Object &curves_ob);

/**
 * Allocate an array of `TransVert` for cursor/selection snapping (See
 * `ED_transverts_create_from_obedit` in `view3d_snap.cc`).
 * \note: the `TransVert` elements in \a tvs are expected to write to the positions of \a curves.
 */
void transverts_from_curves_positions_create(bke::CurvesGeometry &curves, TransVertStore *tvs);

/* -------------------------------------------------------------------- */
/** \name Poll Functions
 * \{ */

bool editable_curves_with_surface_poll(bContext *C);
bool editable_curves_in_edit_mode_poll(bContext *C);
bool curves_with_surface_poll(bContext *C);
bool editable_curves_poll(bContext *C);
bool curves_poll(bContext *C);

/** \} */

/* -------------------------------------------------------------------- */
/** \name Operators
 * \{ */

void CURVES_OT_attribute_set(wmOperatorType *ot);

/** \} */

/* -------------------------------------------------------------------- */
/** \name Mask Functions
 * \{ */

/**
 * Return a mask of all the end points in the curves.
 * \param curves_mask (optional): The curves that should be used in the resulting point mask.
 * \param amount_start: The amount of points to mask from the front.
 * \param amount_end: The amount of points to mask from the back.
 * \param inverted: Invert the resulting mask.
 */
IndexMask end_points(const bke::CurvesGeometry &curves,
                     int amount_start,
                     int amount_end,
                     bool inverted,
                     IndexMaskMemory &memory);
IndexMask end_points(const bke::CurvesGeometry &curves,
                     const IndexMask &curves_mask,
                     int amount_start,
                     int amount_end,
                     bool inverted,
                     IndexMaskMemory &memory);

/**
 * Return a mask of random points or curves.
 *
 * \param mask (optional): The elements that should be used in the resulting mask. This mask should
 * be in the same domain as the \a selection_domain. \param random_seed: The seed for the \a
 * RandomNumberGenerator. \param probability: Determines how likely a point/curve will be chosen.
 * If set to 0.0, nothing will be in the mask, if set to 1.0 everything will be in the mask.
 */
IndexMask random_mask(const bke::CurvesGeometry &curves,
                      eAttrDomain selection_domain,
                      uint32_t random_seed,
                      float probability,
                      IndexMaskMemory &memory);
IndexMask random_mask(const bke::CurvesGeometry &curves,
                      const IndexMask &mask,
                      eAttrDomain selection_domain,
                      uint32_t random_seed,
                      float probability,
                      IndexMaskMemory &memory);

/** \} */

/* -------------------------------------------------------------------- */
/** \name Selection
 *
 * Selection on curves can be stored on either attribute domain: either per-curve or per-point. It
 * can be stored with a float or boolean data-type. The boolean data-type is faster, smaller, and
 * corresponds better to edit-mode selections, but the float data type is useful for soft selection
 * (like masking) in sculpt mode.
 *
 * The attribute API is used to do the necessary type and domain conversions when necessary, and
 * can handle most interaction with the selection attribute, but these functions implement some
 * helpful utilities on top of that.
 * \{ */

void fill_selection_false(GMutableSpan span);
void fill_selection_true(GMutableSpan span);
void fill_selection_false(GMutableSpan selection, const IndexMask &mask);
void fill_selection_true(GMutableSpan selection, const IndexMask &mask);

/**
 * Return true if any element is selected, on either domain with either type.
 */
bool has_anything_selected(const bke::CurvesGeometry &curves);
bool has_anything_selected(const bke::CurvesGeometry &curves, const IndexMask &mask);

/**
 * Return true if any element in the span is selected, on either domain with either type.
 */
bool has_anything_selected(GSpan selection);
bool has_anything_selected(const VArray<bool> &varray, IndexRange range_to_check);
bool has_anything_selected(const VArray<bool> &varray, const IndexMask &indices_to_check);

/**
 * Find curves that have any point selected (a selection factor greater than zero),
 * or curves that have their own selection factor greater than zero.
 */
IndexMask retrieve_selected_curves(const bke::CurvesGeometry &curves, IndexMaskMemory &memory);
IndexMask retrieve_selected_curves(const Curves &curves_id, IndexMaskMemory &memory);

/**
 * Find points that are selected (a selection factor greater than zero),
 * or points in curves with a selection factor greater than zero).
 */
IndexMask retrieve_selected_points(const bke::CurvesGeometry &curves, IndexMaskMemory &memory);
IndexMask retrieve_selected_points(const Curves &curves_id, IndexMaskMemory &memory);

/**
 * If the ".selection" attribute doesn't exist, create it with the requested type (bool or float).
 */
bke::GSpanAttributeWriter ensure_selection_attribute(bke::CurvesGeometry &curves,
                                                     eAttrDomain selection_domain,
                                                     eCustomDataType create_type);

/** Apply a change to a single curve or point. Avoid using this when affecting many elements. */
void apply_selection_operation_at_index(GMutableSpan selection, int index, eSelectOp sel_op);

/**
 * (De)select all the curves.
 *
 * \param mask (optional): The elements that should be affected. This mask should be in the domain
 * of the \a selection_domain.
 * \param action: One of SEL_TOGGLE, SEL_SELECT, SEL_DESELECT, or SEL_INVERT. See
 * "ED_select_utils.hh".
 */
void select_all(bke::CurvesGeometry &curves, eAttrDomain selection_domain, int action);
void select_all(bke::CurvesGeometry &curves,
                const IndexMask &mask,
                eAttrDomain selection_domain,
                int action);

/**
 * Select the points of all curves that have at least one point selected.
 *
 * \param curves_mask (optional): The curves that should be affected.
 */
void select_linked(bke::CurvesGeometry &curves);
void select_linked(bke::CurvesGeometry &curves, const IndexMask &curves_mask);

/**
 * Select alternated points in strokes with already selected points
 *
 * \param curves_mask (optional): The curves that should be affected.
 */
void select_alternate(bke::CurvesGeometry &curves, const bool deselect_ends);
void select_alternate(bke::CurvesGeometry &curves,
                      const IndexMask &curves_mask,
                      const bool deselect_ends);

template<typename T> static T default_for_lookup() {
  if constexpr (std::is_same<T, float>::value || std::is_same<T, int>::value) {
    return 0;
  }
  else if constexpr (std::is_same<T, ColorGeometry4f>::value) {
    return ColorGeometry4f{0.0f, 0.0f, 0.0f, 0.0f};
  }
  else if constexpr (std::is_same<T, std::string>::value) {
    return "default";
  }
  throw std::invalid_argument("Undefined behavior for distance function for the used type");
}

template<typename T>
blender::Set<T> selected_values_for_attribute_in_curve(bke::CurvesGeometry &curves,
                                                       int type,
                                                       std::string attribute_id)
{
  blender::Set<T> selectedValuesForAttribute;
  VArray<T> attributes = *curves.attributes().lookup_or_default<T>(
      attribute_id, ATTR_DOMAIN_POINT, default_for_lookup<T>());
  const OffsetIndices points_by_curve = curves.points_by_curve();
  bke::GSpanAttributeWriter selection = ensure_selection_attribute(
      curves, ATTR_DOMAIN_POINT, CD_PROP_BOOL);

  MutableSpan<bool> selection_typed = selection.span.typed<bool>();

  // for now sequential implementation, grain_size == 1
  threading::parallel_for(curves.curves_range(), 1, [&](const IndexRange range) {
    for (const int curve_i : range) {
      const IndexRange points = points_by_curve[curve_i];

      if (!has_anything_selected(selection.span.slice(points))) {
        continue;
      }

      for (const int index : points.index_range()) {
        if (selection_typed[points[index]]) {
          // careful: problems with concurrency?
          selectedValuesForAttribute.add(attributes[points[index]]);
        }
      }
    }
  });

  selection.finish();
  return selectedValuesForAttribute;
}

template<typename T>
static blender::Set<T> join_sets(blender::Vector<blender::Set<T>> &setsToBeJoined)
{
  Set<T> currentlySelectedValues{};
  for (auto &singleSet : setsToBeJoined) {
    for (auto &value : singleSet) {
      currentlySelectedValues.add(value);
    }
  }
  return currentlySelectedValues;
}

template<typename T> static float distance(T first, T second)
{
  if constexpr (std::is_same<T, float>::value || std::is_same<T, int>::value) {
    return std::abs(first - second);
  }
  else if constexpr (std::is_same<T, ColorGeometry4f>::value) {
    // might be better to normalize and then dot product
    return std::abs(int(rgb_to_grayscale(first)) - int(rgb_to_grayscale(second)));
  }
  throw std::invalid_argument("Undefined behavior for distance function for the used type");
}

template<typename T>
static void select_with_similar_attribute(bke::CurvesGeometry &curves,
                                          blender::Set<T> &set_active_if_similar_to,
                                          float threshold,
                                          int type,
                                          std::string attribute_id)
{
  VArray<T> attributes = *curves.attributes().lookup_or_default<T>(
      attribute_id, ATTR_DOMAIN_POINT, default_for_lookup<T>());
  const OffsetIndices points_by_curve = curves.points_by_curve();
  bke::GSpanAttributeWriter selection = ensure_selection_attribute(
      curves, ATTR_DOMAIN_POINT, CD_PROP_BOOL);

  MutableSpan<bool> selection_typed = selection.span.typed<bool>();
  // for now sequential impl, grain_size == 1
  threading::parallel_for(curves.curves_range(), 1, [&](const IndexRange range) {
    for (const int curve_i : range) {
      const IndexRange points = points_by_curve[curve_i];

      if (!has_anything_selected(selection.span.slice(points))) {
        continue;
      }

      for (const int index : points.index_range()) {
        for (auto &s : set_active_if_similar_to) {
          if (distance<T>(attributes[points[index]], s) <= threshold) {
            selection_typed[points[index]] = true;
          }
        }
      }
    }
  });

  selection.finish();
}

static void select_similar_layer(GreasePencil &grease_pencil,
                                 Scene *scene,
                                 eAttrDomain selection_domain,
                                 int type)
{
  blender::Vector<blender::Set<std::string>> currentlySelectedLayers(
      grease_pencil.drawings().size());
  currentlySelectedLayers.fill(blender::Set<std::string>{});

  grease_pencil.foreach_editable_drawing_in_layer_ex(
      scene->r.cfra,
      [&](int drawing_index,
          blender::bke::greasepencil::Drawing &drawing,
          const blender::bke::greasepencil::Layer *layer) {
        if (!has_anything_selected(drawing.strokes_for_write())) {
          return;
        }
        if constexpr (std::is_same<std::string, std::string>::value) {
          currentlySelectedLayers[drawing_index].add(std::string{layer->name().c_str()});
        }
      });

  Set<std::string> s = join_sets<std::string>(currentlySelectedLayers);

  grease_pencil.foreach_editable_drawing_in_layer_ex(
      scene->r.cfra,
      [&](int drawing_index,
          blender::bke::greasepencil::Drawing &drawing,
          const blender::bke::greasepencil::Layer *layer) {
        if (!s.contains(std::string{layer->name().c_str()})) {
          return;
        }
        select_all(drawing.strokes_for_write(), selection_domain, SEL_SELECT);
      });
}

template<typename T>
static void select_similar(GreasePencil &grease_pencil,
                           Scene *scene,
                           eAttrDomain selection_domain,
                           int type,
                           float threshold,
                           std::string attribute_id)
{
  using namespace blender::ed::greasepencil;
  blender::Vector<blender::Set<T>> currentlySelectedValuesPerDrawing;

  const Array<MutableDrawingInfo> drawings = retrieve_editable_drawings(*scene, grease_pencil);
  threading::parallel_for_each(drawings, [&](const MutableDrawingInfo &info) {
    currentlySelectedValuesPerDrawing.append(
            blender::ed::curves::selected_values_for_attribute_in_curve<T>(
                info.drawing.strokes_for_write(), type, attribute_id));
  });  

  Set<T> currentlySelectedValues = join_sets<T>(currentlySelectedValuesPerDrawing);

  threading::parallel_for_each(drawings, [&](const MutableDrawingInfo &info) {
    blender::ed::curves::select_with_similar_attribute<T>(info.drawing.strokes_for_write(),
                                                              currentlySelectedValues,
                                                              threshold,
                                                              type,
                                                              attribute_id);
  });  
}

/**
 * (De)select all the adjacent points of the current selected points.
 *
 * \param curves_mask (optional): The curves that should be affected.
 */
void select_adjacent(bke::CurvesGeometry &curves, bool deselect);
void select_adjacent(bke::CurvesGeometry &curves, const IndexMask &curves_mask, bool deselect);

/**
 * Helper struct for `closest_elem_find_screen_space`.
 */
struct FindClosestData {
  int index = -1;
  float distance = FLT_MAX;
};

/**
 * Find the closest screen-space point or curve in projected region-space.
 *
 * \return A new point or curve closer than the \a initial input, if one exists.
 */
std::optional<FindClosestData> closest_elem_find_screen_space(
    const ViewContext &vc,
    const Object &object,
    const OffsetIndices<int> points_by_curve,
    Span<float3> deformed_positions,
    const IndexMask &mask,
    eAttrDomain domain,
    int2 coord,
    const FindClosestData &initial);

/**
 * Select points or curves in a (screen-space) rectangle.
 */
bool select_box(const ViewContext &vc,
                bke::CurvesGeometry &curves,
                Span<float3> deformed_positions,
                const IndexMask &mask,
                eAttrDomain selection_domain,
                const rcti &rect,
                eSelectOp sel_op);

/**
 * Select points or curves in a (screen-space) poly shape.
 */
bool select_lasso(const ViewContext &vc,
                  bke::CurvesGeometry &curves,
                  Span<float3> deformed_positions,
                  const IndexMask &mask,
                  eAttrDomain selection_domain,
                  Span<int2> coords,
                  eSelectOp sel_op);

/**
 * Select points or curves in a (screen-space) circle.
 */
bool select_circle(const ViewContext &vc,
                   bke::CurvesGeometry &curves,
                   Span<float3> deformed_positions,
                   const IndexMask &mask,
                   eAttrDomain selection_domain,
                   int2 coord,
                   float radius,
                   eSelectOp sel_op);
/** \} */

/* -------------------------------------------------------------------- */
/** \name Editing
 * \{ */

/**
 * Remove (dissolve) selected curves or points based on the ".selection" attribute.
 * \returns true if any point or curve was removed.
 */
bool remove_selection(bke::CurvesGeometry &curves, eAttrDomain selection_domain);

/** \} */

}  // namespace blender::ed::curves
