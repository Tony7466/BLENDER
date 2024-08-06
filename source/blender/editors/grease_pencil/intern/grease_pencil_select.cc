/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgreasepencil
 */

#include "BKE_curves.hh"

#include "BKE_attribute.hh"
#include "BKE_context.hh"
#include "BKE_grease_pencil.hh"

#include "DNA_object_types.h"

#include "DEG_depsgraph.hh"

#include "ED_curves.hh"
#include "ED_grease_pencil.hh"
#include "ED_screen.hh"
#include "ED_view3d.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"
#include "RNA_enum_types.hh"

#include "WM_api.hh"

#include <iostream>

namespace blender::ed::greasepencil {

inline int wrap_range(const IndexRange range, const int index)
{
  return (index - range.start()) % range.size() + range.start();
}

bool update_segment_selection(bke::CurvesGeometry &curves,
                              const IndexMask &changed_point_mask,
                              const Curves2DBVHTree &tree_data,
                              const IndexRange tree_data_range,
                              const eSelectOp sel_op)
{

  if (changed_point_mask.is_empty()) {
    return false;
  }

  IndexMaskMemory memory;
  const IndexMask changed_curve_mask = ed::curves::curve_mask_from_points(
      curves, changed_point_mask, GrainSize(512), memory);

  std::cout << "Changed points: ";
  changed_point_mask.foreach_index([&](const int index) { std::cout << index << ", "; });
  std::cout << std::endl;
  std::cout << "Changed curves: ";
  changed_curve_mask.foreach_index([&](const int index) { std::cout << index << ", "; });
  std::cout << std::endl;

  const OffsetIndices points_by_curve = curves.points_by_curve();
  const Span<float2> screen_space_positions = tree_data.start_positions.as_span().slice(
      tree_data_range);

  const CurveSegmentsData segment_data = ed::greasepencil::find_curve_segments(
      curves, changed_curve_mask, screen_space_positions, tree_data);

  const OffsetIndices<int> segments_by_curve = OffsetIndices<int>(segment_data.segment_offsets);
  Vector<bke::GSpanAttributeWriter> attribute_writers;
  const eCustomDataType create_type = CD_PROP_BOOL;
  const Span<StringRef> selection_attribute_names =
      ed::curves::get_curves_selection_attribute_names(curves);
  for (const int i : selection_attribute_names.index_range()) {
    attribute_writers.append(ed::curves::ensure_selection_attribute(
        curves, bke::AttrDomain::Point, create_type, selection_attribute_names[i]));
  };

  /* Find all segments that have changed points and fill them. */
  Array<bool> changed_points(curves.points_num());
  changed_point_mask.to_bools(changed_points);

  {
    std::cout << "Curves: " << std::endl;
    for (const int curve_i : segments_by_curve.index_range()) {
      const IndexRange points = points_by_curve[curve_i];
      const IndexRange segments = segments_by_curve[curve_i];
      const Span<int> curve_segment_points = segment_data.segment_start_points.as_span().slice(
          segments);
      const Span<float> curve_segment_fractions =
          segment_data.segment_start_fractions.as_span().slice(segments);
      /* Wrap the last segment around if the first segment does not match the curve start. */
      const bool wrap_last_segment = (!curve_segment_points.is_empty() &&
                                      curve_segment_points.first() == points.first() &&
                                      curve_segment_fractions.first() == 0.0f);

      std::cout << "  Curve " << curve_i << " segments ";
      if (segments.is_empty()) {
        std::cout << "empty" << std::endl;
      }
      else {
        std::cout << segments << std::endl;
      }

      for (const int segment_i : segments) {
        const int next_segment_i = wrap_range(segments, segment_i + 1);

        const int segment_point_i = segment_data.segment_start_points[segment_i];
        const float segment_fraction = segment_data.segment_start_fractions[segment_i];
        const int next_segment_point_i = segment_data.segment_start_points[next_segment_i];
        const float next_segment_fraction = segment_data.segment_start_fractions[next_segment_i];
        const bool wrap_segment = (wrap_last_segment && segment_i == segments.last());

        /* Start point with zero fraction is included. */
        const int first_included_point_i = (segment_fraction == 0.0f ?
                                                segment_point_i :
                                                wrap_range(points, segment_point_i + 1));
        /* End point with zero fraction is excluded. */
        const int last_included_point_i = (next_segment_fraction == 0.0f ?
                                               next_segment_point_i :
                                               wrap_range(points, next_segment_point_i - 1));

        std::cout << "  Segment " << segment_i << " start point "
                  << segment_data.segment_start_points[segment_i] << " fraction "
                  << segment_data.segment_start_fractions[segment_i];
        if (wrap_segment) {
          const IndexRange points_range1 = IndexRange::from_begin_end_inclusive(
              points.first(), last_included_point_i);
          const IndexRange points_range2 = IndexRange::from_begin_end_inclusive(
              first_included_point_i, points.last());
          std::cout << " (wrapped) " << points_range1 << " " << points_range2;
        }
        else {
          const IndexRange points_range = IndexRange::from_begin_end_inclusive(
              first_included_point_i, last_included_point_i);
          std::cout << " " << points_range;
        }
        std::cout << std::endl;
      }
    }
  }

  threading::parallel_for(segments_by_curve.index_range(), 256, [&](const IndexRange range) {
    for (const int curve_i : range) {
      const IndexRange points = points_by_curve[curve_i];
      const IndexRange segments = segments_by_curve[curve_i];
      const Span<int> curve_segment_points = segment_data.segment_start_points.as_span().slice(
          segments);
      const Span<float> curve_segment_fractions =
          segment_data.segment_start_fractions.as_span().slice(segments);
      /* Wrap the last segment around if the first segment does not match the curve start. */
      const bool wrap_last_segment = (!curve_segment_points.is_empty() &&
                                      curve_segment_points.first() == points.first() &&
                                      curve_segment_fractions.first() == 0.0f);

      for (const int segment_i : segments) {
        const int next_segment_i = wrap_range(segments, segment_i + 1);

        const int segment_point_i = segment_data.segment_start_points[segment_i];
        const float segment_fraction = segment_data.segment_start_fractions[segment_i];
        const int next_segment_point_i = segment_data.segment_start_points[next_segment_i];
        const float next_segment_fraction = segment_data.segment_start_fractions[next_segment_i];
        const bool wrap_segment = (wrap_last_segment && segment_i == segments.last());

        /* Start point with zero fraction is included. */
        const int first_included_point_i = (segment_fraction == 0.0f ?
                                                segment_point_i :
                                                wrap_range(points, segment_point_i + 1));
        /* End point with zero fraction is excluded. */
        const int last_included_point_i = (next_segment_fraction == 0.0f ?
                                               next_segment_point_i :
                                               wrap_range(points, next_segment_point_i - 1));

        auto test_points_range = [&](const IndexRange range) -> bool {
          for (const int point_i : range) {
            if (changed_points[point_i]) {
              return true;
            }
          }
          return false;
        };
        auto update_points_range = [&](const IndexRange range) {
          for (auto &attribute_writer : attribute_writers) {
            for (const int point_i : range) {
              ed::curves::apply_selection_operation_at_index(
                  attribute_writer.span, point_i, sel_op);
            }
          }
        };

        /* In case the segment wraps around the curve start there will be two ranges to check,
         * otherwise the second range is empty. */
        if (wrap_segment) {
          const IndexRange points_range1 = IndexRange::from_begin_end_inclusive(
              points.first(), last_included_point_i);
          const IndexRange points_range2 = IndexRange::from_begin_end_inclusive(
              first_included_point_i, points.last());
          if (test_points_range(points_range1) || test_points_range(points_range2)) {
            update_points_range(points_range1);
            update_points_range(points_range2);
          }
        }
        else {
          const IndexRange points_range = IndexRange::from_begin_end_inclusive(
              first_included_point_i, last_included_point_i);
          if (test_points_range(points_range)) {
            update_points_range(points_range);
          }
        }
      }
    }
  });

  for (auto &attribute_writer : attribute_writers) {
    attribute_writer.finish();
  }

  return true;
}

static int select_all_exec(bContext *C, wmOperator *op)
{
  int action = RNA_enum_get(op->ptr, "action");
  Scene *scene = CTX_data_scene(C);
  Object *object = CTX_data_active_object(C);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object->data);
  bke::AttrDomain selection_domain = ED_grease_pencil_selection_domain_get(scene->toolsettings);

  const Vector<MutableDrawingInfo> drawings = retrieve_editable_drawings(*scene, grease_pencil);
  threading::parallel_for_each(drawings, [&](const MutableDrawingInfo &info) {
    IndexMaskMemory memory;
    const IndexMask selectable_elements = retrieve_editable_elements(
        *object, info, selection_domain, memory);
    if (selectable_elements.is_empty()) {
      return;
    }
    blender::ed::curves::select_all(
        info.drawing.strokes_for_write(), selectable_elements, selection_domain, action);
  });

  /* Use #ID_RECALC_GEOMETRY instead of #ID_RECALC_SELECT because it is handled as a generic
   * attribute for now. */
  DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_GEOM | ND_DATA, &grease_pencil);

  return OPERATOR_FINISHED;
}

static void GREASE_PENCIL_OT_select_all(wmOperatorType *ot)
{
  ot->name = "(De)select All Strokes";
  ot->idname = "GREASE_PENCIL_OT_select_all";
  ot->description = "(De)select all visible strokes";

  ot->exec = select_all_exec;
  ot->poll = editable_grease_pencil_poll;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  WM_operator_properties_select_all(ot);
}

static int select_more_exec(bContext *C, wmOperator * /*op*/)
{
  Scene *scene = CTX_data_scene(C);
  Object *object = CTX_data_active_object(C);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object->data);

  const Vector<MutableDrawingInfo> drawings = retrieve_editable_drawings(*scene, grease_pencil);
  threading::parallel_for_each(drawings, [&](const MutableDrawingInfo &info) {
    IndexMaskMemory memory;
    const IndexMask selectable_strokes = ed::greasepencil::retrieve_editable_strokes(
        *object, info.drawing, info.layer_index, memory);
    if (selectable_strokes.is_empty()) {
      return;
    }
    blender::ed::curves::select_adjacent(
        info.drawing.strokes_for_write(), selectable_strokes, false);
  });

  /* Use #ID_RECALC_GEOMETRY instead of #ID_RECALC_SELECT because it is handled as a generic
   * attribute for now. */
  DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_GEOM | ND_DATA, &grease_pencil);

  return OPERATOR_FINISHED;
}

static void GREASE_PENCIL_OT_select_more(wmOperatorType *ot)
{
  ot->name = "Select More";
  ot->idname = "GREASE_PENCIL_OT_select_more";
  ot->description = "Grow the selection by one point";

  ot->exec = select_more_exec;
  ot->poll = editable_grease_pencil_point_selection_poll;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

static int select_less_exec(bContext *C, wmOperator * /*op*/)
{
  Scene *scene = CTX_data_scene(C);
  Object *object = CTX_data_active_object(C);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object->data);

  const Vector<MutableDrawingInfo> drawings = retrieve_editable_drawings(*scene, grease_pencil);
  threading::parallel_for_each(drawings, [&](const MutableDrawingInfo &info) {
    IndexMaskMemory memory;
    const IndexMask selectable_strokes = ed::greasepencil::retrieve_editable_strokes(
        *object, info.drawing, info.layer_index, memory);
    if (selectable_strokes.is_empty()) {
      return;
    }
    blender::ed::curves::select_adjacent(
        info.drawing.strokes_for_write(), selectable_strokes, true);
  });

  /* Use #ID_RECALC_GEOMETRY instead of #ID_RECALC_SELECT because it is handled as a generic
   * attribute for now. */
  DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_GEOM | ND_DATA, &grease_pencil);

  return OPERATOR_FINISHED;
}

static void GREASE_PENCIL_OT_select_less(wmOperatorType *ot)
{
  ot->name = "Select Less";
  ot->idname = "GREASE_PENCIL_OT_select_less";
  ot->description = "Shrink the selection by one point";

  ot->exec = select_less_exec;
  ot->poll = editable_grease_pencil_point_selection_poll;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

static int select_linked_exec(bContext *C, wmOperator * /*op*/)
{
  Scene *scene = CTX_data_scene(C);
  Object *object = CTX_data_active_object(C);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object->data);

  const Vector<MutableDrawingInfo> drawings = retrieve_editable_drawings(*scene, grease_pencil);
  threading::parallel_for_each(drawings, [&](const MutableDrawingInfo &info) {
    IndexMaskMemory memory;
    const IndexMask selectable_strokes = ed::greasepencil::retrieve_editable_strokes(
        *object, info.drawing, info.layer_index, memory);
    if (selectable_strokes.is_empty()) {
      return;
    }
    blender::ed::curves::select_linked(info.drawing.strokes_for_write(), selectable_strokes);
  });

  /* Use #ID_RECALC_GEOMETRY instead of #ID_RECALC_SELECT because it is handled as a generic
   * attribute for now. */
  DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_GEOM | ND_DATA, &grease_pencil);

  return OPERATOR_FINISHED;
}

static void GREASE_PENCIL_OT_select_linked(wmOperatorType *ot)
{
  ot->name = "Select Linked";
  ot->idname = "GREASE_PENCIL_OT_select_linked";
  ot->description = "Select all points in curves with any point selection";

  ot->exec = select_linked_exec;
  ot->poll = editable_grease_pencil_point_selection_poll;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

static int select_random_exec(bContext *C, wmOperator *op)
{
  using namespace blender;
  const float ratio = RNA_float_get(op->ptr, "ratio");
  const int seed = WM_operator_properties_select_random_seed_increment_get(op);
  Scene *scene = CTX_data_scene(C);
  Object *object = CTX_data_active_object(C);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object->data);
  bke::AttrDomain selection_domain = ED_grease_pencil_selection_domain_get(scene->toolsettings);

  const Vector<MutableDrawingInfo> drawings = retrieve_editable_drawings(*scene, grease_pencil);
  threading::parallel_for_each(drawings, [&](const MutableDrawingInfo &info) {
    bke::CurvesGeometry &curves = info.drawing.strokes_for_write();

    IndexMaskMemory memory;
    const IndexMask selectable_elements = retrieve_editable_elements(
        *object, info, selection_domain, memory);
    if (selectable_elements.is_empty()) {
      return;
    }

    const IndexMask random_elements = ed::curves::random_mask(
        curves,
        selectable_elements,
        selection_domain,
        blender::get_default_hash<int>(seed, info.layer_index),
        ratio,
        memory);

    const bool was_anything_selected = ed::curves::has_anything_selected(curves,
                                                                         selectable_elements);
    bke::GSpanAttributeWriter selection = ed::curves::ensure_selection_attribute(
        curves, selection_domain, CD_PROP_BOOL);
    if (!was_anything_selected) {
      curves::fill_selection_true(selection.span, selectable_elements);
    }

    curves::fill_selection_false(selection.span, random_elements);
    selection.finish();
  });

  /* Use #ID_RECALC_GEOMETRY instead of #ID_RECALC_SELECT because it is handled as a generic
   * attribute for now. */
  DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_GEOM | ND_DATA, &grease_pencil);

  return OPERATOR_FINISHED;
}

static void GREASE_PENCIL_OT_select_random(wmOperatorType *ot)
{
  ot->name = "Select Random";
  ot->idname = "GREASE_PENCIL_OT_select_random";
  ot->description = "Selects random points from the current strokes selection";

  ot->exec = select_random_exec;
  ot->poll = editable_grease_pencil_poll;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  WM_operator_properties_select_random(ot);
}

static int select_alternate_exec(bContext *C, wmOperator *op)
{
  const bool deselect_ends = RNA_boolean_get(op->ptr, "deselect_ends");
  Scene *scene = CTX_data_scene(C);
  Object *object = CTX_data_active_object(C);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object->data);

  const Vector<MutableDrawingInfo> drawings = retrieve_editable_drawings(*scene, grease_pencil);
  threading::parallel_for_each(drawings, [&](const MutableDrawingInfo &info) {
    blender::ed::curves::select_alternate(info.drawing.strokes_for_write(), deselect_ends);
  });

  /* Use #ID_RECALC_GEOMETRY instead of #ID_RECALC_SELECT because it is handled as a generic
   * attribute for now. */
  DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_GEOM | ND_DATA, &grease_pencil);

  return OPERATOR_FINISHED;
}

static void GREASE_PENCIL_OT_select_alternate(wmOperatorType *ot)
{
  ot->name = "Select Alternate";
  ot->idname = "GREASE_PENCIL_OT_select_alternate";
  ot->description = "Select alternated points in strokes with already selected points";

  ot->exec = select_alternate_exec;
  ot->poll = editable_grease_pencil_point_selection_poll;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  RNA_def_boolean(ot->srna,
                  "deselect_ends",
                  false,
                  "Deselect Ends",
                  "(De)select the first and last point of each stroke");
}

static int select_ends_exec(bContext *C, wmOperator *op)
{
  const int amount_start = RNA_int_get(op->ptr, "amount_start");
  const int amount_end = RNA_int_get(op->ptr, "amount_end");
  Scene *scene = CTX_data_scene(C);
  Object *object = CTX_data_active_object(C);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object->data);

  const Vector<MutableDrawingInfo> drawings = retrieve_editable_drawings(*scene, grease_pencil);
  threading::parallel_for_each(drawings, [&](const MutableDrawingInfo &info) {
    bke::CurvesGeometry &curves = info.drawing.strokes_for_write();

    IndexMaskMemory memory;
    const IndexMask selectable_strokes = ed::greasepencil::retrieve_editable_strokes(
        *object, info.drawing, info.layer_index, memory);
    if (selectable_strokes.is_empty()) {
      return;
    }
    const IndexMask inverted_end_points_mask = ed::curves::end_points(
        curves, selectable_strokes, amount_start, amount_end, true, memory);

    const IndexMask selectable_points = ed::greasepencil::retrieve_editable_points(
        *object, info.drawing, info.layer_index, memory);
    const bool was_anything_selected = ed::curves::has_anything_selected(curves,
                                                                         selectable_points);
    bke::GSpanAttributeWriter selection = ed::curves::ensure_selection_attribute(
        curves, bke::AttrDomain::Point, CD_PROP_BOOL);
    if (!was_anything_selected) {
      ed::curves::fill_selection_true(selection.span, selectable_points);
    }

    if (selection.span.type().is<bool>()) {
      index_mask::masked_fill(selection.span.typed<bool>(), false, inverted_end_points_mask);
    }
    if (selection.span.type().is<float>()) {
      index_mask::masked_fill(selection.span.typed<float>(), 0.0f, inverted_end_points_mask);
    }
    selection.finish();
  });

  /* Use #ID_RECALC_GEOMETRY instead of #ID_RECALC_SELECT because it is handled as a generic
   * attribute for now. */
  DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_GEOM | ND_DATA, &grease_pencil);

  return OPERATOR_FINISHED;
}

static void GREASE_PENCIL_OT_select_ends(wmOperatorType *ot)
{
  ot->name = "Select Ends";
  ot->idname = "GREASE_PENCIL_OT_select_ends";
  ot->description = "Select end points of strokes";

  ot->exec = select_ends_exec;
  ot->poll = editable_grease_pencil_point_selection_poll;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  RNA_def_int(ot->srna,
              "amount_start",
              0,
              0,
              INT32_MAX,
              "Amount Start",
              "Number of points to select from the start",
              0,
              INT32_MAX);
  RNA_def_int(ot->srna,
              "amount_end",
              1,
              0,
              INT32_MAX,
              "Amount End",
              "Number of points to select from the end",
              0,
              INT32_MAX);
}

static int select_set_mode_exec(bContext *C, wmOperator *op)
{
  using namespace blender::bke::greasepencil;

  /* Set new selection mode. */
  const int mode_new = RNA_enum_get(op->ptr, "mode");
  ToolSettings *ts = CTX_data_tool_settings(C);

  bool changed = (mode_new != ts->gpencil_selectmode_edit);
  ts->gpencil_selectmode_edit = mode_new;

  /* Convert all drawings of the active GP to the new selection domain. */
  const bke::AttrDomain domain = ED_grease_pencil_selection_domain_get(ts);
  Object *object = CTX_data_active_object(C);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object->data);
  Span<GreasePencilDrawingBase *> drawings = grease_pencil.drawings();

  for (const int index : drawings.index_range()) {
    GreasePencilDrawingBase *drawing_base = drawings[index];
    if (drawing_base->type != GP_DRAWING) {
      continue;
    }

    GreasePencilDrawing *drawing = reinterpret_cast<GreasePencilDrawing *>(drawing_base);
    bke::CurvesGeometry &curves = drawing->wrap().strokes_for_write();
    if (curves.points_num() == 0) {
      continue;
    }

    /* Skip curve when the selection domain already matches, or when there is no selection
     * at all. */
    bke::MutableAttributeAccessor attributes = curves.attributes_for_write();
    const std::optional<bke::AttributeMetaData> meta_data = attributes.lookup_meta_data(
        ".selection");
    if ((!meta_data) || (meta_data->domain == domain)) {
      continue;
    }

    /* When the new selection domain is 'curve', ensure all curves with a point selection
     * are selected. */
    if (domain == bke::AttrDomain::Curve) {
      blender::ed::curves::select_linked(curves);
    }

    /* Convert selection domain. */
    const GVArray src = *attributes.lookup(".selection", domain);
    if (src) {
      const CPPType &type = src.type();
      void *dst = MEM_malloc_arrayN(attributes.domain_size(domain), type.size(), __func__);
      src.materialize(dst);

      attributes.remove(".selection");
      if (!attributes.add(".selection",
                          domain,
                          bke::cpp_type_to_custom_data_type(type),
                          bke::AttributeInitMoveArray(dst)))
      {
        MEM_freeN(dst);
      }

      changed = true;

      /* TODO: expand point selection to segments when in 'segment' mode. */
    }
  }

  if (changed) {
    /* Use #ID_RECALC_GEOMETRY instead of #ID_RECALC_SELECT because it is handled as a generic
     * attribute for now. */
    DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY);
    WM_event_add_notifier(C, NC_GEOM | ND_DATA, &grease_pencil);

    WM_main_add_notifier(NC_SPACE | ND_SPACE_VIEW3D, nullptr);
  }

  return OPERATOR_FINISHED;
}

static void GREASE_PENCIL_OT_set_selection_mode(wmOperatorType *ot)
{
  PropertyRNA *prop;

  ot->name = "Select Mode";
  ot->idname = __func__;
  ot->description = "Change the selection mode for Grease Pencil strokes";

  ot->exec = select_set_mode_exec;
  ot->poll = editable_grease_pencil_poll;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  ot->prop = prop = RNA_def_enum(
      ot->srna, "mode", rna_enum_grease_pencil_selectmode_items, 0, "Mode", "");
  RNA_def_property_flag(prop, (PropertyFlag)(PROP_HIDDEN | PROP_SKIP_SAVE));
}

static int grease_pencil_material_select_exec(bContext *C, wmOperator *op)
{
  const Scene *scene = CTX_data_scene(C);
  Object *object = CTX_data_active_object(C);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object->data);
  const bool select = !RNA_boolean_get(op->ptr, "deselect");
  const int material_index = object->actcol - 1;

  if (material_index == -1) {
    return OPERATOR_CANCELLED;
  }

  const Vector<MutableDrawingInfo> drawings = retrieve_editable_drawings(*scene, grease_pencil);
  threading::parallel_for_each(drawings, [&](const MutableDrawingInfo &info) {
    bke::CurvesGeometry &curves = info.drawing.strokes_for_write();

    IndexMaskMemory memory;
    const IndexMask strokes = retrieve_editable_strokes_by_material(
        *object, info.drawing, material_index, memory);
    if (strokes.is_empty()) {
      return;
    }
    bke::GSpanAttributeWriter selection = ed::curves::ensure_selection_attribute(
        curves, bke::AttrDomain::Curve, CD_PROP_BOOL);
    index_mask::masked_fill(selection.span.typed<bool>(), select, strokes);
    selection.finish();
  });

  DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_GEOM | ND_DATA | NA_EDITED, &grease_pencil);

  return OPERATOR_FINISHED;
}

static void GREASE_PENCIL_OT_material_select(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Select Material";
  ot->idname = "GREASE_PENCIL_OT_material_select";
  ot->description = "Select/Deselect all Grease Pencil strokes using current material";

  /* callbacks. */
  ot->exec = grease_pencil_material_select_exec;
  ot->poll = editable_grease_pencil_poll;

  /* flags. */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  /* props */
  ot->prop = RNA_def_boolean(ot->srna, "deselect", false, "Deselect", "Unselect strokes");
  RNA_def_property_flag(ot->prop, PROP_HIDDEN | PROP_SKIP_SAVE);
}

}  // namespace blender::ed::greasepencil

blender::bke::AttrDomain ED_grease_pencil_selection_domain_get(const ToolSettings *tool_settings)
{
  switch (tool_settings->gpencil_selectmode_edit) {
    case GP_SELECTMODE_POINT:
      return blender::bke::AttrDomain::Point;
    case GP_SELECTMODE_STROKE:
      return blender::bke::AttrDomain::Curve;
    case GP_SELECTMODE_SEGMENT:
      return blender::bke::AttrDomain::Point;
  }
  return blender::bke::AttrDomain::Point;
}

bool ED_grease_pencil_segment_selection_enabled(const ToolSettings *tool_settings)
{
  return tool_settings->gpencil_selectmode_edit == GP_SELECTMODE_SEGMENT;
}

void ED_operatortypes_grease_pencil_select()
{
  using namespace blender::ed::greasepencil;
  WM_operatortype_append(GREASE_PENCIL_OT_select_all);
  WM_operatortype_append(GREASE_PENCIL_OT_select_more);
  WM_operatortype_append(GREASE_PENCIL_OT_select_less);
  WM_operatortype_append(GREASE_PENCIL_OT_select_linked);
  WM_operatortype_append(GREASE_PENCIL_OT_select_random);
  WM_operatortype_append(GREASE_PENCIL_OT_select_alternate);
  WM_operatortype_append(GREASE_PENCIL_OT_select_ends);
  WM_operatortype_append(GREASE_PENCIL_OT_set_selection_mode);
  WM_operatortype_append(GREASE_PENCIL_OT_material_select);
}
