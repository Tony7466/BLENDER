/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgreasepencil
 */

#include "BLI_rand.hh"
#include "BLI_vector_set.hh"

#include "BKE_context.h"
#include "BKE_grease_pencil.hh"

#include "DNA_object_types.h"

#include "DEG_depsgraph.h"

#include "ED_curves.h"
#include "ED_grease_pencil.h"
#include "ED_screen.h"
#include "ED_view3d.h"

#include "RNA_access.h"
#include "RNA_define.h"
#include "RNA_enum_types.h"

#include "WM_api.h"

namespace blender::ed::greasepencil {

static int select_all_exec(bContext *C, wmOperator *op)
{
  int action = RNA_enum_get(op->ptr, "action");
  Scene *scene = CTX_data_scene(C);
  Object *object = CTX_data_active_object(C);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object->data);
  eAttrDomain selection_domain = ED_grease_pencil_selection_domain_get(C);

  grease_pencil.foreach_editable_drawing(
      scene->r.cfra, [&](int /*drawing_index*/, blender::bke::greasepencil::Drawing &drawing) {
        blender::ed::curves::select_all(drawing.strokes_for_write(), selection_domain, action);
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

static int select_more_less_exec(bContext *C, wmOperator *op)
{
  Depsgraph *depsgraph = CTX_data_ensure_evaluated_depsgraph(C);
  ViewContext vc;
  Scene *scene = CTX_data_scene(C);
  Object *object = CTX_data_active_object(C);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object->data);

  const bool deselect = RNA_boolean_get(op->ptr, "deselect");

  ED_view3d_viewcontext_init(C, &vc, depsgraph);

  /* In segment mode, we expand the point selection to segments after the selection has changed.
   * For checking segment intersections, we use strokes converted to viewport 2D space. */
  const bool segment_mode = ED_grease_pencil_segment_selection_mode(C);
  int curve_offset = 0;
  Vector<ed::greasepencil::Stroke2DSpace> strokes_2d;
  if (segment_mode) {
    strokes_2d = ed::greasepencil::editable_strokes_in_2d_space_get(&vc, &grease_pencil);
  }

  grease_pencil.foreach_editable_drawing(
      scene->r.cfra, [&](int /*drawing_index*/, GreasePencilDrawing &drawing) {
        /* In segment mode, store the pre-change point selection. */
        Vector<bool> selection_before;
        if (segment_mode) {
          selection_before = ed::greasepencil::point_selection_get(&drawing);
        }

        blender::ed::curves::select_adjacent(drawing.strokes_for_write(), deselect);

        /* In segment mode, expand the changed point selection to segments. */
        if (segment_mode) {
          expand_changed_selection_to_segments(
              selection_before, drawing.geometry.wrap(), curve_offset, strokes_2d);
        }
        curve_offset += drawing.geometry.curve_num;
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

  ot->exec = select_more_less_exec;
  ot->poll = editable_grease_pencil_point_selection_poll;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  /* Set 'more' as operator property. */
  PropertyRNA *prop;
  prop = RNA_def_boolean(ot->srna, "deselect", false, "Select More", "");
  RNA_def_property_flag(prop, PROP_HIDDEN | PROP_SKIP_SAVE);
}

static void GREASE_PENCIL_OT_select_less(wmOperatorType *ot)
{
  ot->name = "Select Less";
  ot->idname = "GREASE_PENCIL_OT_select_less";
  ot->description = "Shrink the selection by one point";

  ot->exec = select_more_less_exec;
  ot->poll = editable_grease_pencil_point_selection_poll;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  /* Set 'less' as operator property. */
  PropertyRNA *prop;
  prop = RNA_def_boolean(ot->srna, "deselect", true, "Select Less", "");
  RNA_def_property_flag(prop, PROP_HIDDEN | PROP_SKIP_SAVE);
}

static int select_linked_exec(bContext *C, wmOperator * /*op*/)
{
  Scene *scene = CTX_data_scene(C);
  Object *object = CTX_data_active_object(C);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object->data);

  grease_pencil.foreach_editable_drawing(
      scene->r.cfra, [](int /*drawing_index*/, blender::bke::greasepencil::Drawing &drawing) {
        blender::ed::curves::select_linked(drawing.strokes_for_write());
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
  ViewContext vc;
  Depsgraph *depsgraph = CTX_data_ensure_evaluated_depsgraph(C);
  Scene *scene = CTX_data_scene(C);
  Object *object = CTX_data_active_object(C);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object->data);

  ED_view3d_viewcontext_init(C, &vc, depsgraph);

  /* Get selection domain from tool settings. */
  const eAttrDomain selection_domain = ED_grease_pencil_selection_domain_get(C);
  const bool segment_mode = ED_grease_pencil_segment_selection_mode(C);

  /* In segment mode, we expand a random point selection to segments.
   * For checking segment intersections, we use strokes converted to viewport 2D space. */
  int curve_offset = 0;
  Vector<ed::greasepencil::Stroke2DSpace> strokes_2d;
  if (segment_mode) {
    strokes_2d = ed::greasepencil::editable_strokes_in_2d_space_get(&vc, &grease_pencil);
  }

  grease_pencil.foreach_editable_drawing(
      scene->r.cfra, [&](int drawing_index, bke::greasepencil::Drawing &drawing) {
        bke::CurvesGeometry &curves = drawing.strokes_for_write();

        if (segment_mode) {
          /* Set random selection values for each segment on all curves. */
          expand_random_selection_to_segments(
            curves,
            curve_offset,
            strokes_2d,
            blender::get_default_hash_2<int>(seed, drawing_index),
            ratio);

          curve_offset += drawing.geometry.curve_num;
        else {
          IndexMaskMemory memory;
          const IndexMask random_elements = ed::curves::random_mask(
              curves,
              selection_domain,
              blender::get_default_hash_2<int>(seed, drawing_index),
              ratio,
              memory);

          const bool was_anything_selected = ed::curves::has_anything_selected(curves);
          bke::GSpanAttributeWriter selection = ed::curves::ensure_selection_attribute(
              curves, selection_domain, CD_PROP_BOOL);
          if (!was_anything_selected) {
            curves::fill_selection_true(selection.span);
          }

          curves::fill_selection_false(selection.span, random_elements);
          selection.finish();
        }
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

  grease_pencil.foreach_editable_drawing(
      scene->r.cfra, [&](int /*drawing_index*/, blender::bke::greasepencil::Drawing &drawing) {
        blender::ed::curves::select_alternate(drawing.strokes_for_write(), deselect_ends);
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
  Depsgraph *depsgraph = CTX_data_ensure_evaluated_depsgraph(C);
  ViewContext vc;
  Scene *scene = CTX_data_scene(C);
  Object *object = CTX_data_active_object(C);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object->data);

  ED_view3d_viewcontext_init(C, &vc, depsgraph);

  /* Get selection domain from tool settings. */
  const bool segment_mode = ED_grease_pencil_segment_selection_mode(C);

  /* In segment mode, we expand the point selection to segments after the selection has changed.
   * For checking segment intersections, we use strokes converted to viewport 2D space. */
  int curve_offset = 0;
  Vector<ed::greasepencil::Stroke2DSpace> strokes_2d;
  if (segment_mode) {
    strokes_2d = ed::greasepencil::editable_strokes_in_2d_space_get(&vc, &grease_pencil);
  }

  grease_pencil.foreach_editable_drawing(
      scene->r.cfra, [&](int /*drawing_index*/, blender::bke::greasepencil::Drawing &drawing) {
        bke::CurvesGeometry &curves = drawing.strokes_for_write();

        /* In segment mode, store the pre-change point selection. */
        Vector<bool> selection_before;
        if (segment_mode) {
          selection_before = ed::greasepencil::point_selection_get(&drawing);
        }

        IndexMaskMemory memory;
        const IndexMask inverted_end_points_mask = ed::curves::end_points(
            curves, amount_start, amount_end, true, memory);

        const bool was_anything_selected = ed::curves::has_anything_selected(curves);
        bke::GSpanAttributeWriter selection = ed::curves::ensure_selection_attribute(
            curves, ATTR_DOMAIN_POINT, CD_PROP_BOOL);
        if (!was_anything_selected) {
          ed::curves::fill_selection_true(selection.span);
        }

        if (selection.span.type().is<bool>()) {
          index_mask::masked_fill(selection.span.typed<bool>(), false, inverted_end_points_mask);
        }
        if (selection.span.type().is<float>()) {
          index_mask::masked_fill(selection.span.typed<float>(), 0.0f, inverted_end_points_mask);
        }
        selection.finish();

        /* In segment mode, expand the changed point selection to segments. */
        if (segment_mode) {
          expand_changed_selection_to_segments(
              selection_before, drawing.geometry.wrap(), curve_offset, strokes_2d);
        }
        curve_offset += drawing.geometry.curve_num;
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

Vector<bool> point_selection_get(const GreasePencilDrawing *drawing)
{
  /* Get point selection in the drawing. */
  bke::CurvesGeometry curves = drawing->geometry.wrap();
  bke::GSpanAttributeWriter selection = ed::curves::ensure_selection_attribute(
      curves, ATTR_DOMAIN_POINT, CD_PROP_BOOL);
  MutableSpan<bool> selection_typed = selection.span.typed<bool>();

  /* Copy selection. */
  Vector<bool> point_selection(selection.span.size());
  for (const int point_i : selection_typed.index_range()) {
    point_selection[point_i] = selection_typed[point_i];
  }

  selection.finish();

  return point_selection;
}

static int expand_changed_selection_point_to_segment(const int segment_stroke_index,
                                                     const int segment_point_start,
                                                     const IndexRange points,
                                                     MutableSpan<bool> &new_selection,
                                                     Vector<bool> &stored_selection,
                                                     const Vector<Stroke2DSpace> &strokes_2d,
                                                     const bool selection_state,
                                                     const bool is_cyclic)
{
  /* Get 2D stroke with segment to expand. */
  const Stroke2DSpace stroke_2d = strokes_2d[segment_stroke_index];
  const int index_offset = stroke_2d.first_index;

  /* Walk forward and backward along the curve from the point where
   * the selection has changed. We expand the changed selection
   * to the entire segment. */
  int point_abs;
  const int directions[2] = {-1, 1};
  for (const int direction : directions) {
    point_abs = segment_point_start;
    bool intersected = false;
    while ((direction == -1 && point_abs > points.first()) ||
           (direction == 1 && point_abs < points.last()))
    {
      int point_rel = point_abs - index_offset;

      /* The end of a segment is reached when the vector between the current and
       * next point intersects with an other stroke. */
      intersected = intersect_segment_strokes_2d(stroke_2d.points[point_rel],
                                                 stroke_2d.points[point_rel + direction],
                                                 segment_stroke_index,
                                                 strokes_2d);
      /* Set selection state. */
      new_selection[point_abs] = selection_state;
      stored_selection[point_abs] = selection_state;

      /* Intersection found, so stop expanding the segment. */
      if (intersected) {
        break;
      }

      point_abs += direction;

      /* Handle cyclic curves. */
      if (is_cyclic) {
        if (direction == -1 && point_abs == points.first()) {
          point_abs = points.last();
        }
        if (direction == 1 && point_abs == points.last()) {
          point_abs = points.first();
        }
      }
    }

    /* Handle last point. */
    if (!intersected) {
      new_selection[point_abs] = selection_state;
      stored_selection[point_abs] = selection_state;
    }
  }

  return point_abs;
}

void expand_changed_selection_to_segments(Vector<bool> &stored_selection,
                                          bke::CurvesGeometry &curves,
                                          const int curve_offset,
                                          const Vector<Stroke2DSpace> &strokes_2d)
{
  /* Compare the new point selection with the stored selection and expand the changed points to
   * segments.
   * Note: we can't use parallel threads here, because the new selection will change during
   * execution.
   */
  const OffsetIndices points_by_curve = curves.points_by_curve();
  const VArray<bool> cyclic = curves.cyclic();
  bke::GSpanAttributeWriter selection = ed::curves::ensure_selection_attribute(
      curves, ATTR_DOMAIN_POINT, CD_PROP_BOOL);
  MutableSpan<bool> new_selection = selection.span.typed<bool>();

  for (const int curve_i : curves.curves_range()) {
    const IndexRange points = points_by_curve[curve_i];

    for (int point_i = points.first(); point_i <= points.last(); point_i++) {
      /* Expand to segment when selection is changed. */
      if (stored_selection[point_i] != new_selection[point_i]) {
        expand_changed_selection_point_to_segment(curve_i + curve_offset,
                                                  point_i,
                                                  points,
                                                  new_selection,
                                                  stored_selection,
                                                  strokes_2d,
                                                  new_selection[point_i],
                                                  cyclic[curve_i]);
      }
    }
  }

  selection.finish();
}

void expand_random_selection_to_segments(bke::CurvesGeometry &curves,
                                         const int curve_offset,
                                         const Vector<Stroke2DSpace> &strokes_2d,
                                         const uint32_t random_seed,
                                         const float probability)
{
  /* Pick a random selection value and expand it to an entire segment.
   * Note: we can't use parallel threads here, because the new selection will change during
   * execution. */
  RandomNumberGenerator rng{random_seed};
  const auto next_bool_random_value = [&]() { return rng.get_float() <= probability; };

  const OffsetIndices points_by_curve = curves.points_by_curve();
  const VArray<bool> cyclic = curves.cyclic();
  bke::GSpanAttributeWriter selection = ed::curves::ensure_selection_attribute(
      curves, ATTR_DOMAIN_POINT, CD_PROP_BOOL);
  MutableSpan<bool> new_selection = selection.span.typed<bool>();
  Vector<bool> stored_selection = Vector<bool>(selection.span.size());

  for (const int curve_i : curves.curves_range()) {
    const IndexRange points = points_by_curve[curve_i];

    /* Loop all segments in curve. */
    int point_i = points.first();
    while (point_i <= points.last()) {
      /* Pick random selection value. */
      const bool random_value = next_bool_random_value();

      point_i = expand_changed_selection_point_to_segment(curve_i + curve_offset,
                                                          point_i,
                                                          points,
                                                          new_selection,
                                                          stored_selection,
                                                          strokes_2d,
                                                          random_value,
                                                          cyclic[curve_i]);
      point_i++;
    }
  }

  selection.finish();
}

}  // namespace blender::ed::greasepencil

eAttrDomain ED_grease_pencil_selection_domain_get(bContext *C)
{
  ToolSettings *ts = CTX_data_tool_settings(C);

  switch (ts->gpencil_selectmode_edit) {
    case GP_SELECTMODE_POINT:
      return ATTR_DOMAIN_POINT;
      break;
    case GP_SELECTMODE_STROKE:
      return ATTR_DOMAIN_CURVE;
      break;
    case GP_SELECTMODE_SEGMENT:
      return ATTR_DOMAIN_POINT;
      break;
  }
  return ATTR_DOMAIN_POINT;
}

bool ED_grease_pencil_segment_selection_mode(bContext *C)
{
  ToolSettings *ts = CTX_data_tool_settings(C);

  return (ts->gpencil_selectmode_edit == GP_SELECTMODE_SEGMENT);
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
}
