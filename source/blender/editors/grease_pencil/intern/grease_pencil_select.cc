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
  int drawing_index_2d = 0;
  Curves2DSpace curves_2d;
  if (segment_mode) {
    curves_2d = ed::greasepencil::editable_strokes_in_2d_space_get(&vc, &grease_pencil);
  }

  grease_pencil.foreach_editable_drawing(
      scene->r.cfra, [&](int /*drawing_index*/, blender::bke::greasepencil::Drawing &drawing) {
        /* In segment mode, store the pre-change point selection. */
        Array<bool> selection_before;
        if (segment_mode) {
          selection_before = ed::greasepencil::point_selection_get(&drawing);
        }

        bke::CurvesGeometry &curves = drawing.strokes_for_write();
        blender::ed::curves::select_adjacent(curves, deselect);

        /* In segment mode, expand the changed point selection to segments. */
        if (segment_mode) {
          expand_changed_selection_to_segments(
              selection_before, curves, drawing_index_2d, &curves_2d);
        }
        drawing_index_2d++;
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
  int drawing_index_2d = 0;
  Curves2DSpace curves_2d;
  if (segment_mode) {
    curves_2d = ed::greasepencil::editable_strokes_in_2d_space_get(&vc, &grease_pencil);
  }

  grease_pencil.foreach_editable_drawing(
      scene->r.cfra, [&](int drawing_index, bke::greasepencil::Drawing &drawing) {
        bke::CurvesGeometry &curves = drawing.strokes_for_write();

        if (segment_mode) {
          /* Set random selection values for each segment on all curves. */
          expand_random_selection_to_segments(
              curves,
              drawing_index_2d,
              &curves_2d,
              blender::get_default_hash_2<int>(seed, drawing_index),
              ratio);

          drawing_index_2d++;
        }
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
  int drawing_index_2d = 0;
  Curves2DSpace curves_2d;
  if (segment_mode) {
    curves_2d = ed::greasepencil::editable_strokes_in_2d_space_get(&vc, &grease_pencil);
  }

  grease_pencil.foreach_editable_drawing(
      scene->r.cfra, [&](int /*drawing_index*/, blender::bke::greasepencil::Drawing &drawing) {
        bke::CurvesGeometry &curves = drawing.strokes_for_write();

        /* In segment mode, store the pre-change point selection. */
        Array<bool> selection_before;
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
              selection_before, curves, drawing_index_2d, &curves_2d);
        }
        drawing_index_2d++;
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
  ts->gpencil_selectmode_edit = mode_new;

  /* Convert all drawings of the active GP to the new selection domain. */
  const eAttrDomain domain = ED_grease_pencil_selection_domain_get(C);
  Object *object = CTX_data_active_object(C);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object->data);
  Span<GreasePencilDrawingBase *> drawings = grease_pencil.drawings();
  bool changed = false;

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
    if (domain == ATTR_DOMAIN_CURVE) {
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
    }
  }

  /* When the new selection mode is 'segment', we expand the point selection to segments. */
  const bool segment_mode = ED_grease_pencil_segment_selection_mode(C);
  if (segment_mode) {
    Depsgraph *depsgraph = CTX_data_ensure_evaluated_depsgraph(C);
    ViewContext vc;
    Scene *scene = CTX_data_scene(C);
    ED_view3d_viewcontext_init(C, &vc, depsgraph);

    /* Get viewport 2D representation of editable curves. */
    int drawing_index_2d = 0;
    Curves2DSpace curves_2d = ed::greasepencil::editable_strokes_in_2d_space_get(&vc,
                                                                                 &grease_pencil);

    /* For performance reasons, we only expand the selection of editable curves in the current
     * frame. */
    grease_pencil.foreach_editable_drawing(
        scene->r.cfra, [&](int /*drawing_index*/, blender::bke::greasepencil::Drawing &drawing) {
          bke::CurvesGeometry &curves = drawing.strokes_for_write();
          Array<bool> selection_before(curves.points_num(), false);

          /* Expand existing point selection to segments. */
          expand_changed_selection_to_segments(
              selection_before, curves, drawing_index_2d, &curves_2d);
          drawing_index_2d++;
          changed = true;
        });
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

Array<bool> point_selection_get(const GreasePencilDrawing *drawing)
{
  /* Get point selection in the drawing. */
  const bke::CurvesGeometry &curves = drawing->geometry.wrap();
  const bke::AttributeAccessor attributes = curves.attributes();
  if (!attributes.contains(".selection")) {
    return Array<bool>(curves.points_num(), false);
  }

  /* Copy selection. */
  const VArray<bool> selection = *attributes.lookup(".selection").typed<bool>();
  Array<bool> point_selection(selection.size());
  for (const int point_i : selection.index_range()) {
    point_selection[point_i] = selection[point_i];
  }

  return point_selection;
}

static int expand_changed_selection_point_to_segment(const int segment_curve_index,
                                                     const int segment_point_start,
                                                     const IndexRange points,
                                                     MutableSpan<bool> &new_selection,
                                                     Array<bool> &old_selection,
                                                     const Curves2DSpace *curves_2d,
                                                     const bool selection_state,
                                                     const bool is_cyclic)
{
  /* Walk forward and backward along the curve from the point where
   * the selection has changed. We expand the changed selection
   * to the entire segment. */
  int point_abs;
  int point_cont_offset = curves_2d->point_offset[segment_curve_index] - points.first();
  const int directions[2] = {-1, 1};

  for (const int direction : directions) {
    point_abs = segment_point_start;
    bool intersected = false;
    while ((direction == -1 && point_abs > points.first()) ||
           (direction == 1 && point_abs < points.last()))
    {
      const int point_cont = point_abs + point_cont_offset;

      /* The end of a segment is reached when the vector between the current and
       * next point intersects with an other stroke. */
      intersected = intersect_segment_strokes_2d(curves_2d->points_2d[point_cont],
                                                 curves_2d->points_2d[point_cont + direction],
                                                 segment_curve_index,
                                                 curves_2d);

      /* Set selection state. */
      new_selection[point_abs] = selection_state;
      old_selection[point_abs] = selection_state;

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
      old_selection[point_abs] = selection_state;
    }
  }

  return point_abs;
}

void expand_changed_selection_to_segments(Array<bool> &old_selection,
                                          bke::CurvesGeometry &curves,
                                          const int drawing_index_2d,
                                          const Curves2DSpace *curves_2d)
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
      if (old_selection[point_i] != new_selection[point_i]) {
        expand_changed_selection_point_to_segment(curve_i +
                                                      curves_2d->curve_offset[drawing_index_2d],
                                                  point_i,
                                                  points,
                                                  new_selection,
                                                  old_selection,
                                                  curves_2d,
                                                  new_selection[point_i],
                                                  cyclic[curve_i]);
      }
    }
  }

  selection.finish();
}

void expand_random_selection_to_segments(bke::CurvesGeometry &curves,
                                         const int drawing_index_2d,
                                         const Curves2DSpace *curves_2d,
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
  Array<bool> old_selection = Array<bool>(selection.span.size());

  for (const int curve_i : curves.curves_range()) {
    const IndexRange points = points_by_curve[curve_i];

    /* Loop all segments in curve. */
    int point_i = points.first();
    while (point_i <= points.last()) {
      /* Pick random selection value. */
      const bool random_value = next_bool_random_value();

      point_i = expand_changed_selection_point_to_segment(
          curve_i + curves_2d->curve_offset[drawing_index_2d],
          point_i,
          points,
          new_selection,
          old_selection,
          curves_2d,
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
  WM_operatortype_append(GREASE_PENCIL_OT_set_selection_mode);
}
