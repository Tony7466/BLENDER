/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgreasepencil
 */

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

bool editable_grease_pencil_poll(bContext *C)
{
  Object *object = CTX_data_active_object(C);
  if (object == nullptr || object->type != OB_GREASE_PENCIL) {
    return false;
  }
  if (!ED_operator_object_active_editable_ex(C, object)) {
    return false;
  }
  if ((object->mode & OB_MODE_EDIT) == 0) {
    return false;
  }
  return true;
}

bool editable_grease_pencil_point_selection_poll(bContext *C)
{
  if (!editable_grease_pencil_poll(C)) {
    return false;
  }

  /* Allowed: point and segment selection mode, not allowed: stroke selection mode. */
  ToolSettings *ts = CTX_data_tool_settings(C);
  return (ts->gpencil_selectmode_edit != GP_SELECTMODE_STROKE);
}

bool editable_grease_pencil_no_segment_selection_poll(bContext *C)
{
  if (!editable_grease_pencil_poll(C)) {
    return false;
  }

  /* Allowed: point and stroke selection mode, not allowed: segment selection mode. */
  ToolSettings *ts = CTX_data_tool_settings(C);
  return (ts->gpencil_selectmode_edit != GP_SELECTMODE_SEGMENT);
}

static int select_all_exec(bContext *C, wmOperator *op)
{
  int action = RNA_enum_get(op->ptr, "action");
  Scene *scene = CTX_data_scene(C);
  Object *object = CTX_data_active_object(C);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object->data);
  bool segment_mode;
  eAttrDomain domain = ED_view3d_grease_pencil_selection_domain_get(C, &segment_mode);

  grease_pencil.foreach_editable_drawing(
      scene->r.cfra, [action, domain](int /*drawing_index*/, GreasePencilDrawing &drawing) {
        blender::ed::curves::select_all(drawing.geometry.wrap(), domain, action);
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

  /* Get selection domain from tool settings. */
  bool segment_mode;
  const eAttrDomain selection_domain = ED_view3d_grease_pencil_selection_domain_get(C,
                                                                                    &segment_mode);
  UNUSED_VARS(selection_domain);

  /* In segment mode, we expand the point selection to segments after the selection has changed.
   * For checking segment intersections, we use strokes converted to viewport 2D space. */
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

        blender::ed::curves::select_adjacent(drawing.geometry.wrap(), deselect);

        /* In segment mode, expand the changed point selection to segments. */
        if (segment_mode) {
          ed::greasepencil::expand_changed_selection_to_segments(
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
      scene->r.cfra, [](int /*drawing_index*/, GreasePencilDrawing &drawing) {
        /* Works the same for 'point' and 'segment' mode. 'Stroke' mode is excluded in poll. */
        blender::ed::curves::select_linked(drawing.geometry.wrap());
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
  const float ratio = RNA_float_get(op->ptr, "ratio");
  const int seed = WM_operator_properties_select_random_seed_increment_get(op);
  Scene *scene = CTX_data_scene(C);
  Object *object = CTX_data_active_object(C);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object->data);

  /* Get selection domain from tool settings. */
  bool segment_mode;
  const eAttrDomain selection_domain = ED_view3d_grease_pencil_selection_domain_get(C,
                                                                                    &segment_mode);

  grease_pencil.foreach_editable_drawing(
      scene->r.cfra, [&](int drawing_index, GreasePencilDrawing &drawing) {
        blender::ed::curves::select_random(drawing.geometry.wrap(),
                                           selection_domain,
                                           blender::get_default_hash_2<int>(seed, drawing_index),
                                           ratio);
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
  ot->poll = editable_grease_pencil_no_segment_selection_poll;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  WM_operator_properties_select_random(ot);
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
  bool segment_mode;
  const eAttrDomain selection_domain = ED_view3d_grease_pencil_selection_domain_get(C,
                                                                                    &segment_mode);
  UNUSED_VARS(selection_domain);

  /* In segment mode, we expand the point selection to segments after the selection has changed.
   * For checking segment intersections, we use strokes converted to viewport 2D space. */
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

        blender::ed::curves::select_ends(drawing.geometry.wrap(), amount_start, amount_end);

        /* In segment mode, expand the changed point selection to segments. */
        if (segment_mode) {
          ed::greasepencil::expand_changed_selection_to_segments(
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

static void keymap_grease_pencil_editing(wmKeyConfig *keyconf)
{
  wmKeyMap *keymap = WM_keymap_ensure(keyconf, "Grease Pencil Edit Mode", 0, 0);
  keymap->poll = editable_grease_pencil_poll;
}

}  // namespace blender::ed::greasepencil
void ED_operatortypes_grease_pencil(void)
{
  using namespace blender::ed::greasepencil;
  WM_operatortype_append(GREASE_PENCIL_OT_select_all);
  WM_operatortype_append(GREASE_PENCIL_OT_select_more);
  WM_operatortype_append(GREASE_PENCIL_OT_select_less);
  WM_operatortype_append(GREASE_PENCIL_OT_select_linked);
  WM_operatortype_append(GREASE_PENCIL_OT_select_random);
  WM_operatortype_append(GREASE_PENCIL_OT_select_ends);
}

void ED_keymap_grease_pencil(wmKeyConfig *keyconf)
{
  using namespace blender::ed::greasepencil;
  keymap_grease_pencil_editing(keyconf);
}
