/* SPDX-FileCopyrightText: 2008 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgpencil
 *
 * Operators for dealing with GP data-blocks and layers.
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "MEM_guardedalloc.h"

#include "BLI_blenlib.h"
#include "BLI_ghash.h"
#include "BLI_math_geom.h"
#include "BLI_math_matrix.h"
#include "BLI_math_vector.h"
#include "BLI_string_utils.hh"
#include "BLI_utildefines.h"

#include "BLT_translation.hh"

#include "DNA_anim_types.h"
#include "DNA_brush_types.h"
#include "DNA_gpencil_legacy_types.h"
#include "DNA_material_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_screen_types.h"
#include "DNA_view3d_types.h"

#include "BKE_anim_data.hh"
#include "BKE_animsys.h"
#include "BKE_brush.hh"
#include "BKE_context.hh"
#include "BKE_deform.hh"
#include "BKE_fcurve_driver.h"
#include "BKE_gpencil_legacy.h"
#include "BKE_lib_id.hh"
#include "BKE_main.hh"
#include "BKE_material.h"
#include "BKE_paint.hh"
#include "BKE_report.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"
#include "RNA_enum_types.hh"

#include "ED_gpencil_legacy.hh"
#include "ED_object.hh"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_build.hh"

#include "gpencil_intern.hh"

/* ************************************************ */
/* Datablock Operators */

/* ******************* Add New Data ************************ */
static bool gpencil_data_add_poll(bContext *C)
{

  /* the base line we have is that we have somewhere to add Grease Pencil data */
  return ED_annotation_data_get_pointers(C, nullptr) != nullptr;
}

/* add new datablock - wrapper around API */
static int gpencil_data_add_exec(bContext *C, wmOperator *op)
{
  PointerRNA gpd_owner = {nullptr};
  bGPdata **gpd_ptr = ED_annotation_data_get_pointers(C, &gpd_owner);

  if (gpd_ptr == nullptr) {
    BKE_report(op->reports, RPT_ERROR, "Nowhere for grease pencil data to go");
    return OPERATOR_CANCELLED;
  }

  /* decrement user count and add new datablock */
  /* TODO: if a datablock exists,
   * we should make a copy of it instead of starting fresh (as in other areas) */
  Main *bmain = CTX_data_main(C);

  /* decrement user count of old GP datablock */
  if (*gpd_ptr) {
    bGPdata *gpd = (*gpd_ptr);
    id_us_min(&gpd->id);
  }

  /* Add new datablock, with a single layer ready to use
   * (so users don't have to perform an extra step). */
  bGPdata *gpd = BKE_gpencil_data_addnew(bmain, DATA_("Annotations"));
  *gpd_ptr = gpd;

  /* tag for annotations */
  gpd->flag |= GP_DATA_ANNOTATIONS;

  /* add new layer (i.e. a "note") */
  BKE_gpencil_layer_addnew(*gpd_ptr, DATA_("Note"), true, false);

  /* notifiers */
  WM_event_add_notifier(C, NC_GPENCIL | ND_DATA | NA_EDITED, nullptr);

  return OPERATOR_FINISHED;
}

void GPENCIL_OT_annotation_add(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Annotation Add New";
  ot->idname = "GPENCIL_OT_annotation_add";
  ot->description = "Add new Annotation data-block";
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  /* callbacks */
  ot->exec = gpencil_data_add_exec;
  ot->poll = gpencil_data_add_poll;
}

/* ******************* Unlink Data ************************ */

/* poll callback for adding data/layers - special */
static bool gpencil_data_unlink_poll(bContext *C)
{
  bGPdata **gpd_ptr = ED_annotation_data_get_pointers(C, nullptr);

  /* only unlink annotation datablocks */
  if ((gpd_ptr != nullptr) && (*gpd_ptr != nullptr)) {
    bGPdata *gpd = (*gpd_ptr);
    if ((gpd->flag & GP_DATA_ANNOTATIONS) == 0) {
      return false;
    }
  }
  /* if we have access to some active data, make sure there's a datablock before enabling this */
  return (gpd_ptr && *gpd_ptr);
}

/* unlink datablock - wrapper around API */
static int gpencil_data_unlink_exec(bContext *C, wmOperator *op)
{
  bGPdata **gpd_ptr = ED_annotation_data_get_pointers(C, nullptr);

  if (gpd_ptr == nullptr) {
    BKE_report(op->reports, RPT_ERROR, "Nowhere for grease pencil data to go");
    return OPERATOR_CANCELLED;
  }
  /* just unlink datablock now, decreasing its user count */
  bGPdata *gpd = (*gpd_ptr);

  id_us_min(&gpd->id);
  *gpd_ptr = nullptr;

  /* notifiers */
  WM_event_add_notifier(C, NC_GPENCIL | ND_DATA | NA_EDITED, nullptr);

  return OPERATOR_FINISHED;
}

void GPENCIL_OT_data_unlink(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Annotation Unlink";
  ot->idname = "GPENCIL_OT_data_unlink";
  ot->description = "Unlink active Annotation data-block";
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  /* callbacks */
  ot->exec = gpencil_data_unlink_exec;
  ot->poll = gpencil_data_unlink_poll;
}

/* ************************************************ */
/* Layer Operators */

/* ******************* Add New Layer ************************ */

/* add new layer - wrapper around API */
static int gpencil_layer_add_exec(bContext *C, wmOperator *op)
{
  const bool is_annotation = STREQ(op->idname, "GPENCIL_OT_layer_annotation_add");

  PointerRNA gpd_owner = {nullptr};
  Main *bmain = CTX_data_main(C);
  Scene *scene = CTX_data_scene(C);
  bGPdata *gpd = nullptr;

  if (is_annotation) {
    bGPdata **gpd_ptr = ED_annotation_data_get_pointers(C, &gpd_owner);
    /* if there's no existing Grease-Pencil data there, add some */
    if (gpd_ptr == nullptr) {
      BKE_report(op->reports, RPT_ERROR, "Nowhere for grease pencil data to go");
      return OPERATOR_CANCELLED;
    }
    /* Annotations */
    if (*gpd_ptr == nullptr) {
      *gpd_ptr = BKE_gpencil_data_addnew(bmain, DATA_("Annotations"));
    }

    /* mark as annotation */
    (*gpd_ptr)->flag |= GP_DATA_ANNOTATIONS;
    BKE_gpencil_layer_addnew(*gpd_ptr, DATA_("Note"), true, false);
    gpd = *gpd_ptr;
  }
  else {
    /* GP Object */
    Object *ob = CTX_data_active_object(C);
    if ((ob != nullptr) && (ob->type == OB_GPENCIL_LEGACY)) {
      gpd = (bGPdata *)ob->data;
      PropertyRNA *prop;
      char name[128];
      prop = RNA_struct_find_property(op->ptr, "new_layer_name");
      if (RNA_property_is_set(op->ptr, prop)) {
        RNA_property_string_get(op->ptr, prop, name);
      }
      else {
        STRNCPY(name, "GP_Layer");
      }
      bGPDlayer *gpl = BKE_gpencil_layer_addnew(gpd, name, true, false);

      /* Add a new frame to make it visible in Dopesheet. */
      if (gpl != nullptr) {
        gpl->actframe = BKE_gpencil_layer_frame_get(gpl, scene->r.cfra, GP_GETFRAME_ADD_NEW);
      }
    }
  }

  /* notifiers */
  if (gpd) {
    DEG_id_tag_update(&gpd->id, ID_RECALC_TRANSFORM | ID_RECALC_GEOMETRY | ID_RECALC_SYNC_TO_EVAL);
  }
  WM_event_add_notifier(C, NC_GPENCIL | ND_DATA | NA_EDITED, nullptr);
  WM_event_add_notifier(C, NC_GPENCIL | ND_DATA | NA_SELECTED, nullptr);

  return OPERATOR_FINISHED;
}

static bool gpencil_add_annotation_poll(bContext *C)
{
  return ED_annotation_data_get_pointers(C, nullptr) != nullptr;
}

void GPENCIL_OT_layer_annotation_add(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Add New Annotation Layer";
  ot->idname = "GPENCIL_OT_layer_annotation_add";
  ot->description = "Add new Annotation layer or note for the active data-block";

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  /* callbacks */
  ot->exec = gpencil_layer_add_exec;
  ot->poll = gpencil_add_annotation_poll;
}
/* ******************* Remove Active Layer ************************* */

static int gpencil_layer_remove_exec(bContext *C, wmOperator *op)
{
  const bool is_annotation = STREQ(op->idname, "GPENCIL_OT_layer_annotation_remove");

  bGPdata *gpd = (!is_annotation) ? ED_gpencil_data_get_active(C) :
                                    ED_annotation_data_get_active(C);
  bGPDlayer *gpl = BKE_gpencil_layer_active_get(gpd);

  /* sanity checks */
  if (ELEM(nullptr, gpd, gpl)) {
    return OPERATOR_CANCELLED;
  }

  if (gpl->flag & GP_LAYER_LOCKED) {
    BKE_report(op->reports, RPT_ERROR, "Cannot delete locked layers");
    return OPERATOR_CANCELLED;
  }

  /* make the layer before this the new active layer
   * - use the one after if this is the first
   * - if this is the only layer, this naturally becomes nullptr
   */
  if (gpl->prev) {
    BKE_gpencil_layer_active_set(gpd, gpl->prev);
  }
  else {
    BKE_gpencil_layer_active_set(gpd, gpl->next);
  }

  /* delete the layer now... */
  BKE_gpencil_layer_delete(gpd, gpl);

  /* Reorder masking. */
  BKE_gpencil_layer_mask_sort_all(gpd);

  /* notifiers */
  DEG_id_tag_update(&gpd->id, ID_RECALC_TRANSFORM | ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_GPENCIL | ND_DATA | NA_EDITED, nullptr);
  WM_event_add_notifier(C, NC_GPENCIL | ND_DATA | NA_SELECTED, nullptr);

  /* Free Grease Pencil data block when last annotation layer is removed, see: #112683. */
  if (is_annotation && gpd->layers.first == nullptr) {
    BKE_gpencil_free_data(gpd, true);

    bGPdata **gpd_ptr = ED_annotation_data_get_pointers(C, nullptr);
    *gpd_ptr = nullptr;

    Main *bmain = CTX_data_main(C);
    BKE_id_free_us(bmain, gpd);
  }

  return OPERATOR_FINISHED;
}

static bool gpencil_active_layer_annotation_poll(bContext *C)
{
  bGPdata *gpd = ED_annotation_data_get_active(C);
  bGPDlayer *gpl = BKE_gpencil_layer_active_get(gpd);

  return (gpl != nullptr);
}

void GPENCIL_OT_layer_annotation_remove(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Remove Annotation Layer";
  ot->idname = "GPENCIL_OT_layer_annotation_remove";
  ot->description = "Remove active Annotation layer";

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  /* callbacks */
  ot->exec = gpencil_layer_remove_exec;
  ot->poll = gpencil_active_layer_annotation_poll;
}
/* ******************* Move Layer Up/Down ************************** */

enum {
  GP_LAYER_MOVE_UP = -1,
  GP_LAYER_MOVE_DOWN = 1,
};

static int gpencil_layer_move_exec(bContext *C, wmOperator *op)
{
  const bool is_annotation = STREQ(op->idname, "GPENCIL_OT_layer_annotation_move");

  bGPdata *gpd = (!is_annotation) ? ED_gpencil_data_get_active(C) :
                                    ED_annotation_data_get_active(C);
  bGPDlayer *gpl = BKE_gpencil_layer_active_get(gpd);

  const int direction = RNA_enum_get(op->ptr, "type") * -1;

  /* sanity checks */
  if (ELEM(nullptr, gpd, gpl)) {
    return OPERATOR_CANCELLED;
  }

  BLI_assert(ELEM(direction, -1, 0, 1)); /* we use value below */
  if (BLI_listbase_link_move(&gpd->layers, gpl, direction)) {
    /* Reorder masking. */
    BKE_gpencil_layer_mask_sort_all(gpd);

    DEG_id_tag_update(&gpd->id, ID_RECALC_TRANSFORM | ID_RECALC_GEOMETRY);
    WM_event_add_notifier(C, NC_GPENCIL | ND_DATA | NA_EDITED, nullptr);
  }

  return OPERATOR_FINISHED;
}

void GPENCIL_OT_layer_annotation_move(wmOperatorType *ot)
{
  static const EnumPropertyItem slot_move[] = {
      {GP_LAYER_MOVE_UP, "UP", 0, "Up", ""},
      {GP_LAYER_MOVE_DOWN, "DOWN", 0, "Down", ""},
      {0, nullptr, 0, nullptr, nullptr},
  };

  /* identifiers */
  ot->name = "Move Annotation Layer";
  ot->idname = "GPENCIL_OT_layer_annotation_move";
  ot->description = "Move the active Annotation layer up/down in the list";

  /* api callbacks */
  ot->exec = gpencil_layer_move_exec;
  ot->poll = gpencil_active_layer_annotation_poll;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  ot->prop = RNA_def_enum(ot->srna, "type", slot_move, 0, "Type", "");
}

/*********************** Vertex Groups ***********************************/

static bool gpencil_vertex_group_poll(bContext *C)
{
  Object *ob = CTX_data_active_object(C);

  if ((ob) && (ob->type == OB_GPENCIL_LEGACY)) {
    Main *bmain = CTX_data_main(C);
    const bGPdata *gpd = (const bGPdata *)ob->data;
    if (BKE_id_is_editable(bmain, &ob->id) &&
        BKE_id_is_editable(bmain, static_cast<const ID *>(ob->data)) &&
        !BLI_listbase_is_empty(&gpd->vertex_group_names))
    {
      if (ELEM(ob->mode, OB_MODE_EDIT_GPENCIL_LEGACY, OB_MODE_SCULPT_GPENCIL_LEGACY)) {
        return true;
      }
    }
  }

  return false;
}

static bool gpencil_vertex_group_weight_poll(bContext *C)
{
  Object *ob = CTX_data_active_object(C);

  if ((ob) && (ob->type == OB_GPENCIL_LEGACY)) {
    Main *bmain = CTX_data_main(C);
    const bGPdata *gpd = (const bGPdata *)ob->data;
    if (BKE_id_is_editable(bmain, &ob->id) &&
        BKE_id_is_editable(bmain, static_cast<const ID *>(ob->data)) &&
        !BLI_listbase_is_empty(&gpd->vertex_group_names))
    {
      if (ob->mode == OB_MODE_WEIGHT_GPENCIL_LEGACY) {
        return true;
      }
    }
  }

  return false;
}

static int gpencil_vertex_group_assign_exec(bContext *C, wmOperator * /*op*/)
{
  ToolSettings *ts = CTX_data_tool_settings(C);
  Object *ob = CTX_data_active_object(C);

  /* sanity checks */
  if (ELEM(nullptr, ts, ob, ob->data)) {
    return OPERATOR_CANCELLED;
  }

  ED_gpencil_vgroup_assign(C, ob, ts->vgroup_weight);

  /* notifiers */
  bGPdata *gpd = static_cast<bGPdata *>(ob->data);
  DEG_id_tag_update(&gpd->id, ID_RECALC_TRANSFORM | ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_GPENCIL | ND_DATA | NA_EDITED | ND_SPACE_PROPERTIES, nullptr);

  return OPERATOR_FINISHED;
}

void GPENCIL_OT_vertex_group_assign(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Assign to Vertex Group";
  ot->idname = "GPENCIL_OT_vertex_group_assign";
  ot->description = "Assign the selected vertices to the active vertex group";

  /* api callbacks */
  ot->poll = gpencil_vertex_group_poll;
  ot->exec = gpencil_vertex_group_assign_exec;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

/* remove point from vertex group */
static int gpencil_vertex_group_remove_from_exec(bContext *C, wmOperator * /*op*/)
{
  Object *ob = CTX_data_active_object(C);

  /* sanity checks */
  if (ELEM(nullptr, ob, ob->data)) {
    return OPERATOR_CANCELLED;
  }

  ED_gpencil_vgroup_remove(C, ob);

  /* notifiers */
  bGPdata *gpd = static_cast<bGPdata *>(ob->data);
  DEG_id_tag_update(&gpd->id, ID_RECALC_TRANSFORM | ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_GPENCIL | ND_DATA | NA_EDITED | ND_SPACE_PROPERTIES, nullptr);

  return OPERATOR_FINISHED;
}

void GPENCIL_OT_vertex_group_remove_from(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Remove from Vertex Group";
  ot->idname = "GPENCIL_OT_vertex_group_remove_from";
  ot->description = "Remove the selected vertices from active or all vertex group(s)";

  /* api callbacks */
  ot->poll = gpencil_vertex_group_poll;
  ot->exec = gpencil_vertex_group_remove_from_exec;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

static int gpencil_vertex_group_select_exec(bContext *C, wmOperator * /*op*/)
{
  Object *ob = CTX_data_active_object(C);

  /* sanity checks */
  if (ELEM(nullptr, ob, ob->data)) {
    return OPERATOR_CANCELLED;
  }

  ED_gpencil_vgroup_select(C, ob);

  /* notifiers */
  bGPdata *gpd = static_cast<bGPdata *>(ob->data);
  DEG_id_tag_update(&gpd->id, ID_RECALC_TRANSFORM | ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_GPENCIL | ND_DATA | NA_EDITED | ND_SPACE_PROPERTIES, nullptr);

  return OPERATOR_FINISHED;
}

void GPENCIL_OT_vertex_group_select(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Select Vertex Group";
  ot->idname = "GPENCIL_OT_vertex_group_select";
  ot->description = "Select all the vertices assigned to the active vertex group";

  /* api callbacks */
  ot->poll = gpencil_vertex_group_poll;
  ot->exec = gpencil_vertex_group_select_exec;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

static int gpencil_vertex_group_deselect_exec(bContext *C, wmOperator * /*op*/)
{
  Object *ob = CTX_data_active_object(C);

  /* sanity checks */
  if (ELEM(nullptr, ob, ob->data)) {
    return OPERATOR_CANCELLED;
  }

  ED_gpencil_vgroup_deselect(C, ob);

  /* notifiers */
  bGPdata *gpd = static_cast<bGPdata *>(ob->data);
  DEG_id_tag_update(&gpd->id, ID_RECALC_TRANSFORM | ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_GPENCIL | ND_DATA | NA_EDITED | ND_SPACE_PROPERTIES, nullptr);

  return OPERATOR_FINISHED;
}

void GPENCIL_OT_vertex_group_deselect(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Deselect Vertex Group";
  ot->idname = "GPENCIL_OT_vertex_group_deselect";
  ot->description = "Deselect all selected vertices assigned to the active vertex group";

  /* api callbacks */
  ot->poll = gpencil_vertex_group_poll;
  ot->exec = gpencil_vertex_group_deselect_exec;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

/* invert */
static int gpencil_vertex_group_invert_exec(bContext *C, wmOperator *op)
{
  ToolSettings *ts = CTX_data_tool_settings(C);
  Object *ob = CTX_data_active_object(C);
  bGPdata *gpd = static_cast<bGPdata *>(ob->data);

  /* sanity checks */
  if (ELEM(nullptr, ts, ob, ob->data)) {
    return OPERATOR_CANCELLED;
  }

  MDeformVert *dvert;
  const int def_nr = gpd->vertex_group_active_index - 1;

  bDeformGroup *defgroup = static_cast<bDeformGroup *>(
      BLI_findlink(&gpd->vertex_group_names, def_nr));
  if (defgroup == nullptr) {
    return OPERATOR_CANCELLED;
  }
  if (defgroup->flag & DG_LOCK_WEIGHT) {
    BKE_report(op->reports, RPT_ERROR, "Current Vertex Group is locked");
    return OPERATOR_CANCELLED;
  }

  CTX_DATA_BEGIN (C, bGPDstroke *, gps, editable_gpencil_strokes) {
    /* Verify the strokes has something to change. */
    if ((gps->totpoints == 0) || (gps->dvert == nullptr)) {
      continue;
    }

    for (int i = 0; i < gps->totpoints; i++) {
      dvert = &gps->dvert[i];
      MDeformWeight *dw = BKE_defvert_find_index(dvert, def_nr);
      if (dw == nullptr) {
        BKE_defvert_add_index_notest(dvert, def_nr, 1.0f);
      }
      else if (dw->weight == 1.0f) {
        BKE_defvert_remove_group(dvert, dw);
      }
      else {
        dw->weight = 1.0f - dw->weight;
      }
    }
  }
  CTX_DATA_END;

  /* notifiers */
  DEG_id_tag_update(&gpd->id, ID_RECALC_TRANSFORM | ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_GPENCIL | ND_DATA | NA_EDITED | ND_SPACE_PROPERTIES, nullptr);

  return OPERATOR_FINISHED;
}

void GPENCIL_OT_vertex_group_invert(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Invert Vertex Group";
  ot->idname = "GPENCIL_OT_vertex_group_invert";
  ot->description = "Invert weights to the active vertex group";

  /* api callbacks */
  ot->poll = gpencil_vertex_group_weight_poll;
  ot->exec = gpencil_vertex_group_invert_exec;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

/* smooth */
static int gpencil_vertex_group_smooth_exec(bContext *C, wmOperator *op)
{
  const float fac = RNA_float_get(op->ptr, "factor");
  const int repeat = RNA_int_get(op->ptr, "repeat");

  ToolSettings *ts = CTX_data_tool_settings(C);
  Object *ob = CTX_data_active_object(C);
  bGPdata *gpd = static_cast<bGPdata *>(ob->data);

  /* sanity checks */
  if (ELEM(nullptr, ts, ob, ob->data)) {
    return OPERATOR_CANCELLED;
  }

  const int def_nr = gpd->vertex_group_active_index - 1;
  bDeformGroup *defgroup = static_cast<bDeformGroup *>(
      BLI_findlink(&gpd->vertex_group_names, def_nr));
  if (defgroup == nullptr) {
    return OPERATOR_CANCELLED;
  }
  if (defgroup->flag & DG_LOCK_WEIGHT) {
    BKE_report(op->reports, RPT_ERROR, "Current Vertex Group is locked");
    return OPERATOR_CANCELLED;
  }

  bGPDspoint *pta, *ptb, *ptc;
  MDeformVert *dverta, *dvertb;

  CTX_DATA_BEGIN (C, bGPDstroke *, gps, editable_gpencil_strokes) {
    /* Verify the strokes has something to change. */
    if ((gps->totpoints == 0) || (gps->dvert == nullptr)) {
      continue;
    }

    for (int s = 0; s < repeat; s++) {
      for (int i = 0; i < gps->totpoints; i++) {
        /* previous point */
        if (i > 0) {
          pta = &gps->points[i - 1];
          dverta = &gps->dvert[i - 1];
        }
        else {
          pta = &gps->points[i];
          dverta = &gps->dvert[i];
        }
        /* current */
        ptb = &gps->points[i];
        dvertb = &gps->dvert[i];
        /* next point */
        if (i + 1 < gps->totpoints) {
          ptc = &gps->points[i + 1];
        }
        else {
          ptc = &gps->points[i];
        }

        float wa = BKE_defvert_find_weight(dverta, def_nr);
        float wb = BKE_defvert_find_weight(dvertb, def_nr);

        /* the optimal value is the corresponding to the interpolation of the weight
         * at the distance of point b
         */
        const float opfac = line_point_factor_v3(&ptb->x, &pta->x, &ptc->x);
        const float optimal = interpf(wa, wb, opfac);
        /* Based on influence factor, blend between original and optimal */
        MDeformWeight *dw = BKE_defvert_ensure_index(dvertb, def_nr);
        if (dw) {
          dw->weight = interpf(wb, optimal, fac);
          CLAMP(dw->weight, 0.0, 1.0f);
        }
      }
    }
  }
  CTX_DATA_END;

  /* notifiers */
  DEG_id_tag_update(&gpd->id, ID_RECALC_TRANSFORM | ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_GPENCIL | ND_DATA | NA_EDITED | ND_SPACE_PROPERTIES, nullptr);

  return OPERATOR_FINISHED;
}

void GPENCIL_OT_vertex_group_smooth(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Smooth Vertex Group";
  ot->idname = "GPENCIL_OT_vertex_group_smooth";
  ot->description = "Smooth weights to the active vertex group";

  /* api callbacks */
  ot->poll = gpencil_vertex_group_weight_poll;
  ot->exec = gpencil_vertex_group_smooth_exec;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  RNA_def_float(ot->srna, "factor", 0.5f, 0.0f, 1.0, "Factor", "", 0.0f, 1.0f);
  RNA_def_int(ot->srna, "repeat", 1, 1, 10000, "Iterations", "", 1, 200);
}

/* normalize */
static int gpencil_vertex_group_normalize_exec(bContext *C, wmOperator *op)
{
  ToolSettings *ts = CTX_data_tool_settings(C);
  Object *ob = CTX_data_active_object(C);
  bGPdata *gpd = static_cast<bGPdata *>(ob->data);

  /* sanity checks */
  if (ELEM(nullptr, ts, ob, ob->data)) {
    return OPERATOR_CANCELLED;
  }

  MDeformVert *dvert = nullptr;
  MDeformWeight *dw = nullptr;
  const int def_nr = gpd->vertex_group_active_index - 1;
  bDeformGroup *defgroup = static_cast<bDeformGroup *>(
      BLI_findlink(&gpd->vertex_group_names, def_nr));
  if (defgroup == nullptr) {
    return OPERATOR_CANCELLED;
  }
  if (defgroup->flag & DG_LOCK_WEIGHT) {
    BKE_report(op->reports, RPT_ERROR, "Current Vertex Group is locked");
    return OPERATOR_CANCELLED;
  }

  CTX_DATA_BEGIN (C, bGPDstroke *, gps, editable_gpencil_strokes) {
    /* Verify the strokes has something to change. */
    if ((gps->totpoints == 0) || (gps->dvert == nullptr)) {
      continue;
    }

    /* look for max value */
    float maxvalue = 0.0f;
    for (int i = 0; i < gps->totpoints; i++) {
      dvert = &gps->dvert[i];
      dw = BKE_defvert_find_index(dvert, def_nr);
      if ((dw != nullptr) && (dw->weight > maxvalue)) {
        maxvalue = dw->weight;
      }
    }

    /* normalize weights */
    if (maxvalue > 0.0f) {
      for (int i = 0; i < gps->totpoints; i++) {
        dvert = &gps->dvert[i];
        dw = BKE_defvert_find_index(dvert, def_nr);
        if (dw != nullptr) {
          dw->weight = dw->weight / maxvalue;
        }
      }
    }
  }
  CTX_DATA_END;

  /* notifiers */
  DEG_id_tag_update(&gpd->id, ID_RECALC_TRANSFORM | ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_GPENCIL | ND_DATA | NA_EDITED | ND_SPACE_PROPERTIES, nullptr);

  return OPERATOR_FINISHED;
}

void GPENCIL_OT_vertex_group_normalize(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Normalize Vertex Group";
  ot->idname = "GPENCIL_OT_vertex_group_normalize";
  ot->description = "Normalize weights to the active vertex group";

  /* api callbacks */
  ot->poll = gpencil_vertex_group_weight_poll;
  ot->exec = gpencil_vertex_group_normalize_exec;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

/* normalize all */
static int gpencil_vertex_group_normalize_all_exec(bContext *C, wmOperator *op)
{
  ToolSettings *ts = CTX_data_tool_settings(C);
  Object *ob = CTX_data_active_object(C);
  bool lock_active = RNA_boolean_get(op->ptr, "lock_active");
  bGPdata *gpd = static_cast<bGPdata *>(ob->data);

  /* sanity checks */
  if (ELEM(nullptr, ts, ob, ob->data)) {
    return OPERATOR_CANCELLED;
  }

  bDeformGroup *defgroup = nullptr;
  MDeformVert *dvert = nullptr;
  MDeformWeight *dw = nullptr;
  const int def_nr = gpd->vertex_group_active_index - 1;
  const int defbase_tot = BLI_listbase_count(&gpd->vertex_group_names);
  if (defbase_tot == 0) {
    return OPERATOR_CANCELLED;
  }

  CTX_DATA_BEGIN (C, bGPDstroke *, gps, editable_gpencil_strokes) {
    /* Verify the strokes has something to change. */
    if ((gps->totpoints == 0) || (gps->dvert == nullptr)) {
      continue;
    }

    /* Loop all points in stroke. */
    for (int i = 0; i < gps->totpoints; i++) {
      int v;
      float sum = 0.0f;
      float sum_lock = 0.0f;
      float sum_unlock = 0.0f;

      /* Get vertex groups and weights. */
      dvert = &gps->dvert[i];

      /* Sum weights. */
      for (v = 0; v < defbase_tot; v++) {
        /* Get vertex group. */
        defgroup = static_cast<bDeformGroup *>(BLI_findlink(&gpd->vertex_group_names, v));
        if (defgroup == nullptr) {
          continue;
        }

        /* Get weight in vertex group. */
        dw = BKE_defvert_find_index(dvert, v);
        if (dw == nullptr) {
          continue;
        }
        sum += dw->weight;

        /* Vertex group locked or unlocked? */
        if ((defgroup->flag & DG_LOCK_WEIGHT) || ((lock_active) && (v == def_nr))) {
          sum_lock += dw->weight;
        }
        else {
          sum_unlock += dw->weight;
        }
      }

      if ((sum == 1.0f) || (sum_unlock == 0.0f)) {
        continue;
      }

      /* Normalize weights. */
      float fac = std::max(0.0f, (1.0f - sum_lock) / sum_unlock);

      for (v = 0; v < defbase_tot; v++) {
        /* Get vertex group. */
        defgroup = static_cast<bDeformGroup *>(BLI_findlink(&gpd->vertex_group_names, v));
        if (defgroup == nullptr) {
          continue;
        }

        /* Get weight in vertex group. */
        dw = BKE_defvert_find_index(dvert, v);
        if (dw == nullptr) {
          continue;
        }

        /* Normalize in unlocked vertex groups only. */
        if (!((defgroup->flag & DG_LOCK_WEIGHT) || ((lock_active) && (v == def_nr)))) {
          dw->weight *= fac;
          CLAMP(dw->weight, 0.0f, 1.0f);
        }
      }
    }
  }
  CTX_DATA_END;

  /* notifiers */
  DEG_id_tag_update(&gpd->id, ID_RECALC_TRANSFORM | ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_GPENCIL | ND_DATA | NA_EDITED | ND_SPACE_PROPERTIES, nullptr);

  return OPERATOR_FINISHED;
}

void GPENCIL_OT_vertex_group_normalize_all(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Normalize All Vertex Group";
  ot->idname = "GPENCIL_OT_vertex_group_normalize_all";
  ot->description =
      "Normalize all weights of all vertex groups, "
      "so that for each vertex, the sum of all weights is 1.0";

  /* api callbacks */
  ot->poll = gpencil_vertex_group_weight_poll;
  ot->exec = gpencil_vertex_group_normalize_all_exec;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  /* props */
  RNA_def_boolean(ot->srna,
                  "lock_active",
                  true,
                  "Lock Active",
                  "Keep the values of the active group while normalizing others");
}

/****************************** Join ***********************************/

/**
 * Callback to pass to #BKE_fcurves_main_cb()
 * for RNA Paths attached to each F-Curve used in the #AnimData.
 */
static void gpencil_joined_fix_animdata_cb(
    ID *id, FCurve *fcu, bGPdata *src_gpd, bGPdata *tar_gpd, GHash *names_map)
{
  ID *src_id = &src_gpd->id;
  ID *dst_id = &tar_gpd->id;

  GHashIterator gh_iter;

  /* Fix paths - If this is the target datablock, it will have some "dirty" paths */
  if ((id == src_id) && fcu->rna_path && strstr(fcu->rna_path, "layers[")) {
    GHASH_ITER (gh_iter, names_map) {
      const char *old_name = static_cast<const char *>(BLI_ghashIterator_getKey(&gh_iter));
      const char *new_name = static_cast<const char *>(BLI_ghashIterator_getValue(&gh_iter));

      /* only remap if changed;
       * this still means there will be some waste if there aren't many drivers/keys */
      if (!STREQ(old_name, new_name) && strstr(fcu->rna_path, old_name)) {
        fcu->rna_path = BKE_animsys_fix_rna_path_rename(
            id, fcu->rna_path, "layers", old_name, new_name, 0, 0, false);

        /* We don't want to apply a second remapping on this F-Curve now,
         * so stop trying to fix names. */
        break;
      }
    }
  }

  /* Fix driver targets */
  if (fcu->driver) {
    /* Fix driver references to invalid ID's */
    LISTBASE_FOREACH (DriverVar *, dvar, &fcu->driver->variables) {
      /* Only change the used targets, since the others will need fixing manually anyway. */
      DRIVER_TARGETS_USED_LOOPER_BEGIN (dvar) {
        /* Change the ID's used. */
        if (dtar->id == src_id) {
          dtar->id = dst_id;

          /* Also check on the sub-target.
           * We duplicate the logic from #drivers_path_rename_fix() here, with our own
           * little twists so that we know that it isn't going to clobber the wrong data
           */
          if (dtar->rna_path && strstr(dtar->rna_path, "layers[")) {
            GHASH_ITER (gh_iter, names_map) {
              const char *old_name = static_cast<const char *>(BLI_ghashIterator_getKey(&gh_iter));
              const char *new_name = static_cast<const char *>(
                  BLI_ghashIterator_getValue(&gh_iter));

              /* Only remap if changed. */
              if (!STREQ(old_name, new_name)) {
                if ((dtar->rna_path) && strstr(dtar->rna_path, old_name)) {
                  /* Fix up path */
                  dtar->rna_path = BKE_animsys_fix_rna_path_rename(
                      id, dtar->rna_path, "layers", old_name, new_name, 0, 0, false);
                  break; /* no need to try any more names for layer path */
                }
              }
            }
          }
        }
      }
      DRIVER_TARGETS_LOOPER_END;
    }
  }
}

int ED_gpencil_join_objects_exec(bContext *C, wmOperator *op)
{
  Main *bmain = CTX_data_main(C);
  Scene *scene = CTX_data_scene(C);
  Depsgraph *depsgraph = CTX_data_ensure_evaluated_depsgraph(C);
  Object *ob_active = CTX_data_active_object(C);
  bool ok = false;

  /* Ensure we're in right mode and that the active object is correct */
  if (!ob_active || ob_active->type != OB_GPENCIL_LEGACY) {
    return OPERATOR_CANCELLED;
  }

  bGPdata *gpd = (bGPdata *)ob_active->data;
  if ((!gpd) || GPENCIL_ANY_MODE(gpd)) {
    return OPERATOR_CANCELLED;
  }

  /* Ensure all rotations are applied before */
  CTX_DATA_BEGIN (C, Object *, ob_iter, selected_editable_objects) {
    if (ob_iter->type == OB_GPENCIL_LEGACY) {
      if ((ob_iter->rot[0] != 0) || (ob_iter->rot[1] != 0) || (ob_iter->rot[2] != 0)) {
        BKE_report(op->reports, RPT_ERROR, "Apply all rotations before join objects");
        return OPERATOR_CANCELLED;
      }
    }
  }
  CTX_DATA_END;

  CTX_DATA_BEGIN (C, Object *, ob_iter, selected_editable_objects) {
    if (ob_iter == ob_active) {
      ok = true;
      break;
    }
  }
  CTX_DATA_END;

  /* that way the active object is always selected */
  if (ok == false) {
    BKE_report(op->reports, RPT_WARNING, "Active object is not a selected grease pencil");
    return OPERATOR_CANCELLED;
  }

  bGPdata *gpd_dst = static_cast<bGPdata *>(ob_active->data);
  Object *ob_dst = ob_active;

  /* loop and join all data */
  CTX_DATA_BEGIN (C, Object *, ob_iter, selected_editable_objects) {
    if ((ob_iter->type == OB_GPENCIL_LEGACY) && (ob_iter != ob_active)) {
      /* we assume that each datablock is not already used in active object */
      if (ob_active->data != ob_iter->data) {
        Object *ob_src = ob_iter;
        bGPdata *gpd_src = static_cast<bGPdata *>(ob_iter->data);

        /* copy vertex groups to the base one's */
        int old_idx = 0;
        LISTBASE_FOREACH (bDeformGroup *, dg, &gpd_src->vertex_group_names) {
          bDeformGroup *vgroup = static_cast<bDeformGroup *>(MEM_dupallocN(dg));
          int idx = BLI_listbase_count(&gpd_dst->vertex_group_names);
          BKE_object_defgroup_unique_name(vgroup, ob_active);
          BLI_addtail(&gpd_dst->vertex_group_names, vgroup);
          /* update vertex groups in strokes in original data */
          LISTBASE_FOREACH (bGPDlayer *, gpl_src, &gpd->layers) {
            LISTBASE_FOREACH (bGPDframe *, gpf, &gpl_src->frames) {
              LISTBASE_FOREACH (bGPDstroke *, gps, &gpf->strokes) {
                MDeformVert *dvert;
                int i;
                if (gps->dvert == nullptr) {
                  continue;
                }
                for (i = 0, dvert = gps->dvert; i < gps->totpoints; i++, dvert++) {
                  if ((dvert->dw != nullptr) && (dvert->dw->def_nr == old_idx)) {
                    dvert->dw->def_nr = idx;
                  }
                }
              }
            }
          }
          old_idx++;
        }
        if (!BLI_listbase_is_empty(&gpd_dst->vertex_group_names) &&
            gpd_dst->vertex_group_active_index == 0)
        {
          gpd_dst->vertex_group_active_index = 1;
        }

        /* add missing materials reading source materials and checking in destination object */
        short *totcol = BKE_object_material_len_p(ob_src);

        for (short i = 0; i < *totcol; i++) {
          Material *tmp_ma = BKE_gpencil_material(ob_src, i + 1);
          BKE_gpencil_object_material_ensure(bmain, ob_dst, tmp_ma);
        }

        /* Duplicate #bGPDlayers. */
        GHash *names_map = BLI_ghash_str_new("joined_gp_layers_map");

        float imat[3][3], bmat[3][3];
        float offset_global[3];
        float offset_local[3];

        sub_v3_v3v3(offset_global, ob_active->loc, ob_iter->object_to_world().location());
        copy_m3_m4(bmat, ob_active->object_to_world().ptr());

        /* Inverse transform for all selected curves in this object,
         * See #object_join_exec for detailed comment on why the safe version is used. */
        invert_m3_m3_safe_ortho(imat, bmat);
        mul_m3_v3(imat, offset_global);
        mul_v3_m3v3(offset_local, imat, offset_global);

        LISTBASE_FOREACH (bGPDlayer *, gpl_src, &gpd_src->layers) {
          bGPDlayer *gpl_new = BKE_gpencil_layer_duplicate(gpl_src, true, true);
          float diff_mat[4][4];
          float inverse_diff_mat[4][4];

          /* recalculate all stroke points */
          BKE_gpencil_layer_transform_matrix_get(depsgraph, ob_iter, gpl_src, diff_mat);
          invert_m4_m4_safe_ortho(inverse_diff_mat, diff_mat);

          Material *ma_src = nullptr;
          LISTBASE_FOREACH (bGPDframe *, gpf, &gpl_new->frames) {
            LISTBASE_FOREACH (bGPDstroke *, gps, &gpf->strokes) {

              /* Reassign material. Look old material and try to find in destination. */
              ma_src = BKE_gpencil_material(ob_src, gps->mat_nr + 1);
              gps->mat_nr = BKE_gpencil_object_material_ensure(bmain, ob_dst, ma_src);

              bGPDspoint *pt;
              int i;
              for (i = 0, pt = gps->points; i < gps->totpoints; i++, pt++) {
                float mpt[3];
                mul_v3_m4v3(mpt, inverse_diff_mat, &pt->x);
                sub_v3_v3(mpt, offset_local);
                mul_v3_m4v3(&pt->x, diff_mat, mpt);
              }
            }
          }

          /* be sure name is unique in new object */
          BLI_uniquename(&gpd_dst->layers,
                         gpl_new,
                         DATA_("GP_Layer"),
                         '.',
                         offsetof(bGPDlayer, info),
                         sizeof(gpl_new->info));
          BLI_ghash_insert(names_map, BLI_strdup(gpl_src->info), gpl_new->info);

          /* add to destination datablock */
          BLI_addtail(&gpd_dst->layers, gpl_new);
        }

        /* Fix all the animation data */
        BKE_fcurves_main_cb(bmain, [&](ID *id, FCurve *fcu) {
          gpencil_joined_fix_animdata_cb(id, fcu, gpd_src, gpd_dst, names_map);
        });
        BLI_ghash_free(names_map, MEM_freeN, nullptr);

        /* Only copy over animdata now, after all the remapping has been done,
         * so that we don't have to worry about ambiguities re which datablock
         * a layer came from!
         */
        if (ob_iter->adt) {
          if (ob_active->adt == nullptr) {
            /* no animdata, so just use a copy of the whole thing */
            ob_active->adt = BKE_animdata_copy(bmain, ob_iter->adt, 0);
          }
          else {
            /* merge in data - we'll fix the drivers manually */
            BKE_animdata_merge_copy(
                bmain, &ob_active->id, &ob_iter->id, ADT_MERGECOPY_KEEP_DST, false);
          }
        }

        if (gpd_src->adt) {
          if (gpd_dst->adt == nullptr) {
            /* no animdata, so just use a copy of the whole thing */
            gpd_dst->adt = BKE_animdata_copy(bmain, gpd_src->adt, 0);
          }
          else {
            /* merge in data - we'll fix the drivers manually */
            BKE_animdata_merge_copy(
                bmain, &gpd_dst->id, &gpd_src->id, ADT_MERGECOPY_KEEP_DST, false);
          }
        }
        DEG_id_tag_update(&gpd_src->id,
                          ID_RECALC_TRANSFORM | ID_RECALC_GEOMETRY | ID_RECALC_SYNC_TO_EVAL);
      }

      /* Free the old object */
      blender::ed::object::base_free_and_unlink(bmain, scene, ob_iter);
    }
  }
  CTX_DATA_END;

  DEG_id_tag_update(&gpd_dst->id,
                    ID_RECALC_TRANSFORM | ID_RECALC_GEOMETRY | ID_RECALC_SYNC_TO_EVAL);
  DEG_relations_tag_update(bmain); /* because we removed object(s) */

  WM_event_add_notifier(C, NC_SCENE | ND_OB_ACTIVE, scene);
  WM_event_add_notifier(C, NC_SCENE | ND_LAYER_CONTENT, scene);

  return OPERATOR_FINISHED;
}

/* **************** Lock and hide any color non used in current layer **************************
 */
static int gpencil_lock_layer_exec(bContext *C, wmOperator * /*op*/)
{
  bGPdata *gpd = ED_gpencil_data_get_active(C);
  Object *ob = CTX_data_active_object(C);
  MaterialGPencilStyle *gp_style = nullptr;

  /* sanity checks */
  if (ELEM(nullptr, gpd)) {
    return OPERATOR_CANCELLED;
  }

  /* first lock and hide all colors */
  Material *ma = nullptr;
  short *totcol = BKE_object_material_len_p(ob);
  if (totcol == nullptr) {
    return OPERATOR_CANCELLED;
  }

  for (short i = 0; i < *totcol; i++) {
    ma = BKE_gpencil_material(ob, i + 1);
    if (ma) {
      gp_style = ma->gp_style;
      gp_style->flag |= GP_MATERIAL_LOCKED;
      gp_style->flag |= GP_MATERIAL_HIDE;
      DEG_id_tag_update(&ma->id, ID_RECALC_SYNC_TO_EVAL);
    }
  }

  /* loop all selected strokes and unlock any color used in active layer */
  LISTBASE_FOREACH (bGPDlayer *, gpl, &gpd->layers) {
    /* only editable and visible layers are considered */
    if (BKE_gpencil_layer_is_editable(gpl) && (gpl->actframe != nullptr) &&
        (gpl->flag & GP_LAYER_ACTIVE))
    {
      LISTBASE_FOREACH_BACKWARD (bGPDstroke *, gps, &gpl->actframe->strokes) {
        /* skip strokes that are invalid for current view */
        if (ED_gpencil_stroke_can_use(C, gps) == false) {
          continue;
        }

        ma = BKE_gpencil_material(ob, gps->mat_nr + 1);
        DEG_id_tag_update(&ma->id, ID_RECALC_SYNC_TO_EVAL);

        gp_style = ma->gp_style;
        /* unlock/unhide color if not unlocked before */
        if (gp_style != nullptr) {
          gp_style->flag &= ~GP_MATERIAL_LOCKED;
          gp_style->flag &= ~GP_MATERIAL_HIDE;
        }
      }
    }
  }
  /* updates */
  DEG_id_tag_update(&gpd->id, ID_RECALC_GEOMETRY);

  /* Copy-on-eval tag is needed, or else no refresh happens */
  DEG_id_tag_update(&gpd->id, ID_RECALC_SYNC_TO_EVAL);

  /* notifiers */
  DEG_id_tag_update(&gpd->id, ID_RECALC_TRANSFORM | ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_GPENCIL | ND_DATA | NA_EDITED, nullptr);

  return OPERATOR_FINISHED;
}

void GPENCIL_OT_lock_layer(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Disable Unused Layer Colors";
  ot->idname = "GPENCIL_OT_lock_layer";
  ot->description = "Lock and hide any color not used in any layer";

  /* api callbacks */
  ot->exec = gpencil_lock_layer_exec;
  ot->poll = gpencil_active_layer_poll;
}

/* ********************* Append Materials in a new object ************************** */
static bool gpencil_materials_copy_to_object_poll(bContext *C)
{
  Object *ob = CTX_data_active_object(C);
  if ((ob == nullptr) || (ob->type != OB_GPENCIL_LEGACY)) {
    return false;
  }
  short *totcolp = BKE_object_material_len_p(ob);
  if (*totcolp == 0) {
    return false;
  }

  return true;
}

static int gpencil_materials_copy_to_object_exec(bContext *C, wmOperator *op)
{
  Main *bmain = CTX_data_main(C);
  const bool only_active = RNA_boolean_get(op->ptr, "only_active");
  Object *ob_src = CTX_data_active_object(C);
  Material *ma_active = BKE_gpencil_material(ob_src, ob_src->actcol);

  CTX_DATA_BEGIN (C, Object *, ob, selected_objects) {
    if ((ob == ob_src) || (ob->type != OB_GPENCIL_LEGACY)) {
      continue;
    }
    /* Duplicate materials. */
    for (int i = 0; i < ob_src->totcol; i++) {
      Material *ma_src = BKE_object_material_get(ob_src, i + 1);
      if (only_active && ma_src != ma_active) {
        continue;
      }

      if (ma_src != nullptr) {
        BKE_gpencil_object_material_ensure(bmain, ob, ma_src);
      }
    }

    /* notifiers */
    DEG_id_tag_update(&ob->id, ID_RECALC_SYNC_TO_EVAL);
  }
  CTX_DATA_END;

  /* notifiers */
  WM_event_add_notifier(C, NC_GPENCIL | ND_DATA | NA_EDITED, nullptr);

  return OPERATOR_FINISHED;
}

void GPENCIL_OT_materials_copy_to_object(wmOperatorType *ot)
{
  PropertyRNA *prop;

  /* identifiers */
  ot->name = "Copy Materials to Selected Object";
  ot->idname = "GPENCIL_OT_materials_copy_to_object";
  ot->description = "Append Materials of the active Grease Pencil to other object";

  /* callbacks */
  ot->exec = gpencil_materials_copy_to_object_exec;
  ot->poll = gpencil_materials_copy_to_object_poll;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  prop = RNA_def_boolean(ot->srna,
                         "only_active",
                         true,
                         "Only Active",
                         "Append only active material, uncheck to append all materials");
  RNA_def_property_translation_context(prop, BLT_I18NCONTEXT_ID_GPENCIL);
  RNA_def_property_flag(prop, PROP_HIDDEN | PROP_SKIP_SAVE);
}
