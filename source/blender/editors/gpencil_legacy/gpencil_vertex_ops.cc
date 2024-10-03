/* SPDX-FileCopyrightText: 2015 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgpencil
 * Brush based operators for editing Grease Pencil strokes.
 */

#include "MEM_guardedalloc.h"

#include "BLI_blenlib.h"
#include "BLI_ghash.h"
#include "BLI_math_color.h"
#include "BLI_math_vector.h"

#include "DNA_brush_types.h"
#include "DNA_gpencil_legacy_types.h"
#include "DNA_material_types.h"

#include "BKE_context.hh"
#include "BKE_material.h"
#include "BKE_paint.hh"
#include "BKE_report.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"

#include "ED_gpencil_legacy.hh"
#include "ED_screen.hh"

#include "DEG_depsgraph.hh"

#include "gpencil_intern.hh"

/* Helper to extract color from vertex color to create a palette. */
static bool gpencil_extract_palette_from_vertex(bContext *C,
                                                const bool selected,
                                                const int threshold)
{
  Main *bmain = CTX_data_main(C);
  Object *ob = CTX_data_active_object(C);
  bool done = false;
  const float range = pow(10.0f, threshold);
  float col[3];

  GHash *color_table = BLI_ghash_int_new(__func__);

  /* Extract all colors. */
  CTX_DATA_BEGIN (C, bGPDlayer *, gpl, editable_gpencil_layers) {
    LISTBASE_FOREACH (bGPDframe *, gpf, &gpl->frames) {
      LISTBASE_FOREACH (bGPDstroke *, gps, &gpf->strokes) {
        if (ED_gpencil_stroke_can_use(C, gps) == false) {
          continue;
        }
        if (ED_gpencil_stroke_material_editable(ob, gpl, gps) == false) {
          continue;
        }
        MaterialGPencilStyle *gp_style = BKE_gpencil_material_settings(ob, gps->mat_nr + 1);
        if (gp_style == nullptr) {
          continue;
        }

        if ((selected) && ((gps->flag & GP_STROKE_SELECT) == 0)) {
          continue;
        }

        bool use_stroke = (gp_style->flag & GP_MATERIAL_STROKE_SHOW);
        bool use_fill = (gp_style->flag & GP_MATERIAL_FILL_SHOW);

        /* Material is disabled. */
        if ((!use_fill) && (!use_stroke)) {
          continue;
        }

        /* Only solid strokes or stencil. */
        if ((use_stroke) && ((gp_style->stroke_style == GP_MATERIAL_STROKE_STYLE_TEXTURE) &&
                             ((gp_style->flag & GP_MATERIAL_STROKE_PATTERN) == 0)))
        {
          continue;
        }

        /* Only solid fill. */
        if ((use_fill) && (gp_style->fill_style != GP_MATERIAL_FILL_STYLE_SOLID)) {
          continue;
        }

        /* Fill color. */
        if (gps->vert_color_fill[3] > 0.0f) {
          col[0] = truncf(gps->vert_color_fill[0] * range) / range;
          col[1] = truncf(gps->vert_color_fill[1] * range) / range;
          col[2] = truncf(gps->vert_color_fill[2] * range) / range;

          uint key = rgb_to_cpack(col[0], col[1], col[2]);

          if (!BLI_ghash_haskey(color_table, POINTER_FROM_INT(key))) {
            BLI_ghash_insert(color_table, POINTER_FROM_INT(key), POINTER_FROM_INT(key));
          }
        }

        /* Read all points to get all colors. */
        bGPDspoint *pt;
        int i;
        for (i = 0, pt = gps->points; i < gps->totpoints; i++, pt++) {
          col[0] = truncf(pt->vert_color[0] * range) / range;
          col[1] = truncf(pt->vert_color[1] * range) / range;
          col[2] = truncf(pt->vert_color[2] * range) / range;

          uint key = rgb_to_cpack(col[0], col[1], col[2]);
          if (!BLI_ghash_haskey(color_table, POINTER_FROM_INT(key))) {
            BLI_ghash_insert(color_table, POINTER_FROM_INT(key), POINTER_FROM_INT(key));
          }
        }
      }
    }
  }
  CTX_DATA_END;

  /* Create the Palette. */
  done = BKE_palette_from_hash(bmain, color_table, ob->id.name + 2, true);

  /* Free memory. */
  BLI_ghash_free(color_table, nullptr, nullptr);

  return done;
}

/* Extract Palette from Vertex Color. */
static bool gpencil_extract_palette_vertex_poll(bContext *C)
{
  /* only supported with grease pencil objects */
  Object *ob = CTX_data_active_object(C);
  if ((ob == nullptr) || (ob->type != OB_GPENCIL_LEGACY)) {
    return false;
  }

  return true;
}

static int gpencil_extract_palette_vertex_exec(bContext *C, wmOperator *op)
{
  const bool selected = RNA_boolean_get(op->ptr, "selected");
  const int threshold = RNA_int_get(op->ptr, "threshold");

  if (gpencil_extract_palette_from_vertex(C, selected, threshold)) {
    BKE_reportf(op->reports, RPT_INFO, "Palette created");
  }
  else {
    BKE_reportf(op->reports, RPT_ERROR, "Unable to find Vertex Information to create palette");
  }

  return OPERATOR_FINISHED;
}

void GPENCIL_OT_extract_palette_vertex(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Extract Palette from Vertex Color";
  ot->idname = "GPENCIL_OT_extract_palette_vertex";
  ot->description = "Extract all colors used in Grease Pencil Vertex and create a Palette";

  /* api callbacks */
  ot->exec = gpencil_extract_palette_vertex_exec;
  ot->poll = gpencil_extract_palette_vertex_poll;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  /* properties */
  ot->prop = RNA_def_boolean(
      ot->srna, "selected", false, "Only Selected", "Convert only selected strokes");
  RNA_def_int(ot->srna, "threshold", 1, 1, 4, "Threshold", "", 1, 4);
}
