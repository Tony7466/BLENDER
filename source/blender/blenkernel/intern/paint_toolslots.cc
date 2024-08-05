/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include <climits>

#include "MEM_guardedalloc.h"

#include "DNA_asset_types.h"
#include "DNA_brush_types.h"
#include "DNA_scene_types.h"

#include "BLI_utildefines.h"

#include "BKE_brush.hh"
#include "BKE_lib_id.hh"
#include "BKE_main.hh"
#include "BKE_paint.hh"

/* -------------------------------------------------------------------- */
/** \name Tool Slot Initialization / Versioning
 *
 * These functions run to update old files (while versioning),
 * take care only to perform low-level functions here.
 * \{ */

void BKE_paint_toolslots_len_ensure(Paint *paint, int len)
{
  /* Tool slots are 'uchar'. */
  BLI_assert(len <= UCHAR_MAX);
  if (paint->tool_brushes_len < len) {
    paint->tool_brushes = static_cast<PaintToolSlot *>(
        MEM_recallocN(paint->tool_brushes, sizeof(*paint->tool_brushes) * len));
    paint->tool_brushes_len = len;
  }
}

#if 0
static void paint_toolslots_init(Main *bmain, Paint *paint)
{
  if (paint == nullptr) {
    return;
  }
  const eObjectMode ob_mode = eObjectMode(paint->runtime.ob_mode);
  for (Brush *brush = static_cast<Brush *>(bmain->brushes.first); brush;
       brush = static_cast<Brush *>(brush->id.next))
  {
    if (brush->ob_mode & ob_mode) {
      const int slot_index = BKE_brush_tool_get(brush, paint);
      BKE_paint_toolslots_len_ensure(paint, slot_index + 1);
      if (paint->tool_brushes[slot_index].brush == nullptr) {
        paint->tool_brushes[slot_index].brush = brush;
        id_us_plus(&brush->id);
      }
    }
  }
}

/**
 * Initialize runtime since this is called from versioning code.
 */
static void paint_toolslots_init_with_runtime(Main *bmain, ToolSettings *ts, Paint *paint)
{
  if (paint == nullptr) {
    return;
  }

  /* Needed so #Paint_Runtime is updated when versioning. */
  BKE_paint_runtime_init(ts, paint);
  BLI_assert(paint->runtime.initialized);
  paint_toolslots_init(bmain, paint);
}

void BKE_paint_toolslots_init_from_main(Main *bmain)
{
  for (Scene *scene = static_cast<Scene *>(bmain->scenes.first); scene;
       scene = static_cast<Scene *>(scene->id.next))
  {
    ToolSettings *ts = scene->toolsettings;
    paint_toolslots_init_with_runtime(bmain, ts, &ts->imapaint.paint);
    if (ts->sculpt) {
      paint_toolslots_init_with_runtime(bmain, ts, &ts->sculpt->paint);
    }
    if (ts->vpaint) {
      paint_toolslots_init_with_runtime(bmain, ts, &ts->vpaint->paint);
    }
    if (ts->wpaint) {
      paint_toolslots_init_with_runtime(bmain, ts, &ts->wpaint->paint);
    }
    if (ts->gp_paint) {
      paint_toolslots_init_with_runtime(bmain, ts, &ts->gp_paint->paint);
    }
    if (ts->gp_vertexpaint) {
      paint_toolslots_init_with_runtime(bmain, ts, &ts->gp_vertexpaint->paint);
    }
    if (ts->gp_sculptpaint) {
      paint_toolslots_init_with_runtime(bmain, ts, &ts->gp_sculptpaint->paint);
    }
    if (ts->gp_weightpaint) {
      paint_toolslots_init_with_runtime(bmain, ts, &ts->gp_weightpaint->paint);
    }
    if (ts->curves_sculpt) {
      paint_toolslots_init_with_runtime(bmain, ts, &ts->curves_sculpt->paint);
    }
  }
}

#endif

/** \} */

void BKE_paint_toolslots_brush_update_ex(Paint *paint,
                                         const Brush *brush,
                                         const AssetWeakReference *brush_asset_reference)
{
  std::optional<int> slot_index = BKE_paint_get_brush_tool_from_obmode(
      brush, eObjectMode(paint->runtime.ob_mode));

  BKE_paint_toolslots_len_ensure(paint, *slot_index + 1);
  PaintToolSlot *tslot = &paint->tool_brushes[*slot_index];
  if (tslot->brush_asset_reference) {
    MEM_delete(tslot->brush_asset_reference);
  }
  tslot->brush_asset_reference = MEM_new<AssetWeakReference>(__func__, *brush_asset_reference);
}

void BKE_paint_toolslots_brush_update(Paint *paint)
{
  if (paint->brush == nullptr) {
    return;
  }
  BKE_paint_toolslots_brush_update_ex(paint, paint->brush, paint->brush_asset_reference);
}

void BKE_paint_brush_validate(Main *bmain, Paint *paint)
{
#if 0
  std::optional<int> slot_index = BKE_paint_get_brush_tool_from_obmode(
      brush, eObjectMode(paint->runtime.ob_mode));

  /* Clear slots with invalid slots or mode (unlikely but possible). */
  const eObjectMode ob_mode = eObjectMode(paint->runtime.ob_mode);
  BLI_assert(ob_mode);
  for (int i = 0; i < paint->tool_brushes_len; i++) {
    PaintToolSlot *tslot = &paint->tool_brushes[i];
    if (tslot->brush_asset_reference) {
      if ((i != BKE_brush_tool_get(tslot->brush, paint)) || (tslot->brush->ob_mode & ob_mode) == 0)
      {
        tslot->brush_asset_reference = nullptr;
      }
    }
  }
#endif

  /* Unlikely but possible the active brush is not currently using a slot. */
  BKE_paint_toolslots_brush_update(paint);

  /* Fill slots from brushes. */
  // paint_toolslots_init(bmain, paint);
}

AssetWeakReference *BKE_paint_toolslots_brush_asset_reference_get(Paint *paint, int slot_index)
{
  if (slot_index < paint->tool_brushes_len) {
    PaintToolSlot *tslot = &paint->tool_brushes[slot_index];
    return tslot->brush_asset_reference;
  }
  return nullptr;
}
