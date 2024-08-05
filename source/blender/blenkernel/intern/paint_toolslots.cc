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

#include "BKE_paint.hh"

static void paint_toolslots_len_ensure(Paint *paint, int len)
{
  /* Tool slots are 'uchar'. */
  BLI_assert(len <= UCHAR_MAX);
  if (paint->tool_brushes_len < len) {
    paint->tool_brushes = static_cast<PaintToolSlot *>(
        MEM_recallocN(paint->tool_brushes, sizeof(*paint->tool_brushes) * len));
    paint->tool_brushes_len = len;
  }
}

static void paint_toolslots_brush_update_ex(Paint *paint,
                                            const Brush *brush,
                                            const AssetWeakReference *brush_asset_reference)
{
  std::optional<int> slot_index = BKE_paint_get_brush_tool_from_obmode(
      brush, eObjectMode(paint->runtime.ob_mode));

  paint_toolslots_len_ensure(paint, *slot_index + 1);
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
  paint_toolslots_brush_update_ex(paint, paint->brush, paint->brush_asset_reference);
}

AssetWeakReference *BKE_paint_toolslots_brush_asset_reference_get(Paint *paint, int slot_index)
{
  if (slot_index < paint->tool_brushes_len) {
    PaintToolSlot *tslot = &paint->tool_brushes[slot_index];
    return tslot->brush_asset_reference;
  }
  return nullptr;
}
