/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include <climits>

#include "MEM_guardedalloc.h"

#include "BLI_string.h"

#include "DNA_asset_types.h"
#include "DNA_brush_types.h"
#include "DNA_scene_types.h"

#include "BKE_paint.hh"

void BKE_paint_toolslots_brush_update(Paint *paint, const char *brush_type_name)
{
  if (paint->brush == nullptr) {
    return;
  }
  if (!brush_type_name[0]) {
    return;
  }

  NamedBrushAssetReference *brush_ref = static_cast<NamedBrushAssetReference *>(
      BLI_findstring_ptr(&paint->active_brush_per_brush_type,
                         brush_type_name,
                         offsetof(NamedBrushAssetReference, name)));

  /* Update existing reference. */
  if (brush_ref) {
    MEM_delete(brush_ref->brush_asset_reference);
    brush_ref->brush_asset_reference = MEM_new<AssetWeakReference>(
        "NamedBrushAssetRefernce update asset ref", *paint->brush_asset_reference);
  }
  /* Add new reference. */
  else {
    NamedBrushAssetReference *brush_ref = MEM_cnew<NamedBrushAssetReference>(
        "NamedBrushAssetReference");

    brush_ref->name = BLI_strdup(brush_type_name);
    brush_ref->brush_asset_reference = MEM_new<AssetWeakReference>(
        "NamedBrushAssetRefernce new asset ref", *paint->brush_asset_reference);
    BLI_addhead(&paint->active_brush_per_brush_type, brush_ref);
  }
}

AssetWeakReference *BKE_paint_toolslots_brush_asset_reference_get(Paint *paint,
                                                                  const char *brush_type_name)
{
  const NamedBrushAssetReference *brush_ref = static_cast<NamedBrushAssetReference *>(
      BLI_findstring_ptr(&paint->active_brush_per_brush_type,
                         brush_type_name,
                         offsetof(NamedBrushAssetReference, name)));
  if (!brush_ref) {
    return nullptr;
  }

  return brush_ref->brush_asset_reference;
}
