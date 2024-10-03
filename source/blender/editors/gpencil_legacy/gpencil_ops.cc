/* SPDX-FileCopyrightText: 2009 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgpencil
 */

#include <cstddef>
#include <cstdio>
#include <cstdlib>

#include "BLI_sys_types.h"

#include "BKE_context.hh"
#include "BKE_paint.hh"

#include "DNA_brush_types.h"
#include "DNA_gpencil_legacy_types.h"
#include "DNA_object_types.h"
#include "DNA_screen_types.h"
#include "DNA_space_types.h"

#include "WM_api.hh"
#include "WM_toolsystem.hh"
#include "WM_types.hh"

#include "RNA_access.hh"

#include "ED_gpencil_legacy.hh"

#include "gpencil_intern.hh"

/* ****************************************** */
/* Grease Pencil Keymaps */

/* Generic Drawing Keymap - Annotations */
static void ed_keymap_gpencil_general(wmKeyConfig *keyconf)
{
  WM_keymap_ensure(keyconf, "Grease Pencil", SPACE_EMPTY, RGN_TYPE_WINDOW);
}

/* ==================== */

void ED_keymap_gpencil_legacy(wmKeyConfig *keyconf)
{
  ed_keymap_gpencil_general(keyconf);
}

/* ****************************************** */

void ED_operatortypes_gpencil_legacy()
{
  /* Annotations -------------------- */

  WM_operatortype_append(GPENCIL_OT_annotate);
  WM_operatortype_append(GPENCIL_OT_annotation_add);
  WM_operatortype_append(GPENCIL_OT_data_unlink);
  WM_operatortype_append(GPENCIL_OT_layer_annotation_add);
  WM_operatortype_append(GPENCIL_OT_layer_annotation_remove);
  WM_operatortype_append(GPENCIL_OT_layer_annotation_move);
  WM_operatortype_append(GPENCIL_OT_blank_frame_add);
  WM_operatortype_append(GPENCIL_OT_annotation_active_frame_delete);

  WM_operatortype_append(GPENCIL_OT_transform_fill);
  WM_operatortype_append(GPENCIL_OT_reset_transform_fill);

  /* color handle */
  WM_operatortype_append(GPENCIL_OT_lock_layer);

  /* Editing (Time) --------------- */

  /* Interpolation */
  WM_operatortype_append(GPENCIL_OT_interpolate);
  WM_operatortype_append(GPENCIL_OT_interpolate_sequence);
  WM_operatortype_append(GPENCIL_OT_interpolate_reverse);

  /* Primitives */
  WM_operatortype_append(GPENCIL_OT_primitive_box);
  WM_operatortype_append(GPENCIL_OT_primitive_line);
  WM_operatortype_append(GPENCIL_OT_primitive_polyline);
  WM_operatortype_append(GPENCIL_OT_primitive_circle);
  WM_operatortype_append(GPENCIL_OT_primitive_curve);

  /* convert old 2.7 files to 2.8 */
  WM_operatortype_append(GPENCIL_OT_convert_old_files);

  /* armatures */
  WM_operatortype_append(GPENCIL_OT_generate_weights);
}

void ED_operatormacros_gpencil()
{
}

/* ****************************************** */
