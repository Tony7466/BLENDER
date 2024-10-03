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

  /* Editing (Buttons) ------------ */


  WM_operatortype_append(GPENCIL_OT_layer_add);
  WM_operatortype_append(GPENCIL_OT_layer_remove);
  WM_operatortype_append(GPENCIL_OT_layer_move);
  WM_operatortype_append(GPENCIL_OT_layer_annotation_add);
  WM_operatortype_append(GPENCIL_OT_layer_annotation_remove);
  WM_operatortype_append(GPENCIL_OT_layer_annotation_move);
  WM_operatortype_append(GPENCIL_OT_layer_duplicate);
  WM_operatortype_append(GPENCIL_OT_layer_duplicate_object);

  WM_operatortype_append(GPENCIL_OT_layer_mask_add);
  WM_operatortype_append(GPENCIL_OT_layer_mask_remove);
  WM_operatortype_append(GPENCIL_OT_layer_mask_move);

  WM_operatortype_append(GPENCIL_OT_hide);
  WM_operatortype_append(GPENCIL_OT_reveal);
  WM_operatortype_append(GPENCIL_OT_lock_all);
  WM_operatortype_append(GPENCIL_OT_unlock_all);
  WM_operatortype_append(GPENCIL_OT_layer_isolate);
  WM_operatortype_append(GPENCIL_OT_layer_merge);

  WM_operatortype_append(GPENCIL_OT_blank_frame_add);

  WM_operatortype_append(GPENCIL_OT_active_frame_delete);
  WM_operatortype_append(GPENCIL_OT_annotation_active_frame_delete);
  WM_operatortype_append(GPENCIL_OT_active_frames_delete_all);
  WM_operatortype_append(GPENCIL_OT_frame_duplicate);
  WM_operatortype_append(GPENCIL_OT_frame_clean_fill);
  WM_operatortype_append(GPENCIL_OT_frame_clean_loose);
  WM_operatortype_append(GPENCIL_OT_frame_clean_duplicate);

  WM_operatortype_append(GPENCIL_OT_convert);
  WM_operatortype_append(GPENCIL_OT_bake_mesh_animation);
  WM_operatortype_append(GPENCIL_OT_bake_grease_pencil_animation);

  WM_operatortype_append(GPENCIL_OT_image_to_grease_pencil);
#ifdef WITH_POTRACE
  WM_operatortype_append(GPENCIL_OT_trace_image);
#endif
  WM_operatortype_append(GPENCIL_OT_stroke_arrange);
  WM_operatortype_append(GPENCIL_OT_stroke_change_color);
  WM_operatortype_append(GPENCIL_OT_material_lock_unused);
  WM_operatortype_append(GPENCIL_OT_stroke_apply_thickness);
  WM_operatortype_append(GPENCIL_OT_stroke_cyclical_set);
  WM_operatortype_append(GPENCIL_OT_stroke_caps_set);
  WM_operatortype_append(GPENCIL_OT_stroke_join);
  WM_operatortype_append(GPENCIL_OT_stroke_flip);
  WM_operatortype_append(GPENCIL_OT_stroke_start_set);
  WM_operatortype_append(GPENCIL_OT_stroke_subdivide);
  WM_operatortype_append(GPENCIL_OT_stroke_simplify);
  WM_operatortype_append(GPENCIL_OT_stroke_simplify_fixed);
  WM_operatortype_append(GPENCIL_OT_stroke_separate);
  WM_operatortype_append(GPENCIL_OT_stroke_split);
  WM_operatortype_append(GPENCIL_OT_stroke_smooth);
  WM_operatortype_append(GPENCIL_OT_stroke_sample);
  WM_operatortype_append(GPENCIL_OT_stroke_merge);
  WM_operatortype_append(GPENCIL_OT_stroke_cutter);
  WM_operatortype_append(GPENCIL_OT_stroke_trim);
  WM_operatortype_append(GPENCIL_OT_stroke_merge_by_distance);
  WM_operatortype_append(GPENCIL_OT_stroke_merge_material);
  WM_operatortype_append(GPENCIL_OT_stroke_reset_vertex_color);
  WM_operatortype_append(GPENCIL_OT_stroke_normalize);
  WM_operatortype_append(GPENCIL_OT_stroke_outline);

  WM_operatortype_append(GPENCIL_OT_material_to_vertex_color);
  WM_operatortype_append(GPENCIL_OT_extract_palette_vertex);
  WM_operatortype_append(GPENCIL_OT_materials_copy_to_object);

  WM_operatortype_append(GPENCIL_OT_transform_fill);
  WM_operatortype_append(GPENCIL_OT_reset_transform_fill);

  /* vertex groups */
  WM_operatortype_append(GPENCIL_OT_vertex_group_assign);
  WM_operatortype_append(GPENCIL_OT_vertex_group_remove_from);
  WM_operatortype_append(GPENCIL_OT_vertex_group_select);
  WM_operatortype_append(GPENCIL_OT_vertex_group_deselect);
  WM_operatortype_append(GPENCIL_OT_vertex_group_invert);
  WM_operatortype_append(GPENCIL_OT_vertex_group_smooth);
  WM_operatortype_append(GPENCIL_OT_vertex_group_normalize);
  WM_operatortype_append(GPENCIL_OT_vertex_group_normalize_all);

  /* color handle */
  WM_operatortype_append(GPENCIL_OT_lock_layer);
  WM_operatortype_append(GPENCIL_OT_material_isolate);
  WM_operatortype_append(GPENCIL_OT_material_hide);
  WM_operatortype_append(GPENCIL_OT_material_reveal);
  WM_operatortype_append(GPENCIL_OT_material_lock_all);
  WM_operatortype_append(GPENCIL_OT_material_unlock_all);
  WM_operatortype_append(GPENCIL_OT_material_select);
  WM_operatortype_append(GPENCIL_OT_material_set);

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
