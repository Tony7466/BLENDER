/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_rect.h"

#include "BKE_context.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_paint.hh"

#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "ED_grease_pencil.hh"

namespace blender::ed::greasepencil {

/* Helper: Calc the maximum bounding box size of strokes to get the zoom level of the viewport.
 * For each stroke, the 2D projected bounding box is calculated and using this data, the total
 * object bounding box (all strokes) is calculated. */
static float gpencil_zoom_level_set(const ARegion &region,
                                    const Brush &brush,
                                    Object &ob,
                                    const IndexMask &layer_mask,
                                    const Span<bool> is_boundary_stroke)
{
  using bke::greasepencil::Layer;

  if (brush.gpencil_settings->flag & GP_BRUSH_FILL_FIT_DISABLE) {
    return 1.0f;
  }

  const BrushGpencilSettings &brush_settings = *brush.gpencil_settings;
  BLI_assert(ob.type == OB_GREASE_PENCIL);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(ob.data);

  BLI_assert(grease_pencil.has_active_layer());
  const Layer &layer = *grease_pencil.get_active_layer();

  /* Init maximum boundbox size. */
  rctf rect_max;
  const float winx_half = region.winx / 2.0f;
  const float winy_half = region.winy / 2.0f;
  BLI_rctf_init(&rect_max,
                0.0f - winx_half,
                region.winx + winx_half,
                0.0f - winy_half,
                region.winy + winy_half);

  float objectbox_min[2], objectbox_max[2];
  INIT_MINMAX2(objectbox_min, objectbox_max);
  rctf rect_bound;
  layer_mask.foreach_index([&](const int layer_index) {
    const Layer &layer = *grease_pencil.layers()[layer_index];

    float4x4 layer_to_world = layer.to_world_space(ob);

    /* Get frame to check. */
    bGPDframe *gpf = BKE_gpencil_layer_frame_get(gpl, tgpf->active_cfra, GP_GETFRAME_USE_PREV);
    if (gpf == nullptr) {
      continue;
    }

    /* Read all strokes. */
    LISTBASE_FOREACH (bGPDstroke *, gps, &gpf->strokes) {
      /* check if stroke can be drawn */
      if ((gps->points == nullptr) || (gps->totpoints < 2)) {
        continue;
      }
      /* check if the color is visible */
      MaterialGPencilStyle *gp_style = BKE_gpencil_material_settings(ob, gps->mat_nr + 1);
      if ((gp_style == nullptr) || (gp_style->flag & GP_MATERIAL_HIDE)) {
        continue;
      }

      /* If the layer must be skipped, but the stroke is not boundary, skip stroke. */
      if ((skip) && ((gps->flag & GP_STROKE_NOFILL) == 0)) {
        continue;
      }

      float boundbox_min[2];
      float boundbox_max[2];
      ED_gpencil_projected_2d_bound_box(&tgpf->gsc, gps, diff_mat, boundbox_min, boundbox_max);
      minmax_v2v2_v2(objectbox_min, objectbox_max, boundbox_min);
      minmax_v2v2_v2(objectbox_min, objectbox_max, boundbox_max);
    }
  });
  /* Clamp max bound box. */
  BLI_rctf_init(
      &rect_bound, objectbox_min[0], objectbox_max[0], objectbox_min[1], objectbox_max[1]);
  float r_xy[2];
  BLI_rctf_clamp(&rect_bound, &rect_max, r_xy);

  /* Calculate total width used. */
  float width = tgpf->region->winx;
  if (rect_bound.xmin < 0.0f) {
    width -= rect_bound.xmin;
  }
  if (rect_bound.xmax > tgpf->region->winx) {
    width += rect_bound.xmax - tgpf->region->winx;
  }
  /* Calculate total height used. */
  float height = tgpf->region->winy;
  if (rect_bound.ymin < 0.0f) {
    height -= rect_bound.ymin;
  }
  if (rect_bound.ymax > tgpf->region->winy) {
    height += rect_bound.ymax - tgpf->region->winy;
  }

  width = ceilf(width);
  height = ceilf(height);

  float zoomx = (width > tgpf->region->winx) ? width / float(tgpf->region->winx) : 1.0f;
  float zoomy = (height > tgpf->region->winy) ? height / float(tgpf->region->winy) : 1.0f;
  if ((zoomx != 1.0f) || (zoomy != 1.0f)) {
    tgpf->zoom = min_ff(max_ff(zoomx, zoomy) * 1.5f, 5.0f);
  }
}

bool fill_strokes(bContext &C, const ARegion &region, const IndexMask &layer_mask)
{
  wmWindow &win = *CTX_wm_window(&C);
  const ToolSettings &ts = *CTX_data_tool_settings(&C);
  const Brush &brush = *BKE_paint_brush(&ts.gp_paint->paint);
  //   auto &op_data = *static_cast<GreasePencilFillOpData *>(op.customdata);
  //   const bool extend_lines = (op_data.fill_extend_fac > 0.0f);

  if (region.regiontype != RGN_TYPE_WINDOW) {
    return false;
  }

  // tgpf->mouse[0] = event->mval[0];
  // tgpf->mouse[1] = event->mval[1];
  // tgpf->is_render = true;
  // /* Define Zoom level. */
  // gpencil_zoom_level_set(tgpf);

  // /* Create Temp stroke. */
  // tgpf->gps_mouse = BKE_gpencil_stroke_new(0, 1, 10.0f);
  // tGPspoint point2D;
  // bGPDspoint *pt = &tgpf->gps_mouse->points[0];
  // copy_v2fl_v2i(point2D.m_xy, tgpf->mouse);
  // gpencil_stroke_convertcoords_tpoint(
  //     tgpf->scene, tgpf->region, tgpf->ob, &point2D, nullptr, &pt->x);

  // /* Hash of selected frames. */
  // GHash *frame_list = BLI_ghash_int_new_ex(__func__, 64);

  // /* If not multi-frame and there is no frame in scene->r.cfra for the active layer,
  //  * create a new frame. */
  // if (!is_multiedit) {
  //   tgpf->gpf = BKE_gpencil_layer_frame_get(
  //       tgpf->gpl,
  //       tgpf->active_cfra,
  //       blender::animrig::is_autokey_on(tgpf->scene) ? GP_GETFRAME_ADD_NEW :
  //       GP_GETFRAME_USE_PREV);
  //   tgpf->gpf->flag |= GP_FRAME_SELECT;

  //   BLI_ghash_insert(frame_list, POINTER_FROM_INT(tgpf->active_cfra), tgpf->gpl->actframe);
  // }
  // else {
  //   BKE_gpencil_frame_selected_hash(tgpf->gpd, frame_list);
  // }

  // /* Loop all frames. */
  // wmWindow *win = CTX_wm_window(C);

  // GHashIterator gh_iter;
  // int total = BLI_ghash_len(frame_list);
  // int i = 1;
  // GHASH_ITER (gh_iter, frame_list) {
  //   /* Set active frame as current for filling. */
  //   tgpf->active_cfra = POINTER_AS_INT(BLI_ghashIterator_getKey(&gh_iter));
  //   int step = (float(i) / float(total)) * 100.0f;
  //   WM_cursor_time(win, step);

  //   if (extend_lines) {
  //     grease_pencil_update_extend(C, op_data);
  //   }

  //   /* Repeat loop until get something. */
  //   tgpf->done = false;
  //   int loop_limit = 0;
  //   while ((!tgpf->done) && (loop_limit < 2)) {
  //     WM_cursor_time(win, loop_limit + 1);
  //     /* Render screen to temp image and do fill. */
  //     gpencil_do_frame_fill(tgpf, is_inverted);

  //     /* restore size */
  //     tgpf->region->winx = short(tgpf->bwinx);
  //     tgpf->region->winy = short(tgpf->bwiny);
  //     tgpf->region->winrct = tgpf->brect;
  //     if (!tgpf->done) {
  //       /* If the zoom was not set before, avoid a loop. */
  //       if (tgpf->zoom == 1.0f) {
  //         loop_limit++;
  //       }
  //       else {
  //         tgpf->zoom = 1.0f;
  //         tgpf->fill_factor = max_ff(
  //             GPENCIL_MIN_FILL_FAC,
  //             min_ff(brush->gpencil_settings->fill_factor, GPENCIL_MAX_FILL_FAC));
  //       }
  //     }
  //     loop_limit++;
  //   }

  //   if (extend_lines) {
  //     stroke_array_free(tgpf);
  //     gpencil_delete_temp_stroke_extension(tgpf, true);
  //   }

  //   i++;
  // }

  WM_cursor_modal_restore(&win);
  /* Free hash table. */
  // BLI_ghash_free(frame_list, nullptr, nullptr);

  /* Free temp stroke. */
  // BKE_gpencil_free_stroke(tgpf->gps_mouse);

  /* push undo data */
  // gpencil_undo_push(tgpf->gpd);

  return true;
}

}  // namespace blender::ed::greasepencil
