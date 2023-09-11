/* SPDX-FileCopyrightText: 2008 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edinterface
 *
 * General Interface Region Code
 *
 * \note Most logic is now in 'interface_region_*.c'
 */

#include <functional>

#include "BLI_listbase.h"
#include "BLI_utildefines.h"
#include "MEM_guardedalloc.h"

#include "BKE_context.h"
#include "BKE_screen.h"

#include "WM_api.hh"
#include "wm_draw.hh"

#include "ED_screen.hh"

#include "interface_intern.hh"

#include "interface_regions_intern.hh"

/* -------------------------------------------------------------------- */
/** \name Temporary regions
 * \{ */

ARegion *ui_region_temp_add(bScreen *screen)
{
  ARegion *region = MEM_cnew<ARegion>(__func__);
  BLI_addtail(&screen->regionbase, region);

  region->regiontype = RGN_TYPE_TEMPORARY;
  region->alignment = RGN_ALIGN_FLOAT;

  return region;
}

void ui_region_temp_remove(bContext *C, bScreen *screen, ARegion *region)
{
  wmWindow *win = CTX_wm_window(C);

  BLI_assert(region->regiontype == RGN_TYPE_TEMPORARY);
  BLI_assert(BLI_findindex(&screen->regionbase, region) != -1);
  if (win) {
    wm_draw_region_clear(win, region);
  }

  ED_region_exit(C, region);
  BKE_area_region_free(nullptr, region); /* nullptr: no space-type. */
  BLI_freelinkN(&screen->regionbase, region);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Button sections
 * \{ */

/**
 * Calculate a bounding box for each "section", that is, each group of buttons separated by a
 * separator spacer button. Sections will be merged if they are closer than
 * #UI_BUTTON_SECTION_MERGE_DISTANCE.
 *
 * If a section is closer than #UI_BUTTON_SECTION_MERGE_DISTANCE to a region edge, it will be
 * extended to the edge.
 *
 * \return the bounding boxes in region space.
 */
static blender::Vector<rcti> button_section_bounds_calc(const ARegion *region)
{
  blender::Vector<rcti> section_bounds;

  const std::function finish_section_fn = [&section_bounds, region](rcti cur_section_bounds) {
    if (!section_bounds.is_empty() &&
        std::abs(section_bounds.last().xmax - cur_section_bounds.xmin) <
            UI_BUTTON_SECTION_MERGE_DISTANCE)
    {
      section_bounds.last().xmax = cur_section_bounds.xmax;
    }
    else {
      section_bounds.append(cur_section_bounds);
    }

    rcti &last_bounds = section_bounds.last();
    /* Extend to region edge if close enough. */
    if (last_bounds.xmin <= UI_BUTTON_SECTION_MERGE_DISTANCE) {
      last_bounds.xmin = 0;
    }
    if (last_bounds.xmax >= (region->winx - UI_BUTTON_SECTION_MERGE_DISTANCE)) {
      last_bounds.xmax = region->winx;
    }
  };

  {
    bool has_section_content = false;
    rcti cur_section_bounds;
    BLI_rcti_init_minmax(&cur_section_bounds);

    LISTBASE_FOREACH (uiBlock *, block, &region->uiblocks) {
      LISTBASE_FOREACH (uiBut *, but, &block->buttons) {
        if (but->type == UI_BTYPE_SEPR_SPACER) {
          /* Start a new section. */
          if (has_section_content) {
            finish_section_fn(cur_section_bounds);

            /* Reset for next section. */
            BLI_rcti_init_minmax(&cur_section_bounds);
            has_section_content = false;
          }
          continue;
        }

        rcti but_pixelrect;
        ui_but_to_pixelrect(&but_pixelrect, region, block, but);
        BLI_rcti_do_minmax_rcti(&cur_section_bounds, &but_pixelrect);
        has_section_content = true;
      }
    }

    /* Finish last section in case the last button is not a spacer. */
    if (has_section_content) {
      finish_section_fn(cur_section_bounds);
    }
  }

  return section_bounds;
}

void UI_region_button_sections_draw(const ARegion *region,
                                    const int /*ThemeColorID*/ colorid,
                                    const uiButtonSectionsAlign align)
{
  const uiStyle *style = UI_style_get_dpi();
  const float aspect = BLI_rctf_size_x(&region->v2d.cur) /
                       (BLI_rcti_size_x(&region->v2d.mask) + 1);
  const float corner_radius = 4.0f * UI_SCALE_FAC / aspect;

  const blender::Vector<rcti> section_bounds = button_section_bounds_calc(region);

  blender::Vector<rcti> padded_bounds;
  for (rcti bounds_copy : section_bounds) {
    BLI_rcti_pad(&bounds_copy, style->buttonspacex, style->buttonspacey);
    /* Clamp, important for the rounded-corners to draw correct. */
    CLAMP_MIN(bounds_copy.xmin, 0);
    CLAMP_MAX(bounds_copy.xmax, region->winx);
    CLAMP_MIN(bounds_copy.ymin, 0);
    CLAMP_MAX(bounds_copy.ymax, region->winy);
    padded_bounds.append(bounds_copy);
  }

  ui_draw_button_sections_background(
      region, padded_bounds, ThemeColorID(colorid), align, corner_radius);
  if (align != uiButtonSectionsAlign::None) {
    ui_draw_button_sections_alignment_separator(region,
                                                padded_bounds,
                                                ThemeColorID(colorid),
                                                align,
                                                /* Slightly bigger corner radius, looks better. */
                                                corner_radius + 1);
  }
}

/** \} */
