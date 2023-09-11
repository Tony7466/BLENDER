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
 * separator spacer button. Sections will be merged if they are closer than \a merge_distance_x.
 *
 * \param merge_distance_x: Bounds that are closer horizontally than this will be merged. Optional,
 *                          set to 0 to enable. Must be >= 0.
 * \return the bounding boxes in region space.
 */
static blender::Vector<rcti> calc_button_section_bounds(const ARegion *region,
                                                        const int merge_distance_x)
{
  blender::Vector<rcti> section_bounds;

  BLI_assert(merge_distance_x >= 0);

  const std::function finish_section_fn = [&section_bounds,
                                           merge_distance_x](const rcti cur_section_bounds) {
    if (merge_distance_x && !section_bounds.is_empty() &&
        std::abs(section_bounds.last().xmax - cur_section_bounds.xmin) < merge_distance_x)
    {
      section_bounds.last().xmax = cur_section_bounds.xmax;
    }
    else {
      section_bounds.append(cur_section_bounds);
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
                                    int /*THemeColorID*/ colorid,
                                    const uiButtonSectionsAlign align)
{
  const float merge_distance_x = UI_BUTTON_SECTION_MERGE_DISTANCE;

  const blender::Vector<rcti> section_bounds = calc_button_section_bounds(region,
                                                                          merge_distance_x);
  ui_draw_button_sections_background_and_separator(
      region, section_bounds, colorid, merge_distance_x, align);
}

/** \} */
