/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edscr
 */

#include "ED_screen.hh"

#include "GPU_batch_presets.h"
#include "GPU_framebuffer.h"
#include "GPU_immediate.h"
#include "GPU_matrix.h"
#include "GPU_platform.h"
#include "GPU_state.h"

#include "BLI_listbase.h"
#include "BLI_math_vector.hh"
#include "BLI_rect.h"

#include "WM_api.hh"

#include "UI_interface.hh"
#include "UI_interface_c.hh"
#include "UI_resources.hh"

#include "screen_intern.h"

/**
 * \brief Screen edges drawing.
 */
static void drawscredge_area_edges(int pos, ScrArea *area)
{
  if (area->totrct.xmin != area->v1->vec.x) {
    immRecti(pos, area->v1->vec.x, area->v1->vec.y, area->totrct.xmin, area->v2->vec.y);
  }
  if (area->totrct.xmax != area->v4->vec.x) {
    immRecti(pos, area->totrct.xmax + 1, area->v1->vec.y, area->v4->vec.x, area->v2->vec.y);
  }
  if (area->totrct.ymin != area->v1->vec.y) {
    immRecti(pos, area->v1->vec.x, area->v1->vec.y, area->v4->vec.x + 1, area->totrct.ymin);
  }
  if (area->totrct.ymax != area->v2->vec.y) {
    immRecti(pos, area->v1->vec.x, area->totrct.ymax + 1, area->v4->vec.x + 1, area->v2->vec.y);
  }
}

void ED_screen_draw_edges(wmWindow *win)
{
  bScreen *screen = WM_window_get_active_screen(win);
  screen->do_draw = false;

  if (screen->state == SCREENFULL) {
    return;
  }

  if (screen->temp && BLI_listbase_is_single(&screen->areabase)) {
    return;
  }

  const int winsize_x = WM_window_pixels_x(win);
  const int winsize_y = WM_window_pixels_y(win);
  float col[4], corner_scale, edge_thickness;
  int verts_per_corner = 0;

  rcti scissor_rect;
  BLI_rcti_init_minmax(&scissor_rect);
  LISTBASE_FOREACH (ScrArea *, area, &screen->areabase) {
    BLI_rcti_do_minmax_v(&scissor_rect, blender::int2{area->v1->vec.x, area->v1->vec.y});
    BLI_rcti_do_minmax_v(&scissor_rect, blender::int2{area->v3->vec.x, area->v3->vec.y});
  }

  if (GPU_type_matches_ex(GPU_DEVICE_INTEL_UHD, GPU_OS_UNIX, GPU_DRIVER_ANY, GPU_BACKEND_OPENGL)) {
    /* For some reason, on linux + Intel UHD Graphics 620 the driver
     * hangs if we don't flush before this. (See #57455) */
    GPU_flush();
  }

  GPU_blend(GPU_BLEND_ALPHA);

  float color[4];
  UI_GetThemeColor4fv(TH_EDITOR_OUTLINE, color);

  GPUVertFormat *format = immVertexFormat();
  const uint pos = GPU_vertformat_attr_add(format, "pos", GPU_COMP_I32, 2, GPU_FETCH_INT_TO_FLOAT);
  immBindBuiltinProgram(GPU_SHADER_3D_UNIFORM_COLOR);
  immUniformThemeColor(TH_EDITOR_OUTLINE);
  immUniformColor4fv(color);

  LISTBASE_FOREACH (ScrArea *, area, &screen->areabase) {
    drawscredge_area_edges(pos, area);
  }

  int topbar_padding = int(ceilf(6.0 * UI_SCALE_FAC));
  int topbar_tab_pad = topbar_padding - U.pixelsize;

  bTheme *btheme = UI_GetTheme();
  immUniformColor4ubv(btheme->tui.wcol_tab.inner_sel);

  LISTBASE_FOREACH (ScrArea *, area, &win->global_areas.areabase) {
    if (area->spacetype == SPACE_TOPBAR) {
      immRecti(pos,
               area->totrct.xmin,
               area->totrct.ymin + U.pixelsize + 1,
               area->totrct.xmax,
               area->totrct.ymin + U.pixelsize + U.pixelsize + topbar_tab_pad);
      break;
    }
  }

  immUniformColor4f(1.0f, 1.0f, 1.0f, 0.1f);
  immRecti(pos, 0, 0, 1, win->sizey);

  immUnbindProgram();

  UI_draw_roundbox_corner_set(UI_CNR_ALL);
  int corners;

  LISTBASE_FOREACH (ScrArea *, area, &screen->areabase) {
    corners = UI_CNR_NONE;

    if (area->totrct.xmin != area->v1->vec.x && area->totrct.ymax != area->v2->vec.y) {
      corners |= UI_CNR_TOP_LEFT;
    }
    if (area->totrct.xmin != area->v1->vec.x && area->totrct.ymin != area->v1->vec.y) {
      corners |= UI_CNR_BOTTOM_LEFT;
    }
    if (area->totrct.xmax != area->v4->vec.x && area->totrct.ymax != area->v2->vec.y) {
      corners |= UI_CNR_TOP_RIGHT;
    }
    if (area->totrct.xmax != area->v4->vec.x && area->totrct.ymin != area->v1->vec.y) {
      corners |= UI_CNR_BOTTOM_RIGHT;
    }

    /* ui_draw_rounded_corners_inverted is off by one on two edges. */
    rcti rect = {
        area->totrct.xmin, area->totrct.xmax + 1, area->totrct.ymin, area->totrct.ymax + 1};
    UI_draw_roundbox_corner_set(corners);
    ui_draw_rounded_corners_inverted(rect, 6.0f * UI_SCALE_FAC, color);
  }

  GPU_blend(GPU_BLEND_NONE);
}

void screen_draw_join_highlight(ScrArea *sa1, ScrArea *sa2)
{
  const eScreenDir dir = area_getorientation(sa1, sa2);
  if (dir == SCREEN_DIR_NONE) {
    return;
  }

  /* Rect of the combined areas. */
  const bool vertical = SCREEN_DIR_IS_VERTICAL(dir);
  rctf combined{};
  combined.xmin = vertical ? std::max(sa1->totrct.xmin, sa2->totrct.xmin) :
                             std::min(sa1->totrct.xmin, sa2->totrct.xmin);
  combined.xmax = vertical ? std::min(sa1->totrct.xmax, sa2->totrct.xmax) :
                             std::max(sa1->totrct.xmax, sa2->totrct.xmax);
  combined.ymin = vertical ? std::min(sa1->totrct.ymin, sa2->totrct.ymin) :
                             std::max(sa1->totrct.ymin, sa2->totrct.ymin);
  combined.ymax = vertical ? std::max(sa1->totrct.ymax, sa2->totrct.ymax) :
                             std::min(sa1->totrct.ymax, sa2->totrct.ymax);

  uint pos_id = GPU_vertformat_attr_add(
      immVertexFormat(), "pos", GPU_COMP_F32, 2, GPU_FETCH_FLOAT);
  immBindBuiltinProgram(GPU_SHADER_3D_UNIFORM_COLOR);
  GPU_blend(GPU_BLEND_ALPHA);

  /* Highlight source (sa1) within combined area. */
  immUniformColor4fv(blender::float4{1.0f, 1.0f, 1.0f, 0.10f});
  immRectf(pos_id,
           std::max(float(sa1->totrct.xmin), combined.xmin),
           std::max(float(sa1->totrct.ymin), combined.ymin),
           std::min(float(sa1->totrct.xmax), combined.xmax),
           std::min(float(sa1->totrct.ymax), combined.ymax));

  /* Highlight destination (sa2) within combined area. */
  immUniformColor4fv(blender::float4{0.0f, 0.0f, 0.0f, 0.25f});
  immRectf(pos_id,
           std::max(float(sa2->totrct.xmin), combined.xmin),
           std::max(float(sa2->totrct.ymin), combined.ymin),
           std::min(float(sa2->totrct.xmax), combined.xmax),
           std::min(float(sa2->totrct.ymax), combined.ymax));

  int offset1;
  int offset2;
  area_getoffsets(sa1, sa2, dir, &offset1, &offset2);
  if (offset1 < 0 || offset2 > 0) {
    /* Show partial areas that will be closed. */
    immUniformColor4fv(blender::float4{0.0f, 0.0f, 0.0f, 0.8f});
    if (vertical) {
      if (sa1->totrct.xmin < combined.xmin) {
        immRectf(pos_id, sa1->totrct.xmin, sa1->totrct.ymin, combined.xmin, sa1->totrct.ymax);
      }
      if (sa2->totrct.xmin < combined.xmin) {
        immRectf(pos_id, sa2->totrct.xmin, sa2->totrct.ymin, combined.xmin, sa2->totrct.ymax);
      }
      if (sa1->totrct.xmax > combined.xmax) {
        immRectf(pos_id, combined.xmax, sa1->totrct.ymin, sa1->totrct.xmax, sa1->totrct.ymax);
      }
      if (sa2->totrct.xmax > combined.xmax) {
        immRectf(pos_id, combined.xmax, sa2->totrct.ymin, sa2->totrct.xmax, sa2->totrct.ymax);
      }
    }
    else {
      if (sa1->totrct.ymin < combined.ymin) {
        immRectf(pos_id, sa1->totrct.xmin, combined.ymin, sa1->totrct.xmax, sa1->totrct.ymin);
      }
      if (sa2->totrct.ymin < combined.ymin) {
        immRectf(pos_id, sa2->totrct.xmin, combined.ymin, sa2->totrct.xmax, sa2->totrct.ymin);
      }
      if (sa1->totrct.ymax > combined.ymax) {
        immRectf(pos_id, sa1->totrct.xmin, sa1->totrct.ymax, sa1->totrct.xmax, combined.ymax);
      }
      if (sa2->totrct.ymax > combined.ymax) {
        immRectf(pos_id, sa2->totrct.xmin, sa2->totrct.ymax, sa2->totrct.xmax, combined.ymax);
      }
    }
  }

  immUnbindProgram();
  GPU_blend(GPU_BLEND_NONE);

  /* Outline the combined area. */
  UI_draw_roundbox_corner_set(UI_CNR_ALL);
  UI_draw_roundbox_4fv(&combined, false, 7 * U.pixelsize, blender::float4{1.0f, 1.0f, 1.0f, 0.8f});
}

void screen_draw_split_preview(ScrArea *area, const eScreenAxis dir_axis, const float fac)
{
  uint pos = GPU_vertformat_attr_add(immVertexFormat(), "pos", GPU_COMP_F32, 2, GPU_FETCH_FLOAT);
  immBindBuiltinProgram(GPU_SHADER_3D_UNIFORM_COLOR);

  /* Split-point. */
  GPU_blend(GPU_BLEND_ALPHA);

  immUniformColor4ub(255, 255, 255, 100);

  immBegin(GPU_PRIM_LINES, 2);

  if (dir_axis == SCREEN_AXIS_H) {
    const float y = (1 - fac) * area->totrct.ymin + fac * area->totrct.ymax;

    immVertex2f(pos, area->totrct.xmin, y);
    immVertex2f(pos, area->totrct.xmax, y);

    immEnd();

    immUniformColor4ub(0, 0, 0, 100);

    immBegin(GPU_PRIM_LINES, 2);

    immVertex2f(pos, area->totrct.xmin, y + 1);
    immVertex2f(pos, area->totrct.xmax, y + 1);

    immEnd();
  }
  else {
    BLI_assert(dir_axis == SCREEN_AXIS_V);
    const float x = (1 - fac) * area->totrct.xmin + fac * area->totrct.xmax;

    immVertex2f(pos, x, area->totrct.ymin);
    immVertex2f(pos, x, area->totrct.ymax);

    immEnd();

    immUniformColor4ub(0, 0, 0, 100);

    immBegin(GPU_PRIM_LINES, 2);

    immVertex2f(pos, x + 1, area->totrct.ymin);
    immVertex2f(pos, x + 1, area->totrct.ymax);

    immEnd();
  }

  GPU_blend(GPU_BLEND_NONE);

  immUnbindProgram();
}
