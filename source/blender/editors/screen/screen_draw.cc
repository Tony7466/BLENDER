/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edscr
 */

#include "ED_screen.hh"

#include "GPU_batch_presets.hh"
#include "GPU_immediate.hh"
#include "GPU_platform.hh"
#include "GPU_state.hh"

#include "BLI_listbase.h"
#include "BLI_math_vector.hh"
#include "BLI_rect.h"

#include "WM_api.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "screen_intern.hh"

#define CORNER_RESOLUTION 3

static void do_vert_pair(blender::gpu::VertBuf *vbo, uint pos, uint *vidx, int corner, int i)
{
  float inter[2];
  inter[0] = cosf(corner * M_PI_2 + (i * M_PI_2 / (CORNER_RESOLUTION - 1.0f)));
  inter[1] = sinf(corner * M_PI_2 + (i * M_PI_2 / (CORNER_RESOLUTION - 1.0f)));

  /* Snap point to edge */
  float div = 1.0f / max_ff(fabsf(inter[0]), fabsf(inter[1]));
  float exter[2];
  mul_v2_v2fl(exter, inter, div);
  exter[0] = roundf(exter[0]);
  exter[1] = roundf(exter[1]);

  if (i == 0 || i == (CORNER_RESOLUTION - 1)) {
    copy_v2_v2(inter, exter);
  }

  /* Line width is 20% of the entire corner size. */
  const float line_width = 0.2f; /* Keep in sync with shader */
  mul_v2_fl(inter, 1.0f - line_width);
  mul_v2_fl(exter, 1.0f + line_width);

  switch (corner) {
    case 0:
      add_v2_v2(inter, blender::float2{-1.0f, -1.0f});
      add_v2_v2(exter, blender::float2{-1.0f, -1.0f});
      break;
    case 1:
      add_v2_v2(inter, blender::float2{1.0f, -1.0f});
      add_v2_v2(exter, blender::float2{1.0f, -1.0f});
      break;
    case 2:
      add_v2_v2(inter, blender::float2{1.0f, 1.0f});
      add_v2_v2(exter, blender::float2{1.0f, 1.0f});
      break;
    case 3:
      add_v2_v2(inter, blender::float2{-1.0f, 1.0f});
      add_v2_v2(exter, blender::float2{-1.0f, 1.0f});
      break;
  }

  GPU_vertbuf_attr_set(vbo, pos, (*vidx)++, inter);
  GPU_vertbuf_attr_set(vbo, pos, (*vidx)++, exter);
}

static blender::gpu::Batch *batch_screen_edges_get(int *corner_len)
{
  static blender::gpu::Batch *screen_edges_batch = nullptr;

  if (screen_edges_batch == nullptr) {
    GPUVertFormat format = {0};
    uint pos = GPU_vertformat_attr_add(&format, "pos", GPU_COMP_F32, 2, GPU_FETCH_FLOAT);

    blender::gpu::VertBuf *vbo = GPU_vertbuf_create_with_format(&format);
    GPU_vertbuf_data_alloc(vbo, CORNER_RESOLUTION * 2 * 4 + 2);

    uint vidx = 0;
    for (int corner = 0; corner < 4; corner++) {
      for (int c = 0; c < CORNER_RESOLUTION; c++) {
        do_vert_pair(vbo, pos, &vidx, corner, c);
      }
    }
    /* close the loop */
    do_vert_pair(vbo, pos, &vidx, 0, 0);

    screen_edges_batch = GPU_batch_create_ex(GPU_PRIM_TRI_STRIP, vbo, nullptr, GPU_BATCH_OWNS_VBO);
    gpu_batch_presets_register(screen_edges_batch);
  }

  if (corner_len) {
    *corner_len = CORNER_RESOLUTION * 2;
  }
  return screen_edges_batch;
}

#undef CORNER_RESOLUTION

static void drawscredge_area_draw(
    int sizex, int sizey, short x1, short y1, short x2, short y2, float edge_thickness)
{
  rctf rect;
  BLI_rctf_init(&rect, float(x1), float(x2), float(y1), float(y2));

  /* right border area */
  if (x2 >= sizex - 1) {
    rect.xmax += edge_thickness * 0.5f;
  }

  /* left border area */
  if (x1 <= 0) { /* otherwise it draws the emboss of window over */
    rect.xmin -= edge_thickness * 0.5f;
  }

  /* top border area */
  if (y2 >= sizey - 1) {
    rect.ymax += edge_thickness * 0.5f;
  }

  /* bottom border area */
  if (y1 <= 0) {
    rect.ymin -= edge_thickness * 0.5f;
  }

  blender::gpu::Batch *batch = batch_screen_edges_get(nullptr);
  GPU_batch_program_set_builtin(batch, GPU_SHADER_2D_AREA_BORDERS);
  GPU_batch_uniform_4fv(batch, "rect", (float *)&rect);
  GPU_batch_draw(batch);
}

/**
 * \brief Screen edges drawing.
 */
static void drawscredge_area(ScrArea *area, int sizex, int sizey, float edge_thickness)
{
  short x1 = area->v1->vec.x;
  short y1 = area->v1->vec.y;
  short x2 = area->v3->vec.x;
  short y2 = area->v3->vec.y;

  drawscredge_area_draw(sizex, sizey, x1, y1, x2, y2, edge_thickness);
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

  GPU_scissor(scissor_rect.xmin,
              scissor_rect.ymin,
              BLI_rcti_size_x(&scissor_rect) + 1,
              BLI_rcti_size_y(&scissor_rect) + 1);

  /* It seems that all areas gets smaller when pixelsize is > 1.
   * So in order to avoid missing pixels we just disable de scissors. */
  if (U.pixelsize <= 1.0f) {
    GPU_scissor_test(true);
  }

  UI_GetThemeColor4fv(TH_EDITOR_OUTLINE, col);
  col[3] = 1.0f;
  corner_scale = U.pixelsize * 8.0f;
  edge_thickness = corner_scale * 0.21f;

  GPU_blend(GPU_BLEND_ALPHA);

  blender::gpu::Batch *batch = batch_screen_edges_get(&verts_per_corner);
  GPU_batch_program_set_builtin(batch, GPU_SHADER_2D_AREA_BORDERS);
  GPU_batch_uniform_1i(batch, "cornerLen", verts_per_corner);
  GPU_batch_uniform_1f(batch, "scale", corner_scale);
  GPU_batch_uniform_4fv(batch, "color", col);

  LISTBASE_FOREACH (ScrArea *, area, &screen->areabase) {
    drawscredge_area(area, winsize_x, winsize_y, edge_thickness);
  }

  GPU_blend(GPU_BLEND_NONE);

  if (U.pixelsize <= 1.0f) {
    GPU_scissor_test(false);
  }
}

void screen_draw_join_highlight(ScrArea *sa1, ScrArea *sa2, eScreenDir dir)
{
  if (dir == SCREEN_DIR_NONE || !sa2) {
    /* Darken source if docking. Done here because might be a different window. */
    rctf rect;
    BLI_rctf_rcti_copy(&rect, &sa1->totrct);
    float darken[4] = {0.0f, 0.0f, 0.0f, 0.8f};
    UI_draw_roundbox_corner_set(UI_CNR_ALL);
    UI_draw_roundbox_4fv_ex(&rect, darken, nullptr, 1.0f, nullptr, U.pixelsize, 6 * U.pixelsize);
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

  int offset1;
  int offset2;
  area_getoffsets(sa1, sa2, dir, &offset1, &offset2);
  if (offset1 < 0 || offset2 > 0) {
    /* Show partial areas that will be closed. */
    uint pos_id = GPU_vertformat_attr_add(
        immVertexFormat(), "pos", GPU_COMP_F32, 2, GPU_FETCH_FLOAT);
    immBindBuiltinProgram(GPU_SHADER_3D_UNIFORM_COLOR);
    GPU_blend(GPU_BLEND_ALPHA);
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
    immUnbindProgram();
    GPU_blend(GPU_BLEND_NONE);
  }

  /* Outline the combined area. */
  UI_draw_roundbox_corner_set(UI_CNR_ALL);
  float outline[4] = {1.0f, 1.0f, 1.0f, 0.4f};
  float inner[4] = {1.0f, 1.0f, 1.0f, 0.15f};
  UI_draw_roundbox_4fv_ex(&combined, inner, nullptr, 1.0f, outline, U.pixelsize, 6 * U.pixelsize);
}

void screen_draw_dock_preview(const struct wmWindow * /* win */,
                              struct ScrArea *area,
                              eAreaDockTarget dock_target,
                              float factor)
{
  rctf dest;
  BLI_rctf_rcti_copy(&dest, &area->totrct);

  if (dock_target == DOCKING_RIGHT) {
    dest.xmin = std::min(dest.xmin + area->winx * (1.0f - factor),
                         dest.xmax - AREAMINX * UI_SCALE_FAC);
  }
  else if (dock_target == DOCKING_LEFT) {
    dest.xmax = std::max(dest.xmax - area->winx * (1.0f - factor),
                         dest.xmin + AREAMINX * UI_SCALE_FAC);
  }
  else if (dock_target == DOCKING_TOP) {
    dest.ymin = std::min(dest.ymin + area->winy * (1.0f - factor),
                         dest.ymax - HEADERY * UI_SCALE_FAC);
  }
  else if (dock_target == DOCKING_BOTTOM) {
    dest.ymax = std::max(dest.ymax - area->winy * (1.0f - factor),
                         dest.ymin + HEADERY * UI_SCALE_FAC);
  }

  UI_draw_roundbox_corner_set(UI_CNR_ALL);
  float inner[4] = {1.0f, 1.0f, 1.0f, 0.15f};
  float outline[4] = {1.0f, 1.0f, 1.0f, 0.4f};
  UI_draw_roundbox_4fv_ex(&dest,
                          (dock_target == DOCKING_NONE) ? nullptr : inner,
                          nullptr,
                          1.0f,
                          outline,
                          U.pixelsize,
                          6 * U.pixelsize);
}

void screen_draw_split_preview(ScrArea *area, const eScreenAxis dir_axis, const float fac)
{
  float outline[4] = {1.0f, 1.0f, 1.0f, 0.4f};
  float inner[4] = {1.0f, 1.0f, 1.0f, 0.15f};
  float border[4];
  UI_GetThemeColor4fv(TH_EDITOR_OUTLINE, border);
  UI_draw_roundbox_corner_set(UI_CNR_ALL);

  rctf rect;
  BLI_rctf_rcti_copy(&rect, &area->totrct);

  if (fac < 0.0001 || fac > 0.9999) {
    /* Highlight the entire area. */
    UI_draw_roundbox_4fv_ex(&rect, inner, nullptr, 1.0f, outline, U.pixelsize, 7 * U.pixelsize);
    return;
  }

  float x = (1 - fac) * rect.xmin + fac * rect.xmax;
  float y = (1 - fac) * rect.ymin + fac * rect.ymax;
  CLAMP(x, rect.xmin + (AREAMINX * UI_SCALE_FAC), rect.xmax - (AREAMINX * UI_SCALE_FAC));
  CLAMP(y, rect.ymin + (HEADERY * UI_SCALE_FAC), rect.ymax - (HEADERY * UI_SCALE_FAC));
  float half_line_width = 2.0f * U.pixelsize;

  /* Outlined rectangle to left/above split position. */
  rect.xmax = (dir_axis == SCREEN_AXIS_V) ? x - half_line_width : rect.xmax;
  rect.ymax = (dir_axis == SCREEN_AXIS_H) ? y - half_line_width : rect.ymax;

  UI_draw_roundbox_4fv_ex(&rect, inner, nullptr, 1.0f, outline, U.pixelsize, 7 * U.pixelsize);

  /* Outlined rectangle to right/below split position. */
  if (dir_axis == SCREEN_AXIS_H) {
    rect.ymin = y + half_line_width;
    rect.ymax = area->totrct.ymax;
  }
  else {
    rect.xmin = x + half_line_width;
    rect.xmax = area->totrct.xmax;
  }
  UI_draw_roundbox_4fv_ex(&rect, inner, nullptr, 1.0f, outline, U.pixelsize, 7 * U.pixelsize);

  /* Darken the split position itself. */
  if (dir_axis == SCREEN_AXIS_H) {
    rect.ymin = y - half_line_width;
    rect.ymax = y + half_line_width;
  }
  else {
    rect.xmin = x - half_line_width;
    rect.xmax = x + half_line_width;
  }
  UI_draw_roundbox_4fv(&rect, true, 0.0f, border);
}
