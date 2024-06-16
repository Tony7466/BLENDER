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

#include "BKE_screen.hh"

#include "BLI_listbase.h"
#include "BLI_math_vector.hh"
#include "BLI_rect.h"

#include "WM_api.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "RNA_access.hh"
#include "RNA_enum_types.hh"

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

    blender::gpu::VertBuf *vbo = GPU_vertbuf_create_with_format(format);
    GPU_vertbuf_data_alloc(*vbo, CORNER_RESOLUTION * 2 * 4 + 2);

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

static void screen_draw_area_icon(float center_x,
                                  float center_y,
                                  int icon,
                                  uchar *color,
                                  float *bg = nullptr,
                                  float *outline = nullptr)
{
  const float bg_width = U.pixelsize * 50.0f;
  const float bg_height = U.pixelsize * 40.0f;
  rctf combined = {center_x - (bg_width / 2.0f),
                   center_x + bg_width - (bg_width / 2.0f),
                   center_y - (bg_height / 2.0f),
                   center_y + bg_height - (bg_height / 2.0f)};
  if (bg) {
    UI_draw_roundbox_4fv_ex(
        &combined, bg, nullptr, 1.0f, outline ? outline : nullptr, U.pixelsize, 6 * U.pixelsize);
  }
  UI_icon_draw_ex(center_x - U.pixelsize * 16.0f,
                  center_y - U.pixelsize * 16.0f,
                  icon,
                  0.5f,
                  float(color[3]) / 255.0f,
                  0.0f,
                  color,
                  false,
                  UI_NO_ICON_OVERLAY_TEXT);
}

static int area_icon(ScrArea *area)
{
  const int index = RNA_enum_from_value(rna_enum_space_type_items, area->spacetype);
  const EnumPropertyItem item = rna_enum_space_type_items[index];
  if (item.value == SPACE_ACTION) {
    const int subtype = area->type->space_subtype_get(area);
    int subindex = RNA_enum_from_value(rna_enum_space_action_mode_items, subtype);
    return rna_enum_space_action_mode_items[subindex].icon;
  }
  if (item.value == SPACE_IMAGE) {
    const int subtype = area->type->space_subtype_get(area);
    int subindex = RNA_enum_from_value(rna_enum_space_image_mode_items, subtype);
    return rna_enum_space_image_mode_items[subindex].icon;
  }
  /* Can't figure out how to differenciate between node editors. */
  return item.icon;
}

void screen_draw_join_highlight(ScrArea *sa1, ScrArea *sa2, eScreenDir dir)
{
  if (dir == SCREEN_DIR_NONE || !sa2) {
    /* Darken source if docking. Done here because might be a different window. */
    rctf rect;
    BLI_rctf_rcti_copy(&rect, &sa1->totrct);
    float darken[4] = {0.0f, 0.0f, 0.0f, 0.7f};
    UI_draw_roundbox_corner_set(UI_CNR_ALL);
    UI_draw_roundbox_4fv_ex(&rect, darken, nullptr, 1.0f, nullptr, U.pixelsize, 6 * U.pixelsize);
    if (BLI_rctf_size_x(&rect) > U.pixelsize * 75.0f &&
        BLI_rctf_size_y(&rect) > U.pixelsize * 60.0f)
    {
      uchar icon_color[4] = {255, 255, 255, 128};
      screen_draw_area_icon(
          (rect.xmin + rect.xmax) / 2.0f, (rect.ymin + rect.ymax) / 2.0f, ICON_CANCEL, icon_color);
    }
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
    immUniformColor4fv(blender::float4{0.0f, 0.0f, 0.0f, 0.7f});

    uchar icon_color[4] = {255, 255, 255, 128};

    if (vertical) {
      if (sa1->totrct.xmin < combined.xmin) {
        immRectf(pos_id, sa1->totrct.xmin, sa1->totrct.ymin, combined.xmin, sa1->totrct.ymax);
        if ((combined.xmin - sa1->totrct.xmin) > U.pixelsize * 75.0f &&
            (sa1->totrct.ymax - sa1->totrct.ymin) > U.pixelsize * 60.0f)
        {
          screen_draw_area_icon((sa1->totrct.xmin + combined.xmin) / 2.0f,
                                (sa1->totrct.ymin + sa1->totrct.ymax) / 2.0f,
                                ICON_CANCEL,
                                icon_color,
                                nullptr);
        }
      }
      if (sa2->totrct.xmin < combined.xmin) {
        immRectf(pos_id, sa2->totrct.xmin, sa2->totrct.ymin, combined.xmin, sa2->totrct.ymax);
        if ((combined.xmin - sa2->totrct.xmin) > U.pixelsize * 75.0f &&
            (sa2->totrct.ymax - sa2->totrct.ymin) > U.pixelsize * 60.0f)
        {
          screen_draw_area_icon((sa2->totrct.xmin + combined.xmin) / 2.0f,
                                (sa2->totrct.ymin + sa2->totrct.ymax) / 2.0f,
                                ICON_CANCEL,
                                icon_color);
        }
      }
      if (sa1->totrct.xmax > combined.xmax) {
        immRectf(pos_id, combined.xmax, sa1->totrct.ymin, sa1->totrct.xmax, sa1->totrct.ymax);
        if ((sa1->totrct.xmax - combined.xmax) > U.pixelsize * 75.0f &&
            (sa1->totrct.ymax - sa1->totrct.ymin) > U.pixelsize * 60.0f)
        {
          screen_draw_area_icon((combined.xmax + sa1->totrct.xmax) / 2.0f,
                                (sa1->totrct.ymin + sa1->totrct.ymax) / 2.0f,
                                ICON_CANCEL,
                                icon_color);
        }
      }
      if (sa2->totrct.xmax > combined.xmax) {
        immRectf(pos_id, combined.xmax, sa2->totrct.ymin, sa2->totrct.xmax, sa2->totrct.ymax);
        if ((sa2->totrct.xmax - combined.xmax) > U.pixelsize * 75.0f &&
            (sa2->totrct.ymax - sa2->totrct.ymin) > U.pixelsize * 60.0f)
        {
          screen_draw_area_icon((combined.xmax + sa2->totrct.xmax) / 2.0f,
                                (sa2->totrct.ymin + sa2->totrct.ymax) / 2.0f,
                                ICON_CANCEL,
                                icon_color);
        }
      }
    }
    else {
      if (sa1->totrct.ymin < combined.ymin) {
        immRectf(pos_id, sa1->totrct.xmin, sa1->totrct.ymin, sa1->totrct.xmax, combined.ymin);
        if ((sa1->totrct.xmax - sa1->totrct.xmin) > U.pixelsize * 75.0f &&
            (combined.ymin - sa1->totrct.ymin) > U.pixelsize * 60.0f)
        {
          screen_draw_area_icon((sa1->totrct.xmin + sa1->totrct.xmax) / 2.0f,
                                (combined.ymin + sa1->totrct.ymin) / 2.0f,
                                ICON_CANCEL,
                                icon_color);
        }
      }
      if (sa2->totrct.ymin < combined.ymin) {
        immRectf(pos_id, sa2->totrct.xmin, sa2->totrct.ymin, sa2->totrct.xmax, combined.ymin);
        if ((sa2->totrct.xmax - sa2->totrct.xmin) > U.pixelsize * 75.0f &&
            (combined.ymin - sa2->totrct.ymin) > U.pixelsize * 60.0f)
        {
          screen_draw_area_icon((sa2->totrct.xmin + sa2->totrct.xmax) / 2.0f,
                                (combined.ymin + sa2->totrct.ymin) / 2.0f,
                                ICON_CANCEL,
                                icon_color);
        }
      }
      if (sa1->totrct.ymax > combined.ymax) {
        immRectf(pos_id, sa1->totrct.xmin, combined.ymax, sa1->totrct.xmax, sa1->totrct.ymax);
        if ((sa1->totrct.xmax - sa1->totrct.xmin) > U.pixelsize * 75.0f &&
            (sa1->totrct.ymax - combined.ymax) > U.pixelsize * 60.0f)
        {
          screen_draw_area_icon((sa1->totrct.xmin + sa1->totrct.xmax) / 2.0f,
                                (sa1->totrct.ymax + combined.ymax) / 2.0f,
                                ICON_CANCEL,
                                icon_color);
        }
      }
      if (sa2->totrct.ymax > combined.ymax) {
        immRectf(pos_id, sa2->totrct.xmin, combined.ymax, sa2->totrct.xmax, sa2->totrct.ymax);
        if ((sa2->totrct.xmax - sa2->totrct.xmin) > U.pixelsize * 75.0f &&
            (sa2->totrct.ymax - combined.ymax) > U.pixelsize * 60.0f)
        {
          screen_draw_area_icon((sa2->totrct.xmin + sa2->totrct.xmax) / 2.0f,
                                (sa2->totrct.ymax + combined.ymax) / 2.0f,
                                ICON_CANCEL,
                                icon_color);
        }
      }
    }
    immUnbindProgram();
    GPU_blend(GPU_BLEND_NONE);
  }

  /* Outline the combined area. */
  UI_draw_roundbox_corner_set(UI_CNR_ALL);
  float outline[4] = {1.0f, 1.0f, 1.0f, 0.4f};
  float inner[4] = {1.0f, 1.0f, 1.0f, 0.10f};
  UI_draw_roundbox_4fv_ex(&combined, inner, nullptr, 1.0f, outline, U.pixelsize, 6 * U.pixelsize);

  /* Icon in center of intersection of combined and sa2 - the subsumed part. */
  rctf sa2tot;
  BLI_rctf_rcti_copy(&sa2tot, &sa2->totrct);
  rctf sa2new;
  BLI_rctf_isect(&combined, &sa2tot, &sa2new);

  if (BLI_rctf_size_x(&sa2new) > U.pixelsize * 75.0f &&
      BLI_rctf_size_y(&sa2new) > U.pixelsize * 60.0f)
  {
    const int icon = area_icon(sa1);
    uchar icon_color[4] = {255, 255, 255, 255};
    float bg_color[4] = {0.0f, 0.0f, 0.0f, 0.4f};
    float outline_color[4] = {1.0f, 1.0f, 1.0f, 0.4f};
    screen_draw_area_icon((sa2new.xmin + sa2new.xmax) / 2.0f,
                          (sa2new.ymin + sa2new.ymax) / 2.0f,
                          icon,
                          icon_color,
                          bg_color,
                          outline_color);
  }
}

void screen_draw_dock_preview(const struct wmWindow * /* win */,
                              ScrArea *source,
                              ScrArea *target,
                              eAreaDockTarget dock_target)
{
  if (dock_target == DOCKING_NONE) {
    return;
  }

  float outline[4] = {1.0f, 1.0f, 1.0f, 0.4f};
  float inner[4] = {1.0f, 1.0f, 1.0f, 0.1f};
  float border[4];
  UI_GetThemeColor4fv(TH_EDITOR_OUTLINE, border);
  UI_draw_roundbox_corner_set(UI_CNR_ALL);
  float half_line_width = 2.0f * U.pixelsize;

  rctf dest;
  rctf remainder;
  BLI_rctf_rcti_copy(&dest, &target->totrct);
  BLI_rctf_rcti_copy(&remainder, &target->totrct);

  const int icon1 = area_icon(source);
  const int icon2 = area_icon(target);

  uchar icon_color[4] = {255, 255, 255, 255};
  float outline_color[4] = {1.0f, 1.0f, 1.0f, 0.4f};
  float bg_color[4] = {0.0f, 0.0f, 0.0f, 0.4f};

  float split;

  if (dock_target == DOCKING_RIGHT) {
    split = std::min(dest.xmin + target->winx * 0.501f, dest.xmax - AREAMINX * UI_SCALE_FAC);
    dest.xmin = split + half_line_width;
    remainder.xmax = split - half_line_width;
  }
  else if (dock_target == DOCKING_LEFT) {
    split = std::max(dest.xmax - target->winx * 0.501f, dest.xmin + AREAMINX * UI_SCALE_FAC);
    dest.xmax = split - half_line_width;
    remainder.xmin = split + half_line_width;
  }
  else if (dock_target == DOCKING_TOP) {
    split = std::min(dest.ymin + target->winy * 0.501f, dest.ymax - HEADERY * UI_SCALE_FAC);
    dest.ymin = split + half_line_width;
    remainder.ymax = split - half_line_width;
  }
  else if (dock_target == DOCKING_BOTTOM) {
    split = std::max(dest.ymax - target->winy * 0.501f, dest.ymin + HEADERY * UI_SCALE_FAC);
    dest.ymax = split - half_line_width;
    remainder.ymin = split + half_line_width;
  }

  if (dock_target == DOCKING_CENTER) {
    UI_draw_roundbox_4fv_ex(&dest, inner, nullptr, 1.0f, outline, U.pixelsize, 6 * U.pixelsize);
    if (BLI_rctf_size_x(&dest) > U.pixelsize * 75.0f &&
        BLI_rctf_size_y(&dest) > U.pixelsize * 60.0f)
    {
      screen_draw_area_icon((dest.xmin + dest.xmax) / 2.0f,
                            (dest.ymin + dest.ymax) / 2.0f,
                            icon1,
                            icon_color,
                            bg_color,
                            outline);
    }
  }
  else {
    UI_draw_roundbox_4fv_ex(&dest, inner, nullptr, 1.0f, outline, U.pixelsize, 6 * U.pixelsize);

    if (BLI_rctf_size_x(&dest) > U.pixelsize * 75.0f &&
        BLI_rctf_size_y(&dest) > U.pixelsize * 60.0f)
    {
      screen_draw_area_icon((dest.xmin + dest.xmax) / 2.0f,
                            (dest.ymin + dest.ymax) / 2.0f,
                            icon1,
                            icon_color,
                            bg_color,
                            outline);
    }

    if (BLI_rctf_size_x(&remainder) > U.pixelsize * 75.0f &&
        BLI_rctf_size_y(&remainder) > U.pixelsize * 60.0f)
    {
      bg_color[3] = 0.3f;
      icon_color[3] = 128;
      screen_draw_area_icon((remainder.xmin + remainder.xmax) / 2.0f,
                            (remainder.ymin + remainder.ymax) / 2.0f,
                            icon2,
                            icon_color,
                            bg_color,
                            nullptr);
    }

    /* Darken the split position itself. */
    if (ELEM(dock_target, DOCKING_RIGHT, DOCKING_LEFT)) {
      dest.xmin = split - half_line_width;
      dest.xmax = split + half_line_width;
    }
    else {
      dest.ymin = split - half_line_width;
      dest.ymax = split + half_line_width;
    }
    UI_draw_roundbox_4fv(&dest, true, 0.0f, border);
  }
}

void screen_draw_split_preview(ScrArea *area, const eScreenAxis dir_axis, const float fac)
{
  float outline[4] = {1.0f, 1.0f, 1.0f, 0.4f};
  float inner[4] = {1.0f, 1.0f, 1.0f, 0.10f};
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
