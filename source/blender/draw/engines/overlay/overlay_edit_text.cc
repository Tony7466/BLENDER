/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2019 Blender Foundation. */

/** \file
 * \ingroup draw_engine
 */

#include "DRW_render.h"

#include "UI_resources.h"

#include "BKE_vfont.h"

#include "DNA_curve_types.h"

#include "overlay_private.hh"

void OVERLAY_edit_text_cache_init(OVERLAY_Data *vedata)
{
  OVERLAY_PassList *psl = vedata->psl;
  OVERLAY_PrivateData *pd = vedata->stl->pd;
  const DRWContextState *draw_ctx = DRW_context_state_get();
  View3D *v3d = draw_ctx->v3d;
  DRWShadingGroup *grp;
  GPUShader *sh;
  DRWState state;

  pd->edit_curve.show_handles = v3d->overlay.handle_display != CURVE_HANDLE_NONE;
  pd->edit_curve.handle_display = v3d->overlay.handle_display;
  pd->shdata.edit_curve_normal_length = v3d->overlay.normals_length;

  /* Run Twice for in-front passes. */
  for (int i = 0; i < 2; i++) {
    state = DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH;
    state |= ((i == 0) ? DRW_STATE_DEPTH_LESS_EQUAL : DRW_STATE_DEPTH_ALWAYS);
    DRW_PASS_CREATE(psl->edit_text_wire_ps[i], state | pd->clipping_state);

    sh = OVERLAY_shader_uniform_color();
    pd->edit_text_wire_grp[i] = grp = DRW_shgroup_create(sh, psl->edit_text_wire_ps[i]);
    DRW_shgroup_uniform_vec4_copy(grp, "ucolor", G_draw.block.color_wire);
  }
  {
    /* Cursor (text caret). */
    state = DRW_STATE_WRITE_COLOR | DRW_STATE_BLEND_ALPHA;
    DRW_PASS_CREATE(psl->edit_text_cursor_ps, state | pd->clipping_state);
    sh = OVERLAY_shader_uniform_color();
    pd->edit_text_cursor_grp = grp = DRW_shgroup_create(sh, psl->edit_text_cursor_ps);
    DRW_shgroup_uniform_vec4(grp, "ucolor", pd->edit_text.cursor_color, 1);

    /* Selection boxes. */
    state = DRW_STATE_WRITE_COLOR | DRW_STATE_BLEND_ALPHA;
    DRW_PASS_CREATE(psl->edit_text_selection_ps, state | pd->clipping_state);
    sh = OVERLAY_shader_uniform_color();
    pd->edit_text_selection_grp = grp = DRW_shgroup_create(sh, psl->edit_text_selection_ps);
    DRW_shgroup_uniform_vec4(grp, "ucolor", pd->edit_text.selection_color, 1);

    /* Highlight text within selection boxes. */
    state = DRW_STATE_WRITE_COLOR | DRW_STATE_BLEND_ALPHA | DRW_STATE_DEPTH_GREATER_EQUAL |
            pd->clipping_state;
    DRW_PASS_INSTANCE_CREATE(psl->edit_text_highlight_ps, psl->edit_text_selection_ps, state);
  }
  {
    /* Create view which will render everything (hopefully) behind the text geometry. */
    DRWView *default_view = (DRWView *)DRW_view_default_get();
    pd->view_edit_text = DRW_view_create_with_zoffset(default_view, draw_ctx->rv3d, -5.0f);
  }
}

static void v2_transform_to_mat4(const float loc[2],
                                 const float rot,
                                 const float scale[2],
                                 float r_mat[4][4])
{
  const float loc_v3[3] = {loc[0], loc[1], 0.0f};
  const float rot_v3[3] = {0.0f, 0.0f, rot};
  const float size_v3[3] = {scale[0], scale[1], 0.0f};
  loc_eul_size_to_mat4(r_mat, loc_v3, rot_v3, size_v3);
}

static void edit_text_cache_populate_select(OVERLAY_Data *vedata, Object *ob)
{
  OVERLAY_PrivateData *pd = vedata->stl->pd;
  const Curve *cu = static_cast<Curve *>(ob->data);
  EditFont *ef = cu->editfont;
  struct GPUBatch *geom = DRW_cache_quad_get();

  for (int i = 0; i < ef->selboxes_len; i++) {
    EditFontSelBox &sb = ef->selboxes[i];
    float final_mat[4][4];
    v2_transform_to_mat4(sb.loc, sb.rot, sb.size, final_mat);
    mul_m4_m4m4(final_mat, ob->object_to_world, final_mat);
    DRW_shgroup_call_obmat(pd->edit_text_selection_grp, geom, final_mat);
  }
}

static void edit_text_cache_populate_cursor(OVERLAY_Data *vedata, Object *ob)
{
  OVERLAY_PrivateData *pd = vedata->stl->pd;
  const Curve *cu = static_cast<Curve *>(ob->data);
  EditFont &edit_font = *cu->editfont;

  float mat[4][4];
  v2_transform_to_mat4(edit_font.curs_location, edit_font.curs_angle, edit_font.curs_size, mat);
  mul_m4_m4m4(mat, ob->object_to_world, mat);

  struct GPUBatch *geom = DRW_cache_quad_get();
  DRW_shgroup_call_obmat(pd->edit_text_cursor_grp, geom, mat);
}

static void edit_text_cache_populate_boxes(OVERLAY_Data *vedata, Object *ob)
{
  OVERLAY_ExtraCallBuffers *cb = OVERLAY_extra_call_buffer_get(vedata, ob);
  const Curve *cu = static_cast<Curve *>(ob->data);

  for (int i = 0; i < cu->totbox; i++) {
    TextBox *tb = &cu->tb[i];
    const bool is_active = (i == (cu->actbox - 1));
    float *color = is_active ? G_draw.block.color_active : G_draw.block.color_wire;

    if ((tb->w != 0.0f) || (tb->h != 0.0f)) {
      float vecs[4][3];
      vecs[0][0] = vecs[1][0] = vecs[2][0] = vecs[3][0] = cu->xof + tb->x;
      vecs[0][1] = vecs[1][1] = vecs[2][1] = vecs[3][1] = cu->yof + tb->y + cu->fsize_realtime;
      vecs[0][2] = vecs[1][2] = vecs[2][2] = vecs[3][2] = 0.001;

      vecs[1][0] += tb->w;
      vecs[2][0] += tb->w;
      vecs[2][1] -= tb->h;
      vecs[3][1] -= tb->h;

      for (int j = 0; j < 4; j++) {
        mul_v3_m4v3(vecs[j], ob->object_to_world, vecs[j]);
      }
      for (int j = 0; j < 4; j++) {
        OVERLAY_extra_line_dashed(cb, vecs[j], vecs[(j + 1) % 4], color);
      }
    }
  }
}

void OVERLAY_edit_text_cache_populate(OVERLAY_Data *vedata, Object *ob)
{
  OVERLAY_PrivateData *pd = vedata->stl->pd;
  struct GPUBatch *geom;
  bool do_in_front = (ob->dtx & OB_DRAW_IN_FRONT) != 0;

  geom = DRW_cache_text_edge_wire_get(ob);
  if (geom) {
    DRW_shgroup_call(pd->edit_text_wire_grp[do_in_front], geom, ob);
  }

  edit_text_cache_populate_select(vedata, ob);
  edit_text_cache_populate_cursor(vedata, ob);
  edit_text_cache_populate_boxes(vedata, ob);
}

void OVERLAY_edit_text_draw(OVERLAY_Data *vedata)
{
  OVERLAY_PrivateData *pd = vedata->stl->pd;
  OVERLAY_PassList *psl = vedata->psl;
  OVERLAY_FramebufferList *fbl = vedata->fbl;

  if (DRW_state_is_fbo()) {
    GPU_framebuffer_bind(fbl->overlay_default_fb);
  }

  DRW_draw_pass(psl->edit_text_wire_ps[0]);
  DRW_draw_pass(psl->edit_text_wire_ps[1]);

  DRW_view_set_active(pd->view_edit_text);

  /* Selection Boxes. */
  UI_GetThemeColor4fv(TH_WIDGET_TEXT_SELECTION, pd->edit_text.selection_color);
  srgb_to_linearrgb_v4(pd->edit_text.selection_color, pd->edit_text.selection_color);
  DRW_draw_pass(psl->edit_text_selection_ps);

  /* Highlight text within selection boxes. */
  UI_GetThemeColor4fv(TH_WIDGET_TEXT_HIGHLIGHT, pd->edit_text.selection_color);
  srgb_to_linearrgb_v4(pd->edit_text.selection_color, pd->edit_text.selection_color);
  DRW_draw_pass(psl->edit_text_highlight_ps);

  /* Cursor (text caret). */
  UI_GetThemeColor4fv(TH_WIDGET_TEXT_CURSOR, pd->edit_text.cursor_color);
  srgb_to_linearrgb_v4(pd->edit_text.cursor_color, pd->edit_text.cursor_color);
  DRW_draw_pass(psl->edit_text_cursor_ps);
}
