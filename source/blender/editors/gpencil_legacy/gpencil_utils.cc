/* SPDX-FileCopyrightText: 2014 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgpencil
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "MEM_guardedalloc.h"

#include "BLI_blenlib.h"
#include "BLI_ghash.h"
#include "BLI_hash.h"
#include "BLI_lasso_2d.hh"
#include "BLI_math_color.h"
#include "BLI_math_matrix.h"
#include "BLI_math_vector.hh"
#include "BLI_time.h"
#include "BLI_utildefines.h"

#include "BLT_translation.hh"

#include "DNA_brush_types.h"
#include "DNA_gpencil_legacy_types.h"
#include "DNA_material_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_screen_types.h"
#include "DNA_space_types.h"
#include "DNA_view3d_types.h"

#include "BKE_action.hh"
#include "BKE_brush.hh"
#include "BKE_collection.hh"
#include "BKE_colortools.hh"
#include "BKE_context.hh"
#include "BKE_deform.hh"
#include "BKE_gpencil_curve_legacy.h"
#include "BKE_gpencil_geom_legacy.h"
#include "BKE_gpencil_legacy.h"
#include "BKE_main.hh"
#include "BKE_material.h"
#include "BKE_object.hh"
#include "BKE_paint.hh"
#include "BKE_preview_image.hh"
#include "BKE_tracking.h"

#include "WM_api.hh"
#include "WM_toolsystem.hh"
#include "WM_types.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"
#include "RNA_enum_types.hh"
#include "RNA_prototypes.hh"

#include "UI_resources.hh"
#include "UI_view2d.hh"

#include "ED_clip.hh"
#include "ED_gpencil_legacy.hh"
#include "ED_object.hh"
#include "ED_select_utils.hh"
#include "ED_transform_snap_object_context.hh"
#include "ED_view3d.hh"

#include "GPU_immediate.hh"
#include "GPU_immediate_util.hh"
#include "GPU_state.hh"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_query.hh"

#include "gpencil_intern.hh"

/* ******************************************************** */
/* Context Wrangling... */

bGPdata **ED_gpencil_data_get_pointers_direct(ScrArea *area, Object *ob, PointerRNA *r_ptr)
{
  /* if there's an active area, check if the particular editor may
   * have defined any special Grease Pencil context for editing...
   */
  if (area) {
    switch (area->spacetype) {
      case SPACE_PROPERTIES: /* properties */
      case SPACE_INFO:       /* header info */
      case SPACE_TOPBAR:     /* Top-bar */
      case SPACE_VIEW3D:     /* 3D-View */
      {
        if (ob && (ob->type == OB_GPENCIL_LEGACY)) {
          /* GP Object. */
          if (r_ptr) {
            *r_ptr = RNA_id_pointer_create(&ob->id);
          }
          return (bGPdata **)&ob->data;
        }
        return nullptr;
      }
      default: /* Unsupported space. */
        return nullptr;
    }
  }

  return nullptr;
}

bGPdata **ED_annotation_data_get_pointers_direct(ID *screen_id,
                                                 ScrArea *area,
                                                 Scene *scene,
                                                 PointerRNA *r_ptr)
{
  /* If there's an active area, check if the particular editor may
   * have defined any special Grease Pencil context for editing. */
  if (area) {
    SpaceLink *sl = static_cast<SpaceLink *>(area->spacedata.first);

    switch (area->spacetype) {
      case SPACE_INFO: /* header info */
      {
        return nullptr;
      }

      case SPACE_TOPBAR:     /* Top-bar */
      case SPACE_VIEW3D:     /* 3D-View */
      case SPACE_PROPERTIES: /* properties */
      {
        if (r_ptr) {
          *r_ptr = RNA_id_pointer_create(&scene->id);
        }
        return &scene->gpd;
      }
      case SPACE_NODE: /* Nodes Editor */
      {
        SpaceNode *snode = (SpaceNode *)sl;

        /* return the GP data for the active node block/node */
        if (snode && snode->nodetree) {
          /* for now, as long as there's an active node tree,
           * default to using that in the Nodes Editor */
          if (r_ptr) {
            *r_ptr = RNA_id_pointer_create(&snode->nodetree->id);
          }
          return &snode->nodetree->gpd;
        }

        /* Even when there is no node-tree, don't allow this to flow to scene. */
        return nullptr;
      }
      case SPACE_SEQ: /* Sequencer */
      {
        SpaceSeq *sseq = (SpaceSeq *)sl;

        /* For now, Grease Pencil data is associated with the space
         * (actually preview region only). */
        if (r_ptr) {
          *r_ptr = RNA_pointer_create(screen_id, &RNA_SpaceSequenceEditor, sseq);
        }
        return &sseq->gpd;
      }
      case SPACE_IMAGE: /* Image/UV Editor */
      {
        SpaceImage *sima = (SpaceImage *)sl;

        /* For now, Grease Pencil data is associated with the space... */
        if (r_ptr) {
          *r_ptr = RNA_pointer_create(screen_id, &RNA_SpaceImageEditor, sima);
        }
        return &sima->gpd;
      }
      case SPACE_CLIP: /* Nodes Editor */
      {
        SpaceClip *sc = (SpaceClip *)sl;
        MovieClip *clip = ED_space_clip_get_clip(sc);

        if (clip) {
          if (sc->gpencil_src == SC_GPENCIL_SRC_TRACK) {
            const MovieTrackingObject *tracking_object = BKE_tracking_object_get_active(
                &clip->tracking);
            MovieTrackingTrack *track = tracking_object->active_track;

            if (!track) {
              return nullptr;
            }

            if (r_ptr) {
              *r_ptr = RNA_pointer_create(&clip->id, &RNA_MovieTrackingTrack, track);
            }
            return &track->gpd;
          }
          if (r_ptr) {
            *r_ptr = RNA_id_pointer_create(&clip->id);
          }
          return &clip->gpd;
        }
        break;
      }
      default: /* unsupported space */
        return nullptr;
    }
  }

  return nullptr;
}

bGPdata **ED_gpencil_data_get_pointers(const bContext *C, PointerRNA *r_ptr)
{
  ScrArea *area = CTX_wm_area(C);
  Object *ob = CTX_data_active_object(C);

  return ED_gpencil_data_get_pointers_direct(area, ob, r_ptr);
}

bGPdata **ED_annotation_data_get_pointers(const bContext *C, PointerRNA *r_ptr)
{
  ID *screen_id = (ID *)CTX_wm_screen(C);
  Scene *scene = CTX_data_scene(C);
  ScrArea *area = CTX_wm_area(C);

  return ED_annotation_data_get_pointers_direct(screen_id, area, scene, r_ptr);
}
/* -------------------------------------------------------- */

bGPdata *ED_gpencil_data_get_active_direct(ScrArea *area, Object *ob)
{
  bGPdata **gpd_ptr = ED_gpencil_data_get_pointers_direct(area, ob, nullptr);
  return (gpd_ptr) ? *(gpd_ptr) : nullptr;
}

bGPdata *ED_annotation_data_get_active_direct(ID *screen_id, ScrArea *area, Scene *scene)
{
  bGPdata **gpd_ptr = ED_annotation_data_get_pointers_direct(screen_id, area, scene, nullptr);
  return (gpd_ptr) ? *(gpd_ptr) : nullptr;
}

bGPdata *ED_gpencil_data_get_active(const bContext *C)
{
  Object *ob = CTX_data_active_object(C);
  if ((ob == nullptr) || (ob->type != OB_GPENCIL_LEGACY)) {
    return nullptr;
  }
  return static_cast<bGPdata *>(ob->data);
}

bGPdata *ED_annotation_data_get_active(const bContext *C)
{
  bGPdata **gpd_ptr = ED_annotation_data_get_pointers(C, nullptr);
  return (gpd_ptr) ? *(gpd_ptr) : nullptr;
}

bGPdata *ED_gpencil_data_get_active_evaluated(const bContext *C)
{
  ScrArea *area = CTX_wm_area(C);

  const Depsgraph *depsgraph = CTX_data_ensure_evaluated_depsgraph(C);
  Object *ob = CTX_data_active_object(C);
  Object *ob_eval = DEG_get_evaluated_object(depsgraph, ob);

  return ED_gpencil_data_get_active_direct(area, ob_eval);
}

/* -------------------------------------------------------- */

bool ED_gpencil_data_owner_is_annotation(PointerRNA *owner_ptr)
{
  /* Key Assumption: If the pointer is an object, we're dealing with a GP Object's data.
   * Otherwise, the GP data-block is being used for annotations (i.e. everywhere else). */
  return ((owner_ptr) && (owner_ptr->type != &RNA_Object));
}

/* ******************************************************** */
/* Poll Callbacks */

bool gpencil_add_poll(bContext *C)
{
  Object *ob = CTX_data_active_object(C);
  if (ob == nullptr) {
    return false;
  }
  bGPdata *gpd = (bGPdata *)ob->data;

  return (gpd != nullptr);
}

bool gpencil_active_layer_poll(bContext *C)
{
  Object *ob = CTX_data_active_object(C);
  if ((ob == nullptr) || (ob->type != OB_GPENCIL_LEGACY)) {
    return false;
  }
  bGPdata *gpd = (bGPdata *)ob->data;
  bGPDlayer *gpl = BKE_gpencil_layer_active_get(gpd);

  return (gpl != nullptr);
}

bool gpencil_active_brush_poll(bContext *C)
{
  ToolSettings *ts = CTX_data_tool_settings(C);
  Paint *paint = &ts->gp_paint->paint;
  if (paint) {
    return (BKE_paint_brush(paint) != nullptr);
  }
  return false;
}

/* ******************************************************** */
/* Dynamic Enums of GP Layers */
/* NOTE: These include an option to create a new layer and use that... */

const EnumPropertyItem *ED_gpencil_layers_enum_itemf(bContext *C,
                                                     PointerRNA * /*ptr*/,
                                                     PropertyRNA * /*prop*/,
                                                     bool *r_free)
{
  bGPdata *gpd = CTX_data_gpencil_data(C);
  bGPDlayer *gpl;
  EnumPropertyItem *item = nullptr, item_tmp = {0};
  int totitem = 0;
  int i = 0;

  if (ELEM(nullptr, C, gpd)) {
    return rna_enum_dummy_DEFAULT_items;
  }

  /* Existing layers */
  for (gpl = static_cast<bGPDlayer *>(gpd->layers.first); gpl; gpl = gpl->next, i++) {
    item_tmp.identifier = gpl->info;
    item_tmp.name = gpl->info;
    item_tmp.value = i;

    if (gpl->flag & GP_LAYER_ACTIVE) {
      item_tmp.icon = ICON_GREASEPENCIL;
    }
    else {
      item_tmp.icon = ICON_NONE;
    }

    RNA_enum_item_add(&item, &totitem, &item_tmp);
  }

  RNA_enum_item_end(&item, &totitem);
  *r_free = true;

  return item;
}

const EnumPropertyItem *ED_gpencil_layers_with_new_enum_itemf(bContext *C,
                                                              PointerRNA * /*ptr*/,
                                                              PropertyRNA * /*prop*/,
                                                              bool *r_free)
{
  bGPdata *gpd = CTX_data_gpencil_data(C);
  bGPDlayer *gpl;
  EnumPropertyItem *item = nullptr, item_tmp = {0};
  int totitem = 0;
  int i = 0;

  if (ELEM(nullptr, C, gpd)) {
    return rna_enum_dummy_DEFAULT_items;
  }

  /* Create new layer */
  /* TODO: have some way of specifying that we don't want this? */
  {
    /* "New Layer" entry */
    item_tmp.identifier = "__CREATE__";
    item_tmp.name = "New Layer";
    item_tmp.value = -1;
    item_tmp.icon = ICON_ADD;
    RNA_enum_item_add(&item, &totitem, &item_tmp);

    /* separator */
    RNA_enum_item_add_separator(&item, &totitem);
  }

  const int tot = BLI_listbase_count(&gpd->layers);
  /* Existing layers */
  for (gpl = static_cast<bGPDlayer *>(gpd->layers.last), i = 0; gpl; gpl = gpl->prev, i++) {
    item_tmp.identifier = gpl->info;
    item_tmp.name = gpl->info;
    item_tmp.value = tot - i - 1;

    if (gpl->flag & GP_LAYER_ACTIVE) {
      item_tmp.icon = ICON_GREASEPENCIL;
    }
    else {
      item_tmp.icon = ICON_NONE;
    }

    RNA_enum_item_add(&item, &totitem, &item_tmp);
  }

  RNA_enum_item_end(&item, &totitem);
  *r_free = true;

  return item;
}

const EnumPropertyItem *ED_gpencil_material_enum_itemf(bContext *C,
                                                       PointerRNA * /*ptr*/,
                                                       PropertyRNA * /*prop*/,
                                                       bool *r_free)
{
  Object *ob = CTX_data_active_object(C);
  EnumPropertyItem *item = nullptr, item_tmp = {0};
  int totitem = 0;
  int i = 0;

  if (ELEM(nullptr, C, ob)) {
    return rna_enum_dummy_DEFAULT_items;
  }

  /* Existing materials */
  for (i = 1; i <= ob->totcol; i++) {
    Material *ma = BKE_object_material_get(ob, i);
    if (ma) {
      item_tmp.identifier = ma->id.name + 2;
      item_tmp.name = ma->id.name + 2;
      item_tmp.value = i;
      item_tmp.icon = ma->preview ? ma->preview->runtime->icon_id : ICON_NONE;

      RNA_enum_item_add(&item, &totitem, &item_tmp);
    }
  }

  RNA_enum_item_end(&item, &totitem);
  *r_free = true;

  return item;
}

/* ******************************************************** */
/* Brush Tool Core */

bool gpencil_stroke_inside_circle(const float mval[2], int rad, int x0, int y0, int x1, int y1)
{
  /* simple within-radius check for now */
  const float screen_co_a[2] = {float(x0), float(y0)};
  const float screen_co_b[2] = {float(x1), float(y1)};

  if (edge_inside_circle(mval, rad, screen_co_a, screen_co_b)) {
    return true;
  }

  /* not inside */
  return false;
}

/* ******************************************************** */
/* Stroke Validity Testing */

bool ED_gpencil_stroke_can_use_direct(const ScrArea *area, const bGPDstroke *gps)
{
  /* sanity check */
  if (ELEM(nullptr, area, gps)) {
    return false;
  }

  /* filter stroke types by flags + spacetype */
  if (gps->flag & GP_STROKE_3DSPACE) {
    /* 3D strokes - only in 3D view */
    return ELEM(area->spacetype, SPACE_VIEW3D, SPACE_PROPERTIES);
  }
  if (gps->flag & GP_STROKE_2DIMAGE) {
    /* Special "image" strokes - only in Image Editor */
    return (area->spacetype == SPACE_IMAGE);
  }
  if (gps->flag & GP_STROKE_2DSPACE) {
    /* 2D strokes (data-space) - for any 2D view (i.e. everything other than 3D view). */
    return (area->spacetype != SPACE_VIEW3D);
  }
  /* view aligned - anything goes */
  return true;
}

bool ED_gpencil_stroke_can_use(const bContext *C, const bGPDstroke *gps)
{
  ScrArea *area = CTX_wm_area(C);
  return ED_gpencil_stroke_can_use_direct(area, gps);
}

bool ED_gpencil_stroke_material_editable(Object *ob, const bGPDlayer *gpl, const bGPDstroke *gps)
{
  /* check if the color is editable */
  MaterialGPencilStyle *gp_style = BKE_gpencil_material_settings(ob, gps->mat_nr + 1);

  if (gp_style != nullptr) {
    if (gp_style->flag & GP_MATERIAL_HIDE) {
      return false;
    }
    if (((gpl->flag & GP_LAYER_UNLOCK_COLOR) == 0) && (gp_style->flag & GP_MATERIAL_LOCKED)) {
      return false;
    }
  }

  return true;
}

/* ******************************************************** */
/* Space Conversion */

void gpencil_point_conversion_init(bContext *C, GP_SpaceConversion *r_gsc)
{
  ScrArea *area = CTX_wm_area(C);
  ARegion *region = CTX_wm_region(C);

  /* zero out the storage (just in case) */
  memset(r_gsc, 0, sizeof(GP_SpaceConversion));
  unit_m4(r_gsc->mat);

  /* store settings */
  r_gsc->scene = CTX_data_scene(C);
  r_gsc->ob = CTX_data_active_object(C);

  r_gsc->area = area;
  r_gsc->region = region;
  r_gsc->v2d = &region->v2d;

  /* init region-specific stuff */
  if (area->spacetype == SPACE_VIEW3D) {
    wmWindow *win = CTX_wm_window(C);
    Scene *scene = CTX_data_scene(C);
    Depsgraph *depsgraph = CTX_data_ensure_evaluated_depsgraph(C);
    View3D *v3d = (View3D *)CTX_wm_space_data(C);
    RegionView3D *rv3d = static_cast<RegionView3D *>(region->regiondata);

    /* init 3d depth buffers */
    view3d_operator_needs_opengl(C);

    view3d_region_operator_needs_opengl(win, region);
    ED_view3d_depth_override(depsgraph, region, v3d, nullptr, V3D_DEPTH_NO_GPENCIL, nullptr);

    /* for camera view set the subrect */
    if (rv3d->persp == RV3D_CAMOB) {
      ED_view3d_calc_camera_border(
          scene, depsgraph, region, v3d, rv3d, true, &r_gsc->subrect_data);
      r_gsc->subrect = &r_gsc->subrect_data;
    }
  }
}

void gpencil_point_to_world_space(const bGPDspoint *pt,
                                  const float diff_mat[4][4],
                                  bGPDspoint *r_pt)
{
  mul_v3_m4v3(&r_pt->x, diff_mat, &pt->x);
}

void gpencil_world_to_object_space(Depsgraph *depsgraph,
                                   Object *obact,
                                   bGPDlayer *gpl,
                                   bGPDstroke *gps)
{
  bGPDspoint *pt;
  int i;

  /* undo matrix */
  float diff_mat[4][4];
  float inverse_diff_mat[4][4];

  BKE_gpencil_layer_transform_matrix_get(depsgraph, obact, gpl, diff_mat);
  zero_axis_bias_m4(diff_mat);
  invert_m4_m4(inverse_diff_mat, diff_mat);

  for (i = 0; i < gps->totpoints; i++) {
    pt = &gps->points[i];
    mul_m4_v3(inverse_diff_mat, &pt->x);
  }
}

void gpencil_world_to_object_space_point(Depsgraph *depsgraph,
                                         Object *obact,
                                         bGPDlayer *gpl,
                                         bGPDspoint *pt)
{
  /* undo matrix */
  float diff_mat[4][4];
  float inverse_diff_mat[4][4];

  BKE_gpencil_layer_transform_matrix_get(depsgraph, obact, gpl, diff_mat);
  zero_axis_bias_m4(diff_mat);
  invert_m4_m4(inverse_diff_mat, diff_mat);

  mul_m4_v3(inverse_diff_mat, &pt->x);
}

void gpencil_point_to_xy(
    const GP_SpaceConversion *gsc, const bGPDstroke *gps, const bGPDspoint *pt, int *r_x, int *r_y)
{
  const ARegion *region = gsc->region;
  const View2D *v2d = gsc->v2d;
  const rctf *subrect = gsc->subrect;
  int xyval[2];

  /* sanity checks */
  BLI_assert(!(gps->flag & GP_STROKE_3DSPACE) || (gsc->area->spacetype == SPACE_VIEW3D));
  BLI_assert(!(gps->flag & GP_STROKE_2DSPACE) || (gsc->area->spacetype != SPACE_VIEW3D));

  if (gps->flag & GP_STROKE_3DSPACE) {
    if (ED_view3d_project_int_global(region, &pt->x, xyval, V3D_PROJ_TEST_NOP) == V3D_PROJ_RET_OK)
    {
      *r_x = xyval[0];
      *r_y = xyval[1];
    }
    else {
      *r_x = V2D_IS_CLIPPED;
      *r_y = V2D_IS_CLIPPED;
    }
  }
  else if (gps->flag & GP_STROKE_2DSPACE) {
    float vec[3] = {pt->x, pt->y, 0.0f};
    mul_m4_v3(gsc->mat, vec);
    UI_view2d_view_to_region_clip(v2d, vec[0], vec[1], r_x, r_y);
  }
  else {
    if (subrect == nullptr) {
      /* normal 3D view (or view space) */
      *r_x = int(pt->x / 100 * region->winx);
      *r_y = int(pt->y / 100 * region->winy);
    }
    else {
      /* camera view, use subrect */
      *r_x = int((pt->x / 100) * BLI_rctf_size_x(subrect)) + subrect->xmin;
      *r_y = int((pt->y / 100) * BLI_rctf_size_y(subrect)) + subrect->ymin;
    }
  }
}

void gpencil_point_to_xy_fl(const GP_SpaceConversion *gsc,
                            const bGPDstroke *gps,
                            const bGPDspoint *pt,
                            float *r_x,
                            float *r_y)
{
  const ARegion *region = gsc->region;
  const View2D *v2d = gsc->v2d;
  const rctf *subrect = gsc->subrect;
  float xyval[2];

  /* sanity checks */
  BLI_assert(!(gps->flag & GP_STROKE_3DSPACE) || (gsc->area->spacetype == SPACE_VIEW3D));
  BLI_assert(!(gps->flag & GP_STROKE_2DSPACE) || (gsc->area->spacetype != SPACE_VIEW3D));

  if (gps->flag & GP_STROKE_3DSPACE) {
    if (ED_view3d_project_float_global(region, &pt->x, xyval, V3D_PROJ_TEST_NOP) ==
        V3D_PROJ_RET_OK)
    {
      *r_x = xyval[0];
      *r_y = xyval[1];
    }
    else {
      *r_x = 0.0f;
      *r_y = 0.0f;
    }
  }
  else if (gps->flag & GP_STROKE_2DSPACE) {
    float vec[3] = {pt->x, pt->y, 0.0f};
    int t_x, t_y;

    mul_m4_v3(gsc->mat, vec);
    UI_view2d_view_to_region_clip(v2d, vec[0], vec[1], &t_x, &t_y);

    if ((t_x == t_y) && (t_x == V2D_IS_CLIPPED)) {
      /* XXX: Or should we just always use the values as-is? */
      *r_x = 0.0f;
      *r_y = 0.0f;
    }
    else {
      *r_x = float(t_x);
      *r_y = float(t_y);
    }
  }
  else {
    if (subrect == nullptr) {
      /* normal 3D view (or view space) */
      *r_x = (pt->x / 100.0f * region->winx);
      *r_y = (pt->y / 100.0f * region->winy);
    }
    else {
      /* camera view, use subrect */
      *r_x = ((pt->x / 100.0f) * BLI_rctf_size_x(subrect)) + subrect->xmin;
      *r_y = ((pt->y / 100.0f) * BLI_rctf_size_y(subrect)) + subrect->ymin;
    }
  }
}

void gpencil_point_3d_to_xy(const GP_SpaceConversion *gsc,
                            const short flag,
                            const float pt[3],
                            float xy[2])
{
  const ARegion *region = gsc->region;
  const View2D *v2d = gsc->v2d;
  const rctf *subrect = gsc->subrect;
  float xyval[2];

  /* sanity checks */
  BLI_assert(gsc->area->spacetype == SPACE_VIEW3D);

  if (flag & GP_STROKE_3DSPACE) {
    if (ED_view3d_project_float_global(region, pt, xyval, V3D_PROJ_TEST_NOP) == V3D_PROJ_RET_OK) {
      xy[0] = xyval[0];
      xy[1] = xyval[1];
    }
    else {
      xy[0] = 0.0f;
      xy[1] = 0.0f;
    }
  }
  else if (flag & GP_STROKE_2DSPACE) {
    float vec[3] = {pt[0], pt[1], 0.0f};
    int t_x, t_y;

    mul_m4_v3(gsc->mat, vec);
    UI_view2d_view_to_region_clip(v2d, vec[0], vec[1], &t_x, &t_y);

    if ((t_x == t_y) && (t_x == V2D_IS_CLIPPED)) {
      /* XXX: Or should we just always use the values as-is? */
      xy[0] = 0.0f;
      xy[1] = 0.0f;
    }
    else {
      xy[0] = float(t_x);
      xy[1] = float(t_y);
    }
  }
  else {
    if (subrect == nullptr) {
      /* normal 3D view (or view space) */
      xy[0] = (pt[0] / 100.0f * region->winx);
      xy[1] = (pt[1] / 100.0f * region->winy);
    }
    else {
      /* camera view, use subrect */
      xy[0] = ((pt[0] / 100.0f) * BLI_rctf_size_x(subrect)) + subrect->xmin;
      xy[1] = ((pt[1] / 100.0f) * BLI_rctf_size_y(subrect)) + subrect->ymin;
    }
  }
}

void ED_gpencil_drawing_reference_get(const Scene *scene,
                                      const Object *ob,
                                      char align_flag,
                                      float r_vec[3])
{
  const float *fp = scene->cursor.location;

  /* if using a gpencil object at cursor mode, can use the location of the object */
  if (align_flag & GP_PROJECT_VIEWSPACE) {
    if (ob && (ob->type == OB_GPENCIL_LEGACY)) {
      /* fallback (no strokes) - use cursor or object location */
      if (align_flag & GP_PROJECT_CURSOR) {
        /* use 3D-cursor */
        copy_v3_v3(r_vec, fp);
      }
      else {
        /* use object location */
        copy_v3_v3(r_vec, ob->object_to_world().location());
        /* Apply layer offset. */
        bGPdata *gpd = static_cast<bGPdata *>(ob->data);
        bGPDlayer *gpl = BKE_gpencil_layer_active_get(gpd);
        if (gpl != nullptr) {
          add_v3_v3(r_vec, gpl->layer_mat[3]);
        }
      }
    }
  }
  else {
    /* use 3D-cursor */
    copy_v3_v3(r_vec, fp);
  }
}

/**
 * Helper to convert 2d to 3d for simple drawing buffer.
 */
static void gpencil_stroke_convertcoords(ARegion *region,
                                         const tGPspoint *point2D,
                                         const float origin[3],
                                         float out[3])
{
  float mval_prj[2];
  float rvec[3];

  copy_v3_v3(rvec, origin);

  const float zfac = ED_view3d_calc_zfac(static_cast<const RegionView3D *>(region->regiondata),
                                         rvec);

  if (ED_view3d_project_float_global(region, rvec, mval_prj, V3D_PROJ_TEST_NOP) == V3D_PROJ_RET_OK)
  {
    float dvec[3];
    float xy_delta[2];
    sub_v2_v2v2(xy_delta, mval_prj, point2D->m_xy);
    ED_view3d_win_to_delta(region, xy_delta, zfac, dvec);
    sub_v3_v3v3(out, rvec, dvec);
  }
  else {
    zero_v3(out);
  }
}

void ED_gpencil_tpoint_to_point(ARegion *region,
                                float origin[3],
                                const tGPspoint *tpt,
                                bGPDspoint *pt)
{
  float p3d[3];
  /* conversion to 3d format */
  gpencil_stroke_convertcoords(region, tpt, origin, p3d);
  copy_v3_v3(&pt->x, p3d);
  zero_v4(pt->vert_color);

  pt->pressure = tpt->pressure;
  pt->strength = tpt->strength;
  pt->uv_fac = tpt->uv_fac;
  pt->uv_rot = tpt->uv_rot;
}

tGPspoint *ED_gpencil_sbuffer_ensure(tGPspoint *buffer_array,
                                     int *buffer_size,
                                     int *buffer_used,
                                     const bool clear)
{
  tGPspoint *p = nullptr;

  /* By default a buffer is created with one block with a predefined number of free points,
   * if the size is not enough, the cache is reallocated adding a new block of free points.
   * This is done in order to keep cache small and improve speed. */
  if (*buffer_used + 1 > *buffer_size) {
    if ((*buffer_size == 0) || (buffer_array == nullptr)) {
      p = static_cast<tGPspoint *>(
          MEM_callocN(sizeof(tGPspoint) * GP_STROKE_BUFFER_CHUNK, "GPencil Sbuffer"));
      *buffer_size = GP_STROKE_BUFFER_CHUNK;
    }
    else {
      *buffer_size += GP_STROKE_BUFFER_CHUNK;
      p = static_cast<tGPspoint *>(MEM_recallocN(buffer_array, sizeof(tGPspoint) * *buffer_size));
    }

    if (p == nullptr) {
      *buffer_size = *buffer_used = 0;
    }

    buffer_array = p;
  }

  /* clear old data */
  if (clear) {
    *buffer_used = 0;
    if (buffer_array != nullptr) {
      memset(buffer_array, 0, sizeof(tGPspoint) * *buffer_size);
    }
  }

  return buffer_array;
}

static void gpencil_layer_new_name_get(bGPdata *gpd, char *r_name, size_t name_maxncpy)
{
  int index = 0;
  LISTBASE_FOREACH (bGPDlayer *, gpl, &gpd->layers) {
    if (strstr(gpl->info, "GP_Layer")) {
      index++;
    }
  }

  if (index == 0) {
    BLI_strncpy(r_name, "GP_Layer", name_maxncpy);
    return;
  }
  BLI_snprintf(r_name, name_maxncpy, "GP_Layer.%03d", index);
}

int ED_gpencil_new_layer_dialog(bContext *C, wmOperator *op)
{
  Object *ob = CTX_data_active_object(C);
  PropertyRNA *prop;
  if (RNA_int_get(op->ptr, "layer") == -1) {
    prop = RNA_struct_find_property(op->ptr, "new_layer_name");
    if (!RNA_property_is_set(op->ptr, prop)) {
      char name[MAX_NAME];
      bGPdata *gpd = static_cast<bGPdata *>(ob->data);
      gpencil_layer_new_name_get(gpd, name, sizeof(name));
      RNA_property_string_set(op->ptr, prop, name);
      return WM_operator_props_dialog_popup(C,
                                            op,
                                            200,
                                            IFACE_("Add New Layer"),
                                            CTX_IFACE_(BLT_I18NCONTEXT_OPERATOR_DEFAULT, "Add"));
    }
  }
  return 0;
}
