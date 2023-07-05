/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edtransform
 */

#include <cstdlib>

#include "MEM_guardedalloc.h"

#include "BLI_math_vector_types.hh"
#include "BLI_string.h"
#include "BLI_utildefines_stack.h"

#include "BKE_context.h"
#include "BKE_editmesh.h"
#include "BKE_editmesh_bvh.h"
#include "BKE_layer.h"
#include "BKE_unit.h"

#include "GPU_immediate.h"
#include "GPU_matrix.h"
#include "GPU_state.h"

#include "ED_mesh.hh"
#include "ED_screen.hh"
#include "ED_uvedit.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "RNA_access.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"
#include "UI_view2d.hh"

#include "BLT_translation.h"

#include "transform.hh"
#include "transform_constraints.hh"
#include "transform_convert.hh"
#include "transform_mode.hh"
#include "transform_snap.hh"

/* -------------------------------------------------------------------- */
/** \name Transform (Edge UV Slide)
 * \{ */
struct EdgeSlideUV {
  /** Loops associated with this UV which are being transformed. */
  blender::Vector<BMLoop *> loops;
  /** A pair of loops whose UVs are target locations in UV space to slide betwen. */
  BMLoop *slide_loop[2];
  /** A pair of target locations in UV space to slide betwen. */
  blender::float2 slide_uv_target[2];
  /** The original location (never changes), used to re-calculate the slide location. */
  blender::float2 orig_uv;
};

static void freeEdgeSlideUVs(TransInfo * /*t*/,
                             TransDataContainer * /*tc*/,
                             TransCustomData *custom_data)
{
  blender::Vector<EdgeSlideUV> *esuvs = static_cast<blender::Vector<EdgeSlideUV> *>(
      custom_data->data);

  if (esuvs == nullptr) {
    return;
  }

  delete esuvs;

  custom_data->data = nullptr;
}
static void calc_side_loops_from_single_terminating_uv(BMEdge *edge,
                                                       BMLoop *orig_loop,
                                                       EdgeSlideUV &esuv,
                                                       BMUVOffsets offsets)
{
  BMEdge *e = (edge == nullptr) ? orig_loop->e : edge;
  BMLoop *loop_v, *loop_side;
  BMIter lv_iter, side_iter;
  BMVert *v_orig = orig_loop->v;
  int i = 0;

  /* Ignore non-manifold edges. */
  if (!BM_edge_is_manifold(e)) {
    return;
  }

  /*Clear tags*/
  BM_ITER_ELEM (loop_v, &lv_iter, v_orig, BM_LOOPS_OF_VERT) {
    BM_elem_flag_disable(loop_v->next, BM_ELEM_TAG_ALT);
    BM_elem_flag_disable(loop_v->prev, BM_ELEM_TAG_ALT);
  }

  BM_ITER_ELEM (loop_v, &lv_iter, v_orig, BM_LOOPS_OF_VERT) {
    if (BM_loop_uv_share_vert_check(loop_v, orig_loop, offsets.uv) &&
        (loop_v->f == e->l->f || loop_v->f == e->l->radial_next->f))
    {
      for (int dir = 0; dir < 2; dir++) {
        BMLoop *l_other = dir ? loop_v->next : loop_v->prev;

        if (!BM_ELEM_CD_GET_BOOL(l_other, offsets.select_vert) &&
            !BM_elem_flag_test(l_other, BM_ELEM_TAG_ALT) && l_other->v != e->v1 &&
            l_other->v != e->v2)
        {
          /*We should never find more than 2 loops that share faces with the edge. */
          BLI_assert(i < 2);
          esuv.slide_loop[i] = l_other;
          i++;

          /*Tag all loops of the side so the same side isn't picked twice. */
          BM_ITER_ELEM (loop_side, &side_iter, l_other->v, BM_LOOPS_OF_VERT) {
            if (BM_loop_uv_share_vert_check(loop_side, l_other, offsets.uv)) {
              BM_elem_flag_enable(loop_side, BM_ELEM_TAG_ALT);
            }
          }
        }
      }
    }
  }
}

static void calc_side_loops_from_pair(BMLoop *orig_loop, EdgeSlideUV &esuv, BMUVOffsets offsets)
{
  BMLoop *loop_v, *loop_side;
  BMIter lv_iter, side_iter;
  BMVert *v_orig = orig_loop->v;
  int i = 0;

  /*Clear tags*/
  BM_ITER_ELEM (loop_v, &lv_iter, v_orig, BM_LOOPS_OF_VERT) {
    BM_elem_flag_disable(loop_v->next, BM_ELEM_TAG_ALT);
    BM_elem_flag_disable(loop_v->prev, BM_ELEM_TAG_ALT);
  }

  BM_ITER_ELEM (loop_v, &lv_iter, v_orig, BM_LOOPS_OF_VERT) {
    /*Pick the first two unselected edges encountered to slide along. */
    if (i == 2) {
      break;
    }
    if (BM_loop_uv_share_vert_check(loop_v, orig_loop, offsets.uv)) {

      for (int dir = 0; dir < 2; dir++) {
        BMLoop *l_other = dir ? loop_v->next : loop_v->prev;

        if (!BM_ELEM_CD_GET_BOOL(l_other, offsets.select_vert) &&
            !BM_elem_flag_test(l_other, BM_ELEM_TAG_ALT))
        {
          esuv.slide_loop[i] = l_other;
          i++;

          /*Tag all loops of the side so the same side isn't picked twice. */
          BM_ITER_ELEM (loop_side, &side_iter, l_other->v, BM_LOOPS_OF_VERT) {
            if (BM_loop_uv_share_vert_check(loop_side, l_other, offsets.uv)) {
              BM_elem_flag_enable(loop_side, BM_ELEM_TAG_ALT);
            }
          }
        }
      }
    }
  }
}

static void flip_and_set_sides(blender::Vector<EdgeSlideUV> &slide_uv, BMUVOffsets offsets)
{
  BMLoop *l_iter;
  BMIter lv_iter;
  BMVert *v_orig;

  for (int i = 0; i < slide_uv.size(); i++) {
    /*Initialize as selected uv. */
    float *uv = BM_ELEM_CD_GET_FLOAT_P(slide_uv[i].loops[0], offsets.uv);
    slide_uv[i].slide_uv_target[0] = uv;
    slide_uv[i].slide_uv_target[1] = uv;

    slide_uv[i].orig_uv = uv;

    if (i != 0) {
      /* Side 0 is tagged with BM_ELEM_TAG and side 1 is tagged with BM_ELEM_TAG_ALT. */
      bool flip = false;
      if (slide_uv[i].slide_loop[0]->v != slide_uv[i].loops[0]->v) {
        v_orig = slide_uv[i].slide_loop[0]->v;

        BM_ITER_ELEM (l_iter, &lv_iter, v_orig, BM_LOOPS_OF_VERT) {
          if (BM_loop_uv_share_vert_check(l_iter, slide_uv[i].slide_loop[0], offsets.uv)) {
            if (l_iter->next->v == slide_uv[i].loops[0]->v ||
                l_iter->prev->v == slide_uv[i].loops[0]->v)
            {
              /*Only look at faces adjacent to the edges. */

              if (BM_elem_flag_test(l_iter->f, BM_ELEM_TAG_ALT)) {
                /*Flip if wrong tag found. */
                flip = true;
              }
            }
          }
        }
      }

      if (slide_uv[i].slide_loop[1]->v != slide_uv[i].loops[0]->v) {

        v_orig = slide_uv[i].slide_loop[1]->v;

        BM_ITER_ELEM (l_iter, &lv_iter, v_orig, BM_LOOPS_OF_VERT) {
          if (BM_loop_uv_share_vert_check(l_iter, slide_uv[i].slide_loop[1], offsets.uv)) {
            if (l_iter->next->v == slide_uv[i].loops[0]->v ||
                l_iter->prev->v == slide_uv[i].loops[0]->v)
            {
              /*Only look at faces adjacent to the edges. */

              if (BM_elem_flag_test(l_iter->f, BM_ELEM_TAG)) {
                /*Flip if wrong tag found. */
                flip = true;
              }
            }
          }
        }
      }

      if (flip) {
        BMLoop *s = slide_uv[i].slide_loop[0];

        slide_uv[i].slide_loop[0] = slide_uv[i].slide_loop[1];
        slide_uv[i].slide_loop[1] = s;
      }
    }

    float *side0 = BM_ELEM_CD_GET_FLOAT_P(slide_uv[i].slide_loop[0], offsets.uv);
    float *side1 = BM_ELEM_CD_GET_FLOAT_P(slide_uv[i].slide_loop[1], offsets.uv);

    slide_uv[i].slide_uv_target[0] = side0;
    slide_uv[i].slide_uv_target[1] = side1;

    /*Tag faces*/
    v_orig = slide_uv[i].slide_loop[0]->v;
    BM_ITER_ELEM (l_iter, &lv_iter, v_orig, BM_LOOPS_OF_VERT) {
      if (BM_loop_uv_share_vert_check(l_iter, slide_uv[i].slide_loop[0], offsets.uv)) {
        if (l_iter->next->v == slide_uv[i].loops[0]->v ||
            l_iter->prev->v == slide_uv[i].loops[0]->v)
        {
          BM_elem_flag_enable(l_iter->f, BM_ELEM_TAG);
        }
      }
    }

    v_orig = slide_uv[i].slide_loop[1]->v;

    BM_ITER_ELEM (l_iter, &lv_iter, v_orig, BM_LOOPS_OF_VERT) {
      if (BM_loop_uv_share_vert_check(l_iter, slide_uv[i].slide_loop[1], offsets.uv)) {
        if (l_iter->next->v == slide_uv[i].loops[0]->v ||
            l_iter->prev->v == slide_uv[i].loops[0]->v)
        {
          BM_elem_flag_enable(l_iter->f, BM_ELEM_TAG_ALT);
        }
      }
    }
  }
}

static blender::Vector<EdgeSlideUV> *create_EdgeSlideUVs(TransInfo *t, TransDataContainer *tc)
{
  Scene *scene = t->scene;
  BMEditMesh *em = BKE_editmesh_from_object(tc->obedit);
  const BMUVOffsets offsets = BM_uv_map_get_offsets(em->bm);

  BMFace *efa;
  BMLoop *l;
  BMIter iter, liter;
  blender::Vector<EdgeSlideUV> *slide_edge_loops = new blender::Vector<EdgeSlideUV>;

  /*Clear tags*/
  BM_ITER_MESH (efa, &iter, em->bm, BM_FACES_OF_MESH) {
    BM_elem_flag_disable(efa, BM_ELEM_TAG);
    BM_elem_flag_disable(efa, BM_ELEM_TAG_ALT);
    BM_ITER_ELEM (l, &liter, efa, BM_LOOPS_OF_FACE) {
      BM_elem_flag_disable(l, BM_ELEM_TAG);
      BM_elem_flag_disable(l, BM_ELEM_TAG_ALT);
    }
  }

  BM_ITER_MESH (efa, &iter, em->bm, BM_FACES_OF_MESH) {
    if (!uvedit_face_visible_test(scene, efa)) {
      continue;
    }
    BM_ITER_ELEM (l, &liter, efa, BM_LOOPS_OF_FACE) {
      if (BM_elem_flag_test(l, BM_ELEM_TAG)) {
        continue;
      }
      if (uvedit_uv_select_test(scene, l, offsets)) {

        blender::Vector<EdgeSlideUV> slide_uv;
        EdgeSlideUV esuv;
        BMLoop *loop_v, *fan_iter;
        BMIter lv_iter;
        BMVert *v_orig = l->v;
        blender::Vector<BMLoop *> connected_loops;
        blender::Set<BMEdge *> unique_edges_orig, unique_edges_iter;

        /* Initialize*/
        esuv.slide_loop[0] = l;
        esuv.slide_loop[1] = l;

        /*Check how many other selected UVs are connected. */
        BM_ITER_ELEM (loop_v, &lv_iter, v_orig, BM_LOOPS_OF_VERT) {
          if (BM_loop_uv_share_vert_check(loop_v, l, offsets.uv)) {
            esuv.loops.append(loop_v);
            BM_elem_flag_enable(loop_v, BM_ELEM_TAG);

            if (uvedit_uv_select_test(scene, loop_v->next, offsets) &&
                !BM_elem_flag_test(loop_v->next, BM_ELEM_TAG))
            {
              connected_loops.append(loop_v->next);
              fan_iter = loop_v;
              unique_edges_orig.add(loop_v->e);
            }
            else if (uvedit_uv_select_test(scene, loop_v->prev, offsets) &&
                     !BM_elem_flag_test(loop_v->prev, BM_ELEM_TAG))
            {
              connected_loops.append(loop_v->prev);
              fan_iter = loop_v;
              unique_edges_orig.add(loop_v->prev->e);
            }
          }
        }

        /*Any vertex with 3+ connected selected edges is ignored. */
        if (unique_edges_orig.size() > 2) {
          continue;
        }

        if (unique_edges_orig.size() == 0) {
          calc_side_loops_from_single_terminating_uv(nullptr, l, esuv, offsets);
        }
        else if (unique_edges_orig.size() == 1) {
          calc_side_loops_from_single_terminating_uv(
              *unique_edges_orig.begin(), fan_iter, esuv, offsets);
        }
        else if (unique_edges_orig.size() == 2) {
          calc_side_loops_from_pair(l, esuv, offsets);
        }

        blender::Vector<EdgeSlideUV> slide_uv1;
        blender::Vector<EdgeSlideUV> slide_uv2;
        slide_uv1.append(esuv);
        slide_uv2.append(esuv);

        for (int i = 0; i < connected_loops.size(); i++) {
          unique_edges_iter = unique_edges_orig;
          BMLoop *connected_loop = connected_loops[i];
          bool has_next = true;

          if (BM_elem_flag_test(connected_loops[i], BM_ELEM_TAG)) {
            continue;
          } /*Prevents from accessing the same direction twice. */
          if (slide_uv1.size() != 1 && slide_uv2.size() != 1) {
            break;
          } /*Stops when 2 sides are found. */

          blender::Vector<EdgeSlideUV> *slide_uv_temp = (slide_uv1.size() == 1) ? &slide_uv1 :
                                                                                  &slide_uv2;

          while (has_next) {
            blender::Set<BMEdge *> prev_unique_edges = unique_edges_iter;
            unique_edges_iter.clear();

            EdgeSlideUV esuv_next;
            BMVert *v_curr = connected_loop->v;
            BMLoop *loop_curr = connected_loop;
            has_next = false;

            /* Initialize*/
            esuv_next.slide_loop[0] = connected_loop;
            esuv_next.slide_loop[1] = connected_loop;

            BM_ITER_ELEM (loop_v, &lv_iter, v_curr, BM_LOOPS_OF_VERT) {
              if (BM_loop_uv_share_vert_check(loop_curr, loop_v, offsets.uv)) {
                esuv_next.loops.append(loop_v);
                BM_elem_flag_enable(loop_v, BM_ELEM_TAG);

                if (uvedit_uv_select_test(scene, loop_v->next, offsets) &&
                    !BM_elem_flag_test(loop_v->next, BM_ELEM_TAG))
                {
                  connected_loop = loop_v->next;
                  unique_edges_iter.add(loop_v->e);
                  fan_iter = loop_v;
                  has_next = true;
                }
                else if (uvedit_uv_select_test(scene, loop_v->prev, offsets) &&
                         !BM_elem_flag_test(loop_v->prev, BM_ELEM_TAG))
                {
                  connected_loop = loop_v->prev;
                  fan_iter = loop_v;
                  unique_edges_iter.add(loop_v->prev->e);
                  has_next = true;
                }
              }
            }

            /*Any vertex with 3+ connected selected edges is ignored. */
            if (unique_edges_iter.size() > 1) {
              break;
            }

            EdgeSlideUV previous = slide_uv_temp->last();
            /*We didn't find a connected loop. */
            if (connected_loop == loop_curr) {
              /*Find edge connecting previous point to current*/
              BMEdge *edge = nullptr;
              for (const auto &e : prev_unique_edges) {
                if (e->v1 == connected_loop->v || e->v2 == connected_loop->v) {
                  edge = e;
                }
              }

              /*We should find a connecting edge*/
              BLI_assert(edge != nullptr);

              calc_side_loops_from_single_terminating_uv(edge, connected_loop, esuv_next, offsets);
            }
            else {
              calc_side_loops_from_pair(loop_curr, esuv_next, offsets);
            }

            slide_uv_temp->append(esuv_next);
          }
        }

        /*Flip slide_uv1 and remove the last element and combine it with slide_uv2. */
        std::reverse(slide_uv1.begin(), slide_uv1.end());
        slide_uv1.pop_last();

        slide_uv.reserve(slide_uv1.size() + slide_uv2.size());
        slide_uv.insert(slide_uv.end(), slide_uv1.begin(), slide_uv1.end());
        slide_uv.insert(slide_uv.end(), slide_uv2.begin(), slide_uv2.end());

        flip_and_set_sides(slide_uv, offsets);
        slide_edge_loops->insert(slide_edge_loops->end(), slide_uv.begin(), slide_uv.end());
      }
    }
  }

  return slide_edge_loops;
}

static void applyEdgeUVSlide(TransInfo *t)
{
  float final = (fabsf(t->values[0]) > 1) ? 1 : fabsf(t->values[0]);
  int side_index = t->values[0] < 0;

  FOREACH_TRANS_DATA_CONTAINER (t, tc) {
    BMEditMesh *em = BKE_editmesh_from_object(tc->obedit);
    const BMUVOffsets offsets = BM_uv_map_get_offsets(em->bm);

    blender::Vector<EdgeSlideUV> *slide_edge_loops = static_cast<blender::Vector<EdgeSlideUV> *>(
        tc->custom.mode.data);

    for (int i = 0; i < slide_edge_loops->size(); i++) {
      for (int j = 0; j < (*slide_edge_loops)[i].loops.size(); j++) {
        float *side = (*slide_edge_loops)[i].slide_uv_target[side_index];
        float *orig = (*slide_edge_loops)[i].orig_uv;
        float *luv = BM_ELEM_CD_GET_FLOAT_P((*slide_edge_loops)[i].loops[j], offsets.uv);
        interp_v2_v2v2(luv, orig, side, fabsf(final));
      }
    }

    DEG_id_tag_update(static_cast<ID *>(tc->obedit->data), ID_RECALC_GEOMETRY);
    WM_main_add_notifier(NC_GEOM | ND_DATA, tc->obedit->data);
  }
}

static void calcEdgeSlideCustomPointsUV(TransInfo *t)
{
  float max_len = 0;
  float *start_uv, *end_uv;
  int start_region[2], end_region[2];

  blender::Vector<EdgeSlideUV> slide_edge_loops = *(static_cast<blender::Vector<EdgeSlideUV> *>(
      TRANS_DATA_CONTAINER_FIRST_OK(t)->custom.mode.data));
  /* Find the longest slide edge to set the range of the mouse input. */
  for (int i = 0; i < slide_edge_loops.size(); i++) {
    float side0_len = len_squared_v2v2(slide_edge_loops[i].slide_uv_target[0],
                                       slide_edge_loops[i].orig_uv);
    float side1_len = len_squared_v2v2(slide_edge_loops[i].slide_uv_target[1],
                                       slide_edge_loops[i].orig_uv);
    float side_len_max;
    int side_max;

    if (side0_len >= side1_len) {
      side_len_max = side0_len;
      side_max = 0;
    }
    else {
      side_len_max = side1_len;
      side_max = 0;
    }

    if (side_len_max > max_len) {
      max_len = side_len_max;
      start_uv = slide_edge_loops[i].orig_uv;
      end_uv = slide_edge_loops[i].slide_uv_target[side_max];
    }
  }

  UI_view2d_view_to_region(
      &t->region->v2d, start_uv[0], start_uv[1], &start_region[0], &start_region[1]);
  UI_view2d_view_to_region(&t->region->v2d, end_uv[0], end_uv[1], &end_region[0], &end_region[1]);

  setCustomPoints(t, &t->mouse, start_region, end_region);
  applyMouseInput(t, &t->mouse, t->mval, t->values);
}

static eRedrawFlag handleEventEdgeSlideUV(TransInfo *t, const wmEvent *event)
{
  if (event->type == MOUSEMOVE) {
    calcEdgeSlideCustomPointsUV(t);
    return TREDRAW_NOTHING;
  }

  return TREDRAW_NOTHING;
}

static void initEdgeUVSlide(TransInfo *t, wmOperator * /*op*/)
{
  blender::Vector<EdgeSlideUV> *esd;
  t->mode = TFM_UV_EDGE_SLIDE;
  bool ok = false;

  FOREACH_TRANS_DATA_CONTAINER (t, tc) {
    esd = create_EdgeSlideUVs(t, tc);
    if (esd) {
      tc->custom.mode.data = esd;
      tc->custom.mode.free_cb = freeEdgeSlideUVs;
      ok = true;
    }
  }

  if (!ok) {
    t->state = TRANS_CANCEL;
    return;
  }

  t->idx_max = 1;
  t->num.flag = 0;
  t->num.idx_max = t->idx_max;

  calcEdgeSlideCustomPointsUV(t);
  initMouseInputMode(t, &t->mouse, INPUT_CUSTOM_RATIO_FLIP);
}

/** \} */

TransModeInfo TransMode_edgeslideuv = {
    /*flags*/ T_NO_CONSTRAINT,
    /*init_fn*/ initEdgeUVSlide,
    /*transform_fn*/ applyEdgeUVSlide,
    /*transform_matrix_fn*/ nullptr,
    /*handle_event_fn*/ handleEventEdgeSlideUV,
    /*snap_distance_fn*/ nullptr,
    /*snap_apply_fn*/ nullptr,
    /*draw_fn*/ nullptr,
};
