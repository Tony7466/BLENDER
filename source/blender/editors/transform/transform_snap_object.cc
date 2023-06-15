/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edtransform
 */

#include "BLI_math.h"
#include "BLI_math_matrix_types.hh"

#include "DNA_armature_types.h"
#include "DNA_curve_types.h"
#include "DNA_scene_types.h"
#include "DNA_screen_types.h"

#include "BKE_armature.h"
#include "BKE_bvhutils.h"
#include "BKE_curve.h"
#include "BKE_duplilist.h"
#include "BKE_editmesh.h"
#include "BKE_geometry_set.hh"
#include "BKE_geometry_set_instances.hh"
#include "BKE_global.h"
#include "BKE_layer.h"
#include "BKE_mesh.hh"
#include "BKE_object.h"
#include "BKE_tracking.h"

#include "DEG_depsgraph_query.h"

#include "ED_transform_snap_object_context.h"
#include "ED_view3d.h"

#include "transform_snap_object.hh"

float Nearest2dUserData::snap_dist_px_sq(const float co[3])
{
  DistProjectedAABBPrecalc *neasrest_precalc = &this->m_nearest_precalc;
  if (!isect_point_planes_v3_negated(this->m_clip_plane, this->m_clip_plane_len, co)) {
    return FLT_MAX;
  }

  float(*pmat)[4] = neasrest_precalc->pmat;

  float co2d[2] = {
      (dot_m4_v3_row_x(pmat, co) + pmat[3][0]),
      (dot_m4_v3_row_y(pmat, co) + pmat[3][1]),
  };

  if (this->m_is_persp) {
    float w = mul_project_m4_v3_zfac(pmat, co);
    mul_v2_fl(co2d, 1.0f / w);
  }

  return len_squared_v2v2(neasrest_precalc->mval, co2d);
}

Nearest2dUserData::Nearest2dUserData(SnapObjectContext *sctx,
                                     Object *ob_eval,
                                     const ID *data_eval,
                                     const float obmat[4][4],
                                     bool skip_occlusion_plane)
    : m_snap_to_flag(sctx->runtime.snap_to_flag),
      m_ob(ob_eval),
      m_data(data_eval),
      m_use_backface_culling(sctx->runtime.params.use_backface_culling),
      get_vert_co(nullptr),
      get_edge_verts_index(nullptr),
      get_tri_verts_index(nullptr),
      get_tri_edges_index(nullptr),
      copy_vert_no(nullptr),
      bm(nullptr),
      m_nearest_edge(),
      m_nearest_point(),
      m_point_type(SCE_SNAP_MODE_NONE),
      m_dist_px_sq_to_edge_center(sctx->dist_px_sq_to_edge_center),
      m_is_persp(sctx->runtime.rv3d ? sctx->runtime.rv3d->is_persp : false)
{
  if (obmat) {
    copy_m4_m4(this->m_obmat, obmat);
  }
  else {
    unit_m4(this->m_obmat);
  }

  if (sctx->runtime.rv3d) {
    mul_m4_m4m4(this->m_pmat_local, sctx->runtime.rv3d->persmat, this->m_obmat);
  }
  else {
    copy_m4_m4(this->m_obmat, this->m_obmat);
  }

  dist_squared_to_projected_aabb_precalc(
      &this->m_nearest_precalc, this->m_pmat_local, sctx->runtime.win_size, sctx->runtime.mval);

  /* Clip Planes Local. */

  float(*clip_planes)[4] = sctx->runtime.clip_plane;
  int clip_plane_len = sctx->runtime.clip_plane_len;

  if (skip_occlusion_plane && sctx->runtime.has_occlusion_plane) {
    /* We snap to vertices even if occluded. */
    clip_planes++;
    clip_plane_len--;
  }

  this->m_clip_plane_len = clip_plane_len;
  float tobmat[4][4];
  transpose_m4_m4(tobmat, this->m_obmat);
  for (int i = clip_plane_len; i--;) {
    mul_v4_m4v4(this->m_clip_plane[i], tobmat, clip_planes[i]);
  }

  copy_v3_fl3(this->m_normal_fallback, 0.0f, 0.0f, 1.0f);
  this->copy_vert_no = [](const int, const Nearest2dUserData *data, float no[3]) {
    copy_v3_v3(no, data->m_normal_fallback);
  };

  copy_v3_v3(this->m_prev_co, sctx->runtime.curr_co);

  this->m_nearest_point.dist_sq = sctx->point.nearest.dist_sq;
  this->m_nearest_edge.dist_sq = sctx->edge.nearest.dist_sq;

  this->m_nearest_point.index = -2;
  this->m_nearest_edge.index = -2;
}

bool Nearest2dUserData::confirm(SnapObjectContext *sctx)
{
  bool ret = false;
  if (this->m_nearest_point.index != -2) {
    sctx->point.nearest = this->m_nearest_point;
    sctx->point.ob = this->m_ob;
    sctx->point.data = this->m_data;
    copy_m4_m4(sctx->point.obmat, this->m_obmat);
    sctx->point.elem = this->m_point_type;
    ret = true;
  }

  if (this->m_nearest_edge.index != -2) {
    sctx->edge.nearest = this->m_nearest_edge;
    sctx->edge.ob = this->m_ob;
    sctx->edge.data = this->m_data;
    copy_m4_m4(sctx->edge.obmat, this->m_obmat);
    sctx->edge.elem = SCE_SNAP_MODE_EDGE;
    sctx->dist_px_sq_to_edge_center = this->m_dist_px_sq_to_edge_center;
    ret = true;
  }

  return ret;
}

float Nearest2dUserData::dist_px_sq(void)
{
  return this->m_snap_to_flag & SCE_SNAP_MODE_POINTS ? this->m_nearest_point.dist_sq :
                                                       this->m_nearest_edge.dist_sq;
}

/* Test BoundBox */
bool Nearest2dUserData::snap_boundbox(float min[3], float max[3])
{
  /* In vertex and edges you need to get the pixel distance from ray to BoundBox,
   * see: #46099, #46816 */

  int isect_type = isect_aabb_planes_v3(this->m_clip_plane, this->m_clip_plane_len, min, max);
  if (isect_type == ISECT_AABB_PLANE_BEHIND_ANY) {
    return false;
  }

  bool dummy[3];
  float bb_dist_px_sq = dist_squared_to_projected_aabb(&this->m_nearest_precalc, min, max, dummy);
  if (bb_dist_px_sq > this->dist_px_sq()) {
    return false;
  }

  return true;
}

bool Nearest2dUserData::snap_point(int index, const float co[3])
{
  BVHTreeNearest *point = &this->m_nearest_point;
  const float dist_sq = this->snap_dist_px_sq(co);
  if (dist_sq < point->dist_sq) {
    point->index = index;
    copy_v3_v3(point->co, co);
    this->copy_vert_no(index, this, point->no);
    point->dist_sq = dist_sq;
    this->m_point_type = SCE_SNAP_MODE_VERTEX;
    return true;
  }
  return false;
}

bool Nearest2dUserData::snap_edge(
    int edge_index, int v0_index, int v1_index, const float v0_co[3], const float v1_co[3])
{
  bool ret = false;

  DistProjectedAABBPrecalc *nearest_precalc = &this->m_nearest_precalc;

  float lambda;
  if (!isect_ray_line_v3(
          nearest_precalc->ray_origin, nearest_precalc->ray_direction, v0_co, v1_co, &lambda))
  {
    return false;
  }
  else {
    sub_v3_v3v3(this->m_normal_fallback, v1_co, v0_co);

    float vmid[3];
    float dist_sq_to_edge_center = 0.0f;
    if (this->m_snap_to_flag & (SCE_SNAP_MODE_EDGE | SCE_SNAP_MODE_EDGE_MIDPOINT)) {
      mid_v3_v3v3(vmid, v0_co, v1_co);
      dist_sq_to_edge_center = this->snap_dist_px_sq(vmid);
    }

    if (this->m_snap_to_flag & SCE_SNAP_MODE_EDGE) {
      if (IN_RANGE(lambda, 0.0f, 1.0f)) {
        BVHTreeNearest *edge = &this->m_nearest_edge;
        float near_co[3];
        interp_v3_v3v3(near_co, v0_co, v1_co, lambda);
        const float dist_sq = this->snap_dist_px_sq(near_co);
        if (dist_sq < edge->dist_sq) {
          edge->index = edge_index;
          copy_v3_v3(edge->co, near_co);
          copy_v3_v3(edge->no, this->m_normal_fallback);
          edge->dist_sq = dist_sq;
          this->m_dist_px_sq_to_edge_center = dist_sq_to_edge_center;
          ret = true;
        }
        else {
          return false;
        }
      }
    }

    if (this->m_snap_to_flag & SCE_SNAP_MODE_EDGE_MIDPOINT) {
      BVHTreeNearest *point = &this->m_nearest_point;
      if (dist_sq_to_edge_center < point->dist_sq &&
          isect_point_planes_v3_negated(this->m_clip_plane, this->m_clip_plane_len, vmid))
      {
        point->index = edge_index;
        copy_v3_v3(point->co, vmid);
        copy_v3_v3(point->no, this->m_normal_fallback);
        point->dist_sq = dist_sq_to_edge_center;
        this->m_point_type = SCE_SNAP_MODE_EDGE_MIDPOINT;
        ret = true;
      }
    }

    if (this->m_snap_to_flag & SCE_SNAP_MODE_VERTEX) {
      int vert_index = lambda < 0.5f ? v0_index : v1_index;
      const float *co = lambda < 0.5f ? v0_co : v1_co;
      if (this->snap_point(vert_index, co)) {
        ret = true;
      }
    }

    if (this->m_snap_to_flag & SCE_SNAP_MODE_EDGE_PERPENDICULAR) {
      float v_near[3], va_g[3], vb_g[3];

      mul_v3_m4v3(va_g, this->m_obmat, v0_co);
      mul_v3_m4v3(vb_g, this->m_obmat, v1_co);
      lambda = line_point_factor_v3(this->m_prev_co, va_g, vb_g);

      if (IN_RANGE(lambda, 0.0f, 1.0f)) {
        BVHTreeNearest *point = &this->m_nearest_point;
        interp_v3_v3v3(v_near, v0_co, v1_co, lambda);
        const float dist_sq = this->snap_dist_px_sq(v_near);
        if (dist_sq < point->dist_sq) {
          point->index = edge_index;
          copy_v3_v3(point->co, v_near);
          copy_v3_v3(point->no, this->m_normal_fallback);
          point->dist_sq = dist_sq;
          this->m_point_type = SCE_SNAP_MODE_EDGE_PERPENDICULAR;
          ret = true;
        }
      }
    }
  }

  return ret;
}

/* -------------------------------------------------------------------- */
/** \name Utilities
 * \{ */

/**
 * Mesh used for snapping.
 *
 * - When the return value is null the `BKE_editmesh_from_object(ob_eval)` should be used.
 * - In rare cases there is no evaluated mesh available and a null result doesn't imply an
 *   edit-mesh, so callers need to account for a null edit-mesh too, see: #96536.
 */
static ID *data_for_snap(Object *ob_eval, eSnapEditType edit_mode_type, bool *r_use_hide)
{
  bool use_hide = false;

  switch (ob_eval->type) {
    case OB_MESH: {
      Mesh *me_eval = BKE_object_get_evaluated_mesh(ob_eval);
      if (BKE_object_is_in_editmode(ob_eval)) {
        if (edit_mode_type == SNAP_GEOM_EDIT) {
          return nullptr;
        }

        Mesh *editmesh_eval_final = BKE_object_get_editmesh_eval_final(ob_eval);
        Mesh *editmesh_eval_cage = BKE_object_get_editmesh_eval_cage(ob_eval);

        if ((edit_mode_type == SNAP_GEOM_FINAL) && editmesh_eval_final) {
          if (editmesh_eval_final->runtime->wrapper_type == ME_WRAPPER_TYPE_BMESH) {
            return nullptr;
          }
          me_eval = editmesh_eval_final;
          use_hide = true;
        }
        else if ((edit_mode_type == SNAP_GEOM_CAGE) && editmesh_eval_cage) {
          if (editmesh_eval_cage->runtime->wrapper_type == ME_WRAPPER_TYPE_BMESH) {
            return nullptr;
          }
          me_eval = editmesh_eval_cage;
          use_hide = true;
        }
      }
      if (r_use_hide) {
        *r_use_hide = use_hide;
      }
      return (ID *)me_eval;
    }
    default:
      break;
  }
  if (r_use_hide) {
    *r_use_hide = use_hide;
  }
  return (ID *)ob_eval->data;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Iterator
 * \{ */

using IterSnapObjsCallback = void (*)(SnapObjectContext *sctx,
                                      Object *ob_eval,
                                      ID *ob_data,
                                      const float obmat[4][4],
                                      bool is_object_active,
                                      bool use_hide);

static bool snap_object_is_snappable(const SnapObjectContext *sctx,
                                     const eSnapTargetOP snap_target_select,
                                     const Base *base_act,
                                     const Base *base)
{
  if (!BASE_VISIBLE(sctx->runtime.v3d, base)) {
    return false;
  }

  if ((snap_target_select == SCE_SNAP_TARGET_ALL) ||
      (base->flag_legacy & BA_TRANSFORM_LOCKED_IN_PLACE))
  {
    return true;
  }

  if (base->flag_legacy & BA_SNAP_FIX_DEPS_FIASCO) {
    return false;
  }

  /* Get attributes of potential target. */
  const bool is_active = (base_act == base);
  const bool is_selected = (base->flag & BASE_SELECTED) || (base->flag_legacy & BA_WAS_SEL);
  const bool is_edited = (base->object->mode == OB_MODE_EDIT);
  const bool is_selectable = (base->flag & BASE_SELECTABLE);
  /* Get attributes of state. */
  const bool is_in_object_mode = (base_act == nullptr) ||
                                 (base_act->object->mode == OB_MODE_OBJECT);

  if (is_in_object_mode) {
    /* Handle target selection options that make sense for object mode. */
    if ((snap_target_select & SCE_SNAP_TARGET_NOT_SELECTED) && is_selected) {
      /* What is selectable or not is part of the object and depends on the mode. */
      return false;
    }
  }
  else {
    /* Handle target selection options that make sense for edit/pose mode. */
    if ((snap_target_select & SCE_SNAP_TARGET_NOT_ACTIVE) && is_active) {
      return false;
    }
    if ((snap_target_select & SCE_SNAP_TARGET_NOT_EDITED) && is_edited && !is_active) {
      /* Base is edited, but not active. */
      return false;
    }
    if ((snap_target_select & SCE_SNAP_TARGET_NOT_NONEDITED) && !is_edited) {
      return false;
    }
  }

  if ((snap_target_select & SCE_SNAP_TARGET_ONLY_SELECTABLE) && !is_selectable) {
    return false;
  }

  return true;
}

/**
 * Walks through all objects in the scene to create the list of objects to snap.
 */
static void iter_snap_objects(SnapObjectContext *sctx, IterSnapObjsCallback sob_callback)
{
  Scene *scene = DEG_get_input_scene(sctx->runtime.depsgraph);
  ViewLayer *view_layer = DEG_get_input_view_layer(sctx->runtime.depsgraph);
  const eSnapTargetOP snap_target_select = sctx->runtime.params.snap_target_select;
  BKE_view_layer_synced_ensure(scene, view_layer);
  Base *base_act = BKE_view_layer_active_base_get(view_layer);

  LISTBASE_FOREACH (Base *, base, BKE_view_layer_object_bases_get(view_layer)) {
    if (!snap_object_is_snappable(sctx, snap_target_select, base_act, base)) {
      continue;
    }

    const bool is_object_active = (base == base_act);
    Object *obj_eval = DEG_get_evaluated_object(sctx->runtime.depsgraph, base->object);
    if (obj_eval->transflag & OB_DUPLI ||
        blender::bke::object_has_geometry_set_instances(*obj_eval)) {
      ListBase *lb = object_duplilist(sctx->runtime.depsgraph, sctx->scene, obj_eval);
      LISTBASE_FOREACH (DupliObject *, dupli_ob, lb) {
        BLI_assert(DEG_is_evaluated_object(dupli_ob->ob));
        sob_callback(
            sctx, dupli_ob->ob, dupli_ob->ob_data, dupli_ob->mat, is_object_active, false);
      }
      free_object_duplilist(lb);
    }

    bool use_hide = false;
    ID *ob_data = data_for_snap(obj_eval, sctx->runtime.params.edit_mode_type, &use_hide);
    sob_callback(sctx, obj_eval, ob_data, obj_eval->object_to_world, is_object_active, use_hide);
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Ray Cast Functions
 * \{ */

/* Store all ray-hits
 * Support for storing all depths, not just the first (ray-cast 'all'). */

static SnapObjectHitDepth *hit_depth_create(const float depth, const float co[3], uint ob_uuid)
{
  SnapObjectHitDepth *hit = MEM_new<SnapObjectHitDepth>(__func__);

  hit->depth = depth;
  copy_v3_v3(hit->co, co);
  hit->ob_uuid = ob_uuid;

  return hit;
}

static int hit_depth_cmp(const void *arg1, const void *arg2)
{
  const SnapObjectHitDepth *h1 = static_cast<const SnapObjectHitDepth *>(arg1);
  const SnapObjectHitDepth *h2 = static_cast<const SnapObjectHitDepth *>(arg2);
  int val = 0;

  if (h1->depth < h2->depth) {
    val = -1;
  }
  else if (h1->depth > h2->depth) {
    val = 1;
  }

  return val;
}

void raycast_all_cb(void *userdata, int index, const BVHTreeRay *ray, BVHTreeRayHit *hit)
{
  RayCastAll_Data *data = static_cast<RayCastAll_Data *>(userdata);
  data->raycast_callback(data->bvhdata, index, ray, hit);
  if (hit->index != -1) {
    /* Get all values in world-space. */
    float location[3];
    float depth;

    /* World-space location. */
    mul_v3_m4v3(location, (float(*)[4])data->obmat, hit->co);
    depth = (hit->dist + data->len_diff) / data->local_scale;

    SnapObjectHitDepth *hit_item = hit_depth_create(depth, location, data->ob_uuid);
    BLI_addtail(data->hit_list, hit_item);
  }
}

bool raycast_tri_backface_culling_test(
    const float dir[3], const float v0[3], const float v1[3], const float v2[3], float no[3])
{
  cross_tri_v3(no, v0, v1, v2);
  return dot_v3v3(no, dir) < 0.0f;
}

/**
 * \note Duplicate args here are documented at #snapObjectsRay
 */
static void raycast_obj_fn(SnapObjectContext *sctx,
                           Object *ob_eval,
                           ID *ob_data,
                           const float obmat[4][4],
                           bool is_object_active,
                           bool use_hide)
{
  if (ob_data == nullptr) {
    if (sctx->runtime.use_occlusion_test_edit && ELEM(ob_eval->dt, OB_BOUNDBOX, OB_WIRE)) {
      /* Do not hit objects that are in wire or bounding box display mode. */
      return;
    }
    if (ob_eval->type == OB_MESH) {
      snap_object_editmesh(sctx, ob_eval, nullptr, obmat, use_hide);
    }
    else {
      return;
    }
  }
  else if (sctx->runtime.params.use_occlusion_test && ELEM(ob_eval->dt, OB_BOUNDBOX, OB_WIRE)) {
    /* Do not hit objects that are in wire or bounding box display mode. */
    return;
  }
  else if (GS(ob_data->name) != ID_ME) {
    return;
  }
  else if (is_object_active && ELEM(ob_eval->type, OB_CURVES_LEGACY, OB_SURF, OB_FONT)) {
    return;
  }
  else {
    snap_object_mesh(sctx, ob_eval, ob_data, obmat, use_hide);
  }
}

/**
 * Main RayCast Function
 * ======================
 *
 * Walks through all objects in the scene to find the `hit` on object surface.
 *
 * \param sctx: Snap context to store data.
 *
 * Read/Write Args
 * ---------------
 *
 * \param ray_depth: maximum depth allowed for r_co,
 * elements deeper than this value will be ignored.
 */
static bool raycastObjects(SnapObjectContext *sctx)
{
  sctx->runtime.snap_to_flag = SCE_SNAP_MODE_FACE;
  Object *ob_prev = sctx->poly.ob;
  sctx->poly.ob = nullptr;
  iter_snap_objects(sctx, raycast_obj_fn);
  if (sctx->poly.ob) {
    return true;
  }
  sctx->poly.ob = ob_prev;
  return false;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Surface Snap Functions
 * \{ */

static void nearest_world_tree_co(BVHTree *tree,
                                  BVHTree_NearestPointCallback nearest_cb,
                                  void *treedata,
                                  float co[3],
                                  float r_co[3],
                                  float r_no[3],
                                  int *r_index,
                                  float *r_dist_sq)
{
  BVHTreeNearest nearest = {};
  nearest.index = -1;
  copy_v3_fl(nearest.co, FLT_MAX);
  nearest.dist_sq = FLT_MAX;

  BLI_bvhtree_find_nearest(tree, co, &nearest, nearest_cb, treedata);

  if (r_co) {
    copy_v3_v3(r_co, nearest.co);
  }
  if (r_no) {
    copy_v3_v3(r_no, nearest.no);
  }
  if (r_index) {
    *r_index = nearest.index;
  }
  if (r_dist_sq) {
    float diff[3];
    sub_v3_v3v3(diff, co, nearest.co);
    *r_dist_sq = len_squared_v3(diff);
  }
}

bool nearest_world_tree(SnapObjectContext *sctx,
                        BVHTree *tree,
                        BVHTree_NearestPointCallback nearest_cb,
                        void *treedata,
                        const float (*obmat)[4],
                        float *r_dist_sq,
                        float *r_loc,
                        float *r_no,
                        int *r_index)
{
  float imat[4][4];
  invert_m4_m4(imat, obmat);

  /* compute offset between init co and prev co in local space */
  float init_co_local[3], curr_co_local[3];
  float delta_local[3];
  mul_v3_m4v3(init_co_local, imat, sctx->runtime.init_co);
  mul_v3_m4v3(curr_co_local, imat, sctx->runtime.curr_co);
  sub_v3_v3v3(delta_local, curr_co_local, init_co_local);

  float dist_sq;
  if (sctx->runtime.params.keep_on_same_target) {
    nearest_world_tree_co(
        tree, nearest_cb, treedata, init_co_local, nullptr, nullptr, nullptr, &dist_sq);
  }
  else {
    /* NOTE: when `params->face_nearest_steps == 1`, the return variables of function below contain
     * the answer.  We could return immediately after updating r_loc, r_no, r_index, but that would
     * also complicate the code. Foregoing slight optimization for code clarity. */
    nearest_world_tree_co(
        tree, nearest_cb, treedata, curr_co_local, nullptr, nullptr, nullptr, &dist_sq);
  }
  if (*r_dist_sq <= dist_sq) {
    return false;
  }
  *r_dist_sq = dist_sq;

  /* scale to make `snap_face_nearest_steps` steps */
  float step_scale_factor = 1.0f / max_ff(1.0f, float(sctx->runtime.params.face_nearest_steps));
  mul_v3_fl(delta_local, step_scale_factor);

  float co_local[3];
  float no_local[3];

  copy_v3_v3(co_local, init_co_local);

  for (int i = 0; i < sctx->runtime.params.face_nearest_steps; i++) {
    add_v3_v3(co_local, delta_local);
    nearest_world_tree_co(
        tree, nearest_cb, treedata, co_local, co_local, no_local, r_index, nullptr);
  }

  copy_v3_v3(r_loc, co_local);
  copy_v3_v3(r_no, no_local);

  return true;
}

static void nearest_world_object_fn(SnapObjectContext *sctx,
                                    Object *ob_eval,
                                    ID *ob_data,
                                    const float obmat[4][4],
                                    bool is_object_active,
                                    bool use_hide)
{
  if (ob_data == nullptr) {
    if (ob_eval->type == OB_MESH) {
      snap_object_editmesh(sctx, ob_eval, nullptr, obmat, use_hide);
    }
    else {
      return;
    }
  }
  else if (GS(ob_data->name) != ID_ME) {
    return;
  }
  else if (is_object_active && ELEM(ob_eval->type, OB_CURVES_LEGACY, OB_SURF, OB_FONT)) {
    return;
  }
  else {
    snap_object_mesh(sctx, ob_eval, ob_data, obmat, use_hide);
  }
}

/**
 * Main Nearest World Surface Function
 * ===================================
 *
 * Walks through all objects in the scene to find the nearest location on target surface.
 *
 * \param sctx: Snap context to store data.
 * \param params: Settings for snapping.
 * \param init_co: Initial location of source point.
 * \param prev_co: Current location of source point after transformation but before snapping.
 */
static bool nearestWorldObjects(SnapObjectContext *sctx)
{
  sctx->runtime.snap_to_flag = SCE_SNAP_MODE_FACE_NEAREST;
  Object *ob_prev = sctx->poly.ob;
  sctx->poly.ob = nullptr;
  iter_snap_objects(sctx, nearest_world_object_fn);
  if (sctx->poly.ob) {
    return true;
  }
  sctx->poly.ob = ob_prev;
  return false;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Callbacks
 * \{ */

void cb_snap_vert(void *userdata,
                  int index,
                  const DistProjectedAABBPrecalc * /*precalc*/,
                  const float (*/*clip_plane*/)[4],
                  const int /*clip_plane_len*/,
                  BVHTreeNearest *nearest)
{
  Nearest2dUserData *nearest2d = static_cast<Nearest2dUserData *>(userdata);
  const float *co;
  nearest2d->get_vert_co(index, nearest2d, &co);

  if (nearest2d->snap_point(index, co)) {
    nearest->index = index;
    nearest->dist_sq = nearest2d->dist_px_sq();
  }
}

void cb_snap_edge(void *userdata,
                  int index,
                  const DistProjectedAABBPrecalc * /*precalc*/,
                  const float (*/*clip_plane*/)[4],
                  const int /*clip_plane_len*/,
                  BVHTreeNearest *nearest)
{
  Nearest2dUserData *nearest2d = static_cast<Nearest2dUserData *>(userdata);

  int vindex[2];
  nearest2d->get_edge_verts_index(index, nearest2d, vindex);

  const float *v_pair[2];
  nearest2d->get_vert_co(vindex[0], nearest2d, &v_pair[0]);
  nearest2d->get_vert_co(vindex[1], nearest2d, &v_pair[1]);

  if (nearest2d->snap_edge(index, vindex[0], vindex[1], v_pair[0], v_pair[1])) {
    nearest->index = index;
    nearest->dist_sq = nearest2d->dist_px_sq();
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Internal Object Snapping API
 * \{ */

static void snap_polygon(SnapObjectContext *sctx,
                         eSnapMode snap_to_flag,
                         Object *ob_eval,
                         const ID *id,
                         const float obmat[4][4],
                         int polygon)
{
  sctx->runtime.snap_to_flag = snap_to_flag;

  if (id) {
    if (GS(id->name) == ID_ME) {
      snap_polygon_mesh(sctx, ob_eval, id, obmat, polygon);
    }
  }
  else {
    snap_polygon_editmesh(sctx, ob_eval, id, obmat, polygon);
  }
}

static void snapArmature(SnapObjectContext *sctx,
                         Object *ob_eval,
                         const float obmat[4][4],
                         bool is_object_active)
{
  if (sctx->runtime.snap_to_flag == SCE_SNAP_MODE_FACE) {
    /* Currently only edge and vert. */
    return;
  }

  Nearest2dUserData nearest2d(sctx, ob_eval, static_cast<ID *>(ob_eval->data), obmat);

  bArmature *arm = static_cast<bArmature *>(ob_eval->data);
  const bool is_editmode = arm->edbo != nullptr;
  if (is_editmode == false) {
    BoundBox *bb = BKE_armature_boundbox_get(ob_eval);
    if (!nearest2d.snap_boundbox(bb->vec[0], bb->vec[6])) {
      return;
    }
  }

  const bool is_posemode = is_object_active && (ob_eval->mode & OB_MODE_POSE);
  const bool skip_selected = (is_editmode || is_posemode) &&
                             (sctx->runtime.params.snap_target_select &
                              SCE_SNAP_TARGET_NOT_SELECTED);

  if (arm->edbo) {
    LISTBASE_FOREACH (EditBone *, eBone, arm->edbo) {
      if (eBone->layer & arm->layer) {
        if (eBone->flag & BONE_HIDDEN_A) {
          /* Skip hidden bones. */
          continue;
        }

        const bool is_selected = (eBone->flag & (BONE_ROOTSEL | BONE_TIPSEL)) != 0;
        if (is_selected && skip_selected) {
          continue;
        }

        nearest2d.snap_edge(-1, -1, -1, eBone->head, eBone->tail);
      }
    }
  }
  else if (ob_eval->pose && ob_eval->pose->chanbase.first) {
    LISTBASE_FOREACH (bPoseChannel *, pchan, &ob_eval->pose->chanbase) {
      Bone *bone = pchan->bone;
      if (!bone || (bone->flag & (BONE_HIDDEN_P | BONE_HIDDEN_PG))) {
        /* Skip hidden bones. */
        continue;
      }

      const bool is_selected = (bone->flag & (BONE_SELECTED | BONE_ROOTSEL | BONE_TIPSEL)) != 0;
      if (is_selected && skip_selected) {
        continue;
      }

      const float *head_vec = pchan->pose_head;
      const float *tail_vec = pchan->pose_tail;

      nearest2d.snap_edge(-1, -1, -1, head_vec, tail_vec);
    }
  }

  nearest2d.confirm(sctx);
}

static void snapCurve(SnapObjectContext *sctx, Object *ob_eval, const float obmat[4][4])
{
  /* Only vertex snapping mode (eg control points and handles) supported for now). */
  if ((sctx->runtime.snap_to_flag & SCE_SNAP_MODE_VERTEX) == 0) {
    return;
  }

  Nearest2dUserData nearest2d(sctx, ob_eval, static_cast<ID *>(ob_eval->data), obmat, true);

  const bool use_obedit = BKE_object_is_in_editmode(ob_eval);
  if (use_obedit == false) {
    BoundBox *bb = BKE_curve_boundbox_get(ob_eval);
    if (!nearest2d.snap_boundbox(bb->vec[0], bb->vec[6])) {
      return;
    }
  }

  Curve *cu = static_cast<Curve *>(ob_eval->data);

  bool skip_selected = (sctx->runtime.params.snap_target_select & SCE_SNAP_TARGET_NOT_SELECTED) !=
                       0;

  bool ret = false;

  LISTBASE_FOREACH (Nurb *, nu, (use_obedit ? &cu->editnurb->nurbs : &cu->nurb)) {
    for (int u = 0; u < nu->pntsu; u++) {
      if (sctx->runtime.snap_to_flag & SCE_SNAP_MODE_VERTEX) {
        if (use_obedit) {
          if (nu->bezt) {
            if (nu->bezt[u].hide) {
              /* Skip hidden. */
              continue;
            }

            bool is_selected = (nu->bezt[u].f2 & SELECT) != 0;
            if (is_selected && skip_selected) {
              continue;
            }

            if (nearest2d.snap_point(-1, nu->bezt[u].vec[1])) {
              ret = true;
            }

            /* Don't snap if handle is selected (moving),
             * or if it is aligning to a moving handle. */
            bool is_selected_h1 = (nu->bezt[u].f1 & SELECT) != 0;
            bool is_selected_h2 = (nu->bezt[u].f3 & SELECT) != 0;
            bool is_autoalign_h1 = (nu->bezt[u].h1 & HD_ALIGN) != 0;
            bool is_autoalign_h2 = (nu->bezt[u].h2 & HD_ALIGN) != 0;
            if (!skip_selected || !(is_selected_h1 || (is_autoalign_h1 && is_selected_h2))) {
              if (nearest2d.snap_point(-1, nu->bezt[u].vec[0])) {
                ret = true;
              }
            }

            if (!skip_selected || !(is_selected_h2 || (is_autoalign_h2 && is_selected_h1))) {
              if (nearest2d.snap_point(-1, nu->bezt[u].vec[2])) {
                ret = true;
              }
            }
          }
          else {
            if (nu->bp[u].hide) {
              /* Skip hidden. */
              continue;
            }

            bool is_selected = (nu->bp[u].f1 & SELECT) != 0;
            if (is_selected && skip_selected) {
              continue;
            }

            if (nearest2d.snap_point(-1, nu->bp[u].vec)) {
              ret = true;
            }
          }
        }
        else {
          /* Curve is not visible outside editmode if nurb length less than two. */
          if (nu->pntsu > 1) {
            if (nu->bezt) {
              if (nearest2d.snap_point(-1, nu->bezt[u].vec[1])) {
                ret = true;
              }
            }
            else {
              if (nearest2d.snap_point(-1, nu->bp[u].vec)) {
                ret = true;
              }
            }
          }
        }
      }
    }
  }

  nearest2d.confirm(sctx);
}

/* may extend later (for now just snaps to empty center) */
static void snap_object_center(SnapObjectContext *sctx, Object *ob_eval, const float obmat[4][4])
{
  if (ob_eval->transflag & OB_DUPLI) {
    return;
  }

  /* For now only vertex supported. */
  if ((sctx->runtime.snap_to_flag & SCE_SNAP_MODE_VERTEX) == 0) {
    return;
  }

  float vec_zero[3] = {0.0f};

  bool ret = false;
  Nearest2dUserData nearest2d(sctx, ob_eval, nullptr, obmat);
  if (nearest2d.snap_point(-1, vec_zero)) {
    ret = true;
  }

  nearest2d.confirm(sctx);
}

static void snapCamera(SnapObjectContext *sctx, Object *ob_eval, const float obmat[4][4])
{
  Scene *scene = sctx->scene;

  float orig_camera_mat[4][4], orig_camera_imat[4][4];
  MovieClip *clip = BKE_object_movieclip_get(scene, ob_eval, false);
  MovieTracking *tracking;

  if (clip == nullptr) {
    snap_object_center(sctx, ob_eval, obmat);
    return;
  }
  if (ob_eval->transflag & OB_DUPLI) {
    return;
  }

  Nearest2dUserData nearest2d(sctx, ob_eval, static_cast<ID *>(ob_eval->data));

  tracking = &clip->tracking;

  BKE_tracking_get_camera_object_matrix(ob_eval, orig_camera_mat);

  invert_m4_m4(orig_camera_imat, orig_camera_mat);

  if (sctx->runtime.snap_to_flag & SCE_SNAP_MODE_VERTEX) {
    LISTBASE_FOREACH (MovieTrackingObject *, tracking_object, &tracking->objects) {
      float reconstructed_camera_mat[4][4], reconstructed_camera_imat[4][4];
      const float(*vertex_obmat)[4];

      if ((tracking_object->flag & TRACKING_OBJECT_CAMERA) == 0) {
        BKE_tracking_camera_get_reconstructed_interpolate(
            tracking, tracking_object, scene->r.cfra, reconstructed_camera_mat);

        invert_m4_m4(reconstructed_camera_imat, reconstructed_camera_mat);
      }

      LISTBASE_FOREACH (MovieTrackingTrack *, track, &tracking_object->tracks) {
        float bundle_pos[3];

        if ((track->flag & TRACK_HAS_BUNDLE) == 0) {
          continue;
        }

        copy_v3_v3(bundle_pos, track->bundle_pos);
        if (tracking_object->flag & TRACKING_OBJECT_CAMERA) {
          vertex_obmat = orig_camera_mat;
        }
        else {
          mul_m4_v3(reconstructed_camera_imat, bundle_pos);
          vertex_obmat = obmat;
        }

        mul_m4_v3(vertex_obmat, bundle_pos);
        if (nearest2d.snap_point(-1, bundle_pos)) {
        }
      }
    }
  }

  if (nearest2d.confirm(sctx)) {
    float imat[4][4];
    invert_m4_m4(imat, obmat);
    mul_m4_v3(imat, sctx->point.nearest.co);
  }
}

/**
 * \note Duplicate args here are documented at #snapObjectsRay
 */
static void snap_obj_fn(SnapObjectContext *sctx,
                        Object *ob_eval,
                        ID *ob_data,
                        const float obmat[4][4],
                        bool is_object_active,
                        bool use_hide)
{
  if (ob_data == nullptr && (ob_eval->type == OB_MESH)) {
    snap_object_editmesh(sctx, ob_eval, nullptr, obmat, use_hide);
  }
  else if (ob_data == nullptr) {
    snap_object_center(sctx, ob_eval, obmat);
  }
  else {
    switch (ob_eval->type) {
      case OB_MESH: {
        if (ob_eval->dt == OB_BOUNDBOX) {
          /* Do not snap to objects that are in bounding box display mode */
          return;
        }
        if (GS(ob_data->name) == ID_ME) {
          snap_object_mesh(sctx, ob_eval, ob_data, obmat, use_hide);
        }
        break;
      }
      case OB_ARMATURE:
        snapArmature(sctx, ob_eval, obmat, is_object_active);
        break;
      case OB_CURVES_LEGACY:
      case OB_SURF:
        if (ob_eval->type == OB_CURVES_LEGACY || BKE_object_is_in_editmode(ob_eval)) {
          snapCurve(sctx, ob_eval, obmat);
          if (sctx->runtime.params.edit_mode_type != SNAP_GEOM_FINAL) {
            break;
          }
        }
        ATTR_FALLTHROUGH;
      case OB_FONT: {
        const Mesh *mesh_eval = BKE_object_get_evaluated_mesh(ob_eval);
        if (mesh_eval) {
          snap_object_mesh(sctx, ob_eval, (ID *)mesh_eval, obmat, use_hide);
        }
        break;
      }
      case OB_EMPTY:
      case OB_GPENCIL_LEGACY:
      case OB_LAMP:
        snap_object_center(sctx, ob_eval, obmat);
        break;
      case OB_CAMERA:
        snapCamera(sctx, ob_eval, obmat);
        break;
    }
  }
}

/**
 * Main Snapping Function
 * ======================
 *
 * Walks through all objects in the scene to find the closest snap element ray.
 *
 * \param sctx: Snap context to store data.
 *
 * Read/Write Args
 * ---------------
 *
 * \param dist_px: Maximum threshold distance (in pixels).
 */
static void snapObjectsRay(SnapObjectContext *sctx, eSnapMode snap_to_flag)
{
  sctx->runtime.snap_to_flag = snap_to_flag;
  iter_snap_objects(sctx, snap_obj_fn);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Public Object Snapping API
 * \{ */

SnapObjectContext *ED_transform_snap_object_context_create(Scene *scene, int /*flag*/)
{
  SnapObjectContext *sctx = MEM_new<SnapObjectContext>(__func__);

  sctx->scene = scene;

  return sctx;
}

void ED_transform_snap_object_context_destroy(SnapObjectContext *sctx)
{
  MEM_delete(sctx);
}

void ED_transform_snap_object_context_set_editmesh_callbacks(
    SnapObjectContext *sctx,
    bool (*test_vert_fn)(BMVert *, void *user_data),
    bool (*test_edge_fn)(BMEdge *, void *user_data),
    bool (*test_face_fn)(BMFace *, void *user_data),
    void *user_data)
{
  bool is_cache_dirty = false;
  if (sctx->callbacks.edit_mesh.test_vert_fn != test_vert_fn) {
    sctx->callbacks.edit_mesh.test_vert_fn = test_vert_fn;
    is_cache_dirty = true;
  }
  if (sctx->callbacks.edit_mesh.test_edge_fn != test_edge_fn) {
    sctx->callbacks.edit_mesh.test_edge_fn = test_edge_fn;
    is_cache_dirty = true;
  }
  if (sctx->callbacks.edit_mesh.test_face_fn != test_face_fn) {
    sctx->callbacks.edit_mesh.test_face_fn = test_face_fn;
    is_cache_dirty = true;
  }
  if (sctx->callbacks.edit_mesh.user_data != user_data) {
    sctx->callbacks.edit_mesh.user_data = user_data;
    is_cache_dirty = true;
  }

  if (is_cache_dirty) {
    sctx->editmesh_caches.clear();
  }
}

static bool snap_object_context_runtime_init(SnapObjectContext *sctx,
                                             Depsgraph *depsgraph,
                                             const ARegion *region,
                                             const View3D *v3d,
                                             eSnapMode snap_to_flag,
                                             const SnapObjectParams *params,
                                             const float ray_start[3],
                                             const float ray_dir[3],
                                             const float ray_depth,
                                             const float mval[2],
                                             const float init_co[3],
                                             const float prev_co[3],
                                             const float dist_px_sq,
                                             ListBase *hit_list,
                                             bool use_occlusion_test)
{
  if (snap_to_flag & (SCE_SNAP_MODE_EDGE_PERPENDICULAR | SCE_SNAP_MODE_FACE_NEAREST)) {
    if (prev_co) {
      copy_v3_v3(sctx->runtime.curr_co, prev_co);
      if (init_co) {
        copy_v3_v3(sctx->runtime.init_co, init_co);
      }
      else {
        snap_to_flag &= ~SCE_SNAP_MODE_FACE_NEAREST;
      }
    }
    else {
      snap_to_flag &= ~(SCE_SNAP_MODE_EDGE_PERPENDICULAR | SCE_SNAP_MODE_FACE_NEAREST);
    }
  }

  if (snap_to_flag == SCE_SNAP_MODE_NONE) {
    return false;
  }

  sctx->runtime.depsgraph = depsgraph;
  sctx->runtime.v3d = v3d;
  sctx->runtime.snap_to_flag = snap_to_flag;
  sctx->runtime.params = *params;
  sctx->runtime.params.use_occlusion_test = use_occlusion_test;
  sctx->runtime.use_occlusion_test_edit = use_occlusion_test &&
                                          (snap_to_flag & SCE_SNAP_MODE_FACE) == 0;
  sctx->runtime.has_occlusion_plane = false;
  sctx->runtime.object_index = 0;

  copy_v3_v3(sctx->runtime.ray_start, ray_start);
  copy_v3_v3(sctx->runtime.ray_dir, ray_dir);

  if (mval) {
    copy_v2_v2(sctx->runtime.mval, mval);
  }

  if (region) {
    const RegionView3D *rv3d = static_cast<RegionView3D *>(region->regiondata);
    sctx->runtime.rv3d = rv3d;
    sctx->runtime.win_size[0] = region->winx;
    sctx->runtime.win_size[1] = region->winy;

    planes_from_projmat(rv3d->persmat,
                        nullptr,
                        nullptr,
                        nullptr,
                        nullptr,
                        sctx->runtime.clip_plane[0],
                        sctx->runtime.clip_plane[1]);

    sctx->runtime.clip_plane_len = 2;
  }
  else {
    sctx->runtime.rv3d = nullptr;
    sctx->runtime.clip_plane_len = 0;
  }

  sctx->poly.nearest.dist_sq = ray_depth;
  zero_v3(sctx->poly.nearest.co);
  zero_v3(sctx->poly.nearest.no);
  sctx->poly.nearest.index = -1;
  zero_m4(sctx->poly.obmat);
  sctx->poly.ob = nullptr;
  sctx->poly.data = nullptr;
  sctx->poly.elem = SCE_SNAP_MODE_NONE;

  sctx->edge.nearest.dist_sq = dist_px_sq;
  zero_v3(sctx->edge.nearest.co);
  zero_v3(sctx->edge.nearest.no);
  sctx->edge.nearest.index = -1;
  zero_m4(sctx->edge.obmat);
  sctx->edge.ob = nullptr;
  sctx->edge.data = nullptr;
  sctx->edge.elem = SCE_SNAP_MODE_NONE;

  sctx->point.nearest.dist_sq = dist_px_sq;
  zero_v3(sctx->point.nearest.co);
  zero_v3(sctx->point.nearest.no);
  sctx->point.nearest.index = -1;
  zero_m4(sctx->point.obmat);
  sctx->point.ob = nullptr;
  sctx->point.data = nullptr;
  sctx->point.elem = SCE_SNAP_MODE_NONE;

  sctx->hit_list = hit_list;
  sctx->dist_px_sq_to_edge_center = FLT_MAX;

  return true;
}

static eSnapMode snap_object_context_return(SnapObjectContext *sctx,
                                            eSnapMode snap_to_flag,
                                            float *ray_depth,
                                            float *dist_px,
                                            float r_loc[3],
                                            float r_no[3],
                                            int *r_index,
                                            Object **r_ob,
                                            float r_obmat[4][4])
{
  eSnapMode ret = SCE_SNAP_MODE_NONE;
  Object *ob;
  float *co, *no, (*obmat)[4];
  int index;
  if ((snap_to_flag & sctx->point.elem) &&
      (sctx->point.elem == SCE_SNAP_MODE_EDGE_MIDPOINT ||
       sctx->point.nearest.dist_sq < sctx->dist_px_sq_to_edge_center))
  {
    ob = sctx->point.ob;
    co = sctx->point.nearest.co;
    no = sctx->point.nearest.no;
    obmat = sctx->point.obmat;
    index = sctx->point.nearest.index;
    ret = sctx->point.elem;
    if (dist_px) {
      *dist_px = blender::math::sqrt(sctx->point.nearest.dist_sq);
    }
  }
  else if (snap_to_flag & sctx->edge.elem) {
    ob = sctx->edge.ob;
    co = sctx->edge.nearest.co;
    no = sctx->edge.nearest.no;
    obmat = sctx->edge.obmat;
    index = sctx->edge.nearest.index;
    ret = sctx->edge.elem;
    if (dist_px) {
      *dist_px = blender::math::sqrt(sctx->edge.nearest.dist_sq);
    }
  }
  else if (snap_to_flag & sctx->poly.elem) {
    ob = sctx->poly.ob;
    co = sctx->poly.nearest.co;
    no = sctx->poly.nearest.no;
    obmat = sctx->poly.obmat;
    index = sctx->poly.nearest.index;
    ret = sctx->poly.elem;
    if (ray_depth) {
      *ray_depth = sctx->poly.nearest.dist_sq;
    }
  }

  if (ret != SCE_SNAP_MODE_NONE) {
    if (r_loc) {
      copy_v3_v3(r_loc, co);
      mul_m4_v3(obmat, r_loc);
    }
    if (r_no) {
      float imat[3][3];
      copy_m3_m4(imat, obmat);
      invert_m3(imat);
      copy_v3_v3(r_no, no);
      mul_transposed_m3_v3(imat, r_no);
      normalize_v3(r_no);
    }
    if (r_index) {
      *r_index = index;
    }
    if (r_ob) {
      *r_ob = ob;
    }
    if (r_obmat) {
      copy_m4_m4(r_obmat, obmat);
    }
  }

  return ret;
}

bool ED_transform_snap_object_project_ray_ex(SnapObjectContext *sctx,
                                             Depsgraph *depsgraph,
                                             const View3D *v3d,
                                             const SnapObjectParams *params,
                                             const float ray_start[3],
                                             const float ray_normal[3],
                                             float *ray_depth,
                                             float r_loc[3],
                                             float r_no[3],
                                             int *r_index,
                                             Object **r_ob,
                                             float r_obmat[4][4])
{
  if (!snap_object_context_runtime_init(sctx,
                                        depsgraph,
                                        nullptr,
                                        v3d,
                                        SCE_SNAP_MODE_FACE,
                                        params,
                                        ray_start,
                                        ray_normal,
                                        !ray_depth || *ray_depth == -1.0f ? BVH_RAYCAST_DIST_MAX :
                                                                            *ray_depth,
                                        nullptr,
                                        nullptr,
                                        nullptr,
                                        0,
                                        nullptr,
                                        params->use_occlusion_test))
  {
    return false;
  }

  if (raycastObjects(sctx)) {
    snap_object_context_return(
        sctx, SCE_SNAP_MODE_FACE, ray_depth, nullptr, r_loc, r_no, r_index, r_ob, r_obmat);
    return true;
  }
  return false;
}

bool ED_transform_snap_object_project_ray_all(SnapObjectContext *sctx,
                                              Depsgraph *depsgraph,
                                              const View3D *v3d,
                                              const SnapObjectParams *params,
                                              const float ray_start[3],
                                              const float ray_normal[3],
                                              float ray_depth,
                                              bool sort,
                                              ListBase *r_hit_list)
{
  if (!snap_object_context_runtime_init(sctx,
                                        depsgraph,
                                        nullptr,
                                        v3d,
                                        SCE_SNAP_MODE_FACE,
                                        params,
                                        ray_start,
                                        ray_normal,
                                        ray_depth == -1.0f ? BVH_RAYCAST_DIST_MAX : ray_depth,
                                        nullptr,
                                        nullptr,
                                        nullptr,
                                        0,
                                        r_hit_list,
                                        params->use_occlusion_test))
  {
    return false;
  }

#ifdef DEBUG
  float ray_depth_prev = sctx->poly.nearest.dist_sq;
#endif
  if (raycastObjects(sctx)) {
    if (sort) {
      BLI_listbase_sort(r_hit_list, hit_depth_cmp);
    }
    /* meant to be readonly for 'all' hits, ensure it is */
#ifdef DEBUG
    BLI_assert(ray_depth_prev == sctx->poly.nearest.dist_sq);
#endif
    return true;
  }
  return false;
}

/**
 * Convenience function for snap ray-casting.
 *
 * Given a ray, cast it into the scene (snapping to faces).
 *
 * \return Snap success
 */
bool ED_transform_snap_object_project_ray(SnapObjectContext *sctx,
                                          Depsgraph *depsgraph,
                                          const View3D *v3d,
                                          const SnapObjectParams *params,
                                          const float ray_start[3],
                                          const float ray_normal[3],
                                          float *ray_depth,
                                          float r_co[3],
                                          float r_no[3])
{
  return ED_transform_snap_object_project_ray_ex(sctx,
                                                 depsgraph,
                                                 v3d,
                                                 params,
                                                 ray_start,
                                                 ray_normal,
                                                 ray_depth,
                                                 r_co,
                                                 r_no,
                                                 nullptr,
                                                 nullptr,
                                                 nullptr);
}

eSnapMode ED_transform_snap_object_project_view3d_ex(SnapObjectContext *sctx,
                                                     Depsgraph *depsgraph,
                                                     const ARegion *region,
                                                     const View3D *v3d,
                                                     eSnapMode snap_to_flag,
                                                     const SnapObjectParams *params,
                                                     const float init_co[3],
                                                     const float mval[2],
                                                     const float prev_co[3],
                                                     float *dist_px,
                                                     float r_loc[3],
                                                     float r_no[3],
                                                     int *r_index,
                                                     Object **r_ob,
                                                     float r_obmat[4][4],
                                                     float r_face_nor[3])
{
  eSnapMode retval = SCE_SNAP_MODE_NONE;

  bool use_occlusion_test = params->use_occlusion_test;
  if (use_occlusion_test && XRAY_ENABLED(v3d)) {
    if (snap_to_flag != SCE_SNAP_MODE_FACE) {
      /* In theory everything is visible in X-Ray except faces. */
      snap_to_flag &= ~SCE_SNAP_MODE_FACE;
      use_occlusion_test = false;
    }
  }

  if (use_occlusion_test || (snap_to_flag & SCE_SNAP_MODE_FACE)) {
    if (!ED_view3d_win_to_ray_clipped_ex(depsgraph,
                                         region,
                                         v3d,
                                         mval,
                                         nullptr,
                                         sctx->runtime.ray_dir,
                                         sctx->runtime.ray_start,
                                         true))
    {
      snap_to_flag &= ~SCE_SNAP_MODE_FACE;
      use_occlusion_test = false;
    }
  }

  if (!snap_object_context_runtime_init(sctx,
                                        depsgraph,
                                        region,
                                        v3d,
                                        snap_to_flag,
                                        params,
                                        sctx->runtime.ray_start,
                                        sctx->runtime.ray_dir,
                                        BVH_RAYCAST_DIST_MAX,
                                        mval,
                                        init_co,
                                        prev_co,
                                        dist_px ? square_f(*dist_px) : FLT_MAX,
                                        nullptr,
                                        use_occlusion_test))
  {
    return retval;
  }

  snap_to_flag = sctx->runtime.snap_to_flag;

  BLI_assert(snap_to_flag & (SCE_SNAP_MODE_GEOM | SCE_SNAP_MODE_FACE_NEAREST));

  /* NOTE: if both face ray-cast and face nearest are enabled, first find result of nearest, then
   * override with ray-cast. */
  if (snap_to_flag & SCE_SNAP_MODE_FACE_NEAREST) {
    nearestWorldObjects(sctx);
  }

  float hit_co[3], hit_no[3];
  bool has_hit = false;
  if ((snap_to_flag & SCE_SNAP_MODE_FACE) || sctx->runtime.params.use_occlusion_test) {
    has_hit = raycastObjects(sctx);
    if (has_hit) {
      mul_v3_m4v3(hit_co, sctx->poly.obmat, sctx->poly.nearest.co);

      float imat[3][3];
      copy_m3_m4(imat, sctx->poly.obmat);
      invert_m3(imat);
      copy_v3_v3(hit_no, sctx->poly.nearest.no);
      mul_transposed_m3_v3(imat, hit_no);
      normalize_v3(hit_no);

      if (r_face_nor) {
        copy_v3_v3(r_face_nor, hit_no);
      }
    }
  }

  if (snap_to_flag & (SCE_SNAP_MODE_VERTEX | SCE_SNAP_MODE_EDGE | SCE_SNAP_MODE_EDGE_MIDPOINT |
                      SCE_SNAP_MODE_EDGE_PERPENDICULAR))
  {
    /* First snap to edge instead of middle or perpendicular. */
    sctx->runtime.snap_to_flag &= (SCE_SNAP_MODE_VERTEX | SCE_SNAP_MODE_EDGE |
                                   SCE_SNAP_MODE_EDGE_MIDPOINT | SCE_SNAP_MODE_EDGE_PERPENDICULAR);

    /* By convention we only snap to the original elements of a curve. */
    if (has_hit && sctx->poly.ob->type != OB_CURVES_LEGACY) {
      /* Try to snap to the polygon first. */
      snap_polygon(sctx,
                   snap_to_flag,
                   sctx->poly.ob,
                   sctx->poly.data,
                   sctx->poly.obmat,
                   sctx->poly.nearest.index);

      /* Compute the new clip_pane. */
      float new_clipplane[4];
      plane_from_point_normal_v3(new_clipplane, hit_co, hit_no);
      if (dot_v3v3(sctx->runtime.clip_plane[0], new_clipplane) > 0.0f) {
        /* The plane is facing the wrong direction. */
        negate_v4(new_clipplane);
      }

      /* Move the new clip-plane a little further. This allows coplanar points to be snappable. */
      new_clipplane[3] += 0.001f;

      /* Add the new clip plane to the beginning of the list. */
      for (int i = sctx->runtime.clip_plane_len; i != 0; i--) {
        copy_v4_v4(sctx->runtime.clip_plane[i], sctx->runtime.clip_plane[i - 1]);
      }

      copy_v4_v4(sctx->runtime.clip_plane[0], new_clipplane);
      sctx->runtime.clip_plane_len++;
      sctx->runtime.has_occlusion_plane = true;
    }

    snapObjectsRay(sctx, snap_to_flag);
  }

  retval = snap_object_context_return(
      sctx, snap_to_flag, nullptr, dist_px, r_loc, r_no, r_index, r_ob, r_obmat);

  return retval;
}

eSnapMode ED_transform_snap_object_project_view3d(SnapObjectContext *sctx,
                                                  Depsgraph *depsgraph,
                                                  const ARegion *region,
                                                  const View3D *v3d,
                                                  const eSnapMode snap_to,
                                                  const SnapObjectParams *params,
                                                  const float init_co[3],
                                                  const float mval[2],
                                                  const float prev_co[3],
                                                  float *dist_px,
                                                  float r_loc[3],
                                                  float r_no[3])
{
  return ED_transform_snap_object_project_view3d_ex(sctx,
                                                    depsgraph,
                                                    region,
                                                    v3d,
                                                    snap_to,
                                                    params,
                                                    init_co,
                                                    mval,
                                                    prev_co,
                                                    dist_px,
                                                    r_loc,
                                                    r_no,
                                                    nullptr,
                                                    nullptr,
                                                    nullptr,
                                                    nullptr);
}

bool ED_transform_snap_object_project_all_view3d_ex(SnapObjectContext *sctx,
                                                    Depsgraph *depsgraph,
                                                    const ARegion *region,
                                                    const View3D *v3d,
                                                    const SnapObjectParams *params,
                                                    const float mval[2],
                                                    float ray_depth,
                                                    bool sort,
                                                    ListBase *r_hit_list)
{
  float ray_start[3], ray_normal[3];

  if (!ED_view3d_win_to_ray_clipped_ex(
          depsgraph, region, v3d, mval, nullptr, ray_normal, ray_start, true))
  {
    return false;
  }

  return ED_transform_snap_object_project_ray_all(
      sctx, depsgraph, v3d, params, ray_start, ray_normal, ray_depth, sort, r_hit_list);
}

/** \} */
