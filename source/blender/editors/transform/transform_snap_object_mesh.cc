/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edtransform
 */

#include "BLI_math.h"
#include "BLI_math_matrix_types.hh"

#include "BKE_bvhutils.h"
#include "BKE_editmesh.h"
#include "BKE_mesh.hh"
#include "BKE_object.h"

#include "ED_transform_snap_object_context.h"

#include "transform_snap_object.hh"

using blender::float3;
using blender::Span;

/* -------------------------------------------------------------------- */
/** \name Snap Object Data
 * \{ */

static void snap_object_data_mesh_get(const Mesh *me_eval,
                                      bool use_hide,
                                      BVHTreeFromMesh *r_treedata)
{
  const Span<float3> vert_positions = me_eval->vert_positions();
  const blender::OffsetIndices polys = me_eval->polys();
  const Span<int> corner_verts = me_eval->corner_verts();

  /* The BVHTree from looptris is always required. */
  BKE_bvhtree_from_mesh_get(
      r_treedata, me_eval, use_hide ? BVHTREE_FROM_LOOPTRI_NO_HIDDEN : BVHTREE_FROM_LOOPTRI, 4);

  BLI_assert(reinterpret_cast<const float3 *>(r_treedata->vert_positions) ==
             vert_positions.data());
  BLI_assert(r_treedata->corner_verts == corner_verts.data());
  BLI_assert(!polys.data() || r_treedata->looptri);
  BLI_assert(!r_treedata->tree || r_treedata->looptri);

  UNUSED_VARS_NDEBUG(vert_positions, polys, corner_verts);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Ray Cast Functions
 * \{ */

/* Store all ray-hits
 * Support for storing all depths, not just the first (ray-cast 'all'). */

/* Callback to ray-cast with back-face culling (#Mesh). */
static void mesh_looptri_raycast_backface_culling_cb(void *userdata,
                                                     int index,
                                                     const BVHTreeRay *ray,
                                                     BVHTreeRayHit *hit)
{
  const BVHTreeFromMesh *data = (BVHTreeFromMesh *)userdata;
  const float(*vert_positions)[3] = data->vert_positions;
  const MLoopTri *lt = &data->looptri[index];
  const float *vtri_co[3] = {
      vert_positions[data->corner_verts[lt->tri[0]]],
      vert_positions[data->corner_verts[lt->tri[1]]],
      vert_positions[data->corner_verts[lt->tri[2]]],
  };
  float dist = bvhtree_ray_tri_intersection(ray, hit->dist, UNPACK3(vtri_co));

  if (dist >= 0 && dist < hit->dist) {
    float no[3];
    if (raycast_tri_backface_culling_test(ray->direction, UNPACK3(vtri_co), no)) {
      hit->index = index;
      hit->dist = dist;
      madd_v3_v3v3fl(hit->co, ray->origin, ray->direction, dist);
      normalize_v3_v3(hit->no, no);
    }
  }
}

static bool raycastMesh(SnapObjectContext *sctx,
                        Object *ob_eval,
                        const Mesh *me_eval,
                        const float obmat[4][4],
                        const uint ob_index,
                        bool use_hide,
                        /* read/write args */
                        float *ray_depth,
                        /* return args */
                        float r_loc[3],
                        float r_no[3],
                        int *r_index,
                        ListBase *r_hit_list)
{
  bool retval = false;

  if (me_eval->totpoly == 0) {
    return retval;
  }

  float imat[4][4];
  float ray_start_local[3], ray_normal_local[3];
  float local_scale, local_depth, len_diff = 0.0f;

  invert_m4_m4(imat, obmat);

  copy_v3_v3(ray_start_local, sctx->runtime.ray_start);
  copy_v3_v3(ray_normal_local, sctx->runtime.ray_dir);

  mul_m4_v3(imat, ray_start_local);
  mul_mat3_m4_v3(imat, ray_normal_local);

  /* local scale in normal direction */
  local_scale = normalize_v3(ray_normal_local);
  local_depth = *ray_depth;
  if (local_depth != BVH_RAYCAST_DIST_MAX) {
    local_depth *= local_scale;
  }

  /* Test BoundBox */
  if (ob_eval->data == me_eval) {
    const BoundBox *bb = BKE_mesh_boundbox_get(ob_eval);
    if (bb) {
      /* was BKE_boundbox_ray_hit_check, see: cf6ca226fa58 */
      if (!isect_ray_aabb_v3_simple(
              ray_start_local, ray_normal_local, bb->vec[0], bb->vec[6], &len_diff, nullptr))
      {
        return retval;
      }
    }
  }

  /* We pass a temp ray_start, set from object's boundbox, to avoid precision issues with
   * very far away ray_start values (as returned in case of ortho view3d), see #50486, #38358.
   */
  if (len_diff > 400.0f) {
    /* Make temporary start point a bit away from bounding-box hit point. */
    len_diff -= local_scale;
    madd_v3_v3fl(ray_start_local, ray_normal_local, len_diff);
    local_depth -= len_diff;
  }
  else {
    len_diff = 0.0f;
  }

  BVHTreeFromMesh treedata;
  snap_object_data_mesh_get(me_eval, use_hide, &treedata);

  const blender::Span<int> looptri_polys = me_eval->looptri_polys();

  if (treedata.tree == nullptr) {
    return retval;
  }

  BLI_assert(treedata.raycast_callback != nullptr);
  if (r_hit_list) {
    RayCastAll_Data data;

    data.bvhdata = &treedata;
    data.raycast_callback = treedata.raycast_callback;
    data.obmat = obmat;
    data.len_diff = len_diff;
    data.local_scale = local_scale;
    data.ob_uuid = ob_index;
    data.hit_list = r_hit_list;

    void *hit_last_prev = data.hit_list->last;
    BLI_bvhtree_ray_cast_all(
        treedata.tree, ray_start_local, ray_normal_local, 0.0f, *ray_depth, raycast_all_cb, &data);

    retval = hit_last_prev != data.hit_list->last;
  }
  else {
    BVHTreeRayHit hit{};
    hit.index = -1;
    hit.dist = local_depth;

    if (BLI_bvhtree_ray_cast(treedata.tree,
                             ray_start_local,
                             ray_normal_local,
                             0.0f,
                             &hit,
                             sctx->runtime.params.use_backface_culling ?
                                 mesh_looptri_raycast_backface_culling_cb :
                                 treedata.raycast_callback,
                             &treedata) != -1)
    {
      hit.dist += len_diff;
      hit.dist /= local_scale;
      if (hit.dist <= *ray_depth) {
        *ray_depth = hit.dist;
        copy_v3_v3(r_loc, hit.co);
        copy_v3_v3(r_no, hit.no);
        *r_index = looptri_polys[hit.index];

        retval = true;
      }
    }
  }

  return retval;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Surface Snap Functions
 * \{ */

static bool nearest_world_mesh(SnapObjectContext *sctx,
                               const Mesh *me_eval,
                               const float (*obmat)[4],
                               bool use_hide,
                               float *r_dist_sq,
                               float *r_loc,
                               float *r_no,
                               int *r_index)
{
  BVHTreeFromMesh treedata;
  snap_object_data_mesh_get(me_eval, use_hide, &treedata);
  if (treedata.tree == nullptr) {
    return false;
  }

  return nearest_world_tree(sctx,
                            treedata.tree,
                            treedata.nearest_callback,
                            &treedata,
                            obmat,
                            r_dist_sq,
                            r_loc,
                            r_no,
                            r_index);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Callbacks
 * \{ */

static void cb_snap_edge_verts(void *userdata,
                               int index,
                               const DistProjectedAABBPrecalc *precalc,
                               const float (*clip_plane)[4],
                               const int clip_plane_len,
                               BVHTreeNearest *nearest)
{
  Nearest2dUserData *nearest2d = static_cast<Nearest2dUserData *>(userdata);

  int vindex[2];
  nearest2d->get_edge_verts_index(index, nearest2d, vindex);

  for (int i = 2; i--;) {
    if (vindex[i] == nearest->index) {
      continue;
    }
    cb_snap_vert(userdata, vindex[i], precalc, clip_plane, clip_plane_len, nearest);
  }
}

static void cb_snap_tri_edges(void *userdata,
                              int index,
                              const DistProjectedAABBPrecalc *precalc,
                              const float (*clip_plane)[4],
                              const int clip_plane_len,
                              BVHTreeNearest *nearest)
{
  Nearest2dUserData *nearest2d = static_cast<Nearest2dUserData *>(userdata);

  if (nearest2d->m_use_backface_culling) {
    int vindex[3];
    nearest2d->get_tri_verts_index(index, nearest2d, vindex);

    const float *t0, *t1, *t2;
    nearest2d->get_vert_co(vindex[0], nearest2d, &t0);
    nearest2d->get_vert_co(vindex[1], nearest2d, &t1);
    nearest2d->get_vert_co(vindex[2], nearest2d, &t2);
    float dummy[3];
    if (raycast_tri_backface_culling_test(precalc->ray_direction, t0, t1, t2, dummy)) {
      return;
    }
  }

  int eindex[3];
  nearest2d->get_tri_edges_index(index, nearest2d, eindex);
  for (int i = 3; i--;) {
    if (eindex[i] != -1) {
      if (eindex[i] == nearest->index) {
        continue;
      }
      cb_snap_edge(userdata, eindex[i], precalc, clip_plane, clip_plane_len, nearest);
    }
  }
}

static void cb_snap_tri_verts(void *userdata,
                              int index,
                              const DistProjectedAABBPrecalc *precalc,
                              const float (*clip_plane)[4],
                              const int clip_plane_len,
                              BVHTreeNearest *nearest)
{
  Nearest2dUserData *nearest2d = static_cast<Nearest2dUserData *>(userdata);

  int vindex[3];
  nearest2d->get_tri_verts_index(index, nearest2d, vindex);

  if (nearest2d->m_use_backface_culling) {
    const float *t0, *t1, *t2;
    nearest2d->get_vert_co(vindex[0], nearest2d, &t0);
    nearest2d->get_vert_co(vindex[1], nearest2d, &t1);
    nearest2d->get_vert_co(vindex[2], nearest2d, &t2);
    float dummy[3];
    if (raycast_tri_backface_culling_test(precalc->ray_direction, t0, t1, t2, dummy)) {
      return;
    }
  }

  for (int i = 3; i--;) {
    if (vindex[i] == nearest->index) {
      continue;
    }
    cb_snap_vert(userdata, vindex[i], precalc, clip_plane, clip_plane_len, nearest);
  }
}

static void cb_mvert_co_get(const int index, const Nearest2dUserData *data, const float **r_co)
{
  *r_co = data->vert_positions[index];
}

static void cb_mvert_no_copy(const int index, const Nearest2dUserData *data, float r_no[3])
{
  copy_v3_v3(r_no, data->vert_normals[index]);
}

static void cb_medge_verts_get(const int index, const Nearest2dUserData *data, int r_v_index[2])
{
  const blender::int2 &edge = data->edges[index];

  r_v_index[0] = edge[0];
  r_v_index[1] = edge[1];
}

static void cb_mlooptri_edges_get(const int index, const Nearest2dUserData *data, int r_v_index[3])
{
  const blender::int2 *edges = data->edges;
  const int *corner_verts = data->corner_verts;
  const int *corner_edges = data->corner_edges;
  const MLoopTri *lt = &data->looptris[index];
  for (int j = 2, j_next = 0; j_next < 3; j = j_next++) {
    const blender::int2 &edge = edges[corner_edges[lt->tri[j]]];
    const int tri_edge[2] = {corner_verts[lt->tri[j]], corner_verts[lt->tri[j_next]]};
    if (ELEM(edge[0], tri_edge[0], tri_edge[1]) && ELEM(edge[1], tri_edge[0], tri_edge[1])) {
      // printf("real edge found\n");
      r_v_index[j] = corner_edges[lt->tri[j]];
    }
    else {
      r_v_index[j] = -1;
    }
  }
}

static void cb_mlooptri_verts_get(const int index, const Nearest2dUserData *data, int r_v_index[3])
{
  const int *corner_verts = data->corner_verts;
  const MLoopTri *looptri = &data->looptris[index];

  r_v_index[0] = corner_verts[looptri->tri[0]];
  r_v_index[1] = corner_verts[looptri->tri[1]];
  r_v_index[2] = corner_verts[looptri->tri[2]];
}

static void nearest2d_data_init_mesh(const Mesh *mesh, Nearest2dUserData *r_nearest2d)
{
  r_nearest2d->get_vert_co = cb_mvert_co_get;
  r_nearest2d->get_edge_verts_index = cb_medge_verts_get;
  r_nearest2d->copy_vert_no = cb_mvert_no_copy;
  r_nearest2d->get_tri_verts_index = cb_mlooptri_verts_get;
  r_nearest2d->get_tri_edges_index = cb_mlooptri_edges_get;

  r_nearest2d->vert_positions = mesh->vert_positions().data();
  r_nearest2d->vert_normals = mesh->vert_normals().data();
  r_nearest2d->edges = mesh->edges().data();
  r_nearest2d->corner_verts = mesh->corner_verts().data();
  r_nearest2d->corner_edges = mesh->corner_edges().data();
  r_nearest2d->looptris = mesh->looptris().data();
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Internal Object Snapping API
 * \{ */

static eSnapMode mesh_snap_mode_supported(const Mesh *mesh)
{
  eSnapMode snap_mode_supported = SCE_SNAP_MODE_NONE;
  if (mesh->totpoly) {
    snap_mode_supported |= SCE_SNAP_MODE_FACE | SCE_SNAP_MODE_FACE_NEAREST;
  }
  if (mesh->totedge) {
    snap_mode_supported |= SCE_SNAP_MODE_EDGE | SCE_SNAP_MODE_EDGE_MIDPOINT |
                           SCE_SNAP_MODE_EDGE_PERPENDICULAR;
  }
  if (mesh->totvert) {
    snap_mode_supported |= SCE_SNAP_MODE_VERTEX;
  }
  return snap_mode_supported;
}

void snap_polygon_mesh(
    SnapObjectContext *sctx, Object *ob_eval, const ID *id, const float obmat[4][4], int polygon)
{
  const Mesh *mesh_eval = reinterpret_cast<const Mesh *>(id);

  Nearest2dUserData nearest2d(sctx, ob_eval, id, obmat);
  nearest2d_data_init_mesh(mesh_eval, &nearest2d);

  const blender::IndexRange poly = mesh_eval->polys()[polygon];

  if (nearest2d.m_snap_to_flag &
      (SCE_SNAP_MODE_EDGE | SCE_SNAP_MODE_EDGE_MIDPOINT | SCE_SNAP_MODE_EDGE_PERPENDICULAR))
  {
    BLI_assert(nearest2d.edges != nullptr);
    const int *poly_edges = &nearest2d.corner_edges[poly.start()];
    for (int i = poly.size(); i--;) {
      int vindex[2];
      nearest2d.get_edge_verts_index(poly_edges[i], &nearest2d, vindex);

      const float *v_pair[2];
      nearest2d.get_vert_co(vindex[0], &nearest2d, &v_pair[0]);
      nearest2d.get_vert_co(vindex[1], &nearest2d, &v_pair[1]);

      nearest2d.snap_edge(poly_edges[i], vindex[0], vindex[1], v_pair[0], v_pair[1]);
    }
  }
  else {
    BLI_assert(nearest2d.m_snap_to_flag & SCE_SNAP_MODE_VERTEX);
    const int *poly_verts = &nearest2d.corner_verts[poly.start()];
    for (int i = poly.size(); i--;) {
      const float *co;
      nearest2d.get_vert_co(poly_verts[i], &nearest2d, &co);

      nearest2d.snap_point(poly_verts[i], co);
    }
  }
  nearest2d.confirm(sctx);
}

static void snap_edge_mesh(SnapObjectContext *sctx,
                           Nearest2dUserData *nearest2d,
                           BVHTree *bvhtree_looptri,
                           BVHTree *bvhtree_loose_edges)
{
  BVHTreeNearest nearest = {};
  nearest.index = -1;
  nearest.dist_sq = nearest2d->dist_px_sq();

  if (bvhtree_loose_edges) {
    /* Snap to loose edges. */
    BLI_bvhtree_find_nearest_projected(bvhtree_loose_edges,
                                       nearest2d->m_pmat_local,
                                       sctx->runtime.win_size,
                                       sctx->runtime.mval,
                                       nearest2d->m_clip_plane,
                                       nearest2d->m_clip_plane_len,
                                       &nearest,
                                       cb_snap_edge,
                                       nearest2d);
  }

  if (bvhtree_looptri) {
    /* Snap to looptris. */
    BLI_bvhtree_find_nearest_projected(bvhtree_looptri,
                                       nearest2d->m_pmat_local,
                                       sctx->runtime.win_size,
                                       sctx->runtime.mval,
                                       nearest2d->m_clip_plane,
                                       nearest2d->m_clip_plane_len,
                                       &nearest,
                                       cb_snap_tri_edges,
                                       nearest2d);
  }
}

static void snap_vert_mesh(SnapObjectContext *sctx,
                           Nearest2dUserData *nearest2d,
                           BVHTree *bvhtree_looptri,
                           BVHTree *bvhtree_loose_edges)
{
  const Mesh *mesh_eval = reinterpret_cast<const Mesh *>(nearest2d->m_data);

  BVHTreeFromMesh treedata_dummy;
  BVHTree *bvhtree_loose_verts = BKE_bvhtree_from_mesh_get(
      &treedata_dummy, mesh_eval, BVHTREE_FROM_LOOSEVERTS, 2);
  BLI_assert(treedata_dummy.cached);

  BVHTreeNearest nearest = {};
  nearest.index = -1;
  nearest.dist_sq = nearest2d->dist_px_sq();

  if (bvhtree_loose_verts) {
    /* snap to loose verts */
    BLI_bvhtree_find_nearest_projected(bvhtree_loose_verts,
                                       nearest2d->m_pmat_local,
                                       sctx->runtime.win_size,
                                       sctx->runtime.mval,
                                       nearest2d->m_clip_plane,
                                       nearest2d->m_clip_plane_len,
                                       &nearest,
                                       cb_snap_vert,
                                       nearest2d);
  }

  if (bvhtree_loose_edges) {
    /* Snap to loose edge verts. */
    BLI_bvhtree_find_nearest_projected(bvhtree_loose_edges,
                                       nearest2d->m_pmat_local,
                                       sctx->runtime.win_size,
                                       sctx->runtime.mval,
                                       nearest2d->m_clip_plane,
                                       nearest2d->m_clip_plane_len,
                                       &nearest,
                                       cb_snap_edge_verts,
                                       nearest2d);
  }

  if (bvhtree_looptri) {
    /* Snap to looptris. */
    BLI_bvhtree_find_nearest_projected(bvhtree_looptri,
                                       nearest2d->m_pmat_local,
                                       sctx->runtime.win_size,
                                       sctx->runtime.mval,
                                       nearest2d->m_clip_plane,
                                       nearest2d->m_clip_plane_len,
                                       &nearest,
                                       cb_snap_tri_verts,
                                       nearest2d);
  }
}

static bool snapMesh(SnapObjectContext *sctx,
                     eSnapMode snap_to_flag,
                     Object *ob_eval,
                     ID *id,
                     const float obmat[4][4],
                     bool use_hide)
{
  if (!(snap_to_flag & (SCE_SNAP_MODE_EDGE | SCE_SNAP_MODE_POINTS))) {
    return false;
  }

  Nearest2dUserData nearest2d(sctx, ob_eval, id, obmat);
  const Mesh *me_eval = reinterpret_cast<const Mesh *>(id);

  /* Test BoundBox */
  if (ob_eval->data == me_eval) {
    BoundBox *bb = BKE_mesh_boundbox_get(ob_eval);
    if (!nearest2d.snap_boundbox(bb->vec[0], bb->vec[6])) {
      return false;
    }
  }

  BVHTreeFromMesh treedata, treedata_dummy;
  snap_object_data_mesh_get(me_eval, use_hide, &treedata);
  nearest2d_data_init_mesh(me_eval, &nearest2d);

  BVHTree *bvhtree_loose_edges = BKE_bvhtree_from_mesh_get(
      &treedata_dummy, me_eval, BVHTREE_FROM_LOOSEEDGES, 2);
  BLI_assert(treedata_dummy.cached);

  if (snap_to_flag &
      (SCE_SNAP_MODE_EDGE | SCE_SNAP_MODE_EDGE_MIDPOINT | SCE_SNAP_MODE_EDGE_PERPENDICULAR))
  {
    if (nearest2d.m_snap_to_flag & SCE_SNAP_MODE_VERTEX) {
      /* Start with loose_verts to lower the fetch threshold. */
      snap_vert_mesh(sctx, &nearest2d, nullptr, nullptr);
    }

    snap_edge_mesh(sctx, &nearest2d, treedata.tree, bvhtree_loose_edges);
  }
  else {
    BLI_assert(nearest2d.m_snap_to_flag & SCE_SNAP_MODE_VERTEX);
    snap_vert_mesh(sctx, &nearest2d, treedata.tree, bvhtree_loose_edges);
  }

  return nearest2d.confirm(sctx);
}

/** \} */

void snap_object_mesh(
    SnapObjectContext *sctx, Object *ob_eval, ID *id, const float obmat[4][4], bool use_hide)
{
  Mesh *mesh_eval = reinterpret_cast<Mesh *>(id);
  eSnapMode snap_mode_used = sctx->runtime.snap_to_flag & mesh_snap_mode_supported(mesh_eval);
  if (snapMesh(sctx, snap_mode_used, ob_eval, id, obmat, use_hide)) {
    /* Pass. */;
  }
  else if ((snap_mode_used & SCE_SNAP_MODE_FACE) && raycastMesh(sctx,
                                                                ob_eval,
                                                                mesh_eval,
                                                                obmat,
                                                                sctx->runtime.object_index++,
                                                                use_hide,
                                                                &sctx->poly.nearest.dist_sq,
                                                                sctx->poly.nearest.co,
                                                                sctx->poly.nearest.no,
                                                                &sctx->poly.nearest.index,
                                                                sctx->hit_list))
  {
    sctx->poly.ob = ob_eval;
    sctx->poly.data = id;
    copy_m4_m4(sctx->poly.obmat, obmat);
    sctx->poly.elem = SCE_SNAP_MODE_FACE;
  }
  else if ((snap_mode_used & SCE_SNAP_MODE_FACE_NEAREST) &&
           nearest_world_mesh(sctx,
                              mesh_eval,
                              obmat,
                              use_hide,
                              &sctx->poly.nearest.dist_sq,
                              sctx->poly.nearest.co,
                              sctx->poly.nearest.no,
                              &sctx->poly.nearest.index))
  {
    sctx->poly.ob = ob_eval;
    sctx->poly.data = id;
    copy_m4_m4(sctx->poly.obmat, obmat);
    sctx->poly.elem = SCE_SNAP_MODE_FACE_NEAREST;
  }
}
