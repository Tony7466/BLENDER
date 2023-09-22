/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup editors
 */
#define DNA_DEPRECATED_ALLOW
#include <stdlib.h>

#include "MEM_guardedalloc.h"

#include "BLI_string.h"
#include "BLI_linklist.h"
#include "BLI_math.h"
#include "BLI_utildefines.h"
#include "BLI_task.h"

#include "BKE_context.h"
#include "BKE_global.h"
#include "BKE_gpencil_legacy.h"
#include "BKE_object.h"
#include "BKE_gpencil_modifier_legacy.h"
#include "BKE_modifier.h"
#include "BKE_scene.h"
#include "BKE_bvhutils.h"
#include "BKE_mesh.h"
#include "BKE_mesh_runtime.h"
#include "BKE_mesh_wrapper.h"
#include "BKE_report.h"

#include "MOD_gpencil_legacy_util.h"

#include "DEG_depsgraph_query.h"
#include "ED_object.h"
#include "ED_anim_api.h"
#include "ED_keyframes_edit.h"
#include "WM_api.h"
#include "UI_interface.h"

#include "RNA_access.h"
#include "RNA_prototypes.h"
#include "RNA_define.h"

#include "DNA_gpencil_modifier_types.h"
#include "DNA_gpencil_legacy_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_scene_types.h"


static void freeData_a(SurDeformGpencilModifierData *smd)
{
  GpencilModifierData *md = (GpencilModifierData *)smd;
  const GpencilModifierTypeInfo *mti = BKE_gpencil_modifier_get_info(md->type);
  mti->freeData(md);
  return;
}

/*Populate the liste each time a operator that alters the bind state is exectuted
static void uilist_populate(SurDeformGpencilModifierData *smd, SurDeformGpencilModifierData *smd_eval )
{
  if (smd->uilist_frame_active != NULL) {
    smd->uilist_frame_active -= smd->uilist_frame_active_index;
    MEM_SAFE_FREE(smd->uilist_frame_active);
  }

  smd->uilist_totframes = 0;

  /*Iterate layers twice: once to know how many frames there are so how much memory to
  allocate, second time to actually fill in the data.
  uint prev_frame_num = 0;
  for (int l = 0; l < smd->num_of_layers; smd->layers++) {
    for (int f = 0; f < smd->layers[l].num_of_frames; smd->layers[l].frames++) {
      if (smd->layers[l].frames[f].frame_number > prev_frame_num) {
        smd->uilist_totframes++;
      }
      prev_frame_num = smd->layers[l].frames[f].frame_number;
    }
  }

  smd->uilist_frame_active = MEM_calloc_arrayN(
      smd->uilist_totframes, sizeof(*smd->uilist_frame_active), "SDefBoundFrames");

  if (smd->uilist_frame_active == NULL) {
    BKE_gpencil_modifier_set_error((GpencilModifierData *)smd_eval, "Out of memory");
    MEM_SAFE_FREE(smd->uilist_frame_active);
    smd->uilist_totframes = 0;
    smd->uilist_frame_active_index = 0;
    return;
  }
  SDefGPBoundFrame *first_bound_frame = smd->uilist_frame_active;

  for (int l = 0; l < smd->num_of_layers; smd->layers++) {
    for (int f = 0; f < smd->layers[l].num_of_frames; smd->layers[l].frames++) {
      if (smd->layers[l].frames[f].frame_number > smd->uilist_frame_active->framenum) {
        /*Add a new frame
        smd->uilist_frame_active->framenum = smd->layers[l].frames[f].frame_number;
      }
    }
  }
  
}
  * /

/* STRUCTS FROM MOD_surfacedeform.c */

typedef struct SDefAdjacency {
  struct SDefAdjacency *next;
  uint index;
} SDefAdjacency;

typedef struct SDefAdjacencyArray {
  SDefAdjacency *first;
  uint num; /* Careful, this is twice the number of polygons (avoids an extra loop) */
} SDefAdjacencyArray;

/**
 * Polygons per edge (only 2, any more will exit calculation).
 */
typedef struct SDefEdgePolys {
  uint polys[2], num;
} SDefEdgePolys;

typedef struct SDefBindCalcData {
  BVHTreeFromMesh *const treeData;
  const SDefAdjacencyArray *const vert_edges;
  const SDefEdgePolys *const edge_polys;
  SDefGPStroke *const current_stroke;  // formerly bind_verts
  const MLoopTri *const looptri;
  const MPoly *const mpoly;
  const MEdge *const medge;
  const MLoop *const mloop;
  /** Coordinates to bind to, transformed into local space (compatible with `vertexCos`). */
  float (*const targetCos)[3];
  /** Coordinates to bind (reference to the modifiers input argument).
   * Coordinates are stored in the grease pencil vert for grease pencil, which is accessed from the
   * stroke, also keeps the referenc from the deformstroke inut argument.
   */
  bGPDstroke *gps;
  float imat[4][4];
  const float falloff;
  int success;
  /** Vertex group lookup data. */
  const MDeformVert *const dvert;
  int const defgrp_index;
  bool const invert_vgroup;
  bool const sparse_bind;
} SDefBindCalcData;

/**
 * This represents the relationship between a point (a source coordinate)
 * and the face-corner it's being bound to (from the target mesh).
 *
 * \note Some of these values could be de-duplicated however these are only
 * needed once when running bind, so optimizing this structure isn't a priority.
 */
typedef struct SDefBindPoly {
  /** Coordinates copied directly from the modifiers input. */
  float (*coords)[3];
  /** Coordinates projected into 2D space using `normal`. */
  float (*coords_v2)[2];
  /** The point being queried projected into 2D space using `normal`. */
  float point_v2[2];
  float weight_angular;
  float weight_dist_proj;
  float weight_dist;
  float weight;
  /** Distances from the centroid to edges flanking the corner vertex, used to penalize
   *  small or long and narrow faces in favor of bigger and more square ones. */
  float scales[2];
  /** Distance weight from the corner vertex to the chord line, used to penalize
   *  cases with the three consecutive vertices being nearly in line. */
  float scale_mid;
  /** Center of `coords` */
  float centroid[3];
  /** Center of `coords_v2` */
  float centroid_v2[2];
  /**
   * The calculated normal of coords (could be shared between faces).
   */
  float normal[3];
  /** Vectors pointing from the centroid to the midpoints of the two edges
   *  flanking the corner vertex. */
  float cent_edgemid_vecs_v2[2][2];
  /** Angle between the cent_edgemid_vecs_v2 vectors. */
  float edgemid_angle;
  /** Angles between the centroid-to-point and cent_edgemid_vecs_v2 vectors.
   *  Positive values measured towards the corner; clamped non-negative. */
  float point_edgemid_angles[2];
  /** Angles between the centroid-to-corner and cent_edgemid_vecs_v2 vectors. */
  float corner_edgemid_angles[2];
  /** Weight of the bind mode based on the corner and two adjacent vertices,
   *  versus the one based on the centroid and the dominant edge. */
  float dominant_angle_weight;
  /** Index of the input polygon. */
  uint index;
  /** Number of vertices in this face. */
  uint verts_num;
  /**
   * This polygons loop-start.
   * \note that we could look this up from the polygon.
   */
  uint loopstart;
  uint edge_inds[2];
  uint edge_vert_inds[2];
  /** The index of this corner in the face (starting at zero). */
  uint corner_ind;
  uint dominant_edge;
  /** When true `point_v2` is inside `coords_v2`. */
  bool inside;
} SDefBindPoly;

typedef struct SDefBindWeightData {
  SDefBindPoly *bind_polys;
  uint polys_num;
  uint binds_num;
} SDefBindWeightData;

typedef struct SDefDeformData {
  const SDefGPVert *const bind_verts;
  float (*const targetCos)[3];
  float (*const vertexCos)[3];
  const MDeformVert *const dvert;
  int const defgrp_index;
  bool const invert_vgroup;
  float const strength;
  bGPDstroke *gps;
} SDefDeformData;

/* Bind result values */
enum {
  MOD_SDEF_BIND_RESULT_SUCCESS = 1,
  MOD_SDEF_BIND_RESULT_GENERIC_ERR = 0,
  MOD_SDEF_BIND_RESULT_MEM_ERR = -1,
  MOD_SDEF_BIND_RESULT_NONMANY_ERR = -2,
  MOD_SDEF_BIND_RESULT_CONCAVE_ERR = -3,
  MOD_SDEF_BIND_RESULT_OVERLAP_ERR = -4,
};

/* Infinite weight flags */
enum {
  MOD_SDEF_INFINITE_WEIGHT_ANGULAR = (1 << 0),
  MOD_SDEF_INFINITE_WEIGHT_DIST_PROJ = (1 << 1),
  MOD_SDEF_INFINITE_WEIGHT_DIST = (1 << 2),
};

/* NEW UTILS */

static void rollback_layers_a(SurDeformGpencilModifierData *smd)
{
  if (smd->layers == NULL)
    return;
  smd->layers = smd->layers->first;
  /* for (int l = 0; l < smd->num_of_layers; l++) {
    if (smd->layers->layer_idx == 0) {
      return;
    }
    else {
      smd->layers--;
    }
  }*/
}

static void rollback_lframes_a(SurDeformGpencilModifierData *smd, SDefGPLayer *layer)
{
  if (layer->frames == NULL)
    return;
  layer->frames = layer->frames->first;
  return;
  for (int f = 0; f < layer->num_of_frames; f++) {
    if (layer->frames->frame_idx == 0) {
      return;
    }
    else {
      layer->frames--;
    }
  }
}

static void rollback_strokes_a(SurDeformGpencilModifierData *smd, SDefGPFrame *frame)
{

  if (frame->strokes == NULL)
    return;
  frame->strokes = frame->strokes->first;
  return;
  for (int s = 0; s < frame->strokes_num; s++) {
    if (frame->strokes->stroke_idx == 0) {
      return;
    }
    else {
      frame->strokes--;
    }
  }
}

/* NEW UTILS END */
/* START MOD_surfacedeform.c FUNCTIONS */

static void freeAdjacencyMap(SDefAdjacencyArray *const vert_edges,
                             SDefAdjacency *const adj_ref,
                             SDefEdgePolys *const edge_polys)
{
  MEM_freeN(edge_polys);

  MEM_freeN(adj_ref);

  MEM_freeN(vert_edges);
}

static int buildAdjacencyMap(const MPoly *poly,
                             const MEdge *edge,
                             const MLoop *const mloop,
                             const uint polys_num,
                             const uint edges_num,
                             SDefAdjacencyArray *const vert_edges,
                             SDefAdjacency *adj,
                             SDefEdgePolys *const edge_polys)
{
  const MLoop *loop;
  /* Find polygons adjacent to edges. */

  for (int i = 0; i < polys_num; i++, poly++) {
    loop = &mloop[poly->loopstart];
    for (int j = 0; j < poly->totloop; j++, loop++) {

      if (edge_polys[loop->e].num == 0) {
        edge_polys[loop->e].polys[0] = i;
        edge_polys[loop->e].polys[1] = -1;
        edge_polys[loop->e].num++;
      }
      else if (edge_polys[loop->e].num == 1) {
        edge_polys[loop->e].polys[1] = i;
        edge_polys[loop->e].num++;
      }
      else {
        return MOD_SDEF_BIND_RESULT_NONMANY_ERR;
      }
    }
  }

  /* Find edges adjacent to vertices */
  for (int i = 0; i < edges_num; i++, edge++) {
    adj->next = vert_edges[edge->v1].first;
    adj->index = i;
    vert_edges[edge->v1].first = adj;
    vert_edges[edge->v1].num += edge_polys[i].num;
    adj++;

    adj->next = vert_edges[edge->v2].first;
    adj->index = i;
    vert_edges[edge->v2].first = adj;
    vert_edges[edge->v2].num += edge_polys[i].num;
    adj++;
  }

  return MOD_SDEF_BIND_RESULT_SUCCESS;
}

BLI_INLINE void sortPolyVertsEdge(uint *indices,
                                  const MLoop *const mloop,
                                  const uint edge,
                                  const uint num)
{
  bool found = false;

  for (int i = 0; i < num; i++) {
    if (mloop[i].e == edge) {
      found = true;
    }
    if (found) {
      *indices = mloop[i].v;
      indices++;
    }
  }

  /* Fill in remaining vertex indices that occur before the edge */
  for (int i = 0; mloop[i].e != edge; i++) {
    *indices = mloop[i].v;
    indices++;
  }
}

BLI_INLINE void sortPolyVertsTri(uint *indices,
                                 const MLoop *const mloop,
                                 const uint loopstart,
                                 const uint num)
{
  for (int i = loopstart; i < num; i++) {
    *indices = mloop[i].v;
    indices++;
  }

  for (int i = 0; i < loopstart; i++) {
    *indices = mloop[i].v;
    indices++;
  }
}

BLI_INLINE uint nearestVert(SDefBindCalcData *const data, const float point_co[3])
{

  BVHTreeNearest nearest = {
      .dist_sq = FLT_MAX,
      .index = -1,
  };
  const MPoly *poly;
  const MEdge *edge;
  const MLoop *loop;
  float t_point[3];
  float max_dist = FLT_MAX;
  float dist;
  uint index = 0;

  mul_v3_m4v3(t_point, data->imat, point_co);

  BLI_bvhtree_find_nearest(
      data->treeData->tree, t_point, &nearest, data->treeData->nearest_callback, data->treeData);

  poly = &data->mpoly[data->looptri[nearest.index].poly];
  loop = &data->mloop[poly->loopstart];

  for (int i = 0; i < poly->totloop; i++, loop++) {
    edge = &data->medge[loop->e];
    dist = dist_squared_to_line_segment_v3(
        point_co, data->targetCos[edge->v1], data->targetCos[edge->v2]);

    if (dist < max_dist) {
      max_dist = dist;
      index = loop->e;
    }
  }

  edge = &data->medge[index];
  if (len_squared_v3v3(point_co, data->targetCos[edge->v1]) <
      len_squared_v3v3(point_co, data->targetCos[edge->v2])) {
    return edge->v1;
  }

  return edge->v2;
}

BLI_INLINE int isPolyValid(const float coords[][2], const uint nr)
{
  float prev_co[2], prev_prev_co[2];
  float curr_vec[2], prev_vec[2];

  if (!is_poly_convex_v2(coords, nr)) {
    return MOD_SDEF_BIND_RESULT_CONCAVE_ERR;
  }

  copy_v2_v2(prev_prev_co, coords[nr - 2]);
  copy_v2_v2(prev_co, coords[nr - 1]);
  sub_v2_v2v2(prev_vec, prev_co, coords[nr - 2]);
  normalize_v2(prev_vec);

  for (int i = 0; i < nr; i++) {
    sub_v2_v2v2(curr_vec, coords[i], prev_co);

    /* Check overlap between directly adjacent vertices. */
    const float curr_len = normalize_v2(curr_vec);
    if (curr_len < FLT_EPSILON) {
      return MOD_SDEF_BIND_RESULT_OVERLAP_ERR;
    }

    /* Check overlap between vertices skipping one. */
    if (len_squared_v2v2(prev_prev_co, coords[i]) < FLT_EPSILON * FLT_EPSILON) {
      return MOD_SDEF_BIND_RESULT_OVERLAP_ERR;
    }

    /* Check for adjacent parallel edges. */
    if (1.0f - dot_v2v2(prev_vec, curr_vec) < FLT_EPSILON) {
      return MOD_SDEF_BIND_RESULT_CONCAVE_ERR;
    }

    copy_v2_v2(prev_prev_co, prev_co);
    copy_v2_v2(prev_co, coords[i]);
    copy_v2_v2(prev_vec, curr_vec);
  }

  return MOD_SDEF_BIND_RESULT_SUCCESS;
}

static void freeBindData(SDefBindWeightData *const bwdata)
{
  SDefBindPoly *bpoly = bwdata->bind_polys;

  if (bwdata->bind_polys) {
    for (int i = 0; i < bwdata->polys_num; bpoly++, i++) {
      MEM_SAFE_FREE(bpoly->coords);
      MEM_SAFE_FREE(bpoly->coords_v2);
    }

    MEM_freeN(bwdata->bind_polys);
  }

  MEM_freeN(bwdata);
}

BLI_INLINE float computeAngularWeight(const float point_angle, const float edgemid_angle)
{
  return sinf(min_ff(point_angle / edgemid_angle, 1) * M_PI_2);
}

BLI_INLINE SDefBindWeightData *computeBindWeights(SDefBindCalcData *const data,
                                                  const float point_co[3])
{
  const uint nearest = nearestVert(data, point_co);
  const SDefAdjacency *const vert_edges = data->vert_edges[nearest].first;
  const SDefEdgePolys *const edge_polys = data->edge_polys;

  const SDefAdjacency *vedge;
  const MPoly *poly;
  const MLoop *loop;

  SDefBindWeightData *bwdata;
  SDefBindPoly *bpoly;

  const float world[3] = {0.0f, 0.0f, 1.0f};
  float avg_point_dist = 0.0f;
  float tot_weight = 0.0f;
  int inf_weight_flags = 0;

  bwdata = MEM_callocN(sizeof(*bwdata), "SDefBindWeightData");
  if (bwdata == NULL) {
    data->success = MOD_SDEF_BIND_RESULT_MEM_ERR;
    return NULL;
  }

  bwdata->polys_num = data->vert_edges[nearest].num / 2;

  bpoly = MEM_calloc_arrayN(bwdata->polys_num, sizeof(*bpoly), "SDefBindPoly");
  if (bpoly == NULL) {
    freeBindData(bwdata);
    data->success = MOD_SDEF_BIND_RESULT_MEM_ERR;
    return NULL;
  }

  bwdata->bind_polys = bpoly;

  /* Loop over all adjacent edges,
   * and build the #SDefBindPoly data for each poly adjacent to those. */
  for (vedge = vert_edges; vedge; vedge = vedge->next) {
    uint edge_ind = vedge->index;

    for (int i = 0; i < edge_polys[edge_ind].num; i++) {
      {
        bpoly = bwdata->bind_polys;

        for (int j = 0; j < bwdata->polys_num; bpoly++, j++) {
          /* If coords isn't allocated, we have reached the first uninitialized `bpoly`. */
          if ((bpoly->index == edge_polys[edge_ind].polys[i]) || (!bpoly->coords)) {
            break;
          }
        }
      }

      /* Check if poly was already created by another edge or still has to be initialized */
      if (!bpoly->coords) {
        float angle;
        float axis[3];
        float tmp_vec_v2[2];
        int is_poly_valid;

        bpoly->index = edge_polys[edge_ind].polys[i];
        bpoly->coords = NULL;
        bpoly->coords_v2 = NULL;

        /* Copy poly data */
        poly = &data->mpoly[bpoly->index];
        loop = &data->mloop[poly->loopstart];

        bpoly->verts_num = poly->totloop;
        bpoly->loopstart = poly->loopstart;

        bpoly->coords = MEM_malloc_arrayN(
            poly->totloop, sizeof(*bpoly->coords), "SDefBindPolyCoords");
        if (bpoly->coords == NULL) {
          freeBindData(bwdata);
          data->success = MOD_SDEF_BIND_RESULT_MEM_ERR;
          return NULL;
        }

        bpoly->coords_v2 = MEM_malloc_arrayN(
            poly->totloop, sizeof(*bpoly->coords_v2), "SDefBindPolyCoords_v2");
        if (bpoly->coords_v2 == NULL) {
          freeBindData(bwdata);
          data->success = MOD_SDEF_BIND_RESULT_MEM_ERR;
          return NULL;
        }

        for (int j = 0; j < poly->totloop; j++, loop++) {
          copy_v3_v3(bpoly->coords[j], data->targetCos[loop->v]);

          /* Find corner and edge indices within poly loop array */
          if (loop->v == nearest) {
            bpoly->corner_ind = j;
            bpoly->edge_vert_inds[0] = (j == 0) ? (poly->totloop - 1) : (j - 1);
            bpoly->edge_vert_inds[1] = (j == poly->totloop - 1) ? (0) : (j + 1);

            bpoly->edge_inds[0] = data->mloop[poly->loopstart + bpoly->edge_vert_inds[0]].e;
            bpoly->edge_inds[1] = loop->e;
          }
        }

        /* Compute polygons parametric data. */
        mid_v3_v3_array(bpoly->centroid, bpoly->coords, poly->totloop);
        normal_poly_v3(bpoly->normal, bpoly->coords, poly->totloop);

        /* Compute poly skew angle and axis */
        angle = angle_normalized_v3v3(bpoly->normal, world);

        cross_v3_v3v3(axis, bpoly->normal, world);
        normalize_v3(axis);

        /* Map coords onto 2d normal plane. */
        map_to_plane_axis_angle_v2_v3v3fl(bpoly->point_v2, point_co, axis, angle);

        zero_v2(bpoly->centroid_v2);
        for (int j = 0; j < poly->totloop; j++) {
          map_to_plane_axis_angle_v2_v3v3fl(bpoly->coords_v2[j], bpoly->coords[j], axis, angle);
          madd_v2_v2fl(bpoly->centroid_v2, bpoly->coords_v2[j], 1.0f / poly->totloop);
        }

        is_poly_valid = isPolyValid(bpoly->coords_v2, poly->totloop);

        if (is_poly_valid != MOD_SDEF_BIND_RESULT_SUCCESS) {
          freeBindData(bwdata);
          data->success = is_poly_valid;
          return NULL;
        }

        bpoly->inside = isect_point_poly_v2(
            bpoly->point_v2, bpoly->coords_v2, poly->totloop, false);

        /* Initialize weight components */
        bpoly->weight_angular = 1.0f;
        bpoly->weight_dist_proj = len_v2v2(bpoly->centroid_v2, bpoly->point_v2);
        bpoly->weight_dist = len_v3v3(bpoly->centroid, point_co);

        avg_point_dist += bpoly->weight_dist;

        /* Common vertex coordinates. */
        const float *const vert0_v2 = bpoly->coords_v2[bpoly->edge_vert_inds[0]];
        const float *const vert1_v2 = bpoly->coords_v2[bpoly->edge_vert_inds[1]];
        const float *const corner_v2 = bpoly->coords_v2[bpoly->corner_ind];

        /* Compute centroid to mid-edge vectors */
        mid_v2_v2v2(bpoly->cent_edgemid_vecs_v2[0], vert0_v2, corner_v2);
        mid_v2_v2v2(bpoly->cent_edgemid_vecs_v2[1], vert1_v2, corner_v2);

        sub_v2_v2(bpoly->cent_edgemid_vecs_v2[0], bpoly->centroid_v2);
        sub_v2_v2(bpoly->cent_edgemid_vecs_v2[1], bpoly->centroid_v2);

        normalize_v2(bpoly->cent_edgemid_vecs_v2[0]);
        normalize_v2(bpoly->cent_edgemid_vecs_v2[1]);

        /* Compute poly scales with respect to the two edges. */
        bpoly->scales[0] = dist_to_line_v2(bpoly->centroid_v2, vert0_v2, corner_v2);
        bpoly->scales[1] = dist_to_line_v2(bpoly->centroid_v2, vert1_v2, corner_v2);

        /* Compute the angle between the edge mid vectors. */
        bpoly->edgemid_angle = angle_normalized_v2v2(bpoly->cent_edgemid_vecs_v2[0],
                                                     bpoly->cent_edgemid_vecs_v2[1]);

        /* Compute the angles between the corner and the edge mid vectors. The angles
         * are computed signed in order to correctly clamp point_edgemid_angles later. */
        float corner_angles[2];

        sub_v2_v2v2(tmp_vec_v2, corner_v2, bpoly->centroid_v2);
        normalize_v2(tmp_vec_v2);

        corner_angles[0] = angle_signed_v2v2(tmp_vec_v2, bpoly->cent_edgemid_vecs_v2[0]);
        corner_angles[1] = angle_signed_v2v2(tmp_vec_v2, bpoly->cent_edgemid_vecs_v2[1]);

        bpoly->corner_edgemid_angles[0] = fabsf(corner_angles[0]);
        bpoly->corner_edgemid_angles[1] = fabsf(corner_angles[1]);

        /* Verify that the computed values are valid (the polygon isn't somehow
         * degenerate despite having passed isPolyValid). */
        if (bpoly->scales[0] < FLT_EPSILON || bpoly->scales[1] < FLT_EPSILON ||
            bpoly->edgemid_angle < FLT_EPSILON || bpoly->corner_edgemid_angles[0] < FLT_EPSILON ||
            bpoly->corner_edgemid_angles[1] < FLT_EPSILON) {
          freeBindData(bwdata);
          data->success = MOD_SDEF_BIND_RESULT_GENERIC_ERR;
          return NULL;
        }

        /* Check for infinite weights, and compute angular data otherwise. */
        if (bpoly->weight_dist < FLT_EPSILON) {
          inf_weight_flags |= MOD_SDEF_INFINITE_WEIGHT_DIST_PROJ;
          inf_weight_flags |= MOD_SDEF_INFINITE_WEIGHT_DIST;
        }
        else if (bpoly->weight_dist_proj < FLT_EPSILON) {
          inf_weight_flags |= MOD_SDEF_INFINITE_WEIGHT_DIST_PROJ;
        }
        else {
          /* Compute angles between the point and the edge mid vectors. */
          float cent_point_vec[2], point_angles[2];

          sub_v2_v2v2(cent_point_vec, bpoly->point_v2, bpoly->centroid_v2);
          normalize_v2(cent_point_vec);

          point_angles[0] = angle_signed_v2v2(cent_point_vec, bpoly->cent_edgemid_vecs_v2[0]) *
                            signf(corner_angles[0]);
          point_angles[1] = angle_signed_v2v2(cent_point_vec, bpoly->cent_edgemid_vecs_v2[1]) *
                            signf(corner_angles[1]);

          if (point_angles[0] <= 0 && point_angles[1] <= 0) {
            /* If the point is outside the corner formed by the edge mid vectors,
             * choose to clamp the closest side and flip the other. */
            if (point_angles[0] < point_angles[1]) {
              point_angles[0] = bpoly->edgemid_angle - point_angles[1];
            }
            else {
              point_angles[1] = bpoly->edgemid_angle - point_angles[0];
            }
          }

          bpoly->point_edgemid_angles[0] = max_ff(0, point_angles[0]);
          bpoly->point_edgemid_angles[1] = max_ff(0, point_angles[1]);

          /* Compute the distance scale for the corner. The base value is the orthogonal
           * distance from the corner to the chord, scaled by sqrt(2) to preserve the old
           * values in case of a square grid. This doesn't use the centroid because the
           * LOOPTRI method only uses these three vertices. */
          bpoly->scale_mid = area_tri_v2(vert0_v2, corner_v2, vert1_v2) /
                             len_v2v2(vert0_v2, vert1_v2) * sqrtf(2);

          if (bpoly->inside) {
            /* When inside, interpolate to centroid-based scale close to the center. */
            float min_dist = min_ff(bpoly->scales[0], bpoly->scales[1]);

            bpoly->scale_mid = interpf(bpoly->scale_mid,
                                       (bpoly->scales[0] + bpoly->scales[1]) / 2,
                                       min_ff(bpoly->weight_dist_proj / min_dist, 1));
          }

          /* Verify that the additional computed values are valid. */
          if (bpoly->scale_mid < FLT_EPSILON ||
              bpoly->point_edgemid_angles[0] + bpoly->point_edgemid_angles[1] < FLT_EPSILON) {
            freeBindData(bwdata);
            data->success = MOD_SDEF_BIND_RESULT_GENERIC_ERR;
            return NULL;
          }
        }
      }
    }
  }

  avg_point_dist /= bwdata->polys_num;

  /* If weights 1 and 2 are not infinite, loop over all adjacent edges again,
   * and build adjacency dependent angle data (depends on all polygons having been computed) */
  if (!inf_weight_flags) {
    for (vedge = vert_edges; vedge; vedge = vedge->next) {
      SDefBindPoly *bpolys[2];
      const SDefEdgePolys *epolys;
      float ang_weights[2];
      uint edge_ind = vedge->index;
      uint edge_on_poly[2];

      epolys = &edge_polys[edge_ind];

      /* Find bind polys corresponding to the edge's adjacent polys */
      bpoly = bwdata->bind_polys;

      for (int i = 0, j = 0; (i < bwdata->polys_num) && (j < epolys->num); bpoly++, i++) {
        if (ELEM(bpoly->index, epolys->polys[0], epolys->polys[1])) {
          bpolys[j] = bpoly;

          if (bpoly->edge_inds[0] == edge_ind) {
            edge_on_poly[j] = 0;
          }
          else {
            edge_on_poly[j] = 1;
          }

          j++;
        }
      }

      /* Compute angular weight component */
      if (epolys->num == 1) {
        ang_weights[0] = computeAngularWeight(bpolys[0]->point_edgemid_angles[edge_on_poly[0]],
                                              bpolys[0]->edgemid_angle);
        bpolys[0]->weight_angular *= ang_weights[0] * ang_weights[0];
      }
      else if (epolys->num == 2) {
        ang_weights[0] = computeAngularWeight(bpolys[0]->point_edgemid_angles[edge_on_poly[0]],
                                              bpolys[0]->edgemid_angle);
        ang_weights[1] = computeAngularWeight(bpolys[1]->point_edgemid_angles[edge_on_poly[1]],
                                              bpolys[1]->edgemid_angle);

        bpolys[0]->weight_angular *= ang_weights[0] * ang_weights[1];
        bpolys[1]->weight_angular *= ang_weights[0] * ang_weights[1];
      }
    }
  }

  /* Compute scaling and falloff:
   * - Scale all weights if no infinite weight is found.
   * - Scale only un-projected weight if projected weight is infinite.
   * - Scale none if both are infinite. */
  if (!inf_weight_flags) {
    bpoly = bwdata->bind_polys;

    for (int i = 0; i < bwdata->polys_num; bpoly++, i++) {
      float corner_angle_weights[2];
      float scale_weight, sqr, inv_sqr;

      corner_angle_weights[0] = bpoly->point_edgemid_angles[0] / bpoly->corner_edgemid_angles[0];
      corner_angle_weights[1] = bpoly->point_edgemid_angles[1] / bpoly->corner_edgemid_angles[1];

      if (isnan(corner_angle_weights[0]) || isnan(corner_angle_weights[1])) {
        freeBindData(bwdata);
        data->success = MOD_SDEF_BIND_RESULT_GENERIC_ERR;
        return NULL;
      }

      /* Find which edge the point is closer to */
      if (corner_angle_weights[0] < corner_angle_weights[1]) {
        bpoly->dominant_edge = 0;
        bpoly->dominant_angle_weight = corner_angle_weights[0];
      }
      else {
        bpoly->dominant_edge = 1;
        bpoly->dominant_angle_weight = corner_angle_weights[1];
      }

      /* Check for invalid weights just in case computations fail. */
      if (bpoly->dominant_angle_weight < 0 || bpoly->dominant_angle_weight > 1) {
        freeBindData(bwdata);
        data->success = MOD_SDEF_BIND_RESULT_GENERIC_ERR;
        return NULL;
      }

      bpoly->dominant_angle_weight = sinf(bpoly->dominant_angle_weight * M_PI_2);

      /* Compute quadratic angular scale interpolation weight */
      {
        const float edge_angle_a = bpoly->point_edgemid_angles[bpoly->dominant_edge];
        const float edge_angle_b = bpoly->point_edgemid_angles[!bpoly->dominant_edge];
        /* Clamp so skinny faces with near zero `edgemid_angle`
         * won't cause numeric problems. see T81988. */
        scale_weight = edge_angle_a / max_ff(edge_angle_a, bpoly->edgemid_angle);
        scale_weight /= scale_weight + (edge_angle_b / max_ff(edge_angle_b, bpoly->edgemid_angle));
      }

      sqr = scale_weight * scale_weight;
      inv_sqr = 1.0f - scale_weight;
      inv_sqr *= inv_sqr;
      scale_weight = sqr / (sqr + inv_sqr);

      BLI_assert(scale_weight >= 0 && scale_weight <= 1);

      /* Compute interpolated scale (no longer need the individual scales,
       * so simply storing the result over the scale in index zero) */
      bpoly->scales[0] = interpf(bpoly->scale_mid,
                                 interpf(bpoly->scales[!bpoly->dominant_edge],
                                         bpoly->scales[bpoly->dominant_edge],
                                         scale_weight),
                                 bpoly->dominant_angle_weight);

      /* Scale the point distance weights, and introduce falloff */
      bpoly->weight_dist_proj /= bpoly->scales[0];
      bpoly->weight_dist_proj = powf(bpoly->weight_dist_proj, data->falloff);

      bpoly->weight_dist /= avg_point_dist;
      bpoly->weight_dist = powf(bpoly->weight_dist, data->falloff);

      /* Re-check for infinite weights, now that all scalings and interpolations are computed */
      if (bpoly->weight_dist < FLT_EPSILON) {
        inf_weight_flags |= MOD_SDEF_INFINITE_WEIGHT_DIST_PROJ;
        inf_weight_flags |= MOD_SDEF_INFINITE_WEIGHT_DIST;
      }
      else if (bpoly->weight_dist_proj < FLT_EPSILON) {
        inf_weight_flags |= MOD_SDEF_INFINITE_WEIGHT_DIST_PROJ;
      }
      else if (bpoly->weight_angular < FLT_EPSILON) {
        inf_weight_flags |= MOD_SDEF_INFINITE_WEIGHT_ANGULAR;
      }
    }
  }
  else if (!(inf_weight_flags & MOD_SDEF_INFINITE_WEIGHT_DIST)) {
    bpoly = bwdata->bind_polys;

    for (int i = 0; i < bwdata->polys_num; bpoly++, i++) {
      /* Scale the point distance weight by average point distance, and introduce falloff */
      bpoly->weight_dist /= avg_point_dist;
      bpoly->weight_dist = powf(bpoly->weight_dist, data->falloff);

      /* Re-check for infinite weights, now that all scalings and interpolations are computed */
      if (bpoly->weight_dist < FLT_EPSILON) {
        inf_weight_flags |= MOD_SDEF_INFINITE_WEIGHT_DIST;
      }
    }
  }

  /* Final loop, to compute actual weights */
  bpoly = bwdata->bind_polys;

  for (int i = 0; i < bwdata->polys_num; bpoly++, i++) {
    /* Weight computation from components */
    if (inf_weight_flags & MOD_SDEF_INFINITE_WEIGHT_DIST) {
      bpoly->weight = bpoly->weight_dist < FLT_EPSILON ? 1.0f : 0.0f;
    }
    else if (inf_weight_flags & MOD_SDEF_INFINITE_WEIGHT_DIST_PROJ) {
      bpoly->weight = bpoly->weight_dist_proj < FLT_EPSILON ? 1.0f / bpoly->weight_dist : 0.0f;
    }
    else if (inf_weight_flags & MOD_SDEF_INFINITE_WEIGHT_ANGULAR) {
      bpoly->weight = bpoly->weight_angular < FLT_EPSILON ?
                          1.0f / bpoly->weight_dist_proj / bpoly->weight_dist :
                          0.0f;
    }
    else {
      bpoly->weight = 1.0f / bpoly->weight_angular / bpoly->weight_dist_proj / bpoly->weight_dist;
    }

    /* Apply after other kinds of scaling so the faces corner angle is always
     * scaled in a uniform way, preventing heavily sub-divided triangle fans
     * from having a lop-sided influence on the weighting, see T81988. */
    bpoly->weight *= bpoly->edgemid_angle / M_PI;

    tot_weight += bpoly->weight;
  }

  bpoly = bwdata->bind_polys;

  for (int i = 0; i < bwdata->polys_num; bpoly++, i++) {
    bpoly->weight /= tot_weight;

    /* Evaluate if this poly is relevant to bind */
    /* Even though the weights should add up to 1.0,
     * the losses of weights smaller than epsilon here
     * should be negligible... */
    if (bpoly->weight >= FLT_EPSILON) {
      if (bpoly->inside) {
        bwdata->binds_num += 1;
      }
      else {
        if (bpoly->dominant_angle_weight < FLT_EPSILON ||
            1.0f - bpoly->dominant_angle_weight < FLT_EPSILON) {
          bwdata->binds_num += 1;
        }
        else {
          bwdata->binds_num += 2;
        }
      }
    }
  }

  return bwdata;
}

BLI_INLINE float computeNormalDisplacement(const float point_co[3],
                                           const float point_co_proj[3],
                                           const float normal[3])
{
  float disp_vec[3];
  float normal_dist;

  sub_v3_v3v3(disp_vec, point_co, point_co_proj);
  normal_dist = len_v3(disp_vec);

  if (dot_v3v3(disp_vec, normal) < 0) {
    normal_dist *= -1;
  }

  return normal_dist;
}

static void bindVert(void *__restrict userdata,
                     const int index,
                     const TaskParallelTLS *__restrict UNUSED(tls))
{
  SDefBindCalcData *const data = (SDefBindCalcData *)userdata;
  float point_co[3];
  float point_co_proj[3];

  SDefBindWeightData *bwdata;
  SDefGPVert *sdvert = data->current_stroke->verts + index;
  SDefBindPoly *bpoly;
  SDefGPBind *sdbind;

  sdvert->vertex_idx = index;
  const float *vertex_cos = &(data->gps->points[index].x);

  if (data->success != MOD_SDEF_BIND_RESULT_SUCCESS) {
    sdvert->binds = NULL;
    sdvert->binds_num = 0;
    return;
  }

  /*if (data->sparse_bind) {
    float weight = 0.0f;

    if (data->dvert && data->defgrp_index != -1) {
      weight = BKE_defvert_find_weight(&data->dvert[index], data->defgrp_index);
    }

    if (data->invert_vgroup) {
      weight = 1.0f - weight;
    }

    if (weight <= 0) {
      sdvert->binds = NULL;
      sdvert->binds_num = 0;
      return;
    }
  }*/
  copy_v3_v3(point_co, vertex_cos);
  bwdata = computeBindWeights(data, point_co);

  if (bwdata == NULL) {
    sdvert->binds = NULL;
    sdvert->binds_num = 0;
    return;
  }
  sdvert->binds = MEM_calloc_arrayN(bwdata->binds_num, sizeof(*sdvert->binds), "SDefVertBindData");
  if (sdvert->binds == NULL) {
    data->success = MOD_SDEF_BIND_RESULT_MEM_ERR;
    sdvert->binds_num = 0;
    return;
  }

  sdvert->binds_num = bwdata->binds_num;

  sdbind = sdvert->binds;

  bpoly = bwdata->bind_polys;

  for (int i = 0; i < bwdata->binds_num; bpoly++) {
    if (bpoly->weight >= FLT_EPSILON) {
      if (bpoly->inside) {
        const MLoop *loop = &data->mloop[bpoly->loopstart];

        sdbind->influence = bpoly->weight;
        sdbind->verts_num = bpoly->verts_num;

        sdbind->mode = GP_MOD_SDEF_MODE_NGON;
        sdbind->vert_weights = MEM_malloc_arrayN(
            bpoly->verts_num, sizeof(*sdbind->vert_weights), "SDefNgonVertWeights");
        if (sdbind->vert_weights == NULL) {
          data->success = MOD_SDEF_BIND_RESULT_MEM_ERR;
          return;
        }

        sdbind->vert_inds = MEM_malloc_arrayN(
            bpoly->verts_num, sizeof(*sdbind->vert_inds), "SDefNgonVertInds");
        if (sdbind->vert_inds == NULL) {
          data->success = MOD_SDEF_BIND_RESULT_MEM_ERR;
          return;
        }

        interp_weights_poly_v2(
            sdbind->vert_weights, bpoly->coords_v2, bpoly->verts_num, bpoly->point_v2);

        /* Re-project vert based on weights and original poly verts,
         * to reintroduce poly non-planarity */
        zero_v3(point_co_proj);
        for (int j = 0; j < bpoly->verts_num; j++, loop++) {
          madd_v3_v3fl(point_co_proj, bpoly->coords[j], sdbind->vert_weights[j]);
          sdbind->vert_inds[j] = loop->v;
        }

        sdbind->normal_dist = computeNormalDisplacement(point_co, point_co_proj, bpoly->normal);

        sdbind++;
        i++;
      }
      else {
        float tmp_vec[3];
        float cent[3], norm[3];
        float v1[3], v2[3], v3[3];

        if (1.0f - bpoly->dominant_angle_weight >= FLT_EPSILON) {
          sdbind->influence = bpoly->weight * (1.0f - bpoly->dominant_angle_weight);
          sdbind->verts_num = bpoly->verts_num;

          sdbind->mode = GP_MOD_SDEF_MODE_CENTROID;
          sdbind->vert_weights = MEM_malloc_arrayN(
              3, sizeof(*sdbind->vert_weights), "SDefCentVertWeights");
          if (sdbind->vert_weights == NULL) {
            data->success = MOD_SDEF_BIND_RESULT_MEM_ERR;
            return;
          }

          sdbind->vert_inds = MEM_malloc_arrayN(
              bpoly->verts_num, sizeof(*sdbind->vert_inds), "SDefCentVertInds");
          if (sdbind->vert_inds == NULL) {
            data->success = MOD_SDEF_BIND_RESULT_MEM_ERR;
            return;
          }

          sortPolyVertsEdge(sdbind->vert_inds,
                            &data->mloop[bpoly->loopstart],
                            bpoly->edge_inds[bpoly->dominant_edge],
                            bpoly->verts_num);

          copy_v3_v3(v1, data->targetCos[sdbind->vert_inds[0]]);
          copy_v3_v3(v2, data->targetCos[sdbind->vert_inds[1]]);
          copy_v3_v3(v3, bpoly->centroid);

          mid_v3_v3v3v3(cent, v1, v2, v3);
          normal_tri_v3(norm, v1, v2, v3);

          add_v3_v3v3(tmp_vec, point_co, bpoly->normal);

          /* We are sure the line is not parallel to the plane.
           * Checking return value just to avoid warning... */
          if (!isect_line_plane_v3(point_co_proj, point_co, tmp_vec, cent, norm)) {
            BLI_assert(false);
          }

          interp_weights_tri_v3(sdbind->vert_weights, v1, v2, v3, point_co_proj);

          sdbind->normal_dist = computeNormalDisplacement(point_co, point_co_proj, bpoly->normal);

          sdbind++;
          i++;
        }

        if (bpoly->dominant_angle_weight >= FLT_EPSILON) {
          sdbind->influence = bpoly->weight * bpoly->dominant_angle_weight;
          sdbind->verts_num = bpoly->verts_num;

          sdbind->mode = GP_MOD_SDEF_MODE_LOOPTRI;
          sdbind->vert_weights = MEM_malloc_arrayN(
              3, sizeof(*sdbind->vert_weights), "SDefTriVertWeights");
          if (sdbind->vert_weights == NULL) {
            data->success = MOD_SDEF_BIND_RESULT_MEM_ERR;
            return;
          }

          sdbind->vert_inds = MEM_malloc_arrayN(
              bpoly->verts_num, sizeof(*sdbind->vert_inds), "SDefTriVertInds");
          if (sdbind->vert_inds == NULL) {
            data->success = MOD_SDEF_BIND_RESULT_MEM_ERR;
            return;
          }

          sortPolyVertsTri(sdbind->vert_inds,
                           &data->mloop[bpoly->loopstart],
                           bpoly->edge_vert_inds[0],
                           bpoly->verts_num);

          copy_v3_v3(v1, data->targetCos[sdbind->vert_inds[0]]);
          copy_v3_v3(v2, data->targetCos[sdbind->vert_inds[1]]);
          copy_v3_v3(v3, data->targetCos[sdbind->vert_inds[2]]);

          mid_v3_v3v3v3(cent, v1, v2, v3);
          normal_tri_v3(norm, v1, v2, v3);

          add_v3_v3v3(tmp_vec, point_co, bpoly->normal);

          /* We are sure the line is not parallel to the plane.
           * Checking return value just to avoid warning... */
          if (!isect_line_plane_v3(point_co_proj, point_co, tmp_vec, cent, norm)) {
            BLI_assert(false);
          }

          interp_weights_tri_v3(sdbind->vert_weights, v1, v2, v3, point_co_proj);

          sdbind->normal_dist = computeNormalDisplacement(point_co, point_co_proj, bpoly->normal);

          sdbind++;
          i++;
        }
      }
    }
  }

  freeBindData(bwdata);
}


/* ADD AND FREE LAYER AND FRAME FUNCS*/
static bool add_layer(SurDeformGpencilModifierData *smd_orig,
                      SurDeformGpencilModifierData *smd_eval,
                      bGPDlayer *gpl)
{ /* If we're not on the first layer, rollback the pointer*/

  rollback_layers_a(smd_orig);

  /*We can't use MEM_recallocN because it doesn't support arrays.
   */
  smd_orig->num_of_layers++;
  SDefGPLayer *temp_lay_pointer = MEM_calloc_arrayN(
      smd_orig->num_of_layers, sizeof(*smd_orig->layers), "SDefGPLayers");

  if (temp_lay_pointer == NULL) {
    BKE_gpencil_modifier_set_error((GpencilModifierData *)smd_eval, "Out of memory");
    return false;
  }

  /*if this is the first frame, so num_of_frame has just become one, we don't need to copy
   * anything.*/
  if (smd_orig->num_of_layers > 1) {
    memcpy(temp_lay_pointer,
           smd_orig->layers,
           sizeof(*smd_orig->layers) * (smd_orig->num_of_layers - 1));
    MEM_SAFE_FREE(smd_orig->layers);
  }
  smd_orig->layers = temp_lay_pointer;
  uint old_lay_index;
  /*Now smd_orig->layers is on 0.
  * We know this is the first ayer.
  Then we need to bring it to the new layer position.*/
  for (int l = 0; l < smd_orig->num_of_layers - 1; l++) {
    smd_orig->layers->first = temp_lay_pointer;
    old_lay_index = smd_orig->layers->layer_idx;
    smd_orig->layers++;
  }

  /*Fill in the layer data*/
  smd_orig->layers->frames = NULL;
  smd_orig->layers->num_of_frames = 0;
  smd_orig->layers->blender_layer = gpl;
  smd_orig->layers->first = temp_lay_pointer;
  if (smd_orig->num_of_layers > 1) {
    smd_orig->layers->layer_idx = old_lay_index + 1;
  }
  else {
    smd_orig->layers->layer_idx = 0;
  }

  // for (int c = 0; c < 128; c++)
  strcpy(smd_orig->layers->layer_info, gpl->info);
  // smd_orig->layers->layer_idx

  return true;
}

static bool add_frame(SurDeformGpencilModifierData *smd_orig,
                      SurDeformGpencilModifierData *smd_eval,
                      SDefGPLayer *sdef_layer,
                      bGPDframe *gpf)
  {
    rollback_lframes_a(smd_orig, sdef_layer);
    SDefGPFrame *first_frame;
  
    /* every time we add a new frame to our array, by the bind operator, we should
    free the old one and copy in to a new bigger/smaller location.
    We can't use MEM_recallocN because it doesn't support arrays.
    */
    sdef_layer->num_of_frames++;
    void *temp_frames_pointer = MEM_calloc_arrayN(
        sdef_layer->num_of_frames, sizeof(*sdef_layer->frames), "SDefGPFrames");

    if (temp_frames_pointer == NULL) {
      BKE_gpencil_modifier_set_error((GpencilModifierData *)smd_eval, "Out of memory");
      return false;
    }

    /*if this is the first frame, so num_of_frame has just become one, we don't need to copy
     * anything.*/
    if (sdef_layer->num_of_frames > 1) {
      memcpy(temp_frames_pointer,
             sdef_layer->frames,
             sizeof(*sdef_layer->frames) * (sdef_layer->num_of_frames - 1));
      MEM_SAFE_FREE(sdef_layer->frames);
    }

    sdef_layer->frames = temp_frames_pointer;

    /*First frame changed location with the rellocation of the array.*/
    first_frame = sdef_layer->frames;

    /*Now sdef_layer->frames is on 0. we need to bring it to the new frame position.*/
    
    uint frame_idx = 0;
    sdef_layer->frames->frame_idx = frame_idx;
    for (int f = 0; f < sdef_layer->num_of_frames - 1; f++) {
      sdef_layer->frames->first = first_frame;
      sdef_layer->frames++;
      frame_idx++;
      sdef_layer->frames->frame_idx = frame_idx;
    }

    /*Fill in the frame data*/
    sdef_layer->frames->blender_frame = gpf;
    sdef_layer->frames->frame_number = gpf->framenum; // Not storing it because it could just change
    sdef_layer->frames->strokes_num = BLI_listbase_count(&(gpf->strokes));
    sdef_layer->frames->first = first_frame;

    /*Allocate the strokes*/
    sdef_layer->frames->strokes = MEM_calloc_arrayN(
        sdef_layer->frames->strokes_num, sizeof(*sdef_layer->frames->strokes), "SDefGPStrokes");
    if (sdef_layer->frames->strokes == NULL) {
      BKE_gpencil_modifier_set_error((GpencilModifierData *)smd_eval, "Out of memory");
      return false;
    }

    /*if (gps->prev == NULL){ //gps should have different arrays for different frames, so this should
      work. allocate_stroke_array(BLI_listbase_count(&(gpf->strokes)), smd_orig, smd); printf("first
      bind exec \n"); smd_orig->layers->frames->strokes->stroke_idx = 0;
      }
      else {
        uint old_idx = smd_orig->layers->frames->strokes->stroke_idx;
        (smd_orig->layers->frames->strokes)++; /* increase smd->strokes pointer by 1 *
        smd_orig->layers->frames->strokes->stroke_idx = old_idx + 1;   /*increase stroke idx value*
      }*/

    return true;
}

/*Free a single layer *** UNUSED *** */
static bool free_layer(SurDeformGpencilModifierData *smd_orig,
                       SurDeformGpencilModifierData *smd_eval,
                       char *layer_info)
{
  if (!smd_orig->layers)
    return true;
  /* If we're not on the first layer, rollback the pointer*/
  if (smd_orig->layers->layer_idx > 0) {
    rollback_layers_a(smd_orig);
  }
  SDefGPLayer *first_layer = smd_orig->layers->first;
  /*Do the same thing as add, but in reverse*/
  smd_orig->num_of_layers--;
  SDefGPLayer *temp_layers_pointer = MEM_calloc_arrayN(
      smd_orig->num_of_layers, sizeof(*smd_orig->layers), "SDefGPLayers");

  if (temp_layers_pointer == NULL) {
    BKE_gpencil_modifier_set_error((GpencilModifierData *)smd_eval, "Out of memory");
    return false;
  }

  /*if this was the last layer, so num_of_layers has just become 0, we don't need to copy
   * anything.*/
  if (smd_orig->num_of_layers > 0) {
    /* Copy one layer at a time, except the one we are unbinding*/
    for (int l = 0; l < smd_orig->num_of_layers; l++) {
      /*Not sure if this works as intended, unused func so not tested*/
      if (!strcmp(smd_orig->layers[l].layer_info, layer_info)) {
        if (&smd_orig->layers[l] == first_layer) {
          first_layer = &smd_orig->layers[l + 1];
        }
        l--;
        continue;
      }
      memcpy(&temp_layers_pointer[l], &(smd_orig->layers[l]), sizeof(*smd_orig->layers));
    }
    MEM_SAFE_FREE(smd_orig->layers);
  }

  smd_orig->layers = temp_layers_pointer;
  /*Set the first layer again if the first was the one being removed*/
  if (smd_orig->layers->first != first_layer) {
    for (int l = 0; l < smd_orig->num_of_layers; l++) {
      smd_orig->layers[l].first = first_layer;
    }
  }

  return true;
}

/*Free a single frame*/
static bool free_frame(SurDeformGpencilModifierData *smd_orig,
                       SurDeformGpencilModifierData *smd_eval,
                       SDefGPLayer *sdef_layer,
                       uint framenum)
{
  if (sdef_layer->frames == NULL) return true;
  /* If we're not on the first frame, rollback the pointer*/
  if (sdef_layer->frames->frame_idx > 0) {
    rollback_lframes_a(smd_orig, sdef_layer);
  }
  /*Do the same thing as add, but in reverse */
  sdef_layer->num_of_frames--;
  SDefGPFrame *temp_frames_pointer = MEM_calloc_arrayN(
      sdef_layer->num_of_frames, sizeof(*sdef_layer->frames), "SDefGPFrames");

  if (temp_frames_pointer == NULL) {
    BKE_gpencil_modifier_set_error((GpencilModifierData *)smd_eval, "Out of memory");
    return false;
  }
  SDefGPFrame *first_frame = temp_frames_pointer;
  /*if this was the last frame, so num_of_frame has just become 0, we don't need to copy
   * anything.*/
  if (sdef_layer->num_of_frames > 0) {
    /* Copy one frame at a time, except the one we are unbinding*/
    int g = 0;
    for (int f = 0; f < sdef_layer->num_of_frames; f++) {
      if (sdef_layer->frames[g].frame_number == framenum) {
        g++;
        f--;
        continue;
      }
      memcpy(&temp_frames_pointer[f], &(sdef_layer->frames[g]), sizeof(*sdef_layer->frames));
      g++;
    }
    MEM_SAFE_FREE(sdef_layer->frames);
    sdef_layer->frames = first_frame;
    /*Set the first frame again if the first was the one being removed*/
    if (sdef_layer->frames->first != first_frame) {
      for (int f = 0; f < sdef_layer->num_of_frames; f++) {
        sdef_layer->frames[f].first = first_frame;
      }
    }
  }
  else MEM_SAFE_FREE(sdef_layer->frames);

  

  return true;
}


static bool surfacedeformBind_stroke(uint stroke_idx,
                                             SurDeformGpencilModifierData *smd_orig,
                                             SurDeformGpencilModifierData *smd_eval,
                                             bGPDstroke *gps,
                                             int verts_num,
                                             uint target_verts_num,
                                             Mesh *target,
                                             float (*positions)[3],
                                             MPoly *mpoly,
                                             MEdge *medge,
                                             MLoop *mloop,
                                             BVHTreeFromMesh treeData,
                                             SDefAdjacencyArray *vert_edges,
                                             SDefEdgePolys *edge_polys)
{
  SDefGPStroke *current_stroke = smd_orig->layers->frames->strokes;
  current_stroke->stroke_idx = stroke_idx; /*increase stroke idx value*/
  current_stroke->blender_stroke = gps;
  current_stroke->stroke_verts_num = verts_num;
  current_stroke->verts = MEM_calloc_arrayN(
      verts_num, sizeof(*current_stroke->verts), "SDefBindVerts");
  char error_label[128];
  uint framenum = smd_orig->layers->frames->frame_number;

  if (current_stroke->verts == NULL) {
    BKE_gpencil_modifier_set_error((GpencilModifierData *)smd_eval, "Out of memory");
    return false;
  }

  /* smd_orig->mesh_verts_num = verts_num; */

  /*int defgrp_index;
  MDeformVert *dvert;
  MOD_get_vgroup(ob, mesh, smd_orig->defgrp_name, &dvert, &defgrp_index);
  const bool invert_vgroup = (smd_orig->flags & MOD_SDEF_INVERT_VGROUP) != 0;
  const bool sparse_bind = (smd_orig->flags & MOD_SDEF_SPARSE_BIND) != 0;*/

  SDefBindCalcData data = {
      .treeData = &treeData,
      .vert_edges = vert_edges,
      .edge_polys = edge_polys,
      .mpoly = mpoly,
      .medge = medge,
      .mloop = mloop,
      .looptri = BKE_mesh_runtime_looptri_ensure(target),
      .targetCos = MEM_malloc_arrayN(
          target_verts_num, sizeof(float[3]), "SDefTargetBindVertArray"),
      .current_stroke = current_stroke,
      .gps = gps,
      .falloff = smd_orig->falloff,
      .success = MOD_SDEF_BIND_RESULT_SUCCESS,
      /*.dvert = dvert,
      .defgrp_index = defgrp_index,
      .invert_vgroup = invert_vgroup,
      .sparse_bind = sparse_bind,*/
  };

  if (data.targetCos == NULL) {
    BKE_gpencil_modifier_set_error((GpencilModifierData *)smd_eval, "Out of memory");
    BKE_gpencil_modifier_get_info(27)->freeData((GpencilModifierData *)smd_orig);
    return false;
  }

  invert_m4_m4(data.imat, smd_orig->mat);

  for (int i = 0; i < target_verts_num; i++) {
    mul_v3_m4v3(data.targetCos[i], smd_orig->mat, positions[i]);
  }

  TaskParallelSettings settings;
  BLI_parallel_range_settings_defaults(&settings);
  settings.use_threading = (verts_num > 10000);
  BLI_task_parallel_range(0, verts_num, &data, bindVert, &settings);

  MEM_freeN(data.targetCos);

  /*f (sparse_bind) {
    compactSparseBinds(smd_orig);
  }
  else {*/
  //  current_stroke->stroke_verts_num = verts_num;
  //}

  if (data.success == MOD_SDEF_BIND_RESULT_MEM_ERR) {

    BLI_sprintf(error_label,
                "Layer: %s Frame: %u Stroke: %u \n Out of memory",
                smd_orig->layers->layer_info,
                framenum,
                stroke_idx);
    
  }
  else if (data.success == MOD_SDEF_BIND_RESULT_NONMANY_ERR) {
    BLI_sprintf(error_label,
                "Layer: %s Frame: %u Stroke: %u \n Target has edges with more than two polygons",
                smd_orig->layers->layer_info,
                framenum,
                stroke_idx);
   
  }
  else if (data.success == MOD_SDEF_BIND_RESULT_CONCAVE_ERR) {
    BLI_sprintf(error_label,
                "Layer: %s Frame: %u Stroke: %u \n Target contains concave polygons",
                smd_orig->layers->layer_info,
                framenum,
                stroke_idx);

  }
  else if (data.success == MOD_SDEF_BIND_RESULT_OVERLAP_ERR) {
    BLI_sprintf(error_label,
                "Layer: %s Frame: %u Stroke: %u \n Target contains overlapping vertices",
                smd_orig->layers->layer_info,
                framenum,
                stroke_idx);
  }
  else if (data.success == MOD_SDEF_BIND_RESULT_GENERIC_ERR) {
    /* I know this message is vague, but I could not think of a way
     * to explain this with a reasonably sized message.
     * Though it shouldn't really matter all that much,
     * because this is very unlikely to occur */
    BLI_sprintf(error_label,
                "Layer: %s Frame: %u Stroke: %u \n Target contains invalid polygons",
                smd_orig->layers->layer_info,
                framenum,
                stroke_idx);
  }
  else if (current_stroke->stroke_verts_num == 0 || !current_stroke->verts) {
    data.success = MOD_SDEF_BIND_RESULT_GENERIC_ERR;
    BLI_sprintf(error_label,
                "Layer: %s Frame: %u Stroke: %u \n No vertices were bound",
                smd_orig->layers->layer_info,
                framenum,
                stroke_idx);
  }

  if (data.success != 1) {
    BKE_gpencil_modifier_set_error((GpencilModifierData *)smd_eval, error_label);
  }
  return data.success == 1;
}

static bool makeTreeData(BVHTreeFromMesh *treeData,
                         Mesh *target,
                         SurDeformGpencilModifierData *smd_eval,
                         SurDeformGpencilModifierData *smd_orig,
                         SDefAdjacencyArray *vert_edges,
                         SDefAdjacency * adj_array,
                         SDefEdgePolys * edge_polys,
                         MPoly *mpoly,
                         MEdge *medge,
                         MLoop *mloop,
                         uint target_polys_num,
                         uint target_verts_num,
                         uint tedges_num)
{
  int adj_result;

  /*Empty edge polys array*/
  for (int ep = 0; ep < tedges_num; ep++) {

    edge_polys[ep].polys[0] = 0;
    edge_polys[ep].polys[1] = 0;
    edge_polys[ep].num = 0;
  }

  /*Empty vert edges array*/
  for (int ve = 0; ve < target_verts_num; ve++) {
    vert_edges[ve].first = NULL;
    vert_edges[ve].num = 0;
  }
  BKE_bvhtree_from_mesh_get(treeData, target, BVHTREE_FROM_LOOPTRI, 2);
  if (treeData->tree == NULL) {
    BKE_gpencil_modifier_set_error((GpencilModifierData *)smd_eval, "Out of memory");
    //freeAdjacencyMap(vert_edges, adj_array, edge_polys);
    return false;
  }

  // printf("mPoly: %p, mpoly->totloop: %d", mpoly, mpoly->totloop);

  adj_result = buildAdjacencyMap(
      mpoly, medge, mloop, target_polys_num, tedges_num, vert_edges, adj_array, edge_polys);

  if (adj_result == MOD_SDEF_BIND_RESULT_NONMANY_ERR) {
    BKE_gpencil_modifier_set_error((GpencilModifierData *)smd_eval,
                                   "Target has edges with more than two polygons");
    //freeAdjacencyMap(vert_edges, adj_array, edge_polys);
    //free_bvhtree_from_mesh(&treeData);
    return false;
  }
  return true;
}


static bool surfacedeformBind(bContext *C,
                              Object *ob,
                              Depsgraph *depsgraph,
                              SurDeformGpencilModifierData *smd_orig,
                              SurDeformGpencilModifierData *smd_eval,
                              uint target_polys_num,
                              uint target_verts_num,
                              Mesh *target)
{
  BVHTreeFromMesh treeData = {NULL};
  Object *ob_orig = (Object *)DEG_get_original_id(&ob->id);
  bGPdata *gpd = ob_orig->data;
  Scene *scene = DEG_get_evaluated_scene(depsgraph);
  bGPDlayer *gpl_active = BKE_gpencil_layer_active_get(gpd);
  Object *ob_target_orig = smd_orig->target;
  Object *ob_target_eval;
  Mesh *mesh_target;
  Depsgraph *dg =CTX_data_depsgraph_pointer(C);

  /*If unbind mode: unbind and exit */
  if (smd_orig->bind_modes & GP_MOD_SDEF_UNBIND_MODE) 
  {
    if (!smd_orig->layers) {
      BKE_gpencil_modifier_set_error((GpencilModifierData *)smd_eval, "No layer data to unbind");
    }
    rollback_layers_a(smd_orig);
    
    /*free one frame or all the frames?*/
    if (smd_orig->bind_modes & GP_MOD_SDEF_BIND_ALL_FRAMES) 
    {
      /*Just free al of the data*/
      freeData_a(smd_orig);

      /*Set all bound keyframes back to normal*/
      LISTBASE_FOREACH (bGPDlayer *, curr_gpl, &gpd->layers) {
        LISTBASE_FOREACH (bGPDframe *, curr_gpf, &curr_gpl->frames) {
          if (curr_gpf->key_type == BEZT_KEYTYPE_SURDEFBOUND) {
            curr_gpf->key_type = BEZT_KEYTYPE_KEYFRAME;
          }
        }
      }
      
    }
    else {
      bGPDframe *curr_gpf = BKE_gpencil_frame_retime_get(depsgraph, scene, ob, gpl_active);
          uint framenum = curr_gpf->framenum;
      for (int l = 0; l < smd_orig->num_of_layers; l++) {
        /*EXIT IF FRAME IS NOT ALREADY BOUND*/
        for (int f = 0; f < smd_orig->layers[l].num_of_frames; f++) {
          if (smd_orig->layers[l].frames[f].frame_number == framenum)
          {
            free_frame(smd_orig, smd_eval, &(smd_orig->layers[l]), framenum);
            curr_gpf->key_type = BEZT_KEYTYPE_KEYFRAME;
            break;
          }
          curr_gpf->key_type = BEZT_KEYTYPE_KEYFRAME;
          /*If last frame of layer was removed, free the layer data (todo) 
          if (!smd_orig->layers[l].frames[f]) {

          }*/
        }
        
      }
    }
    rollback_layers_a(smd_orig);
    for (int l = 0; l < smd_orig->num_of_layers; l++) {
      if (smd_orig->layers[l].frames != NULL) {
        rollback_lframes_a(smd_orig, &smd_orig->layers[l]);
      }
    }
    return true;  // TODO: checks for operation successful
  }

  /*Bind mode*/


   float(*positions);
   MPoly *mpoly;
   MEdge *medge;
   MLoop *mloop;
  uint tedges_num = target->totedge;
  // uint current_stroke_idx = smd_orig->current_stroke_index;
  SDefAdjacencyArray *vert_edges;
  SDefAdjacency *adj_array;
  SDefEdgePolys *edge_polys;

  // printf("target->mpoly: %p, target->mpoly->totloop: %d", target->mpoly,
  // target->mpoly->totloop); printf("mPoly: %p, mpoly->totloop: %d", mpoly, mpoly->totloop);

  vert_edges = MEM_calloc_arrayN(target_verts_num, sizeof(*vert_edges), "SDefVertEdgeMap");
  if (vert_edges == NULL) {
    BKE_gpencil_modifier_set_error((GpencilModifierData *)smd_eval, "Out of memory");
    return false;
  }

  adj_array = MEM_malloc_arrayN(tedges_num, 2 * sizeof(*adj_array), "SDefVertEdge");
  if (adj_array == NULL) {
    BKE_gpencil_modifier_set_error((GpencilModifierData *)smd_eval, "Out of memory");
    MEM_freeN(vert_edges);
    return false;
  }

  edge_polys = MEM_calloc_arrayN(tedges_num, sizeof(*edge_polys), "SDefEdgeFaceMap");
  if (edge_polys == NULL) {
    BKE_gpencil_modifier_set_error((GpencilModifierData *)smd_eval, "Out of memory");
    MEM_freeN(vert_edges);
    MEM_freeN(adj_array);
    return false;
  }

  /*THERE WAS ALLOCATION OF STROKE VETS ARRAY HERE.*/

  

  smd_orig->target_verts_num = target_verts_num;
  smd_orig->target_polys_num = target_polys_num;
  float current_frame = scene->r.cfra;

 
  LISTBASE_FOREACH (bGPDlayer *, curr_gpl, &gpd->layers)
  {
    
    /* If a layer is already bound, skip it.*/
    rollback_layers_a(smd_orig);
    int l = 0;
    bool resutl = 1;
    while (l != smd_orig->num_of_layers && resutl != 0) {
      resutl = strcmp(smd_orig->layers[l].layer_info, curr_gpl->info);
      if (resutl)
        ++l;
    }
    if (l != smd_orig->num_of_layers) {
      smd_orig->layers = &smd_orig->layers[l];
    }
    else
    {
      add_layer(smd_orig, smd_eval, curr_gpl);
      /* Now smd_orig->layers should point to the new layer. */
    }

    
    

    if (smd_orig->bind_modes & GP_MOD_SDEF_BIND_ALL_FRAMES) {
      LISTBASE_FOREACH (bGPDframe *, curr_gpf, &curr_gpl->frames) {

        rollback_lframes_a(smd_orig, smd_orig->layers);
        

        /* If a frame is already bound, skip it.*/
        int f = 0;
        while (f != smd_orig->layers->num_of_frames &&
                smd_orig->layers->frames[f].frame_number != curr_gpf->framenum)
          ++f;  
        if (f != smd_orig->layers->num_of_frames) {
          continue;
        }
        add_frame(smd_orig, smd_eval, smd_orig->layers, curr_gpf);
        smd_orig->flags |= GP_MOD_SDEF_WITHHOLD_EVALUATION;
        smd_eval->flags |= GP_MOD_SDEF_WITHHOLD_EVALUATION;

        dg = CTX_data_depsgraph_pointer(C);
        scene = CTX_data_scene(C);
        BKE_scene_frame_set(scene, (float)curr_gpf->framenum);
        scene->r.cfra = curr_gpf->framenum;
        BKE_scene_graph_update_for_newframe(dg);
        ob_target_eval = DEG_get_evaluated_object(dg, ob_target_orig);
        mesh_target = BKE_modifier_get_evaluated_mesh_from_evaluated_object(ob_target_eval);
        positions = BKE_mesh_vert_positions(mesh_target);  //  FOR 3.4 +
        mpoly = BKE_mesh_polys(mesh_target);
        medge = BKE_mesh_edges(mesh_target);
        mloop = BKE_mesh_loops(mesh_target);
        tedges_num = mesh_target->totedge;
        if (!makeTreeData(&treeData,
                          mesh_target,
                          smd_eval,
                          smd_orig,
                          vert_edges,
                          adj_array,
                          edge_polys,
                          mpoly,
                          medge,
                          mloop,
                          target_polys_num,
                          target_verts_num,
                          tedges_num))
          continue;
        smd_orig->flags &= ~GP_MOD_SDEF_WITHHOLD_EVALUATION;
        smd_eval->flags &= ~GP_MOD_SDEF_WITHHOLD_EVALUATION;
        if (BLI_listbase_count(&smd_orig->layers->frames->blender_frame->strokes) == 0) {
          smd_orig->layers->frames->strokes_num = 0;
          smd_orig->layers->frames->strokes = NULL;
        }
        uint s = 0;
        uint stroke_error = 0;
        SDefGPStroke *first_stroke; 
        LISTBASE_FOREACH (bGPDstroke *, curr_gps, &curr_gpf->strokes) {
          if (stroke_error)
            continue;
          if (!surfacedeformBind_stroke(s,
                                        smd_orig,
                                        smd_eval,
                                        curr_gps,
                                        curr_gps->totpoints,
                                        target_verts_num,
                                        mesh_target,
                                        positions,
                                        mpoly,
                                        medge,
                                        mloop,
                                        treeData,
                                        vert_edges,
                                        edge_polys)) {
            stroke_error = 1;
            continue;
          }
          if (s == 0) {
            first_stroke = smd_orig->layers->frames->strokes;
          }
          smd_orig->layers->frames->strokes->first = first_stroke; //meglio metterlo nella funzione surfacedeformBind_stroke? Boh 
          s++;
          smd_orig->layers->frames->strokes++;
        }
        if (stroke_error) {
          free_frame(smd_orig, smd_eval, smd_orig->layers, curr_gpf);
          continue;
        }
        /*Stroke pointer is one place beyond end of array, idk if I should have prevented this somehow,
        dirty memory so can't use rollback strokes function. Bring it back to first_stroke we conveniently recorded earler. */
        if (smd_orig->layers->frames->strokes != NULL) {
          smd_orig->layers->frames->strokes = first_stroke;
        }
        free_bvhtree_from_mesh(&treeData);
        curr_gpf->key_type = BEZT_KEYTYPE_SURDEFBOUND;
        //freeAdjacencyMap(vert_edges, adj_array, edge_polys);
      }
      BKE_scene_frame_set(scene, current_frame);
      BKE_scene_graph_update_for_newframe(dg);
    }
    else {
      positions = BKE_mesh_vert_positions(target);  //  FOR 3.4 +
      mpoly = BKE_mesh_polys(target);
      medge = BKE_mesh_edges(target);
      mloop = BKE_mesh_loops(target);
      if (!makeTreeData(&treeData,
                        target,
                        smd_eval,
                        smd_orig,
                        vert_edges,
                        adj_array,
                        edge_polys,
                        mpoly,
                        medge,
                        mloop,
                        target_polys_num,
                        target_verts_num,
                        tedges_num))
        continue;
      /* If a frame is already bound, skip it.*/
      rollback_lframes_a(smd_orig, smd_orig->layers);
      int f = 0;
      bGPDframe *curr_frame = BKE_gpencil_frame_retime_get(
                              depsgraph, scene, ob, curr_gpl);
      while (f != smd_orig->layers->num_of_frames &&
             smd_orig->layers->frames[f].frame_number != curr_frame->framenum
                  )
        ++f;  // credits for this line: "Vlad from Moscow " from stack overflow. This is so
              // fricking smart I can't
      if (f == smd_orig->layers->num_of_frames) {
        add_frame(
            smd_orig,
            smd_eval,
            smd_orig->layers, curr_frame);
        uint s = 0;
        if (BLI_listbase_count(&curr_frame->strokes) == 0) {
          smd_orig->layers->frames->strokes_num = 0;
          smd_orig->layers->frames->strokes = NULL;
        }
        uint stroke_error = 0;
        SDefGPStroke *first_stroke; 
        LISTBASE_FOREACH (bGPDstroke *, curr_gps, &curr_frame->strokes) {
          if (stroke_error)
            continue;
          if (!surfacedeformBind_stroke(s,
                                        smd_orig,
                                        smd_eval,
                                        curr_gps,
                                        curr_gps->totpoints,
                                        target_verts_num,
                                        target,
                                        positions,
                                        mpoly,
                                        medge,
                                        mloop,
                                        treeData,
                                        vert_edges,
                                        edge_polys)) {
            
            stroke_error = 1;
            continue;
          }
          if (s == 0) {
            first_stroke = smd_orig->layers->frames->strokes;
          }
          smd_orig->layers->frames->strokes->first = first_stroke;
          s++;
          smd_orig->layers->frames->strokes++;
        }
        if (stroke_error) {
          free_frame(smd_orig, smd_eval, smd_orig->layers, curr_frame);
          continue;
        }
        /*Stroke pointer is one place beyond end of array, idk if I should have prevented this
        somehow, dirty memory so can't use rollback strokes function. Bring it back to first_stroke
        we conveniently recorded earler. */
        if (smd_orig->layers->frames->strokes != NULL) {
          smd_orig->layers->frames->strokes = first_stroke;
        }

        curr_frame->key_type = BEZT_KEYTYPE_SURDEFBOUND;
      }
      //free_bvhtree_from_mesh(&treeData);
      //freeAdjacencyMap(vert_edges, adj_array, edge_polys);
    }
    rollback_lframes_a(smd_orig, smd_orig->layers);
    
  }
  
 
  free_bvhtree_from_mesh(&treeData);
  freeAdjacencyMap(vert_edges, adj_array, edge_polys);
  
  rollback_layers_a(smd_orig);
  return true;
}


static GpencilModifierData *gpencil_edit_modifier_property_get(wmOperator *op,
                                                                       Object *ob,
                                                                       int type)
{
  if (ob == NULL) {
    return NULL;
  }

  char modifier_name[MAX_NAME];
  GpencilModifierData *md;
  RNA_string_get(op->ptr, "modifier", modifier_name);

  md = BKE_gpencil_modifiers_findby_name(ob, modifier_name);

  if (md && type != 0 && md->type != type) {
    md = NULL;
  }

  return md;
}

/* ------------------------------------------------------------------- */
        /** \name Surface Deform Bind Operator
         * \{ */

static void gpencil_edit_modifier_properties(wmOperatorType *ot)
{
  PropertyRNA *prop = RNA_def_string(
      ot->srna, "modifier", NULL, MAX_NAME, "Modifier", "Name of the modifier to edit");
  RNA_def_property_flag(prop, PROP_HIDDEN);
}

static bool gpencil_edit_modifier_invoke_properties(bContext *C,
                                                    wmOperator *op,
                                                    const wmEvent *event,
                                                    int *r_retval)
{
  if (RNA_struct_property_is_set(op->ptr, "modifier")) {
    return true;
  }

  PointerRNA ctx_ptr = CTX_data_pointer_get_type(C, "modifier", &RNA_GpencilModifier);
  if (ctx_ptr.data != NULL) {
    GpencilModifierData *md = ctx_ptr.data;
    RNA_string_set(op->ptr, "modifier", md->name);
    return true;
  }

  /* Check the custom data of panels under the mouse for a modifier. */
  if (event != NULL) {
    PointerRNA *panel_ptr = UI_region_panel_custom_data_under_cursor(C, event);

    if (!(panel_ptr == NULL || RNA_pointer_is_null(panel_ptr))) {
      if (RNA_struct_is_a(panel_ptr->type, &RNA_GpencilModifier)) {
        GpencilModifierData *md = panel_ptr->data;
        RNA_string_set(op->ptr, "modifier", md->name);
        return true;
      }

      BLI_assert(r_retval != NULL); /* We need the return value in this case. */
      if (r_retval != NULL) {
        *r_retval = (OPERATOR_PASS_THROUGH | OPERATOR_CANCELLED);
      }
      return false;
    }
  }

  if (r_retval != NULL) {
    *r_retval = OPERATOR_CANCELLED;
  }
  return false;
}



/*static bool gpencil_surfacedeform_bind_poll(bContext *C)
{
  return gpencil_edit_modifier_poll_generic(C, &RNA_SurDeformGpencilModifier, 0, true);
} */


static int gpencil_surfacedeform_bind_or_unbind(bContext *C, wmOperator *op)
{
  Object *ob = ED_object_active_context(C);
  Depsgraph *depsgraph = CTX_data_ensure_evaluated_depsgraph(C);
  SurDeformGpencilModifierData *smd_orig = (SurDeformGpencilModifierData *)
      gpencil_edit_modifier_property_get(op, ob, eGpencilModifierType_SurDeform);
  GpencilModifierData *md = &(smd_orig->modifier);
  GpencilModifierData *md_eval;

  Object *object_eval = DEG_get_evaluated_object(depsgraph, ob);

  PointerRNA ob_ptr;
 // RNA_pointer_create(&ob->id, &RNA_GpencilModifier, smd_orig, &ob_ptr);
  const bool current_frame_only = (RNA_enum_get(op->ptr, "curr_frame_or_all_frames") == (1 << 0));
 
  if (smd_orig == NULL) {
    return OPERATOR_CANCELLED;
  }

  if (smd_orig->target) {
    smd_orig->flags |= GP_MOD_SDEF_DO_BIND;
  }


  if (current_frame_only) {
    smd_orig->bind_modes |= GP_MOD_SDEF_BIND_CURRENT_FRAME;
    smd_orig->bind_modes &= ~GP_MOD_SDEF_BIND_ALL_FRAMES;
  }
  else {
    smd_orig->bind_modes &= ~GP_MOD_SDEF_BIND_CURRENT_FRAME;
    smd_orig->bind_modes |= GP_MOD_SDEF_BIND_ALL_FRAMES;
  }

 
  smd_orig->bind_modes &= ~GP_MOD_SDEF_BIND_CURRENT_LAYER;
   

  /*SurDeformGpencilModifierData *smd_eval = (SurDeformGpencilModifierData
  *)BKE_modifier_get_evaluated( depsgraph, ob, &smd->modifier); smd_eval->flags = smd->flags;*/

  /* Update flags to evaluated modifier */
  SurDeformGpencilModifierData *smd_eval;

  if (object_eval == ob) {
    smd_eval = smd_orig;
    md_eval = md;
  }
  else {

    md_eval = BKE_gpencil_modifiers_findby_name(object_eval, md->name);
    smd_eval = (SurDeformGpencilModifierData *)md_eval;
  }

  smd_eval->flags = smd_orig->flags;

  /* START OUSIDE EVALUATION CODE */
  Object *ob_target = DEG_get_evaluated_object(depsgraph, smd_orig->target);
  Mesh *target = BKE_modifier_get_evaluated_mesh_from_evaluated_object(ob_target);
  if (!DEG_is_active(depsgraph)) {
    BKE_gpencil_modifier_set_error(md, "Attempt to unbind from inactive dependency graph");
    return OPERATOR_CANCELLED;
  }

  float tmp_mat[4][4];

  invert_m4_m4(tmp_mat, ob->object_to_world);
  mul_m4_m4m4(smd_orig->mat, tmp_mat, ob_target->object_to_world);

  /* Avoid converting edit-mesh data, binding is an exception. */
  BKE_mesh_wrapper_ensure_mdata(target);
  
  if (!target) {
    BKE_gpencil_modifier_set_error(md, "No valid target mesh");
    return OPERATOR_CANCELLED;
  }
  uint target_verts_num = BKE_mesh_wrapper_vert_len(target);
  uint target_polys_num = BKE_mesh_wrapper_poly_len(target);
  //Mesh *target = <

  if (!surfacedeformBind(C,
                         ob,
                         depsgraph,
                         smd_orig,
                         smd_eval,
                         target_polys_num,
                         target_verts_num,
                         target)) {
    printf("bind failed \n");
    smd_orig->bound_flags = 0;
    freeData_a(smd_orig);
  }
  else
  {
    printf("bind good \n");
  }

  if (md_eval->error) {
    BKE_report(op->reports, RPT_ERROR, md_eval->error);  // report the error
  }
  
  smd_orig->flags &= ~GP_MOD_SDEF_DO_BIND;
  /* END OUSIDE EVALUATION CODE */

  DEG_id_tag_update(&ob->id, ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, ob);

  smd_orig->flags &= ~GP_MOD_SDEF_WITHHOLD_EVALUATION;

  /*printf("smd flag 3 %i\n", smd->flags);
  printf("smd_eval flag 3 %i\n", smd_eval->flags);*/
  smd_orig->bound_flags = 0;
  return OPERATOR_FINISHED;
}


/*  BAKE */

static int bake_frames(bContext *C, wmOperator *op)
{
  Object *ob = ED_object_active_context(C);
  bGPdata *gpd = ob->data;
  Depsgraph *depsgraph = CTX_data_ensure_evaluated_depsgraph(C);
  SurDeformGpencilModifierData *smd_orig = (SurDeformGpencilModifierData *)
      gpencil_edit_modifier_property_get(op, ob, eGpencilModifierType_SurDeform);
  GpencilModifierData *md = &(smd_orig->modifier);
  GpencilModifierData *md_eval;
  const GpencilModifierTypeInfo *mti = BKE_gpencil_modifier_get_info(md->type);
  Scene *scene = DEG_get_evaluated_scene(depsgraph);
  Object *object_eval = DEG_get_evaluated_object(depsgraph, ob);
  bGPdata *gpd_eval = object_eval->data;
  bGPDlayer *gpl_eval;
  PointerRNA ob_ptr;
  bGPDframe *gpf;
  bGPDframe *gpf_eval;
  bGPDstroke *gps_new;

  smd_orig->bake_range_start = RNA_int_get( op->ptr, "frame_start");
  smd_orig->bake_range_end = RNA_int_get(op->ptr, "frame_end");

  int frame_start = smd_orig->bake_range_start;
  int frame_end = smd_orig->bake_range_end;

  /*Iterate the frames in the range*/


  for (int frame = frame_start; frame <= frame_end; frame++) {
    smd_orig->flags |= GP_MOD_SDEF_WITHHOLD_EVALUATION;
    BKE_scene_frame_set(scene, frame);
    BKE_scene_graph_update_for_newframe(depsgraph);
    smd_orig->flags &= ~GP_MOD_SDEF_WITHHOLD_EVALUATION;
    /*Iterate all the layers*/
    int l = 0;
    LISTBASE_FOREACH (bGPDlayer *, gpl, &gpd->layers) {
      /*What if the user decides to bake a bound/existing frame? It must be skipped.*/
      gpf = BKE_gpencil_layer_frame_get(gpl, frame, GP_GETFRAME_USE_PREV);
      if (gpf->framenum == frame)
        continue;
      /* get the current state in current frame (orig)*/
      gpf = BKE_gpencil_layer_frame_get(gpl, frame, GP_GETFRAME_ADD_NEW);
      /*eval*/
      gpl_eval = BLI_rfindlink(&gpd_eval->layers, l);
      gpf_eval = BKE_gpencil_layer_frame_get(gpl_eval, frame, GP_GETFRAME_USE_PREV);
      /*All the properties of the second frame were copied; including the changed color!
      Let's set it back to normal, cuz the baked frame must not be or appear bound.
      gpf->key_type = BEZT_KEYTYPE_KEYFRAME;*/
      LISTBASE_FOREACH (bGPDstroke *, gps, &gpf_eval->strokes)
      {
        gps_new = BKE_gpencil_stroke_duplicate(gps, true, true);
        BLI_addtail(&gpf->strokes, gps_new);
        //mti->deformStroke(md, depsgraph, ob, gpl, gpf, gps);
      }
      l++;
      rollback_lframes_a(smd_orig, smd_orig->layers);
    }
    rollback_layers_a(smd_orig);
  }

  DEG_id_tag_update(&gpd->id, ID_RECALC_GEOMETRY | ID_RECALC_ANIMATION | ID_RECALC_COPY_ON_WRITE);
  return OPERATOR_FINISHED;
}

static int gpsurdef_fill_range(bContext *C, wmOperator *op)
{
  Scene *scene = CTX_data_scene(C);
  Object *ob = ED_object_active_context(C);
  SurDeformGpencilModifierData *smd_orig = (SurDeformGpencilModifierData *)
      gpencil_edit_modifier_property_get(op, ob, eGpencilModifierType_SurDeform);
 
  smd_orig->bake_range_start = scene->r.sfra;
  smd_orig->bake_range_end = scene->r.efra;

  return OPERATOR_FINISHED;
}


/* OPERATORS */

static int gpencil_surfacedeform_bake_exec(bContext *C, wmOperator *op)
{
  Object *ob = ED_object_active_context(C);
  /* set notifiers */
  WM_event_add_notifier(C, NC_GPENCIL | ND_DATA | NA_EDITED, ob);
  WM_event_add_notifier(C, NC_ANIMATION | ND_KEYFRAME | NA_EDITED, NULL);
  
  return bake_frames(C, op);
}

static int gpencil_surfacedeform_bind_exec(bContext *C, wmOperator *op)
{
  Object *ob = ED_object_active_context(C);
  SurDeformGpencilModifierData *smd_orig = (SurDeformGpencilModifierData *)
      gpencil_edit_modifier_property_get(op, ob, eGpencilModifierType_SurDeform);
  smd_orig->bind_modes &= ~GP_MOD_SDEF_UNBIND_MODE;

  /* set notifier that keyframe properties have changed */
  WM_event_add_notifier(C, NC_ANIMATION | ND_KEYFRAME_PROP, NULL);

  return gpencil_surfacedeform_bind_or_unbind(C,  op);
}

static int gpencil_surfacedeform_unbind_exec(bContext *C, wmOperator *op)
{
  Object *ob = ED_object_active_context(C);
  SurDeformGpencilModifierData *smd_orig = (SurDeformGpencilModifierData *)
      gpencil_edit_modifier_property_get(op, ob, eGpencilModifierType_SurDeform);
  smd_orig->bind_modes |= GP_MOD_SDEF_UNBIND_MODE;

  /* set notifier that keyframe properties have changed */
  WM_event_add_notifier(C, NC_ANIMATION | ND_KEYFRAME_PROP, NULL);

  return gpencil_surfacedeform_bind_or_unbind(C,  op);
}

static int gpencil_surfacedeform_fillrange_exec(bContext *C, wmOperator *op)
{
  return gpsurdef_fill_range(C, op);
}


static int gpencil_surfacedeform_bind_invoke(bContext *C,
                                             wmOperator *op,
                                             const wmEvent *UNUSED(event))
{
  if (gpencil_edit_modifier_invoke_properties(C, op, NULL, NULL)) {
    return gpencil_surfacedeform_bind_exec(C, op);
  }
  return OPERATOR_CANCELLED;
}

static int gpencil_surfacedeform_unbind_invoke(bContext *C,
                                             wmOperator *op,
                                             const wmEvent *UNUSED(event))
{
  if (gpencil_edit_modifier_invoke_properties(C, op, NULL, NULL)) {
    return gpencil_surfacedeform_unbind_exec(C, op);
  }
  return OPERATOR_CANCELLED;
}

static int gpencil_surfacedeform_bake_invoke(bContext *C,
                                               wmOperator *op,
                                               const wmEvent *UNUSED(event))
{
  if (gpencil_edit_modifier_invoke_properties(C, op, NULL, NULL)) {
    return gpencil_surfacedeform_bake_exec(C, op);
  }
  return OPERATOR_CANCELLED;
}

static int gpencil_surfacedeform_fillrange_invoke(bContext *C,
                                             wmOperator *op,
                                             const wmEvent *UNUSED(event))
{
  if (gpencil_edit_modifier_invoke_properties(C, op, NULL, NULL)) {
    return gpencil_surfacedeform_fillrange_exec(C, op);
  }
  return OPERATOR_CANCELLED;
}

static const EnumPropertyItem gpsurdef_curr_frame_or_all_frames_items[] = {
  {GP_MOD_SDEF_BIND_CURRENT_FRAME, "CURR_FRAME", 0, "Current Frame", "Bind the current frame"},
  {GP_MOD_SDEF_BIND_ALL_FRAMES, "ALL_FRAMES", 0, "All Frames", "Bind all the frames in the layer(s)"},
  {0, NULL, 0, NULL, NULL},
};

static void bind_unbind_rna_props(wmOperatorType *ot)
{
  RNA_def_enum(ot->srna, "curr_frame_or_all_frames", 
              &gpsurdef_curr_frame_or_all_frames_items,
              0, "", "");
  RNA_def_boolean(ot->srna, "unbind_mode", true, "Unbind", "");
}

void GPENCIL_OT_gpencilsurdeform_bind(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Surface Deform Bind";
  ot->description = "Bind grease pencil points to mesh target in surface deform modifier";
  ot->idname = "GPENCIL_OT_gpencilsurdeform_bind";

  /* api callbacks */
 // ot->poll = gpencil_surfacedeform_bind_poll;
  ot->invoke = gpencil_surfacedeform_bind_invoke;
  ot->exec = gpencil_surfacedeform_bind_exec;

  /* parameters */
  // RNA_def_boolean(ot->srna, "current_frame_only", true, "Only current frame", "");
  bind_unbind_rna_props(ot);

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_INTERNAL;
  gpencil_edit_modifier_properties(ot);
}

void GPENCIL_OT_gpencilsurdeform_unbind(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Surface Deform Unbind";
  ot->description = "Unbind a GP with a surface deform modifier from its mesh";
  ot->idname = "GPENCIL_OT_gpencilsurdeform_unbind";

  /* api callbacks */
 // ot->poll = gpencil_surfacedeform_bind_poll;
  ot->invoke = gpencil_surfacedeform_unbind_invoke;
  ot->exec = gpencil_surfacedeform_unbind_exec;

  /* parameters */
  // RNA_def_boolean(ot->srna, "current_frame_only", true, "Only current frame", "");
  // RNA_def_boolean(ot->srna, "current_layer_only", true, "Only current layer", "");
 // RNA_def_boolean(ot->srna, "unbind_mode", true, "Unbind", "");
  bind_unbind_rna_props(ot);

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_INTERNAL;
  gpencil_edit_modifier_properties(ot);
}

void GPENCIL_OT_gpencilsurdeform_bake(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Surface Deform Bake";
  ot->description = "Bake the chosen frame range. The frames are removed from the modifier's memory";
  ot->idname = "GPENCIL_OT_gpencilsurdeform_bake";

  /* api callbacks */
  // ot->poll = gpencil_surfacedeform_bind_poll;
  ot->invoke = gpencil_surfacedeform_bake_invoke;
  ot->exec = gpencil_surfacedeform_bake_exec;

  /* parameters */
  RNA_def_int(ot->srna, "frame_start", 0, INT_MIN, INT_MAX, "Frame Start", "", INT_MIN, INT_MAX);
  RNA_def_int(ot->srna, "frame_end", 0, INT_MIN, INT_MAX, "Frame End", "", INT_MIN, INT_MAX);

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_INTERNAL;
  gpencil_edit_modifier_properties(ot);
}

void GPENCIL_OT_gpencilsurdeform_fillrange(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Fill with Current Timeline Range";
  ot->description =
      "Fill the start frame and end frame of the bake range of frames with the current Scene's timeline start and end frames";
  ot->idname = "GPENCIL_OT_gpencilsurdeform_fillrange";

  /* api callbacks */
  // ot->poll = gpencil_surfacedeform_bind_poll;
  ot->invoke = gpencil_surfacedeform_fillrange_invoke;
  ot->exec = gpencil_surfacedeform_fillrange_exec;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_INTERNAL;
  gpencil_edit_modifier_properties(ot);
}

void WM_operatortypes_gpencilsurdeform(void)
{
  WM_operatortype_append(GPENCIL_OT_gpencilsurdeform_bind);
  WM_operatortype_append(GPENCIL_OT_gpencilsurdeform_unbind);
  WM_operatortype_append(GPENCIL_OT_gpencilsurdeform_bake);
  WM_operatortype_append(GPENCIL_OT_gpencilsurdeform_fillrange);
}


