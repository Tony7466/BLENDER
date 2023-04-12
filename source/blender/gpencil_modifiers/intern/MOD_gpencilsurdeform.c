

/** \file
 * \ingroup modifiers
 */

#include <stdio.h>

#include "BLI_listbase.h"
#include "BLI_math_geom.h"
#include "BLI_math_vector.h"
#include "BLI_utildefines.h"

#include "BLT_translation.h"

#include "DNA_defaults.h"
#include "DNA_gpencil_modifier_types.h"
#include "DNA_gpencil_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_screen_types.h"
#include "BKE_scene.h"
#include "BKE_context.h"
#include "BKE_deform.h"
#include "BKE_gpencil.h"
#include "BKE_gpencil_geom.h"
#include "BKE_gpencil_modifier.h"
#include "BKE_lib_query.h"
#include "BKE_modifier.h"
#include "BKE_screen.h"

#include "DEG_depsgraph.h"
#include "DEG_depsgraph_build.h"
#include "DEG_depsgraph_query.h"

#include "UI_interface.h"
#include "UI_resources.h"

#include "RNA_access.h"

#include "MOD_gpencil_modifiertypes.h"
#include "MOD_gpencil_ui_common.h"
#include "MOD_gpencil_util.h"

#include "MEM_guardedalloc.h"

/* HEADER FROM MOD_surfacedeform.c */

#include "BLI_alloca.h"
#include "BLI_math.h"
#include "BLI_math_geom.h"
#include "BLI_task.h"

#include "BLT_translation.h"

#include "DNA_defaults.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_screen_types.h"

#include "BKE_bvhutils.h"
#include "BKE_context.h"
#include "BKE_deform.h"
#include "BKE_editmesh.h"
#include "BKE_lib_id.h"
#include "BKE_lib_query.h"
#include "BKE_mesh.h"
#include "BKE_mesh_runtime.h"
#include "BKE_mesh_wrapper.h"
#include "BKE_modifier.h"
#include "BKE_screen.h"

#include "UI_interface.h"
#include "UI_resources.h"

/*#include "BLO_read_write.h"*/

#include "RNA_access.h"
#include "RNA_prototypes.h"

#include "DEG_depsgraph.h"
#include "DEG_depsgraph_query.h"

#include "MEM_guardedalloc.h"


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
  SDefGPStroke *const current_stroke; //formerly bind_verts
  const MLoopTri *const looptri;
  const MPoly *const mpoly;
  const MEdge *const medge;
  const MLoop *const mloop;
  /** Coordinates to bind to, transformed into local space (compatible with `vertexCos`). */
  float (*const targetCos)[3];
  /** Coordinates to bind (reference to the modifiers input argument).
   * Coordinates are stored in the grease pencil vert for grease pencil, which is accessed from the stroke, also
   * keeps the referenc from the deformstroke inut argument.
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

void rollback_layers(SurDeformGpencilModifierData *smd)
{
  if (smd->layers == NULL) return;
  for (int l = 0; l < smd->num_of_layers; l++)
  {
      if (smd->layers->layer_idx == 0)
      {
        return;
      }
      else
      {
        smd->layers--;
      }
  }

}

static void rollback_frames(SurDeformGpencilModifierData *smd, SDefGPLayer *layer)
{
  if (layer->frames == NULL) return;
  for (int f = 0; f < layer->num_of_frames; f++)
  {
      if (layer->frames->frame_idx == 0)
      {
        return;
      }
      else
      {
        layer->frames--;
      }

  }
}

static void rollback_strokes(SurDeformGpencilModifierData *smd, SDefGPFrame *frame)
{
  
  if (frame->strokes == NULL) return;
  for (int s = 0; s< frame->strokes_num; s++)
  {
      if (frame->strokes->stroke_idx == 0)
      {
        return;
      }
      else
      {
        frame->strokes--;
      }

  }
}
struct SurDeformGpencilModifierData *get_original_modifier(Object *ob, SurDeformGpencilModifierData *smd, GpencilModifierData *md)
{
  Object *object_orig = DEG_get_original_object(ob);
  SurDeformGpencilModifierData *smd_orig;
  GpencilModifierData *md_orig;
  

  if (object_orig == ob) {
    smd_orig = smd;
    md_orig = md;  }
  else {
  /*smd_orig = (SurDeformGpencilModifierData *)BKE_gpencil_modifiers_findby_name(object_orig, md->name);}*/
  md_orig = BKE_gpencil_modifiers_findby_name(object_orig, md->name);
  smd_orig = (SurDeformGpencilModifierData *)md_orig;}

  return smd_orig;
}




/* NEW UTILS END */

static void initData(GpencilModifierData *md)
{
  SurDeformGpencilModifierData *smd = (SurDeformGpencilModifierData *)md;

  BLI_assert(MEMCMP_STRUCT_AFTER_IS_ZERO(smd, modifier));

  MEMCPY_STRUCT_AFTER(smd, DNA_struct_default_get(SurDeformGpencilModifierData), modifier);

}

static void freeData(GpencilModifierData *md)
{

  SurDeformGpencilModifierData *smd = (SurDeformGpencilModifierData *)md;
  if (smd->layers)
  {
    rollback_layers(smd);
    for (int l= 0; l < smd->num_of_layers; l++)
    {
      if (smd->layers[l].frames)
      {
        rollback_frames(smd, &smd->layers[l]);
        for(int m= 0; m < smd->layers[l].num_of_frames; m++)
        {
          if (smd->layers[l].frames[m].strokes) 
          {
            rollback_strokes(smd, &smd->layers[l].frames[m] );
            for (int k= 0; k < smd->layers[l].frames[m].strokes_num; k++){
              if (smd->layers[l].frames[m].strokes[k].verts) {
                for (int i = 0; i < smd->layers[l].frames[m].strokes[k].stroke_verts_num; i++) {
                  if (smd->layers[l].frames[m].strokes[k].verts[i].binds) {
                    for (int j = 0; j < smd->layers[l].frames[m].strokes[k].verts[i].binds_num; j++) {
                      MEM_SAFE_FREE(smd->layers[l].frames[m].strokes[k].verts[i].binds[j].vert_inds);
                      MEM_SAFE_FREE(smd->layers[l].frames[m].strokes[k].verts[i].binds[j].vert_weights);
                    }

                    MEM_SAFE_FREE(smd->layers[l].frames[m].strokes[k].verts[i].binds);
                  }
                }
              }
              MEM_SAFE_FREE(smd->layers[l].frames[m].strokes[k].verts);
              }
            MEM_SAFE_FREE(smd->layers[l].frames[m].strokes);
          }
        }
        MEM_SAFE_FREE(smd->layers[l].frames);
      }
    }
    MEM_SAFE_FREE(smd->layers); 
  }
  smd->num_of_layers = 0;
  smd->bound_flags = 0;


}

static void copyData(const GpencilModifierData *md, GpencilModifierData *target)
{
  SurDeformGpencilModifierData *smd = (SurDeformGpencilModifierData *)md;
  SurDeformGpencilModifierData *tsmd = (SurDeformGpencilModifierData *)target;

  BKE_gpencil_modifier_copydata_generic(md, target);
  tsmd->bound_flags = smd->bound_flags;
  if (smd->layers)
  { 
    rollback_layers(smd);
    tsmd->layers = MEM_dupallocN(smd->layers);
    for(int l = 0; l < smd->num_of_layers; l++)
    {
      if (smd->layers[l].frames)
      {
        rollback_frames(smd, &smd->layers[l]);
        tsmd->layers[l].frames = MEM_dupallocN(smd->layers[l].frames);
        for (int f = 0; f < smd->layers[l].num_of_frames; f++)
        {
          if (smd->layers[l].frames[f].strokes)  
          {
            rollback_strokes(smd, &smd->layers[l].frames[f] );
            tsmd->layers[l].frames[f].strokes = MEM_dupallocN(smd->layers[l].frames[f].strokes);

            for(int k = 0; k < smd->layers[l].frames[f].strokes_num; k++)
            {
              if (smd->layers[l].frames[f].strokes[k].verts) 
              {
                tsmd->layers[l].frames[f].strokes[k].verts = MEM_dupallocN(smd->layers[l].frames[f].strokes[k].verts);

                for (int i = 0; i < smd->layers[l].frames[f].strokes[k].stroke_verts_num; i++) 
                {
                  if (smd->layers[l].frames[f].strokes[k].verts[i].binds) 
                  {
                    tsmd->layers[l].frames[f].strokes[k].verts[i].binds = MEM_dupallocN(smd->layers[l].frames[f].strokes[k].verts[i].binds);
                    for (int j = 0; j < smd->layers[l].frames[f].strokes[k].verts[i].binds_num; j++) 
                    {
                      if (smd->layers[l].frames[f].strokes[k].verts[i].binds[j].vert_inds) 
                      {
                        tsmd->layers[l].frames[f].strokes[k].verts[i].binds[j].vert_inds = MEM_dupallocN(
                            smd->layers[l].frames[f].strokes[k].verts[i].binds[j].vert_inds);
                      }

                      if (smd->layers[l].frames[f].strokes[k].verts[i].binds[j].vert_weights) 
                      {
                        tsmd->layers[l].frames[f].strokes[k].verts[i].binds[j].vert_weights = MEM_dupallocN(
                            smd->layers[l].frames[f].strokes[k].verts[i].binds[j].vert_weights);
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }
}

static bool dependsOnTime(GpencilModifierData *md)

{
  return true;
}

static void updateDepsgraph(GpencilModifierData *md, const ModifierUpdateDepsgraphContext *ctx, const int UNUSED(mode))
{
  SurDeformGpencilModifierData *smd = (SurDeformGpencilModifierData *)md;
  if (smd->target != NULL) {
    DEG_add_object_relation(
        ctx->node, smd->target, DEG_OB_COMP_GEOMETRY, "Surface Deform GP Modifier");
    DEG_add_object_relation(
        ctx->node, smd->target, DEG_OB_COMP_TRANSFORM, "Surface Deform GP Modifier");
  }
}

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
  copy_v3_v3(point_co, vertex_cos );
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

        sdbind->mode = MOD_SDEF_MODE_NGON;
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

          sdbind->mode = MOD_SDEF_MODE_CENTROID;
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

          sdbind->mode = MOD_SDEF_MODE_LOOPTRI;
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

/* Allocate the 'strokes' array. UNUSED */
static bool allocate_stroke_array(uint strokes_num, SDefGPFrame *curr_frame, SurDeformGpencilModifierData *smd_eval)
{
  curr_frame->strokes = MEM_malloc_arrayN(strokes_num, sizeof(*curr_frame->strokes), "SDefGPStrokes");
  if (curr_frame->strokes == NULL) {
    BKE_gpencil_modifier_set_error( (GpencilModifierData *)smd_eval, "Out of memory");
    return false;
  }

  return true;
}

static bool add_layer(SurDeformGpencilModifierData *smd_orig, 
                      SurDeformGpencilModifierData *smd_eval,
                      bGPDlayer *gpl
                      )
{ /* If we're not on the first layer, rollback the pointer*/
  
  rollback_layers(smd_orig);
  
  
  /* every time we add a new layer to our array, by the bind operator, we should 
  free the old one and copy in to a new bigger/smaller location. 

  */
 

  /*if this is the first layer, so num_of_layers has just become one, we don't need to copy anything.*/
  /*
  if (smd_orig->num_of_layers > 1)
  {
    smd_orig->layers = MEM_recallocN_id(smd_orig->layers, sizeof(*smd_orig->layers) * smd_orig->num_of_layers, "SDefGPLayers");
  }
  else
  {
    smd_orig->layers = MEM_calloc_arrayN(smd_orig->num_of_layers, sizeof(*smd_orig->layers), "SDefGPLayers");
  } /**/

  /*We can't use MEM_recallocN because it doesn't support arrays.
  */
  smd_orig->num_of_layers++;
  void *temp_lay_pointer = MEM_calloc_arrayN(smd_orig->num_of_layers, sizeof(*smd_orig->layers), "SDefGPLayers");

  if (temp_lay_pointer == NULL) {
    BKE_gpencil_modifier_set_error( (GpencilModifierData *)smd_eval, "Out of memory");
    return false;
  }

  /*if this is the first frame, so num_of_frame has just become one, we don't need to copy anything.*/
  if (smd_orig->num_of_layers > 1)
  {
    memcpy(temp_lay_pointer, smd_orig->layers, sizeof(*smd_orig->layers) * (smd_orig->num_of_layers-1));
    MEM_SAFE_FREE(smd_orig->layers);
  }
  smd_orig->layers = temp_lay_pointer;
 // smd_orig->layers = temp_layers_pointer;
  uint old_lay_index; 
  /*Now smd_orig->layers is on 0. we need to bring it to the new layer position.*/
  for (int l= 0; l < smd_orig->num_of_layers-1; l++)
  { old_lay_index = smd_orig->layers->layer_idx;
    smd_orig->layers++;}

  /*Fill in the layer data*/
  smd_orig->layers->frames = NULL;
  smd_orig->layers->num_of_frames = 0;
  smd_orig->layers->blender_layer = gpl;
   if (smd_orig->num_of_layers > 1)
  {smd_orig->layers->layer_idx = old_lay_index + 1;}
  else
  {smd_orig->layers->layer_idx = 0;}
  
 // for (int c = 0; c < 128; c++)
  //{smd_orig->layers->layer_info[c] = gpl->info[c];}
  //smd_orig->layers->layer_idx
  
  return true;
}

static bool add_frame(SurDeformGpencilModifierData *smd_orig, 
                      SurDeformGpencilModifierData *smd_eval,
                      SDefGPLayer *sdef_layer,
                      bGPDframe *gpf
                      )
{ /* If we're not on the first frame, rollback the pointer*/
  if (sdef_layer->frames != NULL)
  {
    if (sdef_layer->frames->blender_frame->prev != NULL)
      {rollback_frames(smd_orig, sdef_layer);}
  }
  /* every time we add a new frame to our array, by the bind operator, we should 
  free the old one and copy in to a new bigger/smaller location. 

  We can't use MEM_recallocN because it doesn't support arrays.
  */
  sdef_layer->num_of_frames++;
  void *temp_frames_pointer = MEM_calloc_arrayN(sdef_layer->num_of_frames, sizeof(*sdef_layer->frames), "SDefGPFrames");

  if (temp_frames_pointer == NULL) {
    BKE_gpencil_modifier_set_error( (GpencilModifierData *)smd_eval, "Out of memory");
    return false;
  }

  /*if this is the first frame, so num_of_frame has just become one, we don't need to copy anything.*/
  if (sdef_layer->num_of_frames > 1)
  {
    memcpy(temp_frames_pointer, sdef_layer->frames, sizeof(*sdef_layer->frames) * (sdef_layer->num_of_frames-1));
    MEM_SAFE_FREE(sdef_layer->frames);
  }
  
  sdef_layer->frames = temp_frames_pointer;

  /*Now sdef_layer->frames is on 0. we need to bring it to the new frame position.*/
  uint frame_idx = 0;
  sdef_layer->frames->frame_idx = frame_idx;
  for (int f= 0; f < sdef_layer->num_of_frames-1; f++)
  { sdef_layer->frames++;
    frame_idx ++;
    sdef_layer->frames->frame_idx = frame_idx;}

  /*Fill in the frame data*/
  sdef_layer->frames->blender_frame = gpf;
  //sdef_layer->frames->frame_number // Not storing it because it could just change
  sdef_layer->frames->strokes_num = BLI_listbase_count(&(gpf->strokes));
  
  /*Allocate the strokes*/
  sdef_layer->frames->strokes = MEM_malloc_arrayN(sdef_layer->frames->strokes_num, sizeof(*sdef_layer->frames->strokes), "SDefGPStrokes");
  if ( sdef_layer->frames->strokes == NULL) {
      BKE_gpencil_modifier_set_error( (GpencilModifierData *)smd_eval, "Out of memory");
      return false;
    }
  
  
  /*if (gps->prev == NULL){ //gps should have different arrays for different frames, so this should work.
      allocate_stroke_array(BLI_listbase_count(&(gpf->strokes)), smd_orig, smd);
      printf("first bind exec \n");
      smd_orig->layers->frames->strokes->stroke_idx = 0;
    }
    else {
      uint old_idx = smd_orig->layers->frames->strokes->stroke_idx;
      (smd_orig->layers->frames->strokes)++; /* increase smd->strokes pointer by 1 *
      smd_orig->layers->frames->strokes->stroke_idx = old_idx + 1;   /*increase stroke idx value*
    }*/


  return true;
}

/*Free a single layer*/
static bool free_layer(SurDeformGpencilModifierData *smd_orig,
                      SurDeformGpencilModifierData *smd_eval,
                      bGPDlayer *gpl)
{
  /* If we're not on the first layer, rollback the pointer*/
  if (smd_orig->layers->layer_idx > 0)
    {rollback_layers(smd_orig);}

  /*Do the same thing as add, but in reverse*/
  smd_orig->num_of_layers--;
  SDefGPLayer *temp_layers_pointer = MEM_malloc_arrayN(smd_orig->num_of_layers, sizeof(*smd_orig->layers), "SDefGPLayers");

  if (temp_layers_pointer == NULL) {
    BKE_gpencil_modifier_set_error( (GpencilModifierData *)smd_eval, "Out of memory");
    return false;
  }

  /*if this was the last layer, so num_of_layers has just become 0, we don't need to copy anything.*/
  if (smd_orig->num_of_layers > 0)
  {
    /* Copy one layer at a time, except the one we are unbinding*/
    for (int l = 0; l < smd_orig->num_of_layers; l++)
    {
      if (smd_orig->layers[l].blender_layer == gpl) 
      {
        l--;
        continue;
      }
      memcpy(&temp_layers_pointer[l], &(smd_orig->layers[l]), sizeof(*smd_orig->layers));
    }
    MEM_SAFE_FREE(smd_orig->layers);
  }
  
  smd_orig->layers = temp_layers_pointer;

  return true;

}

/*Free a single frame*/
static bool free_frame(SurDeformGpencilModifierData *smd_orig,
                      SurDeformGpencilModifierData *smd_eval,
                      SDefGPLayer *sdef_layer,
                      bGPDframe *gpf)
{
  /* If we're not on the first frame, rollback the pointer*/
  if (sdef_layer->frames->frame_idx > 0)
    {rollback_frames(smd_orig, sdef_layer);}

  /*Do the same thing as add, but in reverse */
  sdef_layer->num_of_frames--;
  SDefGPFrame *temp_frames_pointer = MEM_malloc_arrayN(sdef_layer->num_of_frames, sizeof(*sdef_layer->frames), "SDefGPFrames");

  if (temp_frames_pointer == NULL) {
    BKE_gpencil_modifier_set_error( (GpencilModifierData *)smd_eval, "Out of memory");
    return false;
  }

  /*if this was the last frame, so num_of_frame has just become 0, we don't need to copy anything.*/
  if (sdef_layer->num_of_frames > 0)
  { 
    /* Copy one frame at a time, except the one we are unbinding*/
    for (int f= 0; f < sdef_layer->num_of_frames; f++)
    {
      if (sdef_layer->frames[f].blender_frame == gpf) 
      {
        f--;
        continue;
      }
      memcpy(&temp_frames_pointer[f], &(sdef_layer->frames[f]), sizeof(*sdef_layer->frames));
    }
    MEM_SAFE_FREE(sdef_layer->frames);
    
  }
  
  sdef_layer->frames = temp_frames_pointer;

  /* Free stroke array*/
  MEM_SAFE_FREE(sdef_layer->frames);
  return true;

}






/* Remove vertices without bind data from the bind array. 
static void compactSparseBinds(SurDeformGpencilModifierData *smd)
{
  smd->bind_verts_num = 0;

  for (uint i = 0; i < smd->mesh_verts_num; i++) {
    if (smd->verts[i].binds_num > 0) {
      smd->verts[smd->bind_verts_num++] = smd->verts[i];
    }
  }

  smd->verts = MEM_reallocN_id(
      smd->verts, sizeof(*smd->verts) * smd->bind_verts_num, "SDefBindVerts (sparse)");
} */

static bool surfacedeformBind_stroke(
                              uint stroke_idx,
                              SurDeformGpencilModifierData *smd_orig,
                              SurDeformGpencilModifierData *smd_eval,
                              bGPDstroke *gps,
                              int verts_num,
                              uint target_verts_num,
                              Mesh *target,
                              float(*positions)[3],
                              MPoly *mpoly,
                              MEdge *medge,
                              MLoop *mloop,
                              BVHTreeFromMesh treeData,
                              SDefAdjacencyArray *vert_edges,
                              SDefEdgePolys *edge_polys
                              )
{
  SDefGPStroke *current_stroke = smd_orig->layers->frames->strokes;
  current_stroke->stroke_idx = stroke_idx;   /*increase stroke idx value*/
  current_stroke->blender_stroke = gps;
  current_stroke->stroke_verts_num =verts_num;
  current_stroke->verts = MEM_calloc_arrayN(verts_num, sizeof(*current_stroke->verts), "SDefBindVerts");
  
  if (current_stroke->verts == NULL) {
    BKE_gpencil_modifier_set_error( (GpencilModifierData *)smd_eval, "Out of memory");
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
    BKE_gpencil_modifier_set_error( (GpencilModifierData *)smd_eval, "Out of memory");
    freeData((GpencilModifierData *)smd_orig);
    return false;
  }

  invert_m4_m4(data.imat, smd_orig->mat);

  for (int i = 0; i < target_verts_num; i++) {
    mul_v3_m4v3(data.targetCos[i], smd_orig->mat,  positions[i]);
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
    BKE_gpencil_modifier_set_error( (GpencilModifierData *)smd_eval, "Out of memory");
    freeData((GpencilModifierData *)smd_orig);
  }
  else if (data.success == MOD_SDEF_BIND_RESULT_NONMANY_ERR) {
    BKE_gpencil_modifier_set_error(
         (GpencilModifierData *)smd_eval, "Target has edges with more than two polygons");
    freeData((GpencilModifierData *)smd_orig);
  }
  else if (data.success == MOD_SDEF_BIND_RESULT_CONCAVE_ERR) {
    BKE_gpencil_modifier_set_error( (GpencilModifierData *)smd_eval, "Target contains concave polygons");
    freeData((GpencilModifierData *)smd_orig);
  }
  else if (data.success == MOD_SDEF_BIND_RESULT_OVERLAP_ERR) {
    BKE_gpencil_modifier_set_error( (GpencilModifierData *)smd_eval, "Target contains overlapping vertices");
    freeData((GpencilModifierData *)smd_orig);
  }
  else if (data.success == MOD_SDEF_BIND_RESULT_GENERIC_ERR) {
    /* I know this message is vague, but I could not think of a way
     * to explain this with a reasonably sized message.
     * Though it shouldn't really matter all that much,
     * because this is very unlikely to occur */
    BKE_gpencil_modifier_set_error( (GpencilModifierData *)smd_eval, "Target contains invalid polygons");
    freeData((GpencilModifierData *)smd_orig);
  }
  else if (current_stroke->stroke_verts_num == 0 || !current_stroke->verts) {
    data.success = MOD_SDEF_BIND_RESULT_GENERIC_ERR;
    BKE_gpencil_modifier_set_error( (GpencilModifierData *)smd_eval, "No vertices were bound");
    freeData((GpencilModifierData *)smd_orig);
  }

  
  /*End: pass onto the next stroke*/
  if (stroke_idx < smd_orig->layers->frames->strokes_num - 1) 
  {
    (smd_orig->layers->frames->strokes)++; /* increase smd->strokes pointer by 1 */
    
  }
  return data.success == 1;
}

/*Set the binding combinations flags.*/
static void check_bind_situation(SurDeformGpencilModifierData *smd,
                                Depsgraph *depsgraph,
                                Scene *scene,
                                Object *ob)
{
  smd->bound_flags = 0;
  if (smd->layers == NULL) 
  {return;}
  rollback_layers(smd);
  
  smd->bound_flags |= GP_MOD_SDEF_SOMETHING_BOUND;

  Object *ob_orig = (Object *)DEG_get_original_id(&ob->id);
  bGPdata *gpd = ob_orig->data;
  bGPDlayer *gpl_active = BKE_gpencil_layer_active_get(gpd);
  
  int num_of_layers_with_all_their_frames_bound = 0;
  int num_of_layers_with_their_curr_frame_bound = 0;
  for (int l = 0; l < smd->num_of_layers; l++)
  {

    if (smd->layers[l].num_of_frames == BLI_listbase_count(&(smd->layers[l].blender_layer->frames)))
    {
      /* layer layers[l] has all of its frames bound*/
      num_of_layers_with_all_their_frames_bound++;
      if (smd->layers[l].blender_layer == gpl_active)
      {smd->bound_flags |= GP_MOD_SDEF_CURRENT_LAYER_ALL_FRAMES_BOUND;}
    }
    rollback_frames(smd, &(smd->layers[l]));
    
    for (int f = 0; f < smd->layers[l].num_of_frames; f++)
    {
      if (BKE_gpencil_frame_retime_get(depsgraph, scene, ob, smd->layers[l].blender_layer)->framenum
          == smd->layers[l].frames[f].blender_frame->framenum    )
      {
        if (smd->layers[l].blender_layer == gpl_active)
        {smd->bound_flags |= GP_MOD_SDEF_CURRENT_LAYER_CURRENT_FRAME_BOUND;}
        num_of_layers_with_their_curr_frame_bound++;
        break;
      }
    }
    

  }
  uint totlayers = BLI_listbase_count(&(gpd->layers));
  if   ((smd->num_of_layers == totlayers) &&
        (num_of_layers_with_all_their_frames_bound == smd->num_of_layers))
  {smd->bound_flags |= GP_MOD_SDEF_ALL_LAYERS_AND_FRAMES_BOUND;}
  
    
  if   ((smd->num_of_layers == totlayers) &&
        (num_of_layers_with_their_curr_frame_bound == smd->num_of_layers))
      {smd->bound_flags |= GP_MOD_SDEF_ALL_LAYERS_CURRENT_FRAMES_BOUND;}
    
  
  
}

static bool surfacedeformBind(Object *ob,
                              Depsgraph *depsgraph,
                              SurDeformGpencilModifierData *smd_orig,
                              SurDeformGpencilModifierData *smd_eval,
                              bGPDstroke *gps,
                              bGPDframe *gpf,
                              bGPDlayer *gpl, 
                              uint verts_num,
                              uint target_polys_num,
                              uint target_verts_num,
                              Mesh *target,
                              Mesh *mesh)
{
  BVHTreeFromMesh treeData = {NULL};
  Object *ob_orig = (Object *)DEG_get_original_id(&ob->id);
  bGPdata *gpd = ob_orig->data;
  Scene *scene = DEG_get_evaluated_scene(depsgraph);
  bGPDlayer *gpl_active = BKE_gpencil_layer_active_get(gpd);

  /*If unbind mode: unbind and exit */
  if (smd_orig->bind_modes & GP_MOD_SDEF_UNBIND_MODE)
  {
    rollback_layers(smd_orig);
    if (smd_orig->bind_modes & GP_MOD_SDEF_BIND_ALL_LAYERS) 
    {
      /*free one frame or all the frames?*/
      if (smd_orig->bind_modes & GP_MOD_SDEF_BIND_ALL_FRAMES) 
      {
        /*Just free al of the data*/
        freeData((GpencilModifierData *)smd_orig);
      }
      else
      {
        for (int l = 0; l < smd_orig->num_of_layers; l++)
        {
          
          free_frame(smd_orig, smd_eval, &(smd_orig->layers[l]),  BKE_gpencil_frame_retime_get(depsgraph, scene, ob, smd_orig->layers[l].blender_layer));
          
        }
      }
      
    }
    else /* Unbind only current layer*/
    {
      while (smd_orig->layers->blender_layer != gpl_active) {smd_orig->layers++;}
      /*free one frame or all the frames?*/
      if (smd_orig->bind_modes & GP_MOD_SDEF_BIND_ALL_FRAMES) 
      {
        if (smd_orig->num_of_layers > 1)
        {
          for (int f = 0; f < smd_orig->layers->num_of_frames; f++)
          {
            free_frame(smd_orig, smd_eval, &(smd_orig->layers), smd_orig->layers->frames[f].blender_frame );
          }
          
          {free_layer(smd_orig, smd_eval, smd_orig->layers->blender_layer);}
         
        }
        else // If this is the only layer, free all data
        {freeData((GpencilModifierData *)smd_orig);}
      }
      else
      {
        if (smd_orig->num_of_layers > 1)
        {
          if (smd_orig->layers->num_of_frames > 1)
          {free_frame(smd_orig, smd_eval, &(smd_orig->layers),  BKE_gpencil_frame_retime_get(depsgraph, scene, ob, smd_orig->layers->blender_layer));}
          else
          {free_layer(smd_orig, smd_eval, smd_orig->layers->blender_layer);}
        }
        else
        {// If this is the only layer, only frame, free all data
          if (smd_orig->layers->num_of_frames > 1)
          {free_frame(smd_orig, smd_eval, &(smd_orig->layers),  BKE_gpencil_frame_retime_get(depsgraph, scene, ob, smd_orig->layers->blender_layer));}
          else
         { freeData((GpencilModifierData *)smd_orig);}
        }
      }
      
    }
    rollback_layers(smd_orig);
    for (int l = 0; l < smd_orig->num_of_layers; l++) 
    {
      if (smd_orig->layers[l].frames != NULL)
      {rollback_frames(smd_orig, &smd_orig->layers[l]);}
    }
    return true;    //TODO: checks for operation successful  
  }



  const float(*positions)[3] = BKE_mesh_vert_positions(target);  //  FOR 3.4 +   
  const MPoly *mpoly = BKE_mesh_polys(target);
  const MEdge *medge = BKE_mesh_edges(target);
  const MLoop *mloop = BKE_mesh_loops(target);
  /*const MVert *mvert = target->mvert;   //FOR 3.3-
  const MPoly *mpoly = target->mpoly;
  const MEdge *medge = target->medge;
  const MLoop *mloop = target->mloop; */

  uint tedges_num = target->totedge;
 // uint current_stroke_idx = smd_orig->current_stroke_index;
  int adj_result;
  SDefAdjacencyArray *vert_edges;
  SDefAdjacency *adj_array;
  SDefEdgePolys *edge_polys;

  //printf("target->mpoly: %p, target->mpoly->totloop: %d", target->mpoly, target->mpoly->totloop);
  //printf("mPoly: %p, mpoly->totloop: %d", mpoly, mpoly->totloop);

  vert_edges = MEM_calloc_arrayN(target_verts_num, sizeof(*vert_edges), "SDefVertEdgeMap");
  if (vert_edges == NULL) {
    BKE_gpencil_modifier_set_error( (GpencilModifierData *)smd_eval, "Out of memory");
    return false;
    }
  
  adj_array = MEM_malloc_arrayN(tedges_num, 2 * sizeof(*adj_array), "SDefVertEdge");
  if (adj_array == NULL) {
    BKE_gpencil_modifier_set_error( (GpencilModifierData *)smd_eval, "Out of memory");
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
  
  BKE_bvhtree_from_mesh_get(&treeData, target, BVHTREE_FROM_LOOPTRI, 2);
  if (treeData.tree == NULL) {
    BKE_gpencil_modifier_set_error( (GpencilModifierData *)smd_eval, "Out of memory");
    freeAdjacencyMap(vert_edges, adj_array, edge_polys);
    MEM_freeN(smd_orig->layers);
    smd_orig->layers = NULL;
    return false;
  }

  //printf("mPoly: %p, mpoly->totloop: %d", mpoly, mpoly->totloop);

  adj_result = buildAdjacencyMap(
      mpoly, medge, mloop, target_polys_num, tedges_num, vert_edges, adj_array, edge_polys);
  
  if (adj_result == MOD_SDEF_BIND_RESULT_NONMANY_ERR) {
    BKE_gpencil_modifier_set_error(
         (GpencilModifierData *)smd_eval, "Target has edges with more than two polygons");
    freeAdjacencyMap(vert_edges, adj_array, edge_polys);
    free_bvhtree_from_mesh(&treeData);
    MEM_freeN(smd_orig->layers);
    smd_orig->layers = NULL;
    return false;
  }

  smd_orig->target_verts_num = target_verts_num;
  smd_orig->target_polys_num = target_polys_num;
  float current_frame = BKE_scene_frame_get(scene);
  
  /* Bind all layers and bind only one layer are mutually exclusive. */
  if (smd_orig->bind_modes & GP_MOD_SDEF_BIND_ALL_LAYERS) 
  {
    LISTBASE_FOREACH (bGPDlayer *, curr_gpl, &gpd->layers)
    {
      /* If a layer is already bound, skip it.*/
      rollback_layers(smd_orig);
      int l = 0;
      bool resutl = 1;
      while (l != smd_orig->num_of_layers && resutl != 0) 
      {
        resutl = strcmp(smd_orig->layers[l].blender_layer->info, curr_gpl->info);
        if(resutl) ++l;
      } 
      if ( l != smd_orig->num_of_layers ) 
      {continue;}
      
      add_layer(smd_orig, smd_eval, curr_gpl);
      /* Now smd_orig->layers should point to the new layer. */
      
      if (smd_orig->bind_modes & GP_MOD_SDEF_BIND_ALL_FRAMES) 
      {
        LISTBASE_FOREACH (bGPDframe *, curr_gpf, &curr_gpl->frames)
        {
          /* If a frame is already bound, skip it.*/
          rollback_frames(smd_orig, smd_orig->layers);
          int f = 0;
          while (f != smd_orig->layers->num_of_frames && smd_orig->layers->frames[f].blender_frame->framenum != curr_gpf->framenum) ++f; // credits for this line: "Vlad from Moscow " from stack overflow. This is so fricking smart I can't
          if ( f != smd_orig->layers->num_of_frames ) 
          {continue;}
          add_frame(smd_orig, smd_eval, smd_orig->layers, curr_gpf); 
          smd_orig->flags |= GP_MOD_SDEF_WITHHOLD_EVALUATION;
          smd_eval->flags |= GP_MOD_SDEF_WITHHOLD_EVALUATION;
          BKE_scene_frame_set(scene, (float)curr_gpf->framenum);
          BKE_scene_graph_update_for_newframe(depsgraph);
          smd_orig->flags &= ~GP_MOD_SDEF_WITHHOLD_EVALUATION;
          smd_eval->flags &= ~GP_MOD_SDEF_WITHHOLD_EVALUATION;
          uint s = 0;
          LISTBASE_FOREACH (bGPDstroke *, curr_gps, &curr_gpf->strokes)
          {
            if (!surfacedeformBind_stroke(s, smd_orig, smd_eval, curr_gps, curr_gps->totpoints,
                                 target_verts_num,  target, 
                                positions, mpoly, medge, mloop, treeData, vert_edges, edge_polys))
            {freeAdjacencyMap(vert_edges, adj_array, edge_polys);
            return false;}
            s++;
          }
          rollback_strokes(smd_orig, smd_orig->layers->frames);
            

        }
        
      }
      else
      {
        /* If a frame is already bound, skip it.*/
        rollback_frames(smd_orig, smd_orig->layers);
        int f = 0;
        while (f != smd_orig->layers->num_of_frames && smd_orig->layers->frames[f].blender_frame->framenum != smd_orig->layers->frames->blender_frame->framenum) ++f; // credits for this line: "Vlad from Moscow " from stack overflow. This is so fricking smart I can't
        if ( f == smd_orig->layers->num_of_frames ) 
        {
          add_frame(smd_orig, smd_eval, smd_orig->layers, BKE_gpencil_frame_retime_get(depsgraph, scene, ob, smd_orig->layers->blender_layer));
          uint s = 0;
          LISTBASE_FOREACH (bGPDstroke *, curr_gps, &smd_orig->layers->frames->blender_frame->strokes)
          {
            if (!surfacedeformBind_stroke(s, smd_orig, smd_eval, curr_gps, curr_gps->totpoints,
                                  target_verts_num,  target, 
                                positions, mpoly, medge, mloop, treeData, vert_edges, edge_polys))
            {freeAdjacencyMap(vert_edges, adj_array, edge_polys);
            return false;}
            s++;
          }
          rollback_strokes(smd_orig, smd_orig->layers->frames);
        }
      }
    }
  }
  else /*Bind just current layer (Not much useful now that I think of it, but well...)*/
  {
    /* If a layer is already bound, see if additional frames have to be bound.*/
      int l = 0;
      while (l != smd_orig->num_of_layers && smd_orig->layers[l].blender_layer != gpl_active) ++l; 
      if ( l != smd_orig->num_of_layers ) 
      {
        smd_orig->layers = &smd_orig->layers[l]; // ...Can I do this? Idk I hope
      }
      else
      {add_layer(smd_orig, smd_eval, gpl_active);}
    /* Bind all frames and bind only one frame are mutually exclusive. */
    if (smd_orig->bind_modes & GP_MOD_SDEF_BIND_ALL_FRAMES) 
    {
      LISTBASE_FOREACH (bGPDframe *, curr_gpf, &gpl_active->frames)
        {
          /* If a frame is already bound, skip it.*/
          rollback_frames(smd_orig, smd_orig->layers);
          int f = 0;
          while (f != smd_orig->layers->num_of_frames && smd_orig->layers->frames[f].blender_frame != curr_gpf) ++f; // credits for this line: "Vlad from Moscow " from stack overflow. This is so fricking smart I can't
          if ( f != smd_orig->layers->num_of_frames ) 
          {continue;}
          add_frame(smd_orig, smd_eval, smd_orig->layers, curr_gpf); 
          BKE_scene_frame_set(scene, (float)curr_gpf->framenum);
          BKE_scene_graph_update_for_newframe(depsgraph);
          uint s = 0;
          LISTBASE_FOREACH (bGPDstroke *, curr_gps, &curr_gpf->strokes)
          {
            if (!surfacedeformBind_stroke(s, smd_orig, smd_eval, curr_gps, curr_gps->totpoints,
                                 target_verts_num,  target, 
                                positions, mpoly, medge, mloop, treeData, vert_edges, edge_polys))
            {freeAdjacencyMap(vert_edges, adj_array, edge_polys);
            return false;}
            s++;
          }
          rollback_strokes(smd_orig, smd_orig->layers->frames);
        }
        
    }
    else
    {
      /* If a frame is already bound, skip it.*/
      rollback_frames(smd_orig, smd_orig->layers);
      int f = 0;
      while (f != smd_orig->layers->num_of_frames && smd_orig->layers->frames[f].blender_frame != smd_orig->layers->frames->blender_frame) ++f; // credits for this line: "Vlad from Moscow " from stack overflow. This is so fricking smart I can't
      if ( f == smd_orig->layers->num_of_frames ) 
      {
        add_frame(smd_orig, smd_eval, smd_orig->layers, BKE_gpencil_frame_retime_get(depsgraph, scene, ob, smd_orig->layers->blender_layer));
        uint s = 0;
        LISTBASE_FOREACH (bGPDstroke *, curr_gps, &smd_orig->layers->frames->blender_frame->strokes)
        {
          if (!surfacedeformBind_stroke( s, smd_orig, smd_eval, curr_gps, curr_gps->totpoints,
                                target_verts_num,  target, 
                              positions, mpoly, medge, mloop, treeData, vert_edges, edge_polys))
          {freeAdjacencyMap(vert_edges, adj_array, edge_polys);
          return false;}
          s++;
        }
        rollback_strokes(smd_orig, smd_orig->layers->frames);
      }
    }
  }

  BKE_scene_frame_set(scene, current_frame);
  freeAdjacencyMap(vert_edges, adj_array, edge_polys);
  free_bvhtree_from_mesh(&treeData);
  return true;
}




static void deformVert(void *__restrict userdata,
                       const int index,
                       const TaskParallelTLS *__restrict UNUSED(tls))
{
  const SDefDeformData *const data = (SDefDeformData *)userdata;
  const SDefGPBind *sdbind = data->bind_verts[index].binds;
  const int sdbind_num = data->bind_verts[index].binds_num;
  const unsigned int vertex_idx = data->bind_verts[index].vertex_idx;
  float *const vertexCos = &(data->gps->points[vertex_idx].x);
  float norm[3], temp[3], offset[3];

  /* Retrieve the value of the weight vertex group if specified. */
  float weight = 1.0f;
  
  /*if (data->dvert && data->defgrp_index != -1) {
    weight = BKE_defvert_find_weight(&data->dvert[vertex_idx], data->defgrp_index);

    if (data->invert_vgroup) {
      weight = 1.0f - weight;
    }
  }*/

  /* Check if this vertex will be deformed. If it is not deformed we return and avoid
   * unnecessary calculations. */
  if (weight == 0.0f) {
    return;
  }

  zero_v3(offset);
  
  /* Allocate a `coords_buffer` that fits all the temp-data. */
  int max_verts = 0;
  for (int j = 0; j < sdbind_num; j++) {
    max_verts = MAX2(max_verts, sdbind[j].verts_num);
  }

  const bool big_buffer = max_verts > 256;
  float(*coords_buffer)[3];
  if (UNLIKELY(big_buffer)) {
    coords_buffer = MEM_malloc_arrayN(max_verts, sizeof(*coords_buffer), __func__);
  }
  else {
    coords_buffer = BLI_array_alloca(coords_buffer, max_verts);
  }
  
  for (int j = 0; j < sdbind_num; j++, sdbind++) {
    for (int k = 0; k < sdbind->verts_num; k++) {
      copy_v3_v3(coords_buffer[k], data->targetCos[sdbind->vert_inds[k]]);
    }

    normal_poly_v3(norm, coords_buffer, sdbind->verts_num);
    zero_v3(temp);

    switch (sdbind->mode) {
      /* ---------- looptri mode ---------- */
      case MOD_SDEF_MODE_LOOPTRI: {
        madd_v3_v3fl(temp, data->targetCos[sdbind->vert_inds[0]], sdbind->vert_weights[0]);
        madd_v3_v3fl(temp, data->targetCos[sdbind->vert_inds[1]], sdbind->vert_weights[1]);
        madd_v3_v3fl(temp, data->targetCos[sdbind->vert_inds[2]], sdbind->vert_weights[2]);
        break;
      }

      /* ---------- ngon mode ---------- */
      case MOD_SDEF_MODE_NGON: {
        for (int k = 0; k < sdbind->verts_num; k++) {
          madd_v3_v3fl(temp, coords_buffer[k], sdbind->vert_weights[k]);
        }
        break;
      }

      /* ---------- centroid mode ---------- */
      case MOD_SDEF_MODE_CENTROID: {
        float cent[3];
        mid_v3_v3_array(cent, coords_buffer, sdbind->verts_num);

        madd_v3_v3fl(temp, data->targetCos[sdbind->vert_inds[0]], sdbind->vert_weights[0]);
        madd_v3_v3fl(temp, data->targetCos[sdbind->vert_inds[1]], sdbind->vert_weights[1]);
        madd_v3_v3fl(temp, cent, sdbind->vert_weights[2]);
        break;
      }
    }

    /* Apply normal offset (generic for all modes) */
    madd_v3_v3fl(temp, norm, sdbind->normal_dist);

    madd_v3_v3fl(offset, temp, sdbind->influence);
  }
  
  /* Subtract the vertex coord to get the deformation offset. */
  sub_v3_v3(offset, vertexCos);

  /* Add the offset to start coord multiplied by the strength and weight values. */
  madd_v3_v3fl(vertexCos, offset, data->strength * weight);

  if (UNLIKELY(big_buffer)) {
    MEM_freeN(coords_buffer);
  }
}



static void surfacedeformModifier_do(GpencilModifierData *md,
                                     SurDeformGpencilModifierData *smd,
                                     Depsgraph *depsgraph,
                                     bGPDstroke *gps,
                                     bGPDframe *gpf,
                                     bGPDlayer *gpl,
                                     Object *ob,
                                     Mesh *mesh)
{
  //SurDeformGpencilModifierData *smd = (SurDeformGpencilModifierData *)md;
  
  Mesh *target;
  uint target_verts_num, target_polys_num;
  uint verts_num = gps->totpoints;
  uint strokes_num = BLI_listbase_count(&gpf->strokes);
  SurDeformGpencilModifierData *smd_orig = get_original_modifier(ob, smd, md);


  
  /* Exit function if bind flag is not set  and free bind data if any. */
  if (smd->bound_flags == 0 && !(smd->flags & GP_MOD_SDEF_DO_BIND)) { 
    
    
    if (smd->layers != NULL) {
      printf("Flag disabled, verts not null \n");
      if (!DEG_is_active(depsgraph)) {
        BKE_gpencil_modifier_set_error(md,  "Attempt to bind from inactive dependency graph");
        return;
      } 
      GpencilModifierData *md_orig = (GpencilModifierData *)get_original_modifier(ob, smd, md);
      freeData(md_orig);
    }
    printf("Flag disabled, verts null \n");
    return;
  }
  Object *ob_target = DEG_get_evaluated_object(depsgraph, smd->target);
  target = BKE_modifier_get_evaluated_mesh_from_evaluated_object(ob_target);

  
  

  if (!target) {
    BKE_gpencil_modifier_set_error(md,  "No valid target mesh");
    return;
  }
  target_verts_num = BKE_mesh_wrapper_vert_len(target);
  target_polys_num = BKE_mesh_wrapper_poly_len(target);
  
  /* If not bound, execute bind. */
  if (smd->flags & GP_MOD_SDEF_DO_BIND ) 
  {
    if (!DEG_is_active(depsgraph)) {
      BKE_gpencil_modifier_set_error(md,  "Attempt to unbind from inactive dependency graph");
      return;
    }

    
    float tmp_mat[4][4];

    invert_m4_m4(tmp_mat, ob_target->object_to_world);
    mul_m4_m4m4(smd_orig->mat, tmp_mat, ob_target->object_to_world);

    /* Avoid converting edit-mesh data, binding is an exception. */
    BKE_mesh_wrapper_ensure_mdata(target);
    if (!(gpl->prev == NULL && gps->prev == NULL)) // Only execute the bind function on the very first evaluation~ 
    {return;}


   

    
    if (!surfacedeformBind(ob,
                           depsgraph,
                           smd_orig,
                           smd,
                           gps,
                           gpf,
                           gpl,
                           verts_num,
                           target_polys_num,
                           target_verts_num,
                           target,
                           mesh))  
    {
      printf("bind failed \n");
      smd_orig->bound_flags = 0;
      freeData((GpencilModifierData *) smd_orig);
    }
    printf("bind good \n");
    smd_orig->flags &= ~GP_MOD_SDEF_DO_BIND;
    /*Withhold evaliation for the next strokes, let's reactivate it in the operator*/
    smd_orig->flags |= GP_MOD_SDEF_WITHHOLD_EVALUATION;
    check_bind_situation(smd_orig, depsgraph,DEG_get_evaluated_scene(depsgraph), ob);
    /* Early abort, this is binding 'call', no need to perform whole evaluation. */
    smd->flags = smd_orig->flags;
    return;
  }
  if (!smd->layers)
  {return;}
    
  printf("real evaluation after bind \n");

  if (gpl->prev == NULL && gps->prev == NULL)
  {check_bind_situation(smd_orig, depsgraph,DEG_get_evaluated_scene(depsgraph), ob);}
  smd -> bound_flags = smd_orig -> bound_flags;

  /*Exit evaluation if we are on a frame that is not bound.*/
  if (smd->layers->frames->blender_frame->framenum != gpf->framenum) 
  {return;}

  /* Strokes count on the deforming Frame. */
  uint tot_strokes_num = BLI_listbase_count(&smd->layers->frames->blender_frame->strokes);
  if (smd->layers->frames->strokes_num != tot_strokes_num) {
    BKE_gpencil_modifier_set_error(
        md, "Strokes changed from %u to %u", smd->layers->frames->strokes_num, tot_strokes_num);
    //TODO: free_frame
    return;
  } 

  /* Points count on the deforming Stroke. */
  if (smd->layers->frames->strokes->stroke_verts_num != smd->layers->frames->strokes->blender_stroke->totpoints) {
    BKE_gpencil_modifier_set_error(
        md, "Stroke %u: Points changed from %i to %i", smd->layers->frames->strokes->stroke_idx, smd->layers->frames->strokes->stroke_verts_num, smd->layers->frames->strokes->blender_stroke->totpoints);
    return;
  } 

  /* Geometry count on the target mesh. */
  if (smd->target_polys_num != target_polys_num && smd->target_verts_num == 0) {
    /* Change in the number of polygons does not really imply change in the vertex count, but
     * this is how the modifier worked before the vertex count was known. Follow the legacy
     * logic without requirement to re-bind the mesh. */
    BKE_gpencil_modifier_set_error(
        md, "Target polygons changed from %u to %u", smd->target_polys_num, target_polys_num);
    return;
  }
  if (smd->target_verts_num != 0 && smd->target_verts_num != target_verts_num) {
    if (smd->target_verts_num > target_verts_num) {
      /* Number of vertices on the target did reduce. There is no usable recovery from this. */
      BKE_gpencil_modifier_set_error(
                             md,
                             "Target vertices changed from %u to %u",
                             smd->target_verts_num,
                             target_verts_num);
      return;
    }

    /* Assume the increase in the vertex count means that the "new" vertices in the target mesh are
     * added after the original ones. This covers typical case when target was at the subdivision
     * level 0 and then subdivision was increased (i.e. for the render purposes). 

    BKE_modifier_set_warning(ob,
                             md,
                             "Target vertices changed from %u to %u, continuing anyway",
                             smd->target_verts_num,
                             target_verts_num);*/

    /* In theory we only need the `smd->verts_num` vertices in the `targetCos` for evaluation, but
     * it is not currently possible to request a subset of coordinates: the API expects that the
     * caller needs coordinates of all vertices and asserts for it. */
  }

  /* Early out if modifier would not affect input at all - still *after* the sanity checks
   * (and potential binding) above. */
  if (smd->strength == 0.0f) {
    return;
  }

  /*int defgrp_index;
  MDeformVert *dvert;
  MOD_get_vgroup(ob, mesh, smd->defgrp_name, &dvert, &defgrp_index);
  const bool invert_vgroup = (smd->flags & MOD_SDEF_INVERT_VGROUP) != 0;*/

  /* Actual vertex location update starts here */
  SDefGPLayer *curr_layer = NULL;
  SDefGPFrame *curr_frame = NULL;
  
//  Object *ob_orig = (Object *)DEG_get_original_id(&ob->id);
  //bGPdata *gpd = ob_orig->data;
 // bGPDlayer *gpl_orig = BKE_gpencil_layer_active_get(gpd);
 // bGPDframe *gpf_orig = BKE_gpencil_frame_retime_get(depsgraph, DEG_get_evaluated_scene(depsgraph), ob, gpl_orig);
  
  /*If we are on stroke 0, check all the current frames of all the layers in memory with their pointer
  to bGPDframe to find the right one and point to that. */
  if (gps->prev == NULL)
  {
    rollback_layers(smd);
    for (int l= 0; l < smd->num_of_layers; l++)
    {
      bool resutl = strcmp(smd->layers[l].blender_layer->info, gpl->info);
      if (!resutl) //strcmp returns non zero in case of a difference
      {
        curr_layer = &(smd->layers[l]);
        
      }
    }

    if (!curr_layer) /* If layer or frame not found, retun and pass onto the next evaluation */
    {
      return;
    }

    
    smd->layers = curr_layer;
    rollback_frames(smd, smd->layers);
    for (int f= 0; f < smd->layers->num_of_frames; f++)
    {
      if (smd->layers->frames[f].blender_frame->framenum == gpf->framenum)
      {
        curr_frame = &(smd->layers->frames[f]);
      }
    }
    if (!curr_frame) /* If layer or frame not found, retun and pass onto the next evaluation */
    {
      return;
    }
    smd->layers->frames = curr_frame;
  }

  
  SDefGPStroke *current_sdef_stroke = smd->layers->frames->strokes;
  SDefDeformData data = {
      .bind_verts = current_sdef_stroke->verts,
      .targetCos = MEM_malloc_arrayN(target_verts_num, sizeof(float[3]), "SDefTargetVertArray"),
      .gps = gps,
      /*.dvert = dvert,
      .defgrp_index = defgrp_index,
      .invert_vgroup = invert_vgroup,*/
      .strength = smd->strength,
  };
  
  if (data.targetCos != NULL) {
    BKE_mesh_wrapper_vert_coords_copy_with_mat4(
        target, data.targetCos, target_verts_num, smd->mat);

    TaskParallelSettings settings;
    BLI_parallel_range_settings_defaults(&settings);
    SurDeformGpencilModifierData *smd_orig = get_original_modifier(ob, smd, md);
    settings.use_threading = (current_sdef_stroke->stroke_verts_num > 10000);
    BLI_task_parallel_range(0, current_sdef_stroke->stroke_verts_num, &data, deformVert, &settings);

    MEM_freeN(data.targetCos);
  }

  BKE_gpencil_stroke_geometry_update(ob->data, gps);
  
  
  
  if (gps->next == NULL) /*If we are on the last stroke...*/
  {
    /* ...Go back to the start of the stroke array. */
    uint target_idx = smd->layers->frames->strokes->stroke_idx;
    for (int idx = 0; idx < target_idx; idx++) {
      (smd->layers->frames->strokes)--;
    }

  }
  else { /*Else increase the pointer */
    (smd->layers->frames->strokes)++;
 
  }
}
/* END MOD_surfacedeform.c FUNCTIONS */

/* uses evaluated modifer */
static void deformStroke(GpencilModifierData *md, // every time deform stroke is ran, increases stroke, frame, layer. in a similar way
                         Depsgraph *depsgraph,
                         Object *ob,
                         bGPDlayer *gpl,
                         bGPDframe *gpf, // bGPDframe *gpf = BKE_gpencil_frame_retime_get(depsgraph, scene, ob, gpl)
                         bGPDstroke *gps)
{
  SurDeformGpencilModifierData *smd = (SurDeformGpencilModifierData *)md;
  if (smd->flags & GP_MOD_SDEF_WITHHOLD_EVALUATION) return;



  /* Update flags to evaluated modifier */
  SurDeformGpencilModifierData *smd_orig = get_original_modifier(ob, smd, md);

  smd->flags = smd_orig->flags;

  /*make surface deform modifier sruff */
  /*Mesh *mesh_src = NULL;

  if (smd->defgrp_name[0] != '\0') {
    /Only need to use mesh_src when a vgroup is used.
    mesh_src = MOD_deform_mesh_eval_get(ctx->object, NULL, mesh, NULL, verts_num, false, false);
  }*/

  /*for (int i = 0; i < gps->totpoints; i++) {
  bGPDspoint *pt = &gps->points[i];
  MDeformVert *dvert = gps->dvert != NULL ? &gps->dvert[i] : NULL;
  bGPdata *gpd = ob->data;
  float mat[4][4] = {1, 2, 1, 1};
  mul_m4_v3(mat, &pt->x);
  BKE_gpencil_stroke_geometry_update(gpd, gps);}*/




  /*fill vertex cos with gp points coordinates
  float vertex_cos_array [1000][3];
  float (*vertexCos)[3] = vertex_cos_array;

  for (int i = 0; i < gps->totpoints; i++) {
  bGPDspoint *pt = &gps->points[i];
  MDeformVert *dvert = gps->dvert != NULL ? &gps->dvert[i] : NULL;
  vertex_cos_array[i][0] = pt->x;
  vertex_cos_array[i][1] = pt->y;
  vertex_cos_array[i][2] = pt->z;
  }*/
  
  surfacedeformModifier_do(md, smd, depsgraph, gps, gpf, gpl, ob, NULL /*, mesh_src*/);
 
  /*if (!ELEM(mesh_src, NULL, mesh)) {
    BKE_id_free(NULL, mesh_src);
  }*/
  

  

}

static void bakeModifier(struct Main *UNUSED(bmain),
                         Depsgraph *depsgraph,
                         GpencilModifierData *md,
                         Object *ob)
{
  generic_bake_deform_stroke(depsgraph, md, ob, false, deformStroke);
}

/* uses original modifer */
static bool isDisabled(GpencilModifierData *md, bool UNUSED(useRenderParams))
{
  SurDeformGpencilModifierData *smd = (SurDeformGpencilModifierData *)md;

  /* The object type check is only needed here in case we have a placeholder
   * object assigned (because the library containing the mesh is missing).
   *
   * In other cases it should be impossible to have a type mismatch.
   */
  return (smd->target == NULL || smd->target->type != OB_MESH) &&
         (smd->layers == NULL || smd->bound_flags ==0);
}

static void panel_draw(const bContext *UNUSED(C), Panel *panel)
{
  uiLayout *sub, *row, *col;
  uiLayout *layout = panel->layout;

  PointerRNA ob_ptr;
  PointerRNA *ptr = gpencil_modifier_panel_get_property_pointers(panel, &ob_ptr);

  PointerRNA target_ptr = RNA_pointer_get(ptr, "target");

  

 // bool unbind_mode = RNA_boolean_get(ptr, "unbind_mode"); 
  bool bind_all_frames = (RNA_enum_get(ptr, "curr_frame_or_all_frames") == GP_MOD_SDEF_BIND_ALL_FRAMES);
  bool bind_all_layers = (RNA_enum_get(ptr, "curr_layer_or_all_layers") == GP_MOD_SDEF_BIND_ALL_LAYERS);
  bool all_layers_and_frames_bound = RNA_boolean_get(ptr, "all_layers_and_frames_bound");
  bool all_layers_current_frames_bound = RNA_boolean_get(ptr, "all_layers_current_frames_bound");
  bool current_layer_all_frames_bound = RNA_boolean_get(ptr, "current_layer_all_frames_bound");
  bool current_layer_current_frame_bound = RNA_boolean_get(ptr, "current_layer_current_frame_bound");
  uiLayoutSetPropSep(layout, true);

  col = uiLayoutColumn(layout, false);
  //uiLayoutSetActive(col, !is_bound);
  uiItemR(col, ptr, "target", 0, NULL, ICON_NONE); // TODO: disable layout if bound
  uiItemR(col, ptr, "falloff", 0, NULL, ICON_NONE);

  bool display_unbind = false;

  /*uiItemR(layout, ptr, "strength", 0, NULL, ICON_NONE);
  row = uiLayoutRow(layout, true);
  uiItemPointerR(row, ptr, "vertex_group", &ob_ptr, "vertex_groups", NULL, ICON_NONE);

  col = uiLayoutColumn(layout, false);
  uiLayoutSetEnabled(col, !is_bound);
  uiLayoutSetActive(col, !is_bound && RNA_string_length(ptr, "vertex_group") != 0);
  uiItemR(col, ptr, "use_sparse_bind", 0, NULL, ICON_NONE); */

  uiItemS(layout);

  col = uiLayoutColumn(layout, false);
  /*Yeah I know there should be one big "if" for this .-.*/
  if (bind_all_frames && bind_all_layers) {
    // verify that all frames in all layers are bound. If true, display "unbind"
    if (all_layers_and_frames_bound){
      display_unbind = true;

    }
    
  }
  else if(!bind_all_frames && !bind_all_layers) {
    // verify that the current frame of the current layer is bound. If true, display "unbind"
    if (current_layer_current_frame_bound){
      display_unbind = true;
    }
  }
  else if (!bind_all_frames && bind_all_layers){
    // verify that the current frames of all layers are bound. If true, display "unbind"
    if (all_layers_current_frames_bound){
      display_unbind = true;
    }
  }
  else if (bind_all_frames && !bind_all_layers){
    // verify that all the frames of the current layer are bound. If true, display "unbind"
    if (current_layer_all_frames_bound){
      display_unbind = true;
    }
  }

  if (display_unbind) {
    RNA_boolean_set(ptr, "unbind_mode", true);
    uiLayoutSetActive(col, !RNA_pointer_is_null(&target_ptr));
    uiItemO(col, IFACE_("Unbind"), ICON_NONE, "GPENCIL_OT_gpencilsurdeform_bind");
  }
  else {
    RNA_boolean_set(ptr, "unbind_mode", false);
    uiLayoutSetActive(col, !RNA_pointer_is_null(&target_ptr));
    uiItemO(col, IFACE_("Bind"), ICON_NONE, "GPENCIL_OT_gpencilsurdeform_bind");
  }
  uiLayoutSetPropSep(layout, true);
  uiItemR(layout, ptr, "curr_frame_or_all_frames", UI_ITEM_R_EXPAND, NULL, ICON_NONE); 
  uiItemR(layout, ptr, "curr_layer_or_all_layers", UI_ITEM_R_EXPAND, NULL, ICON_NONE); 
  gpencil_modifier_panel_end(layout, ptr);
}


static void mask_panel_draw(const bContext *UNUSED(C), Panel *panel)
{
  uiLayout *row, *col;
  uiLayout *layout = panel->layout;
  int toggles_flag = UI_ITEM_R_TOGGLE | UI_ITEM_R_FORCE_BLANK_DECORATE;

  PointerRNA *ptr = gpencil_modifier_panel_get_property_pointers(panel, NULL);

  uiLayoutSetPropSep(layout, true);

  uiItemR(layout, ptr, "target", 0, NULL, ICON_NONE);
  gpencil_modifier_panel_end(layout, ptr);


  /*gpencil_modifier_masking_panel_draw(panel, true, true);*/
}

static void panelRegister(ARegionType *region_type)
{
  PanelType *panel_type = gpencil_modifier_panel_register(
      region_type, eGpencilModifierType_SurDeform, panel_draw);

}

static void foreachIDLink(GpencilModifierData *md, Object *ob, IDWalkFunc walk, void *userData)
{
  SurDeformGpencilModifierData *smd = (SurDeformGpencilModifierData *)md;

  walk(userData, ob, (ID **)&smd->target, IDWALK_CB_NOP);
}

GpencilModifierTypeInfo modifierType_Gpencil_SurDeform = {
    /* name */ "Surface Deform",
    /* structName */ "SurDeformGpencilModifierData",
    /* structSize */ sizeof(SurDeformGpencilModifierData),
    /* type */ eGpencilModifierTypeType_Gpencil,
    /* flags */  eGpencilModifierTypeFlag_EnableInEditmode,

    /* copyData */ copyData,

    /* deformStroke */ deformStroke,
    /* generateStrokes */ NULL,
    /* bakeModifier */ NULL,
    /* remapTime */ NULL,

    /* initData */ initData,
    /* freeData */ freeData,
    /* isDisabled */ isDisabled,
    /* updateDepsgraph */ updateDepsgraph,
    /* dependsOnTime */ dependsOnTime,
    /* foreachIDLink */ foreachIDLink,
    /* foreachTexLink */ NULL,
    /* panelRegister */ panelRegister,
};
