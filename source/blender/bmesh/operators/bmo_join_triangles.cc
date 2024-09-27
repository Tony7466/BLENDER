/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bmesh
 *
 * Convert triangle to quads.
 *
 * TODO
 * - convert triangles to any sided faces, not just quads.
 */

#include "MEM_guardedalloc.h"

#include "BLI_heap.h"
#include "BLI_math_geom.h"
#include "BLI_math_rotation.h"
#include "BLI_math_vector.h"
#include "BLI_sort_utils.h"

#include "BKE_customdata.hh"

#include "bmesh.hh"

#include "intern/bmesh_operators_private.hh" /* own include */

/* This code was is used to keep track of our math for the error values and ensure it's not getting
 * out of control. It's left in, in _DEBUG builds only, as guardrails, but it really isn't worth
 * bothering with the asserts in release builds. This COULD be removed outright, but the diagnostic
 * value seems worthwhile given the small performance penalty during debug.
 */
#ifdef _DEBUG
#  define _DEBUG_ONLY_MONITOR_ERROR(val) \
    BLI_assert(!isnan(val) && !isinf(val) && val >= 0 && val <= 2 * M_PI)
#else
#  define _DEBUG_ONLY_MONITOR_ERROR(val)
#endif

#define FACE_OUT (1 << 0)

/** Computes error of a proposed merge quad.  Quads with the lowerst error are merged first.
 *
 *  A quad that is a flat plane has lower error.
 *  A quad with four coners that are all right angles has lower error.
 *    Note parallelograms are higher errror than squares or rectangles.
 *  A quad that is concave has higher error.
 *
 *  \param v1, v2, v3, v4   The four corner coordinates of the quad
 *  \return the computed error associated with the quad
 */
static float quad_calc_error(const float v1[3],
                             const float v2[3],
                             const float v3[3],
                             const float v4[3])
{
  float error = 0.0f;

  /* Normal difference -- A perfectly flat planar face adds a diff of 0. */
  {
    float n1[3], n2[3];
    float angle_a, angle_b;
    float diff;

    normal_tri_v3(n1, v1, v2, v3);
    normal_tri_v3(n2, v1, v3, v4);
    angle_a = compare_v3v3(n1, n2, FLT_EPSILON) ? 0.0f : angle_normalized_v3v3(n1, n2);

    normal_tri_v3(n1, v2, v3, v4);
    normal_tri_v3(n2, v4, v1, v2);
    angle_b = compare_v3v3(n1, n2, FLT_EPSILON) ? 0.0f : angle_normalized_v3v3(n1, n2);

    diff = (angle_a + angle_b) / float(M_PI * 2);
    _DEBUG_ONLY_MONITOR_ERROR(diff);

    error += diff;
  }

  /* Co-linearity -- A face with four right angle corners adds a diff of 0. */
  {
    float edge_vecs[4][3];
    float diff;

    sub_v3_v3v3(edge_vecs[0], v1, v2);
    sub_v3_v3v3(edge_vecs[1], v2, v3);
    sub_v3_v3v3(edge_vecs[2], v3, v4);
    sub_v3_v3v3(edge_vecs[3], v4, v1);

    normalize_v3(edge_vecs[0]);
    normalize_v3(edge_vecs[1]);
    normalize_v3(edge_vecs[2]);
    normalize_v3(edge_vecs[3]);

    /* a completely skinny face is 'pi' after halving */
    diff = (fabsf(angle_normalized_v3v3(edge_vecs[0], edge_vecs[1]) - float(M_PI_2)) +
            fabsf(angle_normalized_v3v3(edge_vecs[1], edge_vecs[2]) - float(M_PI_2)) +
            fabsf(angle_normalized_v3v3(edge_vecs[2], edge_vecs[3]) - float(M_PI_2)) +
            fabsf(angle_normalized_v3v3(edge_vecs[3], edge_vecs[0]) - float(M_PI_2))) /
           float(M_PI * 2);
    _DEBUG_ONLY_MONITOR_ERROR(diff);

    error += diff;
  }

  /* Concavity -- A face with no concavity adds an error of 0. */
  {
    float area_min, area_max, area_a, area_b;
    float diff;

    area_a = area_tri_v3(v1, v2, v3) + area_tri_v3(v1, v3, v4);
    area_b = area_tri_v3(v2, v3, v4) + area_tri_v3(v4, v1, v2);

    area_min = min_ff(area_a, area_b);
    area_max = max_ff(area_a, area_b);

    /* Note use of ternary operator to guard against divide by zero. */
    diff = area_max ? (1.0f - (area_min / area_max)) : 1.0f;
    _DEBUG_ONLY_MONITOR_ERROR(diff);

    error += diff;
  }
  _DEBUG_ONLY_MONITOR_ERROR(error);

  return error;
}

/** Get the corners of the quad that would result after an edge merge.
 *
 *  \param edge     An edge to be merged.  It must be manifold and have triangles on either side.
 *  \param r_v_quad An array of vertices to return the corners.
 */
static void edge_to_quad_verts(const BMEdge *edge, const BMVert *r_v_quad[4])
{
  BLI_assert(edge);
  BLI_assert(BM_edge_is_manifold(edge));
  BLI_assert(edge->l->f->len == 3 && edge->l->radial_next->f->len == 3);
  r_v_quad[0] = edge->l->v;
  r_v_quad[1] = edge->l->radial_next->prev->v;
  r_v_quad[2] = edge->l->next->v;
  r_v_quad[3] = edge->l->prev->v;
}

/* -------------------------------------------------------------------- */
/** \name Delimit processing
 * \{ */

/* cache customdata delimiters */
struct DelimitData_CD {
  int cd_type;
  int cd_size;
  int cd_offset;
  int cd_offset_end;
};

struct DelimitData {
  uint do_seam : 1;
  uint do_sharp : 1;
  uint do_mat : 1;
  uint do_angle_face : 1;
  uint do_angle_shape : 1;

  float angle_face;
  float angle_face__cos;

  float angle_shape;

  DelimitData_CD cdata[4];
  int cdata_len;
};

/** Determines if the loop customdata is contiguous. */
static bool edge_is_contiguous_loop_cd_all(const BMEdge *e, const DelimitData_CD *delimit_data)
{
  int cd_offset;
  for (cd_offset = delimit_data->cd_offset; cd_offset < delimit_data->cd_offset_end;
       cd_offset += delimit_data->cd_size)
  {
    if (BM_edge_is_contiguous_loop_cd(e, delimit_data->cd_type, cd_offset) == false) {
      return false;
    }
  }

  return true;
}

/** Looks up delimit data from custom data.  Used to delimit by color or UV. */
static bool edge_delimit_cdata(CustomData *ldata, eCustomDataType type, DelimitData_CD *r_delim_cd)
{
  const int layer_len = CustomData_number_of_layers(ldata, type);
  r_delim_cd->cd_type = type;
  r_delim_cd->cd_size = CustomData_sizeof(eCustomDataType(r_delim_cd->cd_type));
  r_delim_cd->cd_offset = CustomData_get_n_offset(ldata, type, 0);
  r_delim_cd->cd_offset_end = r_delim_cd->cd_offset + (r_delim_cd->cd_size * layer_len);
  return (r_delim_cd->cd_offset != -1);
}

/** Setup the delimit data from the parameters provided to the operator.
 *
 *  \param bm the bmesh to provide UV or color data
 *  \param op The operator to provide the parameters
 */
static DelimitData configure_delimit_data(BMesh *bm, BMOperator *op)
{
  DelimitData delimit_data = {0};
  delimit_data.do_seam = BMO_slot_bool_get(op->slots_in, "cmp_seam");
  delimit_data.do_sharp = BMO_slot_bool_get(op->slots_in, "cmp_sharp");
  delimit_data.do_mat = BMO_slot_bool_get(op->slots_in, "cmp_materials");

  /* Determine if angle face processing occurs and its parameters */
  float angle_face = BMO_slot_float_get(op->slots_in, "angle_face_threshold");
  if (angle_face < DEG2RADF(180.0f)) {
    delimit_data.angle_face = angle_face;
    delimit_data.angle_face__cos = cosf(angle_face);
    delimit_data.do_angle_face = true;
  }
  else {
    delimit_data.do_angle_face = false;
  }

  /* Determine if angle shape processing occurs and its parameters */
  float angle_shape = BMO_slot_float_get(op->slots_in, "angle_shape_threshold");
  if (angle_shape < DEG2RADF(180.0f)) {
    delimit_data.angle_shape = angle_shape;
    delimit_data.angle_shape = angle_shape;
    delimit_data.do_angle_shape = true;
  }
  else {
    delimit_data.do_angle_shape = false;
  }

  if (BMO_slot_bool_get(op->slots_in, "cmp_uvs") &&
      edge_delimit_cdata(&bm->ldata, CD_PROP_FLOAT2, &delimit_data.cdata[delimit_data.cdata_len]))
  {
    delimit_data.cdata_len += 1;
  }

  delimit_data.cdata[delimit_data.cdata_len].cd_offset = -1;
  if (BMO_slot_bool_get(op->slots_in, "cmp_vcols") &&
      edge_delimit_cdata(
          &bm->ldata, CD_PROP_BYTE_COLOR, &delimit_data.cdata[delimit_data.cdata_len]))
  {
    delimit_data.cdata_len += 1;
  }
  return delimit_data;
}

/** computes if an edge is a delimit edge, therefore should not be considered for merging.
 *
 *  \param e            the edge to check
 *  \param delimit_data the delimit configuration
 *  \return true, if the edge is a delimit edge.
 */
static bool edge_is_delimit(const BMEdge *e, const DelimitData *delimit_data)
{
  BMFace *f_a = e->l->f, *f_b = e->l->radial_next->f;

#if 0
  const bool is_contig = BM_edge_is_contiguous(e);
  float angle;
#endif

  if (delimit_data->do_seam && BM_elem_flag_test(e, BM_ELEM_SEAM)) {
    return true;
  }

  if (delimit_data->do_sharp && (BM_elem_flag_test(e, BM_ELEM_SMOOTH) == 0)) {
    return true;
  }

  if (delimit_data->do_mat && (f_a->mat_nr != f_b->mat_nr)) {
    return true;
  }

  if (delimit_data->do_angle_face) {
    if (dot_v3v3(f_a->no, f_b->no) < delimit_data->angle_face__cos) {
      return true;
    }
  }

  if (delimit_data->do_angle_shape) {
    const BMVert *verts[4];
    edge_to_quad_verts(e, verts);

    /* if we're checking the shape at all, a flipped face is out of the question */
    if (is_quad_flip_v3(verts[0]->co, verts[1]->co, verts[2]->co, verts[3]->co)) {
      return true;
    }
    else {
      float edge_vecs[4][3];

      sub_v3_v3v3(edge_vecs[0], verts[0]->co, verts[1]->co);
      sub_v3_v3v3(edge_vecs[1], verts[1]->co, verts[2]->co);
      sub_v3_v3v3(edge_vecs[2], verts[2]->co, verts[3]->co);
      sub_v3_v3v3(edge_vecs[3], verts[3]->co, verts[0]->co);

      normalize_v3(edge_vecs[0]);
      normalize_v3(edge_vecs[1]);
      normalize_v3(edge_vecs[2]);
      normalize_v3(edge_vecs[3]);

      if ((fabsf(angle_normalized_v3v3(edge_vecs[0], edge_vecs[1]) - float(M_PI_2)) >
           delimit_data->angle_shape) ||
          (fabsf(angle_normalized_v3v3(edge_vecs[1], edge_vecs[2]) - float(M_PI_2)) >
           delimit_data->angle_shape) ||
          (fabsf(angle_normalized_v3v3(edge_vecs[2], edge_vecs[3]) - float(M_PI_2)) >
           delimit_data->angle_shape) ||
          (fabsf(angle_normalized_v3v3(edge_vecs[3], edge_vecs[0]) - float(M_PI_2)) >
           delimit_data->angle_shape))
      {
        return true;
      }
    }
  }

  if (delimit_data->cdata_len) {
    int i;
    for (i = 0; i < delimit_data->cdata_len; i++) {
      if (!edge_is_contiguous_loop_cd_all(e, &delimit_data->cdata[i])) {
        return true;
      }
    }
  }

  return false;
}

/** \} */

/** Given a manifold edge, join the triangles on either side to form a quad.
 *
 *  \param edge the edge to merge.  It must be manifold.
 *  \param bm   the bmesh containing the edge
 *  \return the face that resulted, or nullptr if the merege was rejected.
 */
static BMFace *join_edge(BMEdge *edge, BMesh *bm)
{
  /* Nonmanifold edges can't be merged. */
  BLI_assert(edge);
  BLI_assert(BM_edge_is_manifold(edge));

  /* Identify the loops on either side of the joinable edge */
  BMLoop *loop_a = edge->l;
  BMLoop *loop_b = edge->l->radial_next;

  /* If previous face merges have created quads, which now make this edge un-mergable, then skip it
   * and move on. This DOES happen frequently and that's ok.  It's much easier and more efficient
   * to just skip these edges when we encounter them, than it is to try to search the heap for them
   * and remove them preemptively. */
  if ((loop_a->f->len != 3) || (loop_b->f->len != 3)) {
    return nullptr;
  }

  /* Join the edge and identify the face */
  BMFace *face_new = BM_faces_join_pair(bm, loop_a, loop_b, true);

  if (face_new) {
    /* Tag the face so the selection can be extended to include the new face. */
    BMO_face_flag_enable(bm, face_new, FACE_OUT);
    /* The combined face is not a tri and should not be counted as a
     * remaining triangle for statistics purposes.  Unset the tag. */
    face_new->head.hflag &= ~BM_ELEM_TAG;
  }

  return face_new;
}

/* Given a mesh, convert triangles to quads */
void bmo_join_triangles_exec(BMesh *bm, BMOperator *op)
{
  /* Local varaibles */
  BMIter bm_iter;
  BMEdge *edge;
  BMOIter slot_iter;
  BMFace *face, *f_a, *f_b;

  Heap *join_edges_heap = BLI_heap_new();

  DelimitData delimit_data = configure_delimit_data(bm, op);

  /* Go through every face in the input slot. Mark triangles for processing. */
  BMO_ITER (face, &slot_iter, op->slots_in, "faces", BM_FACE) {

    /* Flag only the triangles.  This flag serves two functions.
     * First, it marks the faces that are to be processed.
     * Second, it is used later in the UI to print the number of triangles remaining. */
    if (face->len == 3) {
      face->head.hflag |= BM_ELEM_TAG;
    }
  }

  /* Go through every edge in the bmesh, Mark any mergable edges. */
  BM_ITER_MESH (edge, &bm_iter, bm, BM_EDGES_OF_MESH) {
    
    /* If the edge is manifold, has a tagged input tri on both sides, and is NOT delmit ...
     * then it's a candidate to merge.*/
    if (BM_edge_face_pair(edge, &f_a, &f_b) && (f_a->head.hflag & BM_ELEM_TAG) &&
        (f_b->head.hflag & BM_ELEM_TAG) && !edge_is_delimit(edge, &delimit_data))
    {
      /* Compute the error that would result from a merge */
      const BMVert *verts[4];
      edge_to_quad_verts(edge, verts);
      float merge_error = quad_calc_error(verts[0]->co, verts[1]->co, verts[2]->co, verts[3]->co);

      /* Record the candidate merge in the heap. */
      BLI_heap_insert(join_edges_heap, merge_error, edge);
    }
  }

  /* Process all possible merges */
  while (!BLI_heap_is_empty(join_edges_heap)) {

    /* Get the best merge from the priority queue.
     * Remove it from the priority queue. */
    BMEdge *edge = reinterpret_cast<BMEdge *>(BLI_heap_pop_min(join_edges_heap));

    /* Attempt the merge. */
    join_edge(edge, bm);
  }

  /* Clean up. */
  BLI_heap_free(join_edges_heap, nullptr);

  /* Return the selection results. */
  BMO_slot_buffer_from_enabled_flag(bm, op, op->slots_out, "faces.out", BM_FACE, FACE_OUT);
}
