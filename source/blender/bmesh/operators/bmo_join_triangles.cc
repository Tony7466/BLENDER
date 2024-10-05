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
#include "BLI_map.hh"
#include "BLI_math_geom.h"
#include "BLI_math_rotation.h"
#include "BLI_math_vector.h"
#include "BLI_sort_utils.h"

#include "BKE_customdata.hh"

#include "bmesh.hh"

#include "intern/bmesh_operators_private.hh" /* own include */

/* This macro was is used to keep track of our math for the error values and ensure it's not
 * getting out of control. It's left in, in debug builds only, as guardrails, but it really isn't
 * worth bothering with the asserts in release builds. This COULD be removed outright, but the
 * diagnostic value seems worthwhile given the small performance penalty during debug.
 */
#ifndef NDEBUG
#  define ASSERT_VALID_ERROR_METRIC(val) \
    BLI_assert(!isnan(val) && !isinf(val) && val >= 0 && val <= 2 * M_PI)
#else
#  define ASSERT_VALID_ERROR_METRIC(val)
#endif

/* USE_JOIN_TRIANGLE_INTERACTIVE_TESTING allows the developer to interrupt the operator midway
 * through to visualize and debug the actions being taken by the merge algorithm.
 *
 * this flag can be turned on on all debug builds, even when the merge_cap or neighbor_debug
 * parameters might not be being passed.  For example, if the API interface is turned off in
 * editmesh_tools.cc, or if join_triangles is called from other operators such as convex_hull that
 * don't expose the testing API, the testing code still behaves. When merge_cap and neighbor_debug
 * are left unset, the default values pick the normal processing path.
 *
 * Usage:
 *   merge cap selects how many merges are performed before stopping in the middle to visualize
 *   intermediate results.
 *   In the UI, this has a range of -1...n, but in the operator parameters, this is actually
 *   passed as 0...n+1.  This allows the default '0' in the parameter to be a sensible default.
 *
 *   neighbor_debug allows each step in the neighbor improvement process to be visualized.  When
 *   nonzero, the quad that was merged and the two triangles being considered for adjustment are
 *   left selected for visualization.  Additionally, the neighbor quads are adjusted to their
 *   'flattened' position to be in-plane with the quad, to allow visualization of that.
 *   the numerical values related to the improvements are printed to the text console.
 *
 *   merge cap = -1 allows the algorithm to run fully.
 *   merge cap = 0 stops before the first merge.  neighbor_debug can be stepped to diagnose every
 *   neighbor improvement that occurs as a result of the pre-existing quads in the mesh (valid
 *   range for neighbor_debug = 0...8*(num of selected pre-existing quads)
 *   merge_cap = 1, 2, 3... stops after the specified number of merges.  neighbor_debug shows
 *   the neighbor improvements for the last quad that merged.  (Valid range 0...8)
 *
 * To turn on interactive testing, the developer needs to:
 *   - enable USE_JOIN_TRIANGLE_INTERACTIVE_TESTING here.
 *   - enable USE_JOIN_TRIANGLE_INTERACTIVE_TESTING in bmesh_opdefines.cc
 *   - enable USE_JOIN_TRIANGLE_TESTING_API in editmesh_tools.cc.
 */
#if 0
#  define USE_JOIN_TRIANGLE_INTERACTIVE_TESTING
#endif

#define FACE_OUT (1 << 0)
#define FACE_INPUT (1 << 2)

/* improvement rnages from 0..1.  Always impove just a a litle. */
constexpr float minimum_improvement = 0.01f;

/* improvement rnages from 0..1.  Never improve fully. */
constexpr float maximum_improvement = 0.99f;

/* -------------------------------------------------------------------- */
/** \name Join Edges state
 * pass a struct to ensure we don't have to pass these four variables everywhere.
 \{ */

struct JoinEdgesState {
  /** A priority queue of `BMEdge*` to be merged, in order of preference. */
  Heap *heap;

  /** An index that allows looking up the `HeapNode` in `s.heap` from a `BMEdge*`. */
  blender::Map<BMEdge *, HeapNode *> index;

  /** True when topology is being used.  Allows skipping expensive processing if not. */
  bool use_topo_influence;

  /** A float indicating the influence for topology.  Ranges from 0-200. */
  float topo_influnce;

  /** a bool indiciating whether to select all merged quads, or just the unmerged triangeles */
  bool select_tris_only;

  /** The same `BMesh*` that was passed to the operator, used in many functions. */
  BMesh *bm;

#ifdef USE_JOIN_TRIANGLE_INTERACTIVE_TESTING
  /* The number of merges to allow before stopping */
  int debug_merge_cap;

  /* the number of merges processed so far */
  int debug_merge_count;

  /* The index of the neighbor to visualise */
  int debug_neighbor;

  /* The number of neighbors processed so far */
  int debug_neighbor_global_count;

  /* true, when merge cap and neighbor debug say to debug. */
  bool debug_this_step;
#endif
};

/** \} */
/* -------------------------------------------------------------------- */

/**
 * Computes error of a proposed merge quad. Quads with the lowest error are merged first.
 *
 * A quad that is a flat plane has lower error.
 *
 * A quad with four corners that are all right angles has lower error.
 * Note parallelograms are higher error than squares or rectangles.
 *
 * A quad that is concave has higher error.
 *
 * \param v1,v2,v3,v4: The four corner coordinates of the quad.
 * \return The computed error associated with the quad.
 */
static float quad_calc_error(const float v1[3],
                             const float v2[3],
                             const float v3[3],
                             const float v4[3])
{
  float error = 0.0f;

  /* Normal difference: a perfectly flat planar face adds a difference of 0. */
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

    ASSERT_VALID_ERROR_METRIC(diff);

    error += diff;
  }

  /* Co-linearity: a face with four right angle corners adds a difference of 0. */
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

    ASSERT_VALID_ERROR_METRIC(diff);

    error += diff;
  }

  /* Concavity: a face with no concavity adds an error of 0. */
  {
    float area_min, area_max, area_a, area_b;
    float diff;

    area_a = area_tri_v3(v1, v2, v3) + area_tri_v3(v1, v3, v4);
    area_b = area_tri_v3(v2, v3, v4) + area_tri_v3(v4, v1, v2);

    area_min = min_ff(area_a, area_b);
    area_max = max_ff(area_a, area_b);

    /* Note use of ternary operator to guard against divide by zero. */
    diff = area_max ? (1.0f - (area_min / area_max)) : 1.0f;

    ASSERT_VALID_ERROR_METRIC(diff);

    error += diff;
  }

  ASSERT_VALID_ERROR_METRIC(error);

  return error;
}

/** Get the corners of the quad that would result after an edge merge.
 *
 * \param e: An edge to be merged. It must be manifold and have triangles on either side.
 * \param r_v_quad: An array of vertices to return the corners.
 */
static void bm_edge_to_quad_verts(const BMEdge *e, const BMVert *r_v_quad[4])
{
  BLI_assert(e);
  BLI_assert(BM_edge_is_manifold(e));
  BLI_assert(e->l->f->len == 3 && e->l->radial_next->f->len == 3);
  r_v_quad[0] = e->l->v;
  r_v_quad[1] = e->l->radial_next->prev->v;
  r_v_quad[2] = e->l->next->v;
  r_v_quad[3] = e->l->prev->v;
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
static bool bm_edge_is_contiguous_loop_cd_all(const BMEdge *e, const DelimitData_CD *delimit_data)
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

/** Looks up delimit data from custom data. Used to delimit by color or UV. */
static bool bm_edge_delimit_cdata(CustomData *ldata,
                                  eCustomDataType type,
                                  DelimitData_CD *r_delim_cd)
{
  const int layer_len = CustomData_number_of_layers(ldata, type);
  r_delim_cd->cd_type = type;
  r_delim_cd->cd_size = CustomData_sizeof(eCustomDataType(r_delim_cd->cd_type));
  r_delim_cd->cd_offset = CustomData_get_n_offset(ldata, type, 0);
  r_delim_cd->cd_offset_end = r_delim_cd->cd_offset + (r_delim_cd->cd_size * layer_len);
  return (r_delim_cd->cd_offset != -1);
}

/**
 * Setup the delimit data from the parameters provided to the operator.
 *
 * \param bm: The mesh to provide UV or color data.
 * \param op: The operator to provide the parameters.
 */
static DelimitData bm_edge_delmimit_data_from_op(BMesh *bm, BMOperator *op)
{
  DelimitData delimit_data = {0};
  delimit_data.do_seam = BMO_slot_bool_get(op->slots_in, "cmp_seam");
  delimit_data.do_sharp = BMO_slot_bool_get(op->slots_in, "cmp_sharp");
  delimit_data.do_mat = BMO_slot_bool_get(op->slots_in, "cmp_materials");

  /* Determine if angle face processing occurs and its parameters. */
  float angle_face = BMO_slot_float_get(op->slots_in, "angle_face_threshold");
  if (angle_face < DEG2RADF(180.0f)) {
    delimit_data.angle_face = angle_face;
    delimit_data.angle_face__cos = cosf(angle_face);
    delimit_data.do_angle_face = true;
  }
  else {
    delimit_data.do_angle_face = false;
  }

  /* Determine if angle shape processing occurs and its parameters. */
  float angle_shape = BMO_slot_float_get(op->slots_in, "angle_shape_threshold");
  if (angle_shape < DEG2RADF(180.0f)) {
    delimit_data.angle_shape = angle_shape;
    delimit_data.do_angle_shape = true;
  }
  else {
    delimit_data.do_angle_shape = false;
  }

  if (BMO_slot_bool_get(op->slots_in, "cmp_uvs") &&
      bm_edge_delimit_cdata(
          &bm->ldata, CD_PROP_FLOAT2, &delimit_data.cdata[delimit_data.cdata_len]))
  {
    delimit_data.cdata_len += 1;
  }

  delimit_data.cdata[delimit_data.cdata_len].cd_offset = -1;
  if (BMO_slot_bool_get(op->slots_in, "cmp_vcols") &&
      bm_edge_delimit_cdata(
          &bm->ldata, CD_PROP_BYTE_COLOR, &delimit_data.cdata[delimit_data.cdata_len]))
  {
    delimit_data.cdata_len += 1;
  }
  return delimit_data;
}

/**
 * Computes if an edge is a delimit edge, therefore should not be considered for merging.
 *
 * \param e: the edge to check
 * \param delimit_data: the delimit configuration
 * \return true, if the edge is a delimit edge.
 */
static bool bm_edge_is_delimit(const BMEdge *e, const DelimitData *delimit_data)
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
    bm_edge_to_quad_verts(e, verts);

    /* if we're checking the shape at all, a flipped face is out of the question */
    if (is_quad_flip_v3(verts[0]->co, verts[1]->co, verts[2]->co, verts[3]->co)) {
      return true;
    }

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

  if (delimit_data->cdata_len) {
    int i;
    for (i = 0; i < delimit_data->cdata_len; i++) {
      if (!bm_edge_is_contiguous_loop_cd_all(e, &delimit_data->cdata[i])) {
        return true;
      }
    }
  }

  return false;
}

/** \} */

/** Given a array of edges and an array of shared loops...
 *  add the new edge and shared loop to each the array...
 *  IF the edge is not already present.
 *
 * \param merge_edges: the array to add the merge edges to
 * \param shared_loops: the array to add the shared loops to
 * \param count: the number of items currenlty in each array
 * \param merge_edge: The new merge edge to add to the array, if it's not a duplicate.
 * \param shared_loop: The new shared loop to add to the array, if the edge isn't a duplicate
 * \return The number of items now in the array
 */
static size_t add_without_duplicates(BMEdge *merge_edges[8],
                                     BMLoop *shared_loops[8],
                                     size_t count,
                                     BMEdge *merge_edge,
                                     BMLoop *shared_loop)
{
  /* Since a quad has no more than 4 neighbor triangles, and each neighbor triangle has no more
   * than two edges to consider, reprioritize_face_neighbors can't possibly call this function more
   * than 8 times so this can't happen. Still, it's good to safeguard against running off the end
   * of the array. */
  BLI_assert(count < 8);

  /* Don't add null pointers */
  if (merge_edge == nullptr)
    return count;

  /* Don't add duplicates */
  for (size_t index = 0; index < count; index++) {
    if (merge_edges[index] == merge_edge)
      return count;
  }

  /* Add the edge and increase the count by 1 */
  merge_edges[count] = merge_edge;
  shared_loops[count] = shared_loop;
  return count + 1;
}

/** Given a loop inside a quad...
 *  if there is a triangle adjacent to it...
 *  add the other two edges of that triangle...
 *  if they are in turn adjacent to another triangle.
 *
 * \param merge_edges: the array of mergable edges to add to
 * \param shared_loops: the array to shared loops to add to
 * \param count: the number of items currenlty in each array
 * \param loop_in_quad: The loop to add the neighboring edges of, if they check out
 * \return The number of items now in the array
 */
static size_t add_neighbors(BMEdge *merge_edges[8],
                            BMLoop *shared_loops[8],
                            size_t count,
                            BMLoop *loop_in_quad)
{
  /* If the edge is not manifold, there is no neighboring face to process. */
  if (!BM_edge_is_manifold(loop_in_quad->e)) {
    return count; /* No new edges added. */
  }

  BMLoop *loop_in_neighbor = loop_in_quad->radial_next;

  /* If the neighboring face is not a triangle, don't process it. */
  if (loop_in_neighbor->f->len != 3)
    return count; /* No new edges added. */

  /* Get the other two loops of the neighboring triangle */
  BMLoop *l_a = loop_in_neighbor->next;
  BMLoop *l_b = loop_in_neighbor->prev;

  /* If l_a is manifold, and the adjacent face is also a triangle, mark it for potential
   * improvement */
  if (BM_edge_is_manifold(l_a->e) && l_a->radial_next->f->len == 3)
    count = add_without_duplicates(merge_edges, shared_loops, count, l_a->e, loop_in_neighbor);

  /* If l_b is manifold, and the adjacent face is also a triangle, mark it for potential
   * improvement */
  if (BM_edge_is_manifold(l_b->e) && l_b->radial_next->f->len == 3)
    count = add_without_duplicates(merge_edges, shared_loops, count, l_b->e, loop_in_neighbor);

  return count; /* added either 0, 1, or 2 edges. */
}

/** Given a quad defined by quad_verts and an existing plane defined by its normal vector...
 *  rotate the quad around the shared_loop edge...
 *  such that the quad's normal is as aligned as possible to the plane normal.
 *  (in other words, alter the vertices to as flat a combination as possible)
 *  return the new coordinates of that flattened-out quad.
 *
 * \param s: State information about the join_triangles process
 * \param quad_verts: Four vertices of a quad, which has shared_loop as one of its edges
 * \param shared_loop: the 'hinge' loop, shared with the neighbor, that lies in the plane.
 * \param plane_normal: The normal vector of the plane to rotate the quad to lie aligned with
 * \param r_quad_coordinates: An array of coordinates to return the four corrected vertex locations
 */
static void rotate_to_plane(JoinEdgesState &s,
                            const BMVert *quad_verts[4],
                            const BMLoop *shared_loop,
                            const float plane_normal[3],
                            float r_quad_coordinates[4][3])
{
#ifdef USE_JOIN_TRIANGLE_INTERACTIVE_TESTING
  if (s.debug_this_step) {
    printf("angle");
  }
#endif

  float rotation_axis[3];
  sub_v3_v3v3(rotation_axis, shared_loop->v->co, shared_loop->next->v->co);
  normalize_v3(rotation_axis);

  float quad_normal[3] = {0};
  normal_quad_v3(
      quad_normal, quad_verts[0]->co, quad_verts[1]->co, quad_verts[2]->co, quad_verts[3]->co);

  float angle = angle_signed_on_axis_v3v3_v3(plane_normal, quad_normal, rotation_axis);

#ifdef USE_JOIN_TRIANGLE_INTERACTIVE_TESTING
  if (s.debug_this_step) {
    printf(" %f, ", angle / M_PI * 180);
  }
#endif

  for (int n = 0; n < 4; n++) {
    if (quad_verts[n] == shared_loop->v || quad_verts[n] == shared_loop->next->v) {
      /* Two coordinates of the quad match the vector that defines the axis of rotation, so they
       * don't change. */
      copy_v3_v3(r_quad_coordinates[n], quad_verts[n]->co);
    }
    else {
      /* The other two coordinates get rotated around the axis, and so they change. */
      float local_coordinate[3];
      sub_v3_v3v3(local_coordinate, quad_verts[n]->co, shared_loop->v->co);
      rotate_normalized_v3_v3v3fl(r_quad_coordinates[n], local_coordinate, rotation_axis, angle);
      add_v3_v3(r_quad_coordinates[n], shared_loop->v->co);
    }
  }
}

/** Given a pair of quads, compute how well aligned they are.
 *
 * Computes a float, indicating alignment.
 * - regular grids of squares have pairs with alignments near 1.
 * - regular grids of parallelograms also have pairs with alignments near 1.
 * - mismatched combinations of squares, diamonds, parallelograms, trapezoids, etc
 *   have alignments near 0.
 * - however, pairs of quads which lie in perpendicular or opposite-facing planes can
 *   still have good alignments. In other words, pairs of quads which share an edge that
 *   defines a sharp corner on a mesh can still have good alignment, if the quads flow
 *   over the corner in a natural way.  The sharp corner ALONE is NOT a penalty.
 *
 * \param s: State information about the join_triangles process
 * \param quad_a_vecs: an array of four unit vectors as float[3].  These are NOT the coordinates of
 *                     the four vertices of quad_a. Instead, They are four unit vectors, aligned
 *                     parallel to the respective edge loop of quad_a.
 * \param quad_b_verts: an array of four BMVert*, giving the four corners of quad_b
 *                      This would ALSO be const, except that interactive testing moves them.
 * \param shared_loop: a BMLoop that is known to be one of the the common manifold loops that is
 *                     shared between the two quads. This is used as a 'hinge' to flatten the two
 *                     quads into the same plane as much as possible.
 * \param plane_normal: The normal vector of quad_a.
 *
 * \return the computed alignment
 *
 * \note  Since we test quad A against up to eight other quads, we precompute and pass in the
 *        quad_a_vecs, instead of starting with verts, and having to recompute the same numbers
 *        eight different times.
 *        That is why the quad_a_vecs and quad_b_verts have different type definitions.
 */
static float compute_alignment(JoinEdgesState &s,
                               const float quad_a_vecs[4][3],
                               BMVert *quad_b_verts[4],
                               const BMLoop *shared_loop,
                               const float plane_normal[3])
{
  /* Many meshes have lots of curvature or sharp edges.  Pairs of quads shoulnd't be penalized
   * SOLELY becuase they represent a curved surface or define an edge.  So we rotate quad_b around
   * its common edge with quad_a until both are, as much as possible, in the same plane.  This
   * ensures the best possible chance to align */
  float quad_b_coordinates[4][3];
  rotate_to_plane(s, quad_b_verts, shared_loop, plane_normal, quad_b_coordinates);

#ifdef USE_JOIN_TRIANGLE_INTERACTIVE_TESTING
  if (s.debug_this_step) {
    copy_v3_v3(quad_b_verts[0]->co, quad_b_coordinates[0]);
    copy_v3_v3(quad_b_verts[1]->co, quad_b_coordinates[1]);
    copy_v3_v3(quad_b_verts[2]->co, quad_b_coordinates[2]);
    copy_v3_v3(quad_b_verts[3]->co, quad_b_coordinates[3]);
  }
#endif

  /* compute the four unit vectors of the quad b edges. */
  float quad_b_vecs[4][3];
  sub_v3_v3v3(quad_b_vecs[0], quad_b_coordinates[0], quad_b_coordinates[1]);
  sub_v3_v3v3(quad_b_vecs[1], quad_b_coordinates[1], quad_b_coordinates[2]);
  sub_v3_v3v3(quad_b_vecs[2], quad_b_coordinates[2], quad_b_coordinates[3]);
  sub_v3_v3v3(quad_b_vecs[3], quad_b_coordinates[3], quad_b_coordinates[0]);
  normalize_v3(quad_b_vecs[0]);
  normalize_v3(quad_b_vecs[1]);
  normalize_v3(quad_b_vecs[2]);
  normalize_v3(quad_b_vecs[3]);

  /* Given that we're not certain of how the the first loop of the quad and the first loop
   * of the proposed merge quad relate to each other, there are four possible combinations
   * to check, to test that the neighbor face and the merged face have good alignment.
   *
   * In theory, a very nuanced analysis involving shared_loop, loop pointers, vertex pointers,
   * etc, would allow determining which sets of vectors are the right matches sets to compare.
   *
   * Do not meddle in the affairs of algorithms, for they are subtle and quick to anger.
   *
   * Instead, this code does the math twice, then it just flips each component by 180 degrees to
   * pick up the other two cases.  Four extra angle tests aren't that much worse than optimal.
   * Brute forcing the math and ending up with with clear and understandable code is better. */

  /* Compute the case if the quads are aligned. */
  float error_a1 = fabsf(angle_normalized_v3v3(quad_a_vecs[0], quad_b_vecs[0]));
  float error_a2 = fabsf(angle_normalized_v3v3(quad_a_vecs[1], quad_b_vecs[1]));
  float error_a3 = fabsf(angle_normalized_v3v3(quad_a_vecs[2], quad_b_vecs[2]));
  float error_a4 = fabsf(angle_normalized_v3v3(quad_a_vecs[3], quad_b_vecs[3]));
  float error_a = error_a1 + error_a2 + error_a3 + error_a4;

  /* compute the case if the quads are 90 degrees rotated */
  float error_b1 = fabsf(angle_normalized_v3v3(quad_a_vecs[0], quad_b_vecs[1]));
  float error_b2 = fabsf(angle_normalized_v3v3(quad_a_vecs[1], quad_b_vecs[2]));
  float error_b3 = fabsf(angle_normalized_v3v3(quad_a_vecs[2], quad_b_vecs[3]));
  float error_b4 = fabsf(angle_normalized_v3v3(quad_a_vecs[3], quad_b_vecs[0]));
  float error_b = error_b1 + error_b2 + error_b3 + error_b4;

  /* Compute the case if the quads are 180 degrees rotated.
   * This is error_a except each error component is individually rotated 180 degrees. */
  float error_c1 = M_PI - error_a1;
  float error_c2 = M_PI - error_a2;
  float error_c3 = M_PI - error_a3;
  float error_c4 = M_PI - error_a4;
  float error_c = error_c1 + error_c2 + error_c3 + error_c4;

  /* Compute the case if the quads are 270 degrees rotated.
   * This is error_b except each error component is individually rotated 180 degrees. */
  float error_d1 = M_PI - error_b1;
  float error_d2 = M_PI - error_b2;
  float error_d3 = M_PI - error_b3;
  float error_d4 = M_PI - error_b4;
  float error_d = error_d1 + error_d2 + error_d3 + error_d4;

  /* Pick the best option and average the four components. */
  float best_error = std::min(std::min(error_a, error_b), std::min(error_c, error_d)) / 4;

  ASSERT_VALID_ERROR_METRIC(best_error);

  /* Based on the best error, we scale how aligned we are to the range 0...1
   * `M_PI / 4` is used here becuase the worst case is a quad with all four edges at 45 degree
   * angles. */
  float alignment = 1 - (best_error / (M_PI / 4));

  /* even if alignment is TRULY awful, still make at least a tiny improvement. */
  if (alignment < minimum_improvement)
    alignment = minimum_improvement;

  ASSERT_VALID_ERROR_METRIC(alignment);

  return alignment;
}

/** Lowers the error of an edge becuase of its proximity to a known good quad.
 *
 * This function is the core of the entire topology_influence algorithm.
 *
 * This function allows an existing, good quad to influence the topology around it.
 * This means a quad with a higher error can end up preferred - when it creates better toplogy -
 * even though there might be an alternate quad with lower numerical error.
 *
 * This algorithm reduces the error of a given edge based on three factors:
 * - The error of the neighboring quad.  The the better the neighbor quad, the more the impact.
 * - The alignment of the proposed new quad the existing quad.
 *   Grids of rectangles or trapezoids improve well.  Trapezoids and diamonds are left alone.
 * - topology_influence.  The higher the operator parameter is set, the more the impact.
 *   To help counteract the alignment penalty, topology_influence is permitted to exceed 100%.
 *
 * Becuase of the reduction due to misalignment, this will reduce the error of an edge, to be
 * closer to the error of the known good quad, and increase its changes of being merged sooner.
 * However, some of the edge's error always remains -- it never is made EQUAL to the lower error
 * from the good face.  This means the influence of an exceptionally good quad will fade away with
 * each successive, neighbor, instead of affecting the ENTIRE mesh.  This is desirable.
 *
 * \param s: State information about the join_triangles process
 * \param merge_edge: the edge to improve
 * \param shared_loop: the edge that is common between the two faces
 * \param neighbor_quad_vecs: four unit vectors, aligned to the four loops around the good quad
 * \param neighbor_quad_error: the error of the neighbor quad
 * \param neighbor_quad_normal: the normal vector of the good quad
 */
static void reprioritize_join(JoinEdgesState &s,
                              BMEdge *merge_edge,
                              BMLoop *shared_loop,
                              float neighbor_quad_vecs[4][3],
                              float neighbor_quad_error,
                              float neighbor_quad_normal[3])
{
  ASSERT_VALID_ERROR_METRIC(neighbor_quad_error);

  /* If the edge wasn't found, (delimit, nonmanifold, etc) -- return.  Nothing to do here. */
  HeapNode *node = s.index.lookup_default(merge_edge, nullptr);
  if (node == nullptr) {
    return;
  }

  float current_join_error = BLI_heap_node_value(node);

  ASSERT_VALID_ERROR_METRIC(current_join_error);

  /* Never make a join WORSE becuase of topology around it.
   * Becuase we are sorted during the join phase of the algorithm, this should ONLY happen when
   * processing any pre-existing quads in the input mesh during setup.  They might have high error.
   * If they do, ignore them. */
  if (neighbor_quad_error > current_join_error) {

#ifdef USE_JOIN_TRIANGLE_INTERACTIVE_TESTING
    /* This should only happen during setup.
     * Indicates an error, if it happens once we've started merging. */
    BLI_assert(s.debug_merge_count == 0);
#endif

    return;
  }

#ifdef USE_JOIN_TRIANGLE_INTERACTIVE_TESTING
  if (s.debug_this_step) {
    printf("Edge improved from ");
  }
#endif

  /* Get the four corners of the quad that would result if we merged */
  const BMVert *merged_quad_verts[4];
  bm_edge_to_quad_verts(merge_edge, merged_quad_verts);

  /* Now compute the alginment.
   * Regular grids of rectangles or trapezoids have high alignment
   * Mismatched combinations of rectangles diamonds and trapezoids have low alignment. */
  float alignment = compute_alignment(
      s, neighbor_quad_vecs, merged_quad_verts, shared_loop, neighbor_quad_normal);

  /* Compute how much the neighbor is better than the candidate.
   * Since the neighbor quad error is smaller, Improvement is always represented as negative. */
  float improvement = (neighbor_quad_error - current_join_error);

  ASSERT_VALID_ERROR_METRIC(-improvement);

  /* Compute the scale factor for how much of that possible improvement we should apply to this
   * edge. This combines... topology_influence, which is a operator setting... and alignment, which
   * is computed. Faces which are diagonal have an alignment of 0% - perfect rectangular grids have
   * an alignment of 100% Neither topology_influence nor alignment can be negative -- therefore the
   * multiplier NEVER makes error worse. once combined, 0 means no improvement, 1 means improve all
   * the way to exactly match the quality of the contributing neighbor. topology_influece is
   * allowed to exceed 1.0, which lets it cancel out some of the alignment penalty. */
  float multiplier = s.topo_influnce * alignment;

  /* However, the combined multiplier shouldn't ever be allowed to exceed 1.0 becuase permitting
   * that would cause exponential growth when alignment is very good, and when that happens, the
   * algorithm becomes crazy.
   *
   * Further, if we allow a multiplier of exactly 1.0, then all eight edges around the neighbor
   * quad would end up with a quality that is EXACTLY equal to the neighbor quad - and each other -
   * losing valuable information about their relative sorting.  In order to preserve that, the
   * multiplier is capped at 99%. -- The last 1% that is left uncorrected is enough to preserve
   * relative ordering.
   *
   * This especially helps in quads that touch 3-poles and 5-poles.  Since those quads naturally
   * have diamond shapes, their initial error values tend to be higher and they sort to the end of
   * the priority queue. Capping improvement at 99% ensures those quads tend to retain their bad
   * sort, meaning they end up surrounded by good gridded quads, then they merge last, which tends
   * to produce better results.*/
  if (multiplier > maximum_improvement) {
    multiplier = maximum_improvement;
  }

  ASSERT_VALID_ERROR_METRIC(multiplier);

  /* improvement is always represented as a negative number (that will reduce error)
   * Based on that convention, `+` is correct here. */
  float new_join_error = current_join_error + improvement * multiplier;

  ASSERT_VALID_ERROR_METRIC(new_join_error);

#ifdef USE_JOIN_TRIANGLE_INTERACTIVE_TESTING
  if (s.debug_this_step) {
    printf("%f to %f with alignment of %f.\n", current_join_error, new_join_error, alignment);
    BMO_face_flag_enable(s.bm, merge_edge->l->f, FACE_OUT);
    BMO_face_flag_enable(s.bm, merge_edge->l->radial_next->f, FACE_OUT);
  }
#endif

  /* Now, update the node value in the heap, which may cause the node
   * to be moved toward the head of the priority queue. */
  BLI_heap_node_value_update(s.heap, node, new_join_error);
}

/** Given a face, find merge_edges which are being considered for merge and improve them
 *
 * \param s: State information about the join_triangles process
 * \param face       A quad
 * \param face_error The current error of the face.
 */
static void reprioritize_face_neighbors(JoinEdgesState &s, BMFace *face, float face_error)
{
  BLI_assert(face);
  BLI_assert(face->len == 4);

  /* Identify any mergable edges of any neighbor triangles that face us.
   * - Some of our four edges... might not be manifold.
   * - Some of our neighbors... might not be triangles.
   * - Some of our neighbor triangles... might have other nonmanifold (unmergable) edges.
   * - Some of our neighbor triangles' manifold edges... might have non-traingle neighbors.
   * Therefore... We can have UP TO EIGHT mergable edges, but we often see less. */
  size_t neighbor_count = 0;
  BMEdge *merge_edges[8];
  BMLoop *shared_loops[8];

  /* Get the four loops around the face */
  BMLoop *l_a = face->l_first;
  BMLoop *l_b = l_a->next;
  BMLoop *l_c = l_b->next;
  BMLoop *l_d = l_c->next;

  /* Add the mergable neighbors for each of those loops. */
  neighbor_count = add_neighbors(merge_edges, shared_loops, neighbor_count, l_a);
  neighbor_count = add_neighbors(merge_edges, shared_loops, neighbor_count, l_b);
  neighbor_count = add_neighbors(merge_edges, shared_loops, neighbor_count, l_c);
  neighbor_count = add_neighbors(merge_edges, shared_loops, neighbor_count, l_d);

  /* Return if there is nothing to do. */
  if (neighbor_count == 0) {
    return;
  }

  /* Compute the four unit vectors around this quad */
  float quad_vecs[4][3];
  sub_v3_v3v3(quad_vecs[0], l_a->v->co, l_b->v->co);
  sub_v3_v3v3(quad_vecs[1], l_b->v->co, l_c->v->co);
  sub_v3_v3v3(quad_vecs[2], l_c->v->co, l_d->v->co);
  sub_v3_v3v3(quad_vecs[3], l_d->v->co, l_a->v->co);
  normalize_v3(quad_vecs[0]);
  normalize_v3(quad_vecs[1]);
  normalize_v3(quad_vecs[2]);
  normalize_v3(quad_vecs[3]);

  /* Reprioritize each neighbor. */
  for (int n = 0; n < neighbor_count; n++) {
#ifdef USE_JOIN_TRIANGLE_INTERACTIVE_TESTING
    s.debug_this_step = (s.debug_merge_cap > 0 && s.debug_merge_count == s.debug_merge_cap &&
                         n + 1 == s.debug_neighbor) ||
                        (++s.debug_neighbor_global_count == s.debug_neighbor &&
                         s.debug_merge_cap == 0);
#endif
    reprioritize_join(s, merge_edges[n], shared_loops[n], quad_vecs, face_error, face->no);
  }
}
/** Given a manifold edge, join the triangles on either side to form a quad.
 *
 * \param s: State information about the join_triangles process
 * \param e: the edge to merge.  It must be manifold.
 * \return the face that resulted, or nullptr if the merege was rejected.
 */
static BMFace *join_edge(JoinEdgesState &s, BMEdge *e)
{
  /* Nonmanifold edges can't be merged. */
  BLI_assert(e);
  BLI_assert(BM_edge_is_manifold(e));

  /* Identify the loops on either side of the joinable edge */
  BMLoop *l_a = e->l;
  BMLoop *l_b = e->l->radial_next;

  /* If previous face merges have created quads, which now make this edge un-mergable, then skip it
   * and move on. This DOES happen frequently and that's ok.  It's much easier and more efficient
   * to just skip these edges when we encounter them, than it is to try to search the heap for them
   * and remove them preemptively. */
  if ((l_a->f->len != 3) || (l_b->f->len != 3)) {
    return nullptr;
  }

#ifdef USE_JOIN_TRIANGLE_INTERACTIVE_TESTING
  /* Stop doing merges in the middle of processing if we reached a user limit.
   * This is allowed so a developer can check steps in the process of the algorithm. */
  if (++s.debug_merge_count > s.debug_merge_cap && s.debug_merge_cap != -1) {
    return nullptr;
  }
#endif

  /* Join the edge and identify the face */
  BMFace *face_new = BM_faces_join_pair(s.bm, l_a, l_b, true);

  if (face_new) {

    /* Tag the face so the selection can be extended to include the new face. */
    if (s.select_tris_only == false) {
      BMO_face_flag_enable(s.bm, face_new, FACE_OUT);
    }
  }

#ifdef USE_JOIN_TRIANGLE_INTERACTIVE_TESTING
  /* If stopping partway through, clear the selection entirely, and instead
   * highlight the faces being considered in the step the user is checking.  */
  if (s.debug_merge_cap != -1 && s.debug_merge_count == s.debug_merge_cap) {
    BMEdge *face;
    BMIter iter;
    BM_ITER_MESH (face, &iter, s.bm, BM_FACES_OF_MESH) {
      BMO_face_flag_disable(s.bm, face, FACE_OUT);
    }
    BMO_face_flag_enable(s.bm, face_new, FACE_OUT);
  }
#endif

  return face_new;
}

/** Given a mesh, convert triangles to quads. */
void bmo_join_triangles_exec(BMesh *bm, BMOperator *op)
{
  BMOIter siter;
  BMFace *f;

  BMIter iter;
  BMEdge *e;

  DelimitData delimit_data = bm_edge_delmimit_data_from_op(bm, op);

  /* initial setup of state */
  JoinEdgesState s;
  s.bm = bm;
  s.topo_influnce = BMO_slot_float_get(op->slots_in, "topology_influence") / 100;
  s.use_topo_influence = (s.topo_influnce != 0);
  s.heap = BLI_heap_new();
  s.index.clear_and_shrink();
  s.select_tris_only = BMO_slot_bool_get(op->slots_in, "select_leftover_triangles");

#ifdef USE_JOIN_TRIANGLE_INTERACTIVE_TESTING
  s.debug_merge_cap = BMO_slot_int_get(op->slots_in, "merge_cap") - 1;
  s.debug_neighbor = BMO_slot_int_get(op->slots_in, "neighbor_debug");
  s.debug_merge_count = 0;
  s.debug_neighbor_global_count = 0;
#endif

  /* Go through every face in the input slot. Mark triangles for processing. */
  BMO_ITER (f, &siter, op->slots_in, "faces", BM_FACE) {
    if (f->len == 3) {
      BMO_face_flag_enable(bm, f, FACE_INPUT);

      /* And setup the initial selction */
      if (s.select_tris_only) {
        BMO_face_flag_enable(s.bm, f, FACE_OUT);
      }
    }
  }

  /* Go through every edge in the bmesh, Mark any mergable edges. */
  BM_ITER_MESH (e, &iter, bm, BM_EDGES_OF_MESH) {
    BMFace *f_a, *f_b;

    /* If the edge is manifold, has a tagged input tri on both sides, and is NOT delmit ...
     * then it's a candidate to merge.*/
    if (BM_edge_face_pair(e, &f_a, &f_b) && BMO_face_flag_test(bm, f_a, FACE_INPUT) &&
        BMO_face_flag_test(bm, f_b, FACE_INPUT) && !bm_edge_is_delimit(e, &delimit_data))
    {
      /* Compute the error that would result from a merge */
      const BMVert *verts[4];
      bm_edge_to_quad_verts(e, verts);
      float merge_error = quad_calc_error(verts[0]->co, verts[1]->co, verts[2]->co, verts[3]->co);

      /* Record the candidate merge in both the heap, and the heap index. */
      HeapNode *node = BLI_heap_insert(s.heap, merge_error, e);
      s.index.add_new(e, node);
    }
  }

  /* Go through all the the faces of the input slot, this time to find quads.
   * Improve the candidates around any preexisting quads in the mesh.
   *
   * NOTE:  This unfortunately misses any quads which are not selected, but
   * which neighbor the selection.  The only alternate would be to iterate the
   * whole mesh, which might be expensive for very large meshes with small selections.
   */
  if (!s.index.is_empty() && s.use_topo_influence) {
    BMO_ITER (f, &siter, op->slots_in, "faces", BM_FACE) {
      if (f->len == 4) {
        BMLoop *l_a = f->l_first;
        BMLoop *l_b = l_a->next;
        BMLoop *l_c = l_b->next;
        BMLoop *l_d = l_c->next;

        /* Flat quads with right angle corners and no concavity have lower error. */
        float error = quad_calc_error(l_a->v->co, l_b->v->co, l_c->v->co, l_d->v->co);

        /* Since we're early in the process we over-prioritize any already existing quads to
         * allow them to have an especially strong influence on the resulting mesh.
         * At a topology influence of 200%, they're considered to be *almost perfect* quads
         * reguardless of their acutal error.  Either way, the multiplier is never completely
         * allowed to reach reach zero.  Intead, 1% of the original error is preserved...
         * which is enough to maintain the relative priority sorting between existing quads. */
        float compensated_errror = error * (2 - s.topo_influnce * maximum_improvement);

        reprioritize_face_neighbors(s, f, compensated_errror);
      }
    }
  }

  /* Process all possible merges */
  while (!BLI_heap_is_empty(s.heap)) {

    /* Get the best merge from the priority queue.
     * Remove it from the both priority queue and the index. */
    float error = BLI_heap_top_value(s.heap);
    BMEdge *edge = reinterpret_cast<BMEdge *>(BLI_heap_pop_min(s.heap));
    s.index.remove(edge);

    /* Attempt the merge. */
    BMFace *new_face = join_edge(s, edge);

    /* If the merge succeeded, improve the neighbors */
    if (new_face && s.use_topo_influence) {
      reprioritize_face_neighbors(s, new_face, error);
    }

#ifdef USE_JOIN_TRIANGLE_INTERACTIVE_TESTING
    /* If we're supposed to stop partway though, do that.
     * This allows a developer to inspect the mesh at intermediate stages of processing. */
    if (s.debug_merge_cap != -1 && s.debug_merge_count >= s.debug_merge_cap) {
      break;
    }
#endif
  }

#ifdef USE_JOIN_TRIANGLE_INTERACTIVE_TESTING
  /* Expect a full processing to have occurred, ONLY if we didn't stop partway through. */
  if (!(s.debug_merge_cap != -1 && s.debug_merge_count >= s.debug_merge_cap)) {
    BLI_assert(BLI_heap_is_empty(s.heap));
    BLI_assert(s.index.is_empty());
  }
#else
  /* Expect a full processing to have occurred */
  BLI_assert(BLI_heap_is_empty(s.heap));
  BLI_assert(s.index.is_empty());
#endif

  /* Clean up. */
  BLI_heap_free(s.heap, nullptr);
  s.index.clear_and_shrink();

  /* Return the selection results. */
  BMO_slot_buffer_from_enabled_flag(bm, op, op->slots_out, "faces.out", BM_FACE, FACE_OUT);
}
