/* SPDX-FileCopyrightText: 2020 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edsculpt
 */

#include <cmath>
#include <cstdlib>

#include <functional>
#include <mutex>
#include <tbb/concurrent_vector.h>
#include <thread>

#include "MEM_guardedalloc.h"

#include "BLI_linklist_stack.h"
#include "BLI_math_geom.h"
#include "BLI_math_vector.h"
#include "BLI_task.h"

#include "DNA_brush_types.h"
#include "DNA_mesh_types.h"
#include "DNA_object_types.h"

#include "BKE_ccg.h"
#include "BKE_context.hh"
#include "BKE_mesh.hh"
#include "BKE_mesh_mapping.hh"
#include "BKE_object.hh"
#include "BKE_paint.hh"
#include "BKE_pbvh_api.hh"

#include "paint_intern.hh"
#include "sculpt_intern.hh"

#include "bmesh.hh"

#define SCULPT_GEODESIC_VERTEX_NONE -1

namespace blender::ed::sculpt_paint::geodesic {

/* Propagate distance from v1 and v2 to v0. */
static bool sculpt_geodesic_mesh_test_dist_add(Span<float3> vert_positions,
                                               const int v0,
                                               const int v1,
                                               const int v2,
                                               float *dists,
                                               GSet *initial_verts)
{
  if (BLI_gset_haskey(initial_verts, POINTER_FROM_INT(v0))) {
    return false;
  }

  BLI_assert(dists[v1] != FLT_MAX);
  if (dists[v0] <= dists[v1]) {
    return false;
  }

  float dist0;
  if (v2 != SCULPT_GEODESIC_VERTEX_NONE) {
    BLI_assert(dists[v2] != FLT_MAX);
    if (dists[v0] <= dists[v2]) {
      return false;
    }
    dist0 = geodesic_distance_propagate_across_triangle(
        vert_positions[v0], vert_positions[v1], vert_positions[v2], dists[v1], dists[v2]);
  }
  else {
    float vec[3];
    sub_v3_v3v3(vec, vert_positions[v1], vert_positions[v0]);
    dist0 = dists[v1] + len_v3(vec);
  }

  if (dist0 < dists[v0]) {
    dists[v0] = dist0;
    return true;
  }

  return false;
}

// TODO: replace by geodesic_mesh_create_parallel is polished
static float *geodesic_mesh_create(Object *ob, GSet *initial_verts, const float limit_radius)
{
  SculptSession *ss = ob->sculpt;
  Mesh *mesh = BKE_object_get_original_mesh(ob);

  const int totvert = mesh->verts_num;
  const int totedge = mesh->edges_num;

  const float limit_radius_sq = limit_radius * limit_radius;

  const Span<float3> vert_positions = SCULPT_mesh_deformed_positions_get(ss);
  const Span<int2> edges = mesh->edges();
  const OffsetIndices faces = mesh->faces();
  const Span<int> corner_verts = mesh->corner_verts();
  const Span<int> corner_edges = mesh->corner_edges();
  const bke::AttributeAccessor attributes = mesh->attributes();
  const VArraySpan<bool> hide_poly = *attributes.lookup<bool>(".hide_poly", bke::AttrDomain::Face);

  float *dists = static_cast<float *>(MEM_malloc_arrayN(totvert, sizeof(float), __func__));
  BitVector<> edge_tag(totedge);

  if (ss->edge_to_face_map.is_empty()) {
    ss->edge_to_face_map = bke::mesh::build_edge_to_face_map(
        faces, corner_edges, edges.size(), ss->edge_to_face_offsets, ss->edge_to_face_indices);
  }
  if (ss->vert_to_edge_map.is_empty()) {
    ss->vert_to_edge_map = bke::mesh::build_vert_to_edge_map(
        edges, mesh->verts_num, ss->vert_to_edge_offsets, ss->vert_to_edge_indices);
  }

  /* Both contain edge indices encoded as *void. */
  BLI_LINKSTACK_DECLARE(queue, void *);
  BLI_LINKSTACK_DECLARE(queue_next, void *);

  BLI_LINKSTACK_INIT(queue);
  BLI_LINKSTACK_INIT(queue_next);

  for (int i = 0; i < totvert; i++) {
    if (BLI_gset_haskey(initial_verts, POINTER_FROM_INT(i))) {
      dists[i] = 0.0f;
    }
    else {
      dists[i] = FLT_MAX;
    }
  }

  /* Masks vertices that are further than limit radius from an initial vertex. As there is no need
   * to define a distance to them the algorithm can stop earlier by skipping them. */
  BitVector<> affected_vert(totvert);
  GSetIterator gs_iter;

  if (limit_radius == FLT_MAX) {
    /* In this case, no need to loop through all initial vertices to check distances as they are
     * all going to be affected. */
    affected_vert.fill(true);
  }
  else {
    /* This is an O(n^2) loop used to limit the geodesic distance calculation to a radius. When
     * this optimization is needed, it is expected for the tool to request the distance to a low
     * number of vertices (usually just 1 or 2). */
    GSET_ITER (gs_iter, initial_verts) {
      const int v = POINTER_AS_INT(BLI_gsetIterator_getKey(&gs_iter));
      const float *v_co = vert_positions[v];
      for (int i = 0; i < totvert; i++) {
        if (len_squared_v3v3(v_co, vert_positions[i]) <= limit_radius_sq) {
          affected_vert[i].set();
        }
      }
    }
  }

  /* Add edges adjacent to an initial vertex to the queue. */
  for (int i = 0; i < totedge; i++) {
    const int v1 = edges[i][0];
    const int v2 = edges[i][1];
    if (!affected_vert[v1] && !affected_vert[v2]) {
      continue;
    }
    if (dists[v1] != FLT_MAX || dists[v2] != FLT_MAX) {
      BLI_LINKSTACK_PUSH(queue, POINTER_FROM_INT(i));
    }
  }

  do {
    while (BLI_LINKSTACK_SIZE(queue)) {
      const int e = POINTER_AS_INT(BLI_LINKSTACK_POP(queue));
      int v1 = edges[e][0];
      int v2 = edges[e][1];

      if (dists[v1] == FLT_MAX || dists[v2] == FLT_MAX) {
        if (dists[v1] > dists[v2]) {
          std::swap(v1, v2);
        }
        sculpt_geodesic_mesh_test_dist_add(
            vert_positions, v2, v1, SCULPT_GEODESIC_VERTEX_NONE, dists, initial_verts);
      }

      for (const int face : ss->edge_to_face_map[e]) {
        if (!hide_poly.is_empty() && hide_poly[face]) {
          continue;
        }
        for (const int v_other : corner_verts.slice(faces[face])) {
          if (ELEM(v_other, v1, v2)) {
            continue;
          }
          if (sculpt_geodesic_mesh_test_dist_add(
                  vert_positions, v_other, v1, v2, dists, initial_verts))
          {
            for (const int e_other : ss->vert_to_edge_map[v_other]) {
              int ev_other;
              if (edges[e_other][0] == v_other) {
                ev_other = edges[e_other][1];
              }
              else {
                ev_other = edges[e_other][0];
              }

              if (e_other != e && !edge_tag[e_other] &&
                  (ss->edge_to_face_map[e_other].is_empty() || dists[ev_other] != FLT_MAX))
              {
                if (affected_vert[v_other] || affected_vert[ev_other]) {
                  edge_tag[e_other].set();
                  BLI_LINKSTACK_PUSH(queue_next, POINTER_FROM_INT(e_other));
                }
              }
            }
          }
        }
      }
    }

    for (LinkNode *lnk = queue_next; lnk; lnk = lnk->next) {
      const int e = POINTER_AS_INT(lnk->link);
      edge_tag[e].reset();
    }

    BLI_LINKSTACK_SWAP(queue, queue_next);

  } while (BLI_LINKSTACK_SIZE(queue));

  BLI_LINKSTACK_FREE(queue);
  BLI_LINKSTACK_FREE(queue_next);

  return dists;
}

/* Parallel vector implementation with multiple writting heads
 for fast parallel writting that minimizes collisions and race
 conditions

 Use it in places where the writting order is not important and
 can tolerate some collission in writting data */
template<typename T> class UnorderedParallelVector {
 private:
  int num_threads;
  std::vector<std::vector<T>> data_heads;

 public:
  UnorderedParallelVector(size_t SIZE, int threads = 4) : num_threads(threads)
  {
    num_threads = std::max(1, threads);
    data_heads.reserve(num_threads);
    const int chunk_size = SIZE / num_threads + 1;
    for (size_t i = 0; i < num_threads; ++i) {
      data_heads.push_back(std::vector<T>());
      data_heads[i].reserve(chunk_size);
    }
  }

  void push_back(T value)
  {
    /* Assign a writing head based on thread ID */
    size_t head_id = std::hash<std::thread::id>{}(std::this_thread::get_id()) % num_threads;
    data_heads[head_id].push_back(value);
  }

  size_t size() const
  {
    size_t max_index = 0;
    for (size_t i = 0; i < num_threads; ++i) {
      max_index += data_heads[i].size();
    }
    return max_index;
  }

  bool empty()
  {
    return size() == 0;
  }

  void clear()
  {
    for (size_t i = 0; i < num_threads; ++i) {
      data_heads[i].clear();
    }
  }

  class iterator {
   private:
    UnorderedParallelVector *ptr;
    size_t col, row;

   public:
    iterator(UnorderedParallelVector *pv, size_t r, size_t c) : ptr(pv), col(c), row(r) {}

    /* Prefix increment operator (++it) */
    iterator &operator++()
    {
      ++col;
      if (col >= ptr->data_heads[row].size()) {
        col = 0;
        /* interleaved buffers can be empty */
        do {
          ++row;
        } while (row < ptr->num_threads && ptr->data_heads[row].empty());

        if (row >= ptr->num_threads) {
          row = ptr->num_threads - 1;
        }
      }
      return *this;
    }

    /* Postfix increment operator (it++) */
    iterator operator++(int)
    {
      iterator temp = *this;
      ++(*this);
      return temp;
    }

    /* Dereference operator */
    int &operator*() const
    {
      return ptr->data_heads[row][col];
    }

    bool operator==(const iterator &other) const
    {
      return ptr == other.ptr && row == other.row && col == other.col;
    }

    bool operator!=(const iterator &other) const
    {
      return !(*this == other);
    }
  };

  iterator begin()
  {
    /* go forward checking the storage vectors sizes */
    int front_index = 0;
    while (this->data_heads[front_index].size() == 0) {
      ++front_index;
    }
    if (front_index >= num_threads) {
      return end();
    }
    return iterator(this, front_index, 0);
  }

  iterator end()
  {
    /* go backwards checking the storage vectors sizes */
    int back_index = num_threads - 1;
    while (back_index > 0 && this->data_heads[back_index].size() == 0) {
      --back_index;
    }
    if (back_index <= 0 && this->data_heads[0].size() == 0) {
      return iterator(this, 0, 0);
    }
    return iterator(this, back_index, this->data_heads[back_index].size() - 1);
  }
};

static float *geodesic_mesh_create_parallel(Object *ob,
                                            GSet *initial_verts,
                                            const float limit_radius)
{
  SculptSession *ss = ob->sculpt;
  Mesh *mesh = BKE_object_get_original_mesh(ob);

  const int totvert = mesh->verts_num;
  const int totedge = mesh->edges_num;

  const Span<float3> vert_positions = SCULPT_mesh_deformed_positions_get(ss);
  const Span<int2> edges = mesh->edges();
  const OffsetIndices faces = mesh->faces();
  const Span<int> corner_verts = mesh->corner_verts();
  const Span<int> corner_edges = mesh->corner_edges();
  const bke::AttributeAccessor attributes = mesh->attributes();
  const VArraySpan<bool> hide_poly = *attributes.lookup<bool>(".hide_poly", bke::AttrDomain::Face);
  float *dists = static_cast<float *>(MEM_malloc_arrayN(totvert, sizeof(float), __func__));

  if (ss->edge_to_face_map.is_empty()) {
    ss->edge_to_face_map = bke::mesh::build_edge_to_face_map(
        faces, corner_edges, edges.size(), ss->edge_to_face_offsets, ss->edge_to_face_indices);
  }
  if (ss->vert_to_edge_map.is_empty()) {
    ss->vert_to_edge_map = bke::mesh::build_vert_to_edge_map(
        edges, mesh->verts_num, ss->vert_to_edge_offsets, ss->vert_to_edge_indices);
  }

  threading::parallel_for(IndexRange(0, totvert), 4096, [&](IndexRange range) {
    for (const int i : range) {
      if (BLI_gset_haskey(initial_verts, POINTER_FROM_INT(i))) {
        dists[i] = 0.0f;
      }
      else {
        dists[i] = FLT_MAX;
      }
    }
  });

  /* Masks vertices that are further than limit radius from an initial vertex. As there is no need
   * to define a distance to them the algorithm can stop earlier by skipping them. */
  BitVector<> affected_vert(totvert);
  if (limit_radius == FLT_MAX) {
    /* In this case, no need to loop through all initial vertices to check distances as they are
     * all going to be affected. */
    affected_vert.fill(true);
  }
  else {
    const float limit_radius_sq = limit_radius * limit_radius;
    GSetIterator gs_iter;
    /* This is an O(n^2) loop used to limit the geodesic distance calculation to a radius. When
     * this optimization is needed, it is expected for the tool to request the distance to a low
     * number of vertices (usually just 1 or 2). */
    GSET_ITER (gs_iter, initial_verts) {
      const int v = POINTER_AS_INT(BLI_gsetIterator_getKey(&gs_iter));
      const float *v_co = vert_positions[v];
      threading::parallel_for(IndexRange(0, totvert), 4096, [&](IndexRange range) {
        for (const int i : range) {
          if (len_squared_v3v3(v_co, vert_positions[i]) <= limit_radius_sq) {
            affected_vert[i].set();
          }
        }
      });
    }
  }

  std::vector<int> queue;
  queue.reserve(totedge);
  UnorderedParallelVector<int> queue_next(totedge, std::thread::hardware_concurrency() / 2);

  /* Add edges adjacent to an initial vertex to the queue.
   Since initial vertex are few only, iterating over its neighbour edges
   instead of over all edges scales better as mesh edge count increases */
  GSetIterator gs_iter;
  GSET_ITER (gs_iter, initial_verts) {
    const int seed_vert = POINTER_AS_INT(BLI_gsetIterator_getKey(&gs_iter));
    for (const int e : ss->vert_to_edge_map[seed_vert]) {
      const int v1 = edges[e][0];
      const int v2 = edges[e][1];
      if ((affected_vert[v1] || affected_vert[v2]) &&
          (dists[v1] != FLT_MAX || dists[v2] != FLT_MAX))
      {
        queue.push_back(e);
      }
    }
  }

  BitVector<> edge_tag(totedge);
  while (!queue.empty()) {
    threading::parallel_for_each(IndexRange(0, queue.size()), [&](const int val) {
      const int e = queue[val];
      int v1 = edges[e][0];
      int v2 = edges[e][1];

      if (dists[v1] == FLT_MAX || dists[v2] == FLT_MAX) {
        if (dists[v1] > dists[v2]) {
          std::swap(v1, v2);
        }
        sculpt_geodesic_mesh_test_dist_add(
            vert_positions, v2, v1, SCULPT_GEODESIC_VERTEX_NONE, dists, initial_verts);
      }

      for (const int face : ss->edge_to_face_map[e]) {
        if (!hide_poly.is_empty() && hide_poly[face]) {
          continue;
        }
        for (const int v_other : corner_verts.slice(faces[face])) {
          if (ELEM(v_other, v1, v2) || !sculpt_geodesic_mesh_test_dist_add(
                                           vert_positions, v_other, v1, v2, dists, initial_verts))
          {
            continue;
          }

          for (const int e_other : ss->vert_to_edge_map[v_other]) {
            const int ev_other = (edges[e_other][0] == v_other) ? edges[e_other][1] :
                                                                  edges[e_other][0];

            if (!(affected_vert[v_other] || affected_vert[ev_other]) || e_other == e ||
                edge_tag[e_other] ||
                (!ss->edge_to_face_map[e_other].is_empty() && dists[ev_other] == FLT_MAX))
            {
              continue;
            }

            edge_tag[e_other].set();
            queue_next.push_back(e_other);
          }
        }
      }
    });

    queue.clear();
    for (auto it = queue_next.begin(); it != queue_next.end(); ++it) {
      edge_tag[*it].reset();
      queue.push_back(*it);
    }
    queue_next.clear();
  }

  return dists;
}

/* For sculpt mesh data that does not support a geodesic distances algorithm, fallback to the
 * distance to each vertex. In this case, only one of the initial vertices will be used to
 * calculate the distance. */
static float *geodesic_fallback_create(Object *ob, GSet *initial_verts)
{
  SculptSession *ss = ob->sculpt;
  Mesh *mesh = BKE_object_get_original_mesh(ob);
  const int totvert = mesh->verts_num;
  float *dists = static_cast<float *>(MEM_malloc_arrayN(totvert, sizeof(float), __func__));
  int first_affected = SCULPT_GEODESIC_VERTEX_NONE;
  GSetIterator gs_iter;
  GSET_ITER (gs_iter, initial_verts) {
    first_affected = POINTER_AS_INT(BLI_gsetIterator_getKey(&gs_iter));
    break;
  }

  if (first_affected == SCULPT_GEODESIC_VERTEX_NONE) {
    for (int i = 0; i < totvert; i++) {
      dists[i] = FLT_MAX;
    }
    return dists;
  }

  const float *first_affected_co = SCULPT_vertex_co_get(
      ss, BKE_pbvh_index_to_vertex(ss->pbvh, first_affected));
  for (int i = 0; i < totvert; i++) {
    PBVHVertRef vertex = BKE_pbvh_index_to_vertex(ss->pbvh, i);

    dists[i] = len_v3v3(first_affected_co, SCULPT_vertex_co_get(ss, vertex));
  }

  return dists;
}

float *distances_create(Object *ob, GSet *initial_verts, const float limit_radius)
{
  SculptSession *ss = ob->sculpt;
  switch (BKE_pbvh_type(ss->pbvh)) {
    case PBVH_FACES:
      return geodesic_mesh_create_parallel(
          ob,
          initial_verts,
          limit_radius);  // geodesic_mesh_create(ob, initial_verts, limit_radius);
    case PBVH_BMESH:
    case PBVH_GRIDS:
      return geodesic_fallback_create(ob, initial_verts);
  }
  BLI_assert_unreachable();
  return nullptr;
}

float *distances_create_from_vert_and_symm(Object *ob,
                                           const PBVHVertRef vertex,
                                           const float limit_radius)
{
  SculptSession *ss = ob->sculpt;
  GSet *initial_verts = BLI_gset_int_new("initial_verts");

  const char symm = SCULPT_mesh_symmetry_xyz_get(ob);
  for (char i = 0; i <= symm; ++i) {
    if (SCULPT_is_symmetry_iteration_valid(i, symm)) {
      PBVHVertRef v = {PBVH_REF_NONE};

      if (i == 0) {
        v = vertex;
      }
      else {
        float location[3];
        flip_v3_v3(location, SCULPT_vertex_co_get(ss, vertex), ePaintSymmetryFlags(i));
        v = SCULPT_nearest_vertex_get(ob, location, FLT_MAX, false);
      }
      if (v.i != PBVH_REF_NONE) {
        BLI_gset_add(initial_verts, POINTER_FROM_INT(BKE_pbvh_vertex_to_index(ss->pbvh, v)));
      }
    }
  }

  float *dists = distances_create(ob, initial_verts, limit_radius);
  BLI_gset_free(initial_verts, nullptr);
  return dists;
}

} /* namespace blender::ed::sculpt_paint::geodesic */
