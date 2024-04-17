/* SPDX-FileCopyrightText: 2020 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edsculpt
 */

#include <cmath>
#include <cstdlib>

#include "MEM_guardedalloc.h"

#include "BLI_enumerable_thread_specific.hh"
#include "BLI_linklist_stack.h"
#include "BLI_math_geom.h"
#include "BLI_math_vector.h"
#include "BLI_set.hh"
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

#include "BLI_timeit.hh"
#include <cassert>

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

  BLI_assert(dists[v1] != std::numeric_limits<float>::max());
  if (dists[v0] <= dists[v1]) {
    return false;
  }

  float dist0;
  if (v2 != SCULPT_GEODESIC_VERTEX_NONE) {
    BLI_assert(dists[v2] != std::numeric_limits<float>::max());
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
    dists[v0]= dist0;
    return true;
  }

  return false;
}

static float *geodesic_mesh_create(Object *ob, GSet *initial_verts, const float limit_radius)
{
  SCOPED_TIMER("geodesic_mesh_create");

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

  threading::parallel_for(IndexRange(totvert), 4096, [&](IndexRange range) {
    for (const int i : range) {
      if (BLI_gset_haskey(initial_verts, POINTER_FROM_INT(i))) {
        dists[i] = 0.0f;
      }
      else {
        dists[i] = std::numeric_limits<float>::max();
      }
    }
  });

  /* Masks vertices that are further than limit radius from an initial vertex. As there is no need
   * to define a distance to them the algorithm can stop earlier by skipping them. */
  Vector<bool> affected_vert;
  if (limit_radius == std::numeric_limits<float>::max()) {
    /* In this case, no need to loop through all initial vertices to check distances as they are
     * all going to be affected. */
    affected_vert.resize(totvert, true);
  }
  else {
    affected_vert.resize(totvert, false);
    const float limit_radius_sq = limit_radius * limit_radius;
    GSetIterator gs_iter;
    /* This is an O(n^2) loop used to limit the geodesic distance calculation to a radius. When
     * this optimization is needed, it is expected for the tool to request the distance to a low
     * number of vertices (usually just 1 or 2). */
    GSET_ITER (gs_iter, initial_verts) {
      const int v = POINTER_AS_INT(BLI_gsetIterator_getKey(&gs_iter));
      const float *v_co = vert_positions[v];
      threading::parallel_for(IndexRange(totvert), 4096, [&](IndexRange range) {
        for (const int i : range) {
          if (len_squared_v3v3(v_co, vert_positions[i]) <= limit_radius_sq) {
            affected_vert[i] = true;
          }
        }
      });
    }
  }

  Vector<int> queue;
  queue.reserve(totedge);

  /* Add edges adjacent to an initial vertex to the queue.
   * Since there are typically few initial vertices, iterating over its
   * neighbour edges instead of over all edges scales better
   * as mesh edge count increases. */
  GSetIterator gs_iter;
  GSET_ITER (gs_iter, initial_verts) {
    const int seed_vert = POINTER_AS_INT(BLI_gsetIterator_getKey(&gs_iter));
    for (const int edge_index : ss->vert_to_edge_map[seed_vert]) {
      const int v1 = edges[edge_index][0];
      const int v2 = edges[edge_index][1];
      if ((affected_vert[v1] || affected_vert[v2]) &&
          (dists[v1] != std::numeric_limits<float>::max() ||
           dists[v2] != std::numeric_limits<float>::max()))
      {
        queue.append(edge_index);
      }
    }
  }

#define USE_ORIGINAL_PR 0

  struct TLS {
    Set<int> new_edges;
  };
  threading::EnumerableThreadSpecific<TLS> all_tls;

  /* Bitmap which is used to ensure new edges are only scheduled once.
   * This is needed because multiple threads might want to schedule an edge, and we do not want
   * duplicates to be in the queue.
   * Using bitmask seems the fastest way of doing so. Alternatives like combining sets from TLS
   * is much slower. There might be different ideas here too. */
  BitVector<> is_edge_scheduled_map(totedge);

  while (!queue.is_empty()) {
    threading::parallel_for(IndexRange(queue.size()),128, [&](IndexRange range) {
      TLS &tls = all_tls.local();
      Set<int> &new_edges = tls.new_edges;

      for (const int value : range) {
        const int edge_index = queue[value];
        int edge_vert_a = edges[edge_index][0];
        int edge_vert_b = edges[edge_index][1];

        if (dists[edge_vert_a] == std::numeric_limits<float>::max() ||
            dists[edge_vert_b] == std::numeric_limits<float>::max())
        {
          if (dists[edge_vert_a] > dists[edge_vert_b]) {
            std::swap(edge_vert_a, edge_vert_b);
          }
          sculpt_geodesic_mesh_test_dist_add(vert_positions,
                                             edge_vert_b,
                                             edge_vert_a,
                                             SCULPT_GEODESIC_VERTEX_NONE,
                                             dists,
                                             initial_verts);
        }

        for (const int face : ss->edge_to_face_map[edge_index]) {
          if (!hide_poly.is_empty() && hide_poly[face]) {
            continue;
          }
          for (const int vertex_other : corner_verts.slice(faces[face])) {
            if (ELEM(vertex_other, edge_vert_a, edge_vert_b)) {
              continue;
            }
            if (!sculpt_geodesic_mesh_test_dist_add(
                    vert_positions, vertex_other, edge_vert_a, edge_vert_b, dists, initial_verts))
            {
              continue;
            }

            for (const int edge_other : ss->vert_to_edge_map[vertex_other]) {
              const int edge_vertex_other = bke::mesh::edge_other_vert(edges[edge_other],
                                                                       vertex_other);

              if (!(affected_vert[vertex_other] || affected_vert[edge_vertex_other]) ||
                  edge_other == edge_index || 
                  (!ss->edge_to_face_map[edge_other].is_empty() &&
                   dists[edge_vertex_other] == std::numeric_limits<float>::max()))
              {
                continue;
              }
              new_edges.add(edge_other);
            }
          }
        }
      }
    });

    queue.clear();
    for (TLS &tls : all_tls) {
      for (const int edge_index : tls.new_edges) {
        if (!is_edge_scheduled_map[edge_index]) {
          queue.append(edge_index);
          is_edge_scheduled_map[edge_index].set();
        }
      }
      tls.new_edges.clear();
    }
   // is_edge_scheduled_map.fill(false);
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
      dists[i] = std::numeric_limits<float>::max();
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
      return geodesic_mesh_create(ob, initial_verts, limit_radius);
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
        v = SCULPT_nearest_vertex_get(ob, location, std::numeric_limits<float>::max(), false);
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

}  // namespace blender::ed::sculpt_paint::geodesic
