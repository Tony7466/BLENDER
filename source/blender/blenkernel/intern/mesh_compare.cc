/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_array.hh"
#include "BLI_math_base.h"
#include "BLI_span.hh"

#include "BKE_attribute.hh"
#include "BKE_attribute_math.hh"
#include "BKE_mesh.hh"
#include "BKE_mesh_mapping.hh"

#include "BKE_mesh_compare.hh"

namespace blender::bke::mesh {

static bool edges_equal(const int2 e1,
                        const int2 e2,
                        const Span<int> verts1,
                        const Span<int> verts2,
                        const Span<int> group_ids)
{
  return (group_ids[verts1[e1.x]] == group_ids[verts2[e2.x]] &&
          group_ids[verts1[e1.y]] == group_ids[verts2[e2.y]]) ||
         (group_ids[verts1[e1.x]] == group_ids[verts2[e2.y]] &&
          group_ids[verts1[e1.y]] == group_ids[verts2[e2.x]]);
}

static bool faces_equal(const IndexRange f1,
                        const IndexRange f2,
                        const Span<int> corner_verts1,
                        const Span<int> corner_verts2,
                        const Span<int> verts1,
                        const Span<int> verts2,
                        const Span<int> group_ids)
{
  if (f1.size() != f2.size()) {
    return false;
  }
  /* If the two faces are equal, they should have the same ordering. Only possible difference is
   * the starting vertex. */
  for (const int offset : f1.index_range()) {
    bool faces_match = true;
    for (const int i : f1.index_range()) {
      const int c1 = f1[i];
      const int c2 = f2[(i + offset) % f1.size()];
      if (group_ids[verts1[corner_verts1[c1]]] != group_ids[verts2[corner_verts2[c2]]]) {
        faces_match = false;
        break;
      }
    }
    if (faces_match) {
      return true;
    }
  }
  return false;
}

/**
 * Sort the indices using the values.
 */
template<typename T> static void sort_indices(MutableSpan<int> indices, const Span<T> &values)
{
  std::stable_sort(
      indices.begin(), indices.end(), [&](int i1, int i2) { return values[i1] < values[i2]; });
}

/**
 * Tries to construct a (bijective) mapping from the vertices of the first mesh to the
 * vertices of the second mesh, such that:
 *  - Edge topology is preserved under this mapping, i.e. if v_1 and v_2 are on an edge in mesh1
 * then f(v_1) and f(v_2) are on an edge in mesh2.
 *  - Face topology is preserved under this mapping, i.e. if v_1, ..., v_n form a face in mesh1,
 * then f(v_1), ..., f(v_n) form a face in mesh2.
 *  - The mapping preserves all vertex attributes, i.e. if attr is some vertex attribute on mesh1,
 * then for every vertex v of mesh1, attr(v) = attr(f(v)).
 *
 * \returns the type of mismatch that occured if the mapping couldn't be constructed.
 */
static std::optional<MeshMismatch> construct_vertex_mapping(
    const Mesh &mesh1, const Mesh &mesh2, MutableSpan<int> r_verts1_to_verts2_map)
{
  /* At first we don't have any information on which vertices correspond. We try to iteratively
   * narrow down groups of vertices which could correspond. As a first step, we use the vertex
   * positions. */

  Array<int> verts1(mesh1.totvert);
  Array<int> verts2(mesh2.totvert);
  Span<float3> vert_positions1 = mesh1.vert_positions();
  Span<float3> vert_positions2 = mesh2.vert_positions();
  std::iota(verts1.begin(), verts1.end(), 0);
  std::iota(verts2.begin(), verts2.end(), 0);
  sort_indices(verts1, vert_positions1);
  sort_indices(verts2, vert_positions2);

  /* We can now narrow down the vertices into groups based on their positions.*/

  /* The id of each group is the index of its first element. */
  Array<int> group_ids(mesh1.totvert);
  float3 previous_pos = vert_positions1[verts1.first()];
  int group_id = 0;
  for (const int i : IndexRange(mesh1.totvert)) {
    float3 pos1 = vert_positions1[verts1[i]];
    float3 pos2 = vert_positions2[verts2[i]];
    if (!compare_v3v3_relative(pos1, pos2, FLT_EPSILON, 64)) {
      /* After sorting, the vertices should have the same positions. */
      return MeshMismatch::VertexAttributes;
    }
    if (!compare_v3v3_relative(pos1, previous_pos, FLT_EPSILON, 64)) {
      /* Different from the previous one, so in a new group. */
      group_id = i;
    }
    group_ids[i] = group_id;
    previous_pos = pos1;
  }

  Array<int> group_sizes(mesh1.totvert);
  int i = mesh1.totvert - 1;
  while (i >= 0) {
    /* The size of the group is the index of its last element + 1. */
    int group_size = group_ids[i] + 1;
    /* Set the group size for each element in the group. */
    for (int k = i - group_size + 1; k <= i; k++) {
      group_sizes[k] = group_size;
    }
    i -= group_size;
  }

  /* Let's try and compare edges. */
  Array<int> vert_to_edge_offsets1;
  Array<int> vert_to_edge_indices1;
  GroupedSpan<int> vert_to_edge_map1;
  vert_to_edge_map1 = bke::mesh::build_vert_to_edge_map(
      mesh1.edges(), mesh1.totvert, vert_to_edge_offsets1, vert_to_edge_indices1);
  Array<int> vert_to_edge_offsets2;
  Array<int> vert_to_edge_indices2;
  GroupedSpan<int> vert_to_edge_map2;
  vert_to_edge_map2 = bke::mesh::build_vert_to_edge_map(
      mesh2.edges(), mesh2.totvert, vert_to_edge_offsets2, vert_to_edge_indices2);

  /* Check that for each vertex in the first mesh, we can find a matching vertex in the second mesh
   * in terms of edge connectivity. */
  for (const int i : IndexRange(mesh1.totvert)) {
    const int v1 = verts1[i];
    const Span<int> edges1 = vert_to_edge_map1[v1];
    bool matching_vertex_found = false;
    /* Try to find a matching vertex. We know that if it exists, it is in the same group. */
    for (const int group_i : IndexRange(group_sizes[i])) {
      const int v2 = verts2[group_i + group_ids[i]];
      const Span<int> edges2 = vert_to_edge_map2[v2];
      if (edges1.size() != edges2.size()) {
        continue;
      }
      matching_vertex_found = true;
      for (const int e1 : edges1) {
        bool found_matching_edge = false;
        for (const int e2 : edges2) {
          if (edges_equal(mesh1.edges()[e1], mesh2.edges()[e2], verts1, verts2, group_ids)) {
            found_matching_edge = true;
            break;
          }
        }
        if (!found_matching_edge) {
          matching_vertex_found = false;
          break;
        }
      }
      if (matching_vertex_found) {
        break;
      }
    }

    /* TODO: use which vertices matched to reduce groups even further. (All the vertices that match
     * form a new group.)*/

    if (!matching_vertex_found) {
      return MeshMismatch::EdgeTopology;
    }
  }

  /* Finally we check the faces. */
  const GroupedSpan<int> vert_to_corner_map1 = mesh1.vert_to_corner_map();
  const GroupedSpan<int> vert_to_corner_map2 = mesh2.vert_to_corner_map();
  const Array<int> corner_to_face_map1 = mesh1.corner_to_face_map();
  const Array<int> corner_to_face_map2 = mesh2.corner_to_face_map();

  /* Analogously to the previous check, we now check if we can match vertices based on the faces.
   */
  for (const int i : IndexRange(mesh1.totvert)) {
    const int v1 = verts1[i];
    const Span<int> corners1 = vert_to_corner_map1[v1];
    bool matching_vertex_found = false;
    /* Try to find a matching vertex. We know that if it exists, it is in the same group. */
    for (const int group_i : IndexRange(group_sizes[i])) {
      const int v2 = verts2[group_i + group_ids[i]];
      const Span<int> corners2 = vert_to_corner_map2[v2];
      if (corners1.size() != corners2.size()) {
        continue;
      }
      matching_vertex_found = true;
      for (const int c1 : corners1) {
        bool found_matching_corner = false;
        const int f1 = corner_to_face_map1[c1];
        for (const int c2 : corners2) {
          const int f2 = corner_to_face_map2[c2];
          if (faces_equal(mesh1.faces()[f1],
                          mesh2.faces()[f2],
                          mesh1.corner_verts(),
                          mesh2.corner_verts(),
                          verts1,
                          verts2,
                          group_ids))
          {
            found_matching_corner = true;
            break;
          }
        }
        if (!found_matching_corner) {
          matching_vertex_found = false;
          break;
        }
      }
      if (matching_vertex_found) {
        break;
      }
    }

    /* TODO: use which vertices matched to reduce groups even further. (All the vertices that match
     * form a new group.)*/

    if (!matching_vertex_found) {
      return MeshMismatch::FaceTopology;
    }
  }

  return {};
}

std::optional<MeshMismatch> meshes_isomorphic(const Mesh &mesh1, const Mesh &mesh2)
{

  /* These will be assumed implicitly later on. */
  if (mesh1.totvert != mesh2.totvert) {
    return MeshMismatch::NumVerts;
  }
  if (mesh1.totedge != mesh2.totedge) {
    return MeshMismatch::NumEdges;
  }
  if (mesh1.totloop != mesh2.totloop) {
    return MeshMismatch::NumCorners;
  }
  if (mesh1.faces_num != mesh2.faces_num) {
    return MeshMismatch::NumFaces;
  }

  /* We first try to construct a bijection between the vertices, since edges, corners and faces are
   * dependent on vertex indices. */
  Array<int> verts1_to_verts2_map(mesh1.totvert);
  return construct_vertex_mapping(mesh1, mesh2, verts1_to_verts2_map);
}

}  // namespace blender::bke::mesh
