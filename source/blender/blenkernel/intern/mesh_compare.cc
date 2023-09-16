/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_array.hh"
#include "BLI_math_base.h"
#include "BLI_ordered_edge.hh"
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
 * Sort the indices using the values.
 */
static void sort_indices_with_id_maps(MutableSpan<int> indices,
                                      const Span<int> &values,
                                      const Span<int> values_to_set,
                                      const Span<int> value_set_ids)
{
  std::stable_sort(indices.begin(), indices.end(), [&](int i1, int i2) {
    return value_set_ids[values_to_set[values[i1]]] < value_set_ids[values_to_set[values[i2]]];
  });
}

/* Sort the elements in each set based on the attribute values. */
template<typename T>
static void sort_per_set_based_on_attributes(const Span<int> set_sizes,
                                             MutableSpan<int> map1,
                                             MutableSpan<int> map2,
                                             const Span<T> values1,
                                             const Span<T> values2)
{
  int i = 0;
  while (i < map1.size()) {
    const int set_size = set_sizes[i];
    if (set_size == 1) {
      /* No need to sort anymore. */
      i += 1;
      continue;
    }

    sort_indices(map1.slice(IndexRange(i, set_size)), values1);
    sort_indices(map2.slice(IndexRange(i, set_size)), values2);
    i += set_size;
  }
}

/* Sort the elements in each set based on the set ids of the values. */
static void sort_per_set_with_id_maps(const Span<int> set_sizes,
                                      MutableSpan<int> map1,
                                      MutableSpan<int> map2,
                                      const Span<int> values1,
                                      const Span<int> values2,
                                      const Span<int> values1_to_set,
                                      const Span<int> values2_to_set,
                                      const Span<int> value_set_ids)
{
  int i = 0;
  while (i < map1.size()) {
    const int set_size = set_sizes[i];
    if (set_size == 1) {
      /* No need to sort anymore. */
      i += 1;
      continue;
    }

    sort_indices_with_id_maps(
        map1.slice(IndexRange(i, set_size)), values1, values1_to_set, value_set_ids);
    sort_indices_with_id_maps(
        map2.slice(IndexRange(i, set_size)), values2, values2_to_set, value_set_ids);
    i += set_size;
  }
}

/**
 * Split the sets into smaller sets based on the sorted attribute values.
 *
 * \returns false if the attributes don't line up.
 */
template<typename T>
static bool update_set_ids(MutableSpan<int> set_ids,
                           const Span<T> &values1,
                           const Span<T> &values2)
{
  T previous = values1[0];
  int set_id = 0;
  for (const int i : values1.index_range()) {
    const T value1 = values1[i];
    const T value2 = values2[i];
    if (value1 != value2) {
      /* TODO: use compare function based on type (like compare_v3v3_relative)*/
      /* They should be the same after sorting. */
      return false;
    }
    if (value1 != previous || set_ids[i] == i) {
      /* TODO: use compare function based on type (like compare_v3v3_relative)*/
      /* Different value, or this was already a different set. */
      set_id = i;
      previous = value1;
    }
    set_ids[i] = set_id;
  }

  return true;
}

/**
 * Split the sets into smaller sets based on the sorted attribute values.
 *
 * \returns false if the attributes don't line up.
 */
static bool update_set_ids_with_id_maps(MutableSpan<int> set_ids,
                                        const Span<int> &values1,
                                        const Span<int> &values2,
                                        const Span<int> &values1_to_set,
                                        const Span<int> &values2_to_set,
                                        const Span<int> &value_set_ids)
{
  int previous = value_set_ids[values1_to_set[values1[0]]];
  int set_id = 0;
  for (const int i : values1.index_range()) {
    const int value1 = value_set_ids[values1_to_set[values1[i]]];
    const int value2 = value_set_ids[values2_to_set[values2[i]]];
    if (value1 != value2) {
      /* They should be the same after sorting. */
      return false;
    }
    if (value1 != previous || set_ids[i] == i) {
      /* Different value, or this was already a different set. */
      set_id = i;
      previous = value1;
    }
    set_ids[i] = set_id;
  }

  return true;
}

/**
 * Update set sizes, using the updated set ids.
 */
static void update_set_sizes(const Span<int> set_ids, MutableSpan<int> set_sizes)
{
  int i = set_ids.size() - 1;
  while (i >= 0) {
    /* The id of a set is the index of its first element, so the size can be computed as the index
     * of the last element minus the id (== index of first element) + 1. */
    int set_size = i - set_ids[i] + 1;
    /* Set the set size for each element in the set. */
    for (int k = i - set_size + 1; k <= i; k++) {
      set_sizes[k] = set_size;
    }
    i -= set_size;
  }
}

static void edges_from_vertex_sets(const Span<int2> edges,
                                   const Span<int> verts_to_set,
                                   const Span<int> vertex_set_ids,
                                   MutableSpan<OrderedEdge> r_edges)
{
  for (const int i : r_edges.index_range()) {
    const int2 e = edges[i];
    r_edges[i] = OrderedEdge(vertex_set_ids[verts_to_set[e.x]], vertex_set_ids[verts_to_set[e.y]]);
  }
}
static bool sort_edges(const Span<int2> edges1,
                       const Span<int2> edges2,
                       const Span<int> verts1_to_set,
                       const Span<int> verts2_to_set,
                       const Span<int> vertex_set_ids,
                       MutableSpan<int> edges1_to_set,
                       MutableSpan<int> edges2_to_set,
                       MutableSpan<int> edge_set_ids,
                       MutableSpan<int> edge_set_sizes)
{
  /* Need `NoInitialization()` because OrderedEdge is not default constructible. */
  Array<OrderedEdge> ordered_edges1(edges1.size(), NoInitialization());
  Array<OrderedEdge> ordered_edges2(edges2.size(), NoInitialization());
  edges_from_vertex_sets(edges1, verts1_to_set, vertex_set_ids, ordered_edges1);
  edges_from_vertex_sets(edges1, verts2_to_set, vertex_set_ids, ordered_edges2);
  sort_per_set_based_on_attributes(edge_set_sizes,
                                   edges1_to_set,
                                   edges2_to_set,
                                   ordered_edges1.as_span(),
                                   ordered_edges2.as_span());
  const bool edges_match = update_set_ids(
      edge_set_ids, ordered_edges1.as_span(), ordered_edges2.as_span());
  if (!edges_match) {
    return false;
  }
  update_set_sizes(edge_set_ids, edge_set_sizes);
  return true;
}

static bool sort_corners_based_on_edges(const Span<int> corner_edges1,
                                        const Span<int> corner_edges2,
                                        const Span<int> edges1_to_set,
                                        const Span<int> edges2_to_set,
                                        const Span<int> edge_set_ids,
                                        MutableSpan<int> corners1_to_set,
                                        MutableSpan<int> corners2_to_set,
                                        MutableSpan<int> corner_set_ids,
                                        MutableSpan<int> corner_set_sizes)
{
  sort_per_set_with_id_maps(corner_set_ids,
                            corners1_to_set,
                            corners2_to_set,
                            corner_edges1,
                            corner_edges2,
                            edges1_to_set,
                            edges2_to_set,
                            edge_set_ids);
  const bool corners_line_up = update_set_ids_with_id_maps(
      corner_set_ids, corner_edges1, corner_edges2, edges1_to_set, edges2_to_set, edge_set_ids);
  if (!corners_line_up) {
    return false;
  }
  update_set_sizes(corner_set_ids, corner_set_sizes);
  return true;
}

static bool sort_corners_based_on_verts(const Span<int> corner_verts1,
                                        const Span<int> corner_verts2,
                                        const Span<int> verts1_to_set,
                                        const Span<int> verts2_to_set,
                                        const Span<int> vertex_set_ids,
                                        MutableSpan<int> corners1_to_set,
                                        MutableSpan<int> corners2_to_set,
                                        MutableSpan<int> corner_set_ids,
                                        MutableSpan<int> corner_set_sizes)
{
  sort_per_set_with_id_maps(corner_set_ids,
                            corners1_to_set,
                            corners2_to_set,
                            corner_verts1,
                            corner_verts2,
                            verts1_to_set,
                            verts2_to_set,
                            vertex_set_ids);
  const bool corners_line_up = update_set_ids_with_id_maps(
      corner_set_ids, corner_verts1, corner_verts2, verts1_to_set, verts2_to_set, vertex_set_ids);
  if (!corners_line_up) {
    return false;
  }
  update_set_sizes(corner_set_ids, corner_set_sizes);
  return true;
}

static std::optional<MeshMismatch> verify_attributes_compatible(
    const AttributeAccessor &mesh1_attributes, const AttributeAccessor &mesh2_attributes)
{
  Set<AttributeIDRef> mesh1_attribute_ids = mesh1_attributes.all_ids();
  Set<AttributeIDRef> mesh2_attribute_ids = mesh2_attributes.all_ids();
  if (mesh1_attribute_ids != mesh2_attribute_ids) {
    return MeshMismatch::Attributes;
  }
  for (const AttributeIDRef &id : mesh1_attribute_ids) {
    GAttributeReader reader1 = mesh1_attributes.lookup(id);
    GAttributeReader reader2 = mesh2_attributes.lookup(id);
    if (reader1.domain != reader2.domain || reader1.varray.type() != reader2.varray.type()) {
      return MeshMismatch::AttributeTypes;
    }
  }
  return {};
}

static std::optional<MeshMismatch> sort_domain_using_attributes(
    const AttributeAccessor &mesh1_attributes,
    const AttributeAccessor &mesh2_attributes,
    const eAttrDomain domain,
    const Span<StringRef> excluded_attributes,
    MutableSpan<int> r_domain1_to_set,
    MutableSpan<int> r_domain2_to_set,
    MutableSpan<int> r_domain_set_ids,
    MutableSpan<int> r_domain_set_sizes)
{

  /* We only need the ids from one mesh, since we know they have the same attributes. */
  Set<AttributeIDRef> attribute_ids = mesh1_attributes.all_ids();
  for (const StringRef name : excluded_attributes) {
    attribute_ids.remove(name);
  }

  for (const AttributeIDRef &id : attribute_ids) {
    GAttributeReader reader1 = mesh1_attributes.lookup(id);
    GAttributeReader reader2 = mesh2_attributes.lookup(id);
    if (reader1.domain != domain) {
      /* We only look at attributes of the given domain. */
      continue;
    }

    std::optional<MeshMismatch> mismatch = {};

    attribute_math::convert_to_static_type(reader1.varray.type(), [&](auto dummy) {
      using T = decltype(dummy);
      const VArraySpan<T> values1 = reader1.varray.typed<T>();
      const VArraySpan<T> values2 = reader2.varray.typed<T>();

      sort_per_set_based_on_attributes(
          r_domain_set_sizes, r_domain1_to_set, r_domain2_to_set, values1, values2);

      const bool attributes_line_up = update_set_ids(r_domain_set_ids, values1, values2);
      if (!attributes_line_up) {
        switch (domain) {
          case ATTR_DOMAIN_POINT:
            mismatch = MeshMismatch::VertexAttributes;
            return;
          case ATTR_DOMAIN_EDGE:
            mismatch = MeshMismatch::EdgeAttributes;
            return;
          case ATTR_DOMAIN_CORNER:
            mismatch = MeshMismatch::CornerAttributes;
            return;
          case ATTR_DOMAIN_FACE:
            mismatch = MeshMismatch::FaceAttributes;
            return;
          default:
            BLI_assert_unreachable();
            break;
        }
        return;
      }
      update_set_sizes(r_domain_set_ids, r_domain_set_sizes);
    });

    if (mismatch) {
      return mismatch;
    }
  }
  return {};
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
  // sort_indices(verts1, vert_positions1);
  // sort_indices(verts2, vert_positions2);

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

  std::optional<MeshMismatch> mismatch = {};

  const AttributeAccessor mesh1_attributes = mesh1.attributes();
  const AttributeAccessor mesh2_attributes = mesh2.attributes();
  mismatch = verify_attributes_compatible(mesh1_attributes, mesh2_attributes);
  if (mismatch) {
    return mismatch;
  }

  Array<int> verts1_to_set(mesh1.totvert);
  Array<int> verts2_to_set(mesh1.totvert);
  Array<int> vertex_set_ids(mesh1.totvert);
  Array<int> vertex_set_sizes(mesh1.totvert);
  std::iota(verts1_to_set.begin(), verts1_to_set.end(), 0);
  std::iota(verts2_to_set.begin(), verts2_to_set.end(), 0);
  vertex_set_ids.fill(0);
  vertex_set_sizes.fill(vertex_set_ids.size());
  mismatch = sort_domain_using_attributes(mesh1_attributes,
                                          mesh2_attributes,
                                          ATTR_DOMAIN_POINT,
                                          {},
                                          verts1_to_set,
                                          verts2_to_set,
                                          vertex_set_ids,
                                          vertex_set_sizes);
  if (mismatch) {
    return mismatch;
  };

  Array<int> edges1_to_set(mesh1.totedge);
  Array<int> edges2_to_set(mesh1.totedge);
  Array<int> edge_set_ids(mesh1.totedge);
  Array<int> edge_set_sizes(mesh1.totedge);
  std::iota(edges1_to_set.begin(), edges1_to_set.end(), 0);
  std::iota(edges2_to_set.begin(), edges2_to_set.end(), 0);
  edge_set_ids.fill(0);
  edge_set_sizes.fill(edge_set_ids.size());
  if (!sort_edges(mesh1.edges(),
                  mesh2.edges(),
                  verts1_to_set,
                  verts2_to_set,
                  vertex_set_ids,
                  edges1_to_set,
                  edges2_to_set,
                  edge_set_ids,
                  edge_set_sizes))
  {
    return MeshMismatch::EdgeTopology;
  }

  mismatch = sort_domain_using_attributes(mesh1_attributes,
                                          mesh2_attributes,
                                          ATTR_DOMAIN_EDGE,
                                          {".edge_verts"},
                                          edges1_to_set,
                                          edges2_to_set,
                                          edge_set_ids,
                                          edge_set_sizes);
  if (mismatch) {
    return mismatch;
  };

  Array<int> corners1_to_set(mesh1.totloop);
  Array<int> corners2_to_set(mesh1.totloop);
  Array<int> corner_set_ids(mesh1.totloop);
  Array<int> corner_set_sizes(mesh1.totloop);
  std::iota(corners1_to_set.begin(), corners1_to_set.end(), 0);
  std::iota(corners2_to_set.begin(), corners2_to_set.end(), 0);
  corner_set_ids.fill(0);
  corner_set_sizes.fill(corner_set_ids.size());

  if (!sort_corners_based_on_verts(mesh1.corner_verts(),
                                   mesh2.corner_verts(),
                                   verts1_to_set,
                                   verts2_to_set,
                                   vertex_set_ids,
                                   corners1_to_set,
                                   corners2_to_set,
                                   corner_set_ids,
                                   corner_set_sizes))
  {
    return MeshMismatch::FaceTopology;
  }

  if (!sort_corners_based_on_edges(mesh1.corner_edges(),
                                   mesh2.corner_edges(),
                                   verts1_to_set,
                                   verts2_to_set,
                                   vertex_set_ids,
                                   corners1_to_set,
                                   corners2_to_set,
                                   corner_set_ids,
                                   corner_set_sizes))
  {
    return MeshMismatch::FaceTopology;
  }

  mismatch = sort_domain_using_attributes(mesh1_attributes,
                                          mesh2_attributes,
                                          ATTR_DOMAIN_CORNER,
                                          {".corner_vert", ".corner_edge"},
                                          corners1_to_set,
                                          corners2_to_set,
                                          corner_set_ids,
                                          corner_set_sizes);
  if (mismatch) {
    return mismatch;
  };

  Array<int> faces1_to_set(mesh1.totloop);
  Array<int> faces2_to_set(mesh1.totloop);
  Array<int> face_set_ids(mesh1.totloop);
  Array<int> face_set_sizes(mesh1.totloop);
  std::iota(faces1_to_set.begin(), faces1_to_set.end(), 0);
  std::iota(faces2_to_set.begin(), faces2_to_set.end(), 0);
  face_set_ids.fill(0);
  face_set_sizes.fill(face_set_ids.size());
  mismatch = sort_domain_using_attributes(mesh1_attributes,
                                          mesh2_attributes,
                                          ATTR_DOMAIN_FACE,
                                          {},
                                          faces1_to_set,
                                          faces2_to_set,
                                          face_set_ids,
                                          face_set_sizes);
  if (mismatch) {
    return mismatch;
  };

  /* We first try to construct a bijection between the vertices, since edges, corners and faces are
   * dependent on vertex indices. */
  Array<int> verts1_to_verts2_map(mesh1.totvert);
  return construct_vertex_mapping(mesh1, mesh2, verts1_to_verts2_map);
}

}  // namespace blender::bke::mesh
