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

static void create_inverse_map(const Span<int> map, MutableSpan<int> inverted_map)
{
  for (const int i : map.index_range()) {
    inverted_map[map[i]] = i;
  }
}

/**
 * Sort the indices using the values.
 */
template<typename T> static void sort_indices(MutableSpan<int> indices, const Span<T> values)
{
  /* We need to have an appropriate comparison function, depending on the type. */
  std::stable_sort(indices.begin(), indices.end(), [&](int i1, int i2) {
    const T value1 = values[i1];
    const T value2 = values[i2];
    if constexpr (std::is_same_v<T, int> || std::is_same_v<T, float> || std::is_same_v<T, bool> ||
                  std::is_same_v<T, int8_t> || std::is_same_v<T, OrderedEdge>)
    {
      /* These types are already comparable. */
      return value1 < value2;
    }
    else if constexpr (std::is_same_v<T, float2>) {
      for (int i = 0; i < 2; i++) {
        if (compare_threshold_relative(value1[i], value2[i], FLT_EPSILON * 60)) {
          return value1[i] < value2[i];
        }
      }
      return false;
    }
    else if constexpr (std::is_same_v<T, float3>) {
      for (int i = 0; i < 3; i++) {
        if (compare_threshold_relative(value1[i], value2[i], FLT_EPSILON * 60)) {
          return value1[i] < value2[i];
        }
      }
      return false;
    }
    else if constexpr (std::is_same_v<T, math::Quaternion> || std::is_same_v<T, ColorGeometry4f>) {
      const float4 value1 = static_cast<float4>(value1);
      const float4 value2 = static_cast<float4>(value2);
      for (int i = 0; i < 4; i++) {
        if (compare_threshold_relative(value1[i], value2[i], FLT_EPSILON * 60)) {
          return value1[i] < value2[i];
        }
      }
      return false;
    }
    else if constexpr (std::is_same_v<T, int2>) {
      for (int i = 0; i < 2; i++) {
        if (value1[i] != value2[i]) {
          return value1[i] < value2[i];
        }
      }
      return false;
    }
    else if constexpr (std::is_same_v<T, ColorGeometry4b>) {
      for (int i = 0; i < 4; i++) {
        if (value1[i] != value2[i]) {
          return value1[i] < value2[i];
        }
      }
      return false;
    }
    else {
      BLI_assert_unreachable();
    }
  });
}

/**
 * Sort the indices using the values.
 */
static void sort_indices_with_id_maps(MutableSpan<int> indices,
                                      const Span<int> values,
                                      const Span<int> values_to_sorted,
                                      const Span<int> set_ids)
{
  std::stable_sort(indices.begin(), indices.end(), [&](int i1, int i2) {
    return set_ids[values_to_sorted[values[i1]]] < set_ids[values_to_sorted[values[i2]]];
  });
}

/* Sort the elements in each set based on the attribute values. */
template<typename T>
static void sort_per_set_based_on_attributes(const Span<int> set_sizes,
                                             MutableSpan<int> sorted_to_domain1,
                                             MutableSpan<int> sorted_to_domain2,
                                             const Span<T> values1,
                                             const Span<T> values2)
{
  int i = 0;
  while (i < sorted_to_domain1.size()) {
    const int set_size = set_sizes[i];
    if (set_size == 1) {
      /* No need to sort anymore. */
      i += 1;
      continue;
    }

    sort_indices(sorted_to_domain1.slice(IndexRange(i, set_size)), values1);
    sort_indices(sorted_to_domain2.slice(IndexRange(i, set_size)), values2);
    i += set_size;
  }
}

/* Sort the elements in each set based on the set ids of the values. */
static void sort_per_set_with_id_maps(const Span<int> set_sizes,
                                      const Span<int> values1,
                                      const Span<int> values2,
                                      const Span<int> values1_to_sorted,
                                      const Span<int> values2_to_sorted,
                                      const Span<int> value_set_ids,
                                      MutableSpan<int> sorted_to_domain1,
                                      MutableSpan<int> sorted_to_domain2)
{
  int i = 0;
  while (i < sorted_to_domain1.size()) {
    const int set_size = set_sizes[i];
    if (set_size == 1) {
      /* No need to sort anymore. */
      i += 1;
      continue;
    }

    sort_indices_with_id_maps(sorted_to_domain1.slice(IndexRange(i, set_size)),
                              values1,
                              values1_to_sorted,
                              value_set_ids);
    sort_indices_with_id_maps(sorted_to_domain2.slice(IndexRange(i, set_size)),
                              values2,
                              values2_to_sorted,
                              value_set_ids);
    i += set_size;
  }
}

template<typename T> static bool values_different(const T value1, const T value2)
{
  if constexpr (std::is_same_v<T, int> || std::is_same_v<T, int2> || std::is_same_v<T, bool> ||
                std::is_same_v<T, int8_t> || std::is_same_v<T, OrderedEdge> ||
                std::is_same_v<T, ColorGeometry4b>)
  {
    /* These types already have a good implementation. */
    return value1 != value2;
  }
  /* The other types are based on floats. */
  else if constexpr (std::is_same_v<T, float>) {
    return compare_threshold_relative(value1, value2, FLT_EPSILON * 60);
  }
  else if constexpr (std::is_same_v<T, float2>) {
    for (int i = 0; i < 2; i++) {
      if (compare_threshold_relative(value1[i], value2[i], FLT_EPSILON * 60)) {
        return true;
      }
    }
    return false;
  }
  else if constexpr (std::is_same_v<T, float3>) {
    for (int i = 0; i < 3; i++) {
      if (compare_threshold_relative(value1[i], value2[i], FLT_EPSILON * 60)) {
        return true;
      }
    }
    return false;
  }
  else if constexpr (std::is_same_v<T, math::Quaternion> || std::is_same_v<T, ColorGeometry4f>) {
    const float4 value1_f = static_cast<float4>(value1);
    const float4 value2_f = static_cast<float4>(value2);
    for (int i = 0; i < 4; i++) {
      if (compare_threshold_relative(value1_f[i], value2_f[i], FLT_EPSILON * 60)) {
        return true;
      }
    }
    return false;
  }
  else {
    BLI_assert_unreachable();
  }
}

/**
 * Split the sets into smaller sets based on the sorted attribute values.
 *
 * \returns false if the attributes don't line up.
 */
template<typename T>
static bool update_set_ids(MutableSpan<int> set_ids,
                           const Span<T> values1,
                           const Span<T> values2,
                           const Span<int> sorted_to_values1,
                           const Span<int> sorted_to_values2)
{
  T previous = values1[0];
  int set_id = 0;
  for (const int i : values1.index_range()) {
    const T value1 = values1[sorted_to_values1[i]];
    const T value2 = values2[sorted_to_values2[i]];
    if (values_different(value1, value2)) {
      /* They should be the same after sorting. */
      return false;
    }
    if (values_different(previous, value1) || set_ids[i] == i) {
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
                                        const Span<int> domain_to_values1,
                                        const Span<int> domain_to_values2,
                                        const Span<int> values1_to_sorted,
                                        const Span<int> values2_to_sorted,
                                        const Span<int> value_set_ids,
                                        const Span<int> sorted_to_domain1,
                                        const Span<int> sorted_to_domain2)
{
  int previous = value_set_ids[values1_to_sorted[domain_to_values1[sorted_to_domain1[0]]]];
  int set_id = 0;
  for (const int i : sorted_to_domain1.index_range()) {
    const int value_id1 =
        value_set_ids[values1_to_sorted[domain_to_values1[sorted_to_domain1[i]]]];
    const int value_id2 =
        value_set_ids[values2_to_sorted[domain_to_values2[sorted_to_domain2[i]]]];
    if (value_id1 != value_id2) {
      /* They should be the same after sorting. */
      return false;
    }
    if (value_id1 != previous || set_ids[i] == i) {
      /* Different value, or this was already a different set. */
      set_id = i;
      previous = value_id1;
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
                                   const Span<int> verts_to_sorted,
                                   const Span<int> vertex_set_ids,
                                   MutableSpan<OrderedEdge> r_edges)
{
  for (const int i : r_edges.index_range()) {
    const int2 e = edges[i];
    r_edges[i] = OrderedEdge(vertex_set_ids[verts_to_sorted[e.x]],
                             vertex_set_ids[verts_to_sorted[e.y]]);
  }
}
static bool sort_edges(const Span<int2> edges1,
                       const Span<int2> edges2,
                       const Span<int> verts1_to_sorted,
                       const Span<int> verts2_to_sorted,
                       const Span<int> vertex_set_ids,
                       MutableSpan<int> sorted_to_edges1,
                       MutableSpan<int> sorted_to_edges2,
                       MutableSpan<int> edge_set_ids,
                       MutableSpan<int> edge_set_sizes)
{
  /* Need `NoInitialization()` because OrderedEdge is not default constructible. */
  Array<OrderedEdge> ordered_edges1(edges1.size(), NoInitialization());
  Array<OrderedEdge> ordered_edges2(edges2.size(), NoInitialization());
  edges_from_vertex_sets(edges1, verts1_to_sorted, vertex_set_ids, ordered_edges1);
  edges_from_vertex_sets(edges1, verts2_to_sorted, vertex_set_ids, ordered_edges2);
  sort_per_set_based_on_attributes(edge_set_sizes,
                                   sorted_to_edges1,
                                   sorted_to_edges2,
                                   ordered_edges1.as_span(),
                                   ordered_edges2.as_span());
  const bool edges_match = update_set_ids(edge_set_ids,
                                          ordered_edges1.as_span(),
                                          ordered_edges2.as_span(),
                                          sorted_to_edges1,
                                          sorted_to_edges2);
  if (!edges_match) {
    return false;
  }
  update_set_sizes(edge_set_ids, edge_set_sizes);
  return true;
}

static bool sort_corners_based_on_edges(const Span<int> corner_edges1,
                                        const Span<int> corner_edges2,
                                        const Span<int> edges1_to_sorted,
                                        const Span<int> edges2_to_sorted,
                                        const Span<int> edge_set_ids,
                                        MutableSpan<int> sorted_to_corners1,
                                        MutableSpan<int> sorted_to_corners2,
                                        MutableSpan<int> corner_set_ids,
                                        MutableSpan<int> corner_set_sizes)
{
  sort_per_set_with_id_maps(corner_set_sizes,
                            corner_edges1,
                            corner_edges2,
                            edges1_to_sorted,
                            edges2_to_sorted,
                            edge_set_ids,
                            sorted_to_corners1,
                            sorted_to_corners2);
  const bool corners_line_up = update_set_ids_with_id_maps(corner_set_ids,
                                                           corner_edges1,
                                                           corner_edges2,
                                                           edges1_to_sorted,
                                                           edges2_to_sorted,
                                                           edge_set_ids,
                                                           sorted_to_corners1,
                                                           sorted_to_corners2);
  if (!corners_line_up) {
    return false;
  }
  update_set_sizes(corner_set_ids, corner_set_sizes);
  return true;
}

static bool sort_corners_based_on_verts(const Span<int> corner_verts1,
                                        const Span<int> corner_verts2,
                                        const Span<int> verts1_to_sorted,
                                        const Span<int> verts2_to_sorted,
                                        const Span<int> vertex_set_ids,
                                        MutableSpan<int> sorted_to_corners1,
                                        MutableSpan<int> sorted_to_corners2,
                                        MutableSpan<int> corner_set_ids,
                                        MutableSpan<int> corner_set_sizes)
{
  sort_per_set_with_id_maps(corner_set_sizes,
                            corner_verts1,
                            corner_verts2,
                            verts1_to_sorted,
                            verts2_to_sorted,
                            vertex_set_ids,
                            sorted_to_corners1,
                            sorted_to_corners2);
  const bool corners_line_up = update_set_ids_with_id_maps(corner_set_ids,
                                                           corner_verts1,
                                                           corner_verts2,
                                                           verts1_to_sorted,
                                                           verts2_to_sorted,
                                                           vertex_set_ids,
                                                           sorted_to_corners1,
                                                           sorted_to_corners2);
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
    MutableSpan<int> sorted_to_domain1,
    MutableSpan<int> sorted_to_domain2,
    MutableSpan<int> domain_set_ids,
    MutableSpan<int> domain_set_sizes)
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

    std::cout << "Sorting " << id.name() << std::endl;

    std::optional<MeshMismatch> mismatch = {};

    attribute_math::convert_to_static_type(reader1.varray.type(), [&](auto dummy) {
      using T = decltype(dummy);
      const VArraySpan<T> values1 = reader1.varray.typed<T>();
      const VArraySpan<T> values2 = reader2.varray.typed<T>();

      sort_per_set_based_on_attributes(
          domain_set_sizes, sorted_to_domain1, sorted_to_domain2, values1, values2);

      const bool attributes_line_up = update_set_ids(
          domain_set_ids, values1, values2, sorted_to_domain1, sorted_to_domain2);
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
      update_set_sizes(domain_set_ids, domain_set_sizes);
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
static std::optional<MeshMismatch> construct_vertex_mapping()
{
  /* TODO. */
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

  std::cout << "domain sizes match" << std::endl;

  std::optional<MeshMismatch> mismatch = {};

  const AttributeAccessor mesh1_attributes = mesh1.attributes();
  const AttributeAccessor mesh2_attributes = mesh2.attributes();
  mismatch = verify_attributes_compatible(mesh1_attributes, mesh2_attributes);
  if (mismatch) {
    return mismatch;
  }

  std::cout << "attributes are compatible" << std::endl;

  Array<int> sorted_to_verts1(mesh1.totvert);
  Array<int> sorted_to_verts2(mesh1.totvert);
  Array<int> vertex_set_ids(mesh1.totvert);
  Array<int> vertex_set_sizes(mesh1.totvert);
  std::iota(sorted_to_verts1.begin(), sorted_to_verts1.end(), 0);
  std::iota(sorted_to_verts2.begin(), sorted_to_verts2.end(), 0);
  vertex_set_ids.fill(0);
  vertex_set_sizes.fill(vertex_set_ids.size());
  mismatch = sort_domain_using_attributes(mesh1_attributes,
                                          mesh2_attributes,
                                          ATTR_DOMAIN_POINT,
                                          {},
                                          sorted_to_verts1,
                                          sorted_to_verts2,
                                          vertex_set_ids,
                                          vertex_set_sizes);
  if (mismatch) {
    return mismatch;
  };

  /* We need the maps going the other way as well. */
  Array<int> verts1_to_sorted(sorted_to_verts1.size());
  Array<int> verts2_to_sorted(sorted_to_verts2.size());
  create_inverse_map(sorted_to_verts1, verts1_to_sorted);
  create_inverse_map(sorted_to_verts2, verts2_to_sorted);

  std::cout << "sorted vertices" << std::endl;

  Array<int> sorted_to_edges1(mesh1.totedge);
  Array<int> sorted_to_edges2(mesh1.totedge);
  Array<int> edge_set_ids(mesh1.totedge);
  Array<int> edge_set_sizes(mesh1.totedge);
  std::iota(sorted_to_edges1.begin(), sorted_to_edges1.end(), 0);
  std::iota(sorted_to_edges2.begin(), sorted_to_edges2.end(), 0);
  edge_set_ids.fill(0);
  edge_set_sizes.fill(edge_set_ids.size());
  if (!sort_edges(mesh1.edges(),
                  mesh2.edges(),
                  verts1_to_sorted,
                  verts2_to_sorted,
                  vertex_set_ids,
                  sorted_to_edges1,
                  sorted_to_edges2,
                  edge_set_ids,
                  edge_set_sizes))
  {
    return MeshMismatch::EdgeTopology;
  }

  std::cout << "sorted edges" << std::endl;

  mismatch = sort_domain_using_attributes(mesh1_attributes,
                                          mesh2_attributes,
                                          ATTR_DOMAIN_EDGE,
                                          {".edge_verts"},
                                          sorted_to_edges1,
                                          sorted_to_edges2,
                                          edge_set_ids,
                                          edge_set_sizes);
  if (mismatch) {
    return mismatch;
  };

  /* We need the maps going the other way as well. */
  Array<int> edges1_to_sorted(sorted_to_edges1.size());
  Array<int> edges2_to_sorted(sorted_to_edges2.size());
  create_inverse_map(sorted_to_edges1, edges1_to_sorted);
  create_inverse_map(sorted_to_edges2, edges2_to_sorted);

  std::cout << "sorted edge domain" << std::endl;

  Array<int> sorted_corners1(mesh1.totloop);
  Array<int> sorted_corners2(mesh1.totloop);
  Array<int> corner_set_ids(mesh1.totloop);
  Array<int> corner_set_sizes(mesh1.totloop);
  std::iota(sorted_corners1.begin(), sorted_corners1.end(), 0);
  std::iota(sorted_corners2.begin(), sorted_corners2.end(), 0);
  corner_set_ids.fill(0);
  corner_set_sizes.fill(corner_set_ids.size());

  if (!sort_corners_based_on_verts(mesh1.corner_verts(),
                                   mesh2.corner_verts(),
                                   verts1_to_sorted,
                                   verts2_to_sorted,
                                   vertex_set_ids,
                                   sorted_corners1,
                                   sorted_corners2,
                                   corner_set_ids,
                                   corner_set_sizes))
  {
    return MeshMismatch::FaceTopology;
  }

  std::cout << "sorted corner verts" << std::endl;

  if (!sort_corners_based_on_edges(mesh1.corner_edges(),
                                   mesh2.corner_edges(),
                                   edges1_to_sorted,
                                   edges2_to_sorted,
                                   edge_set_ids,
                                   sorted_corners1,
                                   sorted_corners2,
                                   corner_set_ids,
                                   corner_set_sizes))
  {
    return MeshMismatch::FaceTopology;
  }

  std::cout << "sorted corner edges" << std::endl;

  mismatch = sort_domain_using_attributes(mesh1_attributes,
                                          mesh2_attributes,
                                          ATTR_DOMAIN_CORNER,
                                          {".corner_vert", ".corner_edge"},
                                          sorted_corners1,
                                          sorted_corners2,
                                          corner_set_ids,
                                          corner_set_sizes);
  if (mismatch) {
    return mismatch;
  };

  std::cout << "sorted corner domain" << std::endl;

  Array<int> sorted_faces1(mesh1.faces_num);
  Array<int> sorted_faces2(mesh1.faces_num);
  Array<int> face_set_ids(mesh1.faces_num);
  Array<int> face_set_sizes(mesh1.faces_num);
  std::iota(sorted_faces1.begin(), sorted_faces1.end(), 0);
  std::iota(sorted_faces2.begin(), sorted_faces2.end(), 0);
  face_set_ids.fill(0);
  face_set_sizes.fill(face_set_ids.size());
  /* TODO: sort the faces using the corner set ids. */
  mismatch = sort_domain_using_attributes(mesh1_attributes,
                                          mesh2_attributes,
                                          ATTR_DOMAIN_FACE,
                                          {},
                                          sorted_faces1,
                                          sorted_faces2,
                                          face_set_ids,
                                          face_set_sizes);
  if (mismatch) {
    return mismatch;
  };

  std::cout << "sorted face domain" << std::endl;

  /* TODO: try to construct the actual bijections. */
  return construct_vertex_mapping();
}

}  // namespace blender::bke::mesh
