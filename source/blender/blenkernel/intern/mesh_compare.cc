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

class IndexMapping {
 private:
  void calculate_inverse_map(const Span<int> map, MutableSpan<int> inverted_map)
  {
    for (const int i : map.index_range()) {
      inverted_map[map[i]] = i;
    }
  }

 public:
  Array<int> from_sorted1;
  Array<int> from_sorted2;
  Array<int> to_sorted1;
  Array<int> to_sorted2;
  Array<int> set_ids;
  Array<int> set_sizes;

  IndexMapping(const int64_t domain_size)
  {
    to_sorted1 = Array<int>(domain_size);
    to_sorted2 = Array<int>(domain_size);
    from_sorted1 = Array<int>(domain_size);
    from_sorted2 = Array<int>(domain_size);
    set_ids = Array<int>(domain_size);
    set_sizes = Array<int>(domain_size);
    std::iota(from_sorted1.begin(), from_sorted1.end(), 0);
    std::iota(from_sorted2.begin(), from_sorted2.end(), 0);
    std::iota(to_sorted1.begin(), to_sorted1.end(), 0);
    std::iota(to_sorted2.begin(), to_sorted2.end(), 0);
    set_ids.fill(0);
    set_sizes.fill(set_ids.size());
  }

  void recalculate_inverse_maps()
  {
    calculate_inverse_map(from_sorted1, to_sorted1);
    calculate_inverse_map(from_sorted2, to_sorted2);
  }
};

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
                       const IndexMapping verts,
                       IndexMapping edges)
{
  /* Need `NoInitialization()` because OrderedEdge is not default constructible. */
  Array<OrderedEdge> ordered_edges1(edges1.size(), NoInitialization());
  Array<OrderedEdge> ordered_edges2(edges2.size(), NoInitialization());
  edges_from_vertex_sets(edges1, verts.to_sorted1, verts.set_ids, ordered_edges1);
  edges_from_vertex_sets(edges1, verts.to_sorted2, verts.set_ids, ordered_edges2);
  sort_per_set_based_on_attributes(edges.set_sizes,
                                   edges.from_sorted1,
                                   edges.from_sorted2,
                                   ordered_edges1.as_span(),
                                   ordered_edges2.as_span());
  const bool edges_match = update_set_ids(edges.set_ids,
                                          ordered_edges1.as_span(),
                                          ordered_edges2.as_span(),
                                          edges.from_sorted1,
                                          edges.from_sorted2);
  if (!edges_match) {
    return false;
  }
  update_set_sizes(edges.set_ids, edges.set_sizes);
  return true;
}

static bool sort_corners_based_on_domain(const Span<int> corner_domain1,
                                         const Span<int> corner_domain2,
                                         const IndexMapping domain,
                                         IndexMapping corners)
{
  sort_per_set_with_id_maps(corners.set_sizes,
                            corner_domain1,
                            corner_domain2,
                            domain.to_sorted1,
                            domain.to_sorted2,
                            domain.set_ids,
                            corners.from_sorted1,
                            corners.from_sorted2);
  const bool corners_line_up = update_set_ids_with_id_maps(corners.set_ids,
                                                           corner_domain1,
                                                           corner_domain2,
                                                           domain.to_sorted1,
                                                           domain.to_sorted2,
                                                           domain.set_ids,
                                                           corners.from_sorted1,
                                                           corners.from_sorted2);
  if (!corners_line_up) {
    return false;
  }
  update_set_sizes(corners.set_ids, corners.set_sizes);
  return true;
}

static void calc_smallest_corner_ids(const Span<int> face_offsets,
                                     const Span<int> corners_to_sorted,
                                     const Span<int> corner_set_ids,
                                     MutableSpan<int> smallest_corner_ids)
{
  for (const int face_i : smallest_corner_ids.index_range()) {
    const int face_start = face_offsets[face_i];
    const int face_end = face_offsets[face_i + 1];
    int smallest = corner_set_ids[corners_to_sorted[face_start]];
    const IndexRange corners = IndexRange(face_start, face_end - face_start);
    for (const int corner_i : corners.drop_front(1)) {
      const int corner_id = corner_set_ids[corners_to_sorted[corner_i]];
      if (corner_id < smallest) {
        smallest = corner_id;
      }
    }
    smallest_corner_ids[face_i] = smallest;
  }
}

static bool sort_faces_based_on_corners(const IndexMapping corners,
                                        const Span<int> face_offsets1,
                                        const Span<int> face_offsets2,
                                        IndexMapping faces)
{
  /* The smallest corner set id, per face. */
  Array<int> smallest_corner_ids1(faces.from_sorted1.size());
  Array<int> smallest_corner_ids2(faces.from_sorted2.size());
  calc_smallest_corner_ids(
      face_offsets1, corners.to_sorted1, corners.set_ids, smallest_corner_ids1);
  calc_smallest_corner_ids(
      face_offsets2, corners.to_sorted2, corners.set_ids, smallest_corner_ids2);
  sort_per_set_based_on_attributes(faces.set_sizes,
                                   faces.from_sorted1,
                                   faces.from_sorted2,
                                   smallest_corner_ids1.as_span(),
                                   smallest_corner_ids2.as_span());
  const bool faces_line_up = update_set_ids(faces.set_ids,
                                            smallest_corner_ids1.as_span(),
                                            smallest_corner_ids2.as_span(),
                                            faces.from_sorted1,
                                            faces.from_sorted2);
  if (!faces_line_up) {
    return false;
  }
  update_set_sizes(faces.set_ids, faces.set_sizes);
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
    IndexMapping &maps)
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
          maps.set_sizes, maps.from_sorted1, maps.from_sorted2, values1, values2);

      const bool attributes_line_up = update_set_ids(
          maps.set_ids, values1, values2, maps.from_sorted1, maps.from_sorted2);
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
      update_set_sizes(maps.set_ids, maps.set_sizes);
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

  IndexMapping verts(mesh1.totvert);
  mismatch = sort_domain_using_attributes(
      mesh1_attributes, mesh2_attributes, ATTR_DOMAIN_POINT, {}, verts);
  if (mismatch) {
    return mismatch;
  };

  /* We need the maps going the other way as well. */
  verts.recalculate_inverse_maps();

  std::cout << "sorted vertices" << std::endl;

  IndexMapping edges(mesh1.totedge);
  if (!sort_edges(mesh1.edges(), mesh2.edges(), verts, edges)) {
    return MeshMismatch::EdgeTopology;
  }

  std::cout << "sorted edges" << std::endl;

  mismatch = sort_domain_using_attributes(
      mesh1_attributes, mesh2_attributes, ATTR_DOMAIN_EDGE, {".edge_verts"}, edges);
  if (mismatch) {
    return mismatch;
  };

  /* We need the maps going the other way as well. */
  edges.recalculate_inverse_maps();

  std::cout << "sorted edge domain" << std::endl;

  IndexMapping corners(mesh1.totloop);
  if (!sort_corners_based_on_domain(mesh1.corner_verts(), mesh2.corner_verts(), verts, corners)) {
    return MeshMismatch::FaceTopology;
  }

  std::cout << "sorted corner verts" << std::endl;

  if (!sort_corners_based_on_domain(mesh1.corner_edges(), mesh2.corner_edges(), edges, corners)) {
    return MeshMismatch::FaceTopology;
  }

  std::cout << "sorted corner edges" << std::endl;

  mismatch = sort_domain_using_attributes(mesh1_attributes,
                                          mesh2_attributes,
                                          ATTR_DOMAIN_CORNER,
                                          {".corner_vert", ".corner_edge"},
                                          corners);
  if (mismatch) {
    return mismatch;
  };

  /* We need the maps going the other way as well. */
  corners.recalculate_inverse_maps();

  std::cout << "sorted corner domain" << std::endl;

  IndexMapping faces(mesh1.faces_num);
  if (!sort_faces_based_on_corners(corners, mesh1.face_offsets(), mesh2.face_offsets(), faces)) {
    return MeshMismatch::FaceTopology;
  }

  mismatch = sort_domain_using_attributes(
      mesh1_attributes, mesh2_attributes, ATTR_DOMAIN_FACE, {}, faces);
  if (mismatch) {
    return mismatch;
  };

  std::cout << "sorted face domain" << std::endl;

  /* TODO: try to construct the actual bijections. */
  return construct_vertex_mapping();
}

}  // namespace blender::bke::mesh
