/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_array_utils.hh"
#include "BLI_enumerable_thread_specific.hh"
#include "BLI_index_mask.hh"
#include "BLI_math_geom.h"
#include "BLI_math_matrix.h"
#include "BLI_ordered_edge.hh"
#include "BLI_polyfill_2d.h"
#include "BLI_polyfill_2d_beautify.h"
#include "BLI_vector_set.hh"

#include "BLI_heap.h"
#include "BLI_memarena.h"

#include "BKE_attribute.hh"
#include "BKE_attribute_math.hh"
#include "BKE_geometry_fields.hh"
#include "BKE_mesh.hh"
#include "BKE_mesh_mapping.hh"

#include "GEO_mesh_triangulate.hh"

namespace blender::geometry {

static Span<int> gather_or_reference(const Span<int> src,
                                     const Span<int16_t> indices,
                                     Vector<int> &dst)
{
  if (unique_sorted_indices::non_empty_is_range(indices)) {
    return src.slice(indices[0], indices.size());
  }
  dst.reinitialize(indices.size());
  for (const int i : indices.index_range()) {
    dst[i] = src[indices[i]];
  }
  return dst.as_span();
}

static Span<int> gather_or_reference(const Span<int> src,
                                     const IndexMaskSegment mask,
                                     Vector<int> &dst)
{
  return gather_or_reference(src.drop_front(mask.offset()), mask.base_span(), dst);
}

static int face_tri_to_corners_num(const int faces_num, const int tris_num)
{
  /* Rearrange terms from #poly_to_tri_count. */
  return tris_num + 2 * faces_num;
}

/**
 * If a significant number of Ngons are selected (> 25% of the faces), then use the
 * face normals cache, in case the cache is persistent (or already calculated).
 */
static Span<float3> face_normals_if_worthwhile(const Mesh &src_mesh, const int selection_size)
{
  if (src_mesh.runtime->face_normals_cache.is_cached()) {
    return src_mesh.face_normals();
  }
  if (selection_size > src_mesh.faces_num / 4) {
    return src_mesh.face_normals();
  }
  return {};
}

static void copy_loose_vert_hint(const Mesh &src, Mesh &dst)
{
  const auto &src_cache = src.runtime->loose_verts_cache;
  if (src_cache.is_cached() && src_cache.data().count == 0) {
    dst.tag_loose_verts_none();
  }
}

static void copy_loose_edge_hint(const Mesh &src, Mesh &dst)
{
  const auto &src_cache = src.runtime->loose_edges_cache;
  if (src_cache.is_cached() && src_cache.data().count == 0) {
    dst.tag_loose_edges_none();
  }
}

static Mesh *create_mesh_no_attributes(const Mesh &params_mesh,
                                       const int verts_num,
                                       const int edges_num,
                                       const int faces_num,
                                       const int corners_num)
{
  Mesh *mesh = BKE_mesh_new_nomain(0, 0, faces_num, 0);
  BKE_mesh_copy_parameters_for_eval(mesh, &params_mesh);
  CustomData_free_layer_named(&mesh->vert_data, "position", 0);
  CustomData_free_layer_named(&mesh->edge_data, ".edge_verts", 0);
  CustomData_free_layer_named(&mesh->loop_data, ".corner_vert", 0);
  CustomData_free_layer_named(&mesh->loop_data, ".corner_edge", 0);
  mesh->totvert = verts_num;
  mesh->totedge = edges_num;
  mesh->totloop = corners_num;
  return mesh;
}

static OffsetIndices<int> calc_faces(const OffsetIndices<int> src_faces,
                                     const IndexMask &unselected,
                                     MutableSpan<int> offsets)
{
  MutableSpan<int> new_tri_offsets = offsets.drop_back(unselected.size());
  offset_indices::fill_constant_group_size(3, new_tri_offsets.first(), new_tri_offsets);
  offset_indices::gather_selected_offsets(
      src_faces, unselected, offsets.take_back(unselected.size() + 1), new_tri_offsets.last());
  return OffsetIndices<int>(offsets);
}

/**
 *  #Edge_0_2       #Edge_1_3
 * 3 ------- 2     3 ------- 2
 * | 1     / |     | \     1 |
 * |     /   |     |   \     |
 * |   /     |     |     \   |
 * | /     0 |     | 0     \ |
 * 0 ------- 1     0 ------- 1
 */
enum class QuadDirection : int8_t {
  Edge_0_2 = 0,
  Edge_1_3 = 1,
};

/**
 * \note This behavior is meant to be the same as #BM_verts_calc_rotate_beauty.
 * The order of vertices requires special attention.
 */
static QuadDirection calc_quad_direction_beauty(const float3 &v0,
                                                const float3 &v1,
                                                const float3 &v2,
                                                const float3 &v3)
{
  const int flip_flag = is_quad_flip_v3(v1, v2, v3, v0);
  if (UNLIKELY(flip_flag & (1 << 0))) {
    return QuadDirection::Edge_0_2;
  }
  if (UNLIKELY(flip_flag & (1 << 1))) {
    return QuadDirection::Edge_1_3;
  }
  return BLI_polyfill_edge_calc_rotate_beauty__area(v1, v2, v3, v0, false) > 0.0f ?
             QuadDirection::Edge_0_2 :
             QuadDirection::Edge_1_3;
}

static void calc_quad_directions(const Span<float3> positions,
                                 const Span<int> face_offsets,
                                 const Span<int> corner_verts,
                                 const TriangulateQuadMode quad_mode,
                                 Vector<float2> &distances,
                                 MutableSpan<QuadDirection> directions)
{
  if (quad_mode == TriangulateQuadMode::Fixed) {
    directions.fill(QuadDirection::Edge_0_2);
  }
  else if (quad_mode == TriangulateQuadMode::Alternate) {
    directions.fill(QuadDirection::Edge_1_3);
  }
  else {
    distances.reinitialize(face_offsets.size());
    for (const int i : face_offsets.index_range()) {
      const Span<int> face_verts = corner_verts.slice(face_offsets[i], 4);
      distances[i][0] = math::distance_squared(positions[face_verts[0]], positions[face_verts[2]]);
      distances[i][1] = math::distance_squared(positions[face_verts[1]], positions[face_verts[3]]);
    }
    if (quad_mode == TriangulateQuadMode::ShortEdge) {
      for (const int i : face_offsets.index_range()) {
        directions[i] = distances[i][0] < distances[i][1] ? QuadDirection::Edge_0_2 :
                                                            QuadDirection::Edge_1_3;
      }
    }
    else if (quad_mode == TriangulateQuadMode::LongEdge) {
      for (const int i : face_offsets.index_range()) {
        directions[i] = distances[i][0] < distances[i][1] ? QuadDirection::Edge_1_3 :
                                                            QuadDirection::Edge_0_2;
      }
    }
  }
  if (quad_mode == TriangulateQuadMode::Beauty) {
    for (const int i : face_offsets.index_range()) {
      const Span<int> face_verts = corner_verts.slice(face_offsets[i], 4);
      directions[i] = calc_quad_direction_beauty(positions[face_verts[0]],
                                                 positions[face_verts[1]],
                                                 positions[face_verts[2]],
                                                 positions[face_verts[3]]);
    }
  }
}

static void calc_quad_corner_maps(const Span<int> face_offsets,
                                  const Span<QuadDirection> directions,
                                  MutableSpan<int> corner_map)
{
  for (const int i : face_offsets.index_range()) {
    MutableSpan<int> quad_map = corner_map.slice(6 * i, 6);
    /* These corner orders give new edges based on the first vertex of each triangle. */
    switch (directions[i]) {
      case QuadDirection::Edge_0_2:
        quad_map.copy_from({2, 0, 1, 0, 2, 3});
        break;
      case QuadDirection::Edge_1_3:
        quad_map.copy_from({1, 3, 0, 3, 1, 2});
        break;
    }
    const int face_start = face_offsets[i];
    for (int &i : quad_map) {
      i += face_start;
    }
  }
}

static void calc_quad_corner_maps(const Span<float3> positions,
                                  const OffsetIndices<int> src_faces,
                                  const Span<int> src_corner_verts,
                                  const IndexMask &quads,
                                  const TriangulateQuadMode quad_mode,
                                  MutableSpan<int> corner_map)
{
  struct TLS {
    Vector<int> offsets;
    Vector<float2> distances;
    Vector<QuadDirection> directions;
  };
  threading::EnumerableThreadSpecific<TLS> tls;

  quads.foreach_segment(GrainSize(1024), [&](const IndexMaskSegment quads, const int64_t pos) {
    TLS &data = tls.local();
    data.directions.reinitialize(quads.size());

    /* Find the offsets of each face in the local selection. We can gather them together even if
     * they arnen't contiguous because we only need to know the start of each face; the size is
     * just four. */
    const Span<int> offsets = gather_or_reference(src_faces.data(), quads, data.offsets);
    calc_quad_directions(
        positions, offsets, src_corner_verts, quad_mode, data.distances, data.directions);
    const IndexRange corners(pos * 6, offsets.size() * 6);
    calc_quad_corner_maps(offsets, data.directions, corner_map.slice(corners));
  });
}

/**
 * Each triangulated quad creates one additional edge in the result mesh, between the two
 * triangles. The corner_verts are just the corners of the quads, and the edges are just the new
 * edges for these quads.
 */
static void calc_quad_edges(const Span<int> corner_verts, MutableSpan<int2> edges)
{
  const int quads_num = corner_verts.size() / 6;
  for (const int quad : IndexRange(quads_num)) {
    const Span<int> quad_verts = corner_verts.slice(6 * quad, 6);
    /* Use the first vertex of each triangle. */
    edges[quad] = int2(quad_verts[0], quad_verts[1]);
  }
}

static void calc_quad_corner_edges(const Span<int> src_corner_edges,
                                   const Span<int> corner_map,
                                   const int edges_start,
                                   MutableSpan<int> corner_edges)
{
  /* Each triangle starts at the new edge and winds in the same order as corner vertices
   * described by the corner map. */
  for (const int tri : IndexRange(corner_map.size() / 3)) {
    const int tri_start = 3 * tri;
    corner_edges[tri_start] = edges_start + tri / 2;
    corner_edges[tri_start + 1] = src_corner_edges[corner_map[tri_start + 1]];
    corner_edges[tri_start + 2] = src_corner_edges[corner_map[tri_start + 2]];
  }
}

static void calc_quad_edges(const Span<int> src_corner_edges,
                            const Span<int> corner_map,
                            const Span<int> corner_verts,
                            const int edges_start,
                            MutableSpan<int2> edges,
                            MutableSpan<int> quad_corner_edges)
{
  const int quads_num = corner_map.size() / 6;
  threading::parallel_for(IndexRange(quads_num), 1024, [&](const IndexRange quads) {
    const IndexRange corners(quads.start() * 6, quads.size() * 6);
    calc_quad_edges(corner_verts.slice(corners), edges.slice(quads));
    // TODO: Loop fission?
    calc_quad_corner_edges(src_corner_edges,
                           corner_map.slice(corners),
                           edges_start + quads.start(),
                           quad_corner_edges.slice(corners));
  });
}

static OffsetIndices<int> calc_tris_by_ngon(const OffsetIndices<int> src_faces,
                                            const IndexMask &ngons,
                                            MutableSpan<int> face_offset_data)
{
  ngons.foreach_index(GrainSize(2048), [&](const int face, const int mask) {
    face_offset_data[mask] = bke::mesh::face_triangles_num(src_faces[face].size());
  });
  return offset_indices::accumulate_counts_to_offsets(face_offset_data);
}

static OffsetIndices<int> calc_edges_by_ngon(const OffsetIndices<int> src_faces,
                                             const IndexMask &selection,
                                             MutableSpan<int> edge_offset_data)
{
  selection.foreach_index(GrainSize(2048), [&](const int face, const int mask) {
    /* The number of new inner edges for each face is the number of corners - 3. */
    edge_offset_data[mask] = src_faces[face].size() - 3;
  });
  return offset_indices::accumulate_counts_to_offsets(edge_offset_data);
}

static void calc_ngon_corner_maps(const Span<float3> positions,
                                  const OffsetIndices<int> src_faces,
                                  const Span<int> src_corner_verts,
                                  const Span<float3> face_normals,
                                  const IndexMask &ngons,
                                  const OffsetIndices<int> tris_by_ngon,
                                  const TriangulateNGonMode ngon_mode,
                                  MutableSpan<int> corner_map)
{
  struct TLS {
    Vector<float3x3> projections;
    Array<int> offset_data;
    Vector<float2> projected_positions;

    /* Only used for the "Beauty" method. */
    MemArena *arena = nullptr;
    Heap *heap = nullptr;

    Set<int> duplicate_faces;

    ~TLS()
    {
      if (arena) {
        BLI_memarena_free(arena);
      }
      if (heap) {
        BLI_heap_free(heap, nullptr);
      }
    }
  };
  threading::EnumerableThreadSpecific<TLS> tls;

  ngons.foreach_segment(GrainSize(128), [&](const IndexMaskSegment ngons, const int pos) {
    TLS &data = tls.local();

    /* In order to simplify and "parallelize" the next loops, gather offsets used to group an array
     * large enough for all the local face corners. */
    data.offset_data.reinitialize(ngons.size() + 1);
    const OffsetIndices local_corner_offsets = offset_indices::gather_selected_offsets(
        src_faces, ngons, data.offset_data);

    /* Use face normals to build projection matrices to make the face positions 2D. */
    data.projections.reinitialize(ngons.size());
    MutableSpan<float3x3> projections = data.projections;
    if (face_normals.is_empty()) {
      for (const int i : ngons.index_range()) {
        const IndexRange src_face = src_faces[ngons[i]];
        const Span<int> face_verts = src_corner_verts.slice(src_face);
        const float3 normal = bke::mesh::face_normal_calc(positions, face_verts);
        axis_dominant_v3_to_m3_negate(projections[i].ptr(), normal);
      }
    }
    else {
      for (const int i : ngons.index_range()) {
        axis_dominant_v3_to_m3_negate(projections[i].ptr(), face_normals[ngons[i]]);
      }
    }

    /* Project the face positions into 2D using the matrices calculated above. */
    data.projected_positions.reinitialize(local_corner_offsets.total_size());
    MutableSpan<float2> projected_positions = data.projected_positions;
    for (const int i : ngons.index_range()) {
      const IndexRange src_face = src_faces[ngons[i]];
      const Span<int> face_verts = src_corner_verts.slice(src_face);
      const float3x3 &matrix = projections[i];

      MutableSpan<float2> positions_2d = projected_positions.slice(local_corner_offsets[i]);
      for (const int i : face_verts.index_range()) {
        mul_v2_m3v3(positions_2d[i], matrix.ptr(), positions[face_verts[i]]);
      }
    }

    if (ngon_mode == TriangulateNGonMode::Beauty) {
      if (!data.arena) {
        data.arena = BLI_memarena_new(BLI_POLYFILL_ARENA_SIZE, __func__);
      }
      if (!data.heap) {
        data.heap = BLI_heap_new_ex(BLI_POLYFILL_ALLOC_NGON_RESERVE);
      }
    }

    /* Calculate the triangulation of corners indices local to each face. */
    for (const int i : ngons.index_range()) {
      const Span<float2> positions_2d = projected_positions.slice(local_corner_offsets[i]);
      const IndexRange tris_range = tris_by_ngon[pos + i];
      MutableSpan<int> map = corner_map.slice(tris_range.start() * 3, tris_range.size() * 3);
      BLI_polyfill_calc(reinterpret_cast<const float(*)[2]>(positions_2d.data()),
                        positions_2d.size(),
                        1,
                        reinterpret_cast<uint(*)[3]>(map.data()));
      if (ngon_mode == TriangulateNGonMode::Beauty) {
        BLI_polyfill_beautify(reinterpret_cast<const float(*)[2]>(positions_2d.data()),
                              positions_2d.size(),
                              reinterpret_cast<uint(*)[3]>(map.data()),
                              data.arena,
                              data.heap);
        BLI_memarena_clear(data.arena);
      }
    }

    /* "Globalize" the triangulation created above so the map source indices reference _all_ of the
     * source vertices, not just within the source face. */
    for (const int i : ngons.index_range()) {
      const IndexRange src_face = src_faces[ngons[i]];
      const IndexRange tris_range = tris_by_ngon[pos + i];
      MutableSpan<int> map = corner_map.slice(tris_range.start() * 3, tris_range.size() * 3);
      for (int &vert : map) {
        vert += src_face.start();
      }
    }
  });
}

static void calc_inner_triangles(const IndexRange src_face,
                                 const Span<int> src_corner_verts,
                                 const Span<int> src_corner_edges,
                                 const Span<int3> corner_tris,
                                 const int edges_start,
                                 MutableSpan<int> corner_edges,
                                 VectorSet<OrderedEdge> &deduplication)
{
  const OrderedEdge last_edge(int(src_face.first()), int(src_face.last()));
  auto add_edge = [&](const OrderedEdge corner_edge) -> int {
    if (corner_edge == last_edge) {
      return src_corner_edges[src_face.last()];
    }
    if (corner_edge.v_high == corner_edge.v_low + 1) {
      return src_corner_edges[corner_edge.v_low];
    }
    const OrderedEdge vert_edge(src_corner_verts[corner_edge.v_low],
                                src_corner_verts[corner_edge.v_high]);
    return edges_start + deduplication.index_of_or_add(vert_edge);
  };

  for (const int i : corner_tris.index_range()) {
    const int3 tri = corner_tris[i];
    corner_edges[3 * i + 0] = add_edge({tri[0], tri[1]});
    corner_edges[3 * i + 1] = add_edge({tri[1], tri[2]});
    corner_edges[3 * i + 2] = add_edge({tri[2], tri[0]});
  }
}

static void calc_ngon_edges(const OffsetIndices<int> src_faces,
                            const Span<int> src_corner_verts,
                            const Span<int> src_corner_edges,
                            const IndexMask &ngons,
                            const OffsetIndices<int> tris_by_ngon,
                            const OffsetIndices<int> edges_by_ngon,
                            const OffsetIndices<int> faces,
                            const int edges_start,
                            const Span<int> corner_verts,
                            MutableSpan<int2> inner_edges,
                            MutableSpan<int> corner_edges)
{
  threading::EnumerableThreadSpecific<VectorSet<OrderedEdge>> tls;
  ngons.foreach_segment(GrainSize(128), [&](const IndexMaskSegment ngons, const int pos) {
    VectorSet<OrderedEdge> &deduplication = tls.local();
    for (const int i : ngons.index_range()) {
      const IndexRange edges = edges_by_ngon[pos + i];
      const IndexRange corners = faces[tris_by_ngon[pos + i]];
      deduplication.clear();
      calc_inner_triangles(src_faces[ngons[i]],
                           src_corner_verts,
                           src_corner_edges,
                           corner_verts.slice(corners).cast<int3>(),
                           edges_start + edges.start(),
                           corner_edges.slice(corners),
                           deduplication);
      inner_edges.slice(edges).copy_from(deduplication.as_span().cast<int2>());
    }
  });
}

struct TriangleRef {
  const int *verts;
  uint64_t hash() const
  {
    return get_default_hash_3(verts[0], verts[1], verts[2]);
  }
  friend bool operator==(const TriangleRef &a, const TriangleRef &b)
  {
    return a.verts[0] == b.verts[0] && a.verts[1] == b.verts[1] && a.verts[2] == b.verts[2];
  }
};

static IndexMask remove_duplicated_tris(const OffsetIndices<int> src_faces,
                                        const Span<int> src_corner_verts,
                                        const IndexMask &unselected,
                                        const IndexMask &src_tris,
                                        const Span<int> corner_map,
                                        IndexMaskMemory &memory)
{
  Array<bool> non_duplicates(src_faces.size());
  unselected.to_bools(non_duplicates);

  threading::EnumerableThreadSpecific<Map<TriangleRef, int>> maps;
  threading::parallel_for(src_tris.index_range(), 1024, [&](const IndexRange range) {
    Map<TriangleRef, int> &map = maps.local();
    src_tris.slice(range).foreach_index([&](const int i) {
      const TriangleRef tri{&src_corner_verts[src_faces[i].start()]};
      map.add_new(tri, i);
    });
  });

  threading::parallel_for_aligned(
      src_tris.index_range(), 1024, bits::BitsPerInt, [&](const IndexRange range) {
        for (const int i : range) {
          const int3 new_tri(src_corner_verts[corner_map[3 * i] + 0],
                             src_corner_verts[corner_map[3 * i] + 1],
                             src_corner_verts[corner_map[3 * i] + 2]);
          if (std::any_of(maps.begin(), maps.end(), [&](const Map<TriangleRef, int> &map) {
                return map.contains({new_tri});
              }))
          {
            non_duplicates[i] = false;
          }
        }
      });

  return IndexMask::from_bools(non_duplicates, memory);
}

template<typename T>
static void copy_quad_face_data_to_tris(const Span<T> src,
                                        const IndexMask &quads,
                                        MutableSpan<T> dst)
{
  quads.foreach_index_optimized<int>([&](const int src_i, const int dst_i) {
    dst[2 * dst_i + 0] = src[src_i];
    dst[2 * dst_i + 1] = src[src_i];
  });
}

static void copy_quad_face_data_to_tris(const GSpan src, const IndexMask &quads, GMutableSpan dst)
{
  bke::attribute_math::convert_to_static_type(src.type(), [&](auto dummy) {
    using T = decltype(dummy);
    copy_quad_face_data_to_tris(src.typed<T>(), quads, dst.typed<T>());
  });
}

std::optional<Mesh *> mesh_triangulate(
    const Mesh &src_mesh,
    const IndexMask &selection,
    const TriangulateNGonMode ngon_mode,
    const TriangulateQuadMode quad_mode,
    const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  const Span<float3> positions = src_mesh.vert_positions();
  const Span<int2> src_edges = src_mesh.edges();
  const OffsetIndices src_faces = src_mesh.faces();
  const Span<int> src_corner_verts = src_mesh.corner_verts();
  const Span<int> src_corner_edges = src_mesh.corner_edges();
  const bke::AttributeAccessor src_attributes = src_mesh.attributes();
#ifdef DEBUG
  selection.foreach_index([&](const int i) { BLI_assert(src_faces[i].size() > 3); });
#endif

  /* Divide the input selection into separate selections for each face type. This isn't necessary
   * for correctness, but considering groups of each face type separately simplifies optimizing
   * for each type. For example, quad triangulation is much simpler than Ngon triangulation. */
  IndexMaskMemory memory;
  IndexMask unselected = selection.complement(src_faces.index_range(), memory);

  const IndexMask src_tris = IndexMask::from_predicate(
      unselected, GrainSize(4096), memory, [&](const int i) { return src_faces[i].size() == 3; });
  const IndexMask quads = IndexMask::from_predicate(
      selection, GrainSize(4096), memory, [&](const int i) { return src_faces[i].size() == 4; });
  const IndexMask ngons = IndexMask::from_predicate(
      selection, GrainSize(4096), memory, [&](const int i) { return src_faces[i].size() > 4; });
  if (quads.is_empty() && ngons.is_empty()) {
    /* All selected faces are already triangles. */
    return std::nullopt;
  }

  /* Calculate group of triangle indices for each selected Ngon to facilitate calculating them in
   * parallel later. */
  Array<int> tri_offsets(ngons.size() + 1);
  const OffsetIndices tris_by_ngon = calc_tris_by_ngon(src_faces, ngons, tri_offsets);
  const int ngon_tris_num = tris_by_ngon.total_size();
  const int quad_tris_num = quads.size() * 2;
  const IndexRange tris_range(ngon_tris_num + quad_tris_num);
  const IndexRange ngon_tris_range = tris_range.take_front(ngon_tris_num);
  const IndexRange quad_tris_range = tris_range.take_front(quad_tris_num);

  const int ngon_corners_num = tris_by_ngon.total_size() * 3;
  const int quad_corners_num = quads.size() * 6;
  const IndexRange tri_corners_range(quad_corners_num + ngon_corners_num);
  const IndexRange ngon_corners_range = tri_corners_range.take_front(ngon_corners_num);
  const IndexRange quad_corners_range = tri_corners_range.take_back(quad_corners_num);

  /* Calculate groups of new inner edges for each selected Ngon so they can be filled in parallel
   * later. */
  Array<int> edge_offset_data(ngons.size() + 1);
  const OffsetIndices edges_by_ngon = calc_edges_by_ngon(src_faces, ngons, edge_offset_data);
  const int ngon_edges_num = edges_by_ngon.total_size();
  const int quad_edges_num = quads.size();
  const IndexRange src_edges_range(0, src_edges.size());
  const IndexRange tri_edges_range(src_edges_range.one_after_last(),
                                   ngon_edges_num + quad_edges_num);
  const IndexRange ngon_edges_range = tri_edges_range.take_front(ngon_edges_num);
  const IndexRange quad_edges_range = tri_edges_range.take_front(quad_edges_num);

  /* An index map that maps from newly created corners in `tri_corners_range` to original corner
   * indices. This is used to interpolate `corner_vert` indices and face corner attributes. If
   * there are no face corner attributes, theoretically the map could be skipped and corner
   * vertex indices could be interpolated immediately, but that isn't done for simplicity. */
  Array<int> corner_map(tri_corners_range.size());

  if (!ngons.is_empty()) {
    calc_ngon_corner_maps(positions,
                          src_faces,
                          src_corner_verts,
                          face_normals_if_worthwhile(src_mesh, ngons.size()),
                          ngons,
                          tris_by_ngon,
                          ngon_mode,
                          corner_map.as_mutable_span().take_front(ngon_corners_num));
  }
  if (!quads.is_empty()) {
    calc_quad_corner_maps(positions,
                          src_faces,
                          src_corner_verts,
                          quads,
                          quad_mode,
                          corner_map.as_mutable_span().take_back(quad_corners_num));
  }

  if (!src_tris.is_empty()) {
    unselected = remove_duplicated_tris(
        src_faces, src_corner_verts, unselected, src_tris, corner_map, memory);
  }
  const IndexRange unselected_range(tris_range.one_after_last(), unselected.size());

  /* Create a mesh with no face corners. We don't know the number of corners from unselected faces.
   * We have to create the face offsets anyway, this will give us the number of corners. */
  Mesh *mesh = create_mesh_no_attributes(src_mesh,
                                         src_mesh.totvert,
                                         tri_edges_range.size() + src_edges.size(),
                                         tris_range.size() + unselected.size(),
                                         0);

  /* Find the face corner ranges using the offsets array from the new mesh. That gives us the
   * final number of face corners. */
  const OffsetIndices faces = calc_faces(src_faces, unselected, mesh->face_offsets_for_write());
  mesh->totloop = faces.total_size();
  const OffsetIndices faces_unselected = faces.slice(unselected_range);

  bke::MutableAttributeAccessor attributes = mesh->attributes_for_write();
  attributes.add<int>(".corner_vert", ATTR_DOMAIN_CORNER, bke::AttributeInitConstruct());
  attributes.add<int>(".corner_edge", ATTR_DOMAIN_CORNER, bke::AttributeInitConstruct());
  attributes.add<int2>(".edge_verts", ATTR_DOMAIN_EDGE, bke::AttributeInitConstruct());

  MutableSpan<int2> edges = mesh->edges_for_write();
  MutableSpan<int> corner_verts = mesh->corner_verts_for_write();
  MutableSpan<int> corner_edges = mesh->corner_edges_for_write();

  if (!ngons.is_empty()) {
    const Span<int> ngon_corner_map = corner_map.as_mutable_span().take_back(ngon_corners_num);
    bke::attribute_math::gather(
        src_corner_verts, ngon_corner_map, corner_verts.slice(ngon_corners_range));
    calc_ngon_edges(src_faces,
                    src_corner_verts,
                    src_corner_edges,
                    ngons,
                    tris_by_ngon,
                    edges_by_ngon,
                    faces.slice(ngon_tris_range),
                    ngon_edges_range.start(),
                    corner_map.as_mutable_span().take_front(ngon_corners_num),
                    edges.slice(ngon_edges_range),
                    corner_edges);
  }

  if (!quads.is_empty()) {
    const Span<int> quad_corner_map = corner_map.as_mutable_span().take_back(quad_corners_num);
    array_utils::gather(src_corner_verts, quad_corner_map, corner_verts.slice(quad_corners_range));
    calc_quad_edges(src_corner_edges,
                    quad_corner_map,
                    corner_verts,
                    quad_edges_range.start(),
                    edges.slice(quad_edges_range),
                    corner_edges.slice(quad_corners_range));
  }

  /* Vertex attributes are totally unnaffected and can be shared with implicit sharing.
   * Use the #CustomData API for better support for vertex groups. */
  CustomData_merge(&src_mesh.vert_data, &mesh->vert_data, CD_MASK_MESH.vmask, mesh->totvert);

  array_utils::copy(src_edges, edges.take_back(src_edges.size()));
  for (auto &attribute : bke::retrieve_attributes_for_transfer(
           src_attributes, attributes, ATTR_DOMAIN_MASK_EDGE, propagation_info, {".edge_verts"}))
  {
    attribute.dst.span.slice(src_edges_range).copy_from(attribute.src);
    GMutableSpan new_data = attribute.dst.span.slice(tri_edges_range);
    /* It would be reasonable interpolate data from connected edges within each face.
     * Currently the data from new edges is just set to the type's default value. */
    const void *default_value = new_data.type().default_value();
    new_data.type().fill_assign_n(default_value, new_data.data(), new_data.size());
    attribute.dst.finish();
  }

  if (CustomData_has_layer(&src_mesh.edge_data, CD_ORIGINDEX)) {
    const Span src(
        static_cast<const int *>(CustomData_get_layer(&src_mesh.edge_data, CD_ORIGINDEX)),
        src_mesh.totedge);
    MutableSpan dst(static_cast<int *>(CustomData_add_layer(
                        &mesh->edge_data, CD_ORIGINDEX, CD_CONSTRUCT, mesh->totedge)),
                    mesh->totedge);
    dst.slice(tri_edges_range).fill(ORIGINDEX_NONE);
    array_utils::copy(src, dst.slice(src_edges_range));
  }

  for (auto &attribute : bke::retrieve_attributes_for_transfer(
           src_attributes, attributes, ATTR_DOMAIN_MASK_FACE, propagation_info, {}))
  {
    bke::attribute_math::gather_to_groups(
        tris_by_ngon, ngons, attribute.src, attribute.dst.span.slice(ngon_tris_range));
    copy_quad_face_data_to_tris(attribute.src, quads, attribute.dst.span.slice(quad_tris_range));
    array_utils::gather(attribute.src, unselected, attribute.dst.span.slice(unselected_range));
    attribute.dst.finish();
  }

  if (CustomData_has_layer(&src_mesh.face_data, CD_ORIGINDEX)) {
    const Span src(
        static_cast<const int *>(CustomData_get_layer(&src_mesh.face_data, CD_ORIGINDEX)),
        src_mesh.faces_num);
    MutableSpan dst(static_cast<int *>(CustomData_add_layer(
                        &mesh->face_data, CD_ORIGINDEX, CD_CONSTRUCT, mesh->faces_num)),
                    mesh->faces_num);
    bke::attribute_math::gather_to_groups(tris_by_ngon, ngons, src, dst.slice(ngon_tris_range));
    copy_quad_face_data_to_tris(src, quads, dst.slice(quad_tris_range));
    array_utils::gather(src, unselected, dst.slice(unselected_range));
  }

  array_utils::gather_group_to_group(
      src_faces, faces_unselected, unselected, src_corner_edges, corner_edges);
  array_utils::gather_group_to_group(
      src_faces, faces_unselected, unselected, src_corner_verts, corner_verts);
  for (auto &attribute : bke::retrieve_attributes_for_transfer(src_attributes,
                                                               attributes,
                                                               ATTR_DOMAIN_MASK_CORNER,
                                                               propagation_info,
                                                               {".corner_vert", ".corner_edge"}))
  {
    bke::attribute_math::gather_group_to_group(
        src_faces, faces_unselected, unselected, attribute.src, attribute.dst.span);
    bke::attribute_math::gather(
        attribute.src, corner_map.as_span(), attribute.dst.span.slice(tri_corners_range));
    attribute.dst.finish();
  }

  mesh->runtime->bounds_cache = src_mesh.runtime->bounds_cache;
  copy_loose_vert_hint(src_mesh, *mesh);
  copy_loose_edge_hint(src_mesh, *mesh);
  return mesh;
}

std::optional<Mesh *> mesh_triangulate(
    const Mesh &src_mesh,
    const fn::Field<bool> &selection_field,
    const TriangulateNGonMode ngon_mode,
    const TriangulateQuadMode quad_mode,
    const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  const bke::MeshFieldContext context{src_mesh, ATTR_DOMAIN_FACE};
  fn::FieldEvaluator evaluator{context, src_mesh.faces_num};
  evaluator.add(selection_field);
  evaluator.evaluate();
  // TODO: Remove triangles from selection.
  const IndexMask selection = evaluator.get_evaluated_as_mask(0);
  if (selection.is_empty()) {
    return std::nullopt;
  }
  return mesh_triangulate(src_mesh, selection, ngon_mode, quad_mode, propagation_info);
}

}  // namespace blender::geometry
