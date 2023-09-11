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
#include "BKE_mesh.hh"
#include "BKE_mesh_mapping.hh"

#include "GEO_mesh_triangulate.hh"

namespace blender::geometry {

static OffsetIndices<int> calc_faces(const OffsetIndices<int> src_faces,
                                     const IndexMask &unselected_range,
                                     MutableSpan<int> face_offsets)
{
  /* Unselected faces are moved to the start of the mesh. */
  offset_indices::gather_selected_offsets(
      src_faces, unselected_range, face_offsets.take_front(unselected_range.size() + 1));

  /* All other faces are triangles added to the end of the mesh. */
  MutableSpan<int> new_tri_offsets = face_offsets.drop_front(unselected_range.size());
  offset_indices::fill_constant_group_size(3, new_tri_offsets.first(), new_tri_offsets);
  return OffsetIndices<int>(face_offsets);
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

static void calc_quad_directions(const Span<float3> positions,
                                 const OffsetIndices<int> faces,
                                 const Span<int> corner_verts,
                                 const TriangulateQuadMode quad_mode,
                                 const IndexMaskSegment quads,
                                 MutableSpan<QuadDirection> directions)
{
  if (quad_mode == TriangulateQuadMode::Fixed) {
    directions.fill(QuadDirection::Edge_0_2);
  }
  else if (quad_mode == TriangulateQuadMode::Alternate) {
    directions.fill(QuadDirection::Edge_1_3);
  }
  else {
    // TODO: Test simplifying this thing ;)
    Array<float, 128> distances_0_2(quads.size());
    Array<float, 128> distances_1_3(quads.size());
    for (const int i : quads.index_range()) {
      const Span<int> face_verts = corner_verts.slice(faces[quads[i]]);
      distances_0_2[i] = math::distance_squared(positions[face_verts[0]],
                                                positions[face_verts[2]]);
      distances_1_3[i] = math::distance_squared(positions[face_verts[1]],
                                                positions[face_verts[3]]);
    }
    if (quad_mode == TriangulateQuadMode::ShortEdge) {
      for (const int i : quads.index_range()) {
        directions[i] = distances_0_2[i] < distances_1_3[i] ? QuadDirection::Edge_0_2 :
                                                              QuadDirection::Edge_1_3;
      }
    }
    else if (quad_mode == TriangulateQuadMode::LongEdge) {
      for (const int i : quads.index_range()) {
        directions[i] = distances_0_2[i] < distances_1_3[i] ? QuadDirection::Edge_1_3 :
                                                              QuadDirection::Edge_0_2;
      }
    }
  }
  if (quad_mode == TriangulateQuadMode::Beauty) {
    // TODO
  }
}

static void calc_quad_corner_maps(const OffsetIndices<int> faces,
                                  const IndexMaskSegment quads,
                                  const Span<QuadDirection> directions,
                                  MutableSpan<int> corner_map)
{
  for (const int i : quads.index_range()) {
    MutableSpan<int> quad_map = corner_map.slice(6 * i, 6);
    switch (directions[i]) {
      case QuadDirection::Edge_0_2:
        quad_map.copy_from({0, 1, 2, 2, 3, 0});
        break;
      case QuadDirection::Edge_1_3:
        quad_map.copy_from({0, 1, 3, 1, 2, 3});
        break;
    }
    const int face_start = faces[quads[i]].start();
    for (int &i : quad_map) {
      i += face_start;
    }
  }
}

static void calc_quad_edges(const OffsetIndices<int> src_faces,
                            const Span<int> src_corner_verts,
                            const IndexMaskSegment quads,
                            const Span<QuadDirection> directions,
                            MutableSpan<int2> edges)
{
  for (const int i : quads.index_range()) {
    const Span<int> face_verts = src_corner_verts.slice(src_faces[quads[i]]);
    switch (directions[i]) {
      case QuadDirection::Edge_0_2:
        edges[i] = int2(face_verts[0], face_verts[2]);
        break;
      case QuadDirection::Edge_1_3:
        edges[i] = int2(face_verts[1], face_verts[3]);
        break;
    }
  }
}

static void calc_quad_corner_edges(const OffsetIndices<int> src_faces,
                                   const Span<int> src_corner_edges,
                                   const IndexMaskSegment quads,
                                   const Span<QuadDirection> directions,
                                   const int edges_start,
                                   MutableSpan<int> corner_edges)
{
  for (const int i : quads.index_range()) {
    const Span<int> src = src_corner_edges.slice(src_faces[quads[i]]);
    const int new_edge = edges_start + i;
    MutableSpan<int> dst = corner_edges.slice(6 * i, 6);
    switch (directions[i]) {
      case QuadDirection::Edge_0_2:
        dst.copy_from({src[0], src[1], new_edge, src[2], src[3], new_edge});
        break;
      case QuadDirection::Edge_1_3:
        dst.copy_from({src[0], new_edge, src[3], src[1], src[2], new_edge});
        break;
    }
  }
}

static void triangulate_quads(const Span<float3> positions,
                              const OffsetIndices<int> src_faces,
                              const Span<int> src_corner_verts,
                              const Span<int> src_corner_edges,
                              const IndexMask &quads,
                              const TriangulateQuadMode quad_mode,
                              const int edges_start,
                              MutableSpan<int> corner_map,
                              MutableSpan<int2> edges,
                              MutableSpan<int> quad_corner_edges)
{
  quads.foreach_segment(GrainSize(1024), [&](const IndexMaskSegment quads, const int64_t pos) {
    // TODO: TLS
    Array<QuadDirection, 1024> directions(quads.size());
    calc_quad_directions(positions, src_faces, src_corner_verts, quad_mode, quads, directions);

    const IndexRange corners(pos * 6, quads.size() * 6);
    calc_quad_corner_maps(src_faces, quads, directions, corner_map.slice(corners));
    calc_quad_edges(
        src_faces, src_corner_verts, quads, directions, edges.slice(pos, quads.size()));
    calc_quad_corner_edges(src_faces,
                           src_corner_edges,
                           quads,
                           directions,
                           edges_start + pos,
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

static OffsetIndices<int> gather_selected_offsets(const OffsetIndices<int> src,
                                                  const IndexMaskSegment mask,
                                                  MutableSpan<int> dst)
{
  for (const int i : mask.index_range()) {
    dst[i] = src[mask[i]].size();
  }
  return offset_indices::accumulate_counts_to_offsets(dst);
}

static VectorSet<OrderedEdge> calc_inner_triangles(const Span<int> face_verts,
                                                   const Span<int> face_edges,
                                                   const int edge_offset,
                                                   const Span<int3> tris,
                                                   MutableSpan<int> corner_edges,
                                                   VectorSet<OrderedEdge> &inner_edges)
{
  auto add_edge = [&](const OrderedEdge corner_edge) -> int {
    if (corner_edge == OrderedEdge(0, face_verts.size() - 1)) {
      return face_edges.last();
    }
    if (corner_edge.v_high == corner_edge.v_low + 1) {
      return face_edges[corner_edge.v_low];
    }
    const OrderedEdge edge(face_verts[corner_edge.v_low], face_verts[corner_edge.v_high]);
    return edge_offset + inner_edges.index_of_or_add(edge);
  };

  for (const int i : tris.index_range()) {
    const int3 tri = tris[i];
    corner_edges[3 * i + 0] = add_edge({tri[0], tri[1]});
    corner_edges[3 * i + 1] = add_edge({tri[1], tri[2]});
    corner_edges[3 * i + 2] = add_edge({tri[2], tri[0]});
  }

  return inner_edges;
}

static void triangulate_ngons(const Span<float3> positions,
                              const OffsetIndices<int> src_faces,
                              const Span<int> src_corner_verts,
                              const Span<int> src_corner_edges,
                              const Span<float3> face_normals,
                              const IndexMask &ngons,
                              const OffsetIndices<int> tris_by_ngon,
                              const OffsetIndices<int> edges_by_ngon,
                              const TriangulateNGonMode ngon_mode,
                              const OffsetIndices<int> faces,
                              const int edges_start,
                              const int corner_map_offset,
                              MutableSpan<int> corner_map,
                              MutableSpan<int2> inner_edges,
                              MutableSpan<int> corner_edges)
{
  struct TLS {
    Vector<float3x3> projections;
    Array<int> offset_data;
    Vector<float2> projected_positions;
    VectorSet<OrderedEdge> inner_edges;

    /* Only used for the "Beauty" method. */
    MemArena *arena = nullptr;
    Heap *heap = nullptr;

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

    // TODO: Test just using iterator variables rather than offsets within the task.
    data.offset_data.reinitialize(ngons.size() + 1);
    const OffsetIndices offsets = gather_selected_offsets(src_faces, ngons, data.offset_data);

    data.projected_positions.reinitialize(offsets.total_size());
    MutableSpan<float2> projected_positions = data.projected_positions;
    for (const int i : ngons.index_range()) {
      const IndexRange src_face = src_faces[ngons[i]];
      const Span<int> face_verts = src_corner_verts.slice(src_face);
      const float3x3 &matrix = projections[i];

      MutableSpan<float2> positions_2d = projected_positions.slice(offsets[i]);
      for (const int i : face_verts.index_range()) {
        mul_v2_m3v3(positions_2d[i], matrix.ptr(), positions[face_verts[i]]);
      }
    }

    for (const int i : ngons.index_range()) {
      const Span<float2> positions_2d = projected_positions.slice(offsets[i]);
      const IndexRange tris = tris_by_ngon[pos + i];
      const IndexRange corners = faces[tris];
      MutableSpan<int> map = corner_map.slice(corners.shift(-corner_map_offset));
      BLI_polyfill_calc(reinterpret_cast<const float(*)[2]>(positions_2d.data()),
                        positions_2d.size(),
                        1,
                        reinterpret_cast<uint(*)[3]>(map.cast<uint3>().data()));
      if (ngon_mode == TriangulateNGonMode::Beauty) {
        if (!data.arena) {
          data.arena = BLI_memarena_new(BLI_POLYFILL_ARENA_SIZE, __func__);
        }
        if (!data.heap) {
          data.heap = BLI_heap_new_ex(BLI_POLYFILL_ALLOC_NGON_RESERVE);
        }
        BLI_polyfill_beautify(reinterpret_cast<const float(*)[2]>(positions_2d.data()),
                              positions_2d.size(),
                              reinterpret_cast<uint(*)[3]>(map.cast<uint3>().data()),
                              data.arena,
                              data.heap);
      }
    }

    for (const int i : ngons.index_range()) {
      const IndexRange src_face = src_faces[ngons[i]];
      const IndexRange tris = tris_by_ngon[pos + i];
      const IndexRange edges = edges_by_ngon[pos + i];
      const IndexRange corners = faces[tris];
      MutableSpan<int> map = corner_map.slice(corners.shift(-corner_map_offset));

      data.inner_edges.clear();
      calc_inner_triangles(src_corner_verts.slice(src_face),
                           src_corner_edges.slice(src_face),
                           edges_start + edges.start(),
                           map.cast<int3>(),
                           corner_edges.slice(corners),
                           data.inner_edges);
      inner_edges.slice(edges).copy_from(data.inner_edges.as_span().cast<int2>());
    }

    for (const int i : ngons.index_range()) {
      const IndexRange src_face = src_faces[ngons[i]];
      const IndexRange tris = tris_by_ngon[pos + i];
      const IndexRange corners = faces[tris];
      MutableSpan<int> map = corner_map.slice(corners.shift(-corner_map_offset));
      for (int &vert : map) {
        vert += src_face.start();
      }
    }
  });
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

template<typename T>
static void copy_to_quad_tris(const Span<T> src, const IndexMask &quads, MutableSpan<T> dst)
{
  quads.foreach_index_optimized<int>([&](const int src_i, const int dst_i) {
    dst[2 * dst_i + 0] = src[src_i];
    dst[2 * dst_i + 1] = src[src_i];
  });
}

static void copy_to_quad_tris(const GSpan src, const IndexMask &quads, GMutableSpan dst)
{
  bke::attribute_math::convert_to_static_type(src.type(), [&](auto dummy) {
    using T = decltype(dummy);
    copy_to_quad_tris(src.typed<T>(), quads, dst.typed<T>());
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

  /* Divide the input selection into separate selections for each face type. This isn't necessary
   * for correctness, but considering groups of each face type separately simplifies optimizing for
   * each type. For example, quad triangulation is much simpler than Ngon triangulation. */
  IndexMaskMemory memory;
  const IndexMask copy_tris = IndexMask::from_predicate(
      selection, GrainSize(4096), memory, [&](const int i) { return src_faces[i].size() == 3; });
  const IndexMask quads = IndexMask::from_predicate(
      selection, GrainSize(4096), memory, [&](const int i) { return src_faces[i].size() == 4; });
  const IndexMask ngons = IndexMask::from_predicate(
      selection, GrainSize(4096), memory, [&](const int i) { return src_faces[i].size() > 4; });
  if (quads.is_empty() && ngons.is_empty()) {
    /* All selected faces are already triangles. */
    return std::nullopt;
  }

  const IndexMask unselected = selection.complement(src_faces.index_range(), memory);

  /* Calculate group of triangle indices for each selected Ngon to facilitate calculating them in
   * parallel later. */
  Array<int> tri_offsets(ngons.size() + 1);
  const OffsetIndices tris_by_ngon = calc_tris_by_ngon(src_faces, ngons, tri_offsets);

  /* Calculate groups of new inner edges for each selected Ngon so they can be filled in parallel
   * later. */
  Array<int> edge_offset_data(ngons.size() + 1);
  const OffsetIndices edges_by_ngon = calc_edges_by_ngon(src_faces, ngons, edge_offset_data);

  /* Unselected faces are moved to the beginning of the new mesh. */
  const IndexRange unselected_range(0, unselected.size());
  /* Selected faces that are already triangles are copied explicitly after the unselected faces. */
  const IndexRange copy_tris_range(unselected_range.one_after_last(), copy_tris.size());
  /* Triangles created from Ngons. */
  const IndexRange ngon_tris(copy_tris_range.one_after_last(), tris_by_ngon.total_size());
  /* Triangles generated from quads. */
  const IndexRange quad_tris(ngon_tris.one_after_last(), quads.size() * 2);
  /* All newly created triangles (from Ngons and quads). */
  const IndexRange tris(ngon_tris.start(), ngon_tris.size() + quad_tris.size());

  /* New edges from the inner part of Ngons. */
  const IndexRange ngon_edges(src_edges.size(), edges_by_ngon.total_size());
  /* A single edge connecting the opposite corners of each quad. */
  const IndexRange quad_edges(ngon_edges.one_after_last(), quads.size());
  /* All newly created edges (from Ngons and quads). */
  const IndexRange inner_edges(ngon_edges.start(), ngon_edges.size() + quad_edges.size());

  /* Create a mesh with no face corners, since we don't know how many there will be yet. */
  Mesh *mesh = create_mesh_no_attributes(src_mesh,
                                         src_mesh.totvert,
                                         src_edges.size() + inner_edges.size(),
                                         unselected_range.size() + copy_tris_range.size() +
                                             tris.size(),
                                         0);
  bke::MutableAttributeAccessor attributes = mesh->attributes_for_write();
  attributes.add<int2>(".edge_verts", ATTR_DOMAIN_EDGE, bke::AttributeInitConstruct());

  /* Find the face corner ranges using the offsets array from the new mesh. That gives us the final
   * number of face corners. */
  const OffsetIndices faces = calc_faces(src_faces, copy_tris, mesh->face_offsets_for_write());
  mesh->totloop = faces.total_size();
  attributes.add<int>(".corner_vert", ATTR_DOMAIN_CORNER, bke::AttributeInitConstruct());
  attributes.add<int>(".corner_edge", ATTR_DOMAIN_CORNER, bke::AttributeInitConstruct());

  /* All unnaffected corners from unselected faces, shifted to the front of the mesh. */
  const IndexRange unselected_corners = faces[unselected_range];
  /* Corners from selected faces that are already triangulated. */
  const IndexRange copy_tri_corners = faces[copy_tris_range];
  /* Corners from triangulated Ngons. */
  const IndexRange ngon_tri_corners = faces[ngon_tris];
  /* Corners from triangulated quads. */
  const IndexRange quad_tri_corners = faces[quad_tris];
  /* All newly created corners (from Ngons and quads). */
  const IndexRange tri_corners = faces[tris];

  /* An index map that maps from newly created corners in `tri_corners` to original corner indices.
   * This is used to interpolate `corner_vert` indices and face corner attributes. If there are no
   * face corner attributes, theoretically the map could be skipped and corner vertex indices could
   * be interpolated immediately, but that isn't done for simplicity. */
  Array<int> corner_map(tri_corners.size());

  MutableSpan<int2> edges = mesh->edges_for_write();
  MutableSpan<int> corner_edges = mesh->corner_edges_for_write();

  if (!ngons.is_empty()) {
    Span<float3> src_face_normals;
    if (mesh->runtime->face_normals_cache.is_cached() || ngons.size() > src_faces.size() / 4) {
      /* If a significant number of Ngons are selected (> 25% of the faces), then use the
       * face normals cache, in case the cache is persistent (or already calculated). */
      src_face_normals = src_mesh.face_normals();
    }
    triangulate_ngons(positions,
                      src_faces,
                      src_corner_verts,
                      src_corner_edges,
                      src_face_normals,
                      ngons,
                      tris_by_ngon,
                      edges_by_ngon,
                      ngon_mode,
                      faces.slice(ngon_tris),
                      ngon_edges.start(),
                      unselected_range.size() + copy_tri_corners.size(),
                      corner_map.as_mutable_span().take_front(ngon_tri_corners.size()),
                      edges.slice(ngon_edges),
                      corner_edges);
  }

  if (!quads.is_empty()) {
    triangulate_quads(positions,
                      src_faces,
                      src_corner_verts,
                      src_corner_edges,
                      quads,
                      quad_mode,
                      quad_edges.start(),
                      corner_map.as_mutable_span().take_back(quad_tri_corners.size()),
                      edges.slice(quad_edges),
                      corner_edges.slice(quad_tri_corners));
  }

  /* Vertex attributes are totally unnaffected and can be shared with implicit sharing. */
  bke::copy_attributes(src_attributes, ATTR_DOMAIN_POINT, propagation_info, {}, attributes);
  CustomData_merge(&src_mesh.vert_data, &mesh->vert_data, CD_MASK_ORIGINDEX, mesh->totvert);

  /* Edges and corner edges from unselected faces and already-trianguated faces are
   * unaffected and are copied to the result mesh like other generic attributes. */
  edges.take_front(src_edges.size()).copy_from(src_edges);
  array_utils::gather_group_to_group(
      src_faces, faces.slice(unselected_range), unselected, src_corner_edges, corner_edges);
  array_utils::gather_group_to_group(
      src_faces, faces.slice(copy_tris_range), copy_tris, src_corner_edges, corner_edges);

  src_attributes.for_all([&](const bke::AttributeIDRef &id,
                             const bke::AttributeMetaData meta_data) {
    if (meta_data.data_type == CD_PROP_STRING) {
      return true;
    }
    if (meta_data.domain == ATTR_DOMAIN_POINT) {
      return true;
    }
    if (ELEM(id.name(), ".corner_edge", ".edge_verts")) {
      return true;
    }

    const GVArraySpan src = *src_attributes.lookup(id, meta_data.domain);
    bke::GSpanAttributeWriter dst = attributes.lookup_or_add_for_write_only_span(
        id, meta_data.domain, meta_data.data_type);

    switch (meta_data.domain) {
      case ATTR_DOMAIN_EDGE: {
        dst.span.take_front(src.size()).copy_from(src);
        GMutableSpan new_data = dst.span.take_back(inner_edges.size());
        /* Though it would be reasonable interpolate data from connected edges within each face,
         * currently the data from new edges is just set to the type's default value. */
        dst.span.type().default_construct_n(new_data.data(), new_data.size());
        break;
      }
      case ATTR_DOMAIN_FACE: {
        array_utils::gather(src, unselected, dst.span.slice(unselected_range));
        array_utils::gather(src, copy_tris, dst.span.slice(copy_tris_range));
        copy_to_quad_tris(src, quads, dst.span.slice(quad_tris));
        bke::attribute_math::gather_to_groups(src, ngons, tris_by_ngon, dst.span.slice(ngon_tris));
        break;
      }
      case ATTR_DOMAIN_CORNER: {
        bke::attribute_math::gather_group_to_group(src_faces,
                                                   faces.slice(unselected_range),
                                                   unselected,
                                                   src,
                                                   dst.span.slice(unselected_corners));
        bke::attribute_math::gather_group_to_group(src_faces,
                                                   faces.slice(copy_tris_range),
                                                   copy_tris,
                                                   src,
                                                   dst.span.slice(copy_tri_corners));
        bke::attribute_math::gather(src, corner_map.as_span(), dst.span.slice(tri_corners));
        break;
      }
      default:
        break;
    }

    dst.finish();
    return true;
  });

  if (const int *src = static_cast<const int *>(
          CustomData_get_layer(&src_mesh.edge_data, CD_ORIGINDEX)))
  {
    int *dst = static_cast<int *>(
        CustomData_add_layer(&mesh->edge_data, CD_ORIGINDEX, CD_CONSTRUCT, mesh->totedge));
    std::copy(src, src + src_mesh.totedge, dst);
    MutableSpan(dst, mesh->totedge).drop_front(src_mesh.totedge).fill(ORIGINDEX_NONE);
  }

  if (const int *src_data = static_cast<const int *>(
          CustomData_get_layer(&src_mesh.face_data, CD_ORIGINDEX)))
  {
    int *dst_data = static_cast<int *>(
        CustomData_add_layer(&mesh->face_data, CD_ORIGINDEX, CD_CONSTRUCT, mesh->faces_num));
    const Span src(src_data, src_mesh.faces_num);
    MutableSpan dst(dst_data, mesh->faces_num);
    array_utils::gather(src, unselected, dst.slice(unselected_range));
    array_utils::gather(src, copy_tris_range, dst.slice(copy_tris_range));
    copy_to_quad_tris(src, quads, dst.slice(quad_tris));
    bke::attribute_math::gather_to_groups(src, ngons, tris_by_ngon, dst.slice(ngon_tris));
  }

  mesh->runtime->bounds_cache = src_mesh.runtime->bounds_cache;
  copy_loose_vert_hint(src_mesh, *mesh);
  copy_loose_edge_hint(src_mesh, *mesh);
  return mesh;
}

}  // namespace blender::geometry
