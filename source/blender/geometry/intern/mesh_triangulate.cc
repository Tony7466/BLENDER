/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_array_utils.hh"
#include "BLI_index_mask.hh"
#include "BLI_math_geom.h"
#include "BLI_math_matrix.h"
#include "BLI_ordered_edge.hh"
#include "BLI_polyfill_2d.h"
#include "BLI_polyfill_2d_beautify.h"
#include "BLI_vector_set.hh"

// TODO: Remove
#include "BLI_heap.h"
#include "BLI_memarena.h"

#include "BKE_attribute.hh"
#include "BKE_attribute_math.hh"
#include "BKE_mesh.hh"
#include "BKE_mesh_mapping.hh"

#include "GEO_mesh_triangulate.hh"

namespace blender::geometry {

static OffsetIndices<int> calc_faces(const OffsetIndices<int> src_faces,
                                     const IndexMask &unselected,
                                     MutableSpan<int> face_offsets)
{
  /* Unselected faces are moved to the start of the mesh. */
  offset_indices::gather_selected_offsets(src_faces, unselected, face_offsets);

  /* All other faces are triangles added to the end of the mesh. */
  MutableSpan<int> new_tri_offsets = face_offsets.drop_front(unselected.size());
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
        directions[i] = distances_0_2[i] < distances_1_3[i] ? QuadDirection::Edge_0_2 :
                                                              QuadDirection::Edge_1_3;
      }
    }
  }
  // TODO: Beauty
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
                                   MutableSpan<int> corner_edges)
{
  for (const int i : quads.index_range()) {
    const Span<int> src = src_corner_edges.slice(src_faces[quads[i]]);
    MutableSpan<int> dst = corner_edges.slice(6 * i, 6);
    switch (directions[i]) {
      case QuadDirection::Edge_0_2:
        dst.copy_from({src[0], src[1], i, src[2], src[3], i});
        break;
      case QuadDirection::Edge_1_3:
        dst.copy_from({src[0], i, src[3], src[1], src[2], i});
        break;
    }
  }
}

static void triangulate_quads(const Span<float3> positions,
                              const OffsetIndices<int> src_faces,
                              const Span<int> src_corner_verts,
                              const Span<int> src_corner_edges,
                              const IndexMask &quads,
                              const OffsetIndices<int> tris_by_ngon,
                              const OffsetIndices<int> edges_by_ngon,
                              const TriangulateQuadMode quad_mode,
                              const OffsetIndices<int> all_faces,
                              MutableSpan<int> corner_map,
                              MutableSpan<int4> new_edge_neighbors,
                              MutableSpan<int2> edges,
                              MutableSpan<int> corner_edges)
{
  new_edge_neighbors.fill(int4(0));
  quads.foreach_segment(GrainSize(1024), [&](const IndexMaskSegment quads, const int64_t pos) {
    // TODO: TLS
    Array<QuadDirection, 1024> directions(quads.size());
    calc_quad_directions(positions, src_faces, src_corner_verts, quad_mode, quads, directions);
    calc_quad_corner_maps(src_faces, quads, directions, corner_map);
    calc_quad_edges(
        src_faces, src_corner_verts, quads, directions, edges.slice(pos, quads.size()));
    calc_quad_corner_edges(src_faces, src_corner_edges, quads, directions, corner_edges);
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

static VectorSet<OrderedEdge> calc_inner_triangles(const Span<int> face_verts,
                                                   const Span<int> face_edges,
                                                   const int edge_offset,
                                                   const Span<uint3> tris,
                                                   MutableSpan<int> corner_edges)
{
  VectorSet<OrderedEdge> inner_edges;
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
    const uint3 tri = tris[i];
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
                              const IndexMask &selection,
                              const OffsetIndices<int> tris_by_ngon,
                              const OffsetIndices<int> edges_by_ngon,
                              const TriangulateNGonMode ngon_mode,
                              const OffsetIndices<int> all_faces,
                              MutableSpan<int> corner_map,
                              MutableSpan<int4> new_edge_neighbors,
                              MutableSpan<int2> edges,
                              MutableSpan<int> corner_edges)
{
  Array<float2> projected_positions(144);
  const int new_tri_start = src_faces.size() - selection.size();
  new_edge_neighbors.fill(int4(0));
  selection.foreach_index(GrainSize(512), [&](const int face, const int mask) {
    const IndexRange src_corners = src_faces[face];

    const Span<int> src_face_verts = src_corner_verts.slice(src_corners);
    const Span<int> src_face_edges = src_corner_edges.slice(src_corners);

    const IndexRange tris = tris_by_ngon[mask];
    MutableSpan<int> corner_map = corner_map.slice(tris.start() * 3, tris.size() * 3);
    const OffsetIndices faces = all_faces.slice(tris.shift(new_tri_start));
    const IndexRange new_corners(faces.data().first(), faces.size() * 3);

    MutableSpan<int> face_edges = corner_edges.slice(new_corners);

    const IndexRange new_edge_range = edges_by_ngon[mask];

    const float3 normal = bke::mesh::face_normal_calc(positions, src_face_verts);
    float3x3 projection;
    axis_dominant_v3_to_m3_negate(projection.ptr(), normal);

    Array<float2, 64> positions_2d(src_face_verts.size());
    for (const int i : src_face_verts.index_range()) {
      mul_v2_m3v3(positions_2d[i], projection.ptr(), positions[src_face_verts[i]]);
    }

    MutableSpan<uint3> triangulation = corner_map.cast<uint3>();
    BLI_polyfill_calc(reinterpret_cast<const float(*)[2]>(positions_2d.data()),
                      positions_2d.size(),
                      1,
                      reinterpret_cast<uint(*)[3]>(triangulation.data()));

    if (ngon_mode == TriangulateNGonMode::Beauty) {
      MemArena *arena = BLI_memarena_new(BLI_POLYFILL_ARENA_SIZE, __func__);
      Heap *heap = BLI_heap_new_ex(BLI_POLYFILL_ALLOC_NGON_RESERVE);
      BLI_polyfill_beautify(reinterpret_cast<const float(*)[2]>(positions_2d.data()),
                            positions_2d.size(),
                            reinterpret_cast<uint(*)[3]>(triangulation.data()),
                            arena,
                            heap);
      BLI_memarena_clear(arena);
      BLI_heap_free(heap, nullptr);
    }

    const VectorSet<OrderedEdge> inner_edges = calc_inner_triangles(
        src_face_verts, src_face_edges, new_edge_range.start(), triangulation, face_edges);

    for (int &vert : corner_map) {
      vert += src_corners.start();
    }

    BLI_assert(inner_edges.size() == new_edge_range.size());
    edges.slice(edges_by_ngon[mask]).copy_from(inner_edges.as_span().cast<int2>());
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
static void mix_edge_data_from_neighbors(const Span<T> src,
                                         const Span<int4> indices,
                                         MutableSpan<T> dst)
{
  threading::parallel_for(dst.index_range(), 1024, [&](const IndexRange range) {
    for (const int i : range) {
      dst[i] = bke::attribute_math::mix4(float4(1.0f),
                                         src[indices[i][0]],
                                         src[indices[i][1]],
                                         src[indices[i][2]],
                                         src[indices[i][3]]);
    }
  });
}

std::optional<Mesh *> mesh_triangulate(
    const Mesh &src_mesh,
    const IndexMask &selection,
    const TriangulateNGonMode ngon_mode,
    const TriangulateQuadMode quad_mode,
    const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  const Span<int2> src_edges = src_mesh.edges();
  const OffsetIndices src_faces = src_mesh.faces();
  const Span<int> src_corner_verts = src_mesh.corner_verts();
  const Span<int> src_corner_edges = src_mesh.corner_edges();
  const bke::AttributeAccessor src_attributes = src_mesh.attributes();

  IndexMaskMemory memory;
  const IndexMask quads = IndexMask::from_predicate(
      selection, GrainSize(4096), memory, [&](const int face) {
        return src_faces[face].size() == 4;
      });
  const IndexMask ngons = IndexMask::from_predicate(
      selection, GrainSize(4096), memory, [&](const int face) {
        return src_faces[face].size() > 4;
      });
  if (quads.is_empty() && ngons.is_empty()) {
    /* All selected faces are already triangles. */
    return std::nullopt;
  }
  const IndexMask unselected = selection.complement(src_faces.index_range(), memory);

  Array<int> tri_offsets(ngons.size() + 1);
  const OffsetIndices tris_by_ngon = calc_tris_by_ngon(src_faces, ngons, tri_offsets);

  const IndexRange quad_tris(unselected.size(), quads.size() * 2);
  const IndexRange ngon_tris(quad_tris.one_after_last(), tris_by_ngon.total_size());

  Array<int> edge_offset_data(selection.size() + 1);
  const OffsetIndices edges_by_ngon = calc_edges_by_ngon(src_faces, selection, edge_offset_data);

  const IndexRange quad_edges(src_edges.size(), quads.size());
  const IndexRange ngon_edges(quad_edges.one_after_last(), edges_by_ngon.total_size());

  Mesh *mesh = create_mesh_no_attributes(src_mesh,
                                         src_mesh.totvert,
                                         src_edges.size() + quad_edges.size() + ngon_edges.size(),
                                         unselected.size() + quad_tris.size() + ngon_tris.size(),
                                         0);
  bke::MutableAttributeAccessor attributes = mesh->attributes_for_write();

  const OffsetIndices faces = calc_faces(src_faces, unselected, mesh->face_offsets_for_write());

  const IndexRange quad_tri_corners = faces[quad_tris];
  const IndexRange ngon_tri_corners = faces[ngon_tris];

  mesh->totloop = faces.total_size();
  attributes.add<int>(".corner_vert", ATTR_DOMAIN_CORNER, bke::AttributeInitConstruct());
  attributes.add<int>(".corner_edge", ATTR_DOMAIN_CORNER, bke::AttributeInitConstruct());
  attributes.add<int2>(".edge_verts", ATTR_DOMAIN_EDGE, bke::AttributeInitConstruct());

  Array<int> new_to_old_corner_map(quad_tri_corners.size() + ngon_tri_corners.size());

  // TODO: Only calculate edge neighbors if there are edge attributes to interpolate.
  Array<int4> new_edge_old_neighbors(edges_by_ngon.total_size());

  MutableSpan<int2> edges = mesh->edges_for_write();
  MutableSpan<int> corner_edges = mesh->corner_edges_for_write();

  triangulate_quads(src_mesh.vert_positions(),
                    src_faces,
                    src_corner_verts,
                    src_corner_edges,
                    selection,
                    tris_by_ngon,
                    edges_by_ngon,
                    quad_mode,
                    faces,
                    new_to_old_corner_map,
                    new_edge_old_neighbors,
                    edges,
                    corner_edges);
  triangulate_ngons(src_mesh.vert_positions(),
                    src_faces,
                    src_corner_verts,
                    src_corner_edges,
                    selection,
                    tris_by_ngon,
                    edges_by_ngon,
                    ngon_mode,
                    faces,
                    new_to_old_corner_map,
                    new_edge_old_neighbors,
                    edges,
                    corner_edges);

  edges.take_front(src_edges.size()).copy_from(src_edges);

  bke::copy_attributes(src_attributes, ATTR_DOMAIN_POINT, propagation_info, {}, attributes);

  src_attributes.for_all([&](const bke::AttributeIDRef &id,
                             const bke::AttributeMetaData meta_data) {
    if (meta_data.data_type == CD_PROP_STRING) {
      return true;
    }
    if (ELEM(id.name(), ".corner_edge", ".edge_verts")) {
      return true;
    }
    if (meta_data.domain == ATTR_DOMAIN_POINT) {
      return true;
    }

    const GVArraySpan src = *src_attributes.lookup(id, meta_data.domain);
    bke::GSpanAttributeWriter dst = attributes.lookup_or_add_for_write_only_span(
        id, meta_data.domain, meta_data.data_type);

    switch (meta_data.domain) {
      case ATTR_DOMAIN_EDGE: {
        dst.span.take_front(src.size()).copy_from(src);
        bke::attribute_math::convert_to_static_type(meta_data.data_type, [&](auto dummy) {
          using T = decltype(dummy);
          mix_edge_data_from_neighbors(
              src.typed<T>(),
              new_edge_old_neighbors,
              dst.span.typed<T>().take_back(new_edge_old_neighbors.size()));
        });
        break;
      }
      case ATTR_DOMAIN_FACE: {
        array_utils::gather(src, unselected, dst.span.take_front(unselected.size()));
        bke::attribute_math::gather_to_groups(
            src, selection, tris_by_ngon, dst.span.take_back(tris_by_ngon.total_size()));
        break;
      }
      case ATTR_DOMAIN_CORNER: {
        bke::attribute_math::gather_group_to_group(src_faces, faces, unselected, src, dst.span);
        bke::attribute_math::gather(
            src, new_to_old_corner_map.as_span(), dst.span.take_back(new_tris_num * 3));
        break;
      }
      default:
        break;
    }

    dst.finish();
    return true;
  });

  CustomData_merge(&src_mesh.vert_data, &mesh->vert_data, CD_MASK_ORIGINDEX, mesh->totedge);

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
    array_utils::gather(src, unselected, dst.take_front(unselected.size()));
    bke::attribute_math::gather_to_groups(
        src, selection, tris_by_ngon, dst.take_back(tris_by_ngon.total_size()));
  }

  mesh->runtime->bounds_cache = src_mesh.runtime->bounds_cache;
  copy_loose_vert_hint(src_mesh, *mesh);
  copy_loose_edge_hint(src_mesh, *mesh);
  return mesh;
}

}  // namespace blender::geometry
