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

static OffsetIndices<int> calc_tri_groups(const OffsetIndices<int> src_faces,
                                          const IndexMask &selection,
                                          MutableSpan<int> face_offset_data)
{
  selection.foreach_index(GrainSize(2048), [&](const int face, const int mask) {
    face_offset_data[mask] = bke::mesh::face_triangles_num(src_faces[face].size());
  });
  return offset_indices::accumulate_counts_to_offsets(face_offset_data);
}

static OffsetIndices<int> calc_edge_groups(const OffsetIndices<int> src_faces,
                                           const IndexMask &selection,
                                           const int src_edges_num,
                                           MutableSpan<int> edge_offset_data)
{
  selection.foreach_index(GrainSize(2048), [&](const int face, const int mask) {
    /* The number of new inner edges for each face is the number of corners - 3. */
    edge_offset_data[mask] = src_faces[face].size() - 3;
  });
  return offset_indices::accumulate_counts_to_offsets(edge_offset_data, src_edges_num);
}

static OffsetIndices<int> calc_new_face_offsets(const OffsetIndices<int> src_faces,
                                                const IndexMask &selection_inverse,
                                                MutableSpan<int> face_offsets)
{
  /* Unselected faces are moved to the start of the mesh. */
  offset_indices::gather_selected_offsets(src_faces, selection_inverse, face_offsets);

  /* All new faces are triangles, and are added to the end of the mesh. */
  MutableSpan<int> new_tri_offsets = face_offsets.drop_front(selection_inverse.size());
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
enum class QuadDirection {
  Edge_0_2,
  Edge_1_3,
};

static QuadDirection calc_quad_direction(const Span<float3> positions,
                                         const Span<int> face_verts,
                                         const TriangulateQuadMode quad_mode)
{
  switch (quad_mode) {
    case TriangulateQuadMode::Beauty:
      break;
    case TriangulateQuadMode::Fixed:
      return QuadDirection::Edge_0_2;
    case TriangulateQuadMode::Alternate:
      return QuadDirection::Edge_1_3;
    case TriangulateQuadMode::ShortEdge: {
      const float distance_0_2 = math::distance_squared(positions[face_verts[0]],
                                                        positions[face_verts[2]]);
      const float distance_1_3 = math::distance_squared(positions[face_verts[1]],
                                                        positions[face_verts[3]]);
      return distance_0_2 < distance_1_3 ? QuadDirection::Edge_0_2 : QuadDirection::Edge_1_3;
    }
    case TriangulateQuadMode::LongEdge: {
      const float distance_0_2 = math::distance_squared(positions[face_verts[0]],
                                                        positions[face_verts[2]]);
      const float distance_1_3 = math::distance_squared(positions[face_verts[1]],
                                                        positions[face_verts[3]]);
      return distance_0_2 < distance_1_3 ? QuadDirection::Edge_1_3 : QuadDirection::Edge_0_2;
    }
  }
  BLI_assert_unreachable();
  return QuadDirection::Edge_0_2;
}

static void triangulate_quad_edges(const Span<int> face_verts,
                                   const Span<int> face_edges,
                                   const QuadDirection direction,
                                   const int new_edge_index,
                                   MutableSpan<int2> edges,
                                   MutableSpan<int> corner_edges)
{
  switch (direction) {
    case QuadDirection::Edge_0_2:
      edges[new_edge_index] = int2(face_verts[0], face_verts[2]);
      corner_edges.copy_from({face_edges[0],
                              face_edges[1],
                              new_edge_index,
                              face_edges[2],
                              face_edges[3],
                              new_edge_index});
      break;
    case QuadDirection::Edge_1_3:
      edges[new_edge_index] = int2(face_verts[1], face_verts[3]);
      corner_edges.copy_from({face_edges[0],
                              new_edge_index,
                              face_edges[3],
                              face_edges[1],
                              face_edges[2],
                              new_edge_index});
      break;
  }
}

static void calc_quad_corner_map(const QuadDirection direction,
                                 const int start,
                                 MutableSpan<int> new_to_old_corner_map)
{
  switch (direction) {
    case QuadDirection::Edge_0_2:
      new_to_old_corner_map.copy_from({0, 1, 2, 2, 3, 0});
      break;
    case QuadDirection::Edge_1_3:
      new_to_old_corner_map.copy_from({0, 1, 3, 1, 2, 3});
      break;
  }
  for (int &i : new_to_old_corner_map) {
    i += start;
  }
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

static void triangulate_faces(const Span<float3> positions,
                              const OffsetIndices<int> src_faces,
                              const Span<int> src_corner_verts,
                              const Span<int> src_corner_edges,
                              const IndexMask &selection,
                              const OffsetIndices<int> tri_groups,
                              const OffsetIndices<int> edge_groups,
                              const TriangulateNGonMode ngon_mode,
                              const TriangulateQuadMode quad_mode,
                              const OffsetIndices<int> all_faces,
                              MutableSpan<int> new_to_old_corner_map,
                              MutableSpan<int4> new_edge_neighbors,
                              MutableSpan<int2> edges,
                              MutableSpan<int> corner_edges)
{
  const int new_tri_start = src_faces.size() - selection.size();
  new_edge_neighbors.fill(int4(0));
  selection.foreach_index(GrainSize(512), [&](const int face, const int mask) {
    const IndexRange src_corners = src_faces[face];
    if (src_corners.size() == 3) {
      return;
    }

    const Span<int> src_face_verts = src_corner_verts.slice(src_corners);
    const Span<int> src_face_edges = src_corner_edges.slice(src_corners);

    const IndexRange new_tri_group = tri_groups[mask];
    const OffsetIndices faces = all_faces.slice(new_tri_group.shift(new_tri_start));
    const IndexRange new_corners(faces.data().first(), faces.size() * 3);

    MutableSpan<int> face_edges = corner_edges.slice(new_corners);

    const IndexRange new_edge_range = edge_groups[mask];

    if (src_corners.size() == 4) {
      const QuadDirection direction = calc_quad_direction(positions, src_face_verts, quad_mode);
      calc_quad_corner_map(
          direction, src_corners.start(), new_to_old_corner_map.slice(new_corners));
      triangulate_quad_edges(
          src_face_verts, src_face_edges, direction, new_edge_range.start(), edges, face_edges);
    }
    else {
      const float3 normal = bke::mesh::face_normal_calc(positions, src_face_verts);
      float3x3 projection;
      axis_dominant_v3_to_m3_negate(projection.ptr(), normal);

      Array<float2, 64> positions_2d(src_face_verts.size());
      for (const int i : src_face_verts.index_range()) {
        mul_v2_m3v3(positions_2d[i], projection.ptr(), positions[src_face_verts[i]]);
      }

      MutableSpan<uint3> triangulation = new_to_old_corner_map.slice(new_corners).cast<uint3>();
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

      BLI_assert(inner_edges.size() == new_edge_range.size());
      edges.slice(edge_groups[mask]).copy_from(inner_edges.as_span().cast<int2>());
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
  mesh->totvert = verts_num;
  mesh->totedge = edges_num;
  mesh->totloop = corners_num;
  CustomData_free_layer_named(&mesh->vert_data, "position", 0);
  CustomData_free_layer_named(&mesh->edge_data, ".edge_verts", 0);
  CustomData_free_layer_named(&mesh->loop_data, ".corner_vert", 0);
  CustomData_free_layer_named(&mesh->loop_data, ".corner_edge", 0);
  BKE_mesh_copy_parameters_for_eval(mesh, &params_mesh);
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

  Array<int> tri_offsets(selection.size() + 1);
  const OffsetIndices tri_groups = calc_tri_groups(src_faces, selection, tri_offsets);
  const int new_tris_num = tri_groups.total_size();
  if (new_tris_num == 0) {
    /* All selected faces are already triangles. */
    return std::nullopt;
  }

  Array<int> edge_offset_data(selection.size() + 1);
  const OffsetIndices edge_groups = calc_edge_groups(
      src_faces, selection, src_mesh.totedge, edge_offset_data);

  IndexMaskMemory memory;
  const IndexMask selection_inverse = selection.complement(src_faces.index_range(), memory);

  Mesh *mesh = create_mesh_no_attributes(src_mesh,
                                         src_mesh.totvert,
                                         edge_groups.total_size(),
                                         src_faces.size() + new_tris_num - selection.size(),
                                         0);
  bke::MutableAttributeAccessor attributes = mesh->attributes_for_write();

  const OffsetIndices faces = calc_new_face_offsets(
      src_faces, selection_inverse, mesh->face_offsets_for_write());

  mesh->totloop = faces.total_size();
  attributes.add<int>(".corner_vert", ATTR_DOMAIN_CORNER, bke::AttributeInitConstruct());
  attributes.add<int>(".corner_edge", ATTR_DOMAIN_CORNER, bke::AttributeInitConstruct());
  attributes.add<int2>(".edge_verts", ATTR_DOMAIN_EDGE, bke::AttributeInitConstruct());

  Array<int> new_to_old_corner_map(new_tris_num * 3);

  MutableSpan<int2> edges = mesh->edges_for_write();
  edges.take_front(src_edges.size()).copy_from(src_edges);

  MutableSpan<int> corner_edges = mesh->corner_edges_for_write();

  // TODO: Only calculate edge neighbors if there are edge attributes to interpolate.
  Array<int4> new_edge_old_neighbors(edge_groups.total_size());

  triangulate_faces(src_mesh.vert_positions(),
                    src_faces,
                    src_corner_verts,
                    src_corner_edges,
                    selection,
                    tri_groups,
                    edge_groups,
                    ngon_mode,
                    quad_mode,
                    faces,
                    new_to_old_corner_map,
                    new_edge_old_neighbors,
                    edges,
                    corner_edges);

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
        array_utils::gather(src, selection_inverse, dst.span.take_front(selection_inverse.size()));
        bke::attribute_math::gather_to_groups(
            src, selection, tri_groups, dst.span.take_back(tri_groups.total_size()));
        break;
      }
      case ATTR_DOMAIN_CORNER: {
        bke::attribute_math::gather_group_to_group(
            src_faces, faces, selection_inverse, src, dst.span);
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

  if (int *orig_indices = static_cast<int *>(
          CustomData_get_layer_for_write(&mesh->edge_data, CD_ORIGINDEX, mesh->totedge)))
  {
    MutableSpan(orig_indices, mesh->totedge).drop_front(src_edges.size()).fill(ORIGINDEX_NONE);
  }

  mesh->runtime->bounds_cache = src_mesh.runtime->bounds_cache;
  copy_loose_vert_hint(src_mesh, *mesh);
  copy_loose_edge_hint(src_mesh, *mesh);
  return mesh;
}

}  // namespace blender::geometry
