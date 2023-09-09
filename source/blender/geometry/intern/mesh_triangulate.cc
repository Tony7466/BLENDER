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

//   IndexMaskMemory memory;
// const IndexMask quads = IndexMask::from_predicate(
//     selection, GrainSize(4096), memory, [&](const int face) { return faces[face].size() == 3;
//     });
// const IndexMask ngons = IndexMask::from_predicate(
//     selection, GrainSize(4096), memory, [&](const int face) { return faces[face].size() > 4; });

static OffsetIndices<int> calc_new_face_groups(const OffsetIndices<int> orig_faces,
                                               const IndexMask &selection,
                                               MutableSpan<int> face_offset_data)
{
  selection.foreach_index(GrainSize(2048), [&](const int face, const int mask) {
    /* Reuse the original face for the first triangle. */
    face_offset_data[mask] = bke::mesh::face_triangles_num(orig_faces[face].size()) - 1;
  });
  return offset_indices::accumulate_counts_to_offsets(face_offset_data);
}

static OffsetIndices<int> create_new_edge_offsets(const OffsetIndices<int> orig_faces,
                                                  const IndexMask &selection,
                                                  MutableSpan<int> edge_offset_data)
{
  selection.foreach_index(GrainSize(2048), [&](const int face, const int mask) {
    /* The number of new edges for each face is the number of corners - 3. */
    edge_offset_data[mask] = orig_faces[face].size() - 3;
  });
  return offset_indices::accumulate_counts_to_offsets(edge_offset_data);
}

static OffsetIndices<int> create_new_faces(const OffsetIndices<int> orig_faces,
                                           const IndexMask &selection,
                                           const IndexMask &selection_inverse,
                                           const int new_faces_num,
                                           MutableSpan<int> new_faces_data)
{
  if (selection.size() == orig_faces.size()) {
    offset_indices::fill_constant_group_size(3, 0, new_faces_data);
    return OffsetIndices<int>(new_faces_data);
  }

  /* The reused faces are triangles. */
  index_mask::masked_fill(new_faces_data, 3, selection);
  offset_indices::copy_group_sizes(orig_faces, selection_inverse, new_faces_data);
  offset_indices::accumulate_counts_to_offsets(new_faces_data.take_front(orig_faces.size()));

  /* All new faces are triangles.*/
  MutableSpan<int> new_faces = new_faces_data.take_back(new_faces_num);
  offset_indices::fill_constant_group_size(3, new_faces.first(), new_faces);
  return OffsetIndices<int>(new_faces_data);
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

static void triangulate_quad(const Span<float3> positions,
                             const Span<int> face_verts,
                             const Span<int> face_edges,
                             const TriangulateQuadMode quad_mode,
                             const int new_edge_index,
                             MutableSpan<int2> new_edges,
                             int3 &r_tri_verts_a,
                             int3 &r_tri_verts_b,
                             int3 &r_tri_edges_a,
                             int3 &r_tri_edges_b)
{
  const QuadDirection direction = calc_quad_direction(positions, face_verts, quad_mode);
  switch (direction) {
    case QuadDirection::Edge_0_2:
      r_tri_verts_a = int3(face_verts[0], face_verts[1], face_verts[2]);
      r_tri_verts_b = int3(face_verts[2], face_verts[3], face_verts[0]);

      new_edges[new_edge_index] = int2(face_verts[0], face_verts[2]);
      r_tri_edges_a = int3(face_edges[0], face_edges[1], new_edge_index);
      r_tri_edges_b = int3(face_edges[2], face_edges[3], new_edge_index);
      break;
    case QuadDirection::Edge_1_3:
      r_tri_verts_a = int3(face_verts[0], face_verts[1], face_verts[3]);
      r_tri_verts_b = int3(face_verts[1], face_verts[2], face_verts[3]);

      new_edges[new_edge_index] = int2(face_verts[1], face_verts[3]);
      r_tri_edges_a = int3(face_edges[0], new_edge_index, face_edges[3]);
      r_tri_edges_b = int3(face_edges[1], face_edges[2], new_edge_index);
      break;
  }
}

static VectorSet<OrderedEdge> calc_inner_triangles(const Span<int> face_verts,
                                                   const Span<int> face_edges,
                                                   const int edge_offset,
                                                   const Span<uint3> tris,
                                                   MutableSpan<int> corner_edges_first,
                                                   MutableSpan<int> corner_edges)
{
  VectorSet<OrderedEdge> inner_edges;
  auto add_edge = [&](const int corner_1, const int corner_2) -> int {
    if (math::distance(corner_1, corner_2) == 1) {
      return face_edges[math::min(corner_1, corner_2)];
    }
    const OrderedEdge edge(face_verts[corner_1], face_verts[corner_2]);
    return edge_offset + inner_edges.index_of_or_add(edge);
  };

  const uint3 tri_first = tris.first();
  corner_edges_first[0] = add_edge(tri_first[0], tri_first[1]);
  corner_edges_first[1] = add_edge(tri_first[1], tri_first[2]);
  corner_edges_first[2] = add_edge(tri_first[2], tri_first[0]);

  for (const int i : tris.index_range().drop_front(1)) {
    const uint3 tri = tris[i];
    corner_edges[3 * i + 0] = add_edge(tri[0], tri[1]);
    corner_edges[3 * i + 1] = add_edge(tri[1], tri[2]);
    corner_edges[3 * i + 2] = add_edge(tri[2], tri[0]);
  }

  return inner_edges;
}

static void triangulate_faces(const Span<float3> positions,
                              const Span<int2> orig_edges,
                              const OffsetIndices<int> orig_faces,
                              const Span<int> orig_corner_verts,
                              const Span<int> orig_corner_edges,
                              const IndexMask &selection,
                              const OffsetIndices<int> new_face_groups,
                              const OffsetIndices<int> new_edge_groups,
                              const TriangulateNGonMode ngon_mode,
                              const TriangulateQuadMode quad_mode,
                              const OffsetIndices<int> faces,
                              MutableSpan<int> corner_orig_indices,
                              MutableSpan<int2> edges,
                              MutableSpan<int> corner_verts,
                              MutableSpan<int> corner_edges)
{
  selection.foreach_index(GrainSize(512), [&](const int face, const int mask) {
    const IndexRange orig_corners = orig_faces[face];
    const int triangles_num = bke::mesh::face_triangles_num(orig_corners.size());
    if (triangles_num == 1) {
      return;
    }

    const Span<int> face_verts = orig_corner_verts.slice(orig_corners);
    const Span<int> face_edges = orig_corner_edges.slice(orig_corners);

    const IndexRange new_face_group = new_face_groups[mask];
    const IndexRange reused_face = faces[face];
    const OffsetIndices new_faces = faces.slice(new_face_group.shift(orig_faces.size()));
    const IndexRange new_corners(new_faces.data().first(), new_faces.size() * 3);

    MutableSpan<int> reused_corner_verts = corner_verts.slice(reused_face);
    MutableSpan<int> new_corner_verts = corner_verts.slice(new_corners);

    MutableSpan<int> reused_corner_edges = corner_edges.slice(reused_face);
    MutableSpan<int> new_corner_edges = corner_edges.slice(new_corners);

    const IndexRange new_edge_range = new_edge_groups[mask];

    if (orig_corners.size() == 4) {
      triangulate_quad(positions,
                       face_verts,
                       face_edges,
                       quad_mode,
                       new_edge_range.first(),
                       edges,
                       reused_corner_verts.cast<int3>().first(),
                       new_corner_verts.cast<int3>().first(),
                       reused_corner_edges.cast<int3>().first(),
                       new_corner_edges.cast<int3>().first());
    }
    else {
      const float3 normal = bke::mesh::face_normal_calc(positions, face_verts);
      float3x3 projection;
      axis_dominant_v3_to_m3_negate(projection.ptr(), normal);

      Array<float2, 64> positions_2d(face_verts.size());
      for (const int i : face_verts.index_range()) {
        mul_v2_m3v3(positions_2d[i], projection.ptr(), positions[face_verts[i]]);
      }

      MutableSpan<uint3> triangulation = corner_orig_indices.slice(new_corners).cast<uint3>();
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

      const VectorSet<OrderedEdge> inner_edges = calc_inner_triangles(face_verts,
                                                                      face_edges,
                                                                      new_edge_range.start() +
                                                                          orig_edges.size(),
                                                                      triangulation,
                                                                      reused_corner_edges,
                                                                      new_corner_edges);

      BLI_assert(inner_edges.size() == new_edge_range.size());
      edges.slice(new_edge_groups[mask]).copy_from(inner_edges.as_span().cast<int2>());

      const uint3 first_tri = triangulation.first();
      corner_verts[reused_face[0]] = face_verts[first_tri[0]];
      corner_verts[reused_face[1]] = face_verts[first_tri[1]];
      corner_verts[reused_face[2]] = face_verts[first_tri[2]];

      for (const int i : triangulation.index_range().drop_front(1)) {
        const uint3 tri = triangulation[i];
        const IndexRange new_face = new_faces[i];
        corner_verts[new_face[0]] = face_verts[tri[0]];
        corner_verts[new_face[1]] = face_verts[tri[1]];
        corner_verts[new_face[2]] = face_verts[tri[2]];
      }
    }
  });
}

static void remove_non_propagated_attributes(
    bke::MutableAttributeAccessor attributes,
    const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  if (propagation_info.propagate_all) {
    return;
  }
  Set<bke::AttributeIDRef> ids_to_remove = attributes.all_ids();
  ids_to_remove.remove_if([&](const bke::AttributeIDRef &id) {
    if (!id.is_anonymous()) {
      return true;
    }
    if (propagation_info.propagate(id.anonymous_id())) {
      return true;
    }
    return false;
  });
  for (const bke::AttributeIDRef &id : ids_to_remove) {
    attributes.remove(id);
  }
}

static void resize_mesh(Mesh &mesh,
                        const int edges_num,
                        const int faces_num,
                        const int corners_num)
{
  /* Remove types that aren't supported for interpolation in this node. */
  if (mesh.totedge != edges_num) {
    CustomData_free_layers(&mesh.edge_data, CD_FREESTYLE_EDGE, mesh.totedge);
    const int old_edges_num = mesh.totedge;
    mesh.totedge = edges_num;
    CustomData_realloc(&mesh.edge_data, old_edges_num, mesh.totedge);
  }
  if (mesh.faces_num != faces_num) {
    CustomData_free_layers(&mesh.face_data, CD_FREESTYLE_FACE, mesh.faces_num);
    const int old_faces_num = mesh.faces_num;
    mesh.faces_num = faces_num;
    CustomData_realloc(&mesh.face_data, old_faces_num, mesh.faces_num);
    implicit_sharing::resize_trivial_array(&mesh.face_offset_indices,
                                           &mesh.runtime->face_offsets_sharing_info,
                                           old_faces_num == 0 ? 0 : (old_faces_num + 1),
                                           mesh.faces_num + 1);
    /* Set common values for convenience. */
    mesh.face_offset_indices[0] = 0;
    mesh.face_offset_indices[mesh.faces_num] = corners_num;
  }
  if (mesh.totloop != corners_num) {
    CustomData_free_layers(&mesh.loop_data, CD_NORMAL, mesh.totloop);
    CustomData_free_layers(&mesh.loop_data, CD_MDISPS, mesh.totloop);
    CustomData_free_layers(&mesh.loop_data, CD_TANGENT, mesh.totloop);
    CustomData_free_layers(&mesh.loop_data, CD_PAINT_MASK, mesh.totloop);
    CustomData_free_layers(&mesh.loop_data, CD_MLOOPTANGENT, mesh.totloop);
    CustomData_free_layers(&mesh.loop_data, CD_GRID_PAINT_MASK, mesh.totloop);
    CustomData_free_layers(&mesh.loop_data, CD_CUSTOMLOOPNORMAL, mesh.totloop);
    const int old_loops_num = mesh.totloop;
    mesh.totloop = corners_num;
    CustomData_realloc(&mesh.loop_data, old_loops_num, mesh.totloop);
  }
}

void triangulate(Mesh &mesh,
                 const IndexMask &selection,
                 const TriangulateNGonMode ngon_mode,
                 const TriangulateQuadMode quad_mode,
                 const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  const Span<int2> orig_edges = mesh.edges();
  const OffsetIndices orig_faces = mesh.faces();
  const Span<int> orig_corner_verts = mesh.corner_verts();
  const Span<int> orig_corner_edges = mesh.corner_edges();

  Array<int> face_offsets(selection.size() + 1);
  const OffsetIndices new_face_groups = calc_new_face_groups(orig_faces, selection, face_offsets);
  const int new_faces_num = new_face_groups.total_size();
  if (new_faces_num == 0) {
    /* All selected faces are already triangles. */
    return;
  }

  Array<int> edge_offset_data(selection.size() + 1);
  const OffsetIndices new_edge_groups = create_new_edge_offsets(
      orig_faces, selection, edge_offset_data);

  IndexMaskMemory memory;
  const IndexMask selection_inverse = selection.complement(orig_faces.index_range(), memory);

  Array<int> new_faces_data(orig_faces.size() + new_faces_num + 1);
  const OffsetIndices new_faces = create_new_faces(
      orig_faces, selection, selection_inverse, new_faces_num, new_faces_data);

  Array<int2> edges(new_edge_groups.total_size());
  Array<int> corner_verts(new_faces.total_size());
  Array<int> corner_edges(new_faces.total_size());

  array_utils::gather_group_to_group(
      orig_faces, new_faces, selection_inverse, orig_corner_verts, corner_verts.as_mutable_span());
  array_utils::gather_group_to_group(
      orig_faces, new_faces, selection_inverse, orig_corner_edges, corner_edges.as_mutable_span());

  triangulate_faces(mesh.vert_positions(),
                    orig_edges,
                    orig_faces,
                    orig_corner_verts,
                    orig_corner_edges,
                    selection,
                    new_face_groups,
                    new_edge_groups,
                    ngon_mode,
                    quad_mode,
                    new_faces,
                    edges,
                    corner_verts,
                    corner_edges);

  remove_non_propagated_attributes(mesh.attributes_for_write(), propagation_info);
  resize_mesh(mesh, orig_edges.size() + edges.size(), new_faces.size(), new_faces.total_size());

  mesh.edges_for_write().take_back(edges.size()).copy_from(edges);
  mesh.face_offsets_for_write().copy_from(new_faces.data());
  mesh.corner_verts_for_write().copy_from(corner_verts);
  mesh.corner_edges_for_write().copy_from(corner_edges);

  // TODO: Keep bounds the same
  BKE_mesh_tag_topology_changed(&mesh);
}

}  // namespace blender::geometry
