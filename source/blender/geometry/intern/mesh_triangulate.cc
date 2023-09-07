/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_array_utils.hh"
#include "BLI_index_mask.hh"
#include "BLI_math_geom.h"
#include "BLI_math_matrix.h"
#include "BLI_polyfill_2d.h"
#include "BLI_polyfill_2d_beautify.h"

// TODO: Remove
#include "BLI_heap.h"
#include "BLI_memarena.h"

#include "BKE_attribute.hh"
#include "BKE_attribute_math.hh"
#include "BKE_mesh.hh"
#include "BKE_mesh_mapping.hh"

#include "GEO_mesh_triangulate.hh"

namespace blender::geometry {

static OffsetIndices<int> create_face_offsets(const OffsetIndices<int> orig_faces,
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

static OffsetIndices<int> create_new_corner_offsets(const OffsetIndices<int> orig_faces,
                                                    const IndexMask &selection,
                                                    const IndexMask &selection_inverse,
                                                    const int new_faces_num,
                                                    MutableSpan<int> corner_offset_data)
{
  if (selection.size() == orig_faces.size()) {
    offset_indices::fill_constant_group_size(3, 0, corner_offset_data);
    return OffsetIndices<int>(corner_offset_data);
  }

  /* The reused faces are triangles. */
  index_mask::masked_fill(corner_offset_data, 3, selection);
  offset_indices::copy_group_sizes(orig_faces, selection_inverse, corner_offset_data);
  offset_indices::accumulate_counts_to_offsets(corner_offset_data.take_front(orig_faces.size()));

  /* All new faces are triangles.*/
  MutableSpan<int> new_faces = corner_offset_data.take_back(new_faces_num);
  offset_indices::fill_constant_group_size(3, new_faces.first(), new_faces);
  return OffsetIndices<int>(corner_offset_data);
}

enum class QuadOption {
  /**
   * 3 ----- 2
   * | 1   / |
   * |   /   |
   * | /  0  |
   * 0 ----- 1
   */
  Edge_0_2,
  /**
   * 3 ----- 2
   * | \   1 |
   * |   \   |
   * |  0  \ |
   * 0 ----- 1
   */
  Edge_1_3,
};

struct QuadTriangulation {
  std::array<int3, 2> corners;
  int2 new_edge;
};

static void triangulate_quad_0_2(const Span<int> face_verts,
                                 const Span<int> face_edges,
                                 const int new_edge_index,
                                 MutableSpan<int2> new_edges,
                                 MutableSpan<int3> r_corner_verts,
                                 MutableSpan<int3> r_corner_edges)
{
  new_edges[new_edge_index] = int2(face_verts[0], face_verts[2]);
  r_corner_verts.copy_from({int3(face_verts[0], face_verts[1], face_verts[2]),
                            int3(face_verts[2], face_verts[3], face_verts[0])});
  r_corner_edges.copy_from({int3(face_edges[0], face_edges[1], new_edge_index),
                            int3(face_edges[2], face_edges[3], new_edge_index)});
}

static void triangulate_quad_1_3(const Span<int> face_verts,
                                 const Span<int> face_edges,
                                 const int new_edge_index,
                                 MutableSpan<int2> new_edges,
                                 MutableSpan<int3> r_corner_verts,
                                 MutableSpan<int3> r_corner_edges)
{
  new_edges[new_edge_index] = int2(face_verts[1], face_verts[3]);
  r_corner_verts.copy_from({int3(face_verts[0], face_verts[1], face_verts[3]),
                            int3(face_verts[1], face_verts[2], face_verts[3])});
  r_corner_edges.copy_from({int3(face_edges[0], new_edge_index, face_edges[3]),
                            int3(face_edges[1], face_edges[2], new_edge_index)});
}

static void triangulate_quad(const Span<float3> positions,
                             const Span<int> face_verts,
                             const Span<int> face_edges,
                             const TriangulateQuadMode quad_mode,
                             const int new_edge_index,
                             MutableSpan<int2> new_edges,
                             MutableSpan<int3> corner_verts,
                             MutableSpan<int3> corner_edges)
{
  switch (quad_mode) {
    case TriangulateQuadMode::Beauty:
      break;
    case TriangulateQuadMode::Fixed:
      triangulate_quad_0_2(
          face_verts, face_edges, new_edge_index, new_edges, corner_verts, corner_edges);
      break;
    case TriangulateQuadMode::Alternate:
      triangulate_quad_1_3(
          face_verts, face_edges, new_edge_index, new_edges, corner_verts, corner_edges);
      break;
    case TriangulateQuadMode::ShortEdge:
      const float distance_0_2 = math::distance_squared(positions[face_verts[0]],
                                                        positions[face_verts[2]]);
      const float distance_1_3 = math::distance_squared(positions[face_verts[1]],
                                                        positions[face_verts[3]]);
      if (distance_0_2 < distance_1_3) {
        triangulate_quad_0_2(
            face_verts, face_edges, new_edge_index, new_edges, corner_verts, corner_edges);
      }
      else {
        triangulate_quad_1_3(
            face_verts, face_edges, new_edge_index, new_edges, corner_verts, corner_edges);
      }
      break;
    case TriangulateQuadMode::LongEdge:
      const float distance_0_2 = math::distance_squared(positions[face_verts[0]],
                                                        positions[face_verts[2]]);
      const float distance_1_3 = math::distance_squared(positions[face_verts[1]],
                                                        positions[face_verts[3]]);
      if (distance_0_2 < distance_1_3) {
        triangulate_quad_1_3(
            face_verts, face_edges, new_edge_index, new_edges, corner_verts, corner_edges);
      }
      else {
        triangulate_quad_0_2(
            face_verts, face_edges, new_edge_index, new_edges, corner_verts, corner_edges);
      }
      break;
  }
  BLI_assert_unreachable();
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
                              const OffsetIndices<int> new_faces,
                              MutableSpan<int2> edges,
                              MutableSpan<int> corner_verts,
                              MutableSpan<int> corner_edges)
{
  // TODO: Split to a separate loop for each face type (triangle, quad, ngon).
  selection.foreach_index(GrainSize(512), [&](const int face, const int mask) {
    const IndexRange orig_corners = orig_faces[face];
    if (orig_corners.size() == 3) {
      return;
    }

    const IndexRange new_faces = new_face_groups[mask].shift(orig_faces.size());

    const Span<int> face_verts = orig_corner_verts.slice(orig_corners);
    const Span<int> face_edges = orig_corner_edges.slice(orig_corners);

    MutableSpan<int2> new_edges = edges.slice(new_edge_groups[mask].shift(orig_edges.size()));

    if (orig_corners.size() == 4) {
      triangulate_quad(positions,
                       face_verts,
                       face_edges,
                       quad_mode,
                       new_edge_groups[mask].shift(orig_edges.size()).start(),
                       edges,
                       corner_verts,
                       corner_edges);
    }
    else {
      const float3 normal = bke::mesh::face_normal_calc(positions, face_verts);
      float3x3 projection;
      axis_dominant_v3_to_m3_negate(projection.ptr(), normal);

      Array<float2, 64> positions_2d(face_verts.size());
      for (const int i : face_verts.index_range()) {
        mul_v2_m3v3(positions_2d[i], projection.ptr(), positions[face_verts[i]]);
      }

      Array<uint3, 64> triangulation(new_faces.size());
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
    }
  });
}

void triangulate(Mesh &mesh,
                 const IndexMask &selection,
                 const TriangulateNGonMode ngon_mode,
                 const TriangulateQuadMode quad_mode,
                 const bke::AnonymousAttributePropagationInfo & /*propagation_info*/)
{
  const Span<int2> orig_edges = mesh.edges();
  const OffsetIndices orig_faces = mesh.faces();
  const Span<int> orig_corner_verts = mesh.corner_verts();
  const Span<int> orig_corner_edges = mesh.corner_edges();

  Array<int> face_offsets(selection.size());
  const OffsetIndices new_face_groups = create_face_offsets(orig_faces, selection, face_offsets);
  const int new_faces_num = new_face_groups.total_size();
  if (new_faces_num == 0) {
    /* All selected faces are already triangles. */
    return;
  }

  Array<int> edge_offset_data(selection.size());
  const OffsetIndices new_edge_groups = create_new_edge_offsets(
      orig_faces, selection, edge_offset_data);

  IndexMaskMemory memory;
  const IndexMask selection_inverse = selection.complement(orig_faces.index_range(), memory);

  Array<int> corner_offset_data(orig_faces.size() + new_faces_num);
  const OffsetIndices new_faces = create_new_corner_offsets(
      orig_faces, selection, selection_inverse, new_faces_num, corner_offset_data);

  Array<int2> edges(new_edge_groups.total_size());
  Array<int> corner_verts(new_faces.total_size());
  Array<int> corner_edges(new_faces.total_size());

  array_utils::gather_group_to_group(
      orig_faces, new_faces, selection_inverse, orig_corner_verts, corner_verts.as_mutable_span());
  array_utils::gather_group_to_group(
      orig_faces, new_faces, selection_inverse, orig_corner_edges, corner_edges.as_mutable_span());

  const Span<float3> face_normals = mesh.face_normals();

  CustomData_realloc(&mesh.edge_data, orig_edges.size(), edges.size());
  CustomData_realloc(&mesh.face_data, orig_faces.size(), orig_faces.size() + new_faces_num);
  CustomData_realloc(&mesh.loop_data, orig_corner_verts.size(), corner_verts.size());
}

}  // namespace blender::geometry
