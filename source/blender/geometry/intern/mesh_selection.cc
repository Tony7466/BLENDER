/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <iostream>

#include "BLI_index_mask.hh"
#include "BLI_timeit.hh"

#include "BKE_mesh.hh"

#include "GEO_mesh_selection.hh"

namespace blender::geometry {

template<typename T> std::ostream &operator<<(std::ostream &stream, const Span<T> span)
{
  for (const int64_t i : span.index_range()) {
    stream << span[i] << (span.size() - 1 == i ? "" : ", ");
  }
  return stream;
}

template<typename T> std::ostream &operator<<(std::ostream &stream, MutableSpan<T> span)
{
  stream << span.as_span();
  return stream;
}

IndexMask vert_selection_from_edge(const Span<int2> edges,
                                   const IndexMask &edge_mask,
                                   const int verts_num,
                                   IndexMaskMemory &memory)
{
  Array<bool> array(verts_num, false);
  edge_mask.foreach_index_optimized<int>(GrainSize(4096), [&](const int i) {
    array[edges[i][0]] = true;
    array[edges[i][1]] = true;
  });
  return IndexMask::from_bools(array, memory);
}

static IndexMask mapped_corner_selection_from_face(const OffsetIndices<int> faces,
                                                   const IndexMask &face_mask,
                                                   const Span<int> corner_verts_or_edges,
                                                   const int verts_or_edges_num,
                                                   IndexMaskMemory &memory)
{
  Array<bool> array(verts_or_edges_num, false);
  face_mask.foreach_index(GrainSize(512), [&](const int64_t i) {
    array.as_mutable_span().fill_indices(corner_verts_or_edges.slice(faces[i]), true);
  });
  return IndexMask::from_bools(array, memory);
}

IndexMask vert_selection_from_face(const OffsetIndices<int> faces,
                                   const IndexMask &face_mask,
                                   const Span<int> corner_verts,
                                   const int verts_num,
                                   IndexMaskMemory &memory)
{
  return mapped_corner_selection_from_face(faces, face_mask, corner_verts, verts_num, memory);
}

IndexMask edge_selection_from_face(const OffsetIndices<int> faces,
                                   const IndexMask &face_mask,
                                   const Span<int> corner_edges,
                                   const int edges_num,
                                   IndexMaskMemory &memory)
{
  return mapped_corner_selection_from_face(faces, face_mask, corner_edges, edges_num, memory);
}

IndexMask edge_selection_from_vert(const Span<int2> edges,
                                   const Span<bool> vert_selection,
                                   IndexMaskMemory &memory)
{
  return IndexMask::from_predicate(
      edges.index_range(), GrainSize(1024), memory, [&](const int64_t i) {
        const int2 edge = edges[i];
        return vert_selection[edge[0]] && vert_selection[edge[1]];
      });
}

static IndexMask face_selection_from_mapped_corner(const OffsetIndices<int> faces,
                                                   const Span<int> corner_verts_or_edges,
                                                   const Span<bool> vert_or_edge_selection,
                                                   IndexMaskMemory &memory)
{
  return IndexMask::from_predicate(
      faces.index_range(), GrainSize(1024), memory, [&](const int64_t i) {
        const Span<int> indices = corner_verts_or_edges.slice(faces[i]);
        return std::all_of(indices.begin(), indices.end(), [&](const int i) {
          return vert_or_edge_selection[i];
        });
      });
}

#if (0)

IndexMask face_selection_from_vert(const OffsetIndices<int> faces,
                                   const Span<int> corner_verts,
                                   const Span<bool> vert_selection,
                                   IndexMaskMemory &memory)
{
  SCOPED_TIMER_AVERAGED("old vert selection");
  return face_selection_from_mapped_corner(faces, corner_verts, vert_selection, memory);
}

IndexMask face_selection_from_edge(const OffsetIndices<int> faces,
                                   const Span<int> corner_edges,
                                   const Span<bool> edge_mask,
                                   IndexMaskMemory &memory)
{
  SCOPED_TIMER_AVERAGED("old vert selection");
  return face_selection_from_mapped_corner(faces, corner_edges, edge_mask, memory);
}

#else

IndexMask face_selection_from_vert(const OffsetIndices<int> faces,
                                   const Span<int> corner_verts,
                                   const Span<bool> vert_selection,
                                   IndexMaskMemory &memory)
{
  IndexMaskMemory tmp_memory;
  IndexMask corner_selection;
  {
    SCOPED_TIMER_AVERAGED("construct mask");
    corner_selection = IndexMask::from_predicate(
        corner_verts.index_range(), GrainSize(1024), memory, [&](const int64_t i) {
          return vert_selection[corner_verts[i]];
        });
  }

  BLI_assert(corner_selection.all_of(faces, memory) ==
             face_selection_from_mapped_corner(faces, corner_verts, vert_selection, memory));
  SCOPED_TIMER_AVERAGED("new vert selection");
  return corner_selection.all_of(faces, memory);
}

IndexMask face_selection_from_edge(const OffsetIndices<int> faces,
                                   const Span<int> corner_edges,
                                   const Span<bool> edge_mask,
                                   IndexMaskMemory &memory)
{
  IndexMaskMemory tmp_memory;
  IndexMask corner_selection;
  {
    SCOPED_TIMER_AVERAGED("construct mask");
    corner_selection = IndexMask::from_predicate(
        corner_edges.index_range(), GrainSize(1024), memory, [&](const int64_t i) {
          return edge_mask[corner_edges[i]];
        });
  }

  BLI_assert(corner_selection.all_of(faces, memory) ==
             face_selection_from_mapped_corner(faces, corner_edges, edge_mask, memory));
  SCOPED_TIMER_AVERAGED("new edge selection");
  return corner_selection.all_of(faces, memory);
}

#endif

}  // namespace blender::geometry
