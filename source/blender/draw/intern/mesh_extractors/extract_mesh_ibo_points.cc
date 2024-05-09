/* SPDX-FileCopyrightText: 2021 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw
 */

#include "BLI_array_utils.hh"

#include "GPU_index_buffer.hh"

#include "draw_subdivision.hh"
#include "extract_mesh.hh"

namespace blender::draw {

/* ---------------------------------------------------------------------- */
/** \name Extract Point Indices
 * \{ */

static IndexMask calc_mesh_vert_visibility(const MeshRenderData &mr,
                                           const IndexMask &mask,
                                           IndexMaskMemory &memory)
{
  IndexMask visible = mask;
  if (!mr.hide_vert.is_empty()) {
    visible = IndexMask::from_bools_inverse(visible, mr.hide_vert, memory);
  }
  if (mr.v_origindex != nullptr) {
    const int *orig_index = mr.v_origindex;
    visible = IndexMask::from_predicate(visible, GrainSize(4096), memory, [&](const int64_t i) {
      return orig_index[i] != ORIGINDEX_NONE;
    });
  }
  return visible;
}

static void extract_points_mesh(const MeshRenderData &mr, gpu::IndexBuf &points)
{
  const Span<int> corner_verts = mr.corner_verts;
  const int max_index = mr.corners_num + mr.loose_edges.size() * 2 + mr.loose_verts.size();

  IndexMaskMemory memory;
  const IndexMask visible_verts = calc_mesh_vert_visibility(mr, IndexMask(mr.verts_num), memory);

  GPUIndexBufBuilder builder;
  GPU_indexbuf_init(&builder, GPU_PRIM_LINES, visible_verts.size(), max_index);
  MutableSpan<uint> data = GPU_indexbuf_get_data(&builder);

  /* This code fills the index buffer in a non-deterministic way, with non-atomic access to
   * `used`/`map`. This is okay because any of the possible face corner indices are correct, since
   * they all correspond to the same #Mesh vertex. The separate arrays exist as a performance
   * optimization to avoid writing to the VBO. */
  if (visible_verts.size() == mr.verts_num && mr.loose_edges.is_empty()) {
    Array<bool> used(mr.verts_num, false);
    threading::memory_bandwidth_bound_task(
        used.as_span().size_in_bytes() + data.size_in_bytes() + corner_verts.size_in_bytes(),
        [&]() {
          threading::parallel_for(corner_verts.index_range(), 4096, [&](const IndexRange range) {
            for (const int corner : range) {
              const int vert = corner_verts[corner];
              if (!used[vert]) {
                data[vert] = corner;
                used[vert] = true;
              }
            }
          });
        });
    GPU_indexbuf_build_in_place_ex(&builder, 0, max_index, false, &points);
    return;
  }

  /* Compresses the vertex indices into the smaller range of visible vertices in the IBO. */
  Array<int> map(mr.verts_num, -1);
  threading::memory_bandwidth_bound_task(
      map.as_span().size_in_bytes() + data.size_in_bytes() + corner_verts.size_in_bytes(), [&]() {
        index_mask::build_reverse_map(visible_verts, map.as_mutable_span());
        threading::parallel_for(corner_verts.index_range(), 2048, [&](const IndexRange range) {
          for (const int corner : range) {
            const int vert = corner_verts[corner];
            if (map[vert] != -1) {
              data[map[vert]] = corner;
              map[vert] = -1;
            }
          }
        });
      });

  const int loose_edges_start = mr.corners_num;
  const Span<int2> edges = mr.edges;
  const Span<int> loose_edges = mr.loose_edges;
  threading::parallel_for(loose_edges.index_range(), 2048, [&](const IndexRange range) {
    for (const int i : range) {
      const int2 edge = edges[loose_edges[i]];
      if (map[edge[0]] != -1) {
        data[map[edge[0]]] = loose_edges_start + i;
        map[edge[0]] = -1;
      }
      if (map[edge[1]] != -1) {
        data[map[edge[1]]] = loose_edges_start + i;
        map[edge[1]] = -1;
      }
    }
  });

  const int loose_verts_start = mr.corners_num + loose_edges.size() * 2;
  const Span<int> loose_verts = mr.loose_verts;
  threading::parallel_for(loose_verts.index_range(), 2048, [&](const IndexRange range) {
    for (const int i : range) {
      const int vert = loose_verts[i];
      if (map[vert] != -1) {
        data[map[vert]] = loose_verts_start + i;
        map[vert] = -1;
      }
    }
  });

  GPU_indexbuf_build_in_place_ex(&builder, 0, max_index, false, &points);
}

static void extract_points_bm(const MeshRenderData &mr, gpu::IndexBuf &points)
{
  BMesh &bm = *mr.bm;

  IndexMaskMemory memory;
  const BoundedBitSpan loose_verts_bits = mr.mesh->verts_no_face().is_loose_bits;
  const IndexMask all_loose_verts = IndexMask::from_bits(loose_verts_bits, memory);
  const IndexMask visible_loose_verts = calc_mesh_vert_visibility(mr, all_loose_verts, memory);
  const int max_index = mr.corners_num + visible_loose_verts.size() * 2;

  const IndexMask non_loose_verts = all_loose_verts.complement(IndexRange(mr.verts_num), memory);
  const IndexMask visible_non_loose_verts = calc_mesh_vert_visibility(mr, non_loose_verts, memory);

  GPUIndexBufBuilder builder;
  GPU_indexbuf_init(&builder,
                    GPU_PRIM_LINES,
                    visible_non_loose_verts.size() + visible_loose_verts.size(),
                    max_index);
  MutableSpan<uint> data = GPU_indexbuf_get_data(&builder);

  /* Make use of BMesh's vertex to loop topology knowledge to iterate over edges instead of
   * iterating over faces and defining edges implicitly as done in the #Mesh extraction. */
  visible_non_loose_verts.foreach_index(GrainSize(4096), [&](const int i, const int pos) {
    const BMVert &vert = *BM_vert_at_index(&bm, i);
    data[pos] = BM_elem_index_get(vert.e->l);
  });

  const uint loose_vert_index_start = mr.corners_num;
  MutableSpan<uint> loose_vert_data = data.take_back(visible_loose_verts.size());
  if (visible_loose_verts.size() == all_loose_verts.size()) {
    visible.shift(start, memory).to_indices(data.cast<int32_t>());
  }
  else {
    visible_loose_verts.foreach_index(GrainSize(4096), [&](const int i, const int pos) {
      const BMVert &vert = *BM_vert_at_index(&bm, i);
      // TODO: Wrong
      loose_vert_data[pos] = loose_vert_index_start + BM_elem_index_get(&vert);
    });
  }

  GPU_indexbuf_build_in_place_ex(&builder, 0, max_index, false, &points);
}

void extract_points(const MeshRenderData &mr, gpu::IndexBuf &points)
{
  if (mr.extract_type == MR_EXTRACT_MESH) {
    extract_points_mesh(mr, points);
  }
  else {
    extract_points_bm(mr, points);
  }
}

static void extract_points_subdiv(const MeshRenderData &mr,
                                  const DRWSubdivCache &subdiv_cache,
                                  gpu::IndexBuf &points)
{
  const Span<bool> hide_vert = mr.hide_vert;
  const Span<int> subdiv_corner_orig_verts{
      static_cast<int *>(GPU_vertbuf_get_data(subdiv_cache.verts_orig_index)),
      subdiv_cache.num_subdiv_loops};

  IndexMaskMemory memory;
  IndexMask visible_non_loose = IndexMask::from_predicate(
      subdiv_corner_orig_verts.index_range(), GrainSize(4096), memory, [&](const int i) {
        return subdiv_corner_orig_verts[i] != -1;
      });
  if (!hide_vert.is_empty()) {
    visible_non_loose = IndexMask::from_predicate(
        visible_non_loose, GrainSize(4096), memory, [&](const int subdiv_corner) {
          const int vert = subdiv_corner_orig_verts[subdiv_corner];
          return !hide_vert[vert];
        });
  }
  if (mr.v_origindex) {
    const int *orig_index = mr.v_origindex;
    visible_non_loose = IndexMask::from_predicate(
        visible_non_loose, GrainSize(4096), memory, [&](const int subdiv_corner) {
          const int vert = subdiv_corner_orig_verts[subdiv_corner];
          return orig_index[vert] != -1;
        });
  }

  const Span<int> loose_edges = mr.loose_edges;
  const DRWSubdivLooseGeom &loose_info = subdiv_cache.loose_info;
  const int edges_per_coarse_edge = loose_info.edges_per_coarse_edge;
  const int verts_per_coarse_edge = edges_per_coarse_edge * 2;
  const int subdiv_loose_edges_num = loose_edges.size() * edges_per_coarse_edge;

  // TODO: Loose edges
  IndexMask visible_non_loose = IndexMask::from_predicate(
      loose_edges.index_range(), GrainSize(4096), memory, [&](const int i) {
        return subdiv_corner_orig_verts[i] != -1;
      });
  const Span<DRWSubdivLooseEdge> loose_edges = draw_subdiv_cache_get_loose_edges(subdiv_cache);
  for (const DRWSubdivLooseEdge &loose_edge : loose_edges) {
    const DRWSubdivLooseVertex &v1 = loose_info.verts[loose_edge.loose_subdiv_v1_index];
    const DRWSubdivLooseVertex &v2 = loose_info.verts[loose_edge.loose_subdiv_v2_index];
    if (v1.coarse_vertex_index != -1u) {
      vert_set_mesh(elb, mr, v1.coarse_vertex_index, offset);
    }
    if (v2.coarse_vertex_index != -1u) {
      vert_set_mesh(elb, mr, v2.coarse_vertex_index, offset + 1);
    }

    offset += 2;
  }

  const Span<int> loose_verts = mr.loose_verts;
  IndexMask visible_loose(loose_verts.size());
  if (!hide_vert.is_empty()) {
    visible_loose = IndexMask::from_predicate(
        visible_loose, GrainSize(4096), memory, [&](const int i) {
          const int vert = loose_verts[i];
          return !hide_vert[vert] != -1;
        });
  }
  if (mr.v_origindex) {
    const int *orig_index = mr.v_origindex;
    visible_loose = IndexMask::from_predicate(
        visible_loose, GrainSize(4096), memory, [&](const int i) {
          const int vert = loose_verts[i];
          return orig_index[vert] != -1;
        });
  }

  Span<DRWSubdivLooseVertex> loose_verts = draw_subdiv_cache_get_loose_verts(subdiv_cache);

  const IndexMask all_loose_verts = IndexMask::from_bits(loose_verts_bits, memory);
  const IndexMask visible_loose_verts = calc_mesh_vert_visibility(mr, all_loose_verts, memory);

  GPUIndexBufBuilder builder;
  GPU_indexbuf_init(&builder,
                    GPU_PRIM_POINTS,
                    visible_non_loose.size(),
                    visible_points.size() +
                        subdiv_cache.loose_info.coarse_edge_indices.size() * 2 +
                        subdiv_cache.loose_info.coarse_vert_indices.size());
  MutableSpan<uint> data = GPU_indexbuf_get_data(&builder);
  visible_points.to_indices<int32_t>(data.take_front(visible_points.size()).cast<int32_t>());

  GPU_indexbuf_build_in_place(&builder, &points);
}

static void extract_points_loose_geom_subdiv(const DRWSubdivCache &subdiv_cache,
                                             const MeshRenderData &mr,
                                             void * /*buffer*/,
                                             void *data)
{
  const DRWSubdivLooseGeom &loose_info = subdiv_cache.loose_info;
  const int loose_indices_num = loose_info.loop_len;
  if (loose_indices_num == 0) {
    return;
  }

  GPUIndexBufBuilder *elb = static_cast<GPUIndexBufBuilder *>(data);

  uint offset = subdiv_cache.num_subdiv_loops;

  if (mr.extract_type != MR_EXTRACT_BMESH) {
  }
  else {
    Span<DRWSubdivLooseEdge> loose_edges = draw_subdiv_cache_get_loose_edges(subdiv_cache);

    for (const DRWSubdivLooseEdge &loose_edge : loose_edges) {
      const DRWSubdivLooseVertex &v1 = loose_info.verts[loose_edge.loose_subdiv_v1_index];
      const DRWSubdivLooseVertex &v2 = loose_info.verts[loose_edge.loose_subdiv_v2_index];
      if (v1.coarse_vertex_index != -1u) {
        BMVert *eve = mr.v_origindex ? bm_original_vert_get(mr, v1.coarse_vertex_index) :
                                       BM_vert_at_index(mr.bm, v1.coarse_vertex_index);
        vert_set_bm(elb, eve, offset);
      }
      if (v2.coarse_vertex_index != -1u) {
        BMVert *eve = mr.v_origindex ? bm_original_vert_get(mr, v2.coarse_vertex_index) :
                                       BM_vert_at_index(mr.bm, v2.coarse_vertex_index);
        vert_set_bm(elb, eve, offset + 1);
      }

      offset += 2;
    }
    Span<DRWSubdivLooseVertex> loose_verts = draw_subdiv_cache_get_loose_verts(subdiv_cache);

    for (const DRWSubdivLooseVertex &loose_vert : loose_verts) {
      BMVert *eve = mr.v_origindex ? bm_original_vert_get(mr, loose_vert.coarse_vertex_index) :
                                     BM_vert_at_index(mr.bm, loose_vert.coarse_vertex_index);
      vert_set_bm(elb, eve, offset);
      offset += 1;
    }
  }
}

/** \} */

}  // namespace blender::draw
