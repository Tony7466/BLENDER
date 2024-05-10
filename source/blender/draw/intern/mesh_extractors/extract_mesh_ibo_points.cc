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

static IndexMask calc_vert_visibility_mesh(const MeshRenderData &mr,
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

/**
 * Fill the index buffer in a parallel non-deterministic fashion. This is okay because any of the
 * possible face corner indices are correct, since they all correspond to the same #Mesh vertex.
 * The separate arrays exist as a performance optimization to avoid writing to the VBO.
 */
template<typename Fn>
static void process_ibo_verts_mesh(const MeshRenderData &mr, const Fn &process_vert_fn)
{
  const Span<int> corner_verts = mr.corner_verts;
  threading::parallel_for(corner_verts.index_range(), 2048, [&](const IndexRange range) {
    for (const int corner : range) {
      process_vert_fn(corner, corner_verts[corner]);
    }
  });

  const int loose_edges_start = mr.corners_num;
  const Span<int2> edges = mr.edges;
  const Span<int> loose_edges = mr.loose_edges;
  threading::parallel_for(loose_edges.index_range(), 2048, [&](const IndexRange range) {
    for (const int i : range) {
      const int2 edge = edges[loose_edges[i]];
      process_vert_fn(loose_edges_start + i + 0, edge[0]);
      process_vert_fn(loose_edges_start + i + 1, edge[1]);
    }
  });

  const int loose_verts_start = mr.corners_num + loose_edges.size() * 2;
  const Span<int> loose_verts = mr.loose_verts;
  threading::parallel_for(loose_verts.index_range(), 2048, [&](const IndexRange range) {
    for (const int i : range) {
      process_vert_fn(loose_verts_start + i, loose_verts[i]);
    }
  });
}

static void extract_points_mesh(const MeshRenderData &mr, gpu::IndexBuf &points)
{
  const Span<int> corner_verts = mr.corner_verts;
  const int max_index = mr.corners_num + mr.loose_edges.size() * 2 + mr.loose_verts.size();

  IndexMaskMemory memory;
  const IndexMask visible_verts = calc_vert_visibility_mesh(mr, IndexMask(mr.verts_num), memory);

  GPUIndexBufBuilder builder;
  GPU_indexbuf_init(&builder, GPU_PRIM_POINTS, visible_verts.size(), max_index);
  MutableSpan<uint> data = GPU_indexbuf_get_data(&builder);

  threading::memory_bandwidth_bound_task(corner_verts.size_in_bytes(), [&]() {
    if (visible_verts.size() == mr.verts_num) {
      Array<bool> used(mr.verts_num, false);
      process_ibo_verts_mesh(mr, [&](const int ibo_index, const int vert) {
        if (!used[vert]) {
          data[vert] = ibo_index;
          used[vert] = true;
        }
      });
    }
    else {
      /* Compress the vertex indices into the smaller range of visible vertices in the IBO. */
      Array<int> map(mr.verts_num, -1);
      index_mask::build_reverse_map(visible_verts, map.as_mutable_span());
      process_ibo_verts_mesh(mr, [&](const int ibo_index, const int vert) {
        if (map[vert] != -1) {
          data[map[vert]] = ibo_index;
          map[vert] = -1;
        }
      });
    }
  });

  GPU_indexbuf_build_in_place_ex(&builder, 0, max_index, false, &points);
}

template<typename Fn>
static void process_ibo_verts_bm(const MeshRenderData &mr, const Fn &process_vert_fn)
{
  BMesh &bm = *mr.bm;

  threading::parallel_for(IndexRange(mr.verts_num), 2048, [&](const IndexRange range) {
    for (const int i : range) {
      BMVert &vert = *BM_vert_at_index(&bm, i);
      if (const BMLoop *loop = BM_vert_find_first_loop(&vert)) {
        process_vert_fn(BM_elem_index_get(loop), i);
      }
    }
  });

  const int loose_edges_start = mr.corners_num;
  const Span<int> loose_edges = mr.loose_edges;
  threading::parallel_for(loose_edges.index_range(), 2048, [&](const IndexRange range) {
    for (const int i : range) {
      const BMEdge &edge = *BM_edge_at_index(&bm, loose_edges[i]);
      process_vert_fn(loose_edges_start + i * 2 + 0, BM_elem_index_get(edge.v1));
      process_vert_fn(loose_edges_start + i * 2 + 1, BM_elem_index_get(edge.v2));
    }
  });

  const int loose_verts_start = mr.corners_num + loose_edges.size() * 2;
  const Span<int> loose_verts = mr.loose_verts;
  threading::parallel_for(loose_verts.index_range(), 2048, [&](const IndexRange range) {
    for (const int i : range) {
      process_vert_fn(loose_verts_start + i,
                      BM_elem_index_get(BM_vert_at_index(&bm, loose_verts[i])));
    }
  });
}

static void extract_points_bm(const MeshRenderData &mr, gpu::IndexBuf &points)
{
  BMesh &bm = *mr.bm;

  IndexMaskMemory memory;
  const IndexMask visible_verts = IndexMask::from_predicate(
      IndexRange(bm.totvert), GrainSize(2048), memory, [&](const int i) {
        return !BM_elem_flag_test_bool(BM_vert_at_index(&const_cast<BMesh &>(bm), i),
                                       BM_ELEM_HIDDEN);
      });
  const int max_index = mr.corners_num + mr.loose_edges.size() * 2 + mr.loose_verts.size();

  GPUIndexBufBuilder builder;
  GPU_indexbuf_init(&builder, GPU_PRIM_POINTS, visible_verts.size(), max_index);
  MutableSpan<uint> data = GPU_indexbuf_get_data(&builder);

  if (mr.loose_verts.is_empty() && mr.loose_edges.is_empty()) {
    /* Make use of BMesh's vertex to loop topology knowledge to iterate over verts instead of
     * iterating over faces and defining points implicitly as done in the #Mesh extraction. */
    visible_verts.foreach_index(GrainSize(4096), [&](const int i, const int pos) {
      BMVert &vert = *BM_vert_at_index(&bm, i);
      data[pos] = BM_elem_index_get(BM_vert_find_first_loop(&vert));
    });
  }
  else if (visible_verts.size() == bm.totvert) {
    Array<bool> used(mr.verts_num, false);
    process_ibo_verts_bm(mr, [&](const int ibo_index, const int vert) {
      if (!used[vert]) {
        data[vert] = ibo_index;
        used[vert] = true;
      }
    });
  }
  else {
    /* Compress the vertex indices into the smaller range of visible vertices in the IBO. */
    Array<int> map(mr.verts_num, -1);
    index_mask::build_reverse_map(visible_verts, map.as_mutable_span());
    process_ibo_verts_bm(mr, [&](const int ibo_index, const int vert) {
      if (map[vert] != -1) {
        data[map[vert]] = ibo_index;
        map[vert] = -1;
      }
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

static IndexMask calc_vert_visibility_mapped_mesh(const MeshRenderData &mr,
                                                  const IndexMask &mask,
                                                  const Span<int> map,
                                                  IndexMaskMemory &memory)
{
  IndexMask visible = mask;
  if (!mr.hide_vert.is_empty()) {
    const Span<bool> hide_vert = mr.hide_vert;
    visible = IndexMask::from_predicate(
        visible, GrainSize(4096), memory, [&](const int i) { return !hide_vert[map[i]]; });
  }
  if (mr.v_origindex != nullptr) {
    const int *orig_index = mr.v_origindex;
    visible = IndexMask::from_predicate(visible, GrainSize(4096), memory, [&](const int i) {
      return orig_index[map[i]] != ORIGINDEX_NONE;
    });
  }
  return visible;
}

static void extract_points_subdiv_mesh(const MeshRenderData &mr,
                                       const DRWSubdivCache &subdiv_cache,
                                       gpu::IndexBuf &points)
{

  const Span<bool> hide_vert = mr.hide_vert;
  const Span<int> corner_orig_verts{
      static_cast<int *>(GPU_vertbuf_get_data(subdiv_cache.verts_orig_index)),
      subdiv_cache.num_subdiv_loops};

  IndexMaskMemory memory;
  IndexMask visible_corners = IndexMask::from_predicate(
      corner_orig_verts.index_range(), GrainSize(4096), memory, [&](const int i) {
        return corner_orig_verts[i] != -1;
      });
  visible_corners = calc_vert_visibility_mapped_mesh(
      mr, visible_corners, corner_orig_verts, memory);

  const Span<int2> coarse_edges = mr.edges;
  const Span<int> loose_edges = mr.loose_edges;
  const DRWSubdivLooseGeom &loose_info = subdiv_cache.loose_info;
  const int edges_per_coarse_edge = loose_info.edges_per_coarse_edge;
  const int verts_per_coarse_edge = edges_per_coarse_edge * 2;
  const int loose_edge_verts_start = subdiv_cache.num_subdiv_loops;

  int loose_edge_verts_num;
  Vector<int> visible_loose_edge_verts;
  if (hide_vert.is_empty() && !mr.v_origindex) {
    loose_edge_verts_num = mr.loose_edges.size() * 2;
  }
  else {
    const auto show_vert = [&](const int vert) {
      if (!hide_vert.is_empty() && hide_vert[vert]) {
        return false;
      }
      if (mr.v_origindex && mr.v_origindex[vert] == ORIGINDEX_NONE) {
        return false;
      }
      return true;
    };

    visible_loose_edge_verts.reserve(mr.loose_edges.size() * 2);
    for (const int i : loose_edges.index_range()) {
      const int2 coarse_edge = coarse_edges[loose_edges[i]];
      const IndexRange edge_verts_range(loose_edge_verts_start + i * verts_per_coarse_edge,
                                        verts_per_coarse_edge);

      if (show_vert(coarse_edge[0])) {
        visible_loose_edge_verts.append(edge_verts_range.first());
      }
      if (show_vert(coarse_edge[1])) {
        visible_loose_edge_verts.append(edge_verts_range.last());
      }
    }
    loose_edge_verts_num = visible_loose_edge_verts.size();
  }

  const Span<int> loose_verts = mr.loose_verts;
  const IndexMask visible_loose = calc_vert_visibility_mapped_mesh(
      mr, IndexMask(loose_verts.size()), loose_verts, memory);

  const int max_index = subdiv_cache.num_subdiv_loops +
                        loose_edges.size() * verts_per_coarse_edge + loose_verts.size();

  GPUIndexBufBuilder builder;
  GPU_indexbuf_init(&builder,
                    GPU_PRIM_POINTS,
                    visible_corners.size() + visible_loose_edge_verts.size() + loose_verts.size(),
                    max_index);
  MutableSpan<uint> data = GPU_indexbuf_get_data(&builder);
  visible_corners.to_indices<int32_t>(data.take_front(visible_corners.size()).cast<int32_t>());

  data.drop_front(visible_corners.size())
      .drop_back(loose_verts.size())
      .copy_from(visible_loose_edge_verts.as_span().cast<uint>());

  const int loose_verts_start = loose_edge_verts_start + loose_edge_verts_num;
  visible_loose.shift(loose_verts_start, memory)
      .to_indices<int32_t>(data.take_back(visible_loose.size()).cast<int32_t>());

  GPU_indexbuf_build_in_place_ex(&builder, 0, max_index, false, &points);
}

void extract_points_subdiv(const MeshRenderData &mr,
                           const DRWSubdivCache &subdiv_cache,
                           gpu::IndexBuf &points)
{
  if (mr.extract_type == MR_EXTRACT_MESH) {
    extract_points_subdiv_mesh(mr, subdiv_cache, points);
  }
  else {
    // extract_points_subdiv_bm(mr, subdiv_cache, points);
  }
}

// static void extract_points_loose_geom_subdiv(const DRWSubdivCache &subdiv_cache,
//                                              const MeshRenderData &mr,
//                                              void * /*buffer*/,
//                                              void *data)
// {
//   const DRWSubdivLooseGeom &loose_info = subdiv_cache.loose_info;
//   if (mr.loose_verts.is_empty() && mr.loose_edges.is_empty()) {
//     return;
//   }

//   GPUIndexBufBuilder *elb = static_cast<GPUIndexBufBuilder *>(data);

//   uint offset = subdiv_cache.num_subdiv_loops;

//   Span<DRWSubdivLooseEdge> loose_edges = draw_subdiv_cache_get_loose_edges(subdiv_cache);

//   for (const DRWSubdivLooseEdge &loose_edge : loose_edges) {
//     const DRWSubdivLooseVertex &v1 = loose_info.verts[loose_edge.loose_subdiv_v1_index];
//     const DRWSubdivLooseVertex &v2 = loose_info.verts[loose_edge.loose_subdiv_v2_index];
//     if (v1.coarse_vertex_index != -1u) {
//       BMVert *eve = mr.v_origindex ? bm_original_vert_get(mr, v1.coarse_vertex_index) :
//                                      BM_vert_at_index(mr.bm, v1.coarse_vertex_index);
//       vert_set_bm(elb, eve, offset);
//     }
//     if (v2.coarse_vertex_index != -1u) {
//       BMVert *eve = mr.v_origindex ? bm_original_vert_get(mr, v2.coarse_vertex_index) :
//                                      BM_vert_at_index(mr.bm, v2.coarse_vertex_index);
//       vert_set_bm(elb, eve, offset + 1);
//     }

//     offset += 2;
//   }
//   Span<DRWSubdivLooseVertex> loose_verts = draw_subdiv_cache_get_loose_verts(subdiv_cache);

//   for (const DRWSubdivLooseVertex &loose_vert : loose_verts) {
//     BMVert *eve = mr.v_origindex ? bm_original_vert_get(mr, loose_vert.coarse_vertex_index) :
//                                    BM_vert_at_index(mr.bm, loose_vert.coarse_vertex_index);
//     vert_set_bm(elb, eve, offset);
//     offset += 1;
//   }
// }

}  // namespace blender::draw
