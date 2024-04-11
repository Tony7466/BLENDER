/* SPDX-FileCopyrightText: 2021 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw
 */

#include "GPU_index_buffer.hh"

#include "extract_mesh.hh"

#include "draw_cache_inline.hh"
#include "draw_subdivision.hh"

namespace blender::draw {

/* ---------------------------------------------------------------------- */
/** \name Extract Edges Indices
 * \{ */

static IndexMask calc_edge_visibility(const MeshRenderData &mr,
                                      const IndexMask &start_mask,
                                      IndexMaskMemory &memory)
{
  IndexMask visible = start_mask;
  if (!mr.mesh->runtime->subsurf_optimal_display_edges.is_empty()) {
    const BoundedBitSpan visible_bits = mr.mesh->runtime->subsurf_optimal_display_edges;
    visible = IndexMask::from_bits(visible, visible_bits, memory);
  }
  if (!mr.hide_edge.is_empty()) {
    visible = IndexMask::from_bools(visible, mr.hide_edge, memory);
  }
  if (mr.hide_unmapped_edges && mr.e_origindex != nullptr) {
    const int *orig_index = mr.e_origindex;
    visible = IndexMask::from_predicate(visible, GrainSize(4096), memory, [&](const int64_t i) {
      return orig_index[i] != ORIGINDEX_NONE;
    });
  }
  return visible;
}

static IndexMask calc_loose_edge_visibility(const MeshRenderData &mr, IndexMaskMemory &memory)
{
  return calc_edge_visibility(mr, IndexMask::from_indices(mr.loose_edges, memory), memory);
}
static IndexMask calc_edge_visibility(const MeshRenderData &mr, IndexMaskMemory &memory)
{
  return calc_edge_visibility(mr, IndexMask(mr.edges_num), memory);
}

/* In the GPU vertex buffers, the value for each vertex is duplicated to each of its vertex
 * corners. So the edges on the GPU connect face corners rather than vertices. */
static uint2 edge_from_corners(const IndexRange face, const int corner)
{
  const int corner_next = bke::mesh::face_corner_next(face, corner);
  return uint2(corner, corner_next);
}

static void fill_loose_lines_ibo(const int corners_num, MutableSpan<uint2> data)
{
  threading::memory_bandwidth_bound_task(data.size_in_bytes(), [&]() {
    threading::parallel_for(data.index_range(), 4096, [&](const IndexRange range) {
      for (const int loose_edge : range) {
        const int index = corners_num + loose_edge * 2;
        data[loose_edge] = uint2(index, index + 1);
      }
    });
  });
}

void extract_lines_mesh(const MeshRenderData &mr, gpu::IndexBuf *lines, gpu::IndexBuf *lines_loose)
{
  if (DRW_ibo_requested(lines_loose) && !DRW_ibo_requested(lines)) {
    IndexMaskMemory memory;
    const IndexMask visible = calc_loose_edge_visibility(mr, memory);

    GPUIndexBufBuilder builder;
    GPU_indexbuf_init(
        &builder, GPU_PRIM_LINES, visible.size(), mr.corners_num + mr.loose_indices_num);
    MutableSpan<uint2> data = GPU_indexbuf_get_data(&builder).cast<uint2>();
    BLI_assert(data.size() == visible.size());

    fill_loose_lines_ibo(mr.corners_num, data);

    GPU_indexbuf_build_in_place_ex(
        &builder, 0, mr.corners_num + visible.size() * 2, false, lines_loose);
    return;
  }

  IndexMaskMemory memory;
  const IndexMask visible = calc_edge_visibility(mr, memory);

  // TODO: Also add loose edges here.

  GPUIndexBufBuilder builder;
  GPU_indexbuf_init(&builder, GPU_PRIM_LINES, visible.size(), mr.corners_num);
  MutableSpan<uint2> data = GPU_indexbuf_get_data(&builder).cast<uint2>();

  const OffsetIndices faces = mr.faces;
  const Span<int> corner_edges = mr.corner_edges;
  if (visible.size() == mr.edges_num) {
    /* Use separate boolean array to avoid writing to the same indices again multiple times,
     * possibly from different threads. This is slightly beneficial because booleans are 8 times
     * smaller than the `uint2` for each edge. */
    Array<bool> used(mr.edges_num, false);
    threading::memory_bandwidth_bound_task(
        used.as_span().size_in_bytes() + data.size_in_bytes() + corner_edges.size_in_bytes(),
        [&]() {
          threading::parallel_for(faces.index_range(), 1024, [&](const IndexRange range) {
            for (const int face_index : range) {
              const IndexRange face = faces[face_index];
              for (const int corner : face) {
                const int edge = corner_edges[corner];
                if (used[edge]) {
                  continue;
                }
                data[edge] = edge_from_corners(face, corner);
                used[edge] = true;
              }
            }
          });
        });
  }
  else {
    Array<int> map(mr.corners_num, -1);
    threading::memory_bandwidth_bound_task(
        map.as_span().size_in_bytes() + data.size_in_bytes() + corner_edges.size_in_bytes(),
        [&]() {
          index_mask::build_reverse_map(visible, map.as_mutable_span());
          threading::parallel_for(faces.index_range(), 1024, [&](const IndexRange range) {
            for (const int face_index : range) {
              const IndexRange face = faces[face_index];
              for (const int corner : face) {
                const int edge = corner_edges[corner];
                if (map[edge] == -1) {
                  continue;
                }
                data[map[edge]] = edge_from_corners(face, corner);
                map[edge] = -1;
              }
            }
          });
        });
  }

  GPU_indexbuf_build_in_place_ex(&builder, 0, mr.corners_num + visible.size() * 2, false, lines);
  return;
}

struct MeshExtract_LinesData {
  GPUIndexBufBuilder elb;
  BitSpan optimal_display_edges;
  const int *e_origindex;
  Span<bool> hide_edge;
  bool test_visibility;
};

BLI_INLINE bool is_edge_visible(const MeshExtract_LinesData *data, const int edge)
{
  if (!data->hide_edge.is_empty() && data->hide_edge[edge]) {
    return false;
  }
  if (data->e_origindex && data->e_origindex[edge] == ORIGINDEX_NONE) {
    return false;
  }
  if (!data->optimal_display_edges.is_empty() && !data->optimal_display_edges[edge]) {
    return false;
  }
  return true;
}

static void extract_lines_init(const MeshRenderData &mr,
                               MeshBatchCache & /*cache*/,
                               void * /*buf*/,
                               void *tls_data)
{
  MeshExtract_LinesData *data = static_cast<MeshExtract_LinesData *>(tls_data);
  /* Put loose edges at the end. */
  GPU_indexbuf_init(&data->elb,
                    GPU_PRIM_LINES,
                    mr.edges_num + mr.loose_edges_num,
                    mr.corners_num + mr.loose_indices_num);

  if (mr.extract_type == MR_EXTRACT_MESH) {
    data->optimal_display_edges = mr.mesh->runtime->subsurf_optimal_display_edges;
    data->e_origindex = mr.hide_unmapped_edges ? mr.e_origindex : nullptr;
    data->hide_edge = mr.use_hide ? Span(mr.hide_edge) : Span<bool>();

    data->test_visibility = !data->optimal_display_edges.is_empty() || data->e_origindex ||
                            !data->hide_edge.is_empty();
  }
}

static void extract_lines_iter_face_bm(const MeshRenderData & /*mr*/,
                                       const BMFace *f,
                                       const int /*f_index*/,
                                       void *tls_data)
{
  MeshExtract_LinesData *data = static_cast<MeshExtract_LinesData *>(tls_data);
  GPUIndexBufBuilder *elb = &data->elb;
  BMLoop *l_iter, *l_first;
  /* Use #BMLoop.prev to match mesh order (to avoid minor differences in data extraction). */
  l_iter = l_first = BM_FACE_FIRST_LOOP(f)->prev;
  do {
    if (!BM_elem_flag_test(l_iter->e, BM_ELEM_HIDDEN)) {
      GPU_indexbuf_set_line_verts(elb,
                                  BM_elem_index_get(l_iter->e),
                                  BM_elem_index_get(l_iter),
                                  BM_elem_index_get(l_iter->next));
    }
    else {
      GPU_indexbuf_set_line_restart(elb, BM_elem_index_get(l_iter->e));
    }
  } while ((l_iter = l_iter->next) != l_first);
}

static void extract_lines_iter_loose_edge_bm(const MeshRenderData &mr,
                                             const BMEdge *eed,
                                             const int loose_edge_i,
                                             void *tls_data)
{
  MeshExtract_LinesData *data = static_cast<MeshExtract_LinesData *>(tls_data);
  GPUIndexBufBuilder *elb = &data->elb;
  const int l_index_offset = mr.edges_num + loose_edge_i;
  if (!BM_elem_flag_test(eed, BM_ELEM_HIDDEN)) {
    const int l_index = mr.corners_num + loose_edge_i * 2;
    GPU_indexbuf_set_line_verts(elb, l_index_offset, l_index, l_index + 1);
  }
  else {
    GPU_indexbuf_set_line_restart(elb, l_index_offset);
  }
  /* Don't render the edge twice. */
  GPU_indexbuf_set_line_restart(elb, BM_elem_index_get(eed));
}

static void extract_lines_task_reduce(void *_userdata_to, void *_userdata_from)
{
  GPUIndexBufBuilder *elb_to = static_cast<GPUIndexBufBuilder *>(_userdata_to);
  GPUIndexBufBuilder *elb_from = static_cast<GPUIndexBufBuilder *>(_userdata_from);
  GPU_indexbuf_join(elb_to, elb_from);
}

static void extract_lines_finish(const MeshRenderData & /*mr*/,
                                 MeshBatchCache & /*cache*/,
                                 void *buf,
                                 void *tls_data)
{
  MeshExtract_LinesData *data = static_cast<MeshExtract_LinesData *>(tls_data);
  GPUIndexBufBuilder *elb = &data->elb;
  gpu::IndexBuf *ibo = static_cast<gpu::IndexBuf *>(buf);
  GPU_indexbuf_build_in_place(elb, ibo);
}

static void extract_lines_init_subdiv(const DRWSubdivCache &subdiv_cache,
                                      const MeshRenderData & /*mr*/,
                                      MeshBatchCache & /*cache*/,
                                      void *buffer,
                                      void * /*data*/)
{
  const DRWSubdivLooseGeom &loose_geom = subdiv_cache.loose_geom;
  gpu::IndexBuf *ibo = static_cast<gpu::IndexBuf *>(buffer);
  GPU_indexbuf_init_build_on_device(ibo,
                                    subdiv_cache.num_subdiv_loops * 2 + loose_geom.edge_len * 2);

  if (subdiv_cache.num_subdiv_loops == 0) {
    return;
  }

  draw_subdiv_build_lines_buffer(subdiv_cache, ibo);
}

static void extract_lines_loose_geom_subdiv(const DRWSubdivCache &subdiv_cache,
                                            const MeshRenderData &mr,
                                            void *buffer,
                                            void * /*data*/)
{
  const DRWSubdivLooseGeom &loose_geom = subdiv_cache.loose_geom;
  if (loose_geom.edge_len == 0) {
    return;
  }

  /* Update flags for loose edges, points are already handled. */
  static GPUVertFormat format;
  if (format.attr_len == 0) {
    GPU_vertformat_attr_add(&format, "data", GPU_COMP_U32, 1, GPU_FETCH_INT);
  }

  gpu::VertBuf *flags = GPU_vertbuf_calloc();
  GPU_vertbuf_init_with_format(flags, &format);

  Span<DRWSubdivLooseEdge> loose_edges = draw_subdiv_cache_get_loose_edges(subdiv_cache);
  GPU_vertbuf_data_alloc(flags, loose_edges.size());

  uint *flags_data = static_cast<uint *>(GPU_vertbuf_get_data(flags));

  switch (mr.extract_type) {
    case MR_EXTRACT_MESH: {
      const int *e_origindex = (mr.hide_unmapped_edges) ? mr.e_origindex : nullptr;
      if (e_origindex == nullptr) {
        const Span<bool> hide_edge = mr.hide_edge;
        if (!hide_edge.is_empty()) {
          for (DRWSubdivLooseEdge edge : loose_edges) {
            *flags_data++ = hide_edge[edge.coarse_edge_index];
          }
        }
        else {
          MutableSpan<uint>(flags_data, loose_edges.size()).fill(0);
        }
      }
      else {
        if (mr.bm) {
          for (DRWSubdivLooseEdge edge : loose_edges) {
            const BMEdge *bm_edge = bm_original_edge_get(mr, edge.coarse_edge_index);
            *flags_data++ = (bm_edge) ? BM_elem_flag_test_bool(bm_edge, BM_ELEM_HIDDEN) != 0 : 1;
          }
        }
        else {
          const Span<bool> hide_edge = mr.hide_edge;
          if (!hide_edge.is_empty()) {
            for (DRWSubdivLooseEdge edge : loose_edges) {
              int e = edge.coarse_edge_index;

              if (e_origindex && e_origindex[e] != ORIGINDEX_NONE) {
                *flags_data++ = hide_edge[edge.coarse_edge_index];
              }
              else {
                *flags_data++ = false;
              }
            }
          }
          else {
            MutableSpan<uint>(flags_data, loose_edges.size()).fill(0);
          }
        }
      }
      break;
    }
    case MR_EXTRACT_BMESH: {
      BMesh *bm = mr.bm;
      for (DRWSubdivLooseEdge edge : loose_edges) {
        const BMEdge *bm_edge = BM_edge_at_index(bm, edge.coarse_edge_index);
        *flags_data++ = BM_elem_flag_test_bool(bm_edge, BM_ELEM_HIDDEN) != 0;
      }
      break;
    }
  }

  gpu::IndexBuf *ibo = static_cast<gpu::IndexBuf *>(buffer);
  draw_subdiv_build_lines_loose_buffer(subdiv_cache, ibo, flags, uint(loose_geom.edge_len));

  GPU_vertbuf_discard(flags);
}

constexpr MeshExtract create_extractor_lines()
{
  MeshExtract extractor = {nullptr};
  extractor.init = extract_lines_init;
  extractor.iter_face_bm = extract_lines_iter_face_bm;
  extractor.iter_loose_edge_bm = extract_lines_iter_loose_edge_bm;
  extractor.init_subdiv = extract_lines_init_subdiv;
  extractor.iter_loose_geom_subdiv = extract_lines_loose_geom_subdiv;
  extractor.task_reduce = extract_lines_task_reduce;
  extractor.finish = extract_lines_finish;
  extractor.data_type = MR_DATA_NONE;
  extractor.data_size = sizeof(MeshExtract_LinesData);
  extractor.use_threading = true;
  extractor.mesh_buffer_offset = offsetof(MeshBufferList, ibo.lines);
  return extractor;
}

/** \} */

/* ---------------------------------------------------------------------- */
/** \name Extract Lines and Loose Edges Sub Buffer
 * \{ */

static void extract_lines_loose_subbuffer(const MeshRenderData &mr, MeshBatchCache &cache)
{
  BLI_assert(cache.final.buff.ibo.lines);
  /* Multiply by 2 because these are edges indices. */
  const int start = mr.edges_num * 2;
  const int len = mr.loose_edges_num * 2;
  GPU_indexbuf_create_subrange_in_place(
      cache.final.buff.ibo.lines_loose, cache.final.buff.ibo.lines, start, len);
  cache.no_loose_wire = (len == 0);
}

static void extract_lines_with_lines_loose_finish(const MeshRenderData &mr,
                                                  MeshBatchCache &cache,
                                                  void *buf,
                                                  void *tls_data)
{
  MeshExtract_LinesData *data = static_cast<MeshExtract_LinesData *>(tls_data);
  GPUIndexBufBuilder *elb = &data->elb;
  gpu::IndexBuf *ibo = static_cast<gpu::IndexBuf *>(buf);
  GPU_indexbuf_build_in_place(elb, ibo);
  extract_lines_loose_subbuffer(mr, cache);
}

static void extract_lines_with_lines_loose_finish_subdiv(const DRWSubdivCache &subdiv_cache,
                                                         const MeshRenderData & /*mr*/,
                                                         MeshBatchCache &cache,
                                                         void * /*buf*/,
                                                         void * /*_data*/)
{
  /* Multiply by 2 because these are edges indices. */
  const int start = subdiv_cache.num_subdiv_loops * 2;
  const int len = subdiv_cache.loose_geom.edge_len * 2;
  GPU_indexbuf_create_subrange_in_place(
      cache.final.buff.ibo.lines_loose, cache.final.buff.ibo.lines, start, len);
  cache.no_loose_wire = (len == 0);
}

constexpr MeshExtract create_extractor_lines_with_lines_loose()
{
  MeshExtract extractor = {nullptr};
  extractor.init = extract_lines_init;
  extractor.iter_face_bm = extract_lines_iter_face_bm;
  extractor.iter_loose_edge_bm = extract_lines_iter_loose_edge_bm;
  extractor.task_reduce = extract_lines_task_reduce;
  extractor.finish = extract_lines_with_lines_loose_finish;
  extractor.init_subdiv = extract_lines_init_subdiv;
  extractor.iter_loose_geom_subdiv = extract_lines_loose_geom_subdiv;
  extractor.finish_subdiv = extract_lines_with_lines_loose_finish_subdiv;
  extractor.data_type = MR_DATA_NONE;
  extractor.data_size = sizeof(MeshExtract_LinesData);
  extractor.use_threading = true;
  extractor.mesh_buffer_offset = offsetof(MeshBufferList, ibo.lines);
  return extractor;
}

/** \} */

/* ---------------------------------------------------------------------- */
/** \name Extract Loose Edges Sub Buffer
 * \{ */

static void extract_lines_loose_only_init(const MeshRenderData &mr,
                                          MeshBatchCache &cache,
                                          void *buf,
                                          void * /*tls_data*/)
{
  BLI_assert(buf == cache.final.buff.ibo.lines_loose);
  UNUSED_VARS_NDEBUG(buf);
  extract_lines_loose_subbuffer(mr, cache);
}

static void extract_lines_loose_only_init_subdiv(const DRWSubdivCache & /*subdiv_cache*/,
                                                 const MeshRenderData &mr,
                                                 MeshBatchCache &cache,
                                                 void *buffer,
                                                 void * /*data*/)
{
  BLI_assert(buffer == cache.final.buff.ibo.lines_loose);
  UNUSED_VARS_NDEBUG(buffer);
  extract_lines_loose_subbuffer(mr, cache);
}

constexpr MeshExtract create_extractor_lines_loose_only()
{
  MeshExtract extractor = {nullptr};
  extractor.init = extract_lines_loose_only_init;
  extractor.init_subdiv = extract_lines_loose_only_init_subdiv;
  extractor.data_type = MR_DATA_LOOSE_GEOM;
  extractor.data_size = 0;
  extractor.use_threading = false;
  extractor.mesh_buffer_offset = offsetof(MeshBufferList, ibo.lines_loose);
  return extractor;
}

/** \} */

const MeshExtract extract_lines = create_extractor_lines();

}  // namespace blender::draw
