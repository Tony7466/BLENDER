/* SPDX-FileCopyrightText: 2021 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw
 */

#include "BLI_array_utils.hh"

#include "BKE_editmesh.hh"

#include "GPU_index_buffer.h"

#include "extract_mesh.hh"

#include "draw_subdivision.hh"

namespace blender::draw {

/* ---------------------------------------------------------------------- */
/** \name Extract Triangles Indices (multi material)
 * \{ */

static inline void copy_tris_to_vbo(const Span<int3> corner_tris,
                                    const bool hidden,
                                    const IndexRange mesh_tris,
                                    const IndexRange vbo_tris,
                                    const uint32_t restart_value,
                                    MutableSpan<uint3> vbo_data)
{
  if (hidden) {
    vbo_data.slice(vbo_tris).cast<uint32_t>().fill(restart_value);
  }
  else {
    vbo_data.slice(vbo_tris).copy_from(corner_tris.slice(mesh_tris).cast<uint3>());
  }
}

static void extract_tris_mesh(const MeshRenderData &mr,
                              const uint32_t restart_value,
                              MutableSpan<uint3> vbo_data)
{
  const OffsetIndices faces = mr.faces;
  const Span<int3> corner_tris = mr.corner_tris;
  const Span<bool> hide_poly = mr.hide_poly;
  if (!mr.face_sorted->face_tri_offsets.has_value()) {
    if (hide_poly.is_empty()) {
      array_utils::copy(corner_tris.cast<uint3>(), vbo_data);
    }
    else {
      threading::parallel_for(corner_tris.index_range(), 2048, [&](const IndexRange range) {
        for (const int face : range) {
          const IndexRange tris = bke::mesh::face_triangles_range(faces, face);
          copy_tris_to_vbo(corner_tris, hide_poly[face], tris, tris, restart_value, vbo_data);
        }
      });
    }
  }
  else {
    if (hide_poly.is_empty()) {
      const Span<int> face_tri_offsets = mr.face_sorted->face_tri_offsets->as_span();
      threading::parallel_for(corner_tris.index_range(), 2048, [&](const IndexRange range) {
        for (const int face : range) {
          const IndexRange mesh_tris = bke::mesh::face_triangles_range(faces, face);
          copy_tris_to_vbo(corner_tris,
                           false,
                           mesh_tris,
                           IndexRange(face_tri_offsets[face], mesh_tris.size()),
                           restart_value,
                           vbo_data);
        }
      });
    }
    else {
      const Span<int> face_tri_offsets = mr.face_sorted->face_tri_offsets->as_span();
      threading::parallel_for(corner_tris.index_range(), 2048, [&](const IndexRange range) {
        for (const int face : range) {
          const IndexRange mesh_tris = bke::mesh::face_triangles_range(faces, face);
          copy_tris_to_vbo(corner_tris,
                           hide_poly[face],
                           mesh_tris,
                           IndexRange(face_tri_offsets[face], mesh_tris.size()),
                           restart_value,
                           vbo_data);
        }
      });
    }
  }
}

static void extract_tris_bmesh(const MeshRenderData &mr,
                               const uint32_t restart_value,
                               MutableSpan<uint3> vbo_data)
{
  BMesh &bm = *mr.bm;
  BMLoop *(*looptris)[3] = mr.edit_bmesh->looptris;
  const Span<int> face_tri_offsets = mr.face_sorted->face_tri_offsets ?
                                         mr.face_sorted->face_tri_offsets->as_span() :
                                         Span<int>();
  threading::parallel_for(IndexRange(bm.totface), 1024, [&](const IndexRange range) {
    for (const int face_index : range) {
      const BMFace &face = *BM_face_at_index(&bm, face_index);
      const int loop_index = BM_elem_index_get(BM_FACE_FIRST_LOOP(&face));
      const IndexRange tris(poly_to_tri_count(face_index, loop_index),
                            bke::mesh::face_triangles_num(face.len));

      const IndexRange vbo_tris(
          face_tri_offsets.is_empty() ? tris.start() : face_tri_offsets[face_index], tris.size());

      if (BM_elem_flag_test(&face, BM_ELEM_HIDDEN)) {
        vbo_data.slice(vbo_tris).fill(uint3(restart_value));
      }
      else {
        for (const int i : tris.index_range()) {
          vbo_data[vbo_tris[i]] = uint3(BM_elem_index_get(looptris[tris[i]][0]),
                                        BM_elem_index_get(looptris[tris[i]][1]),
                                        BM_elem_index_get(looptris[tris[i]][2]));
        }
      }
    }
  });
}

static void extract_tris_finish(const MeshRenderData &mr,
                                GPUIndexBufBuilder &builder,
                                MeshBatchCache &cache,
                                GPUIndexBuf &ibo)
{
  // TODO: Track if restart indices used.
  GPU_indexbuf_build_in_place_ex(&builder, 0, mr.loop_len, true, &ibo);

  /* Create ibo sub-ranges. Always do this to avoid error when the standard surface batch
   * is created before the surfaces-per-material. */
  if (mr.use_final_mesh && cache.tris_per_mat) {
    int mat_start = 0;
    for (int i = 0; i < mr.mat_len; i++) {
      /* These IBOs have not been queried yet but we create them just in case they are needed
       * later since they are not tracked by mesh_buffer_cache_create_requested(). */
      if (cache.tris_per_mat[i] == nullptr) {
        cache.tris_per_mat[i] = GPU_indexbuf_calloc();
      }
      const int mat_tri_len = mr.face_sorted->mat_tri_counts[i];
      /* Multiply by 3 because these are triangle indices. */
      const int start = mat_start * 3;
      const int len = mat_tri_len * 3;
      GPU_indexbuf_create_subrange_in_place(cache.tris_per_mat[i], &ibo, start, len);
      mat_start += mat_tri_len;
    }
  }
}

static void extract_tris_init(const MeshRenderData &mr,
                              MeshBatchCache &cache,
                              void *ibo_v,
                              void * /*tls_data*/)
{
  GPUIndexBuf &ibo = *static_cast<GPUIndexBuf *>(ibo_v);

  GPUIndexBufBuilder builder;
  GPU_indexbuf_init(&builder, GPU_PRIM_TRIS, mr.face_sorted->visible_tri_len, mr.loop_len);

  MutableSpan<uint32_t> index_data(GPU_indexbuf_get_data(&builder),
                                   mr.face_sorted->visible_tri_len * 3);
  const uint32_t restart_value = GPU_indexbuf_get_restart_value(&builder);

  if (mr.extract_type == MR_EXTRACT_MESH) {
    extract_tris_mesh(mr, restart_value, index_data.cast<uint3>());
  }
  else {
    extract_tris_bmesh(mr, restart_value, index_data.cast<uint3>());
  }

  extract_tris_finish(mr, builder, cache, ibo);
}

static void extract_tris_init_subdiv(const DRWSubdivCache &subdiv_cache,
                                     const MeshRenderData & /*mr*/,
                                     MeshBatchCache &cache,
                                     void *buffer,
                                     void * /*data*/)
{
  GPUIndexBuf *ibo = static_cast<GPUIndexBuf *>(buffer);
  /* Initialize the index buffer, it was already allocated, it will be filled on the device. */
  GPU_indexbuf_init_build_on_device(ibo, subdiv_cache.num_subdiv_triangles * 3);

  if (cache.tris_per_mat) {
    for (int i = 0; i < cache.mat_len; i++) {
      if (cache.tris_per_mat[i] == nullptr) {
        cache.tris_per_mat[i] = GPU_indexbuf_calloc();
      }

      /* Multiply by 6 since we have 2 triangles per quad. */
      const int start = subdiv_cache.mat_start[i] * 6;
      const int len = (subdiv_cache.mat_end[i] - subdiv_cache.mat_start[i]) * 6;
      GPU_indexbuf_create_subrange_in_place(cache.tris_per_mat[i], ibo, start, len);
    }
  }

  draw_subdiv_build_tris_buffer(subdiv_cache, ibo, cache.mat_len);
}

constexpr MeshExtract create_extractor_tris()
{
  MeshExtract extractor = {nullptr};
  extractor.init = extract_tris_init;
  extractor.init_subdiv = extract_tris_init_subdiv;
  extractor.data_type = MR_DATA_CORNER_TRI | MR_DATA_POLYS_SORTED;
  extractor.use_threading = true;
  extractor.mesh_buffer_offset = offsetof(MeshBufferList, ibo.tris);
  return extractor;
}

/** \} */

const MeshExtract extract_tris = create_extractor_tris();

}  // namespace blender::draw
