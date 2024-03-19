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
                                    const IndexRange ibo_tris,
                                    const uint32_t restart_value,
                                    MutableSpan<uint3> data)
{
  if (hidden) {
    data.slice(ibo_tris).cast<uint32_t>().fill(restart_value);
  }
  else {
    data.slice(ibo_tris).copy_from(corner_tris.slice(mesh_tris).cast<uint3>());
  }
}

static bool use_corner_tris_directly(const MeshRenderData &mr)
{
  if (mr.face_sorted->face_tri_offsets.has_value()) {
    /* Faces are uploaded sorted into contiguous chunks of matching
     * materials which may not match the original face order. */
    return false;
  }
  if (!mr.hide_poly.is_empty()) {
    /* Values for hidden faces must be changed. */
    return false;
  }
  return true;
}

static void extract_tris_mesh(const MeshRenderData &mr, GPUIndexBuf &ibo)
{
  const OffsetIndices faces = mr.faces;
  const Span<int3> corner_tris = mr.corner_tris;

  if (use_corner_tris_directly(mr)) {
    /* There are no hidden faces and no reordering is necessary to group faces with the same
     * material. The corner indices from #Mesh::corner_tris() can be copied directly to the GPU. */
    GPU_indexbuf_build_in_place_from_memory(&ibo,
                                            GPU_PRIM_TRIS,
                                            corner_tris.cast<uint32_t>().data(),
                                            mr.face_sorted->visible_tris_num,
                                            0,
                                            mr.corners_num,
                                            false);
    return;
  }

  const Span<bool> hide_poly = mr.hide_poly;

  GPUIndexBufBuilder builder;
  GPU_indexbuf_init(&builder, GPU_PRIM_TRIS, mr.face_sorted->visible_tris_num, mr.corners_num);
  MutableSpan<uint3> data = GPU_indexbuf_get_data(&builder).cast<uint3>();
  const uint32_t restart_value = GPU_indexbuf_get_restart_value(&builder);

  if (!mr.face_sorted->face_tri_offsets.has_value()) {
    /* No reordering for grouping faces with the same material, but there are hidden faces. */
    threading::parallel_for(corner_tris.index_range(), 2048, [&](const IndexRange range) {
      for (const int face : range) {
        const IndexRange tris = bke::mesh::face_triangles_range(faces, face);
        copy_tris_to_vbo(corner_tris, hide_poly[face], tris, tris, restart_value, data);
      }
    });
    return;
  }

  const Span<int> face_tri_offsets = mr.face_sorted->face_tri_offsets->as_span();
  threading::parallel_for(corner_tris.index_range(), 2048, [&](const IndexRange range) {
    for (const int face : range) {
      const IndexRange mesh_tris = bke::mesh::face_triangles_range(faces, face);
      copy_tris_to_vbo(corner_tris,
                       hide_poly.is_empty() ? false : hide_poly[face],
                       mesh_tris,
                       IndexRange(face_tri_offsets[face], mesh_tris.size()),
                       restart_value,
                       data);
    }
  });

  // TODO: Track if restart indices used.
  GPU_indexbuf_build_in_place_ex(&builder, 0, mr.corners_num, true, &ibo);
}

static void extract_tris_bmesh(const MeshRenderData &mr, GPUIndexBuf &ibo)
{
  GPUIndexBufBuilder builder;
  GPU_indexbuf_init(&builder, GPU_PRIM_TRIS, mr.face_sorted->visible_tris_num, mr.corners_num);
  MutableSpan<uint3> data = GPU_indexbuf_get_data(&builder).cast<uint3>();
  const uint32_t restart_value = GPU_indexbuf_get_restart_value(&builder);

  BMesh &bm = *mr.bm;
  BMLoop *(*looptris)[3] = mr.edit_bmesh->looptris;
  const Span<int> face_tri_offsets = mr.face_sorted->face_tri_offsets ?
                                         mr.face_sorted->face_tri_offsets->as_span() :
                                         Span<int>();
  threading::parallel_for(IndexRange(bm.totface), 1024, [&](const IndexRange range) {
    for (const int face_index : range) {
      const BMFace &face = *BM_face_at_index(&bm, face_index);
      const int loop_index = BM_elem_index_get(BM_FACE_FIRST_LOOP(&face));
      const IndexRange bm_tris(poly_to_tri_count(face_index, loop_index),
                               bke::mesh::face_triangles_num(face.len));

      const IndexRange ibo_tris(face_tri_offsets.is_empty() ? bm_tris.start() :
                                                              face_tri_offsets[face_index],
                                bm_tris.size());

      if (BM_elem_flag_test(&face, BM_ELEM_HIDDEN)) {
        data.slice(ibo_tris).fill(uint3(restart_value));
      }
      else {
        for (const int i : bm_tris.index_range()) {
          data[ibo_tris[i]] = uint3(BM_elem_index_get(looptris[bm_tris[i]][0]),
                                    BM_elem_index_get(looptris[bm_tris[i]][1]),
                                    BM_elem_index_get(looptris[bm_tris[i]][2]));
        }
      }
    }
  });

  // TODO: Track if restart indices used.
  GPU_indexbuf_build_in_place_ex(&builder, 0, mr.corners_num, true, &ibo);
}

static void extract_tris_finish(const MeshRenderData &mr, MeshBatchCache &cache, GPUIndexBuf &ibo)
{
  /* Create ibo sub-ranges. Always do this to avoid error when the standard surface batch
   * is created before the surfaces-per-material. */
  if (mr.use_final_mesh && cache.tris_per_mat) {
    int mat_start = 0;
    for (int i = 0; i < mr.materials_num; i++) {
      /* These IBOs have not been queried yet but we create them just in case they are needed
       * later since they are not tracked by mesh_buffer_cache_create_requested(). */
      if (cache.tris_per_mat[i] == nullptr) {
        cache.tris_per_mat[i] = GPU_indexbuf_calloc();
      }
      const int mat_tri_len = mr.face_sorted->tris_num_by_material[i];
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

  if (mr.extract_type == MR_EXTRACT_MESH) {
    extract_tris_mesh(mr, ibo);
  }
  else {
    extract_tris_bmesh(mr, ibo);
  }

  extract_tris_finish(mr, cache, ibo);
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
