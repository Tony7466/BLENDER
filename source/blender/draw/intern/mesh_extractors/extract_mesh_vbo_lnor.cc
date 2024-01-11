/* SPDX-FileCopyrightText: 2021 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw
 */

#include "BLI_array_utils.hh"

#include "extract_mesh.hh"

#include "draw_subdivision.hh"

namespace blender::draw {

/* ---------------------------------------------------------------------- */
/** \name Extract Loop Normal
 * \{ */

static void compress_normals(const Span<float3> src, MutableSpan<GPUPackedNormal> dst)
{
  threading::parallel_for(src.index_range(), 2048, [&](const IndexRange range) {
    for (const int vert : range) {
      dst[vert] = GPU_normal_convert_i10_v3(src[vert]);
    }
  });
}

static void extract_vert_normals(const MeshRenderData &mr, MutableSpan<GPUPackedNormal> normals)
{
  const Span<float3> vert_normals = mr.vert_normals;
  Array<GPUPackedNormal> vert_normals_converted(vert_normals.size());
  compress_normals(vert_normals, vert_normals_converted);
  array_utils::gather(vert_normals_converted.as_span(), mr.corner_verts, normals);
}

static void extract_face_normals(const MeshRenderData &mr, MutableSpan<GPUPackedNormal> normals)
{
  const OffsetIndices faces = mr.faces;
  const Span<float3> face_normals = mr.face_normals;
  threading::parallel_for(faces.index_range(), 4096, [&](const IndexRange range) {
    for (const int face : range) {
      normals.slice(faces[face]).fill(GPU_normal_convert_i10_v3(face_normals[face]));
    }
  });
}

static void extract_normals_mesh(const MeshRenderData &mr, MutableSpan<GPUPackedNormal> normals)
{
  if (mr.normals_domain == bke::MeshNormalDomain::Face) {
    extract_face_normals(mr, normals);
  }
  else if (mr.normals_domain == bke::MeshNormalDomain::Point) {
    extract_vert_normals(mr, normals);
  }
  else if (!mr.loop_normals.is_empty()) {
    compress_normals(mr.loop_normals, normals);
  }
  else if (mr.sharp_faces.is_empty()) {
    extract_vert_normals(mr, normals);
  }
  else {
    const OffsetIndices faces = mr.faces;
    const Span<int> corner_verts = mr.corner_verts;
    const Span<bool> sharp_faces = mr.sharp_faces;
    const Span<float3> vert_normals = mr.vert_normals;
    const Span<float3> face_normals = mr.face_normals;
    threading::parallel_for(faces.index_range(), 2048, [&](const IndexRange range) {
      for (const int face : range) {
        if (sharp_faces[face]) {
          normals.slice(faces[face]).fill(GPU_normal_convert_i10_v3(face_normals[face]));
        }
        else {
          for (const int corner : faces[face]) {
            normals[corner] = GPU_normal_convert_i10_v3(vert_normals[corner_verts[corner]]);
          }
        }
      }
    });
  }
}

static void extract_paint_overlay_flags(const MeshRenderData &mr,
                                        MutableSpan<GPUPackedNormal> normals)
{
  if (mr.select_poly.is_empty() && mr.hide_poly.is_empty() && (!mr.edit_bmesh || !mr.v_origindex))
  {
    return;
  }
  const OffsetIndices faces = mr.faces;
  threading::parallel_for(faces.index_range(), 1024, [&](const IndexRange range) {
    if (!mr.select_poly.is_empty()) {
      const Span<bool> select_poly = mr.select_poly;
      for (const int face : range) {
        if (select_poly[face]) {
          for (const int corner : faces[face]) {
            normals[corner].w = 1;
          }
        }
      }
    }
    if (!mr.hide_poly.is_empty()) {
      const Span<bool> hide_poly = mr.hide_poly;
      for (const int face : range) {
        if (hide_poly[face]) {
          for (const int corner : faces[face]) {
            normals[corner].w = -1;
          }
        }
      }
    }
    if (mr.edit_bmesh && mr.v_origindex) {
      const Span<int> corner_verts = mr.corner_verts;
      const Span<int> orig_indices(mr.v_origindex, mr.vert_len);
      for (const int face : range) {
        for (const int corner : faces[face]) {
          if (orig_indices[corner_verts[corner]] == ORIGINDEX_NONE) {
            normals[corner].w = -1;
          }
        }
      }
    }
  });
}

static void extract_lnor_init(const MeshRenderData &mr,
                              MeshBatchCache & /*cache*/,
                              void *buf,
                              void * /*tls_data*/)
{
  GPUVertBuf *vbo = static_cast<GPUVertBuf *>(buf);
  static GPUVertFormat format = {0};
  if (format.attr_len == 0) {
    GPU_vertformat_attr_add(&format, "nor", GPU_COMP_I10, 4, GPU_FETCH_INT_TO_FLOAT_UNIT);
    GPU_vertformat_alias_add(&format, "lnor");
  }
  GPU_vertbuf_init_with_format(vbo, &format);
  GPU_vertbuf_data_alloc(vbo, mr.loop_len);

  MutableSpan vbo_data(static_cast<GPUPackedNormal *>(GPU_vertbuf_get_data(vbo)),
                       GPU_vertbuf_get_vertex_len(vbo));
  if (mr.extract_type == MR_EXTRACT_MESH) {
    extract_normals_mesh(mr, vbo_data);
    extract_paint_overlay_flags(mr, vbo_data);
  }
}

static void extract_lnor_iter_face_bm(const MeshRenderData &mr,
                                      const BMFace *f,
                                      const int /*f_index*/,
                                      void *data)
{
  BMLoop *l_iter, *l_first;
  l_iter = l_first = BM_FACE_FIRST_LOOP(f);
  do {
    const int l_index = BM_elem_index_get(l_iter);
    if (!mr.loop_normals.is_empty()) {
      (*(GPUPackedNormal **)data)[l_index] = GPU_normal_convert_i10_v3(mr.loop_normals[l_index]);
    }
    else {
      if (BM_elem_flag_test(f, BM_ELEM_SMOOTH)) {
        (*(GPUPackedNormal **)data)[l_index] = GPU_normal_convert_i10_v3(
            bm_vert_no_get(mr, l_iter->v));
      }
      else {
        (*(GPUPackedNormal **)data)[l_index] = GPU_normal_convert_i10_v3(bm_face_no_get(mr, f));
      }
    }
    (*(GPUPackedNormal **)data)[l_index].w = BM_elem_flag_test(f, BM_ELEM_HIDDEN) ? -1 : 0;
  } while ((l_iter = l_iter->next) != l_first);
}

static GPUVertFormat *get_subdiv_lnor_format()
{
  static GPUVertFormat format = {0};
  if (format.attr_len == 0) {
    GPU_vertformat_attr_add(&format, "nor", GPU_COMP_F32, 4, GPU_FETCH_FLOAT);
    GPU_vertformat_alias_add(&format, "lnor");
  }
  return &format;
}

static void extract_lnor_init_subdiv(const DRWSubdivCache &subdiv_cache,
                                     const MeshRenderData & /*mr*/,
                                     MeshBatchCache &cache,
                                     void *buffer,
                                     void * /*data*/)
{
  GPUVertBuf *vbo = static_cast<GPUVertBuf *>(buffer);
  GPUVertBuf *pos_nor = cache.final.buff.vbo.pos;
  BLI_assert(pos_nor);
  GPU_vertbuf_init_build_on_device(vbo, get_subdiv_lnor_format(), subdiv_cache.num_subdiv_loops);
  draw_subdiv_build_lnor_buffer(subdiv_cache, pos_nor, vbo);
}

constexpr MeshExtract create_extractor_lnor()
{
  MeshExtract extractor = {nullptr};
  extractor.init = extract_lnor_init;
  extractor.init_subdiv = extract_lnor_init_subdiv;
  extractor.iter_face_bm = extract_lnor_iter_face_bm;
  extractor.data_type = MR_DATA_LOOP_NOR;
  extractor.data_size = sizeof(GPUPackedNormal *);
  extractor.use_threading = true;
  extractor.mesh_buffer_offset = offsetof(MeshBufferList, vbo.nor);
  return extractor;
}

/** \} */

/* ---------------------------------------------------------------------- */
/** \name Extract HQ Loop Normal
 * \{ */

static void extract_lnor_hq_init(const MeshRenderData &mr,
                                 MeshBatchCache & /*cache*/,
                                 void *buf,
                                 void *tls_data)
{
  GPUVertBuf *vbo = static_cast<GPUVertBuf *>(buf);
  static GPUVertFormat format = {0};
  if (format.attr_len == 0) {
    GPU_vertformat_attr_add(&format, "nor", GPU_COMP_I16, 4, GPU_FETCH_INT_TO_FLOAT_UNIT);
    GPU_vertformat_alias_add(&format, "lnor");
  }
  GPU_vertbuf_init_with_format(vbo, &format);
  GPU_vertbuf_data_alloc(vbo, mr.loop_len);

  *(short4 **)tls_data = static_cast<short4 *>(GPU_vertbuf_get_data(vbo));
}

static void extract_lnor_hq_iter_face_bm(const MeshRenderData &mr,
                                         const BMFace *f,
                                         const int /*f_index*/,
                                         void *data)
{
  BMLoop *l_iter, *l_first;
  l_iter = l_first = BM_FACE_FIRST_LOOP(f);
  do {
    const int l_index = BM_elem_index_get(l_iter);
    if (!mr.loop_normals.is_empty()) {
      normal_float_to_short_v3(&(*(short4 **)data)[l_index].x, mr.loop_normals[l_index]);
    }
    else {
      if (BM_elem_flag_test(f, BM_ELEM_SMOOTH)) {
        normal_float_to_short_v3(&(*(short4 **)data)[l_index].x, bm_vert_no_get(mr, l_iter->v));
      }
      else {
        normal_float_to_short_v3(&(*(short4 **)data)[l_index].x, bm_face_no_get(mr, f));
      }
    }
  } while ((l_iter = l_iter->next) != l_first);
}

static void extract_lnor_hq_iter_face_mesh(const MeshRenderData &mr,
                                           const int face_index,
                                           void *data)
{
  const bool hidden = !mr.hide_poly.is_empty() && mr.hide_poly[face_index];

  for (const int corner : mr.faces[face_index]) {
    const int vert = mr.corner_verts[corner];
    short4 *lnor_data = &(*(short4 **)data)[corner];
    if (!mr.loop_normals.is_empty()) {
      normal_float_to_short_v3(&lnor_data->x, mr.loop_normals[corner]);
    }
    else if (mr.normals_domain == bke::MeshNormalDomain::Face ||
             (!mr.sharp_faces.is_empty() && mr.sharp_faces[face_index]))
    {
      normal_float_to_short_v3(&lnor_data->x, mr.face_normals[face_index]);
    }
    else {
      normal_float_to_short_v3(&lnor_data->x, mr.vert_normals[vert]);
    }

    /* Flag for paint mode overlay.
     * Only use origindex in edit mode where it is used to display the edge-normals.
     * In paint mode it will use the un-mapped data to draw the wire-frame. */
    if (hidden || (mr.edit_bmesh && (mr.v_origindex) && mr.v_origindex[vert] == ORIGINDEX_NONE)) {
      lnor_data->w = -1;
    }
    else if (!mr.select_poly.is_empty() && mr.select_poly[face_index]) {
      lnor_data->w = 1;
    }
    else {
      lnor_data->w = 0;
    }
  }
}

constexpr MeshExtract create_extractor_lnor_hq()
{
  MeshExtract extractor = {nullptr};
  extractor.init = extract_lnor_hq_init;
  extractor.init_subdiv = extract_lnor_init_subdiv;
  extractor.iter_face_bm = extract_lnor_hq_iter_face_bm;
  extractor.iter_face_mesh = extract_lnor_hq_iter_face_mesh;
  extractor.data_type = MR_DATA_LOOP_NOR;
  extractor.data_size = sizeof(short4 *);
  extractor.use_threading = true;
  extractor.mesh_buffer_offset = offsetof(MeshBufferList, vbo.nor);
  return extractor;
}

/** \} */

const MeshExtract extract_nor = create_extractor_lnor();
const MeshExtract extract_nor_hq = create_extractor_lnor_hq();

}  // namespace blender::draw
