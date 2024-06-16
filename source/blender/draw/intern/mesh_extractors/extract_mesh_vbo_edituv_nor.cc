/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw
 */

#include "BKE_attribute.hh"
#include "BKE_mesh.hh"

#include "BKE_editmesh.hh"

#include "extract_mesh.hh"

#include "draw_subdivision.hh"

namespace blender::draw {

/* ---------------------------------------------------------------------- */
/** \name Extract Edit UV normals
 * \{ */

static void extract_edituv_uv_nor_bm(const MeshRenderData &mr, MutableSpan<float3> vbo_data)
{
  const BMesh &bm = *mr.bm;
  const int uv_offset = CustomData_get_offset(&bm.ldata, CD_PROP_FLOAT2);

  const BMFace *face;
  BMIter f_iter;
  BM_ITER_MESH (face, &f_iter, &const_cast<BMesh &>(bm), BM_FACES_OF_MESH) {
    for (int i = 0; i < face->len; i++) {
      vbo_data[BM_elem_index_get(face->l_first) + i] = {0.0f, 0.0f, -1.0f};
    }
    const Span<std::array<BMLoop *, 3>> looptris = mr.edit_bmesh->looptris;
    for (std::array<BMLoop *, 3> elt : looptris) {
      const float2 luv_0 = BM_ELEM_CD_GET_FLOAT2_P(elt[0], uv_offset);
      const float2 luv_1 = BM_ELEM_CD_GET_FLOAT2_P(elt[1], uv_offset);
      const float2 luv_2 = BM_ELEM_CD_GET_FLOAT2_P(elt[2], uv_offset);

      const float3 e1 = {luv_0[0] - luv_1[0], luv_0[1] - luv_1[1], 0.0f};
      const float3 e2 = {luv_2[0] - luv_0[0], luv_2[1] - luv_0[1], 0.0f};

      float3 normal = {e1[1] * e2[2] - e1[2] * e2[1],
                       e1[2] * e2[0] - e1[0] * e2[2],
                       e1[0] * e2[1] - e1[1] * e2[0]};
      normalize_v3(normal);
      if (normal.z > 0.0f) {
        vbo_data[BM_elem_index_get(elt[0])] = normal;
        vbo_data[BM_elem_index_get(elt[1])] = normal;
        vbo_data[BM_elem_index_get(elt[2])] = normal;
      }
    }
  }
}

static void extract_edituv_uv_nor_mesh(const MeshRenderData &mr, MutableSpan<float3> vbo_data)
{
  const OffsetIndices faces = mr.faces;
  const Mesh &mesh = *mr.mesh;
  const bke::AttributeAccessor attributes = mesh.attributes();
  const StringRef name = CustomData_get_active_layer_name(&mesh.corner_data, CD_PROP_FLOAT2);
  const VArraySpan uv_map = *attributes.lookup<float2>(name, bke::AttrDomain::Corner);

  for (const int face_index : faces.index_range()) {
    const IndexRange face = faces[face_index];
    const int corner_end = face.start() + face.size();
    for (int i = face.start(); i < corner_end; i++) {
      vbo_data[i] = {0.0f, 0.0f, -1.0f};
    }

    int tri_first_index_real = poly_to_tri_count(face_index, face.start());
    int tri_len = face.size() - 2;
    const Span<int3> corner_tris = mesh.corner_tris();
    for (int offs = 0; offs < tri_len; offs++) {
      const int3 &tri = corner_tris[tri_first_index_real + offs];
      const float2 luv_0 = uv_map[tri[0]];
      const float2 luv_1 = uv_map[tri[1]];
      const float2 luv_2 = uv_map[tri[2]];

      const float3 e1 = {luv_0[0] - luv_1[0], luv_0[1] - luv_1[1], 0.0f};
      const float3 e2 = {luv_2[0] - luv_0[0], luv_2[1] - luv_0[1], 0.0f};

      float3 normal = {e1[1] * e2[2] - e1[2] * e2[1],
                       e1[2] * e2[0] - e1[0] * e2[2],
                       e1[0] * e2[1] - e1[1] * e2[0]};
      normalize_v3(normal);
      if (normal.z > 0.0f) {
        vbo_data[tri[0]] = normal;
        vbo_data[tri[1]] = normal;
        vbo_data[tri[2]] = normal;
      }
    }
  }
}

void extract_edituv_nor(const MeshRenderData &mr, gpu::VertBuf &vbo)
{
  static GPUVertFormat format = {0};
  if (format.attr_len == 0) {
    GPU_vertformat_attr_add(&format, "uv_nor", GPU_COMP_F32, 3, GPU_FETCH_FLOAT);
  }

  GPU_vertbuf_init_with_format(vbo, format);
  GPU_vertbuf_data_alloc(vbo, mr.corners_num);

  MutableSpan vbo_data(static_cast<float3 *>(GPU_vertbuf_get_data(vbo)), mr.corners_num);

  if (mr.extract_type == MR_EXTRACT_BMESH) {
    extract_edituv_uv_nor_bm(mr, vbo_data);
  }
  else {
    extract_edituv_uv_nor_mesh(mr, vbo_data);
  }
}

static GPUVertFormat &get_edituv_nor_subdiv()
{
  static GPUVertFormat format = {0};
  if (format.attr_len == 0) {
    GPU_vertformat_attr_add(&format, "uv_nor", GPU_COMP_F32, 3, GPU_FETCH_FLOAT);
  }
  return format;
}

void extract_edituv_nor_subdiv(const MeshRenderData &mr,
                               const DRWSubdivCache &subdiv_cache,
                               const MeshBatchCache &cache,
                               gpu::VertBuf &vbo)
{
  GPU_vertbuf_init_build_on_device(vbo, get_edituv_nor_subdiv(), subdiv_cache.num_subdiv_loops);

  gpu::VertBuf *uvs = cache.final.buff.vbo.uv;

  /* UVs are stored contiguously so we need to compute the offset in the UVs buffer for the
   * active UV layer. */
  const CustomData *cd_ldata = (mr.extract_type == MR_EXTRACT_MESH) ? &mr.mesh->corner_data :
                                                                      &mr.bm->ldata;

  uint32_t uv_layers = cache.cd_used.uv;
  /* HACK to fix #68857 */
  if (mr.extract_type == MR_EXTRACT_BMESH && cache.cd_used.edit_uv == 1) {
    int layer = CustomData_get_active_layer(cd_ldata, CD_PROP_FLOAT2);
    if (layer != -1 && !CustomData_layer_is_anonymous(cd_ldata, CD_PROP_FLOAT2, layer)) {
      uv_layers |= (1 << layer);
    }
  }

  int uvs_offset = 0;
  for (int i = 0; i < MAX_MTFACE; i++) {
    if (uv_layers & (1 << i)) {
      if (i == CustomData_get_active_layer(cd_ldata, CD_PROP_FLOAT2)) {
        break;
      }

      uvs_offset += 1;
    }
  }

  /* The data is at `offset * num loops`, and we have 2 values per index. */
  uvs_offset *= subdiv_cache.num_subdiv_loops * 2;

  draw_subdiv_build_edituv_orientation_buffer(subdiv_cache, uvs, uvs_offset, &vbo);
}

}  // namespace blender::draw
