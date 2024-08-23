/* SPDX-FileCopyrightText: 2005 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 *
 * bke::pbvh::Tree drawing.
 * Embeds GPU meshes inside of bke::pbvh::Tree nodes, used by mesh sculpt mode.
 */

#include <algorithm>
#include <climits>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "MEM_guardedalloc.h"

#include "BLI_bitmap.h"
#include "BLI_function_ref.hh"
#include "BLI_ghash.h"
#include "BLI_index_range.hh"
#include "BLI_map.hh"
#include "BLI_math_color.h"
#include "BLI_math_vector_types.hh"
#include "BLI_string.h"
#include "BLI_string_ref.hh"
#include "BLI_timeit.hh"
#include "BLI_utildefines.h"
#include "BLI_vector.hh"

#include "DNA_mesh_types.h"
#include "DNA_object_types.h"

#include "BKE_attribute.hh"
#include "BKE_attribute_math.hh"
#include "BKE_ccg.hh"
#include "BKE_customdata.hh"
#include "BKE_mesh.hh"
#include "BKE_paint.hh"
#include "BKE_pbvh_api.hh"
#include "BKE_subdiv_ccg.hh"

#include "DEG_depsgraph_query.hh"

#include "GPU_batch.hh"

#include "DRW_engine.hh"
#include "DRW_pbvh.hh"

#include "attribute_convert.hh"
#include "bmesh.hh"
#include "gpu_private.hh"

#define MAX_PBVH_BATCH_KEY 512
#define MAX_PBVH_VBOS 16

namespace blender::draw::pbvh {

struct OrigMeshData {
  StringRef active_color;
  StringRef default_color;
  StringRef active_uv_map;
  StringRef default_uv_map;
  int face_set_default;
  int face_set_seed;
  OrigMeshData(const Mesh &mesh)
      : active_color(mesh.active_color_attribute),
        default_color(mesh.default_color_attribute),
        active_uv_map(CustomData_get_active_layer_name(&mesh.corner_data, CD_PROP_FLOAT2)),
        default_uv_map(CustomData_get_render_layer_name(&mesh.corner_data, CD_PROP_FLOAT2))
  {
  }
};

static const GPUVertFormat &position_format()
{
  static GPUVertFormat format{};
  if (format.attr_len == 0) {
    GPU_vertformat_attr_add(&format, "pos", GPU_COMP_F32, 3, GPU_FETCH_FLOAT);
  }
  return format;
}

static const GPUVertFormat &position_format()
{
  static GPUVertFormat format{};
  if (format.attr_len == 0) {
    GPU_vertformat_attr_add(&format, "pos", GPU_COMP_F32, 3, GPU_FETCH_FLOAT);
  }
  return format;
}

static const GPUVertFormat &normal_format()
{
  static GPUVertFormat format{};
  if (format.attr_len == 0) {
    GPU_vertformat_attr_add(&format, "nor", GPU_COMP_I16, 3, GPU_FETCH_INT_TO_FLOAT_UNIT);
  }
  return format;
}

static const GPUVertFormat &mask_format()
{
  static GPUVertFormat format{};
  if (format.attr_len == 0) {
    GPU_vertformat_attr_add(&format, "msk", GPU_COMP_F32, 1, GPU_FETCH_FLOAT);
  }
  return format;
}

static const GPUVertFormat &face_set_format()
{
  static GPUVertFormat format{};
  if (format.attr_len == 0) {
    GPU_vertformat_attr_add(&format, "fset", GPU_COMP_U8, 3, GPU_FETCH_INT_TO_FLOAT_UNIT);
  }
  return format;
}

static GPUVertFormat attribute_format(const OrigMeshData &orig_mesh_data,
                                      const StringRefNull name,
                                      const eCustomDataType data_type)
{
  GPUVertFormat format = draw::init_format_for_attribute(data_type, "data");

  bool is_render, is_active;
  const char *prefix = "a";

  if (CD_TYPE_AS_MASK(data_type) & CD_MASK_COLOR_ALL) {
    prefix = "c";
    is_active = orig_mesh_data.active_color == name;
    is_render = orig_mesh_data.default_color == name;
  }
  if (data_type == CD_PROP_FLOAT2) {
    prefix = "u";
    is_active = orig_mesh_data.active_uv_map == name;
    is_render = orig_mesh_data.default_uv_map == name;
  }

  DRW_cdlayer_attr_aliases_add(&format, prefix, data_type, name.c_str(), is_render, is_active);
  return format;
}

static GPUVertFormat format_for_request(const OrigMeshData &orig_mesh_data,
                                        const AttributeRequest &request)
{
  if (const CustomRequest *request_type = std::get_if<CustomRequest>(&request)) {
    GPUVertFormat format;
    switch (*request_type) {
      case CustomRequest::Position:
        return position_format();
      case CustomRequest::Normal:
        return normal_format();
      case CustomRequest::Mask:
        return mask_format();
      case CustomRequest::FaceSet:
        return face_set_format();
    }
  }
  else {
    const GenericRequest &attr = std::get<GenericRequest>(request);
    return attribute_format(orig_mesh_data, attr.name, attr.type);
  }
  BLI_assert_unreachable();
  return {};
}

static bool pbvh_attr_supported(const AttributeRequest &request)
{
  if (std::holds_alternative<CustomRequest>(request)) {
    return true;
  }
  const GenericRequest &attr = std::get<GenericRequest>(request);
  if (!ELEM(attr.domain, bke::AttrDomain::Point, bke::AttrDomain::Face, bke::AttrDomain::Corner)) {
    /* blender::bke::pbvh::Tree drawing does not support edge domain attributes. */
    return false;
  }
  bool type_supported = false;
  bke::attribute_math::convert_to_static_type(attr.type, [&](auto dummy) {
    using T = decltype(dummy);
    using Converter = AttributeConverter<T>;
    using VBOType = typename Converter::VBOType;
    if constexpr (!std::is_void_v<VBOType>) {
      type_supported = true;
    }
  });
  return type_supported;
}

inline short4 normal_float_to_short(const float3 &value)
{
  short3 result;
  normal_float_to_short_v3(result, value);
  return short4(result.x, result.y, result.z, 0);
}

template<typename T>
void extract_data_vert_mesh(const Span<int> corner_verts,
                            const Span<int3> corner_tris,
                            const Span<int> tri_faces,
                            const Span<bool> hide_poly,
                            const Span<T> attribute,
                            const Span<int> tris,
                            gpu::VertBuf &vbo)
{
  using Converter = AttributeConverter<T>;
  using VBOType = typename Converter::VBOType;
  VBOType *data = vbo.data<VBOType>().data();
  for (const int tri : tris) {
    if (!hide_poly.is_empty() && hide_poly[tri_faces[tri]]) {
      continue;
    }
    for (int i : IndexRange(3)) {
      const int vert = corner_verts[corner_tris[tri][i]];
      *data = Converter::convert(attribute[vert]);
      data++;
    }
  }
}

template<typename T>
void extract_data_face_mesh(const Span<int> tri_faces,
                            const Span<bool> hide_poly,
                            const Span<T> attribute,
                            const Span<int> tris,
                            gpu::VertBuf &vbo)
{
  using Converter = AttributeConverter<T>;
  using VBOType = typename Converter::VBOType;

  VBOType *data = vbo.data<VBOType>().data();
  for (const int tri : tris) {
    const int face = tri_faces[tri];
    if (!hide_poly.is_empty() && hide_poly[face]) {
      continue;
    }
    std::fill_n(data, 3, Converter::convert(attribute[face]));
    data += 3;
  }
}

template<typename T>
void extract_data_corner_mesh(const Span<int3> corner_tris,
                              const Span<int> tri_faces,
                              const Span<bool> hide_poly,
                              const Span<T> attribute,
                              const Span<int> tris,
                              gpu::VertBuf &vbo)
{
  using Converter = AttributeConverter<T>;
  using VBOType = typename Converter::VBOType;

  VBOType *data = vbo.data<VBOType>().data();
  for (const int tri : tris) {
    if (!hide_poly.is_empty() && hide_poly[tri_faces[tri]]) {
      continue;
    }
    for (int i : IndexRange(3)) {
      const int corner = corner_tris[tri][i];
      *data = Converter::convert(attribute[corner]);
      data++;
    }
  }
}

template<typename T> const T &bmesh_cd_vert_get(const BMVert &vert, const int offset)
{
  return *static_cast<const T *>(POINTER_OFFSET(vert.head.data, offset));
}

template<typename T> const T &bmesh_cd_loop_get(const BMLoop &loop, const int offset)
{
  return *static_cast<const T *>(POINTER_OFFSET(loop.head.data, offset));
}

template<typename T> const T &bmesh_cd_face_get(const BMFace &face, const int offset)
{
  return *static_cast<const T *>(POINTER_OFFSET(face.head.data, offset));
}

template<typename T>
void extract_data_vert_bmesh(const Set<BMFace *, 0> &faces, const int cd_offset, gpu::VertBuf &vbo)
{
  using Converter = AttributeConverter<T>;
  using VBOType = typename Converter::VBOType;
  VBOType *data = vbo.data<VBOType>().data();

  for (const BMFace *face : faces) {
    if (BM_elem_flag_test(face, BM_ELEM_HIDDEN)) {
      continue;
    }
    const BMLoop *l = face->l_first;
    *data = Converter::convert(bmesh_cd_vert_get<T>(*l->prev->v, cd_offset));
    data++;
    *data = Converter::convert(bmesh_cd_vert_get<T>(*l->v, cd_offset));
    data++;
    *data = Converter::convert(bmesh_cd_vert_get<T>(*l->next->v, cd_offset));
    data++;
  }
}

template<typename T>
void extract_data_face_bmesh(const Set<BMFace *, 0> &faces, const int cd_offset, gpu::VertBuf &vbo)
{
  using Converter = AttributeConverter<T>;
  using VBOType = typename Converter::VBOType;
  VBOType *data = vbo.data<VBOType>().data();

  for (const BMFace *face : faces) {
    if (BM_elem_flag_test(face, BM_ELEM_HIDDEN)) {
      continue;
    }
    std::fill_n(data, 3, Converter::convert(bmesh_cd_face_get<T>(*face, cd_offset)));
    data += 3;
  }
}

template<typename T>
void extract_data_corner_bmesh(const Set<BMFace *, 0> &faces,
                               const int cd_offset,
                               gpu::VertBuf &vbo)
{
  using Converter = AttributeConverter<T>;
  using VBOType = typename Converter::VBOType;
  VBOType *data = vbo.data<VBOType>().data();

  for (const BMFace *face : faces) {
    if (BM_elem_flag_test(face, BM_ELEM_HIDDEN)) {
      continue;
    }
    const BMLoop *l = face->l_first;
    *data = Converter::convert(bmesh_cd_loop_get<T>(*l->prev, cd_offset));
    data++;
    *data = Converter::convert(bmesh_cd_loop_get<T>(*l, cd_offset));
    data++;
    *data = Converter::convert(bmesh_cd_loop_get<T>(*l->next, cd_offset));
    data++;
  }
}

static const CustomData *get_cdata(const BMesh &bm, const bke::AttrDomain domain)
{
  switch (domain) {
    case bke::AttrDomain::Point:
      return &bm.vdata;
    case bke::AttrDomain::Corner:
      return &bm.ldata;
    case bke::AttrDomain::Face:
      return &bm.pdata;
    default:
      return nullptr;
  }
}

template<typename T> T fallback_value_for_fill()
{
  return T();
}

template<> ColorGeometry4f fallback_value_for_fill()
{
  return ColorGeometry4f(1.0f, 1.0f, 1.0f, 1.0f);
}

template<> ColorGeometry4b fallback_value_for_fill()
{
  return fallback_value_for_fill<ColorGeometry4f>().encode();
}

static int count_visible_tris_mesh(const Span<int> tris,
                                   const Span<int> tri_faces,
                                   const Span<bool> hide_poly)
{
  if (hide_poly.is_empty()) {
    return tris.size();
  }
  return std::count_if(
      tris.begin(), tris.end(), [&](const int tri) { return !hide_poly[tri_faces[tri]]; });
}

static int count_visible_tris_bmesh(const Set<BMFace *, 0> &faces)
{
  return std::count_if(faces.begin(), faces.end(), [&](const BMFace *face) {
    return !BM_elem_flag_test_bool(face, BM_ELEM_HIDDEN);
  });
}

static void fill_vbo_normal_mesh(const Span<int> corner_verts,
                                 const Span<int3> corner_tris,
                                 const Span<int> tri_faces,
                                 const Span<bool> sharp_faces,
                                 const Span<bool> hide_poly,
                                 const Span<float3> vert_normals,
                                 const Span<float3> face_normals,
                                 const Span<int> tris,
                                 gpu::VertBuf &vert_buf)
{
  short4 *data = vert_buf.data<short4>().data();

  short4 face_no;
  int last_face = -1;
  for (const int tri : tris) {
    const int face = tri_faces[tri];
    if (!hide_poly.is_empty() && hide_poly[face]) {
      continue;
    }
    if (!sharp_faces.is_empty() && sharp_faces[face]) {
      if (face != last_face) {
        face_no = normal_float_to_short(face_normals[face]);
        last_face = face;
      }
      std::fill_n(data, 3, face_no);
      data += 3;
    }
    else {
      for (const int i : IndexRange(3)) {
        const int vert = corner_verts[corner_tris[tri][i]];
        *data = normal_float_to_short(vert_normals[vert]);
        data++;
      }
    }
  }
}

static void fill_vbo_mask_mesh(const Span<int> corner_verts,
                               const Span<int3> corner_tris,
                               const Span<int> tri_faces,
                               const Span<bool> hide_poly,
                               const Span<float> mask,
                               const Span<int> tris,
                               gpu::VertBuf &vbo)
{
  float *data = vbo.data<float>().data();
  for (const int tri : tris) {
    if (!hide_poly.is_empty() && hide_poly[tri_faces[tri]]) {
      continue;
    }
    for (int i : IndexRange(3)) {
      const int vert = corner_verts[corner_tris[tri][i]];
      *data = mask[vert];
      data++;
    }
  }
}

static void fill_vbo_face_set_mesh(const Span<int> tri_faces,
                                   const Span<bool> hide_poly,
                                   const Span<int> face_sets,
                                   const int color_default,
                                   const int color_seed,
                                   const Span<int> tris,
                                   gpu::VertBuf &vert_buf)
{
  uchar4 *data = vert_buf.data<uchar4>().data();
  int last_face = -1;
  uchar4 fset_color(UCHAR_MAX);
  for (const int tri : tris) {
    if (!hide_poly.is_empty() && hide_poly[tri_faces[tri]]) {
      continue;
    }
    const int face = tri_faces[tri];
    if (last_face != face) {
      last_face = face;

      const int id = face_sets[face];

      if (id != color_default) {
        BKE_paint_face_set_overlay_color_get(id, color_seed, fset_color);
      }
      else {
        /* Skip for the default color face set to render it white. */
        fset_color[0] = fset_color[1] = fset_color[2] = UCHAR_MAX;
      }
    }
    std::fill_n(data, 3, fset_color);
    data += 3;
  }
}

static void fill_vbo_attribute_mesh(const Span<int> corner_verts,
                                    const Span<int3> corner_tris,
                                    const Span<int> tri_faces,
                                    const Span<bool> hide_poly,
                                    const GSpan attribute,
                                    const bke::AttrDomain domain,
                                    const Span<int> tris,
                                    gpu::VertBuf &vert_buf)
{
  bke::attribute_math::convert_to_static_type(attribute.type(), [&](auto dummy) {
    using T = decltype(dummy);
    if constexpr (!std::is_void_v<typename AttributeConverter<T>::VBOType>) {
      switch (domain) {
        case bke::AttrDomain::Point:
          extract_data_vert_mesh<T>(corner_verts,
                                    corner_tris,
                                    tri_faces,
                                    hide_poly,
                                    attribute.typed<T>(),
                                    tris,
                                    vert_buf);
          break;
        case bke::AttrDomain::Face:
          extract_data_face_mesh<T>(tri_faces, hide_poly, attribute.typed<T>(), tris, vert_buf);
          break;
        case bke::AttrDomain::Corner:
          extract_data_corner_mesh<T>(
              corner_tris, tri_faces, hide_poly, attribute.typed<T>(), tris, vert_buf);
          break;
        default:
          BLI_assert_unreachable();
      }
    }
  });
}

static void fill_vbo_position_grids(const CCGKey &key,
                                    const Span<CCGElem *> grids,
                                    const bool use_flat_layout,
                                    const Span<int> grid_indices,
                                    gpu::VertBuf &vert_buf)
{
  float3 *data = vert_buf.data<float3>().data();
  if (use_flat_layout) {
    const int grid_size_1 = key.grid_size - 1;
    for (const int i : grid_indices.index_range()) {
      CCGElem *grid = grids[grid_indices[i]];
      for (int y = 0; y < grid_size_1; y++) {
        for (int x = 0; x < grid_size_1; x++) {
          *data = CCG_grid_elem_co(key, grid, x, y);
          data++;
          *data = CCG_grid_elem_co(key, grid, x + 1, y);
          data++;
          *data = CCG_grid_elem_co(key, grid, x + 1, y + 1);
          data++;
          *data = CCG_grid_elem_co(key, grid, x, y + 1);
          data++;
        }
      }
    }
  }
  else {
    for (const int i : grid_indices.index_range()) {
      CCGElem *grid = grids[grid_indices[i]];
      for (const int offset : IndexRange(key.grid_area)) {
        *data = CCG_elem_offset_co(key, grid, offset);
        data++;
      }
    }
  }
}

static void fill_vbo_normal_grids(const CCGKey &key,
                                  const Span<CCGElem *> grids,
                                  const Span<int> grid_to_face_map,
                                  const Span<bool> sharp_faces,
                                  const bool use_flat_layout,
                                  const Span<int> grid_indices,
                                  gpu::VertBuf &vert_buf)
{

  short4 *data = vert_buf.data<short4>().data();

  if (use_flat_layout) {
    const int grid_size_1 = key.grid_size - 1;
    for (const int i : grid_indices.index_range()) {
      const int grid_index = grid_indices[i];
      CCGElem *grid = grids[grid_index];
      if (!sharp_faces.is_empty() && sharp_faces[grid_to_face_map[grid_index]]) {
        for (int y = 0; y < grid_size_1; y++) {
          for (int x = 0; x < grid_size_1; x++) {
            float3 no;
            normal_quad_v3(no,
                           CCG_grid_elem_co(key, grid, x, y + 1),
                           CCG_grid_elem_co(key, grid, x + 1, y + 1),
                           CCG_grid_elem_co(key, grid, x + 1, y),
                           CCG_grid_elem_co(key, grid, x, y));
            std::fill_n(data, 4, normal_float_to_short(no));
            data += 4;
          }
        }
      }
      else {
        for (int y = 0; y < grid_size_1; y++) {
          for (int x = 0; x < grid_size_1; x++) {
            std::fill_n(data, 4, normal_float_to_short(CCG_grid_elem_no(key, grid, x, y)));
            data += 4;
          }
        }
      }
    }
  }
  else {
    /* The non-flat VBO layout does not support sharp faces. */
    for (const int i : grid_indices.index_range()) {
      CCGElem *grid = grids[grid_indices[i]];
      for (const int offset : IndexRange(key.grid_area)) {
        *data = normal_float_to_short(CCG_elem_offset_no(key, grid, offset));
        data++;
      }
    }
  }
}

static void fill_vbo_mask_grids(const CCGKey &key,
                                const Span<CCGElem *> grids,
                                const bool use_flat_layout,
                                const Span<int> grid_indices,
                                gpu::VertBuf &vert_buf)
{
  if (key.has_mask) {
    float *data = vert_buf.data<float>().data();
    if (use_flat_layout) {
      const int grid_size_1 = key.grid_size - 1;
      for (const int i : grid_indices.index_range()) {
        CCGElem *grid = grids[grid_indices[i]];
        for (int y = 0; y < grid_size_1; y++) {
          for (int x = 0; x < grid_size_1; x++) {
            *data = CCG_grid_elem_mask(key, grid, x, y);
            data++;
            *data = CCG_grid_elem_mask(key, grid, x + 1, y);
            data++;
            *data = CCG_grid_elem_mask(key, grid, x + 1, y + 1);
            data++;
            *data = CCG_grid_elem_mask(key, grid, x, y + 1);
            data++;
          }
        }
      }
    }
    else {
      for (const int i : grid_indices.index_range()) {
        CCGElem *grid = grids[grid_indices[i]];
        for (const int offset : IndexRange(key.grid_area)) {
          *data = CCG_elem_offset_mask(key, grid, offset);
          data++;
        }
      }
    }
  }
  else {
    vert_buf.data<float>().fill(0.0f);
  }
}

static void fill_vbo_face_set_grids(const CCGKey &key,
                                    const Span<int> grid_to_face_map,
                                    const Span<int> face_sets,
                                    const int color_default,
                                    const int color_seed,
                                    const bool use_flat_layout,
                                    const Span<int> grid_indices,
                                    gpu::VertBuf &vert_buf)
{
  const int verts_per_grid = use_flat_layout ? square_i(key.grid_size - 1) * 4 :
                                               square_i(key.grid_size);
  uchar4 *data = vert_buf.data<uchar4>().data();
  for (const int i : grid_indices.index_range()) {
    uchar4 color{UCHAR_MAX};
    const int fset = face_sets[grid_to_face_map[grid_indices[i]]];
    if (fset != color_default) {
      BKE_paint_face_set_overlay_color_get(fset, color_seed, color);
    }

    std::fill_n(data, verts_per_grid, color);
    data += verts_per_grid;
  }
}

static void fill_vbos_grids(const Object &object,
                            const OrigMeshData &orig_mesh_data,
                            const Span<bke::pbvh::Node> nodes,
                            const Span<bool> use_flat_layout,
                            const IndexMask &nodes_to_update,
                            const AttributeRequest &request,
                            const MutableSpan<gpu::VertBuf *> vbos)
{
  const SculptSession &ss = *object.sculpt;
  const SubdivCCG &subdiv_ccg = *ss.subdiv_ccg;
  const CCGKey key = BKE_subdiv_ccg_key_top_level(subdiv_ccg);
  const Span<CCGElem *> grids = subdiv_ccg.grids;

  if (const CustomRequest *request_type = std::get_if<CustomRequest>(&request)) {
    switch (*request_type) {
      case CustomRequest::Position: {
        nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {
          fill_vbo_position_grids(
              key, grids, use_flat_layout[i], bke::pbvh::node_grid_indices(nodes[i]), *vbos[i]);
        });
        break;
      }
      case CustomRequest::Normal: {
        const Mesh &mesh = *static_cast<const Mesh *>(object.data);
        const Span<int> grid_to_face_map = subdiv_ccg.grid_to_face_map;
        const bke::AttributeAccessor attributes = mesh.attributes();
        const VArraySpan sharp_faces = *attributes.lookup<bool>("sharp_face",
                                                                bke::AttrDomain::Face);
        nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {
          fill_vbo_normal_grids(key,
                                grids,
                                grid_to_face_map,
                                sharp_faces,
                                use_flat_layout[i],
                                bke::pbvh::node_grid_indices(nodes[i]),
                                *vbos[i]);
        });

        break;
      }
      case CustomRequest::Mask: {
        nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {
          fill_vbo_mask_grids(
              key, grids, use_flat_layout[i], bke::pbvh::node_grid_indices(nodes[i]), *vbos[i]);
        });
        break;
      }
      case CustomRequest::FaceSet: {
        const Mesh &mesh = *static_cast<const Mesh *>(object.data);
        const Span<int> grid_to_face_map = subdiv_ccg.grid_to_face_map;
        const bke::AttributeAccessor attributes = mesh.attributes();
        if (const VArray<int> face_sets = *attributes.lookup<int>(".sculpt_face_set",
                                                                  bke::AttrDomain::Face))
        {
          const VArraySpan<int> face_sets_span(face_sets);
          nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {
            fill_vbo_face_set_grids(key,
                                    grid_to_face_map,
                                    face_sets_span,
                                    orig_mesh_data.face_set_default,
                                    orig_mesh_data.face_set_seed,
                                    use_flat_layout[i],
                                    bke::pbvh::node_grid_indices(nodes[i]),
                                    *vbos[i]);
          });
        }
        else {
          nodes_to_update.foreach_index(
              GrainSize(1), [&](const int i) { vbos[i]->data<uchar4>().fill(uchar4{UCHAR_MAX}); });
        }
        break;
      }
    }
  }
  else {
    const eCustomDataType type = std::get<GenericRequest>(request).type;
    nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {
      bke::attribute_math::convert_to_static_type(type, [&](auto dummy) {
        using T = decltype(dummy);
        using Converter = AttributeConverter<T>;
        using VBOType = typename Converter::VBOType;
        if constexpr (!std::is_void_v<VBOType>) {
          vbo.vert_buf->data<VBOType>().fill(Converter::convert(fallback_value_for_fill<T>()));
        }
      });
    });
  }
}

static void fill_vbos_mesh(const Object &object,
                           const OrigMeshData &orig_mesh_data,
                           const Span<bke::pbvh::Node> nodes,
                           const IndexMask &nodes_to_update,
                           const AttributeRequest &request,
                           const MutableSpan<gpu::VertBuf *> vbos)
{
  const Mesh &mesh = *static_cast<const Mesh *>(object.data);
  const Span<int> corner_verts = mesh.corner_verts();
  const Span<int3> corner_tris = mesh.corner_tris();
  const Span<int> tri_faces = mesh.corner_tri_faces();
  const bke::AttributeAccessor attributes = mesh.attributes();
  const VArraySpan hide_poly = *attributes.lookup<bool>(".hide_poly", bke::AttrDomain::Face);

  if (const CustomRequest *request_type = std::get_if<CustomRequest>(&request)) {
    switch (*request_type) {
      case CustomRequest::Position: {
        const Span<float3> vert_positions = bke::pbvh::vert_positions_eval_from_eval(object);
        nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {
          extract_data_vert_mesh<float3>(corner_verts,
                                         corner_tris,
                                         tri_faces,
                                         hide_poly,
                                         vert_positions,
                                         bke::pbvh::node_tri_indices(nodes[i]),
                                         *vbos[i]);
        });
        break;
      }
      case CustomRequest::Normal: {
        const Span<float3> vert_normals = bke::pbvh::vert_normals_eval_from_eval(object);
        const Span<float3> face_normals = bke::pbvh::face_normals_eval_from_eval(object);
        const VArraySpan sharp_faces = *attributes.lookup<bool>("sharp_face",
                                                                bke::AttrDomain::Face);
        nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {
          fill_vbo_normal_mesh(corner_verts,
                               corner_tris,
                               tri_faces,
                               sharp_faces,
                               hide_poly,
                               vert_normals,
                               face_normals,
                               bke::pbvh::node_tri_indices(nodes[i]),
                               *vbos[i]);
        });
        break;
      }
      case CustomRequest::Mask: {
        const VArraySpan mask = *attributes.lookup<float>(".sculpt_mask", bke::AttrDomain::Point);
        if (!mask.is_empty()) {
          nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {
            extract_data_vert_mesh<float>(corner_verts,
                                          corner_tris,
                                          tri_faces,
                                          hide_poly,
                                          mask,
                                          bke::pbvh::node_tri_indices(nodes[i]),
                                          *vbos[i]);
          });
        }
        else {
          nodes_to_update.foreach_index(GrainSize(16),
                                        [&](const int i) { vbos[i]->data<float>().fill(0.0f); });
        }
        break;
      }
      case CustomRequest::FaceSet: {
        const VArraySpan face_sets = *attributes.lookup<int>(".sculpt_face_set",
                                                             bke::AttrDomain::Face);
        if (!face_sets.is_empty()) {
          nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {
            fill_vbo_face_set_mesh(tri_faces,
                                   hide_poly,
                                   face_sets,
                                   orig_mesh_data.face_set_default,
                                   orig_mesh_data.face_set_seed,
                                   bke::pbvh::node_tri_indices(nodes[i]),
                                   *vbos[i]);
          });
        }
        else {
          nodes_to_update.foreach_index(
              GrainSize(16), [&](const int i) { vbos[i]->data<uchar4>().fill(uchar4(255)); });
        }
        break;
      }
    }
  }
  else {
    const GenericRequest &attr = std::get<GenericRequest>(request);
    const StringRef name = attr.name;
    const bke::AttrDomain domain = attr.domain;
    const eCustomDataType data_type = attr.type;
    const GVArraySpan attribute = *attributes.lookup_or_default(name, domain, data_type);
    nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {
      fill_vbo_attribute_mesh(corner_verts,
                              corner_tris,
                              tri_faces,
                              hide_poly,
                              attribute,
                              domain,
                              bke::pbvh::node_tri_indices(nodes[i]),
                              *vbos[i]);
    });
  }
}

static void fill_vbo_position_bmesh(const Set<BMFace *, 0> &faces, gpu::VertBuf &vbo)
{
  float3 *data = vbo.data<float3>().data();
  for (const BMFace *face : faces) {
    if (BM_elem_flag_test(face, BM_ELEM_HIDDEN)) {
      continue;
    }
    const BMLoop *l = face->l_first;
    *data = l->prev->v->co;
    data++;
    *data = l->v->co;
    data++;
    *data = l->next->v->co;
    data++;
  }
}

static void fill_vbo_normal_bmesh(const Set<BMFace *, 0> &faces, gpu::VertBuf &vbo)
{
  short4 *data = vbo.data<short4>().data();
  for (const BMFace *face : faces) {
    if (BM_elem_flag_test(face, BM_ELEM_HIDDEN)) {
      continue;
    }
    if (BM_elem_flag_test(face, BM_ELEM_SMOOTH)) {
      const BMLoop *l = face->l_first;
      *data = normal_float_to_short(l->prev->v->no);
      data++;
      *data = normal_float_to_short(l->v->no);
      data++;
      *data = normal_float_to_short(l->next->v->no);
      data++;
    }
    else {
      std::fill_n(data, 3, normal_float_to_short(face->no));
      data += 3;
    }
  }
}

static void fill_vbo_mask_bmesh(const Set<BMFace *, 0> &faces,
                                const int cd_offset,
                                gpu::VertBuf &vbo)
{
  float *data = vbo.data<float>().data();
  for (const BMFace *face : faces) {
    if (BM_elem_flag_test(face, BM_ELEM_HIDDEN)) {
      continue;
    }
    const BMLoop *l = face->l_first;
    *data = bmesh_cd_vert_get<float>(*l->prev->v, cd_offset);
    data++;
    *data = bmesh_cd_vert_get<float>(*l->v, cd_offset);
    data++;
    *data = bmesh_cd_vert_get<float>(*l->next->v, cd_offset);
    data++;
  }
}

static void fill_vbo_face_set_bmesh(const Set<BMFace *, 0> &faces,
                                    const int color_default,
                                    const int color_seed,
                                    const int offset,
                                    gpu::VertBuf &vbo)
{
  uchar4 *data = vbo.data<uchar4>().data();
  for (const BMFace *face : faces) {
    if (BM_elem_flag_test(face, BM_ELEM_HIDDEN)) {
      continue;
    }
    uchar4 color{UCHAR_MAX};
    const int fset = bmesh_cd_face_get<int>(*face, offset);
    if (fset != color_default) {
      BKE_paint_face_set_overlay_color_get(fset, color_seed, color);
    }
    std::fill_n(data, 3, color);
    data += 3;
  }
}

static void fill_vbo_attribute_bmesh(const Set<BMFace *, 0> &faces,
                                     const eCustomDataType data_type,
                                     const bke::AttrDomain domain,
                                     const int offset,
                                     gpu::VertBuf &vbo)
{
  bke::attribute_math::convert_to_static_type(data_type, [&](auto dummy) {
    using T = decltype(dummy);
    if constexpr (!std::is_void_v<typename AttributeConverter<T>::VBOType>) {
      switch (domain) {
        case bke::AttrDomain::Point:
          extract_data_vert_bmesh<T>(faces, offset, vbo);
          break;
        case bke::AttrDomain::Face:
          extract_data_face_bmesh<T>(faces, offset, vbo);
          break;
        case bke::AttrDomain::Corner:
          extract_data_corner_bmesh<T>(faces, offset, vbo);
          break;
        default:
          BLI_assert_unreachable();
      }
    }
  });
}

static void fill_vbos_bmesh(const Object &object,
                            const OrigMeshData &orig_mesh_data,
                            const Span<bke::pbvh::Node> nodes,
                            const IndexMask &nodes_to_update,
                            const AttributeRequest &request,
                            const MutableSpan<gpu::VertBuf *> vbos)
{
  const SculptSession &ss = *object.sculpt;
  const BMesh &bm = *ss.bm;
  if (const CustomRequest *request_type = std::get_if<CustomRequest>(&request)) {
    switch (*request_type) {
      case CustomRequest::Position: {
        nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {
          fill_vbo_position_bmesh(
              BKE_pbvh_bmesh_node_faces(&const_cast<bke::pbvh::Node &>(nodes[i])), *vbos[i]);
        });
        break;
      }
      case CustomRequest::Normal: {
        nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {
          fill_vbo_normal_bmesh(
              BKE_pbvh_bmesh_node_faces(&const_cast<bke::pbvh::Node &>(nodes[i])), *vbos[i]);
        });
        break;
      }
      case CustomRequest::Mask: {
        const int cd_offset = CustomData_get_offset_named(
            &bm.vdata, CD_PROP_FLOAT, ".sculpt_mask");
        if (cd_offset != -1) {
          nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {
            extract_data_vert_bmesh<float>(
                BKE_pbvh_bmesh_node_faces(&const_cast<bke::pbvh::Node &>(nodes[i])),
                cd_offset,
                *vbos[i]);
          });
        }
        else {
          nodes_to_update.foreach_index(GrainSize(16),
                                        [&](const int i) { vbos[i]->data<float>().fill(0.0f); });
        }
        break;
      }
      case CustomRequest::FaceSet: {
        const int cd_offset = CustomData_get_offset_named(
            &bm.pdata, CD_PROP_INT32, ".sculpt_face_set");

        if (cd_offset != -1) {
          nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {
            fill_vbo_face_set_bmesh(
                BKE_pbvh_bmesh_node_faces(&const_cast<bke::pbvh::Node &>(nodes[i])),
                orig_mesh_data.face_set_default,
                orig_mesh_data.face_set_seed,
                cd_offset,
                *vbos[i]);
          });
        }
        else {
          nodes_to_update.foreach_index(
              GrainSize(16), [&](const int i) { vbos[i]->data<uchar4>().fill(uchar4(255)); });
        }
        break;
      }
    }
  }
  else {
    const GenericRequest &attr = std::get<GenericRequest>(request);
    const StringRefNull name = attr.name;
    const bke::AttrDomain domain = attr.domain;
    const eCustomDataType data_type = attr.type;
    const CustomData &custom_data = *get_cdata(bm, domain);
    const int offset = CustomData_get_offset_named(&custom_data, data_type, name);
    nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {
      fill_vbo_attribute_bmesh(BKE_pbvh_bmesh_node_faces(&const_cast<bke::pbvh::Node &>(nodes[i])),
                               data_type,
                               domain,
                               offset,
                               *vbos[i]);
    });
  }
}

static gpu::IndexBuf *create_index_faces(const Span<int2> edges,
                                         const Span<int> corner_verts,
                                         const Span<int> corner_edges,
                                         const Span<int3> corner_tris,
                                         const Span<int> tri_faces,
                                         const Span<bool> hide_poly,
                                         const Span<int> tri_indices)
{
  /* Calculate number of edges. */
  int edge_count = 0;
  for (const int tri_i : tri_indices) {
    if (!hide_poly.is_empty() && hide_poly[tri_faces[tri_i]]) {
      continue;
    }
    const int3 real_edges = bke::mesh::corner_tri_get_real_edges(
        edges, corner_verts, corner_edges, corner_tris[tri_i]);
    if (real_edges[0] != -1) {
      edge_count++;
    }
    if (real_edges[1] != -1) {
      edge_count++;
    }
    if (real_edges[2] != -1) {
      edge_count++;
    }
  }

  GPUIndexBufBuilder builder;
  GPU_indexbuf_init(&builder, GPU_PRIM_LINES, edge_count, INT_MAX);
  MutableSpan<uint2> data = GPU_indexbuf_get_data(&builder).cast<uint2>();

  int edge_i = 0;
  int vert_i = 0;
  for (const int tri_i : tri_indices) {
    if (!hide_poly.is_empty() && hide_poly[tri_faces[tri_i]]) {
      continue;
    }

    const int3 real_edges = bke::mesh::corner_tri_get_real_edges(
        edges, corner_verts, corner_edges, corner_tris[tri_i]);

    if (real_edges[0] != -1) {
      data[edge_i] = uint2(vert_i, vert_i + 1);
      edge_i++;
    }
    if (real_edges[1] != -1) {
      data[edge_i] = uint2(vert_i + 1, vert_i + 2);
      edge_i++;
    }
    if (real_edges[2] != -1) {
      data[edge_i] = uint2(vert_i + 2, vert_i);
      edge_i++;
    }

    vert_i += 3;
  }

  gpu::IndexBuf *ibo = GPU_indexbuf_calloc();
  GPU_indexbuf_build_in_place_ex(&builder, 0, vert_i, false, ibo);
  return ibo;
}

static gpu::IndexBuf *create_index_bmesh(const Set<BMFace *, 0> &faces,
                                         const int visible_faces_num)
{
  GPUIndexBufBuilder elb_lines;
  GPU_indexbuf_init(&elb_lines, GPU_PRIM_LINES, visible_faces_num * 3, INT_MAX);

  int v_index = 0;

  for (const BMFace *face : faces) {
    if (BM_elem_flag_test(face, BM_ELEM_HIDDEN)) {
      continue;
    }

    GPU_indexbuf_add_line_verts(&elb_lines, v_index, v_index + 1);
    GPU_indexbuf_add_line_verts(&elb_lines, v_index + 1, v_index + 2);
    GPU_indexbuf_add_line_verts(&elb_lines, v_index + 2, v_index);

    v_index += 3;
  }

  return GPU_indexbuf_build(&elb_lines);
}

static void create_tri_index_grids(const Span<int> grid_indices,
                                   int display_gridsize,
                                   GPUIndexBufBuilder &elb,
                                   const BitGroupVector<> &grid_hidden,
                                   const int gridsize,
                                   const int skip,
                                   const int totgrid)
{
  uint offset = 0;
  const uint grid_vert_len = gridsize * gridsize;
  for (int i = 0; i < totgrid; i++, offset += grid_vert_len) {
    uint v0, v1, v2, v3;

    const BoundedBitSpan gh = grid_hidden.is_empty() ? BoundedBitSpan() :
                                                       grid_hidden[grid_indices[i]];

    for (int y = 0; y < gridsize - skip; y += skip) {
      for (int x = 0; x < gridsize - skip; x += skip) {
        /* Skip hidden grid face */
        if (!gh.is_empty() && paint_is_grid_face_hidden(gh, gridsize, x, y)) {
          continue;
        }
        /* Indices in a Clockwise QUAD disposition. */
        v0 = offset + CCG_grid_xy_to_index(gridsize, x, y);
        v1 = offset + CCG_grid_xy_to_index(gridsize, x + skip, y);
        v2 = offset + CCG_grid_xy_to_index(gridsize, x + skip, y + skip);
        v3 = offset + CCG_grid_xy_to_index(gridsize, x, y + skip);

        GPU_indexbuf_add_tri_verts(&elb, v0, v2, v1);
        GPU_indexbuf_add_tri_verts(&elb, v0, v3, v2);
      }
    }
  }
}

static void create_tri_index_grids_flat_layout(const Span<int> grid_indices,
                                               int display_gridsize,
                                               GPUIndexBufBuilder &elb,
                                               const BitGroupVector<> &grid_hidden,
                                               const int gridsize,
                                               const int skip,
                                               const int totgrid)
{
  uint offset = 0;
  const uint grid_vert_len = square_uint(gridsize - 1) * 4;

  for (int i = 0; i < totgrid; i++, offset += grid_vert_len) {
    const BoundedBitSpan gh = grid_hidden.is_empty() ? BoundedBitSpan() :
                                                       grid_hidden[grid_indices[i]];

    uint v0, v1, v2, v3;
    for (int y = 0; y < gridsize - skip; y += skip) {
      for (int x = 0; x < gridsize - skip; x += skip) {
        /* Skip hidden grid face */
        if (!gh.is_empty() && paint_is_grid_face_hidden(gh, gridsize, x, y)) {
          continue;
        }

        v0 = (y * (gridsize - 1) + x) * 4;

        if (skip > 1) {
          v1 = (y * (gridsize - 1) + x + skip - 1) * 4;
          v2 = ((y + skip - 1) * (gridsize - 1) + x + skip - 1) * 4;
          v3 = ((y + skip - 1) * (gridsize - 1) + x) * 4;
        }
        else {
          v1 = v2 = v3 = v0;
        }

        /* VBO data are in a Clockwise QUAD disposition.  Note
         * that vertices might be in different quads if we're
         * building a coarse index buffer.
         */
        v0 += offset;
        v1 += offset + 1;
        v2 += offset + 2;
        v3 += offset + 3;

        GPU_indexbuf_add_tri_verts(&elb, v0, v2, v1);
        GPU_indexbuf_add_tri_verts(&elb, v0, v3, v2);
      }
    }
  }
}

static void create_lines_index_grids(const Span<int> grid_indices,
                                     int display_gridsize,
                                     GPUIndexBufBuilder &elb_lines,
                                     const BitGroupVector<> &grid_hidden,
                                     const int gridsize,
                                     const int skip,
                                     const int totgrid)
{
  uint offset = 0;
  const uint grid_vert_len = gridsize * gridsize;
  for (int i = 0; i < totgrid; i++, offset += grid_vert_len) {
    uint v0, v1, v2, v3;
    bool grid_visible = false;

    const BoundedBitSpan gh = grid_hidden.is_empty() ? BoundedBitSpan() :
                                                       grid_hidden[grid_indices[i]];

    for (int y = 0; y < gridsize - skip; y += skip) {
      for (int x = 0; x < gridsize - skip; x += skip) {
        /* Skip hidden grid face */
        if (!gh.is_empty() && paint_is_grid_face_hidden(gh, gridsize, x, y)) {
          continue;
        }
        /* Indices in a Clockwise QUAD disposition. */
        v0 = offset + CCG_grid_xy_to_index(gridsize, x, y);
        v1 = offset + CCG_grid_xy_to_index(gridsize, x + skip, y);
        v2 = offset + CCG_grid_xy_to_index(gridsize, x + skip, y + skip);
        v3 = offset + CCG_grid_xy_to_index(gridsize, x, y + skip);

        GPU_indexbuf_add_line_verts(&elb_lines, v0, v1);
        GPU_indexbuf_add_line_verts(&elb_lines, v0, v3);

        if (y / skip + 2 == display_gridsize) {
          GPU_indexbuf_add_line_verts(&elb_lines, v2, v3);
        }
        grid_visible = true;
      }

      if (grid_visible) {
        GPU_indexbuf_add_line_verts(&elb_lines, v1, v2);
      }
    }
  }
}

static void create_lines_index_grids_flat_layout(const Span<int> grid_indices,
                                                 int display_gridsize,
                                                 GPUIndexBufBuilder &elb_lines,
                                                 const BitGroupVector<> &grid_hidden,
                                                 const int gridsize,
                                                 const int skip,
                                                 const int totgrid)
{
  uint offset = 0;
  const uint grid_vert_len = square_uint(gridsize - 1) * 4;

  for (int i = 0; i < totgrid; i++, offset += grid_vert_len) {
    bool grid_visible = false;
    const BoundedBitSpan gh = grid_hidden.is_empty() ? BoundedBitSpan() :
                                                       grid_hidden[grid_indices[i]];

    uint v0, v1, v2, v3;
    for (int y = 0; y < gridsize - skip; y += skip) {
      for (int x = 0; x < gridsize - skip; x += skip) {
        /* Skip hidden grid face */
        if (!gh.is_empty() && paint_is_grid_face_hidden(gh, gridsize, x, y)) {
          continue;
        }

        v0 = (y * (gridsize - 1) + x) * 4;

        if (skip > 1) {
          v1 = (y * (gridsize - 1) + x + skip - 1) * 4;
          v2 = ((y + skip - 1) * (gridsize - 1) + x + skip - 1) * 4;
          v3 = ((y + skip - 1) * (gridsize - 1) + x) * 4;
        }
        else {
          v1 = v2 = v3 = v0;
        }

        /* VBO data are in a Clockwise QUAD disposition.  Note
         * that vertices might be in different quads if we're
         * building a coarse index buffer.
         */
        v0 += offset;
        v1 += offset + 1;
        v2 += offset + 2;
        v3 += offset + 3;

        GPU_indexbuf_add_line_verts(&elb_lines, v0, v1);
        GPU_indexbuf_add_line_verts(&elb_lines, v0, v3);

        if (y / skip + 2 == display_gridsize) {
          GPU_indexbuf_add_line_verts(&elb_lines, v2, v3);
        }
        grid_visible = true;
      }

      if (grid_visible) {
        GPU_indexbuf_add_line_verts(&elb_lines, v1, v2);
      }
    }
  }
}

static void calc_material_indices(const Object &object,
                                  const Span<bke::pbvh::Node> nodes,
                                  const IndexMask &nodes_to_update,
                                  MutableSpan<int> node_materials)
{
  const SculptSession &ss = *object.sculpt;
  const bke::pbvh::Tree &pbvh = *ss.pbvh;
  switch (pbvh.type()) {
    case bke::pbvh::Type::Mesh: {
      const Mesh &mesh = *static_cast<const Mesh *>(object.data);
      const Span<int> tri_faces = mesh.corner_tri_faces();
      const bke::AttributeAccessor attributes = mesh.attributes();
      const VArray material_indices = *attributes.lookup_or_default<int>(
          "material_index", bke::AttrDomain::Face, 0);
      nodes_to_update.foreach_index(GrainSize(16), [&](const int i) {
        const Span<int> tris = bke::pbvh::node_tri_indices(nodes[i]);
        if (tris.is_empty()) {
          node_materials[i] = 0;
        }
        node_materials[i] = material_indices[tri_faces[tris.first()]];
      });
      break;
    }
    case bke::pbvh::Type::Grids: {
      const Mesh &mesh = *static_cast<const Mesh *>(object.data);
      const bke::AttributeAccessor attributes = mesh.attributes();
      const VArray material_indices = *attributes.lookup_or_default<int>(
          "material_index", bke::AttrDomain::Face, 0);
      const SubdivCCG &subdiv_ccg = *ss.subdiv_ccg;
      const Span<int> grid_faces = subdiv_ccg.grid_to_face_map;
      nodes_to_update.foreach_index(GrainSize(16), [&](const int i) {
        const Span<int> grids = bke::pbvh::node_grid_indices(nodes[i]);
        if (grids.is_empty()) {
          node_materials[i] = 0;
        }
        node_materials[i] = material_indices[grid_faces[grids.first()]];
      });
      break;
    }
    case bke::pbvh::Type::BMesh:
      node_materials.fill(0);
      break;
  }
}

static void ensure_use_flat_layout_check(const Object &object,
                                         const Span<bke::pbvh::Node> nodes,
                                         DrawCache &draw_data)
{
  const bke::pbvh::Tree &pbvh = *object.sculpt->pbvh;
  switch (pbvh.type()) {
    case bke::pbvh::Type::Mesh:
      break;
    case bke::pbvh::Type::Grids: {
      if (draw_data.use_flat_layout.size() == nodes.size()) {
        return;
      }
      draw_data.use_flat_layout.resize(nodes.size());
      const MutableSpan<bool> use_flat_layout = draw_data.use_flat_layout;

      const Mesh &mesh = *static_cast<const Mesh *>(object.data);
      const bke::AttributeAccessor attributes = mesh.attributes();
      const VArraySpan sharp_faces = *attributes.lookup<bool>("sharp_face", bke::AttrDomain::Face);
      if (sharp_faces.is_empty()) {
        use_flat_layout.fill(false);
      }
      else {
        const SubdivCCG &subdiv_ccg = *object.sculpt->subdiv_ccg;
        const Span<int> grid_to_face_map = subdiv_ccg.grid_to_face_map;
        threading::parallel_for(nodes.index_range(), 4, [&](const IndexRange range) {
          for (const int i : range) {
            const Span<int> grids = bke::pbvh::node_grid_indices(nodes[i]);
            use_flat_layout[i] = std::any_of(grids.begin(), grids.end(), [&](const int grid) {
              return sharp_faces[grid_to_face_map[grid]];
            });
          }
        });
      }
      break;
    }
    case bke::pbvh::Type::BMesh:
      break;
  }
}

static gpu::IndexBuf *create_tri_index_grids(const CCGKey &key,
                                             const SubdivCCG &subdiv_ccg,
                                             const Span<bool> sharp_faces,
                                             const bool do_coarse,
                                             const Span<int> grid_indices,
                                             const bool use_flat_layout)
{
  const BitGroupVector<> &grid_hidden = subdiv_ccg.grid_hidden;
  const Span<int> grid_to_face_map = subdiv_ccg.grid_to_face_map;

  int gridsize = key.grid_size;
  int display_gridsize = gridsize;
  int totgrid = grid_indices.size();
  int skip = 1;

  const int display_level = do_coarse ? 0 : key.level;

  if (display_level < key.level) {
    display_gridsize = (1 << display_level) + 1;
    skip = 1 << (key.level - display_level - 1);
  }

  GPUIndexBufBuilder elb;

  // TODO
  uint visible_quad_len = bke::pbvh::count_grid_quads(
      grid_hidden, grid_indices, key.grid_size, display_gridsize);

  GPU_indexbuf_init(&elb, GPU_PRIM_TRIS, 2 * visible_quad_len, INT_MAX);

  if (use_flat_layout) {
    create_tri_index_grids_flat_layout(
        grid_indices, display_gridsize, elb, grid_hidden, gridsize, skip, totgrid);
  }
  else {
    create_tri_index_grids(
        grid_indices, display_gridsize, elb, grid_hidden, gridsize, skip, totgrid);
  }

  return GPU_indexbuf_build(&elb);
}

static gpu::IndexBuf *create_lines_index_grids(const CCGKey &key,
                                               const SubdivCCG &subdiv_ccg,
                                               const Span<bool> sharp_faces,
                                               const bool do_coarse,
                                               const Span<int> grid_indices,
                                               const bool use_flat_layout)
{
  const BitGroupVector<> &grid_hidden = subdiv_ccg.grid_hidden;
  const Span<int> grid_to_face_map = subdiv_ccg.grid_to_face_map;

  int gridsize = key.grid_size;
  int display_gridsize = gridsize;
  int totgrid = grid_indices.size();
  int skip = 1;

  const int display_level = do_coarse ? 0 : key.level;

  if (display_level < key.level) {
    display_gridsize = (1 << display_level) + 1;
    skip = 1 << (key.level - display_level - 1);
  }

  GPUIndexBufBuilder elb_lines;
  GPU_indexbuf_init(&elb_lines,
                    GPU_PRIM_LINES,
                    2 * totgrid * display_gridsize * (display_gridsize - 1),
                    INT_MAX);

  if (use_flat_layout) {
    create_lines_index_grids_flat_layout(
        grid_indices, display_gridsize, elb_lines, grid_hidden, gridsize, skip, totgrid);
  }
  else {
    create_lines_index_grids(
        grid_indices, display_gridsize, elb_lines, grid_hidden, gridsize, skip, totgrid);
  }

  return GPU_indexbuf_build(&elb_lines);
}

static Span<gpu::IndexBuf *> ensure_lines_ibos(const Object &object,
                                               const Span<bke::pbvh::Node> nodes,
                                               const IndexMask &nodes_to_update,
                                               DrawCache &draw_data)
{
  const bke::pbvh::Tree &pbvh = *object.sculpt->pbvh;
  draw_data.tris_ibos.resize(nodes.size(), nullptr);
  MutableSpan<gpu::IndexBuf *> ibos = draw_data.tris_ibos;

  IndexMaskMemory memory;
  const IndexMask nodes_to_calculate = IndexMask::from_predicate(
      nodes_to_update, GrainSize(128), memory, [&](const int i) { return !ibos[i]; });

  switch (pbvh.type()) {
    case bke::pbvh::Type::Mesh: {
      const Mesh &mesh = *static_cast<const Mesh *>(object.data);
      const Span<int2> edges = mesh.edges();
      const Span<int> corner_verts = mesh.corner_verts();
      const Span<int> corner_edges = mesh.corner_edges();
      const Span<int3> corner_tris = mesh.corner_tris();
      const Span<int> tri_faces = mesh.corner_tri_faces();
      const bke::AttributeAccessor attributes = mesh.attributes();
      const VArraySpan hide_poly = *attributes.lookup<bool>(".hide_poly", bke::AttrDomain::Face);
      nodes_to_calculate.foreach_index(GrainSize(1), [&](const int i) {
        ibos[i] = create_index_faces(edges,
                                     corner_verts,
                                     corner_edges,
                                     corner_tris,
                                     tri_faces,
                                     hide_poly,
                                     bke::pbvh::node_tri_indices(nodes[i]));
      });
      break;
    }
    case bke::pbvh::Type::Grids: {
      nodes_to_calculate.foreach_index(GrainSize(1), [&](const int i) {
        const Mesh &base_mesh = *static_cast<const Mesh *>(object.data);
        const bke::AttributeAccessor attributes = base_mesh.attributes();
        const VArraySpan sharp_faces = *attributes.lookup<bool>(".sharp_face",
                                                                bke::AttrDomain::Face);

        const SubdivCCG &subdiv_ccg = *object.sculpt->subdiv_ccg;
        const CCGKey key = BKE_subdiv_ccg_key_top_level(subdiv_ccg);
        ibos[i] = create_lines_index_grids(key,
                                           subdiv_ccg,
                                           sharp_faces,
                                           false,  // TODO
                                           bke::pbvh::node_tri_indices(nodes[i]),
                                           draw_data.use_flat_layout[i]);
      });
      break;
    }
    case bke::pbvh::Type::BMesh: {
      nodes_to_calculate.foreach_index(GrainSize(1), [&](const int i) {
        const int visible_faces_num = 100;
        ibos[i] = create_index_bmesh(
            BKE_pbvh_bmesh_node_faces(&const_cast<bke::pbvh::Node &>(nodes[i])),
            visible_faces_num);
      });
      break;
    }
  }
}

static void ensure_vbos_allocated_mesh(const Object &object,
                                       const GPUVertFormat &format,
                                       const Span<bke::pbvh::Node> nodes,
                                       const IndexMask &nodes_to_update,
                                       const MutableSpan<gpu::VertBuf *> vbos)
{
  const Mesh &mesh = *static_cast<Mesh *>(object.data);
  const Span<int> tri_faces = mesh.corner_tri_faces();
  const bke::AttributeAccessor attributes = mesh.attributes();
  const VArraySpan hide_poly = *attributes.lookup<bool>(".hide_poly", bke::AttrDomain::Face);
  nodes_to_update.foreach_index(GrainSize(16), [&](const int i) {
    if (!vbos[i]) {
      vbos[i] = GPU_vertbuf_create_with_format(format);
    }
    const Span<int> tris = bke::pbvh::node_tri_indices(nodes[i]);
    const int verts_num = count_visible_tris_mesh(tris, tri_faces, hide_poly) * 3;
    if (GPU_vertbuf_get_vertex_len(vbos[i]) != verts_num) {
      GPU_vertbuf_data_alloc(*vbos[i], verts_num);
    }
  });
}

static void ensure_vbos_allocated_grids(const Object &object,
                                        const GPUVertFormat &format,
                                        const Span<bke::pbvh::Node> nodes,
                                        const IndexMask &nodes_to_update,
                                        const MutableSpan<gpu::VertBuf *> vbos)
{
  const SculptSession &ss = *object.sculpt;
  const SubdivCCG &subdiv_ccg = *ss.subdiv_ccg;
  const BitGroupVector<> &grid_hidden = subdiv_ccg.grid_hidden;
  const CCGKey key = BKE_subdiv_ccg_key_top_level(subdiv_ccg);
  nodes_to_update.foreach_index(GrainSize(16), [&](const int i) {
    if (!vbos[i]) {
      vbos[i] = GPU_vertbuf_create_with_format(format);
    }
    const int verts_per_grid = square_i(key.grid_size - 1) * 4;  // TODO: use_flat_layout
    const int verts_num = bke::pbvh::node_grid_indices(nodes[i]).size() * verts_per_grid;
    if (GPU_vertbuf_get_vertex_len(vbos[i]) != verts_num) {
      GPU_vertbuf_data_alloc(*vbos[i], verts_num);
    }
  });
}

static void ensure_vbos_allocated_bmesh(const Object &object,
                                        const GPUVertFormat &format,
                                        const Span<bke::pbvh::Node> nodes,
                                        const IndexMask &nodes_to_update,
                                        const MutableSpan<gpu::VertBuf *> vbos)
{
  const SculptSession &ss = *object.sculpt;
  const BMesh &bm = *ss.bm;
  nodes_to_update.foreach_index(GrainSize(16), [&](const int i) {
    if (!vbos[i]) {
      vbos[i] = GPU_vertbuf_create_with_format(format);
    }
    const Set<BMFace *, 0> &faces = BKE_pbvh_bmesh_node_faces(
        &const_cast<bke::pbvh::Node &>(nodes[i]));
    const int verts_num = count_visible_tris_bmesh(faces) * 3;
    if (GPU_vertbuf_get_vertex_len(vbos[i]) != verts_num) {
      GPU_vertbuf_data_alloc(*vbos[i], verts_num);
    }
  });
}

static Span<gpu::VertBuf *> ensure_vbos(const Object &object,
                                        const OrigMeshData &orig_mesh_data,
                                        const Span<bke::pbvh::Node> nodes,
                                        const IndexMask &nodes_to_update,
                                        const AttributeRequest &attr,
                                        DrawCache &draw_data)
{
  Vector<gpu::VertBuf *> &vbos = draw_data.attribute_vbos.lookup_or_add_default(attr);
  vbos.resize(nodes.size(), nullptr);

  const GPUVertFormat format = format_for_request(orig_mesh_data, attr);

  const bke::pbvh::Tree &pbvh = *object.sculpt->pbvh;
  switch (pbvh.type()) {
    case bke::pbvh::Type::Mesh: {
      ensure_vbos_allocated_mesh(object, format, nodes, nodes_to_update, vbos);
      fill_vbos_mesh(object, orig_mesh_data, nodes, nodes_to_update, attr, vbos);
      break;
    }
    case bke::pbvh::Type::Grids: {
      ensure_vbos_allocated_grids(object, format, nodes, nodes_to_update, vbos);
      fill_vbos_grids(
          object, orig_mesh_data, nodes, draw_data.use_flat_layout, nodes_to_update, attr, vbos);
      break;
    }
    case bke::pbvh::Type::BMesh: {
      ensure_vbos_allocated_bmesh(object, format, nodes, nodes_to_update, vbos);
      fill_vbos_bmesh(object, orig_mesh_data, nodes, nodes_to_update, attr, vbos);
      break;
    }
  }

  return vbos;
}

static Span<gpu::IndexBuf *> ensure_tris_ibos(const Object &object,
                                              const Span<bke::pbvh::Node> nodes,
                                              const IndexMask &nodes_to_update,
                                              DrawCache &draw_data)
{
  const bke::pbvh::Tree &pbvh = *object.sculpt->pbvh;
  switch (pbvh.type()) {
    case bke::pbvh::Type::Mesh:
      break;
    case bke::pbvh::Type::Grids: {
      draw_data.tris_ibos.resize(nodes.size(), nullptr);
      const MutableSpan<gpu::IndexBuf *> ibos = draw_data.tris_ibos;
      IndexMaskMemory memory;
      const IndexMask nodes_to_calculate = IndexMask::from_predicate(
          nodes_to_update, GrainSize(8196), memory, [&](const int i) { return !ibos[i]; });

      const Mesh &mesh = *static_cast<const Mesh *>(object.data);
      const bke::AttributeAccessor attributes = mesh.attributes();
      const VArraySpan sharp_faces = *attributes.lookup<bool>(".sharp_face",
                                                              bke::AttrDomain::Face);
      const SubdivCCG &subdiv_ccg = *object.sculpt->subdiv_ccg;
      const CCGKey key = BKE_subdiv_ccg_key_top_level(subdiv_ccg);

      nodes_to_calculate.foreach_index(GrainSize(1), [&](const int i) {
        ibos[i] = create_tri_index_grids(key,
                                         subdiv_ccg,
                                         sharp_faces,
                                         false,  // TODO
                                         bke::pbvh::node_grid_indices(nodes[i]),
                                         draw_data.use_flat_layout[i]);
      });
      break;
    }
    case bke::pbvh::Type::BMesh:
      break;
  }
  return draw_data.tris_ibos;
}

Span<gpu::Batch *> ensure_tris_batches(const Object &object,
                                       const ViewportRequest &request,
                                       const IndexMask &nodes_to_update,
                                       DrawCache &draw_data)
{
  const Object &object_orig = *DEG_get_original_object(&const_cast<Object &>(object));
  const Mesh &mesh_orig = *static_cast<const Mesh *>(object_orig.data);

  const bke::pbvh::Tree &pbvh = *object.sculpt->pbvh;
  const Span<bke::pbvh::Node> nodes = pbvh.nodes_;

  ensure_use_flat_layout_check(object, nodes, draw_data);

  const OrigMeshData orig_mesh_data{mesh_orig};

  Vector<Span<gpu::VertBuf *>> requested_vbos;
  for (const AttributeRequest &attr : request.attributes) {
    requested_vbos.append(
        ensure_vbos(object, orig_mesh_data, nodes, nodes_to_update, attr, draw_data));
  }

  const Span<gpu::IndexBuf *> ibos = ensure_tris_ibos(object, nodes, nodes_to_update, draw_data);

  Vector<gpu::Batch *> &batches = draw_data.tris_batches.lookup_or_add_default(request);
  batches.resize(nodes.size(), nullptr);
  nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {
    if (!batches[i]) {
      GPU_batch_create(GPU_PRIM_LINES, nullptr, ibos[i]);
      for (Span<gpu::VertBuf *> vbos : requested_vbos) {
        GPU_batch_vertbuf_add(batches[i], vbos[i], false);
      }
    }
  });

  return batches;
}

static void clear_all_data(DrawCache &batches)
{
  batches.lines_ibos.clear();
  batches.tris_ibos.clear();
  batches.attribute_vbos.clear();

  batches.lines_batches.clear();
  batches.tris_batches.clear();
}

Span<gpu::Batch *> ensure_lines_batches(const Object &object,
                                        const ViewportRequest &request,  // TODO: Use coarseness
                                        const IndexMask &nodes_to_update,
                                        DrawCache &draw_data)
{
  const Object &object_orig = *DEG_get_original_object(&const_cast<Object &>(object));
  const Mesh &mesh_orig = *static_cast<const Mesh *>(object_orig.data);

  const bke::pbvh::Tree &pbvh = *object.sculpt->pbvh;
  const Span<bke::pbvh::Node> nodes = pbvh.nodes_;

  const Span<gpu::VertBuf *> position_vbos = ensure_vbos(
      object, OrigMeshData{mesh_orig}, nodes, nodes_to_update, CustomRequest::Position, draw_data);
  const Span<gpu::IndexBuf *> ibos = ensure_lines_ibos(object, nodes, nodes_to_update, draw_data);

  draw_data.lines_batches.resize(nodes.size(), nullptr);
  MutableSpan<gpu::Batch *> batches = draw_data.lines_batches;

  const Span<gpu::IndexBuf *> ibos = draw_data.lines_ibos;

  nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {
    if (!batches[i]) {
      GPU_batch_create(GPU_PRIM_LINES, nullptr, ibos[i]);
      GPU_batch_vertbuf_add(batches[i], position_vbos[i], false);
    }
  });

  return draw_data.lines_batches;
}

Span<int> ensure_material_indices(const Object &object, DrawCache &draw_data)
{
  const bke::pbvh::Tree &pbvh = *object.sculpt->pbvh;
  const Span<bke::pbvh::Node> nodes = pbvh.nodes_;
  if (draw_data.material_indices.size() != nodes.size()) {
    // TODO: Allow to be empty when no material indices
    draw_data.material_indices.reinitialize(nodes.size());
    calc_material_indices(object, nodes, nodes.index_range(), draw_data.material_indices);
  }
  return draw_data.material_indices;
}

}  // namespace blender::draw::pbvh
