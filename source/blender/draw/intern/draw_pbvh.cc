/* SPDX-FileCopyrightText: 2005 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 *
 * PBVH drawing.
 * Embeds GPU meshes inside of PBVH nodes, used by mesh sculpt mode.
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
#include "BLI_set.hh"
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

#include "GPU_batch.hh"

#include "DRW_engine.hh"
#include "DRW_pbvh.hh"

#include "attribute_convert.hh"
#include "bmesh.hh"
#include "draw_pbvh.hh"
#include "gpu_private.hh"

#define MAX_PBVH_BATCH_KEY 512
#define MAX_PBVH_VBOS 16

namespace blender::draw::pbvh {

static bool pbvh_attr_supported(const AttributeRequest &request)
{
  if (std::holds_alternative<CustomRequest>(request)) {
    return true;
  }
  const GenericRequest &attr = std::get<GenericRequest>(request);
  if (!ELEM(attr.domain, bke::AttrDomain::Point, bke::AttrDomain::Face, bke::AttrDomain::Corner)) {
    /* PBVH drawing does not support edge domain attributes. */
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

static std::string calc_request_key(const AttributeRequest &request)
{
  char buf[512];
  if (const CustomRequest *request_type = std::get_if<CustomRequest>(&request)) {
    SNPRINTF(buf, "%d:%d:", int(*request_type) + CD_NUMTYPES, 0);
  }
  else {
    const GenericRequest &attr = std::get<GenericRequest>(request);
    const StringRefNull name = attr.name;
    const bke::AttrDomain domain = attr.domain;
    const eCustomDataType data_type = attr.type;
    SNPRINTF(buf, "%d:%d:%s", int(data_type), int(domain), name.c_str());
  }
  return buf;
}

struct PBVHVbo {
  AttributeRequest request;
  gpu::VertBuf *vert_buf = nullptr;
  std::string key;

  PBVHVbo(const AttributeRequest &request) : request(request)
  {
    key = calc_request_key(request);
  }

  void clear_data()
  {
    GPU_vertbuf_clear(vert_buf);
  }
};

inline short4 normal_float_to_short(const float3 &value)
{
  short3 result;
  normal_float_to_short_v3(result, value);
  return short4(result.x, result.y, result.z, 0);
}

template<typename T>
void extract_data_vert_faces(const Span<int> corner_verts,
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
void extract_data_face_faces(const PBVH_GPU_Args &args, const Span<T> attribute, gpu::VertBuf &vbo)
{
  using Converter = AttributeConverter<T>;
  using VBOType = typename Converter::VBOType;

  const Span<int> tri_faces = args.tri_faces;
  const Span<bool> hide_poly = args.hide_poly;

  VBOType *data = vbo.data<VBOType>().data();
  for (const int tri_i : args.prim_indices) {
    const int face = tri_faces[tri_i];
    if (!hide_poly.is_empty() && hide_poly[face]) {
      continue;
    }
    std::fill_n(data, 3, Converter::convert(attribute[face]));
    data += 3;
  }
}

template<typename T>
void extract_data_corner_faces(const PBVH_GPU_Args &args,
                               const Span<T> attribute,
                               gpu::VertBuf &vbo)
{
  using Converter = AttributeConverter<T>;
  using VBOType = typename Converter::VBOType;

  const Span<int3> corner_tris = args.corner_tris;
  const Span<int> tri_faces = args.tri_faces;
  const Span<bool> hide_poly = args.hide_poly;

  VBOType *data = vbo.data<VBOType>().data();
  for (const int tri_i : args.prim_indices) {
    if (!hide_poly.is_empty() && hide_poly[tri_faces[tri_i]]) {
      continue;
    }
    for (int i : IndexRange(3)) {
      const int corner = corner_tris[tri_i][i];
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
void extract_data_vert_bmesh(const PBVH_GPU_Args &args, const int cd_offset, gpu::VertBuf &vbo)
{
  using Converter = AttributeConverter<T>;
  using VBOType = typename Converter::VBOType;
  VBOType *data = vbo.data<VBOType>().data();

  for (const BMFace *f : *args.bm_faces) {
    if (BM_elem_flag_test(f, BM_ELEM_HIDDEN)) {
      continue;
    }
    const BMLoop *l = f->l_first;
    *data = Converter::convert(bmesh_cd_vert_get<T>(*l->prev->v, cd_offset));
    data++;
    *data = Converter::convert(bmesh_cd_vert_get<T>(*l->v, cd_offset));
    data++;
    *data = Converter::convert(bmesh_cd_vert_get<T>(*l->next->v, cd_offset));
    data++;
  }
}

template<typename T>
void extract_data_face_bmesh(const PBVH_GPU_Args &args, const int cd_offset, gpu::VertBuf &vbo)
{
  using Converter = AttributeConverter<T>;
  using VBOType = typename Converter::VBOType;
  VBOType *data = vbo.data<VBOType>().data();

  for (const BMFace *f : *args.bm_faces) {
    if (BM_elem_flag_test(f, BM_ELEM_HIDDEN)) {
      continue;
    }
    std::fill_n(data, 3, Converter::convert(bmesh_cd_face_get<T>(*f, cd_offset)));
    data += 3;
  }
}

template<typename T>
void extract_data_corner_bmesh(const PBVH_GPU_Args &args, const int cd_offset, gpu::VertBuf &vbo)
{
  using Converter = AttributeConverter<T>;
  using VBOType = typename Converter::VBOType;
  VBOType *data = vbo.data<VBOType>().data();

  for (const BMFace *f : *args.bm_faces) {
    if (BM_elem_flag_test(f, BM_ELEM_HIDDEN)) {
      continue;
    }
    const BMLoop *l = f->l_first;
    *data = Converter::convert(bmesh_cd_loop_get<T>(*l->prev, cd_offset));
    data++;
    *data = Converter::convert(bmesh_cd_loop_get<T>(*l, cd_offset));
    data++;
    *data = Converter::convert(bmesh_cd_loop_get<T>(*l->next, cd_offset));
    data++;
  }
}

struct NodeBatch {
  Vector<int> vbos;
  gpu::Batch *tris = nullptr, *lines = nullptr;
  /* Coarse multi-resolution, will use full-sized VBOs only index buffer changes. */
  bool is_coarse = false;

  void sort_vbos(Vector<PBVHVbo> &master_vbos)
  {
    struct cmp {
      Vector<PBVHVbo> &master_vbos;

      cmp(Vector<PBVHVbo> &_master_vbos) : master_vbos(_master_vbos) {}

      bool operator()(const int &a, const int &b)
      {
        return master_vbos[a].key < master_vbos[b].key;
      }
    };

    std::sort(vbos.begin(), vbos.end(), cmp(master_vbos));
  }

  std::string build_key(Vector<PBVHVbo> &master_vbos)
  {
    std::string key = "";

    if (is_coarse) {
      key += "c:";
    }

    sort_vbos(master_vbos);

    for (int vbo_i : vbos) {
      key += master_vbos[vbo_i].key + ":";
    }

    return key;
  }
};

static const CustomData *get_cdata(bke::AttrDomain domain, const PBVH_GPU_Args &args)
{
  switch (domain) {
    case bke::AttrDomain::Point:
      return args.vert_data;
    case bke::AttrDomain::Corner:
      return args.corner_data;
    case bke::AttrDomain::Face:
      return args.face_data;
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

struct NodeBatches {
  Vector<PBVHVbo> vbos;
  Map<std::string, NodeBatch> batches;
  gpu::IndexBuf *tri_index = nullptr;
  gpu::IndexBuf *lines_index = nullptr;
  int faces_count = 0; /* Used by PBVH_BMESH and PBVH_GRIDS */
  bool use_flat_layout = false;

  int material_index = 0;

  /* Stuff for displaying coarse multires grids. */
  gpu::IndexBuf *tri_index_coarse = nullptr;
  gpu::IndexBuf *lines_index_coarse = nullptr;
  int coarse_level = 0; /* Coarse multires depth. */
  int tris_count_coarse = 0, lines_count_coarse = 0;

  NodeBatches(const PBVH_GPU_Args &args);
  ~NodeBatches();

  void update(const PBVH_GPU_Args &args);
  void update_pre(const PBVH_GPU_Args &args);

  int create_vbo(const AttributeRequest &request, const PBVH_GPU_Args &args);
  int ensure_vbo(const AttributeRequest &request, const PBVH_GPU_Args &args);

  void create_index(const PBVH_GPU_Args &args);
};

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

static int count_visible_tris_bmesh(const Set<BMFace *, 0> faces)
{
  return std::count_if(faces.begin(), faces.end(), [&](const BMFace *face) {
    return !BM_elem_flag_test_bool(face, BM_ELEM_HIDDEN);
  });
}

static int count_faces(const PBVH_GPU_Args &args)
{
  int count = 0;

  switch (args.pbvh_type) {
    case PBVH_FACES: {
      break;
    }
    case PBVH_GRIDS: {
      count = bke::pbvh::count_grid_quads(args.subdiv_ccg->grid_hidden,
                                          args.grid_indices,
                                          args.ccg_key.grid_size,
                                          args.ccg_key.grid_size);

      break;
    }
    case PBVH_BMESH: {
      break;
    }
  }

  return count;
}

NodeBatches::NodeBatches(const PBVH_GPU_Args &args)
{
  faces_count = count_faces(args);
}

NodeBatches::~NodeBatches()
{
  for (NodeBatch &batch : batches.values()) {
    GPU_BATCH_DISCARD_SAFE(batch.tris);
    GPU_BATCH_DISCARD_SAFE(batch.lines);
  }

  for (PBVHVbo &vbo : vbos) {
    GPU_vertbuf_discard(vbo.vert_buf);
  }

  GPU_INDEXBUF_DISCARD_SAFE(tri_index);
  GPU_INDEXBUF_DISCARD_SAFE(lines_index);
  GPU_INDEXBUF_DISCARD_SAFE(tri_index_coarse);
  GPU_INDEXBUF_DISCARD_SAFE(lines_index_coarse);
}

static std::string build_key(const Span<AttributeRequest> requests, bool do_coarse_grids)
{
  NodeBatch batch;
  Vector<PBVHVbo> vbos;

  for (const int i : requests.index_range()) {
    const AttributeRequest &request = requests[i];
    if (!pbvh_attr_supported(request)) {
      continue;
    }
    vbos.append_as(request);
    batch.vbos.append(i);
  }

  batch.is_coarse = do_coarse_grids;
  return batch.build_key(vbos);
}

int NodeBatches::ensure_vbo(const AttributeRequest &request, const PBVH_GPU_Args &args)
{
  for (const int i : vbos.index_range()) {
    if (this->vbos[i].request == request) {
      return i;
    }
  }
  return this->create_vbo(request, args);
}

static void fill_vbo_normal_faces(const Span<int> corner_verts,
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
    const int face_i = tri_faces[tri];
    if (!hide_poly.is_empty() && hide_poly[face_i]) {
      continue;
    }
    if (!sharp_faces.is_empty() && sharp_faces[face_i]) {
      if (face_i != last_face) {
        face_no = normal_float_to_short(face_normals[face_i]);
        last_face = face_i;
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

static void fill_vbo_face_sets_mesh(const Span<int3> corner_tris,
                                    const Span<int> tri_faces,
                                    const Span<bool> hide_poly,
                                    const Span<int> face_sets,
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
    const int face_i = tri_faces[tri];
    if (last_face != face_i) {
      last_face = face_i;

      const int fset = face_sets[face_i];

      if (fset != face_sets_color_default) {
        BKE_paint_face_set_overlay_color_get(fset, face_sets_color_seed, fset_color);
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

static void fill_vbo_grids_intern(
    PBVHVbo &vbo,
    const PBVH_GPU_Args &args,
    FunctionRef<void(FunctionRef<void(int x, int y, int grid_index, CCGElem *elems[4], int i)>
                         func)> foreach_grids)
{
  uint vert_per_grid = square_i(args.ccg_key.grid_size - 1) * 4;
  uint vert_count = args.grid_indices.size() * vert_per_grid;

  int existing_num = GPU_vertbuf_get_vertex_len(vbo.vert_buf);

  if (vbo.vert_buf->data<uchar>().data() == nullptr || existing_num != vert_count) {
    /* Allocate buffer if not allocated yet or size changed. */
    GPU_vertbuf_data_alloc(*vbo.vert_buf, vert_count);
  }

  GPUVertBufRaw access;
  GPU_vertbuf_attr_get_raw_data(vbo.vert_buf, 0, &access);

  if (const CustomRequest *request_type = std::get_if<CustomRequest>(&vbo.request)) {
    switch (*request_type) {
      case CustomRequest::Position: {
        foreach_grids([&](int /*x*/, int /*y*/, int /*grid_index*/, CCGElem *elems[4], int i) {
          *static_cast<float3 *>(GPU_vertbuf_raw_step(&access)) = CCG_elem_co(args.ccg_key,
                                                                              elems[i]);
        });
        break;
      }
      case CustomRequest::Normal: {
        const Span<int> grid_to_face_map = args.subdiv_ccg->grid_to_face_map;
        const bke::AttributeAccessor attributes = args.mesh->attributes();
        const VArraySpan sharp_faces = *attributes.lookup<bool>("sharp_face",
                                                                bke::AttrDomain::Face);

        foreach_grids([&](int /*x*/, int /*y*/, int grid_index, CCGElem *elems[4], int /*i*/) {
          float3 no(0.0f, 0.0f, 0.0f);

          const bool smooth = !(!sharp_faces.is_empty() &&
                                sharp_faces[grid_to_face_map[grid_index]]);

          if (smooth) {
            no = CCG_elem_no(args.ccg_key, elems[0]);
          }
          else {
            normal_quad_v3(no,
                           CCG_elem_co(args.ccg_key, elems[3]),
                           CCG_elem_co(args.ccg_key, elems[2]),
                           CCG_elem_co(args.ccg_key, elems[1]),
                           CCG_elem_co(args.ccg_key, elems[0]));
          }

          short sno[3];

          normal_float_to_short_v3(sno, no);

          *static_cast<short3 *>(GPU_vertbuf_raw_step(&access)) = sno;
        });
        break;
      }
      case CustomRequest::Mask: {
        if (args.ccg_key.has_mask) {
          foreach_grids([&](int /*x*/, int /*y*/, int /*grid_index*/, CCGElem *elems[4], int i) {
            *static_cast<float *>(GPU_vertbuf_raw_step(&access)) = CCG_elem_mask(args.ccg_key,
                                                                                 elems[i]);
          });
        }
        else {
          vbo.vert_buf->data<float>().fill(0.0f);
        }
        break;
      }
      case CustomRequest::FaceSet: {
        const bke::AttributeAccessor attributes = args.mesh->attributes();
        if (const VArray<int> face_sets = *attributes.lookup<int>(".sculpt_face_set",
                                                                  bke::AttrDomain::Face))
        {
          const VArraySpan<int> face_sets_span(face_sets);
          foreach_grids(
              [&](int /*x*/, int /*y*/, int grid_index, CCGElem * /*elems*/[4], int /*i*/) {
                uchar face_set_color[4] = {UCHAR_MAX, UCHAR_MAX, UCHAR_MAX, UCHAR_MAX};

                const int face_index = BKE_subdiv_ccg_grid_to_face_index(*args.subdiv_ccg,
                                                                         grid_index);
                const int fset = face_sets_span[face_index];

                /* Skip for the default color Face Set to render it white. */
                if (fset != args.face_sets_color_default) {
                  BKE_paint_face_set_overlay_color_get(
                      fset, args.face_sets_color_seed, face_set_color);
                }

                *static_cast<uchar4 *>(GPU_vertbuf_raw_step(&access)) = face_set_color;
              });
        }
        else {
          const uchar white[4] = {UCHAR_MAX, UCHAR_MAX, UCHAR_MAX};
          foreach_grids(
              [&](int /*x*/, int /*y*/, int /*grid_index*/, CCGElem * /*elems*/[4], int /*i*/) {
                *static_cast<uchar4 *>(GPU_vertbuf_raw_step(&access)) = white;
              });
        }
        break;
      }
    }
  }
  else {
    const eCustomDataType type = std::get<GenericRequest>(vbo.request).type;
    bke::attribute_math::convert_to_static_type(type, [&](auto dummy) {
      using T = decltype(dummy);
      using Converter = AttributeConverter<T>;
      using VBOType = typename Converter::VBOType;
      if constexpr (!std::is_void_v<VBOType>) {
        std::fill_n(vbo.vert_buf->data<VBOType>().data(),
                    GPU_vertbuf_get_vertex_len(vbo.vert_buf),
                    Converter::convert(fallback_value_for_fill<T>()));
      }
    });
  }
}

static void fill_vbo_grids(PBVHVbo &vbo, const PBVH_GPU_Args &args, const bool use_flat_layout)
{
  int gridsize = args.ccg_key.grid_size;

  uint totgrid = args.grid_indices.size();

  auto foreach_flat =
      [&](FunctionRef<void(int x, int y, int grid_index, CCGElem *elems[4], int i)> func) {
        for (int i = 0; i < totgrid; i++) {
          const int grid_index = args.grid_indices[i];

          CCGElem *grid = args.grids[grid_index];

          for (int y = 0; y < gridsize - 1; y++) {
            for (int x = 0; x < gridsize - 1; x++) {
              CCGElem *elems[4] = {
                  CCG_grid_elem(args.ccg_key, grid, x, y),
                  CCG_grid_elem(args.ccg_key, grid, x + 1, y),
                  CCG_grid_elem(args.ccg_key, grid, x + 1, y + 1),
                  CCG_grid_elem(args.ccg_key, grid, x, y + 1),
              };

              func(x, y, grid_index, elems, 0);
              func(x + 1, y, grid_index, elems, 1);
              func(x + 1, y + 1, grid_index, elems, 2);
              func(x, y + 1, grid_index, elems, 3);
            }
          }
        }
      };

  auto foreach_indexed =
      [&](FunctionRef<void(int x, int y, int grid_index, CCGElem *elems[4], int i)> func) {
        for (int i = 0; i < totgrid; i++) {
          const int grid_index = args.grid_indices[i];

          CCGElem *grid = args.grids[grid_index];

          for (int y = 0; y < gridsize; y++) {
            for (int x = 0; x < gridsize; x++) {
              CCGElem *elems[4] = {
                  CCG_grid_elem(args.ccg_key, grid, x, y),
                  CCG_grid_elem(args.ccg_key, grid, min_ii(x + 1, gridsize - 1), y),
                  CCG_grid_elem(args.ccg_key,
                                grid,
                                min_ii(x + 1, gridsize - 1),
                                min_ii(y + 1, gridsize - 1)),
                  CCG_grid_elem(args.ccg_key, grid, x, min_ii(y + 1, gridsize - 1)),
              };

              func(x, y, grid_index, elems, 0);
            }
          }
        }
      };

  if (use_flat_layout) {
    fill_vbo_grids_intern(vbo, args, foreach_flat);
  }
  else {
    fill_vbo_grids_intern(vbo, args, foreach_indexed);
  }
}

static void fill_vbo_faces(PBVHVbo &vbo, const PBVH_GPU_Args &args)
{
  const int totvert = count_faces(args) * 3;

  int existing_num = GPU_vertbuf_get_vertex_len(vbo.vert_buf);

  if (vbo.vert_buf->data<uchar>().data() == nullptr || existing_num != totvert) {
    /* Allocate buffer if not allocated yet or size changed. */
    GPU_vertbuf_data_alloc(*vbo.vert_buf, totvert);
  }

  gpu::VertBuf &vert_buf = *vbo.vert_buf;

  const bke::AttributeAccessor attributes = args.mesh->attributes();

  if (const CustomRequest *request_type = std::get_if<CustomRequest>(&vbo.request)) {
    switch (*request_type) {
      case CustomRequest::Position: {
        break;
      }
      case CustomRequest::Normal: {
        break;
      }
      case CustomRequest::Mask: {
        float *data = vert_buf.data<float>().data();
        if (const VArray<float> mask = *attributes.lookup<float>(".sculpt_mask",
                                                                 bke::AttrDomain::Point))
        {
          const VArraySpan<float> mask_span(mask);
          const Span<int> corner_verts = args.corner_verts;
          const Span<int3> corner_tris = args.corner_tris;
          const Span<int> tri_faces = args.tri_faces;
          const Span<bool> hide_poly = args.hide_poly;

          for (const int tri_i : args.prim_indices) {
            if (!hide_poly.is_empty() && hide_poly[tri_faces[tri_i]]) {
              continue;
            }
            for (int i : IndexRange(3)) {
              const int vert = corner_verts[corner_tris[tri_i][i]];
              *data = mask_span[vert];
              data++;
            }
          }
        }
        else {
          MutableSpan(data, totvert).fill(0);
        }
        break;
      }
      case CustomRequest::FaceSet: {
        break;
      }
    }
  }
  else {
    const GenericRequest &request = std::get<GenericRequest>(vbo.request);
    const StringRef name = request.name;
    const bke::AttrDomain domain = request.domain;
    const eCustomDataType data_type = request.type;
    const GVArraySpan attribute = *attributes.lookup_or_default(name, domain, data_type);
    bke::attribute_math::convert_to_static_type(data_type, [&](auto dummy) {
      using T = decltype(dummy);
      if constexpr (!std::is_void_v<typename AttributeConverter<T>::VBOType>) {
        switch (domain) {
          case bke::AttrDomain::Point:
            extract_data_vert_faces<T>(args, attribute.typed<T>(), vert_buf);
            break;
          case bke::AttrDomain::Face:
            extract_data_face_faces<T>(args, attribute.typed<T>(), vert_buf);
            break;
          case bke::AttrDomain::Corner:
            extract_data_corner_faces<T>(args, attribute.typed<T>(), vert_buf);
            break;
          default:
            BLI_assert_unreachable();
        }
      }
    });
  }
}

static void gpu_flush(MutableSpan<PBVHVbo> vbos)
{
  for (PBVHVbo &vbo : vbos) {
    if (vbo.vert_buf && vbo.vert_buf->data<char>().data()) {
      GPU_vertbuf_use(vbo.vert_buf);
    }
  }
}

static void fill_vbo_bmesh(PBVHVbo &vbo, const PBVH_GPU_Args &args)
{
  int existing_num = GPU_vertbuf_get_vertex_len(vbo.vert_buf);

  int vert_count = count_faces(args) * 3;

  if (vbo.vert_buf->data<uchar>().data() == nullptr || existing_num != vert_count) {
    /* Allocate buffer if not allocated yet or size changed. */
    GPU_vertbuf_data_alloc(*vbo.vert_buf, vert_count);
  }

  if (const CustomRequest *request_type = std::get_if<CustomRequest>(&vbo.request)) {
    switch (*request_type) {
      case CustomRequest::Position: {
        float3 *data = vbo.vert_buf->data<float3>().data();
        for (const BMFace *f : *args.bm_faces) {
          if (BM_elem_flag_test(f, BM_ELEM_HIDDEN)) {
            continue;
          }
          const BMLoop *l = f->l_first;
          *data = l->prev->v->co;
          data++;
          *data = l->v->co;
          data++;
          *data = l->next->v->co;
          data++;
        }
        break;
      }
      case CustomRequest::Normal: {
        short4 *data = vbo.vert_buf->data<short4>().data();
        for (const BMFace *f : *args.bm_faces) {
          if (BM_elem_flag_test(f, BM_ELEM_HIDDEN)) {
            continue;
          }
          if (BM_elem_flag_test(f, BM_ELEM_SMOOTH)) {
            const BMLoop *l = f->l_first;
            *data = normal_float_to_short(l->prev->v->no);
            data++;
            *data = normal_float_to_short(l->v->no);
            data++;
            *data = normal_float_to_short(l->next->v->no);
            data++;
          }
          else {
            std::fill_n(data, 3, normal_float_to_short(f->no));
            data += 3;
          }
        }
        break;
      }
      case CustomRequest::Mask: {
        const int cd_offset = args.cd_mask_layer;
        if (cd_offset != -1) {
          float *data = vbo.vert_buf->data<float>().data();

          for (const BMFace *f : *args.bm_faces) {
            if (BM_elem_flag_test(f, BM_ELEM_HIDDEN)) {
              continue;
            }
            const BMLoop *l = f->l_first;
            *data = bmesh_cd_vert_get<float>(*l->prev->v, cd_offset);
            data++;
            *data = bmesh_cd_vert_get<float>(*l->v, cd_offset);
            data++;
            *data = bmesh_cd_vert_get<float>(*l->next->v, cd_offset);
            data++;
          }
        }
        else {
          vbo.vert_buf->data<float>().fill(0.0f);
        }
        break;
      }
      case CustomRequest::FaceSet: {
        const int cd_offset = CustomData_get_offset_named(
            &args.bm->pdata, CD_PROP_INT32, ".sculpt_face_set");

        uchar4 *data = vbo.vert_buf->data<uchar4>().data();
        if (cd_offset != -1) {
          for (const BMFace *f : *args.bm_faces) {
            if (BM_elem_flag_test(f, BM_ELEM_HIDDEN)) {
              continue;
            }

            const int fset = bmesh_cd_face_get<int>(*f, cd_offset);

            uchar4 fset_color;
            if (fset != args.face_sets_color_default) {
              BKE_paint_face_set_overlay_color_get(fset, args.face_sets_color_seed, fset_color);
            }
            else {
              /* Skip for the default color face set to render it white. */
              fset_color[0] = fset_color[1] = fset_color[2] = UCHAR_MAX;
            }
            std::fill_n(data, 3, fset_color);
            data += 3;
          }
          break;
        }
        else {
          MutableSpan(data, GPU_vertbuf_get_vertex_len(vbo.vert_buf)).fill(uchar4(255));
        }
      }
    }
  }
  else {
    const GenericRequest &request = std::get<GenericRequest>(vbo.request);
    const bke::AttrDomain domain = request.domain;
    const eCustomDataType data_type = request.type;
    const CustomData &custom_data = *get_cdata(domain, args);
    const int cd_offset = CustomData_get_offset_named(&custom_data, data_type, request.name);
    bke::attribute_math::convert_to_static_type(data_type, [&](auto dummy) {
      using T = decltype(dummy);
      if constexpr (!std::is_void_v<typename AttributeConverter<T>::VBOType>) {
        switch (domain) {
          case bke::AttrDomain::Point:
            extract_data_vert_bmesh<T>(args, cd_offset, *vbo.vert_buf);
            break;
          case bke::AttrDomain::Face:
            extract_data_face_bmesh<T>(args, cd_offset, *vbo.vert_buf);
            break;
          case bke::AttrDomain::Corner:
            extract_data_corner_bmesh<T>(args, cd_offset, *vbo.vert_buf);
            break;
          default:
            BLI_assert_unreachable();
        }
      }
    });
  }
}

void NodeBatches::update(const PBVH_GPU_Args &args)
{
  if (!lines_index) {
    create_index(args);
  }
  for (PBVHVbo &vbo : vbos) {
    switch (args.pbvh_type) {
      case PBVH_FACES:
        fill_vbo_faces(vbo, args);
        break;
      case PBVH_GRIDS:
        fill_vbo_grids(vbo, args, use_flat_layout);
        break;
      case PBVH_BMESH:
        fill_vbo_bmesh(vbo, args);
        break;
    }
  }
}

int NodeBatches::create_vbo(const AttributeRequest &request, const PBVH_GPU_Args &args)
{
  GPUVertFormat format;
  GPU_vertformat_clear(&format);
  if (const CustomRequest *request_type = std::get_if<CustomRequest>(&request)) {
    switch (*request_type) {
      case CustomRequest::Position:
        GPU_vertformat_attr_add(&format, "pos", GPU_COMP_F32, 3, GPU_FETCH_FLOAT);
        break;
      case CustomRequest::Normal:
        GPU_vertformat_attr_add(&format, "nor", GPU_COMP_I16, 3, GPU_FETCH_INT_TO_FLOAT_UNIT);
        break;
      case CustomRequest::Mask:
        GPU_vertformat_attr_add(&format, "msk", GPU_COMP_F32, 1, GPU_FETCH_FLOAT);
        break;
      case CustomRequest::FaceSet:
        GPU_vertformat_attr_add(&format, "fset", GPU_COMP_U8, 3, GPU_FETCH_INT_TO_FLOAT_UNIT);
        break;
    }
  }
  else {
    const GenericRequest &attr = std::get<GenericRequest>(request);
    const StringRefNull name = attr.name;
    const bke::AttrDomain domain = attr.domain;
    const eCustomDataType data_type = attr.type;

    format = draw::init_format_for_attribute(data_type, "data");

    const CustomData *cdata = get_cdata(domain, args);

    bool is_render, is_active;
    const char *prefix = "a";

    if (CD_TYPE_AS_MASK(data_type) & CD_MASK_COLOR_ALL) {
      prefix = "c";
      is_active = StringRef(args.active_color) == name;
      is_render = StringRef(args.render_color) == name;
    }
    if (data_type == CD_PROP_FLOAT2) {
      prefix = "u";
      is_active = StringRef(CustomData_get_active_layer_name(cdata, data_type)) == name;
      is_render = StringRef(CustomData_get_render_layer_name(cdata, data_type)) == name;
    }

    DRW_cdlayer_attr_aliases_add(&format, prefix, data_type, name.c_str(), is_render, is_active);
  }

  vbos.append_as(request);
  vbos.last().vert_buf = GPU_vertbuf_create_with_format_ex(format, GPU_USAGE_STATIC);
  switch (args.pbvh_type) {
    case PBVH_FACES:
      fill_vbo_faces(vbos.last(), args);
      break;
    case PBVH_GRIDS:
      fill_vbo_grids(vbos.last(), args, use_flat_layout);
      break;
    case PBVH_BMESH:
      fill_vbo_bmesh(vbos.last(), args);
      break;
  }

  return vbos.index_range().last();
}

void NodeBatches::update_pre(const PBVH_GPU_Args &args)
{
  if (args.pbvh_type == PBVH_BMESH) {
    int count = count_faces(args);

    if (faces_count != count) {
      for (PBVHVbo &vbo : vbos) {
        vbo.clear_data();
      }

      GPU_INDEXBUF_DISCARD_SAFE(tri_index);
      GPU_INDEXBUF_DISCARD_SAFE(lines_index);
      GPU_INDEXBUF_DISCARD_SAFE(tri_index_coarse);
      GPU_INDEXBUF_DISCARD_SAFE(lines_index_coarse);

      faces_count = count;
    }
  }
}

static gpu::IndexBuf *calc_lines_index_mesh(const Span<int2> edges,
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

static gpu::IndexBuf *calc_lines_index_bmesh(const Set<BMFace *, 0> faces,
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

static void create_tris_from_grids(const Span<int> grid_indices,
                                   int display_gridsize,
                                   GPUIndexBufBuilder &elb,
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

        GPU_indexbuf_add_tri_verts(&elb, v0, v2, v1);
        GPU_indexbuf_add_tri_verts(&elb, v0, v3, v2);

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

static void create_quads_from_grids(const Span<int> grid_indices,
                                    int display_gridsize,
                                    GPUIndexBufBuilder &elb,
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

        GPU_indexbuf_add_tri_verts(&elb, v0, v2, v1);
        GPU_indexbuf_add_tri_verts(&elb, v0, v3, v2);

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
                                  const Span<PBVHNode *> nodes,
                                  const IndexMask &nodes_to_update,
                                  MutableSpan<int> node_materials)
{
  const SculptSession &ss = *object.sculpt;
  const PBVH &pbvh = *ss.pbvh;
  switch (BKE_pbvh_type(pbvh)) {
    case PBVH_FACES: {
      const Mesh &mesh = *static_cast<const Mesh *>(object.data);
      const Span<int> tri_faces = mesh.corner_tri_faces();
      const bke::AttributeAccessor attributes = mesh.attributes();
      const VArray material_indices = *attributes.lookup_or_default<int>(
          "material_index", bke::AttrDomain::Face, 0);
      nodes_to_update.foreach_index(GrainSize(16), [&](const int i) {
        const Span<int> tris = bke::pbvh::node_tri_indices(*nodes[i]);
        if (tris.is_empty()) {
          node_materials[i] = 0;
        }
        node_materials[i] = material_indices[tri_faces[tris.first()]];
      });
      break;
    }
    case PBVH_GRIDS: {
      const SubdivCCG &subdiv_ccg = *ss.subdiv_ccg;
      const Span<int> grid_faces = subdiv_ccg.grid_to_face_map;
      const Mesh &mesh = *static_cast<const Mesh *>(object.data);
      const bke::AttributeAccessor attributes = mesh.attributes();
      const VArray material_indices = *attributes.lookup_or_default<int>(
          "material_index", bke::AttrDomain::Face, 0);
      nodes_to_update.foreach_index(GrainSize(16), [&](const int i) {
        const Span<int> grids = bke::pbvh::node_grid_indices(*nodes[i]);
        if (grids.is_empty()) {
          node_materials[i] = 0;
        }
        node_materials[i] = material_indices[grid_faces[grids.first()]];
      });
      break;
    }
    case PBVH_BMESH:
      node_materials.fill(0);
      break;
  }
}

static gpu::IndexBuf *create_index_grids(const SubdivCCG &subdiv_ccg,
                                         const Span<bool> sharp_faces,
                                         const CCGKey &key,
                                         const Span<int> grid_indices,
                                         const bool do_coarse,
                                         NodeBatches &batches)
{
  const BitGroupVector<> &grid_hidden = subdiv_ccg.grid_hidden;
  const Span<int> grid_to_face_map = subdiv_ccg.grid_to_face_map;

  int gridsize = key.grid_size;
  int display_gridsize = gridsize;
  int totgrid = grid_indices.size();
  int skip = 1;

  const int display_level = do_coarse ? batches.coarse_level : key.level;

  if (display_level < key.level) {
    display_gridsize = (1 << display_level) + 1;
    skip = 1 << (key.level - display_level - 1);
  }

  batches.use_flat_layout = !sharp_faces.is_empty() &&
                            std::any_of(
                                grid_indices.begin(), grid_indices.end(), [&](const int grid) {
                                  return sharp_faces[grid_to_face_map[grid]];
                                });

  GPUIndexBufBuilder elb, elb_lines;

  uint visible_quad_len = bke::pbvh::count_grid_quads(
      grid_hidden, grid_indices, key.grid_size, display_gridsize);

  GPU_indexbuf_init(&elb, GPU_PRIM_TRIS, 2 * visible_quad_len, INT_MAX);
  GPU_indexbuf_init(&elb_lines,
                    GPU_PRIM_LINES,
                    2 * totgrid * display_gridsize * (display_gridsize - 1),
                    INT_MAX);

  if (batches.use_flat_layout) {
    create_quads_from_grids(
        grid_indices, display_gridsize, elb, elb_lines, grid_hidden, gridsize, skip, totgrid);
  }
  else {
    create_tris_from_grids(
        grid_indices, display_gridsize, elb, elb_lines, grid_hidden, gridsize, skip, totgrid);
  }

  if (do_coarse) {
    batches.tri_index_coarse = GPU_indexbuf_build(&elb);
    batches.lines_index_coarse = GPU_indexbuf_build(&elb_lines);
    batches.tris_count_coarse = visible_quad_len;
    batches.lines_count_coarse = totgrid * display_gridsize * (display_gridsize - 1);
  }
  else {
    batches.tri_index = GPU_indexbuf_build(&elb);
    batches.lines_index = GPU_indexbuf_build(&elb_lines);
  }
}

static NodeBatch create_batch(NodeBatches &batches,
                              const Span<AttributeRequest> requests,
                              const PBVH_GPU_Args &args,
                              bool do_coarse_grids)
{
  if (!batches.lines_index) {
    batches.create_index(args);
  }

  NodeBatch batch;

  batch.tris = GPU_batch_create(GPU_PRIM_TRIS,
                                nullptr,
                                /* can be nullptr if buffer is empty */
                                do_coarse_grids ? batches.tri_index_coarse : batches.tri_index);
  batch.is_coarse = do_coarse_grids;

  if (batches.lines_index) {
    batch.lines = GPU_batch_create(GPU_PRIM_LINES,
                                   nullptr,
                                   do_coarse_grids ? batches.lines_index_coarse :
                                                     batches.lines_index);
  }

  for (const AttributeRequest &request : requests) {
    if (!pbvh_attr_supported(request)) {
      continue;
    }
    const int i = batches.ensure_vbo(request, args);
    batch.vbos.append(i);
    const PBVHVbo &vbo = batches.vbos[i];

    GPU_batch_vertbuf_add(batch.tris, vbo.vert_buf, false);
    if (batch.lines) {
      GPU_batch_vertbuf_add(batch.lines, vbo.vert_buf, false);
    }
  }

  return batch;
}

static NodeBatch &ensure_batch(NodeBatches &batches,
                               const Span<AttributeRequest> requests,
                               const PBVH_GPU_Args &args,
                               const bool do_coarse_grids)
{
  return batches.batches.lookup_or_add_cb(build_key(requests, do_coarse_grids), [&]() {
    return create_batch(batches, requests, args, do_coarse_grids);
  });
}

void node_update(NodeBatches *batches, const PBVH_GPU_Args &args)
{
  batches->update(args);
}

void node_gpu_flush(NodeBatches *batches)
{
  gpu_flush(batches->vbos);
}

NodeBatches *node_create(const PBVH_GPU_Args &args)
{
  NodeBatches *batches = new NodeBatches(args);
  return batches;
}

void node_free(NodeBatches *batches)
{
  delete batches;
}

gpu::Batch *tris_get(NodeBatches *batches,
                     const Span<AttributeRequest> attrs,
                     const PBVH_GPU_Args &args,
                     bool do_coarse_grids)
{
  do_coarse_grids &= args.pbvh_type == PBVH_GRIDS;
  NodeBatch &batch = ensure_batch(*batches, attrs, args, do_coarse_grids);
  return batch.tris;
}

gpu::Batch *lines_get(NodeBatches *batches,
                      const Span<AttributeRequest> attrs,
                      const PBVH_GPU_Args &args,
                      bool do_coarse_grids)
{
  do_coarse_grids &= args.pbvh_type == PBVH_GRIDS;
  NodeBatch &batch = ensure_batch(*batches, attrs, args, do_coarse_grids);
  return batch.lines;
}

void update_pre(NodeBatches *batches, const PBVH_GPU_Args &args)
{
  batches->update_pre(args);
}

int material_index_get(NodeBatches *batches)
{
  return batches->material_index;
}

struct ViewportRequest {
  Set<AttributeRequest> attributes;
  bool use_coarse_grids;
};

struct PBVHDrawData {
  Vector<gpu::IndexBuf *> lines_ibos;
  Vector<gpu::IndexBuf *> tris_ibos;
  Map<AttributeRequest, Vector<gpu::VertBuf *>> attribute_vbos;

  Vector<gpu::Batch *> lines_batches;
  Map<ViewportRequest, Vector<gpu::Batch *>> tris_batches;
};

static void clear_all_data(PBVHDrawData &batches)
{
  batches.lines_ibos.clear();
  batches.tris_ibos.clear();
  batches.attribute_vbos.clear();

  batches.lines_batches.clear();
  batches.tris_batches.clear();
}

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

static GPUVertFormat attribute_format(const StringRefNull name, const eCustomDataType data_type)
{
  GPUVertFormat format = draw::init_format_for_attribute(data_type, "data");

  bool is_render, is_active;
  const char *prefix = "a";

  if (CD_TYPE_AS_MASK(data_type) & CD_MASK_COLOR_ALL) {
    prefix = "c";
    is_active = StringRef(args.active_color) == name;
    is_render = StringRef(args.render_color) == name;
  }
  if (data_type == CD_PROP_FLOAT2) {
    prefix = "u";
    is_active = StringRef(CustomData_get_active_layer_name(cdata, data_type)) == name;
    is_render = StringRef(CustomData_get_render_layer_name(cdata, data_type)) == name;
  }

  DRW_cdlayer_attr_aliases_add(&format, prefix, data_type, name.c_str(), is_render, is_active);
  return format;
}

static GPUVertFormat format_for_request(const Object &object, const AttributeRequest &request)
{
  const PBVH &pbvh = *object.sculpt->pbvh;
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
    return attribute_format(attr.name, attr.type);
  }
}

static void ensure_vbos_allocation_size(const Object &object,
                                        const Span<const PBVHNode *> nodes,
                                        const IndexMask &nodes_to_update,
                                        const AttributeRequest &request,
                                        const MutableSpan<gpu::VertBuf *> vbos)
{
  const SculptSession &ss = *object.sculpt;
  const PBVH &pbvh = *ss.pbvh;
  const GPUVertFormat format = format_for_request(object, request);
  switch (BKE_pbvh_type(pbvh)) {
    case PBVH_FACES: {
      const Mesh &mesh = *static_cast<Mesh *>(object.data);  // TODO
      const Span<int> tri_faces = mesh.corner_tri_faces();
      const bke::AttributeAccessor attributes = mesh.attributes();
      const VArraySpan hide_poly = *attributes.lookup<bool>(".hide_poly", bke::AttrDomain::Face);
      nodes_to_update.foreach_index(GrainSize(16), [&](const int i) {
        if (!vbos[i]) {
          vbos[i] = GPU_vertbuf_create_with_format(format);
        }
        const Span<int> tris = bke::pbvh::node_tri_indices(*nodes[i]);
        const int verts_num = count_visible_tris_mesh(tris, tri_faces, hide_poly) * 3;
        if (GPU_vertbuf_get_vertex_len(vbos[i]) != verts_num) {
          GPU_vertbuf_data_alloc(*vbos[i], verts_num);
        }
      });
      break;
    }
    case PBVH_GRIDS: {
      const SubdivCCG &subdiv_ccg = *ss.subdiv_ccg;
      const BitGroupVector<> &grid_hidden = subdiv_ccg.grid_hidden;
      const CCGKey key = *BKE_pbvh_get_grid_key(pbvh);
      nodes_to_update.foreach_index(GrainSize(16), [&](const int i) {
        if (!vbos[i]) {
          vbos[i] = GPU_vertbuf_create_with_format(format);
        }
        const int verts_per_grid = square_i(key.grid_size - 1) * 4;
        const int verts_num = bke::pbvh::node_grid_indices(*nodes[i]).size() * verts_per_grid;
        if (GPU_vertbuf_get_vertex_len(vbos[i]) != verts_num) {
          GPU_vertbuf_data_alloc(*vbos[i], verts_num);
        }
      });
      break;
    }
    case PBVH_BMESH: {
      const BMesh &bm = *ss.bm;
      nodes_to_update.foreach_index(GrainSize(16), [&](const int i) {
        if (!vbos[i]) {
          vbos[i] = GPU_vertbuf_create_with_format(format);
        }
        const Set<BMFace *, 0> faces = BKE_pbvh_bmesh_node_faces(const_cast<PBVHNode *>(nodes[i]));
        const int verts_num = count_visible_tris_bmesh(faces) * 3;
        if (GPU_vertbuf_get_vertex_len(vbos[i]) != verts_num) {
          GPU_vertbuf_data_alloc(*vbos[i], verts_num);
        }
      });
      break;
    }
  }
}

static void fill_vbos(const Object &object,
                      const Span<const PBVHNode *> nodes,
                      const IndexMask &nodes_to_update,
                      const AttributeRequest &request,
                      const MutableSpan<gpu::VertBuf *> vbos)
{
  const PBVH &pbvh = *object.sculpt->pbvh;
  if (const CustomRequest *request_type = std::get_if<CustomRequest>(&request)) {
    switch (*request_type) {
      case CustomRequest::Position:
        switch (BKE_pbvh_type(pbvh)) {
          case PBVH_FACES: {
            const Mesh &mesh = *static_cast<const Mesh *>(object.data);
            const Span<float3> vert_positions = BKE_pbvh_get_vert_positions(pbvh);
            const Span<int> corner_verts = mesh.corner_verts();
            const Span<int3> corner_tris;  // TODO
            const Span<int> tri_faces = mesh.corner_tri_faces();
            const bke::AttributeAccessor attributes = mesh.attributes();
            const VArraySpan hide_poly = *attributes.lookup<bool>(".hide_poly",
                                                                  bke::AttrDomain::Face);
            nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {
              extract_data_vert_faces<float3>(corner_verts,
                                              corner_tris,
                                              tri_faces,
                                              hide_poly,
                                              vert_positions,
                                              bke::pbvh::node_tri_indices(*nodes[i]),
                                              *vbos[i]);
            });
            break;
          }
          case PBVH_GRIDS: {
            break;
          }
          case PBVH_BMESH: {
            break;
          }
        }
        break;
      case CustomRequest::Normal:
        switch (BKE_pbvh_type(pbvh)) {
          case PBVH_FACES: {
            const Mesh &mesh = *static_cast<const Mesh *>(object.data);
            const Span<float3> vert_normals = BKE_pbvh_get_vert_normals(pbvh);
            const Span<float3> face_normals = BKE_pbvh_get_face_normals(pbvh);  // TODO
            const Span<int> corner_verts = mesh.corner_verts();
            const Span<int3> corner_tris;  // TODO
            const Span<int> tri_faces = mesh.corner_tri_faces();
            const bke::AttributeAccessor attributes = mesh.attributes();
            const VArraySpan sharp_faces = *attributes.lookup<bool>(".sharp_face",
                                                                    bke::AttrDomain::Face);
            const VArraySpan hide_poly = *attributes.lookup<bool>(".hide_poly",
                                                                  bke::AttrDomain::Face);
            nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {
              fill_vbo_normal_faces(corner_verts,
                                    corner_tris,
                                    tri_faces,
                                    hide_poly,
                                    sharp_faces,
                                    vert_normals,
                                    face_normals,
                                    bke::pbvh::node_tri_indices(*nodes[i]),
                                    *vbos[i]);
            });
            break;
          }
          case PBVH_GRIDS: {
            break;
          }
          case PBVH_BMESH: {
            break;
          }
        }
        break;
      case CustomRequest::Mask:
        switch (BKE_pbvh_type(pbvh)) {
          case PBVH_FACES: {
            const Mesh &mesh = *static_cast<const Mesh *>(object.data);
            const Span<int> corner_verts = mesh.corner_verts();
            const Span<int3> corner_tris;  // TODO
            const Span<int> tri_faces = mesh.corner_tri_faces();
            const bke::AttributeAccessor attributes = mesh.attributes();
            const VArraySpan hide_poly = *attributes.lookup<bool>(".hide_poly",
                                                                  bke::AttrDomain::Face);
            const VArraySpan mask = *attributes.lookup<float>(".sculpt_mask",
                                                              bke::AttrDomain::Point);
            nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {
              extract_data_vert_faces<float>(corner_verts,
                                             corner_tris,
                                             tri_faces,
                                             hide_poly,
                                             mask,
                                             bke::pbvh::node_tri_indices(*nodes[i]),
                                             *vbos[i]);
            });
            break;
          }
          case PBVH_GRIDS: {
            break;
          }
          case PBVH_BMESH: {
            break;
          }
        }
        break;
      case CustomRequest::FaceSet:
        switch (BKE_pbvh_type(pbvh)) {
          case PBVH_FACES: {
            const Mesh &mesh = *static_cast<const Mesh *>(object.data);
            const Span<int> corner_verts = mesh.corner_verts();
            const Span<int3> corner_tris;  // TODO
            const Span<int> tri_faces = mesh.corner_tri_faces();
            const bke::AttributeAccessor attributes = mesh.attributes();
            const VArraySpan hide_poly = *attributes.lookup<bool>(".hide_poly",
                                                                  bke::AttrDomain::Face);
            const VArraySpan face_sets = *attributes.lookup<int>(".sculpt_face_set",
                                                                 bke::AttrDomain::Face);
            nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {
              fill_vbo_face_sets_mesh(corner_tris,
                                      tri_faces,
                                      hide_poly,
                                      face_sets,
                                      bke::pbvh::node_tri_indices(*nodes[i]),
                                      *vbos[i]);
            });
            break;
          }
          case PBVH_GRIDS: {
            break;
          }
          case PBVH_BMESH: {
            break;
          }
        }
        break;
    }
  }
  else {
    const GenericRequest &attr = std::get<GenericRequest>(request);
    const StringRefNull name = attr.name;
    const bke::AttrDomain domain = attr.domain;
    const eCustomDataType data_type = attr.type;
    const GPUVertFormat format = attribute_format(name, data_type);

    switch (BKE_pbvh_type(pbvh)) {
      case PBVH_FACES:
        break;
      case PBVH_GRIDS:
        break;
      case PBVH_BMESH:
        break;
    }
  }
}

static Span<gpu::VertBuf *> ensure_vbos(const Object &object,
                                        const Span<PBVHNode *> nodes,
                                        const IndexMask &nodes_to_update,
                                        const AttributeRequest &attr,
                                        PBVHDrawData &draw_data)
{
  Vector<gpu::VertBuf *> &vbos = draw_data.attribute_vbos.lookup_or_add_default(attr);
  vbos.resize(nodes.size(), nullptr);

  ensure_vbos_allocation_size(object, nodes, nodes_to_update, attr, vbos);
  fill_vbos(object, nodes, nodes_to_update, attr, vbos);

  return vbos;
}

static Span<gpu::IndexBuf *> ensure_tris_ibos(const Object &object,
                                              const Span<const PBVHNode *> nodes,
                                              const IndexMask &nodes_to_update,
                                              PBVHDrawData &draw_data)
{
  const PBVH &pbvh = *object.sculpt->pbvh;
  draw_data.tris_ibos.resize(nodes.size(), nullptr);
  switch (BKE_pbvh_type(pbvh)) {
    case PBVH_FACES:
      break;
    case PBVH_GRIDS:
      nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {

      });
      break;
    case PBVH_BMESH:
      break;
  }
}

Span<gpu::Batch *> ensure_tris_batches(const ViewportRequest &request,
                                       const Object &object,
                                       const bool update_only_visible,
                                       PBVHDrawData &draw_data)
{
  const PBVH &pbvh = *object.sculpt->pbvh;
  const bool update_only_visible = false;  // TODO
  const IndexMask nodes_to_update;         // TODO

  Vector<PBVHNode *> nodes = bke::pbvh::search_gather(const_cast<PBVH &>(pbvh), {});

  Vector<Span<gpu::VertBuf *>> requested_vbos;
  for (const AttributeRequest &attr : request.attributes) {
    requested_vbos.append(ensure_vbos(object, nodes, nodes_to_update, attr, draw_data));
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
}

static Span<gpu::IndexBuf *> ensure_lines_ibos(const Object &object,
                                               const Span<const PBVHNode *> nodes,
                                               const IndexMask &nodes_to_update,
                                               PBVHDrawData &draw_data)
{
  const PBVH &pbvh = *object.sculpt->pbvh;
  draw_data.tris_ibos.resize(nodes.size(), nullptr);
  MutableSpan<gpu::IndexBuf *> ibos = draw_data.tris_ibos;

  IndexMaskMemory memory;
  const IndexMask ibos_to_update = IndexMask::from_predicate(
      nodes_to_update, GrainSize(128), memory, [&](const int i) { return ibos[i] == nullptr; });

  switch (BKE_pbvh_type(pbvh)) {
    case PBVH_FACES: {
      const Mesh &mesh = *static_cast<const Mesh *>(object.data);
      const Span<int2> edges = mesh.edges();
      const Span<int> corner_verts = mesh.corner_verts();
      const Span<int> corner_edges = mesh.corner_edges();
      const Span<int3> corner_tris = mesh.corner_tris();  // TODO
      const Span<int> tri_faces = mesh.corner_tri_faces();
      const bke::AttributeAccessor attributes = mesh.attributes();
      const VArraySpan hide_poly = *attributes.lookup<bool>(".hide_poly", bke::AttrDomain::Face);
      ibos_to_update.foreach_index(GrainSize(1), [&](const int i) {
        ibos[i] = calc_lines_index_mesh(edges,
                                        corner_verts,
                                        corner_edges,
                                        corner_tris,
                                        tri_faces,
                                        hide_poly,
                                        bke::pbvh::node_tri_indices(*nodes[i]));
      });
      break;
    }
    case PBVH_GRIDS: {
      break;
    }
    case PBVH_BMESH: {
      ibos_to_update.foreach_index(GrainSize(1), [&](const int i) {
        const int visible_faces_num = 100;
        ibos[i] = calc_lines_index_bmesh(
            BKE_pbvh_bmesh_node_faces(const_cast<PBVHNode *>(nodes[i])), visible_faces_num);
      });
      break;
    }
  }
}

Span<gpu::Batch *> ensure_lines_batches(const Object &object,
                                        const bool update_only_visible,
                                        PBVHDrawData &draw_data)
{
  const PBVH &pbvh = *object.sculpt->pbvh;

  Vector<PBVHNode *> nodes = bke::pbvh::search_gather(const_cast<PBVH &>(pbvh), {});
  const IndexMask nodes_to_update;  // TODO

  const Span<gpu::VertBuf *> position_vbos = ensure_vbos(
      object, nodes, nodes_to_update, CustomRequest::Position, draw_data);
  const Span<gpu::IndexBuf *> ibos = ensure_lines_ibos(object, nodes, nodes_to_update, draw_data);

  draw_data.lines_batches.resize(nodes.size(), nullptr);
  MutableSpan<gpu::Batch *> batches = draw_data.lines_batches;

  const Span<gpu::IndexBuf *> ibos = draw_data.lines_ibos;

  nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {
    if (!batches[i]) {
      GPU_batch_create(GPU_PRIM_LINES, nullptr, ibos[i]);
      GPU_batch_vertbuf_add(batches[i], position_vbos[i], false);
    }
    // TODO: Nodes that already existed but were updated?
  });
}

}  // namespace blender::draw::pbvh
