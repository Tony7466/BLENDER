/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw
 *
 * \brief Grease Pencil API for render engines
 */

#include "BKE_curves.hh"
#include "BKE_grease_pencil.h"
#include "BKE_grease_pencil.hh"

#include "BLI_task.hh"

#include "DNA_grease_pencil_types.h"

#include "DRW_engine.h"
#include "DRW_render.h"

#include "GPU_batch.h"

#include "draw_cache_impl.h"

#include "../engines/gpencil/gpencil_defines.h"
#include "../engines/gpencil/gpencil_shader_shared.h"

namespace blender::draw {

struct GreasePencilBatchCache {
  /** Instancing Data */
  GPUVertBuf *vbo;
  GPUVertBuf *vbo_col;
  /** Indices in material order, then stroke order with fill first.
   * Strokes can be individually rendered using `gps->runtime.stroke_start` and
   * `gps->runtime.fill_start`. */
  GPUIndexBuf *ibo;
  /** Batches */
  GPUBatch *geom_batch;
  /** Stroke lines only */
  GPUBatch *lines_batch;

  /** Cache is dirty. */
  bool is_dirty;
  /** Last cached frame. */
  int cache_frame;
};

/* -------------------------------------------------------------------- */
/** \name Vertex Formats
 * \{ */

/* MUST match the format below. */
struct GreasePencilStrokeVert {
  /** Position and radius packed in the same attribute. */
  float pos[3], radius;
  /** Material Index, Stroke Index, Point Index, Packed aspect + hardness + rotation. */
  int32_t mat, stroke_id, point_id, packed_asp_hard_rot;
  /** UV and opacity packed in the same attribute. */
  float uv_fill[2], u_stroke, opacity;
};

static GPUVertFormat *grease_pencil_stroke_format()
{
  static GPUVertFormat format = {0};
  if (format.attr_len == 0) {
    GPU_vertformat_attr_add(&format, "pos", GPU_COMP_F32, 4, GPU_FETCH_FLOAT);
    GPU_vertformat_attr_add(&format, "ma", GPU_COMP_I32, 4, GPU_FETCH_INT);
    GPU_vertformat_attr_add(&format, "uv", GPU_COMP_F32, 4, GPU_FETCH_FLOAT);
  }
  return &format;
}

/* MUST match the format below. */
struct GreasePencilColorVert {
  float vcol[4]; /* Vertex color */
  float fcol[4]; /* Fill color */
};

static GPUVertFormat *grease_pencil_color_format()
{
  static GPUVertFormat format = {0};
  if (format.attr_len == 0) {
    GPU_vertformat_attr_add(&format, "col", GPU_COMP_F32, 4, GPU_FETCH_FLOAT);
    GPU_vertformat_attr_add(&format, "fcol", GPU_COMP_F32, 4, GPU_FETCH_FLOAT);
  }
  return &format;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Internal Utilities
 * \{ */

static bool grease_pencil_batch_cache_valid(const GreasePencil &grease_pencil)
{
  BLI_assert(grease_pencil.runtime != nullptr);
  const GreasePencilBatchCache *cache = static_cast<GreasePencilBatchCache *>(
      grease_pencil.runtime->batch_cache);
  return (cache && cache->is_dirty == false);
}

static GreasePencilBatchCache *grease_pencil_batch_cache_init(GreasePencil &grease_pencil,
                                                              int cfra)
{
  BLI_assert(grease_pencil.runtime != nullptr);
  GreasePencilBatchCache *cache = static_cast<GreasePencilBatchCache *>(
      grease_pencil.runtime->batch_cache);
  if (!cache) {
    cache = MEM_new<GreasePencilBatchCache>(__func__);
    grease_pencil.runtime->batch_cache = cache;
  }
  else {
    grease_pencil.runtime->batch_cache = {};
  }

  cache->is_dirty = false;
  cache->cache_frame = cfra;

  return cache;
}

static void grease_pencil_batch_cache_clear(GreasePencil &grease_pencil)
{
  BLI_assert(grease_pencil.runtime != nullptr);
  GreasePencilBatchCache *cache = static_cast<GreasePencilBatchCache *>(
      grease_pencil.runtime->batch_cache);
  if (cache == nullptr) {
    return;
  }

  GPU_BATCH_DISCARD_SAFE(cache->lines_batch);
  GPU_BATCH_DISCARD_SAFE(cache->geom_batch);
  GPU_VERTBUF_DISCARD_SAFE(cache->vbo);
  GPU_VERTBUF_DISCARD_SAFE(cache->vbo_col);
  GPU_INDEXBUF_DISCARD_SAFE(cache->ibo);

  cache->is_dirty = true;
}

static GreasePencilBatchCache *grease_pencil_batch_cache_get(GreasePencil &grease_pencil, int cfra)
{
  BLI_assert(grease_pencil.runtime != nullptr);
  GreasePencilBatchCache *cache = static_cast<GreasePencilBatchCache *>(
      grease_pencil.runtime->batch_cache);
  if (!grease_pencil_batch_cache_valid(grease_pencil)) {
    grease_pencil_batch_cache_clear(grease_pencil);
    return grease_pencil_batch_cache_init(grease_pencil, cfra);
  }

  return cache;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Vertex Buffers
 * \{ */

static int count_cyclic_curves(bke::CurvesGeometry &curves)
{
  VArray<bool> cyclic = curves.cyclic();
  const CommonVArrayInfo info = cyclic.common_info();
  if (info.type == CommonVArrayInfo::Type::Single) {
    const bool single = *static_cast<const bool *>(info.data);
    return single ? curves.curves_num() : 0;
  }
  else if (info.type == CommonVArrayInfo::Type::Span) {
    const Span<bool> span(static_cast<const bool *>(info.data), cyclic.size());
    return span.count(true);
  }
  return threading::parallel_reduce(
      cyclic.index_range(),
      2048,
      0,
      [&](const IndexRange range, const int init) {
        /* Alternatively, this could use #materialize to retrieve many values at once. */
        int sum = init;
        for (const int64_t i : range) {
          if (cyclic[i]) {
            sum++;
          }
        }
        return sum;
      },
      [&](const int a, const int b) { return a + b; });
}

BLI_INLINE int32_t pack_rotation_aspect_hardness(float rot, float asp, float hard)
{
  int32_t packed = 0;
  /* Aspect uses 9 bits */
  float asp_normalized = (asp > 1.0f) ? (1.0f / asp) : asp;
  packed |= int32_t(unit_float_to_uchar_clamp(asp_normalized));
  /* Store if inversed in the 9th bit. */
  if (asp > 1.0f) {
    packed |= 1 << 8;
  }
  /* Rotation uses 9 bits */
  /* Rotation are in [-90°..90°] range, so we can encode the sign of the angle + the cosine
   * because the cosine will always be positive. */
  packed |= int32_t(unit_float_to_uchar_clamp(cosf(rot))) << 9;
  /* Store sine sign in 9th bit. */
  if (rot < 0.0f) {
    packed |= 1 << 17;
  }
  /* Hardness uses 8 bits */
  packed |= int32_t(unit_float_to_uchar_clamp(hard)) << 18;
  return packed;
}

static void grease_pencil_batches_ensure(GreasePencil &grease_pencil, int cfra)
{
  BLI_assert(grease_pencil.runtime != nullptr);
  GreasePencilBatchCache *cache = static_cast<GreasePencilBatchCache *>(
      grease_pencil.runtime->batch_cache);

  if (cache->vbo != nullptr) {
    return;
  }

  /* Should be discarded together. */
  BLI_assert(cache->vbo == nullptr && cache->ibo == nullptr);
  BLI_assert(cache->geom_batch == nullptr);

  /* First count how many vertices and triangles are needed for the whole object. */
  int total_points_num = 0;
  int total_triangles_num = 0;
  grease_pencil.foreach_visible_drawing(
      cfra, [&](GreasePencilDrawing &drawing, blender::bke::gpencil::Layer &layer) {
        bke::CurvesGeometry &curves = drawing.geometry.wrap();
        int num_cyclic = count_cyclic_curves(curves);
        /* One vertex is stored before and after as padding. Cyclic strokes have one extra
         * vertex.*/
        total_points_num += curves.points_num() + num_cyclic + curves.curves_num() * 2;
        total_triangles_num += (curves.points_num() + num_cyclic) * 2;
        total_triangles_num += drawing.runtime->triangles_cache.size();
      });

  GPUUsageType vbo_flag = GPU_USAGE_STATIC | GPU_USAGE_FLAG_BUFFER_TEXTURE_ONLY;
  /* Create VBOs. */
  GPUVertFormat *format = grease_pencil_stroke_format();
  GPUVertFormat *format_col = grease_pencil_color_format();
  cache->vbo = GPU_vertbuf_create_with_format_ex(format, vbo_flag);
  cache->vbo_col = GPU_vertbuf_create_with_format_ex(format_col, vbo_flag);
  /* Add extra space at the end of the buffer because of quad load. */
  GPU_vertbuf_data_alloc(cache->vbo, total_points_num + 2);
  GPU_vertbuf_data_alloc(cache->vbo_col, total_points_num + 2);

  GPUIndexBufBuilder ibo;
  MutableSpan<GreasePencilStrokeVert> verts = {
      static_cast<GreasePencilStrokeVert *>(GPU_vertbuf_get_data(cache->vbo)),
      GPU_vertbuf_get_vertex_len(cache->vbo)};
  MutableSpan<GreasePencilColorVert> cols = {
      static_cast<GreasePencilColorVert *>(GPU_vertbuf_get_data(cache->vbo_col)),
      GPU_vertbuf_get_vertex_len(cache->vbo_col)};
  /* Create IBO. */
  GPU_indexbuf_init(&ibo, GPU_PRIM_TRIS, total_triangles_num, 0xFFFFFFFFu);

  /* Fill buffers with data. */
  int v = 0;
  grease_pencil.foreach_visible_drawing(
      cfra, [&](GreasePencilDrawing &drawing, blender::bke::gpencil::Layer &layer) {
        bke::CurvesGeometry &curves = drawing.geometry.wrap();
        bke::AttributeAccessor attributes = curves.attributes();
        offset_indices::OffsetIndices<int> points_by_curve = curves.points_by_curve();
        const Span<float3> positions = curves.positions();
        const VArray<bool> cyclic = curves.cyclic();
        VArray<float> radii = attributes.lookup_or_default<float>(
            "radius", ATTR_DOMAIN_POINT, 1.0f);
        VArray<float> opacities = attributes.lookup_or_default<float>(
            "opacity", ATTR_DOMAIN_POINT, 1.0f);
        VArray<int> materials = attributes.lookup_or_default<int>(
            "material_index", ATTR_DOMAIN_CURVE, -1);

        auto populate_point = [&](int curve_i, int point_i, int v_start, int v) {
          copy_v3_v3(verts[v].pos, positions[point_i]);
          verts[v].radius = radii[point_i];
          verts[v].opacity = opacities[point_i];
          verts[v].point_id = v;
          verts[v].stroke_id = v_start;
          verts[v].mat = materials[curve_i] % GPENCIL_MATERIAL_BUFFER_LEN;

          /* TODO */
          verts[v].packed_asp_hard_rot = pack_rotation_aspect_hardness(0.0f, 1.0f, 1.0f);
          /* TODO */
          verts[v].u_stroke = 0;
          /* TODO */
          verts[v].uv_fill[0] = verts[v].uv_fill[1] = 0;

          /* TODO */
          copy_v4_v4(cols[v].vcol, float4(0.0f, 0.0f, 0.0f, 0.0f));
          copy_v4_v4(cols[v].fcol, float4(0.0f, 0.0f, 0.0f, 0.0f));

          /* TODO */
          cols[v].fcol[3] = (int(cols[v].fcol[3] * 10000.0f) * 10.0f) + 1.0f;

          int v_mat = (v << GP_VERTEX_ID_SHIFT) | GP_IS_STROKE_VERTEX_BIT;
          GPU_indexbuf_add_tri_verts(&ibo, v_mat + 0, v_mat + 1, v_mat + 2);
          GPU_indexbuf_add_tri_verts(&ibo, v_mat + 2, v_mat + 1, v_mat + 3);
        };

        int t = 0;
        for (const int curve_i : curves.curves_range()) {
          IndexRange points = points_by_curve[curve_i];
          const bool is_cyclic = cyclic[curve_i];
          const int v_start = v;
          int num_triangles = 0;

          /* First point is not drawn. */
          verts[v].mat = -1;
          v++;

          if (points.size() > 3) {
            num_triangles = points.size() - 2;
            for (const int tri_i : IndexRange(num_triangles)) {
              uint3 tri = drawing.runtime->triangles_cache[t + tri_i];
              GPU_indexbuf_add_tri_verts(&ibo,
                                         (v + tri.x) << GP_VERTEX_ID_SHIFT,
                                         (v + tri.y) << GP_VERTEX_ID_SHIFT,
                                         (v + tri.z) << GP_VERTEX_ID_SHIFT);
            }
          }

          for (const int point_i : points) {
            populate_point(curve_i, point_i, v_start, v);
            v++;
          }

          if (is_cyclic) {
            populate_point(curve_i, points.first(), v_start, v);
            v++;
          }

          /* Last point is not drawn. */
          verts[v].mat = -1;
          v++;

          t += num_triangles;
        }
      });

  /* Mark last 2 verts as invalid. */
  verts[total_points_num + 0].mat = -1;
  verts[total_points_num + 1].mat = -1;
  /* Also mark first vert as invalid. */
  verts[0].mat = -1;

  /* Finish the IBO. */
  cache->ibo = GPU_indexbuf_build(&ibo);
  /* Create the batches */
  cache->geom_batch = GPU_batch_create(GPU_PRIM_TRIS, cache->vbo, cache->ibo);
  /* Allow creation of buffer texture. */
  GPU_vertbuf_use(cache->vbo);
  GPU_vertbuf_use(cache->vbo_col);

  cache->is_dirty = false;
}

/** \} */

}  // namespace blender::draw

void DRW_grease_pencil_batch_cache_dirty_tag(GreasePencil *grease_pencil, int mode)
{
  using namespace blender::draw;
  BLI_assert(grease_pencil->runtime != nullptr);
  GreasePencilBatchCache *cache = static_cast<GreasePencilBatchCache *>(
      grease_pencil->runtime->batch_cache);
  if (cache == nullptr) {
    return;
  }
  switch (mode) {
    case BKE_GREASEPENCIL_BATCH_DIRTY_ALL:
      cache->is_dirty = true;
      break;
    default:
      BLI_assert_unreachable();
  }
}

void DRW_grease_pencil_batch_cache_validate(GreasePencil *grease_pencil)
{
  using namespace blender::draw;
  BLI_assert(grease_pencil->runtime != nullptr);
  if (!grease_pencil_batch_cache_valid(*grease_pencil)) {
    grease_pencil_batch_cache_clear(*grease_pencil);
    /* TODO: pass correct frame here? */
    grease_pencil_batch_cache_init(*grease_pencil, 0);
  }
}

void DRW_grease_pencil_batch_cache_free(GreasePencil *grease_pencil)
{
  using namespace blender::draw;
  grease_pencil_batch_cache_clear(*grease_pencil);
  MEM_delete(static_cast<GreasePencilBatchCache *>(grease_pencil->runtime->batch_cache));
  grease_pencil->runtime->batch_cache = nullptr;
}

GPUBatch *DRW_cache_grease_pencil_get(Object *ob, int cfra)
{
  using namespace blender::draw;
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(ob->data);
  GreasePencilBatchCache *cache = grease_pencil_batch_cache_get(grease_pencil, cfra);
  grease_pencil_batches_ensure(grease_pencil, cfra);

  return cache->geom_batch;
}

GPUVertBuf *DRW_cache_grease_pencil_position_buffer_get(Object *ob, int cfra)
{
  using namespace blender::draw;
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(ob->data);
  GreasePencilBatchCache *cache = grease_pencil_batch_cache_get(grease_pencil, cfra);
  grease_pencil_batches_ensure(grease_pencil, cfra);

  return cache->vbo;
}

GPUVertBuf *DRW_cache_grease_pencil_color_buffer_get(Object *ob, int cfra)
{
  using namespace blender::draw;
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(ob->data);
  GreasePencilBatchCache *cache = grease_pencil_batch_cache_get(grease_pencil, cfra);
  grease_pencil_batches_ensure(grease_pencil, cfra);

  return cache->vbo_col;
}
