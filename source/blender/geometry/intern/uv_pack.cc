/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup eduv
 */

#include "GEO_uv_pack.hh"

#include "BLI_array.hh"
#include "BLI_boxpack_2d.h"
#include "BLI_convexhull_2d.h"
#include "BLI_listbase.h"
#include "BLI_math.h"
#include "BLI_rect.h"
#include "BLI_vector.hh"

#include "DNA_meshdata_types.h"
#include "DNA_scene_types.h"
#include "DNA_space_types.h"

#include "MEM_guardedalloc.h"

namespace blender::geometry {

static float signedDistanceToEdge(float2 probe, float2 uva, float2 uvb)
{
  float2 edge = uvb - uva;
  float2 side = probe - uva;

  const float edge_length_squared = blender::math::length_squared(edge);
  if (edge_length_squared < 1e-40f) {
    return blender::math::length(side);
  }
  float cross = (edge.x * side.y - edge.y * side.x);
  if (cross <= 0) {
    /* Result will be non-positive, can be unbounded. */
    return cross / sqrtf(edge_length_squared);
  }

  /* Distance to endpoint or closest point on edge. */
  float close[2];
  closest_to_line_segment_v2(close, probe, uva, uvb);
  return len_v2v2(close, probe);
}

void PackIsland::addTriangle(const float2 uv0, const float2 uv1, const float2 uv2)
{
  /* Be careful with winding. */
  if (signedDistanceToEdge(uv0, uv1, uv2) < 0.0f) {
    triangleVertices.append(uv0);
    triangleVertices.append(uv1);
    triangleVertices.append(uv2);
  }
  else {
    triangleVertices.append(uv0);
    triangleVertices.append(uv2);
    triangleVertices.append(uv1);
  }
}

UVPackIsland_Params::UVPackIsland_Params()
{
  /* TEMPORARY, set every thing to "zero" for backwards compatibility. */
  rotate = false;
  only_selected_uvs = false;
  only_selected_faces = false;
  use_seams = false;
  correct_aspect = false;
  ignore_pinned = false;
  pin_unselected = false;
  margin = 0.001f;
  margin_method = ED_UVPACK_MARGIN_SCALED;
  udim_base_offset[0] = 0.0f;
  udim_base_offset[1] = 0.0f;
  shape_method = ED_UVPACK_SHAPE_AABB;
}

/* Compact representation for AABB packers. */
class UVAABBIsland {
 public:
  float2 uv_diagonal;
  float2 uv_placement;
  int64_t index;
};

/**
 * Pack AABB islands using the "Alpaca" strategy, with no rotation.
 *
 * Each box is packed into an "L" shaped region, gradually filling up space.
 * "Alpaca" is a pun, as it's pronounced the same as "L-Packer" in English.
 *
 * In theory, alpaca_turbo should be the fastest non-trivial packer, hence the "turbo" suffix.
 *
 * Technically, the algorithm here is only `O(n)`, In practice, to get reasonable results,
 * the input must be pre-sorted, which costs an additional `O(nlogn)` time complexity.
 */
static void pack_islands_alpaca_turbo(const Span<UVAABBIsland *> islands,
                                      float *r_max_u,
                                      float *r_max_v)
{
  /* Exclude an initial AABB near the origin. */
  float next_u1 = *r_max_u;
  float next_v1 = *r_max_v;
  bool zigzag = next_u1 < next_v1; /* Horizontal or Vertical strip? */

  float u0 = zigzag ? next_u1 : 0.0f;
  float v0 = zigzag ? 0.0f : next_v1;

  /* Visit every island in order. */
  for (UVAABBIsland *island : islands) {
    float dsm_u = island->uv_diagonal.x;
    float dsm_v = island->uv_diagonal.y;

    bool restart = false;
    if (zigzag) {
      restart = (next_v1 < v0 + dsm_v);
    }
    else {
      restart = (next_u1 < u0 + dsm_u);
    }
    if (restart) {
      /* We're at the end of a strip. Restart from U axis or V axis. */
      zigzag = next_u1 < next_v1;
      u0 = zigzag ? next_u1 : 0.0f;
      v0 = zigzag ? 0.0f : next_v1;
    }

    /* Place the island. */
    island->uv_placement.x = u0;
    island->uv_placement.y = v0;
    if (zigzag) {
      /* Move upwards. */
      v0 += dsm_v;
      next_u1 = max_ff(next_u1, u0 + dsm_u);
      next_v1 = max_ff(next_v1, v0);
    }
    else {
      /* Move sideways. */
      u0 += dsm_u;
      next_v1 = max_ff(next_v1, v0 + dsm_v);
      next_u1 = max_ff(next_u1, u0);
    }
  }

  /* Write back total pack AABB. */
  *r_max_u = next_u1;
  *r_max_v = next_v1;
}

class Occupancy {
 public:
  Occupancy();

  void increaseScale();
  bool traceTriangle(const float2 &uv0,
                     const float2 &uv1,
                     const float2 &uv2,
                     const float margin,
                     const bool write);

  bool traceIsland(PackIsland *island,
                   const float scale,
                   const float margin,
                   const float2 origin,
                   float u,
                   float v,
                   const bool write);

  int bitmapRadix;
  float bitmapScale;
  float bitmapScaleReciprocal;
  blender::Array<bool> bitmap;

 private:
  float2 witness;
  int triangleWitness;
};

Occupancy::Occupancy() : bitmapRadix(512), bitmap(bitmapRadix * bitmapRadix, false)
{
  bitmapScale = 1.0f / bitmapRadix;
  bitmapScaleReciprocal = 1.0f / bitmapScale;
  witness.x = -1;
  witness.y = -1;
  triangleWitness = 0;
}

void Occupancy::increaseScale()
{
  bitmapScale *= 2.0f;
  bitmapScaleReciprocal = 1.0f / bitmapScale;
  for (int i = 0; i < bitmapRadix * bitmapRadix; i++) {
    bitmap[i] = false;
  }
  witness.x = -1;
  witness.y = -1;
}

static bool pointInsideFatTriangle(
    float2 probe, float2 uv0, float2 uv1, float2 uv2, const float epsilon)
{
  const float dist01 = signedDistanceToEdge(probe, uv0, uv1);
  const float dist12 = signedDistanceToEdge(probe, uv1, uv2);
  const float dist20 = signedDistanceToEdge(probe, uv2, uv0);

  if (dist01 <= epsilon && dist12 <= epsilon && dist20 <= epsilon) {
    return true; /* Inside. */
  }
  return false; /* Outside. */
}

bool Occupancy::traceTriangle(
    const float2 &uv0, const float2 &uv1, const float2 &uv2, const float margin, const bool write)
{
  const float x0 = min_fff(uv0.x, uv1.x, uv2.x);
  const float y0 = min_fff(uv0.y, uv1.y, uv2.y);
  const float x1 = max_fff(uv0.x, uv1.x, uv2.x);
  const float y1 = max_fff(uv0.y, uv1.y, uv2.y);
  int ix0 = std::max(int(floorf((x0 - margin) * bitmapScaleReciprocal)), 0);
  int iy0 = std::max(int(floorf((y0 - margin) * bitmapScaleReciprocal)), 0);
  int ix1 = std::min(int(floorf((x1 + margin) * bitmapScaleReciprocal + 1)), bitmapRadix);
  int iy1 = std::min(int(floorf((y1 + margin) * bitmapScaleReciprocal + 1)), bitmapRadix);

  const float2 uv0s = uv0 * bitmapScaleReciprocal;
  const float2 uv1s = uv1 * bitmapScaleReciprocal;
  const float2 uv2s = uv2 * bitmapScaleReciprocal;

  float epsilon = 0.70710678118f; /* sqrt(0.5) pixels. */
  epsilon = std::max(epsilon, margin * bitmapScaleReciprocal - epsilon);

  if (!write) {
    if (ix0 <= witness.x && witness.x < ix1) {
      if (iy0 <= witness.y && witness.y < iy1) {
        if (pointInsideFatTriangle(witness, uv0s, uv1s, uv2s, epsilon)) {
          return true;
        }
      }
    }
  }

  /* Iterate in opposite direction to outer search to improve witness effectiveness. */
  for (int y = iy1 - 1; y >= iy0; --y) {
    for (int x = ix1 - 1; x >= ix0; --x) {
      bool *hotspot = &bitmap[y * bitmapRadix + x];
      if (!write && !*hotspot) {
        continue;
      }
      float2 probe(x, y);
      if (pointInsideFatTriangle(probe, uv0s, uv1s, uv2s, epsilon)) {
        if (write) {
          *hotspot = true;
        }
        else {
          witness = probe;
          return true;
        }
      }
    }
  }
  return false;
}

bool Occupancy::traceIsland(PackIsland *island,
                            const float scale,
                            const float margin,
                            const float2 origin,
                            float u,
                            float v,
                            const bool write)
{

  if (!write) {
    if (u < 0.0f || v < 0.0f) {
      return true;
    }
  }
  int s = int(island->triangleVertices.size());
  for (int i = 0; i < s; i += 3) {
    int j = (i + triangleWitness) % s;
    bool occupied = traceTriangle(
        (island->triangleVertices[j] - origin) * scale + float2(u, v),
        (island->triangleVertices[j + 1] - origin) * scale + float2(u, v),
        (island->triangleVertices[j + 2] - origin) * scale + float2(u, v),
        margin,
        write);

    if (!write && occupied) {
      triangleWitness = j;
      return true;
    }
  }
  return false;
}

/**
 * Pack irregular islands using the "xatlas" strategy, with no rotation.
 *
 * Loosely based on the 'xatlas' code by Jonathan Young
 * from https://github.com/jpcy/xatlas
 *
 * A brute force packer (BFPacker) with accelerators:
 * - Uses a Bitmap Occupancy class.
 * - Uses a "Witness".
 * - Write with `margin * 2`, read with `margin == 0`.
 * - Lazy resetting of BF search.
 */
static void pack_island_xatlas(const Span<UVAABBIsland *> island_indices,
                               const Span<PackIsland *> islands,
                               BoxPack *box_array,
                               const float scale,
                               const float margin,
                               float *r_max_u,
                               float *r_max_v)
{
  Occupancy occupancy;
  float max_u = 0.0f;
  float max_v = 0.0f;

  int s0 = 0;
  int i = 0;
  while (i < island_indices.size()) {
    PackIsland *island = islands[island_indices[i]->index];
    bool found = false;
    float best_u = 1.2f;
    float best_v = 0.5f;

    if (i < 128 || (i & 31) == 16) {
      s0 = 0;
    }

    float2 origin(island->bounds_rect.xmin, island->bounds_rect.ymin);
    for (int s = s0; s < occupancy.bitmapRadix; s++) {
      for (int t = 0; t <= s; t++) {
        float u = s * occupancy.bitmapScale - BLI_rctf_size_x(&island->bounds_rect) * scale;
        float v = t * occupancy.bitmapScale - BLI_rctf_size_y(&island->bounds_rect) * scale;

        if (!occupancy.traceIsland(island, scale, 0.0f, origin, u, v, false)) {
          best_u = u;
          best_v = v;
          s0 = std::max(s0, s - 25); /* TODO: Tune me. */
          s = t = 1048576;           /* Fixme. */
          found = true;
          break;
        }
        u = t * occupancy.bitmapScale - BLI_rctf_size_x(&island->bounds_rect) * scale;
        v = s * occupancy.bitmapScale - BLI_rctf_size_y(&island->bounds_rect) * scale;
        if (!occupancy.traceIsland(island, scale, 0.0f, origin, u, v, false)) {
          best_u = u;
          best_v = v;
          s0 = std::max(s0, s - 25); /* TODO: Tune me. */
          s = t = 1048576;           /* Fixme. */
          found = true;
          break;
        }
      }
    }

    if (best_u + BLI_rctf_size_x(&island->bounds_rect) * scale + 2 * margin >
        0.75f * occupancy.bitmapRadix * occupancy.bitmapScale) {
      found = false;
    }

    if (!found) {
      s0 = 0;
      occupancy.increaseScale();
      for (int j = 0; j < i; j++) {
        PackIsland *island = islands[island_indices[j]->index];
        BoxPack *box = box_array + j;
        float2 origin(island->bounds_rect.xmin, island->bounds_rect.ymin);
        occupancy.traceIsland(island, scale, 2.0f * margin, origin, box->x, box->y, true);
      }
      continue;
    }

    BoxPack *box = box_array + i;
    box->x = best_u;
    box->y = best_v;
    max_u = std::max(best_u + BLI_rctf_size_x(&island->bounds_rect) * scale + 2 * margin, max_u);
    max_v = std::max(best_v + BLI_rctf_size_y(&island->bounds_rect) * scale + 2 * margin, max_v);
    occupancy.traceIsland(island, scale, 2.0f * margin, origin, box->x, box->y, true);
    i++; /* Next island. */
  }

  *r_max_u = max_u;
  *r_max_v = max_v;
}

static float pack_islands_scale_margin(const Span<PackIsland *> islands,
                                       BoxPack *box_array,
                                       const float scale,
                                       const float margin,
                                       const eUVPackIsland_ShapeMethod shape_method)
{
  /* #BLI_box_pack_2d produces layouts with high packing efficiency, but has `O(n^3)`
   * time complexity, causing poor performance if there are lots of islands. See: #102843.
   * #pack_islands_alpaca_turbo is designed to be the fastest packing method, `O(nlogn)`,
   * but has poor packing efficiency if the AABBs have a spread of sizes and aspect ratios.
   * Here, we merge the best properties of both packers into one combined packer.
   *
   * The free tuning parameter, `alpaca_cutoff` will determine how many islands are packed
   * using each method.
   *
   * The current strategy is:
   * - Sort islands in size order.
   * - Call #BLI_box_pack_2d on the first `alpaca_cutoff` islands.
   * - Call #pack_islands_alpaca_turbo on the remaining islands.
   * - Combine results.
   */

  /* First, copy information from our input into the AABB structure. */
  Array<UVAABBIsland *> aabbs(islands.size());
  for (const int64_t i : islands.index_range()) {
    PackIsland *pack_island = islands[i];
    UVAABBIsland *aabb = new UVAABBIsland();
    aabb->index = i;
    aabb->uv_diagonal.x = BLI_rctf_size_x(&pack_island->bounds_rect) * scale + 2 * margin;
    aabb->uv_diagonal.y = BLI_rctf_size_y(&pack_island->bounds_rect) * scale + 2 * margin;
    aabbs[i] = aabb;
  }

  /* Sort from "biggest" to "smallest". */
  std::stable_sort(aabbs.begin(), aabbs.end(), [](const UVAABBIsland *a, const UVAABBIsland *b) {
    /* Just choose the AABB with larger rectangular area. */
    return b->uv_diagonal.x * b->uv_diagonal.y < a->uv_diagonal.x * a->uv_diagonal.y;
  });

  /* Partition island_vector, largest will go to box_pack, the rest alpaca_turbo.
   * See discussion above for details. */
  int64_t alpaca_cutoff = int64_t(1024); /* TODO: Tune constant. */
  int64_t max_box_pack = std::min(alpaca_cutoff, islands.size());

  /* Prepare for box_pack_2d. */
  for (const int64_t i : islands.index_range()) {
    UVAABBIsland *aabb = aabbs[i];
    BoxPack *box = &box_array[i];
    box->index = int(aabb->index);
    box->w = aabb->uv_diagonal.x;
    box->h = aabb->uv_diagonal.y;
  }

  /* Call box_pack_2d (slow for large N.) */
  float max_u = 0.0f;
  float max_v = 0.0f;
  if (shape_method == ED_UVPACK_SHAPE_CONCAVE_HOLE) {
    pack_island_xatlas(aabbs.as_span().take_front(max_box_pack),
                       islands,
                       box_array,
                       scale,
                       margin,
                       &max_u,
                       &max_v);
  }
  else {
    BLI_box_pack_2d(box_array, int(max_box_pack), &max_u, &max_v);
  }

  /* At this stage, `max_u` and `max_v` contain the box_pack UVs. */

  /* Call Alpaca. */
  pack_islands_alpaca_turbo(aabbs.as_span().drop_front(max_box_pack), &max_u, &max_v);

  /* Write back Alpaca UVs. */
  for (int64_t index = max_box_pack; index < aabbs.size(); index++) {
    UVAABBIsland *aabb = aabbs[index];
    BoxPack *box = &box_array[index];
    box->x = aabb->uv_placement.x;
    box->y = aabb->uv_placement.y;
  }

  /* Memory management. */
  for (int64_t i : aabbs.index_range()) {
    UVAABBIsland *aabb = aabbs[i];
    aabbs[i] = nullptr;
    delete aabb;
  }

  return std::max(max_u, max_v);
}

static float pack_islands_margin_fraction(const Span<PackIsland *> &island_vector,
                                          BoxPack *box_array,
                                          const float margin_fraction,
                                          const eUVPackIsland_ShapeMethod shape_method)
{
  /*
   * Root finding using a combined search / modified-secant method.
   * First, use a robust search procedure to bracket the root within a factor of 10.
   * Then, use a modified-secant method to converge.
   *
   * This is a specialized solver using domain knowledge to accelerate convergence. */

  float scale_low = 0.0f;
  float value_low = 0.0f;
  float scale_high = 0.0f;
  float value_high = 0.0f;
  float scale_last = 0.0f;

  /* Scaling smaller than `min_scale_roundoff` is unlikely to fit and
   * will destroy information in existing UVs. */
  float min_scale_roundoff = 1e-5f;

  /* Certain inputs might have poor convergence properties.
   * Use `max_iteration` to prevent an infinite loop. */
  int max_iteration = 25;
  for (int iteration = 0; iteration < max_iteration; iteration++) {
    float scale = 1.0f;

    if (iteration == 0) {
      BLI_assert(iteration == 0);
      BLI_assert(scale == 1.0f);
      BLI_assert(scale_low == 0.0f);
      BLI_assert(scale_high == 0.0f);
    }
    else if (scale_low == 0.0f) {
      BLI_assert(scale_high > 0.0f);
      /* Search mode, shrink layout until we can find a scale that fits. */
      scale = scale_high * 0.1f;
    }
    else if (scale_high == 0.0f) {
      BLI_assert(scale_low > 0.0f);
      /* Search mode, grow layout until we can find a scale that doesn't fit. */
      scale = scale_low * 10.0f;
    }
    else {
      /* Bracket mode, use modified secant method to find root. */
      BLI_assert(scale_low > 0.0f);
      BLI_assert(scale_high > 0.0f);
      BLI_assert(value_low <= 0.0f);
      BLI_assert(value_high >= 0.0f);
      if (scale_high < scale_low * 1.0001f) {
        /* Convergence. */
        break;
      }

      /* Secant method for area. */
      scale = (sqrtf(scale_low) * value_high - sqrtf(scale_high) * value_low) /
              (value_high - value_low);
      scale = scale * scale;

      if (iteration & 1) {
        /* Modified binary-search to improve robustness. */
        scale = sqrtf(scale * sqrtf(scale_low * scale_high));
      }
    }

    scale = std::max(scale, min_scale_roundoff);

    /* Evaluate our `f`. */
    scale_last = scale;
    float max_uv = pack_islands_scale_margin(
        island_vector, box_array, scale_last, margin_fraction, shape_method);
    float value = sqrtf(max_uv) - 1.0f;

    if (value <= 0.0f) {
      scale_low = scale;
      value_low = value;
    }
    else {
      scale_high = scale;
      value_high = value;
      if (scale == min_scale_roundoff) {
        /* Unable to pack without damaging UVs. */
        scale_low = scale;
        break;
      }
    }
  }

  const bool flush = true;
  if (flush) {
    /* Write back best pack as a side-effect. First get best pack. */
    if (scale_last != scale_low) {
      scale_last = scale_low;
      float max_uv = pack_islands_scale_margin(
          island_vector, box_array, scale_last, margin_fraction, shape_method);
      UNUSED_VARS(max_uv);
      /* TODO (?): `if (max_uv < 1.0f) { scale_last /= max_uv; }` */
    }

    /* Then expand FaceIslands by the correct amount. */
    for (const int64_t index : island_vector.index_range()) {
      BoxPack *box = &box_array[index];
      box->x /= scale_last;
      box->y /= scale_last;
      PackIsland *island = island_vector[index];
      BLI_rctf_pad(
          &island->bounds_rect, margin_fraction / scale_last, margin_fraction / scale_last);
    }
  }
  return scale_last;
}

static float calc_margin_from_aabb_length_sum(const Span<PackIsland *> &island_vector,
                                              const UVPackIsland_Params &params)
{
  /* Logic matches behavior from #geometry::uv_parametrizer_pack.
   * Attempt to give predictable results not dependent on current UV scale by using
   * `aabb_length_sum` (was "`area`") to multiply the margin by the length (was "area"). */
  double aabb_length_sum = 0.0f;
  for (PackIsland *island : island_vector) {
    float w = BLI_rctf_size_x(&island->bounds_rect);
    float h = BLI_rctf_size_y(&island->bounds_rect);
    aabb_length_sum += sqrtf(w * h);
  }
  return params.margin * aabb_length_sum * 0.1f;
}

static BoxPack *pack_islands_box_array(const Span<PackIsland *> &island_vector,
                                       const UVPackIsland_Params &params,
                                       float r_scale[2])
{
  BoxPack *box_array = static_cast<BoxPack *>(
      MEM_mallocN(sizeof(*box_array) * island_vector.size(), __func__));

  if (params.margin == 0.0f) {
    /* Special case for zero margin. Margin_method is ignored as all formulas give same result. */
    const float max_uv = pack_islands_scale_margin(
        island_vector, box_array, 1.0f, 0.0f, params.shape_method);
    r_scale[0] = 1.0f / max_uv;
    r_scale[1] = r_scale[0];
    return box_array;
  }

  if (params.margin_method == ED_UVPACK_MARGIN_FRACTION) {
    /* Uses a line search on scale. ~10x slower than other method. */
    const float scale = pack_islands_margin_fraction(
        island_vector, box_array, params.margin, params.shape_method);
    r_scale[0] = scale;
    r_scale[1] = scale;
    /* pack_islands_margin_fraction will pad FaceIslands, return early. */
    return box_array;
  }

  float margin = params.margin;
  switch (params.margin_method) {
    case ED_UVPACK_MARGIN_ADD:    /* Default for Blender 2.8 and earlier. */
      break;                      /* Nothing to do. */
    case ED_UVPACK_MARGIN_SCALED: /* Default for Blender 3.3 and later. */
      margin = calc_margin_from_aabb_length_sum(island_vector, params);
      break;
    case ED_UVPACK_MARGIN_FRACTION: /* Added as an option in Blender 3.4. */
      BLI_assert_unreachable();     /* Handled above. */
      break;
    default:
      BLI_assert_unreachable();
  }

  const float max_uv = pack_islands_scale_margin(
      island_vector, box_array, 1.0f, margin, params.shape_method);
  r_scale[0] = 1.0f / max_uv;
  r_scale[1] = r_scale[0];

  for (int index = 0; index < island_vector.size(); index++) {
    PackIsland *island = island_vector[index];
    BLI_rctf_pad(&island->bounds_rect, margin, margin);
  }
  return box_array;
}

void pack_islands(const Span<PackIsland *> &islands,
                  const UVPackIsland_Params &params,
                  float r_scale[2])
{
  BoxPack *box_array = pack_islands_box_array(islands, params, r_scale);

  for (int64_t i : islands.index_range()) {
    BoxPack *box = box_array + i;
    PackIsland *island = islands[box->index];
    island->pre_translate.x = box->x - island->bounds_rect.xmin;
    island->pre_translate.y = box->y - island->bounds_rect.ymin;
  }

  MEM_freeN(box_array);
}

}  // namespace blender::geometry
