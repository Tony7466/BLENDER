/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <map>
#include <mutex>
#include <random>

#include "GEO_reverse_uv_sampler.hh"

#include "BLI_bounds.hh"
#include "BLI_enumerable_thread_specific.hh"
#include "BLI_index_mask.hh"
#include "BLI_linear_allocator_chunked_list.hh"
#include "BLI_math_geom.h"
#include "BLI_math_vector.hh"
#include "BLI_offset_indices.hh"
#include "BLI_task.hh"
#include "BLI_timeit.hh"

namespace blender::geometry {

static inline int2 uv_to_cell_key(const float2 &uv,
                           const float2 &uv_base,
                           const int2 &resolution,
                           int2 span)
{
  auto key = int2{int((uv.x - uv_base.x) * resolution.x), int((uv.y - uv_base.y) * resolution.y)};
  return int2{std::max(0, std::min(span.x - 1, key.x)), std::max(0, std::min(span.y - 1, key.y))};
}

ReverseUVSampler::ReverseUVSampler(const Span<float2> uv_map,
                                   const Span<int3> corner_tris,
                                   float2 supp_uv_min,
                                   float2 supp_uv_max)
    : uv_map_(uv_map),
      corner_tris_(corner_tris),
      resolution_(int2{0, 0}),
      supp_uv_min_(supp_uv_min),
      supp_uv_max_(supp_uv_max)
{
  if (corner_tris_.size() == 0)
    return;

  std::string format = "ReverseUVSampler::ReverseUVSampler(" +
                       std::to_string(corner_tris_.size()) + ")";
  SCOPED_TIMER_AVERAGED(format);
  const int3 &tri = corner_tris_[0];
  const float2 &uv_0 = uv_map_[tri[0]];
  float2 min_uv, max_uv;
  min_uv = max_uv = uv_0;
  int live_count = 0;

  Vector<int8_t> mask(corner_tris_.size());

  bool support_set = (supp_uv_max_.x > supp_uv_min_.x && supp_uv_max_.y > supp_uv_min_.y);

  /* We walk through the list of triangles three times.
   * 1st pass: calculate the bounding box and determine which triangles are
   * to be used for sampling. Set the cell resolution.
   * 2nd pass: calculate how many triangles hit each cell. Allocate the index table.
   * 3rd pass: populate the index table. */
    for (const int tri_i : corner_tris_.index_range()) {
      const int3 &tri = corner_tris_[tri_i];
      const float2 &uv_0 = uv_map_[tri[0]];
      const float2 &uv_1 = uv_map_[tri[1]];
      const float2 &uv_2 = uv_map_[tri[2]];
      float2 tri_min_uv = math::min(math::min(uv_0, uv_1), uv_2);
      float2 tri_max_uv = math::max(math::max(uv_0, uv_1), uv_2);
      if ((!support_set) || (tri_max_uv.x >= supp_uv_min_.x && tri_min_uv.x <= supp_uv_max_.x &&
                             tri_max_uv.y >= supp_uv_min_.y && tri_min_uv.y <= supp_uv_max_.y)) {
        live_count++;
        mask[tri_i] = 1;
        min_uv = math::min(min_uv, tri_min_uv);
        max_uv = math::max(max_uv, tri_max_uv);
      } else {
        mask[tri_i] = 0;
      }
  }

  /* Balance between construction time and evaluation time. Smaller values 
   * lead to faster construction (fewer cells) but slower lookups. At 1.0,
   * we expect ~2-4 triangles per cell. */
  float magic_number = 0.5f;
  float mult = magic_number * sqrt(float(live_count));
  live_count_ = live_count;

  resolution_.x = math::max<int>(1, mult / (max_uv[0] - min_uv[0] + 1e-6));
  resolution_.y = math::max<int>(1, mult / (max_uv[1] - min_uv[1] + 1e-6));

  uv_base_ = min_uv;
  uv_max_ = max_uv;
  span_ = int2{int((max_uv.x - min_uv.x) * resolution_.x),
               int((max_uv.y - min_uv.y) * resolution_.y)};
  span_.x += 1;
  span_.y += 1;

  cell_populations_.resize(span_.x * span_.y + 1, 0);

  for (const int tri_i : corner_tris_.index_range()) {
    if (!mask[tri_i])
      continue;
    const int3 &tri = corner_tris_[tri_i];
    const float2 &uv_0 = uv_map_[tri[0]];
    const float2 &uv_1 = uv_map_[tri[1]];
    const float2 &uv_2 = uv_map_[tri[2]];
    float2 tri_min_uv = math::min(math::min(uv_0, uv_1), uv_2);
    float2 tri_max_uv = math::max(math::max(uv_0, uv_1), uv_2);
    const int2 min_key = uv_to_cell_key(tri_min_uv, uv_base_, resolution_, span_);
    const int2 max_key = uv_to_cell_key(tri_max_uv, uv_base_, resolution_, span_);
    for (int key_y = min_key.y; key_y <= max_key.y; key_y++) {
      for (int key_x = min_key.x; key_x <= max_key.x; key_x++) {
        cell_populations_[key_x + key_y * span_.x + 1]++;
      }
    }
  }

  int total_entries = 0;
  for (int i = 0; i < span_.x * span_.y; i++) {
    total_entries += cell_populations_[i + 1];
    cell_populations_[i + 1] = total_entries;
  }
  corner_tris_by_cell_.resize(total_entries);

  for (const int tri_i : corner_tris_.index_range()) {
    if (!mask[tri_i])
      continue;

    const int3 &tri = corner_tris_[tri_i];
    const float2 &uv_0 = uv_map_[tri[0]];
    const float2 &uv_1 = uv_map_[tri[1]];
    const float2 &uv_2 = uv_map_[tri[2]];
    float2 tri_min_uv = math::min(math::min(uv_0, uv_1), uv_2);
    float2 tri_max_uv = math::max(math::max(uv_0, uv_1), uv_2);
    const int2 min_key = uv_to_cell_key(tri_min_uv, uv_base_, resolution_, span_);
    const int2 max_key = uv_to_cell_key(tri_max_uv, uv_base_, resolution_, span_);
    for (int key_y = min_key.y; key_y <= max_key.y; key_y++) {
      for (int key_x = min_key.x; key_x <= max_key.x; key_x++) {
        int &addr = cell_populations_[key_x + key_y * span_.x];
        corner_tris_by_cell_[addr] = tri_i;
        addr++;
      }
    }
  }
}

void ReverseUVSampler::sample(
    const float2 &query_uv, ReverseUVSampler::Result &result, bool hint) const
{
    if (hint && result.type == ResultType::Ok) {
      const int tri_i = result.tri_index;
      if (tri_i >= 0 && tri_i < corner_tris_.size()) {
        const int3 &tri = corner_tris_[tri_i];
        const float2 &uv_0 = uv_map_[tri[0]];
        const float2 &uv_1 = uv_map_[tri[1]];
        const float2 &uv_2 = uv_map_[tri[2]];
        float3 bary_weights;
        if (barycentric_coords_v2(uv_0, uv_1, uv_2, query_uv, bary_weights)) {
          /* If #query_uv is in the triangle, the distance is <= 0. Otherwise, the larger the
           * distance, the further away the uv is from the triangle. */
          const float x_dist = std::max(-bary_weights.x, bary_weights.x - 1.0f);
          const float y_dist = std::max(-bary_weights.y, bary_weights.y - 1.0f);
          const float z_dist = std::max(-bary_weights.z, bary_weights.z - 1.0f);
          const float dist = std::max(std::max(x_dist, y_dist), z_dist);

          if (dist <= 0.0f) {
            result = Result{ResultType::Ok, tri_i, math::clamp(bary_weights, 0.0f, 1.0f)};
            return;
          }
        }
      }
    }

    const int2 cell_key = uv_to_cell_key(query_uv, uv_base_, resolution_, span_);

    int cell = cell_key.x + cell_key.y * span_.x;
    int start_pos = (cell == 0) ? 0 : cell_populations_[cell - 1];
    int end_pos = cell_populations_[cell];
    auto tri_indices = Span<int>(&corner_tris_by_cell_[start_pos], end_pos - start_pos);

    float best_dist = FLT_MAX;
    float3 best_bary_weights;
    int best_tri_index;

    /* The distance to an edge that is allowed to be inside or outside the triangle. Without this,
     * the lookup can fail for floating point accuracy reasons when the uv is almost exact on an
     * edge. */
    const float edge_epsilon = 0.0001f;
    /* If uv triangles are very small, it may look like the query hits multiple triangles due to
     * floating point precision issues. Better just pick one of the triangles instead of failing the
     * entire operation in this case. */
    const float area_epsilon = 0.0001f;

    for (const int tri_i : tri_indices) {
      const int3 &tri = corner_tris_[tri_i];
      const float2 &uv_0 = uv_map_[tri[0]];
      const float2 &uv_1 = uv_map_[tri[1]];
      const float2 &uv_2 = uv_map_[tri[2]];
      float3 bary_weights;
      if (!barycentric_coords_v2(uv_0, uv_1, uv_2, query_uv, bary_weights)) {
        continue;
      }

      /* If #query_uv is in the triangle, the distance is <= 0. Otherwise, the larger the distance,
       * the further away the uv is from the triangle. */
      const float x_dist = std::max(-bary_weights.x, bary_weights.x - 1.0f);
      const float y_dist = std::max(-bary_weights.y, bary_weights.y - 1.0f);
      const float z_dist = std::max(-bary_weights.z, bary_weights.z - 1.0f);
      const float dist = std::max({x_dist, y_dist, z_dist});

      if (dist <= 0.0f && best_dist <= 0.0f) {
        const float worse_dist = std::max(dist, best_dist);
        /* Allow ignoring multiple triangle intersections if the uv is almost exactly on an edge. */
        if (worse_dist < -edge_epsilon) {
          const int3 &best_tri = corner_tris_[tri_i];
          const float best_tri_area = area_tri_v2(
              uv_map_[best_tri[0]], uv_map_[best_tri[1]], uv_map_[best_tri[2]]);
          const float current_tri_area = area_tri_v2(uv_0, uv_1, uv_2);
          if (best_tri_area > area_epsilon && current_tri_area > area_epsilon) {
            /* The uv sample is in multiple triangles. */
            result = Result{ResultType::Multiple};
            return;
          }
        }
      }

      if (dist < best_dist) {
        best_dist = dist;
        best_bary_weights = bary_weights;
        best_tri_index = tri_i;
      }
    }

    /* Allow using the closest (but not intersecting) triangle if the uv is almost exactly on an
     * edge. */
    if (best_dist < edge_epsilon) {
      result = Result{ResultType::Ok, best_tri_index, math::clamp(best_bary_weights, 0.0f, 1.0f)};
      return;
    }

    result = Result{ResultType::None, best_tri_index, best_bary_weights};
  }

  void ReverseUVSampler::sample_many(
      const Span<float2> query_uvs, MutableSpan<Result> r_results, bool hints) const
  {
    BLI_assert(query_uvs.size() == r_results.size());
    threading::parallel_for(query_uvs.index_range(), 256, [&](const IndexRange range) {
      for (const int i : range) {
        this->sample(query_uvs[i], r_results[i], hints);
      }
    });
  }

}  // namespace blender::geometry
