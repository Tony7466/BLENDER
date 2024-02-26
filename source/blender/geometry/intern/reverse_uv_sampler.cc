/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <algorithm>
#include <fmt/format.h>
#include <mutex>

#include "GEO_reverse_uv_sampler.hh"

#include "BLI_bounds.hh"
#include "BLI_enumerable_thread_specific.hh"
#include "BLI_index_mask.hh"
#include "BLI_math_geom.h"
#include "BLI_math_vector.hh"
#include "BLI_offset_indices.hh"
#include "BLI_task.hh"
#include "BLI_timeit.hh"

namespace blender::geometry {

struct Row {
  int x_min = 0;
  int x_max = 0;
  Vector<int> offsets;
  Vector<int> tri_indices;
};

struct ReverseUVSampler::LookupGrid {
  int y_min = 0;
  Array<Row> rows;
};

struct TriWithRange {
  int tri_index;
  IndexRange range;
};

struct TriWithRangeGroup {
  int filled_num = 0;
  std::array<TriWithRange, 8> tris;
  TriWithRangeGroup *next = nullptr;
};

struct LocalRowData {
  TriWithRangeGroup *tris = nullptr;
  int x_min = INT32_MAX;
  int x_max = INT32_MIN;
};

struct LocalData {
  LinearAllocator<> allocator;
  Map<int, destruct_ptr<LocalRowData>> rows;
};

static int2 uv_to_cell_key(const float2 &uv, const int resolution)
{
  return int2{uv * resolution};
}

static Bounds<int2> tri_to_key_bounds(const int3 &tri,
                                      const int resolution,
                                      const Span<float2> uv_map)
{
  const float2 &uv_0 = uv_map[tri[0]];
  const float2 &uv_1 = uv_map[tri[1]];
  const float2 &uv_2 = uv_map[tri[2]];

  const int2 key_0 = uv_to_cell_key(uv_0, resolution);
  const int2 key_1 = uv_to_cell_key(uv_1, resolution);
  const int2 key_2 = uv_to_cell_key(uv_2, resolution);

  const int2 min_key = math::min(math::min(key_0, key_1), key_2);
  const int2 max_key = math::max(math::max(key_0, key_1), key_2);

  return {min_key, max_key};
}

ReverseUVSampler::ReverseUVSampler(const Span<float2> uv_map, const Span<int3> corner_tris)
    : ReverseUVSampler(uv_map, corner_tris, IndexMask(corner_tris.size()))
{
}

BLI_NOINLINE static void sort_into_y_buckets(
    const Span<float2> uv_map,
    const Span<int3> corner_tris,
    const IndexMask &corner_tris_mask,
    const int resolution,
    threading::EnumerableThreadSpecific<LocalData> &data_per_thread)
{
  SCOPED_TIMER_AVERAGED("sort into y buckets");
  corner_tris_mask.foreach_segment(GrainSize(512), [&](const IndexMaskSegment tris) {
    LocalData &local_data = data_per_thread.local();
    for (const int tri_i : tris) {
      const int3 &tri = corner_tris[tri_i];
      const Bounds<int2> key_bounds = tri_to_key_bounds(tri, resolution, uv_map);
      const IndexRange x_range = IndexRange::from_begin_end_inclusive(key_bounds.min.x,
                                                                      key_bounds.max.x);
      const TriWithRange tri_with_range{tri_i, x_range};

      for (const int key_y :
           IndexRange::from_begin_end_inclusive(key_bounds.min.y, key_bounds.max.y))
      {
        LocalRowData &row = *local_data.rows.lookup_or_add_cb(
            key_y, [&]() { return local_data.allocator.construct<LocalRowData>(); });

        if (row.tris == nullptr || row.tris->filled_num == row.tris->tris.size()) {
          static_assert(std::is_trivially_destructible_v<TriWithRangeGroup>);
          TriWithRangeGroup *new_group =
              local_data.allocator.construct<TriWithRangeGroup>().release();
          new_group->next = row.tris;
          row.tris = new_group;
        }

        row.tris->tris[row.tris->filled_num++] = tri_with_range;
        row.x_min = std::min<int>(row.x_min, x_range.first());
        row.x_max = std::max<int>(row.x_max, x_range.last());
      }
    }
  });
}

BLI_NOINLINE static void fill_rows(const Span<int> all_ys,
                                   const Span<const LocalData *> local_data_vec,
                                   const Bounds<int> y_bounds,
                                   ReverseUVSampler::LookupGrid &lookup_grid)
{
  SCOPED_TIMER_AVERAGED("fill rows");
  threading::parallel_for(all_ys.index_range(), 8, [&](const IndexRange all_ys_range) {
    for (const int y : all_ys.slice(all_ys_range)) {
      Row &row = lookup_grid.rows[y - y_bounds.min];

      Vector<const LocalRowData *, 32> local_rows;
      for (const LocalData *local_data : local_data_vec) {
        if (const destruct_ptr<LocalRowData> *local_row = local_data->rows.lookup_ptr(y)) {
          local_rows.append(local_row->get());
        }
      }

      int x_min = INT32_MAX;
      int x_max = INT32_MIN;
      for (const LocalRowData *local_row : local_rows) {
        x_min = std::min(x_min, local_row->x_min);
        x_max = std::max(x_max, local_row->x_max);
      }

      const int x_num = x_max - x_min + 1;
      row.offsets.resize(x_num + 1, 0);
      {
        MutableSpan<int> counts = row.offsets;
        for (const LocalRowData *local_row : local_rows) {
          for (const TriWithRangeGroup *group = local_row->tris; group; group = group->next) {
            for (const int i : IndexRange(group->filled_num)) {
              const TriWithRange &tri_with_range = group->tris[i];
              for (const int x : tri_with_range.range) {
                counts[x - x_min]++;
              }
            }
          }
        }
        offset_indices::accumulate_counts_to_offsets(counts);
      }
      const int tri_indices_num = row.offsets.last();
      row.tri_indices.resize(tri_indices_num);

      Array<int, 1000> current_offsets(x_num, 0);
      for (const LocalRowData *local_row : local_rows) {
        for (const TriWithRangeGroup *group = local_row->tris; group; group = group->next) {
          for (const int i : IndexRange(group->filled_num)) {
            const TriWithRange &tri_with_range = group->tris[i];
            for (const int x : tri_with_range.range) {
              const int offset_x = x - x_min;
              row.tri_indices[row.offsets[offset_x] + current_offsets[offset_x]] =
                  tri_with_range.tri_index;
              current_offsets[offset_x]++;
            }
          }
        }
      }

      row.x_min = x_min;
      row.x_max = x_max;
    }
  });
}

ReverseUVSampler::ReverseUVSampler(const Span<float2> uv_map,
                                   const Span<int3> corner_tris,
                                   const IndexMask &corner_tris_mask)
    : uv_map_(uv_map), corner_tris_(corner_tris), lookup_grid_(std::make_unique<LookupGrid>())
{
  resolution_ = std::max<int>(3, std::sqrt(corner_tris.size()) * 3);
  if (corner_tris.is_empty()) {
    return;
  }

  threading::EnumerableThreadSpecific<LocalData> data_per_thread;
  sort_into_y_buckets(uv_map_, corner_tris_, corner_tris_mask, resolution_, data_per_thread);

  VectorSet<int> all_ys;
  Vector<const LocalData *> local_data_vec;
  for (const LocalData &local_data : data_per_thread) {
    local_data_vec.append(&local_data);
    for (const int y : local_data.rows.keys()) {
      all_ys.add(y);
    }
  }

  const Bounds<int> y_bounds = *bounds::min_max(all_ys.as_span());

  const int rows_num = y_bounds.max - y_bounds.min + 1;
  lookup_grid_->rows.reinitialize(rows_num);

  fill_rows(all_ys, local_data_vec, y_bounds, *lookup_grid_);

  // fmt::println("Rows");
  // for (const int row_i : lookup_grid_->rows.index_range()) {
  //   const Row &row = lookup_grid_->rows[row_i];
  //   fmt::println("  Row {}:", row_i);
  //   const OffsetIndices<int> offsets{row.offsets};
  //   for (const int x_i : offsets.index_range()) {
  //     fmt::println("    {}: {}",
  //                  x_i + row.x_min,
  //                  fmt::join(row.tri_indices.as_span().slice(offsets[x_i]), ", "));
  //   }
  // }
}

static Span<int> lookup_tris_in_cell(const int2 cell,
                                     const ReverseUVSampler::LookupGrid &lookup_grid)
{
  if (cell.y < lookup_grid.y_min) {
    return {};
  }
  if (cell.y >= lookup_grid.y_min + lookup_grid.rows.size()) {
    return {};
  }
  const Row &row = lookup_grid.rows[cell.y - lookup_grid.y_min];
  if (cell.x < row.x_min) {
    return {};
  }
  if (cell.x > row.x_max) {
    return {};
  }
  const int offset = row.offsets[cell.x - row.x_min];
  const int tris_num = row.offsets[cell.x - row.x_min + 1] - offset;
  return row.tri_indices.as_span().slice(offset, tris_num);
}

ReverseUVSampler::Result ReverseUVSampler::sample(const float2 &query_uv) const
{
  const int2 cell_key = uv_to_cell_key(query_uv, resolution_);
  const Span<int> tri_indices = lookup_tris_in_cell(cell_key, *lookup_grid_);

  float best_dist = FLT_MAX;
  float3 best_bary_weights;
  int best_tri_index;

  /* The distance to an edge that is allowed to be inside or outside the triangle. Without this,
   * the lookup can fail for floating point accuracy reasons when the uv is almost exact on an
   * edge. */
  const float edge_epsilon = 0.00001f;

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
        /* The uv sample is in multiple triangles. */
        return Result{ResultType::Multiple};
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
    return Result{ResultType::Ok, best_tri_index, math::clamp(best_bary_weights, 0.0f, 1.0f)};
  }

  return Result{};
}

ReverseUVSampler::~ReverseUVSampler() = default;

void ReverseUVSampler::sample_many(const Span<float2> query_uvs,
                                   MutableSpan<Result> r_results) const
{
  BLI_assert(query_uvs.size() == r_results.size());
  threading::parallel_for(query_uvs.index_range(), 256, [&](const IndexRange range) {
    for (const int i : range) {
      r_results[i] = this->sample(query_uvs[i]);
    }
  });
}

}  // namespace blender::geometry
