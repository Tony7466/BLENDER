/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <optional>

#include "BLI_math_vector_types.hh"
#include "BLI_multi_value_map.hh"
#include "BLI_span.hh"
#include "BLI_vector.hh"
#include "BLI_offset_indices.hh"

namespace blender::geometry {

/**
 * Can find the polygon/triangle that maps to a specific uv coordinate.
 *
 * \note this uses a trivial implementation currently that has to be replaced.
 */
class ReverseUVSampler {
 public:
  struct LookupGrid;

 private:
  Span<float2> uv_map_;
  Span<int3> corner_tris_;
  int2 resolution_;
  int2 span_;
  int live_count_;
  
  Vector<int> corner_tris_by_cell_;
  Vector<int> cell_populations_;

  float2 uv_base_, uv_max_, supp_uv_min_, supp_uv_max_;

 public:
  ReverseUVSampler(Span<float2> uv_map,
                   Span<int3> corner_tris,
                   float2 supp_uv_min = float2{0.0f, 0.0f},
                   float2 supp_uv_max = float2{0.0f, 0.0f});

  enum class ResultType {
    None,
    Ok,
    Multiple,
  };

  struct Result {
    ResultType type = ResultType::None;
    int tri_index = -1;
    float3 bary_weights;
  };

  void sample(const float2 &query_uv, Result &result, bool hint = false) const;
  Result sample(const float2 &query_uv) const
  {
    Result result;
    sample(query_uv, result);
    return result;
  }
  void sample_many(Span<float2> query_uvs,
                   MutableSpan<Result> r_results,
                   bool hints = false) const;
};

}  // namespace blender::geometry
