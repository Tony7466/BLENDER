/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <functional>

#include "BLI_array_utils.hh"
#include "BLI_math_matrix_types.hh"
#include "BLI_math_quaternion_types.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_threads.h"

#include "atomic_ops.h"

namespace blender::array_utils {

void copy(const IndexMask &src_mask,
          const IndexMask &dst_mask,
          const GVArray &src,
          GMutableSpan dst,
          const int64_t grain_size)
{
  BLI_assert(src_mask.size() == dst_mask.size());
  BLI_assert(src_mask.min_array_size() <= src.size());
  BLI_assert(dst_mask.min_array_size() <= dst.size());
  threading::parallel_for(src_mask.index_range(), grain_size, [&](const IndexRange range) {
    const IndexMask src_local_mask = src_mask.slice(range);
    const IndexMask dst_local_mask = dst_mask.slice(range);

    const std::optional<IndexRange> src_range = src_local_mask.to_range();
    const std::optional<IndexRange> dst_range = dst_local_mask.to_range();
    if (src_range.has_value() && dst_range.has_value()) {
      src.materialize(IndexMask(*src_range), dst[dst_range->start()]);
      return;
    }
    if (dst_range.has_value()) {
      src.materialize_compressed(src_local_mask, dst[dst_range->start()]);
      return;
    }
    IndexMask::foreach_segment_zipped({src_local_mask, dst_local_mask},
                                      [&](const Span<IndexMaskSegment> segments) {
                                        const IndexMaskSegment src_segment = segments[0];
                                        const IndexMaskSegment dst_segment = segments[1];
                                        for (const int64_t i : src_segment.index_range()) {
                                          src.get(src_segment[i], dst[dst_segment[i]]);
                                        }
                                        return true;
                                      });
  });
}

void copy_groups(const IndexMask &src_mask,
                 const IndexMask &dst_mask,
                 const OffsetIndices<int> src_offsets,
                 const OffsetIndices<int> dst_offsets,
                 const GVArray &src,
                 GMutableSpan dst,
                 const int64_t grain_size)
{
  BLI_assert(src_mask.size() == dst_mask.size());
  [[maybe_unused]] const IndexRange mask_range = src_mask.index_range();
  BLI_assert(std::all_of(mask_range.begin(), mask_range.end(), [&](const int64_t index) {
    return src_offsets[src_mask[index]].size() == dst_offsets[dst_mask[index]].size();
  }));
  BLI_assert(std::all_of(mask_range.begin(), mask_range.end(), [&](const int64_t index) {
    return src_offsets[src_mask[index]].last() < src.size();
  }));
  BLI_assert(std::all_of(mask_range.begin(), mask_range.end(), [&](const int64_t index) {
    return dst_offsets[dst_mask[index]].last() < dst.size();
  }));
  threading::parallel_for(
      src_mask.index_range(),
      grain_size,
      [&](const IndexRange range) {
        const IndexMask src_local_mask = src_mask.slice(range);
        const IndexMask dst_local_mask = dst_mask.slice(range);

        const std::optional<IndexRange> src_range = src_local_mask.to_range();
        const std::optional<IndexRange> dst_range = dst_local_mask.to_range();
        if (src_range.has_value() && dst_range.has_value()) {
          const IndexRange src_values = src_offsets[*src_range];
          const IndexRange dst_values = dst_offsets[*dst_range];
          src.materialize(IndexMask(src_values), dst[dst_values.start()]);
          return;
        }

        IndexMask::foreach_segment_zipped(
            {src_local_mask, dst_local_mask}, [&](const Span<IndexMaskSegment> segments) {
              const IndexMaskSegment src_segment = segments[0];
              const IndexMaskSegment dst_segment = segments[1];
              for (const int64_t i : src_segment.index_range()) {
                const IndexRange src_values = src_offsets[src_segment[i]];
                const IndexRange dst_values = dst_offsets[dst_segment[i]];
                src.materialize(IndexMask(src_values), dst[dst_values.start()]);
              }
              return true;
            });
      },
      threading::accumulated_task_sizes([&](const IndexRange range) -> int64_t {
        const IndexMask src_local_mask = src_mask.slice(range);
        const std::optional<IndexRange> src_range = src_local_mask.to_range();
        if (src_range.has_value()) {
          return src_offsets[*src_range].size();
        }
        const IndexMask dst_local_mask = dst_mask.slice(range);
        const std::optional<IndexRange> dst_range = dst_local_mask.to_range();
        if (dst_range.has_value()) {
          return dst_offsets[*dst_range].size();
        }
        const int64_t src_max_bound = src_offsets[src_local_mask.bounds()].size();
        const int64_t dst_max_bound = dst_offsets[dst_local_mask.bounds()].size();
        return std::min(src_max_bound, dst_max_bound);
      }));
}

void copy(const GVArray &src, GMutableSpan dst, const int64_t grain_size)
{
  copy(IndexMask(src.size()), IndexMask(dst.size()), src, dst, grain_size);
}

void copy(GSpan src, GMutableSpan dst, const int64_t grain_size)
{
  copy(GVArray::ForSpan(src), dst, grain_size);
}

void copy(const GVArray &src, const IndexMask &mask, GMutableSpan dst, const int64_t grain_size)
{
  copy(mask, mask, src, dst, grain_size);
}

void copy(GSpan src, const IndexMask &mask, GMutableSpan dst, const int64_t grain_size)
{
  copy(GVArray::ForSpan(src), mask, dst, grain_size);
}

void gather(const GVArray &src, const IndexMask &mask, GMutableSpan dst, const int64_t grain_size)
{
  copy(mask, IndexMask(dst.size()), src, dst, grain_size);
}

void gather(GSpan src, const IndexMask &mask, GMutableSpan dst, const int64_t grain_size)
{
  gather(GVArray::ForSpan(src), mask, dst, grain_size);
}

void scatter(const GVArray &src, const IndexMask &mask, GMutableSpan dst, const int64_t grain_size)
{
  copy(IndexMask(src.size()), mask, src, dst, grain_size);
}

void scatter(GSpan src, const IndexMask &mask, GMutableSpan dst, const int64_t grain_size)
{
  scatter(GVArray::ForSpan(src), mask, dst, grain_size);
}

void copy_group_to_group(const OffsetIndices<int> src_offsets,
                         const OffsetIndices<int> dst_offsets,
                         const IndexMask &mask,
                         const GVArray &src,
                         GMutableSpan dst,
                         const int64_t grain_size)
{
  copy_groups(mask, mask, src_offsets, dst_offsets, src, dst, grain_size);
}

void copy_group_to_group(const OffsetIndices<int> src_offsets,
                         const OffsetIndices<int> dst_offsets,
                         const IndexMask &mask,
                         GSpan src,
                         GMutableSpan dst,
                         const int64_t grain_size)
{
  copy_group_to_group(src_offsets, dst_offsets, mask, GVArray::ForSpan(src), dst, grain_size);
}

void gather_group_to_group(const OffsetIndices<int> src_offsets,
                           const OffsetIndices<int> dst_offsets,
                           const IndexMask &mask,
                           const GVArray &src,
                           GMutableSpan dst,
                           const int64_t grain_size)
{
  copy_groups(mask, IndexMask(dst.size()), src_offsets, dst_offsets, src, dst, grain_size);
}

void gather_group_to_group(const OffsetIndices<int> src_offsets,
                           const OffsetIndices<int> dst_offsets,
                           const IndexMask &mask,
                           GSpan src,
                           GMutableSpan dst,
                           const int64_t grain_size)
{
  gather_group_to_group(src_offsets, dst_offsets, mask, GVArray::ForSpan(src), dst, grain_size);
}

void scatter_group_to_group(const OffsetIndices<int> src_offsets,
                            const OffsetIndices<int> dst_offsets,
                            const IndexMask &mask,
                            const GVArray &src,
                            GMutableSpan dst,
                            const int64_t grain_size)
{
  copy_groups(IndexMask(src.size()), mask, src_offsets, dst_offsets, src, dst, grain_size);
}

void scatter_group_to_group(const OffsetIndices<int> src_offsets,
                            const OffsetIndices<int> dst_offsets,
                            const IndexMask &mask,
                            GSpan src,
                            GMutableSpan dst,
                            const int64_t grain_size)
{
  scatter_group_to_group(src_offsets, dst_offsets, mask, GVArray::ForSpan(src), dst, grain_size);
}

template void copy<bool>(const IndexMask &src_mask,
                         const IndexMask &dst_mask,
                         const VArray<bool> &src,
                         MutableSpan<bool> dst,
                         int64_t grain_size);
template void copy<int>(const IndexMask &src_mask,
                        const IndexMask &dst_mask,
                        const VArray<int> &src,
                        MutableSpan<int> dst,
                        int64_t grain_size);
template void copy<int2>(const IndexMask &src_mask,
                         const IndexMask &dst_mask,
                         const VArray<int2> &src,
                         MutableSpan<int2> dst,
                         int64_t grain_size);
template void copy<int3>(const IndexMask &src_mask,
                         const IndexMask &dst_mask,
                         const VArray<int3> &src,
                         MutableSpan<int3> dst,
                         int64_t grain_size);
template void copy<int8_t>(const IndexMask &src_mask,
                           const IndexMask &dst_mask,
                           const VArray<int8_t> &src,
                           MutableSpan<int8_t> dst,
                           int64_t grain_size);
template void copy<int64_t>(const IndexMask &src_mask,
                            const IndexMask &dst_mask,
                            const VArray<int64_t> &src,
                            MutableSpan<int64_t> dst,
                            int64_t grain_size);
template void copy<float>(const IndexMask &src_mask,
                          const IndexMask &dst_mask,
                          const VArray<float> &src,
                          MutableSpan<float> dst,
                          int64_t grain_size);
template void copy<float2>(const IndexMask &src_mask,
                           const IndexMask &dst_mask,
                           const VArray<float2> &src,
                           MutableSpan<float2> dst,
                           int64_t grain_size);
template void copy<float3>(const IndexMask &src_mask,
                           const IndexMask &dst_mask,
                           const VArray<float3> &src,
                           MutableSpan<float3> dst,
                           int64_t grain_size);
template void copy<float4>(const IndexMask &src_mask,
                           const IndexMask &dst_mask,
                           const VArray<float4> &src,
                           MutableSpan<float4> dst,
                           int64_t grain_size);
template void copy<float2x2>(const IndexMask &src_mask,
                             const IndexMask &dst_mask,
                             const VArray<float2x2> &src,
                             MutableSpan<float2x2> dst,
                             int64_t grain_size);
template void copy<float3x3>(const IndexMask &src_mask,
                             const IndexMask &dst_mask,
                             const VArray<float3x3> &src,
                             MutableSpan<float3x3> dst,
                             int64_t grain_size);
template void copy<float4x4>(const IndexMask &src_mask,
                             const IndexMask &dst_mask,
                             const VArray<float4x4> &src,
                             MutableSpan<float4x4> dst,
                             int64_t grain_size);
template void copy<math::Quaternion>(const IndexMask &src_mask,
                                     const IndexMask &dst_mask,
                                     const VArray<math::Quaternion> &src,
                                     MutableSpan<math::Quaternion> dst,
                                     int64_t grain_size);
template void copy<std::string>(const IndexMask &src_mask,
                                const IndexMask &dst_mask,
                                const VArray<std::string> &src,
                                MutableSpan<std::string> dst,
                                int64_t grain_size);

template void copy_groups<bool>(const IndexMask &src_mask,
                                const IndexMask &dst_mask,
                                const OffsetIndices<int> src_offsets,
                                const OffsetIndices<int> dst_offsets,
                                const VArray<bool> &src,
                                MutableSpan<bool> dst,
                                int64_t grain_size);
template void copy_groups<int>(const IndexMask &src_mask,
                               const IndexMask &dst_mask,
                               const OffsetIndices<int> src_offsets,
                               const OffsetIndices<int> dst_offsets,
                               const VArray<int> &src,
                               MutableSpan<int> dst,
                               int64_t grain_size);
template void copy_groups<int2>(const IndexMask &src_mask,
                                const IndexMask &dst_mask,
                                const OffsetIndices<int> src_offsets,
                                const OffsetIndices<int> dst_offsets,
                                const VArray<int2> &src,
                                MutableSpan<int2> dst,
                                int64_t grain_size);
template void copy_groups<int3>(const IndexMask &src_mask,
                                const IndexMask &dst_mask,
                                const OffsetIndices<int> src_offsets,
                                const OffsetIndices<int> dst_offsets,
                                const VArray<int3> &src,
                                MutableSpan<int3> dst,
                                int64_t grain_size);
template void copy_groups<int8_t>(const IndexMask &src_mask,
                                  const IndexMask &dst_mask,
                                  const OffsetIndices<int> src_offsets,
                                  const OffsetIndices<int> dst_offsets,
                                  const VArray<int8_t> &src,
                                  MutableSpan<int8_t> dst,
                                  int64_t grain_size);
template void copy_groups<int64_t>(const IndexMask &src_mask,
                                   const IndexMask &dst_mask,
                                   const OffsetIndices<int> src_offsets,
                                   const OffsetIndices<int> dst_offsets,
                                   const VArray<int64_t> &src,
                                   MutableSpan<int64_t> dst,
                                   int64_t grain_size);
template void copy_groups<float>(const IndexMask &src_mask,
                                 const IndexMask &dst_mask,
                                 const OffsetIndices<int> src_offsets,
                                 const OffsetIndices<int> dst_offsets,
                                 const VArray<float> &src,
                                 MutableSpan<float> dst,
                                 int64_t grain_size);
template void copy_groups<float2>(const IndexMask &src_mask,
                                  const IndexMask &dst_mask,
                                  const OffsetIndices<int> src_offsets,
                                  const OffsetIndices<int> dst_offsets,
                                  const VArray<float2> &src,
                                  MutableSpan<float2> dst,
                                  int64_t grain_size);
template void copy_groups<float3>(const IndexMask &src_mask,
                                  const IndexMask &dst_mask,
                                  const OffsetIndices<int> src_offsets,
                                  const OffsetIndices<int> dst_offsets,
                                  const VArray<float3> &src,
                                  MutableSpan<float3> dst,
                                  int64_t grain_size);
template void copy_groups<float4>(const IndexMask &src_mask,
                                  const IndexMask &dst_mask,
                                  const OffsetIndices<int> src_offsets,
                                  const OffsetIndices<int> dst_offsets,
                                  const VArray<float4> &src,
                                  MutableSpan<float4> dst,
                                  int64_t grain_size);
template void copy_groups<float2x2>(const IndexMask &src_mask,
                                    const IndexMask &dst_mask,
                                    const OffsetIndices<int> src_offsets,
                                    const OffsetIndices<int> dst_offsets,
                                    const VArray<float2x2> &src,
                                    MutableSpan<float2x2> dst,
                                    int64_t grain_size);
template void copy_groups<float3x3>(const IndexMask &src_mask,
                                    const IndexMask &dst_mask,
                                    const OffsetIndices<int> src_offsets,
                                    const OffsetIndices<int> dst_offsets,
                                    const VArray<float3x3> &src,
                                    MutableSpan<float3x3> dst,
                                    int64_t grain_size);
template void copy_groups<float4x4>(const IndexMask &src_mask,
                                    const IndexMask &dst_mask,
                                    const OffsetIndices<int> src_offsets,
                                    const OffsetIndices<int> dst_offsets,
                                    const VArray<float4x4> &src,
                                    MutableSpan<float4x4> dst,
                                    int64_t grain_size);
template void copy_groups<math::Quaternion>(const IndexMask &src_mask,
                                            const IndexMask &dst_mask,
                                            const OffsetIndices<int> src_offsets,
                                            const OffsetIndices<int> dst_offsets,
                                            const VArray<math::Quaternion> &src,
                                            MutableSpan<math::Quaternion> dst,
                                            int64_t grain_size);
template void copy_groups<std::string>(const IndexMask &src_mask,
                                       const IndexMask &dst_mask,
                                       const OffsetIndices<int> src_offsets,
                                       const OffsetIndices<int> dst_offsets,
                                       const VArray<std::string> &src,
                                       MutableSpan<std::string> dst,
                                       int64_t grain_size);

void count_indices(const Span<int> indices, MutableSpan<int> counts)
{
  if (indices.size() < 8192 || BLI_system_thread_count() < 4) {
    for (const int i : indices) {
      counts[i]++;
    }
  }
  else {
    threading::parallel_for(indices.index_range(), 4096, [&](const IndexRange range) {
      for (const int i : indices.slice(range)) {
        atomic_add_and_fetch_int32(&counts[i], 1);
      }
    });
  }
}

void invert_booleans(MutableSpan<bool> span)
{
  threading::parallel_for(span.index_range(), 4096, [&](IndexRange range) {
    for (const int i : range) {
      span[i] = !span[i];
    }
  });
}

void invert_booleans(MutableSpan<bool> span, const IndexMask &mask)
{
  mask.foreach_index_optimized<int64_t>([&](const int64_t i) { span[i] = !span[i]; });
}

static bool all_equal(const Span<bool> span, const bool test)
{
  return std::all_of(span.begin(), span.end(), [&](const bool value) { return value == test; });
}

static bool all_equal(const VArray<bool> &varray, const IndexRange range, const bool test)
{
  return std::all_of(
      range.begin(), range.end(), [&](const int64_t i) { return varray[i] == test; });
}

BooleanMix booleans_mix_calc(const VArray<bool> &varray, const IndexRange range_to_check)
{
  if (varray.is_empty()) {
    return BooleanMix::None;
  }
  const CommonVArrayInfo info = varray.common_info();
  if (info.type == CommonVArrayInfo::Type::Single) {
    return *static_cast<const bool *>(info.data) ? BooleanMix::AllTrue : BooleanMix::AllFalse;
  }
  if (info.type == CommonVArrayInfo::Type::Span) {
    const Span<bool> span(static_cast<const bool *>(info.data), varray.size());
    return threading::parallel_reduce(
        range_to_check,
        4096,
        BooleanMix::None,
        [&](const IndexRange range, const BooleanMix init) {
          if (init == BooleanMix::Mixed) {
            return init;
          }
          const Span<bool> slice = span.slice(range);
          const bool compare = (init == BooleanMix::None) ? slice.first() :
                                                            (init == BooleanMix::AllTrue);
          if (all_equal(slice, compare)) {
            return compare ? BooleanMix::AllTrue : BooleanMix::AllFalse;
          }
          return BooleanMix::Mixed;
        },
        [&](BooleanMix a, BooleanMix b) { return (a == b) ? a : BooleanMix::Mixed; });
  }
  return threading::parallel_reduce(
      range_to_check,
      2048,
      BooleanMix::None,
      [&](const IndexRange range, const BooleanMix init) {
        if (init == BooleanMix::Mixed) {
          return init;
        }
        /* Alternatively, this could use #materialize to retrieve many values at once. */
        const bool compare = (init == BooleanMix::None) ? varray[range.first()] :
                                                          (init == BooleanMix::AllTrue);
        if (all_equal(varray, range, compare)) {
          return compare ? BooleanMix::AllTrue : BooleanMix::AllFalse;
        }
        return BooleanMix::Mixed;
      },
      [&](BooleanMix a, BooleanMix b) { return (a == b) ? a : BooleanMix::Mixed; });
}

int64_t count_booleans(const VArray<bool> &varray, const IndexMask &mask)
{
  if (varray.is_empty() || mask.is_empty()) {
    return 0;
  }
  /* Check if mask is full. */
  if (varray.size() == mask.size()) {
    const CommonVArrayInfo info = varray.common_info();
    if (info.type == CommonVArrayInfo::Type::Single) {
      return *static_cast<const bool *>(info.data) ? varray.size() : 0;
    }
    if (info.type == CommonVArrayInfo::Type::Span) {
      const Span<bool> span(static_cast<const bool *>(info.data), varray.size());
      return threading::parallel_reduce(
          varray.index_range(),
          4096,
          0,
          [&](const IndexRange range, const int64_t init) {
            const Span<bool> slice = span.slice(range);
            return init + std::count(slice.begin(), slice.end(), true);
          },
          std::plus<int64_t>());
    }
    return threading::parallel_reduce(
        varray.index_range(),
        2048,
        0,
        [&](const IndexRange range, const int64_t init) {
          int64_t value = init;
          /* Alternatively, this could use #materialize to retrieve many values at once. */
          for (const int64_t i : range) {
            value += int64_t(varray[i]);
          }
          return value;
        },
        std::plus<int64_t>());
  }
  const CommonVArrayInfo info = varray.common_info();
  if (info.type == CommonVArrayInfo::Type::Single) {
    return *static_cast<const bool *>(info.data) ? mask.size() : 0;
  }
  int64_t value = 0;
  mask.foreach_segment([&](const IndexMaskSegment segment) {
    for (const int64_t i : segment) {
      value += int64_t(varray[i]);
    }
  });
  return value;
}

int64_t count_booleans(const VArray<bool> &varray)
{
  return count_booleans(varray, IndexMask(varray.size()));
}

bool indices_are_range(Span<int> indices, IndexRange range)
{
  if (indices.size() != range.size()) {
    return false;
  }
  return threading::parallel_reduce(
      range.index_range(),
      4096,
      true,
      [&](const IndexRange part, const bool is_range) {
        const Span<int> local_indices = indices.slice(part);
        const IndexRange local_range = range.slice(part);
        return is_range &&
               std::equal(local_indices.begin(), local_indices.end(), local_range.begin());
      },
      std::logical_and<bool>());
}

}  // namespace blender::array_utils
