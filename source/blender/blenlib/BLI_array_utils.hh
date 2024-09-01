/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <numeric>

#include "BLI_generic_span.hh"
#include "BLI_generic_virtual_array.hh"
#include "BLI_index_mask.hh"
#include "BLI_offset_indices.hh"
#include "BLI_task.hh"
#include "BLI_task_size_hints.hh"
#include "BLI_virtual_array.hh"

namespace blender {
template<typename T, int Size> struct VecBase;
using int2 = VecBase<int32_t, 2>;
using int3 = VecBase<int32_t, 3>;
using int4 = VecBase<int32_t, 4>;
using float2 = VecBase<float, 2>;
using float3 = VecBase<float, 3>;
using float4 = VecBase<float, 4>;

template<typename T, int NumCol, int NumRow, int Alignment> struct alignas(Alignment) MatBase;
using float2x2 = MatBase<float, 2, 2, (((2 * 2) % 4 == 0) ? 4 : 1) * sizeof(float)>;
using float3x3 = MatBase<float, 3, 3, (((3 * 3) % 4 == 0) ? 4 : 1) * sizeof(float)>;
using float4x4 = MatBase<float, 4, 4, (((4 * 4) % 4 == 0) ? 4 : 1) * sizeof(float)>;

namespace math {
template<typename T> struct QuaternionBase;
using Quaternion = QuaternionBase<float>;
}  // namespace math
}  // namespace blender

namespace blender::array_utils {

template<typename T>
inline void copy(const IndexMask &src_mask,
                 const IndexMask &dst_mask,
                 const VArray<T> &src,
                 MutableSpan<T> dst,
                 const int64_t grain_size = 4096)
{
  BLI_assert(src_mask.size() == dst_mask.size());
  threading::parallel_for(src_mask.index_range(), grain_size, [&](const IndexRange range) {
    const IndexMask src_local_mask = src_mask.slice(range);
    const IndexMask dst_local_mask = dst_mask.slice(range);

    const std::optional<IndexRange> src_range = src_local_mask.to_range();
    const std::optional<IndexRange> dst_range = dst_local_mask.to_range();
    if (src_range.has_value() && dst_range.has_value()) {
      src.materialize(IndexMask(*src_range), dst.slice(*dst_range));
      return;
    }
    if (!src_range.has_value() && !dst_range.has_value()) {
      devirtualize_varray(src, [&](const auto &src) {
        IndexMask::foreach_segment_zipped({src_local_mask, dst_local_mask},
                                          [&](const Span<IndexMaskSegment> segments) {
                                            const IndexMaskSegment src_segment = segments[0];
                                            const IndexMaskSegment dst_segment = segments[1];
                                            for (const int64_t i : src_segment.index_range()) {
                                              dst[dst_segment[i]] = src[src_segment[i]];
                                            }
                                            return true;
                                          });
      });
      return;
    }
    if (src_range.has_value()) {
      devirtualize_varray(src, [&](const auto &src) {
        dst_local_mask.foreach_index_optimized<int64_t>(
            [&](const int64_t index, const int64_t pos) {
              dst[index] = src[pos + src_range->start()];
            });
      });
      return;
    }
    if (dst_range.has_value()) {
      src.materialize_compressed(src_local_mask, dst.slice(*dst_range));
    }
  });
}

void copy(const IndexMask &src_mask,
          const IndexMask &dst_mask,
          GSpan src,
          GMutableSpan dst,
          const int64_t grain_size = 4096);

template<typename T>
inline void copy_groups(const IndexMask &src_mask,
                        const IndexMask &dst_mask,
                        const OffsetIndices<int> src_offsets,
                        const OffsetIndices<int> dst_offsets,
                        const VArray<T> &src,
                        MutableSpan<T> dst,
                        const int64_t grain_size = 4096)
{
  BLI_assert(src_mask.size() == dst_mask.size());
  [[maybe_unused]] const IndexRange mask_range = src_mask.index_range();
  BLI_assert(std::all_of(mask_range.begin(), mask_range.end(), [&](const int64_t index) {
    return src_offsets[src_mask[index]].size() == dst_offsets[dst_mask[index]].size();
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
          src.materialize(IndexMask(src_values), dst.slice(dst_values));
          return;
        }

        IndexMask::foreach_segment_zipped(
            {src_local_mask, dst_local_mask}, [&](const Span<IndexMaskSegment> segments) {
              const IndexMaskSegment src_segment = segments[0];
              const IndexMaskSegment dst_segment = segments[1];
              for (const int64_t i : src_segment.index_range()) {
                const IndexRange src_values = src_offsets[src_segment[i]];
                const IndexRange dst_values = dst_offsets[dst_segment[i]];
                src.materialize(IndexMask(src_values), dst.slice(dst_values));
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

void copy_groups(const IndexMask &src_mask,
                 const IndexMask &dst_mask,
                 const OffsetIndices<int> src_offsets,
                 const OffsetIndices<int> dst_offsets,
                 const GVArray &src,
                 GMutableSpan dst,
                 const int64_t grain_size = 4096);

extern template void copy<bool>(const IndexMask &src_mask,
                                const IndexMask &dst_mask,
                                const VArray<bool> &src,
                                MutableSpan<bool> dst,
                                int64_t grain_size);
extern template void copy<int>(const IndexMask &src_mask,
                               const IndexMask &dst_mask,
                               const VArray<int> &src,
                               MutableSpan<int> dst,
                               int64_t grain_size);
extern template void copy<int2>(const IndexMask &src_mask,
                                const IndexMask &dst_mask,
                                const VArray<int2> &src,
                                MutableSpan<int2> dst,
                                int64_t grain_size);
extern template void copy<int3>(const IndexMask &src_mask,
                                const IndexMask &dst_mask,
                                const VArray<int3> &src,
                                MutableSpan<int3> dst,
                                int64_t grain_size);
extern template void copy<int8_t>(const IndexMask &src_mask,
                                  const IndexMask &dst_mask,
                                  const VArray<int8_t> &src,
                                  MutableSpan<int8_t> dst,
                                  int64_t grain_size);
extern template void copy<int64_t>(const IndexMask &src_mask,
                                   const IndexMask &dst_mask,
                                   const VArray<int64_t> &src,
                                   MutableSpan<int64_t> dst,
                                   int64_t grain_size);
extern template void copy<float>(const IndexMask &src_mask,
                                 const IndexMask &dst_mask,
                                 const VArray<float> &src,
                                 MutableSpan<float> dst,
                                 int64_t grain_size);
extern template void copy<float2>(const IndexMask &src_mask,
                                  const IndexMask &dst_mask,
                                  const VArray<float2> &src,
                                  MutableSpan<float2> dst,
                                  int64_t grain_size);
extern template void copy<float3>(const IndexMask &src_mask,
                                  const IndexMask &dst_mask,
                                  const VArray<float3> &src,
                                  MutableSpan<float3> dst,
                                  int64_t grain_size);
extern template void copy<float4>(const IndexMask &src_mask,
                                  const IndexMask &dst_mask,
                                  const VArray<float4> &src,
                                  MutableSpan<float4> dst,
                                  int64_t grain_size);
extern template void copy<float2x2>(const IndexMask &src_mask,
                                    const IndexMask &dst_mask,
                                    const VArray<float2x2> &src,
                                    MutableSpan<float2x2> dst,
                                    int64_t grain_size);
extern template void copy<float3x3>(const IndexMask &src_mask,
                                    const IndexMask &dst_mask,
                                    const VArray<float3x3> &src,
                                    MutableSpan<float3x3> dst,
                                    int64_t grain_size);
extern template void copy<float4x4>(const IndexMask &src_mask,
                                    const IndexMask &dst_mask,
                                    const VArray<float4x4> &src,
                                    MutableSpan<float4x4> dst,
                                    int64_t grain_size);
extern template void copy<math::Quaternion>(const IndexMask &src_mask,
                                            const IndexMask &dst_mask,
                                            const VArray<math::Quaternion> &src,
                                            MutableSpan<math::Quaternion> dst,
                                            int64_t grain_size);
extern template void copy<std::string>(const IndexMask &src_mask,
                                       const IndexMask &dst_mask,
                                       const VArray<std::string> &src,
                                       MutableSpan<std::string> dst,
                                       int64_t grain_size);

extern template void copy_groups<bool>(const IndexMask &src_mask,
                                       const IndexMask &dst_mask,
                                       const OffsetIndices<int> src_offsets,
                                       const OffsetIndices<int> dst_offsets,
                                       const VArray<bool> &src,
                                       MutableSpan<bool> dst,
                                       int64_t grain_size);
extern template void copy_groups<int>(const IndexMask &src_mask,
                                      const IndexMask &dst_mask,
                                      const OffsetIndices<int> src_offsets,
                                      const OffsetIndices<int> dst_offsets,
                                      const VArray<int> &src,
                                      MutableSpan<int> dst,
                                      int64_t grain_size);
extern template void copy_groups<int2>(const IndexMask &src_mask,
                                       const IndexMask &dst_mask,
                                       const OffsetIndices<int> src_offsets,
                                       const OffsetIndices<int> dst_offsets,
                                       const VArray<int2> &src,
                                       MutableSpan<int2> dst,
                                       int64_t grain_size);
extern template void copy_groups<int3>(const IndexMask &src_mask,
                                       const IndexMask &dst_mask,
                                       const OffsetIndices<int> src_offsets,
                                       const OffsetIndices<int> dst_offsets,
                                       const VArray<int3> &src,
                                       MutableSpan<int3> dst,
                                       int64_t grain_size);
extern template void copy_groups<int8_t>(const IndexMask &src_mask,
                                         const IndexMask &dst_mask,
                                         const OffsetIndices<int> src_offsets,
                                         const OffsetIndices<int> dst_offsets,
                                         const VArray<int8_t> &src,
                                         MutableSpan<int8_t> dst,
                                         int64_t grain_size);
extern template void copy_groups<int64_t>(const IndexMask &src_mask,
                                          const IndexMask &dst_mask,
                                          const OffsetIndices<int> src_offsets,
                                          const OffsetIndices<int> dst_offsets,
                                          const VArray<int64_t> &src,
                                          MutableSpan<int64_t> dst,
                                          int64_t grain_size);
extern template void copy_groups<float>(const IndexMask &src_mask,
                                        const IndexMask &dst_mask,
                                        const OffsetIndices<int> src_offsets,
                                        const OffsetIndices<int> dst_offsets,
                                        const VArray<float> &src,
                                        MutableSpan<float> dst,
                                        int64_t grain_size);
extern template void copy_groups<float2>(const IndexMask &src_mask,
                                         const IndexMask &dst_mask,
                                         const OffsetIndices<int> src_offsets,
                                         const OffsetIndices<int> dst_offsets,
                                         const VArray<float2> &src,
                                         MutableSpan<float2> dst,
                                         int64_t grain_size);
extern template void copy_groups<float3>(const IndexMask &src_mask,
                                         const IndexMask &dst_mask,
                                         const OffsetIndices<int> src_offsets,
                                         const OffsetIndices<int> dst_offsets,
                                         const VArray<float3> &src,
                                         MutableSpan<float3> dst,
                                         int64_t grain_size);
extern template void copy_groups<float4>(const IndexMask &src_mask,
                                         const IndexMask &dst_mask,
                                         const OffsetIndices<int> src_offsets,
                                         const OffsetIndices<int> dst_offsets,
                                         const VArray<float4> &src,
                                         MutableSpan<float4> dst,
                                         int64_t grain_size);
extern template void copy_groups<float2x2>(const IndexMask &src_mask,
                                           const IndexMask &dst_mask,
                                           const OffsetIndices<int> src_offsets,
                                           const OffsetIndices<int> dst_offsets,
                                           const VArray<float2x2> &src,
                                           MutableSpan<float2x2> dst,
                                           int64_t grain_size);
extern template void copy_groups<float3x3>(const IndexMask &src_mask,
                                           const IndexMask &dst_mask,
                                           const OffsetIndices<int> src_offsets,
                                           const OffsetIndices<int> dst_offsets,
                                           const VArray<float3x3> &src,
                                           MutableSpan<float3x3> dst,
                                           int64_t grain_size);
extern template void copy_groups<float4x4>(const IndexMask &src_mask,
                                           const IndexMask &dst_mask,
                                           const OffsetIndices<int> src_offsets,
                                           const OffsetIndices<int> dst_offsets,
                                           const VArray<float4x4> &src,
                                           MutableSpan<float4x4> dst,
                                           int64_t grain_size);
extern template void copy_groups<math::Quaternion>(const IndexMask &src_mask,
                                                   const IndexMask &dst_mask,
                                                   const OffsetIndices<int> src_offsets,
                                                   const OffsetIndices<int> dst_offsets,
                                                   const VArray<math::Quaternion> &src,
                                                   MutableSpan<math::Quaternion> dst,
                                                   int64_t grain_size);
extern template void copy_groups<std::string>(const IndexMask &src_mask,
                                              const IndexMask &dst_mask,
                                              const OffsetIndices<int> src_offsets,
                                              const OffsetIndices<int> dst_offsets,
                                              const VArray<std::string> &src,
                                              MutableSpan<std::string> dst,
                                              int64_t grain_size);

template<typename T>
inline void copy(const VArray<T> &src, MutableSpan<T> dst, const int64_t grain_size = 4096)
{
  copy<T>(IndexMask(src.size()), IndexMask(dst.size()), src, dst, grain_size);
}

template<typename T>
inline void copy(const Span<T> src, MutableSpan<T> dst, const int64_t grain_size = 4096)
{
  copy<T>(VArray<T>::ForSpan(src), dst, grain_size);
}

void copy(const GVArray &src, GMutableSpan dst, const int64_t grain_size = 4096);

void copy(GSpan src, GMutableSpan dst, const int64_t grain_size = 4096);

template<typename T>
inline void copy(const VArray<T> &src,
                 const IndexMask &mask,
                 MutableSpan<T> dst,
                 const int64_t grain_size = 4096)
{
  copy<T>(mask, mask, src, dst, grain_size);
}

template<typename T>
inline void copy(const Span<T> src,
                 const IndexMask &mask,
                 MutableSpan<T> dst,
                 const int64_t grain_size = 4096)
{
  copy<T>(VArray<T>::ForSpan(src), mask, dst, grain_size);
}

void copy(const GVArray &src,
          const IndexMask &mask,
          GMutableSpan dst,
          const int64_t grain_size = 4096);

void copy(GSpan src, const IndexMask &mask, GMutableSpan dst, const int64_t grain_size = 4096);

template<typename T>
inline void gather(const VArray<T> &src,
                   const IndexMask &mask,
                   MutableSpan<T> dst,
                   const int64_t grain_size = 4096)
{
  copy<T>(mask, IndexMask(dst.size()), src, dst, grain_size);
}

template<typename T>
inline void gather(const Span<T> src,
                   const IndexMask &mask,
                   MutableSpan<T> dst,
                   const int64_t grain_size = 4096)
{
  gather(VArray<T>::ForSpan(src), mask, dst, grain_size);
}

void gather(const GVArray &src,
            const IndexMask &mask,
            GMutableSpan dst,
            const int64_t grain_size = 4096);

void gather(GSpan src, const IndexMask &mask, GMutableSpan dst, const int64_t grain_size = 4096);

template<typename T>
inline void scatter(const VArray<T> &src,
                    const IndexMask &mask,
                    MutableSpan<T> dst,
                    const int64_t grain_size = 4096)
{
  copy<T>(IndexMask(src.size()), mask, src, dst, grain_size);
}

template<typename T>
inline void scatter(const Span<T> src,
                    const IndexMask &mask,
                    MutableSpan<T> dst,
                    const int64_t grain_size = 4096)
{
  scatter<T>(VArray<T>::ForSpan(src), mask, dst, grain_size);
}

void scatter(const GVArray &src,
             const IndexMask &mask,
             GMutableSpan dst,
             const int64_t grain_size = 4096);

void scatter(GSpan src, const IndexMask &mask, GMutableSpan dst, const int64_t grain_size = 4096);

template<typename T>
inline void copy_group_to_group(const OffsetIndices<int> src_offsets,
                                const OffsetIndices<int> dst_offsets,
                                const IndexMask &mask,
                                const VArray<T> &src,
                                MutableSpan<T> dst,
                                const int64_t grain_size = 4096)
{
  copy_groups<T>(mask, mask, src_offsets, dst_offsets, src, dst, grain_size);
}

template<typename T>
inline void copy_group_to_group(const OffsetIndices<int> src_offsets,
                                const OffsetIndices<int> dst_offsets,
                                const IndexMask &mask,
                                const Span<T> src,
                                MutableSpan<T> dst,
                                const int64_t grain_size = 4096)
{
  copy_group_to_group<T>(src_offsets, dst_offsets, mask, VArray<T>::ForSpan(src), dst, grain_size);
}

void copy_group_to_group(const OffsetIndices<int> src_offsets,
                         const OffsetIndices<int> dst_offsets,
                         const IndexMask &mask,
                         const GVArray &src,
                         GMutableSpan dst,
                         const int64_t grain_size = 4096);

void copy_group_to_group(const OffsetIndices<int> src_offsets,
                         const OffsetIndices<int> dst_offsets,
                         const IndexMask &mask,
                         GSpan src,
                         GMutableSpan dst,
                         const int64_t grain_size = 4096);

template<typename T>
inline void gather_group_to_group(const OffsetIndices<int> src_offsets,
                                  const OffsetIndices<int> dst_offsets,
                                  const IndexMask &mask,
                                  const VArray<T> &src,
                                  MutableSpan<T> dst,
                                  const int64_t grain_size = 4096)
{
  copy_groups<T>(mask, IndexMask(dst.size()), src_offsets, dst_offsets, src, dst, grain_size);
}

template<typename T>
inline void gather_group_to_group(const OffsetIndices<int> src_offsets,
                                  const OffsetIndices<int> dst_offsets,
                                  const IndexMask &mask,
                                  const Span<T> src,
                                  MutableSpan<T> dst,
                                  const int64_t grain_size = 4096)
{
  gather_group_to_group<T>(
      src_offsets, dst_offsets, mask, VArray<T>::ForSpan(src), dst, grain_size);
}

void gather_group_to_group(const OffsetIndices<int> src_offsets,
                           const OffsetIndices<int> dst_offsets,
                           const IndexMask &mask,
                           const GVArray &src,
                           GMutableSpan dst,
                           const int64_t grain_size = 4096);

void gather_group_to_group(const OffsetIndices<int> src_offsets,
                           const OffsetIndices<int> dst_offsets,
                           const IndexMask &mask,
                           GSpan src,
                           GMutableSpan dst,
                           const int64_t grain_size = 4096);

template<typename T>
inline void scatter_group_to_group(const OffsetIndices<int> src_offsets,
                                   const OffsetIndices<int> dst_offsets,
                                   const IndexMask &mask,
                                   const VArray<T> &src,
                                   MutableSpan<T> dst,
                                   const int64_t grain_size = 4096)
{
  copy<T>(IndexMask(src.size()), mask, src_offsets, dst_offsets, src, dst, grain_size);
}

template<typename T>
inline void scatter_group_to_group(const OffsetIndices<int> src_offsets,
                                   const OffsetIndices<int> dst_offsets,
                                   const IndexMask &mask,
                                   const Span<T> src,
                                   MutableSpan<T> dst,
                                   const int64_t grain_size = 4096)
{
  scatter_group_to_group<T>(
      src_offsets, dst_offsets, mask, VArray<T>::ForSpan(src), dst, grain_size);
}

void scatter_group_to_group(const OffsetIndices<int> src_offsets,
                            const OffsetIndices<int> dst_offsets,
                            const IndexMask &mask,
                            const GVArray &src,
                            GMutableSpan dst,
                            const int64_t grain_size = 4096);

void scatter_group_to_group(const OffsetIndices<int> src_offsets,
                            const OffsetIndices<int> dst_offsets,
                            const IndexMask &mask,
                            GSpan src,
                            GMutableSpan dst,
                            const int64_t grain_size = 4096);

/**
 * Fill the specified indices of the destination with the values in the source span.
 */
template<typename T, typename IndexT>
inline void scatter(const Span<T> src,
                    const Span<IndexT> indices,
                    MutableSpan<T> dst,
                    const int64_t grain_size = 4096)
{
  BLI_assert(indices.size() == src.size());
  threading::parallel_for(indices.index_range(), grain_size, [&](const IndexRange range) {
    for (const int64_t i : range) {
      dst[indices[i]] = src[i];
    }
  });
}

/**
 * Fill the destination span by gathering indexed values from the `src` array.
 */
template<typename T, typename IndexT>
inline void gather(const VArray<T> &src,
                   const Span<IndexT> indices,
                   MutableSpan<T> dst,
                   const int64_t grain_size = 4096)
{
  BLI_assert(indices.size() == dst.size());
  devirtualize_varray(src, [&](const auto &src) {
    threading::parallel_for(indices.index_range(), grain_size, [&](const IndexRange range) {
      for (const int64_t i : range) {
        dst[i] = src[indices[i]];
      }
    });
  });
}

/**
 * Fill the destination span by gathering indexed values from the `src` array.
 */
template<typename T, typename IndexT>
inline void gather(const Span<T> src,
                   const Span<IndexT> indices,
                   MutableSpan<T> dst,
                   const int64_t grain_size = 4096)
{
  gather<T, IndexT>(VArray<T>::ForSpan(src), indices, dst, grain_size);
}

template<typename T>
inline void gather_to_groups(const OffsetIndices<int> dst_offsets,
                             const IndexMask &src_selection,
                             const Span<T> src,
                             MutableSpan<T> dst)
{
  src_selection.foreach_index(GrainSize(1024), [&](const int src_i, const int dst_i) {
    dst.slice(dst_offsets[dst_i]).fill(src[src_i]);
  });
}

/**
 * Count the number of occurrences of each index.
 * \param indices: The indices to count.
 * \param counts: The number of occurrences of each index. Typically initialized to zero.
 * Must be large enough to contain the maximum index.
 *
 * \note The memory referenced by the two spans must not overlap.
 */
void count_indices(Span<int> indices, MutableSpan<int> counts);

void invert_booleans(MutableSpan<bool> span);
void invert_booleans(MutableSpan<bool> span, const IndexMask &mask);

int64_t count_booleans(const VArray<bool> &varray);
int64_t count_booleans(const VArray<bool> &varray, const IndexMask &mask);

enum class BooleanMix {
  None,
  AllFalse,
  AllTrue,
  Mixed,
};
BooleanMix booleans_mix_calc(const VArray<bool> &varray, IndexRange range_to_check);
inline BooleanMix booleans_mix_calc(const VArray<bool> &varray)
{
  return booleans_mix_calc(varray, varray.index_range());
}

/**
 * Finds all the index ranges for which consecutive values in \a span equal \a value.
 */
template<typename T> inline Vector<IndexRange> find_all_ranges(const Span<T> span, const T &value)
{
  if (span.is_empty()) {
    return Vector<IndexRange>();
  }
  Vector<IndexRange> ranges;
  int64_t length = (span.first() == value) ? 1 : 0;
  for (const int64_t i : span.index_range().drop_front(1)) {
    if (span[i - 1] == value && span[i] != value) {
      ranges.append(IndexRange::from_end_size(i, length));
      length = 0;
    }
    else if (span[i] == value) {
      length++;
    }
  }
  if (length > 0) {
    ranges.append(IndexRange::from_end_size(span.size(), length));
  }
  return ranges;
}

/**
 * Fill the span with increasing indices: 0, 1, 2, ...
 * Optionally, the start value can be provided.
 */
template<typename T> inline void fill_index_range(MutableSpan<T> span, const T start = 0)
{
  std::iota(span.begin(), span.end(), start);
}

template<typename T>
bool indexed_data_equal(const Span<T> all_values, const Span<int> indices, const Span<T> values)
{
  for (const int i : indices.index_range()) {
    if (all_values[indices[i]] != values[i]) {
      return false;
    }
  }
  return true;
}

bool indices_are_range(Span<int> indices, IndexRange range);

}  // namespace blender::array_utils
