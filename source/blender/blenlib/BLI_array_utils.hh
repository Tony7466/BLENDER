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
inline void copy(const VArray<T> &src,
                 const IndexMask &src_mask,
                 const IndexMask &dst_mask,
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

template<typename T>
inline void copy_groups(const VArray<T> &src,
                        const IndexMask &src_mask,
                        const IndexMask &dst_mask,
                        const OffsetIndices<int> src_offsets,
                        const OffsetIndices<int> dst_offsets,
                        MutableSpan<T> dst,
                        const int64_t grain_size = 4096)
{
  BLI_assert(src_mask.size() == dst_mask.size());
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
          src.materialize(IndexMask(*src_values), dst.slice(*dst_values));
          return;
        }
        if (!src_range.has_value() && !dst_range.has_value()) {
          IndexMask::foreach_segment_zipped(
              {src_local_mask, dst_local_mask}, [&](const Span<IndexMaskSegment> segments) {
                const IndexMaskSegment src_segment = segments[0];
                const IndexMaskSegment dst_segment = segments[1];
                for (const int64_t i : src_segment.index_range()) {
                  const IndexRange src_values = src_offsets[src_segment[i]];
                  const IndexRange dst_values = dst_offsets[dst_segment[i]];
                  BLI_assert(src_values.size() == dst_values.size());
                  src.materialize(IndexMask(src_values), dst.data() + dst_values->start());
                }
                return true;
              });
          return;
        }
        if (src_range.has_value()) {
          const OffsetIndices<int> src_local_offsets = src_offsets.slice(*src_range);
          dst_local_mask.foreach_index_optimized<int64_t>(
              [&](const int64_t index, const int64_t pos) {
                const IndexRange src_values = src_local_offsets[pos];
                const IndexRange dst_values = dst_offsets[index];
                BLI_assert(src_values.size() == dst_values.size());
                src.materialize(IndexMask(src_values), dst.data() + dst_values->start());
              });
          return;
        }
        if (dst_range.has_value()) {
          const OffsetIndices<int> dst_local_offsets = dst_offsets.slice(*dst_range);
          src_local_mask.foreach_index_optimized<int64_t>(
              [&](const int64_t index, const int64_t pos) {
                const IndexRange src_values = src_offsets[index];
                const IndexRange dst_values = dst_local_offsets[pos];
                BLI_assert(src_values.size() == dst_values.size());
                src.materialize(IndexMask(src_values), dst.slice(*dst_values));
              });
        }
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
        const int64_t src_min_bound = src_offsets[src_local_mask.bounds()].size();
        const int64_t dst_min_bound = dst_offsets[dst_local_mask.bounds()].size();
        return std::min(src_min_bound, dst_min_bound);
      }));
}

template<>
inline void copy<bool>(const VArray<bool> &src,
                       const IndexMask &src_mask,
                       const IndexMask &dst_mask,
                       MutableSpan<bool> dst,
                       int64_t grain_size);
template<>
inline void copy<int>(const VArray<int> &src,
                      const IndexMask &src_mask,
                      const IndexMask &dst_mask,
                      MutableSpan<int> dst,
                      int64_t grain_size);
template<>
inline void copy<int2>(const VArray<int2> &src,
                       const IndexMask &src_mask,
                       const IndexMask &dst_mask,
                       MutableSpan<int2> dst,
                       int64_t grain_size);
template<>
inline void copy<int3>(const VArray<int3> &src,
                       const IndexMask &src_mask,
                       const IndexMask &dst_mask,
                       MutableSpan<int3> dst,
                       int64_t grain_size);
template<>
inline void copy<int8_t>(const VArray<int8_t> &src,
                         const IndexMask &src_mask,
                         const IndexMask &dst_mask,
                         MutableSpan<int8_t> dst,
                         int64_t grain_size);
template<>
inline void copy<int64_t>(const VArray<int64_t> &src,
                          const IndexMask &src_mask,
                          const IndexMask &dst_mask,
                          MutableSpan<int64_t> dst,
                          int64_t grain_size);
template<>
inline void copy<float>(const VArray<float> &src,
                        const IndexMask &src_mask,
                        const IndexMask &dst_mask,
                        MutableSpan<float> dst,
                        int64_t grain_size);
template<>
inline void copy<float2>(const VArray<float2> &src,
                         const IndexMask &src_mask,
                         const IndexMask &dst_mask,
                         MutableSpan<float2> dst,
                         int64_t grain_size);
template<>
inline void copy<float3>(const VArray<float3> &src,
                         const IndexMask &src_mask,
                         const IndexMask &dst_mask,
                         MutableSpan<float3> dst,
                         int64_t grain_size);
template<>
inline void copy<float4>(const VArray<float4> &src,
                         const IndexMask &src_mask,
                         const IndexMask &dst_mask,
                         MutableSpan<float4> dst,
                         int64_t grain_size);
template<>
inline void copy<float2x2>(const VArray<float2x2> &src,
                           const IndexMask &src_mask,
                           const IndexMask &dst_mask,
                           MutableSpan<float2x2> dst,
                           int64_t grain_size);
template<>
inline void copy<float3x3>(const VArray<float3x3> &src,
                           const IndexMask &src_mask,
                           const IndexMask &dst_mask,
                           MutableSpan<float3x3> dst,
                           int64_t grain_size);
template<>
inline void copy<float4x4>(const VArray<float4x4> &src,
                           const IndexMask &src_mask,
                           const IndexMask &dst_mask,
                           MutableSpan<float4x4> dst,
                           int64_t grain_size);
template<>
inline void copy<math::Quaternion>(const VArray<math::Quaternion> &src,
                                   const IndexMask &src_mask,
                                   const IndexMask &dst_mask,
                                   MutableSpan<math::Quaternion> dst,
                                   int64_t grain_size);
template<>
inline void copy<std::string>(const VArray<std::string> &src,
                              const IndexMask &src_mask,
                              const IndexMask &dst_mask,
                              MutableSpan<std::string> dst,
                              int64_t grain_size);

template<>
inline void copy_groups<bool>(const VArray<bool> &src,
                              const IndexMask &src_mask,
                              const IndexMask &dst_mask,
                              const OffsetIndices<int> src_offsets,
                              const OffsetIndices<int> dst_offsets,
                              MutableSpan<bool> dst,
                              int64_t grain_size);
template<>
inline void copy_groups<int>(const VArray<int> &src,
                             const IndexMask &src_mask,
                             const IndexMask &dst_mask,
                             const OffsetIndices<int> src_offsets,
                             const OffsetIndices<int> dst_offsets,
                             MutableSpan<int> dst,
                             int64_t grain_size);
template<>
inline void copy_groups<int2>(const VArray<int2> &src,
                              const IndexMask &src_mask,
                              const IndexMask &dst_mask,
                              const OffsetIndices<int> src_offsets,
                              const OffsetIndices<int> dst_offsets,
                              MutableSpan<int2> dst,
                              int64_t grain_size);
template<>
inline void copy_groups<int3>(const VArray<int3> &src,
                              const IndexMask &src_mask,
                              const IndexMask &dst_mask,
                              const OffsetIndices<int> src_offsets,
                              const OffsetIndices<int> dst_offsets,
                              MutableSpan<int3> dst,
                              int64_t grain_size);
template<>
inline void copy_groups<int8_t>(const VArray<int8_t> &src,
                                const IndexMask &src_mask,
                                const IndexMask &dst_mask,
                                const OffsetIndices<int> src_offsets,
                                const OffsetIndices<int> dst_offsets,
                                MutableSpan<int8_t> dst,
                                int64_t grain_size);
template<>
inline void copy_groups<int64_t>(const VArray<int64_t> &src,
                                 const IndexMask &src_mask,
                                 const IndexMask &dst_mask,
                                 const OffsetIndices<int> src_offsets,
                                 const OffsetIndices<int> dst_offsets,
                                 MutableSpan<int64_t> dst,
                                 int64_t grain_size);
template<>
inline void copy_groups<float>(const VArray<float> &src,
                               const IndexMask &src_mask,
                               const IndexMask &dst_mask,
                               const OffsetIndices<int> src_offsets,
                               const OffsetIndices<int> dst_offsets,
                               MutableSpan<float> dst,
                               int64_t grain_size);
template<>
inline void copy_groups<float2>(const VArray<float2> &src,
                                const IndexMask &src_mask,
                                const IndexMask &dst_mask,
                                const OffsetIndices<int> src_offsets,
                                const OffsetIndices<int> dst_offsets,
                                MutableSpan<float2> dst,
                                int64_t grain_size);
template<>
inline void copy_groups<float3>(const VArray<float3> &src,
                                const IndexMask &src_mask,
                                const IndexMask &dst_mask,
                                const OffsetIndices<int> src_offsets,
                                const OffsetIndices<int> dst_offsets,
                                MutableSpan<float3> dst,
                                int64_t grain_size);
template<>
inline void copy_groups<float4>(const VArray<float4> &src,
                                const IndexMask &src_mask,
                                const IndexMask &dst_mask,
                                const OffsetIndices<int> src_offsets,
                                const OffsetIndices<int> dst_offsets,
                                MutableSpan<float4> dst,
                                int64_t grain_size);
template<>
inline void copy_groups<float2x2>(const VArray<float2x2> &src,
                                  const IndexMask &src_mask,
                                  const IndexMask &dst_mask,
                                  const OffsetIndices<int> src_offsets,
                                  const OffsetIndices<int> dst_offsets,
                                  MutableSpan<float2x2> dst,
                                  int64_t grain_size);
template<>
inline void copy_groups<float3x3>(const VArray<float3x3> &src,
                                  const IndexMask &src_mask,
                                  const IndexMask &dst_mask,
                                  const OffsetIndices<int> src_offsets,
                                  const OffsetIndices<int> dst_offsets,
                                  MutableSpan<float3x3> dst,
                                  int64_t grain_size);
template<>
inline void copy_groups<float4x4>(const VArray<float4x4> &src,
                                  const IndexMask &src_mask,
                                  const IndexMask &dst_mask,
                                  const OffsetIndices<int> src_offsets,
                                  const OffsetIndices<int> dst_offsets,
                                  MutableSpan<float4x4> dst,
                                  int64_t grain_size);
template<>
inline void copy_groups<math::Quaternion>(const VArray<math::Quaternion> &src,
                                          const IndexMask &src_mask,
                                          const IndexMask &dst_mask,
                                          const OffsetIndices<int> src_offsets,
                                          const OffsetIndices<int> dst_offsets,
                                          MutableSpan<math::Quaternion> dst,
                                          int64_t grain_size);
template<>
inline void copy_groups<std::string>(const VArray<std::string> &src,
                                     const IndexMask &src_mask,
                                     const IndexMask &dst_mask,
                                     const OffsetIndices<int> src_offsets,
                                     const OffsetIndices<int> dst_offsets,
                                     MutableSpan<std::string> dst,
                                     int64_t grain_size);

template<typename T>
inline void copy(const Span<T> src,
                 const IndexMask &src_mask,
                 const IndexMask &dst_mask,
                 MutableSpan<T> dst,
                 const int64_t grain_size = 4096)
{
  copy<T>(VArray<T>::ForSpan(src), src_mask, dst_mask, dst, grain_size);
}

inline void copy(const GVArray &src,
                 const IndexMask &src_mask,
                 const IndexMask &dst_mask,
                 GMutableSpan dst,
                 const int64_t grain_size = 4096);

inline void copy(GSpan src,
                 const IndexMask &src_mask,
                 const IndexMask &dst_mask,
                 GMutableSpan dst,
                 const int64_t grain_size = 4096);

template<typename T>
inline void copy_groups(Span<T> src,
                        const IndexMask &src_mask,
                        const IndexMask &dst_mask,
                        const OffsetIndices<int> src_offsets,
                        const OffsetIndices<int> dst_offsets,
                        MutableSpan<T> dst,
                        const int64_t grain_size = 4096)
{
  copy_groups(
      VArray<T>::ForSpan(src), src_mask, dst_mask, src_offsets, dst_offsets, dst, grain_size);
}

inline void copy_groups(const GVArray &src,
                        const IndexMask &src_mask,
                        const IndexMask &dst_mask,
                        const OffsetIndices<int> src_offsets,
                        const OffsetIndices<int> dst_offsets,
                        GMutableSpan dst,
                        const int64_t grain_size = 4096);

inline void copy_groups(GSpan src,
                        const IndexMask &src_mask,
                        const IndexMask &dst_mask,
                        const OffsetIndices<int> src_offsets,
                        const OffsetIndices<int> dst_offsets,
                        GMutableSpan dst,
                        const int64_t grain_size = 4096);

template<typename T>
inline void copy(const VArray<T> &src,
                 const IndexMask &mask,
                 MutableSpan<T> dst,
                 const int64_t grain_size = 4096)
{
  copy<T>(src, mask, mask, dst, grain_size);
}

template<typename T>
inline void copy(const Span<T> src,
                 const IndexMask &mask,
                 MutableSpan<T> dst,
                 const int64_t grain_size = 4096)
{
  copy<T>(src, mask, mask, dst, grain_size);
}

inline void copy(const GVArray &src,
                 const IndexMask &mask,
                 GMutableSpan dst,
                 const int64_t grain_size = 4096);

inline void copy(GSpan src,
                 const IndexMask &mask,
                 GMutableSpan dst,
                 const int64_t grain_size = 4096);

template<typename T>
inline void copy(const VArray<T> &src, MutableSpan<T> dst, const int64_t grain_size = 4096)
{
  copy<T>(src, IndexMask(src.size()), IndexMask(dst.size()), dst, grain_size);
}

template<typename T>
inline void copy(const Span<T> src, MutableSpan<T> dst, const int64_t grain_size = 4096)
{
  copy<T>(src, IndexMask(src.size()), IndexMask(dst.size()), dst, grain_size);
}

inline void copy(const GVArray &src, GMutableSpan dst, const int64_t grain_size = 4096);

inline void copy(GSpan src, GMutableSpan dst, const int64_t grain_size = 4096);

template<typename T>
inline void gather(const VArray<T> &src,
                   const IndexMask &mask,
                   MutableSpan<T> dst,
                   const int64_t grain_size = 4096)
{
  copy<T>(src, mask, IndexMask(dst.size()), dst, grain_size);
}

template<typename T>
inline void gather(const Span<T> src,
                   const IndexMask &mask,
                   MutableSpan<T> dst,
                   const int64_t grain_size = 4096)
{
  copy<T>(src, mask, IndexMask(dst.size()), dst, grain_size);
}

inline void gather(const GVArray &src,
                   const IndexMask &mask,
                   GMutableSpan dst,
                   const int64_t grain_size = 4096);

inline void gather(GSpan src,
                   const IndexMask &mask,
                   GMutableSpan dst,
                   const int64_t grain_size = 4096);

template<typename T>
inline void scatter(const VArray<T> &src,
                    const IndexMask &mask,
                    MutableSpan<T> dst,
                    const int64_t grain_size = 4096)
{
  copy<T>(src, IndexMask(src.size()), mask, dst, grain_size);
}

template<typename T>
inline void scatter(const Span<T> src,
                    const IndexMask &mask,
                    MutableSpan<T> dst,
                    const int64_t grain_size = 4096)
{
  copy<T>(src, IndexMask(src.size()), mask, dst, grain_size);
}

inline void scatter(const GVArray &src,
                    const IndexMask &mask,
                    GMutableSpan dst,
                    const int64_t grain_size = 4096);

inline void scatter(GSpan src,
                    const IndexMask &mask,
                    GMutableSpan dst,
                    const int64_t grain_size = 4096);

template<typename T>
inline void copy_group_to_group(const OffsetIndices<int> src_offsets,
                                const OffsetIndices<int> dst_offsets,
                                const VArray<T> &src,
                                MutableSpan<T> dst,
                                const int64_t grain_size = 4096)
{
  copy<T>(src,
          IndexMask(src.size()),
          IndexMask(dst.size()),
          src_offsets,
          dst_offsets,
          dst,
          grain_size);
}

template<typename T>
inline void copy_group_to_group(const OffsetIndices<int> src_offsets,
                                const OffsetIndices<int> dst_offsets,
                                const Span<T> src,
                                MutableSpan<T> dst,
                                const int64_t grain_size = 4096)
{
  copy<T>(src,
          IndexMask(src.size()),
          IndexMask(dst.size()),
          src_offsets,
          dst_offsets,
          dst,
          grain_size);
}

inline void copy_group_to_group(const OffsetIndices<int> src_offsets,
                                const OffsetIndices<int> dst_offsets,
                                const GVArray &src,
                                GMutableSpan dst,
                                const int64_t grain_size = 4096);

inline void copy_group_to_group(const OffsetIndices<int> src_offsets,
                                const OffsetIndices<int> dst_offsets,
                                GSpan src,
                                GMutableSpan dst,
                                const int64_t grain_size = 4096);

template<typename T>
inline void gather_group_to_group(const OffsetIndices<int> src_offsets,
                                  const OffsetIndices<int> dst_offsets,
                                  const VArray<T> &src,
                                  const IndexMask &mask,
                                  MutableSpan<T> dst,
                                  const int64_t grain_size = 4096)
{
  copy<T>(src, mask, IndexMask(dst.size()), src_offsets, dst_offsets, dst, grain_size);
}

template<typename T>
inline void gather_group_to_group(const OffsetIndices<int> src_offsets,
                                  const OffsetIndices<int> dst_offsets,
                                  const Span<T> src,
                                  const IndexMask &mask,
                                  MutableSpan<T> dst,
                                  const int64_t grain_size = 4096)
{
  copy<T>(src, mask, IndexMask(dst.size()), src_offsets, dst_offsets, dst, grain_size);
}

inline void gather_group_to_group(const OffsetIndices<int> src_offsets,
                                  const OffsetIndices<int> dst_offsets,
                                  const GVArray &src,
                                  const IndexMask &mask,
                                  GMutableSpan dst,
                                  const int64_t grain_size = 4096);

inline void gather_group_to_group(const OffsetIndices<int> src_offsets,
                                  const OffsetIndices<int> dst_offsets,
                                  GSpan src,
                                  const IndexMask &mask,
                                  GMutableSpan dst,
                                  const int64_t grain_size = 4096);

template<typename T>
inline void scatter_group_to_group(const OffsetIndices<int> src_offsets,
                                   const OffsetIndices<int> dst_offsets,
                                   const VArray<T> &src,
                                   const IndexMask &mask,
                                   MutableSpan<T> dst,
                                   const int64_t grain_size = 4096)
{
  copy<T>(src, IndexMask(src.size()), mask, src_offsets, dst_offsets, dst, grain_size);
}

template<typename T>
inline void scatter_group_to_group(const OffsetIndices<int> src_offsets,
                                   const OffsetIndices<int> dst_offsets,
                                   const Span<T> src,
                                   const IndexMask &mask,
                                   MutableSpan<T> dst,
                                   const int64_t grain_size = 4096)
{
  copy<T>(src, IndexMask(src.size()), mask, src_offsets, dst_offsets, dst, grain_size);
}

inline void scatter_group_to_group(const OffsetIndices<int> src_offsets,
                                   const OffsetIndices<int> dst_offsets,
                                   const GVArray &src,
                                   const IndexMask &mask,
                                   GMutableSpan dst,
                                   const int64_t grain_size = 4096);

inline void scatter_group_to_group(const OffsetIndices<int> src_offsets,
                                   const OffsetIndices<int> dst_offsets,
                                   GSpan src,
                                   const IndexMask &mask,
                                   GMutableSpan dst,
                                   const int64_t grain_size = 4096);

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
