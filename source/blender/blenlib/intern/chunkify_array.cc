/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_chunkify_array.hh"
#include "BLI_task.hh"

namespace blender {

static uint64_t integer_power(const uint64_t base, const uint64_t exponent)
{
  uint64_t result = 1;
  for (uint64_t i = 0; i < exponent; i++) {
    result *= base;
  }
  return result;
}

static const int64_t determine_chunk_boundary(const void *data,
                                              const int64_t bytes_num,
                                              const int64_t chunk_i,
                                              const int64_t window_size,
                                              const uint64_t add_factor,
                                              const uint64_t remove_factor,
                                              const int64_t approximate_bytes_per_chunk)
{
  const int64_t begin_window_start = chunk_i * approximate_bytes_per_chunk;
  const int64_t end_window_start = std::min<int64_t>(
      begin_window_start + approximate_bytes_per_chunk, bytes_num - window_size);

  if (begin_window_start >= end_window_start) {
    /* The chunk is too small for a window. Just pick the start as the chunk boundary. */
    return begin_window_start;
  }

  const uint8_t *data_int = static_cast<const uint8_t *>(data);

  uint64_t rolling_hash = 0;
  /* Initialize rolling hash with the first few elements. Every iteration, one element is added
   * to the hash. */
  for (int64_t i = begin_window_start; i < begin_window_start + window_size - 1; i++) {
    rolling_hash = rolling_hash * add_factor + data_int[i];
  }

  /* Remember window with the "best" hash because it will become the chunk boundary. */
  uint64_t best_hash = 0;
  int64_t best_window_start = begin_window_start;

  /* Every iteration finishes a hash. One element is removed and element is added to the hash. */
  for (int64_t window_start = begin_window_start; window_start < end_window_start; window_start++)
  {
    const int64_t window_last = window_start + window_size - 1;
    /* Add element to rolling hash. */
    rolling_hash = rolling_hash * add_factor + data_int[window_last];
    /* Update best hash. */
    if (rolling_hash > best_hash) {
      best_hash = rolling_hash;
      best_window_start = window_start;
    }
    /* Remove element from rolling hash. */
    rolling_hash -= data_int[window_start] * remove_factor;
  }

  return best_window_start;
}

OffsetIndices<int64_t> chunkify_array(const void *data,
                                      const int64_t bytes_num,
                                      const ChunkifyArrayParams &params,
                                      Vector<int64_t> &r_offsets)
{
  BLI_assert(is_power_of_2(params.element_size));
  BLI_assert(bytes_num >= 0);
  const int64_t approximate_bytes_per_chunk = params.element_size *
                                              params.approximate_elements_per_chunk;
  /* A somewhat arbitrary minimum to make sure that the algorithm works as expected. */
  BLI_assert(approximate_bytes_per_chunk >= 8);
  BLI_assert(params.element_size <= approximate_bytes_per_chunk);
  if (bytes_num == 0) {
    return {};
  }
  BLI_assert(data != nullptr);

  const int64_t chunks_num = (bytes_num + approximate_bytes_per_chunk - 1) /
                             approximate_bytes_per_chunk;

  r_offsets.resize(chunks_num + 2);
  r_offsets[0] = 0;
  r_offsets.last() = bytes_num;

  const int64_t type_alignment_mask = ~(uint64_t(params.element_size) - 1);

  /* Must be smaller than the chunk size, but the exact size does not matter too much. */
  const int64_t window_size = std::max((approximate_bytes_per_chunk / 4) & type_alignment_mask,
                                       params.element_size);

  constexpr uint64_t add_factor = 892181971;
  const uint64_t remove_factor = integer_power(add_factor, window_size - 1);

  const int64_t grain_size = std::max<int64_t>(1, 4096 / approximate_bytes_per_chunk);
  threading::parallel_for(IndexRange(chunks_num), grain_size, [&](const IndexRange range) {
    for (const int64_t chunk_i : range) {
      int64_t chunk_boundary = determine_chunk_boundary(data,
                                                        bytes_num,
                                                        chunk_i,
                                                        window_size,
                                                        add_factor,
                                                        remove_factor,
                                                        approximate_bytes_per_chunk);
      chunk_boundary = chunk_boundary & type_alignment_mask;
      r_offsets[chunk_i + 1] = chunk_boundary;
    }
  });

  if (r_offsets[1] == 0) {
    /* Skip the first chunk because it has zero size. */
    return r_offsets.as_span().drop_front(1);
  }
  return r_offsets.as_span();
}

}  // namespace blender
