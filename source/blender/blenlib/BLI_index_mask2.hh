/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <array>

#include "BLI_index_range.hh"
#include "BLI_span.hh"

namespace blender {
namespace index_mask {

class IndexMask;

/* Chunks contain up to 2^14 = 16384 indices. */
static constexpr int64_t chunk_size_shift = 14;
static constexpr int64_t chunk_mask_low = (1 << chunk_size_shift) - 1;
static constexpr int64_t chunk_mask_high = ~chunk_mask_low;
static constexpr int64_t max_chunk_size = (1 << chunk_size_shift);

std::array<int16_t, max_chunk_size> build_static_indices_array();

inline const std::array<int16_t, max_chunk_size> &get_static_indices_array()
{
  alignas(64) static const std::array<int16_t, max_chunk_size> data = build_static_indices_array();
  return data;
}

inline int64_t index_to_chunk_index(const int64_t i)
{
  return i >> chunk_size_shift;
}

inline int64_t size_to_chunk_num(const int64_t size)
{
  return (size + max_chunk_size - 1) >> chunk_size_shift;
}

const IndexMask &get_static_index_mask_for_min_size(const int64_t min_size);

struct ChunkIteratorData {
  int16_t segment_index;
  int16_t index_in_segment;
};

/**
 * A #Chunk contains an ordered list of segments. Each segment is an array of 16-bit integers.
 */
struct Chunk {
  int16_t segments_num;
  int16_t indices_num;
  const int16_t **segment_indices;
  const int16_t *segment_sizes;
  const int16_t *segment_sizes_cumulative;

  ChunkIteratorData end_data() const;
};

struct IndexMaskData {
  int64_t chunks_num;
  int64_t indices_num;
  const Chunk *chunks;
  const int64_t *chunk_offsets;
  const int64_t *chunk_sizes_cumulative;
  ChunkIteratorData begin_it;
  ChunkIteratorData end_it;
};

struct IndexMaskIteratorData {
  int64_t chunk_index;
  ChunkIteratorData chunk_it;
};

class IndexMask {
 private:
  IndexMaskData data_;

 public:
  IndexMask();
  IndexMask(int64_t size);
  IndexMask(IndexRange range);

  IndexMask slice(IndexRange range) const;

  template<typename Fn> void foreach_chunk(const Fn &fn);
  template<typename Fn> void foreach_index_range(const Fn &fn);
  template<typename Fn> void foreach_index_span(const Fn &fn);
  template<typename Fn> void foreach_index(const Fn &fn);
  template<typename Fn> void foreach_index_range_or_span(const Fn &fn);

  const IndexMaskData &data() const;
  IndexMaskData &data_for_inplace_construction();
};

/* -------------------------------------------------------------------- */
/** \name #ChunkIteratorData Inline Methods
 * \{ */

inline bool operator!=(const ChunkIteratorData &a, const ChunkIteratorData &b)
{
  return a.index_in_segment != b.index_in_segment || a.segment_index != b.segment_index;
}

/* -------------------------------------------------------------------- */
/** \name #IndexMaskIteratorData Inline Methods
 * \{ */

inline bool operator!=(const IndexMaskIteratorData &a, const IndexMaskIteratorData &b)
{
  return a.chunk_it != b.chunk_it || a.chunk_index != b.chunk_index;
}

inline bool operator==(const IndexMaskIteratorData &a, const IndexMaskIteratorData &b)
{
  return !(a != b);
}

/* -------------------------------------------------------------------- */
/** \name #IndexMask Inline Methods
 * \{ */

inline IndexMask::IndexMask()
{
  static constexpr int64_t cumulative_sizes_for_empty_mask[1] = {0};

  data_.chunks_num = 0;
  data_.chunks = nullptr;
  data_.chunk_offsets = nullptr;
  data_.chunk_sizes_cumulative = cumulative_sizes_for_empty_mask;
  data_.begin_it.segment_index = 0;
  data_.begin_it.index_in_segment = 0;
  data_.end_it.segment_index = 0;
  data_.end_it.index_in_segment = 0;
}

inline IndexMask::IndexMask(const int64_t size)
{
  *this = get_static_index_mask_for_min_size(size);
  data_.chunks_num = size_to_chunk_num(size);
  data_.end_it.index_in_segment = size & chunk_mask_low;
}

inline IndexMask::IndexMask(const IndexRange range)
{
  if (range.is_empty()) {
    return;
  }
  *this = get_static_index_mask_for_min_size(range.one_after_last());

  const int64_t first_chunk_i = index_to_chunk_index(range.first());
  const int64_t last_chunk_i = index_to_chunk_index(range.last());

  data_.chunks_num = last_chunk_i - first_chunk_i + 1;
  data_.indices_num = range.size();
  data_.chunks -= first_chunk_i;
  data_.chunk_offsets -= first_chunk_i;
  data_.chunk_sizes_cumulative -= first_chunk_i;
  data_.begin_it.segment_index = 0;
  data_.begin_it.index_in_segment = range.first() & chunk_mask_low;
  data_.end_it.segment_index = 0;
  data_.end_it.index_in_segment = range.one_after_last() & chunk_mask_low;
}

inline const IndexMaskData &IndexMask::data() const
{
  return data_;
}

inline IndexMaskData &IndexMask::data_for_inplace_construction()
{
  return const_cast<IndexMaskData &>(data_);
}

inline ChunkIteratorData Chunk::end_data() const
{
  ChunkIteratorData data;
  if (this->segments_num > 0) {
    data.segment_index = this->segments_num - 1;
    data.index_in_segment = this->segment_sizes[0];
  }
  else {
    data.segment_index = 0;
    data.index_in_segment = 0;
  }
  return data;
}

template<typename Fn> inline void IndexMask::foreach_chunk(const Fn &fn)
{
  if (data_.chunks_num == 0) {
    return;
  }
  if (data_.chunks_num == 1) {
    fn(0, data_.begin_it, data_.end_it);
    return;
  }
  {
    /* First chunk. */
    const Chunk &chunk = data_.chunks[0];
    fn(0, data_.begin_it, chunk.end_data());
  }
  /* Middle chunks. */
  for (int64_t chunk_i = 1; chunk_i < data_.chunks_num - 1; chunk_i++) {
    const Chunk &chunk = data_.chunks[chunk_i];
    const ChunkIteratorData chunk_begin = {0, 0};
    fn(chunk_i, chunk_begin, chunk.end_data());
  }
  {
    /* Last chunk */
    const ChunkIteratorData chunk_begin = {0, 0};
    fn(data_.chunks_num - 1, chunk_begin, data_.end_it);
  }
}

template<typename Fn> inline void IndexMask::foreach_index_span(const Fn &fn)
{
  this->foreach_chunk([&](const int64_t chunk_i,
                          const ChunkIteratorData begin_it,
                          const ChunkIteratorData end_it) {
    const int64_t offset = data_.chunk_offsets[chunk_i];
    const Chunk &chunk = data_.chunks[chunk_i];
    for (int16_t segment_i = begin_it.segment_index; segment_i <= end_it.segment_index;
         segment_i++) {
      const int16_t segment_start_i = (segment_i == begin_it.segment_index) ?
                                          begin_it.index_in_segment :
                                          0;
      const int16_t segment_end_i = (segment_i == end_it.segment_index) ?
                                        end_it.index_in_segment :
                                        chunk.segment_sizes[segment_i];
      const int16_t segment_size = segment_end_i - segment_start_i;
      const Span<int16_t> indices{chunk.segment_indices[segment_i] + segment_start_i,
                                  segment_size};
      fn(offset, indices);
    }
  });
}

}  // namespace index_mask

using index_mask::IndexMask;

}  // namespace blender
