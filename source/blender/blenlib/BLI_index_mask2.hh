/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <array>

#include "BLI_index_range.hh"

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

const IndexMask &get_static_index_mask_for_min_size(const int64_t min_size);

/**
 * A #Chunk contains an ordered list of segments. Each segment is an array of 16-bit integers.
 */
struct Chunk {
  int16_t segments_num;
  int16_t indices_num;
  const int16_t **segment_indices;
  const int16_t *segment_sizes;
  const int16_t *segment_sizes_cumulative;
};

struct ChunkIteratorData {
  int16_t segment_index;
  int16_t index_in_segment;
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

class IndexMaskIterator {
 private:
  IndexMaskIteratorData data_;
  const IndexMask *mask_;

  friend IndexMask;

 public:
  IndexMaskIterator &operator++();

  friend bool operator!=(const IndexMaskIterator &a, const IndexMaskIterator &b);
  friend bool operator==(const IndexMaskIterator &a, const IndexMaskIterator &b);
};

class IndexMask {
 private:
  IndexMaskData data_;

  friend IndexMaskIterator;

 public:
  IndexMask();
  IndexMask(int64_t size);
  IndexMask(IndexRange range);

  IndexMaskIterator begin() const;
  IndexMaskIterator end() const;

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
/** \name #IndexMaskIterator Inline Methods
 * \{ */

inline IndexMaskIterator &IndexMaskIterator::operator++()
{
  const Chunk &current_chunk = mask_->data_.chunks[data_.chunk_index];
  const int16_t current_segment_size = current_chunk.segment_sizes[data_.chunk_it.segment_index];
  data_.chunk_it.index_in_segment++;
  if (data_.chunk_it.index_in_segment == current_segment_size) {
    data_.chunk_it.segment_index++;
    data_.chunk_it.index_in_segment = 0;
  }
  return *this;
}

inline bool operator!=(const IndexMaskIterator &a, const IndexMaskIterator &b)
{
  return a.data_ != b.data_;
}

inline bool operator==(const IndexMaskIterator &a, const IndexMaskIterator &b)
{
  return a.data_ != b.data_;
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

inline IndexMaskIterator IndexMask::begin() const
{
  IndexMaskIterator it;
  it.data_.chunk_index = 0;
  it.data_.chunk_it = data_.begin_it;
  it.mask_ = this;
  return it;
}

inline IndexMaskIterator IndexMask::end() const
{
  IndexMaskIterator it;
  it.data_.chunk_index = data_.chunks_num;
  it.data_.chunk_it.segment_index = 0;
  it.data_.chunk_it.index_in_segment = 0;
  it.mask_ = this;
  return it;
}

inline const IndexMaskData &IndexMask::data() const
{
  return data_;
}

inline IndexMaskData &IndexMask::data_for_inplace_construction()
{
  return const_cast<IndexMaskData &>(data_);
}

}  // namespace index_mask

using index_mask::IndexMask;

}  // namespace blender
