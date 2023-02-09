/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_vector.hh"

namespace blender {

static constexpr int64_t index_mask_chunk_shift = 14;
static constexpr int64_t index_mask_chunk_mask_low = (1 << index_mask_chunk_shift) - 1;
static constexpr int64_t index_mask_chunk_mask_high = ~index_mask_chunk_mask_low;
static constexpr int64_t max_index_mask_chunk_size = (1 << index_mask_chunk_shift);

inline const std::array<int16_t, max_index_mask_chunk_size> &get_static_offsets_array()
{
  static const std::array<int16_t, max_index_mask_chunk_size> data = []() {
    static std::array<int16_t, max_index_mask_chunk_size> data;
    for (int16_t i = 0; i < max_index_mask_chunk_size; i++) {
      data[i] = i;
    }
    return data;
  }();
  return data;
}

class IndexMaskChunk {
 private:
  int64_t start_ = 0;
  Span<int16_t> offsets_;

 public:
  IndexMaskChunk() = default;

  IndexMaskChunk(const int64_t start, const Span<int16_t> offsets)
      : start_(start), offsets_(offsets)
  {
    BLI_assert((start & index_mask_chunk_mask_low) == 0);
    BLI_assert(IndexMaskChunk::offsets_are_valid(offsets));
  }

  constexpr int64_t size() const
  {
    return offsets_.size();
  }

  IndexRange index_range() const
  {
    return offsets_.index_range();
  }

  bool is_range() const
  {
    return offsets_.size() > 0 && offsets_.last() - offsets_.first() == offsets_.size() - 1;
  }

  IndexRange as_range() const
  {
    BLI_assert(this->is_range());
    return IndexRange{start_ + offsets_.first(), offsets_.size()};
  }

  int64_t start() const
  {
    return start_;
  }

  Span<int16_t> offsets() const
  {
    return offsets_;
  }

  constexpr int64_t operator[](const int64_t i) const
  {
    return start_ + offsets_[i];
  }

  static bool offsets_are_valid(const Span<int16_t> offsets)
  {
    if (offsets.is_empty()) {
      return true;
    }
    if (offsets.first() < 0) {
      return false;
    }
    if (offsets.last() >= max_index_mask_chunk_size) {
      return false;
    }
    for (int64_t i = 1; i < offsets.size(); i++) {
      if (offsets[i - 1] >= offsets[i]) {
        return false;
      }
    }
    return true;
  }
};

class IndexMask2 {
 private:
  int64_t size_ = 0;
  Span<IndexMaskChunk> chunks_;

 public:
  IndexMask2() = default;

  IndexMask2(const Span<IndexMaskChunk> chunks) : IndexMask2(chunks, this->sum_chunk_sizes())
  {
  }

  IndexMask2(const Span<IndexMaskChunk> chunks, const int64_t size) : size_(size), chunks_(chunks)
  {
    BLI_assert(size_ == this->sum_chunk_sizes());
#ifdef DEBUG
    for (int i = 1; i < chunks.size(); i++) {
      BLI_assert(chunks[i - 1].start() < chunks[i].start());
    }
#endif
  }

  class Iterator {
   public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = int64_t;

   private:
    IndexMaskChunk current_chunk_;
    int64_t index_in_chunk_;
    int64_t chunk_index_;
    const IndexMask2 *mask_;

    friend IndexMask2;

   public:
    constexpr Iterator &operator++()
    {
      index_in_chunk_++;
      if (index_in_chunk_ == current_chunk_.size()) {
        chunk_index_++;
        index_in_chunk_ = 0;
      }
      return *this;
    }

    constexpr friend bool operator!=(const Iterator &a, const Iterator &b)
    {
      BLI_assert(a.mask_ == b.mask_);
      return a.chunk_index_ != b.chunk_index_ || a.index_in_chunk_ != b.index_in_chunk_;
    }

    constexpr friend bool operator==(const Iterator &a, const Iterator &b)
    {
      return !(a != b);
    }

    constexpr int64_t operator*() const
    {
      return current_chunk_[index_in_chunk_];
    }
  };

  Iterator begin() const
  {
    Iterator it;
    it.mask_ = this;
    it.chunk_index_ = 0;
    it.index_in_chunk_ = 0;
    if (!chunks_.is_empty()) {
      it.current_chunk_ = chunks_[0];
    }
    return it;
  }

  Iterator end() const
  {
    Iterator it;
    it.mask_ = this;
    it.chunk_index_ = chunks_.size();
    it.index_in_chunk_ = 0;
    return it;
  }

 private:
  int64_t sum_chunk_sizes() const
  {
    int64_t size = 0;
    for (const IndexMaskChunk &chunk : chunks_) {
      size += chunk.size();
    }
    return size;
  }
};

class IndexMaskForRange {
 private:
  Vector<IndexMaskChunk> chunks_;
  const int64_t size_;

 public:
  IndexMaskForRange(const IndexRange range) : size_(range.size())
  {
    if (range.is_empty()) {
      return;
    }
    const int64_t global_start = range.start();
    const int64_t global_last = range.last();

    const int64_t first_chunk_start = global_start & index_mask_chunk_mask_high;
    const int64_t last_chunk_start = global_last & index_mask_chunk_mask_high;

    const Span<int16_t> static_offsets = get_static_offsets_array();

    chunks_.reserve(((last_chunk_start - first_chunk_start) >> index_mask_chunk_shift) + 1);

    if (first_chunk_start == last_chunk_start) {
      /* Only a single chunk is necessary. */
      const int64_t first_offset = global_start & index_mask_chunk_mask_low;
      const int64_t offsets_num = range.size();
      chunks_.append_unchecked_as(first_chunk_start,
                                  static_offsets.slice(first_offset, offsets_num));
    }
    else {
      {
        /* Add first chunk. */
        const int64_t first_offset = global_start & index_mask_chunk_mask_low;
        chunks_.append_unchecked_as(first_chunk_start, static_offsets.drop_front(first_offset));
      }
      /* Add full chunks in the middle. */
      for (int64_t chunk_start = first_chunk_start + max_index_mask_chunk_size;
           chunk_start < last_chunk_start;
           chunk_start += max_index_mask_chunk_size) {
        chunks_.append_unchecked_as(chunk_start, static_offsets);
      }
      {
        /* Add last chunk. */
        const int64_t last_chunk_size = (global_last & index_mask_chunk_mask_low) + 1;
        chunks_.append_unchecked_as(last_chunk_start, static_offsets.take_front(last_chunk_size));
      }
    }
  }

  operator IndexMask2() const
  {
    return {chunks_, size_};
  }
};

}  // namespace blender
