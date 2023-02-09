/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_vector.hh"

namespace blender {

static constexpr int64_t index_mask_chunk_shift = 14;
static constexpr int64_t index_mask_chunk_lower_mask = (1 << index_mask_chunk_shift) - 1;
static constexpr int64_t max_index_mask_chunk_size = (1 << index_mask_chunk_shift);

class IndexMaskChunk {
 private:
  int64_t start_ = 0;
  Span<int16_t> offsets_;

 public:
  IndexMaskChunk() = default;

  IndexMaskChunk(const int64_t start, const Span<int16_t> offsets)
      : start_(start), offsets_(offsets)
  {
    BLI_assert(start & index_mask_chunk_lower_mask == 0);
    BLI_assert(IndexMaskChunk::offsets_are_valid(offsets));
  }

  int64_t size() const
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

  int64_t operator[](const int64_t i) const
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
      BLI_assert(chunks[i - 1].start < chunks[i].start());
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
    const IndexMask2 &mask_;

   public:
    Iterator(const IndexMask2 &mask, const int64_t chunk_index, const int64_t index_in_chunk)
        : current_chunk_(chunk_index < mask.chunks_.size() ? mask.chunks_[chunk_index] :
                                                             IndexMaskChunk()),
          index_in_chunk_(index_in_chunk),
          chunk_index_(chunk_index),
          mask_(mask)
    {
    }

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
      BLI_assert(&a.mask_ == &b.mask_);
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

}  // namespace blender
