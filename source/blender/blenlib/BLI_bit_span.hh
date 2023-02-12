/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_bit_ref.hh"
#include "BLI_index_range.hh"
#include "BLI_memory_utils.hh"

namespace blender::bits {

class BitIterator {
 private:
  const IntType *data_;
  int64_t bit_index_;

 public:
  BitIterator(const IntType *data, const int64_t bit_index) : data_(data), bit_index_(bit_index)
  {
  }

  BitIterator &operator++()
  {
    bit_index_++;
    return *this;
  }

  BitRef operator*() const
  {
    return BitRef(data_, bit_index_);
  }

  friend bool operator!=(const BitIterator &a, const BitIterator &b)
  {
    BLI_assert(a.data_ == b.data_);
    return a.bit_index_ != b.bit_index_;
  }
};

class MutableBitIterator {
 private:
  IntType *data_;
  int64_t bit_index_;

 public:
  MutableBitIterator(IntType *data, const int64_t bit_index) : data_(data), bit_index_(bit_index)
  {
  }

  MutableBitIterator &operator++()
  {
    bit_index_++;
    return *this;
  }

  MutableBitRef operator*() const
  {
    return MutableBitRef(data_, bit_index_);
  }

  friend bool operator!=(const MutableBitIterator &a, const MutableBitIterator &b)
  {
    BLI_assert(a.data_ == b.data_);
    return a.bit_index_ != b.bit_index_;
  }
};

class BitSpan {
 private:
  const IntType *data_ = nullptr;
  IndexRange bit_range_ = {0, 0};

 public:
  BitSpan() = default;

  BitSpan(const IntType *data, const int64_t size) : data_(data), bit_range_(size)
  {
  }

  BitSpan(const IntType *data, const IndexRange bit_range) : data_(data), bit_range_(bit_range)
  {
  }

  int64_t size() const
  {
    return bit_range_.size();
  }

  bool is_empty() const
  {
    return bit_range_.is_empty();
  }

  BitRef operator[](const int64_t index) const
  {
    BLI_assert(index >= 0);
    BLI_assert(index < bit_range_.size());
    return {data_, bit_range_.start() + index};
  }

  BitSpan slice(const IndexRange range) const
  {
    return {data_, bit_range_.slice(range)};
  }

  const IntType *data() const
  {
    return data_;
  }

  const IndexRange &bit_range() const
  {
    return bit_range_;
  }

  BitIterator begin() const
  {
    return {data_, bit_range_.start()};
  }

  BitIterator end() const
  {
    return {data_, bit_range_.one_after_last()};
  }
};

class MutableBitSpan {
 private:
  IntType *data_ = nullptr;
  IndexRange bit_range_ = {0, 0};

 public:
  MutableBitSpan() = default;

  MutableBitSpan(IntType *data, const int64_t size) : data_(data), bit_range_(size)
  {
  }

  MutableBitSpan(IntType *data, const IndexRange bit_range) : data_(data), bit_range_(bit_range)
  {
  }

  int64_t size() const
  {
    return bit_range_.size();
  }

  bool is_empty() const
  {
    return bit_range_.is_empty();
  }

  MutableBitRef operator[](const int64_t index) const
  {
    BLI_assert(index >= 0);
    BLI_assert(index < bit_range_.size());
    return {data_, bit_range_.start() + index};
  }

  MutableBitSpan slice(const IndexRange range) const
  {
    return {data_, bit_range_.slice(range)};
  }

  IntType *data() const
  {
    return data_;
  }

  const IndexRange &bit_range() const
  {
    return bit_range_;
  }

  MutableBitIterator begin() const
  {
    return {data_, bit_range_.start()};
  }

  MutableBitIterator end() const
  {
    return {data_, bit_range_.one_after_last()};
  }

  operator BitSpan() const
  {
    return {data_, bit_range_};
  }

  void set()
  {
    const AlignedIndexRanges ranges = split_index_range_by_alignment(bit_range_, BitsPerInt);
    {
      IntType &first_int = *int_containing_bit(data_, bit_range_.start());
      const IntType first_int_mask = mask_last_n_bits(ranges.prefix.size());
      first_int |= first_int_mask;
    }
    {
      IntType *start = int_containing_bit(data_, ranges.aligned.start());
      const int64_t ints_to_fill = ranges.aligned.size() / BitsPerInt;
      constexpr IntType fill_value = IntType(-1);
      initialized_fill_n(start, ints_to_fill, fill_value);
    }
    {
      IntType &last_int = *int_containing_bit(data_, bit_range_.one_after_last() - 1);
      const IntType last_int_mask = mask_first_n_bits(ranges.suffix.size());
      last_int |= last_int_mask;
    }
  }

  void reset()
  {
    const AlignedIndexRanges ranges = split_index_range_by_alignment(bit_range_, BitsPerInt);
    {
      IntType &first_int = *int_containing_bit(data_, bit_range_.start());
      const IntType first_int_mask = mask_first_n_bits(ranges.prefix.size());
      first_int &= first_int_mask;
    }
    {
      IntType *start = int_containing_bit(data_, ranges.aligned.start());
      const int64_t ints_to_fill = ranges.aligned.size() / BitsPerInt;
      constexpr IntType fill_value = 0;
      initialized_fill_n(start, ints_to_fill, fill_value);
    }
    {
      IntType &last_int = *int_containing_bit(data_, bit_range_.one_after_last() - 1);
      const IntType last_int_mask = mask_last_n_bits(ranges.suffix.size());
      last_int &= last_int_mask;
    }
  }

  void set(const bool value)
  {
    if (value) {
      this->set();
    }
    else {
      this->reset();
    }
  }
};

}  // namespace blender::bits
