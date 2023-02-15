/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_bit_ref.hh"
#include "BLI_index_range.hh"
#include "BLI_memory_utils.hh"

namespace blender::bits {

class BitIteratorBase {
 protected:
  const BitInt *data_;
  int64_t bit_index_;

 public:
  BitIteratorBase(const BitInt *data, const int64_t bit_index) : data_(data), bit_index_(bit_index)
  {
  }

  BitIteratorBase &operator++()
  {
    bit_index_++;
    return *this;
  }

  friend bool operator!=(const BitIteratorBase &a, const BitIteratorBase &b)
  {
    BLI_assert(a.data_ == b.data_);
    return a.bit_index_ != b.bit_index_;
  }
};

class BitIterator : public BitIteratorBase {
 public:
  BitIterator(const BitInt *data, const int64_t bit_index) : BitIteratorBase(data, bit_index)
  {
  }

  BitRef operator*() const
  {
    return BitRef(data_, bit_index_);
  }
};

class MutableBitIterator : public BitIteratorBase {
 public:
  MutableBitIterator(BitInt *data, const int64_t bit_index) : BitIteratorBase(data, bit_index)
  {
  }

  MutableBitRef operator*() const
  {
    return MutableBitRef(const_cast<BitInt *>(data_), bit_index_);
  }
};

class BitSpan {
 private:
  const BitInt *data_ = nullptr;
  IndexRange bit_range_ = {0, 0};

 public:
  BitSpan() = default;

  BitSpan(const BitInt *data, const int64_t size) : data_(data), bit_range_(size)
  {
  }

  BitSpan(const BitInt *data, const IndexRange bit_range) : data_(data), bit_range_(bit_range)
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

  IndexRange index_range() const
  {
    return IndexRange(bit_range_.size());
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

  const BitInt *data() const
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
  BitInt *data_ = nullptr;
  IndexRange bit_range_ = {0, 0};

 public:
  MutableBitSpan() = default;

  MutableBitSpan(BitInt *data, const int64_t size) : data_(data), bit_range_(size)
  {
  }

  MutableBitSpan(BitInt *data, const IndexRange bit_range) : data_(data), bit_range_(bit_range)
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

  IndexRange index_range() const
  {
    return IndexRange(bit_range_.size());
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

  BitInt *data() const
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
      BitInt &first_int = *int_containing_bit(data_, bit_range_.start());
      const BitInt first_int_mask = mask_range_bits(ranges.prefix.start() & BitIndexMask,
                                                    ranges.prefix.size());
      first_int |= first_int_mask;
    }
    {
      BitInt *start = int_containing_bit(data_, ranges.aligned.start());
      const int64_t ints_to_fill = ranges.aligned.size() / BitsPerInt;
      constexpr BitInt fill_value = BitInt(-1);
      initialized_fill_n(start, ints_to_fill, fill_value);
    }
    {
      BitInt &last_int = *int_containing_bit(data_, bit_range_.one_after_last() - 1);
      const BitInt last_int_mask = mask_first_n_bits(ranges.suffix.size());
      last_int |= last_int_mask;
    }
  }

  void reset()
  {
    const AlignedIndexRanges ranges = split_index_range_by_alignment(bit_range_, BitsPerInt);
    {
      BitInt &first_int = *int_containing_bit(data_, bit_range_.start());
      const BitInt first_int_mask = mask_range_bits(ranges.prefix.start() & BitIndexMask,
                                                    ranges.prefix.size());
      first_int &= ~first_int_mask;
    }
    {
      BitInt *start = int_containing_bit(data_, ranges.aligned.start());
      const int64_t ints_to_fill = ranges.aligned.size() / BitsPerInt;
      constexpr BitInt fill_value = 0;
      initialized_fill_n(start, ints_to_fill, fill_value);
    }
    {
      BitInt &last_int = *int_containing_bit(data_, bit_range_.one_after_last() - 1);
      const BitInt last_int_mask = mask_first_n_bits(ranges.suffix.size());
      last_int &= ~last_int_mask;
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

inline std::ostream &operator<<(std::ostream &stream, const BitSpan &span)
{
  stream << "(Size: " << span.size() << ", ";
  for (const BitRef bit : span) {
    stream << bit;
  }
  stream << ")";
  return stream;
}

inline std::ostream &operator<<(std::ostream &stream, const MutableBitSpan &span)
{
  return stream << BitSpan(span);
}

}  // namespace blender::bits

namespace blender {
using bits::BitSpan;
using bits::MutableBitSpan;
}  // namespace blender
