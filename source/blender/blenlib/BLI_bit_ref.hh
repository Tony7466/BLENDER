/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_index_range.hh"
#include "BLI_utildefines.h"

#include <ostream>

namespace blender::bits {

/** Using a large integer type is better because then it's easier to process many bits at once. */
using IntType = uint64_t;
/** Number of bits that fit into #IntType. */
static constexpr int64_t BitsPerInt = int64_t(sizeof(IntType) * 8);
/** Shift amount to get from a bit index to an int index. Equivalent to `log(BitsPerInt, 2)`. */
static constexpr int64_t BitToIntIndexShift = 3 + (sizeof(IntType) >= 2) + (sizeof(IntType) >= 4) +
                                              (sizeof(IntType) >= 8);
/** Bit mask containing a 1 for the last few bits that index a bit inside of an #IntType. */
static constexpr IntType BitIndexMask = (IntType(1) << BitToIntIndexShift) - 1;

inline IntType mask_first_n_bits(const int64_t n)
{
  BLI_assert(n >= 0);
  BLI_assert(n <= BitsPerInt);
  if (n == BitsPerInt) {
    return IntType(-1);
  }
  return (IntType(1) << n) - 1;
}

inline IntType mask_last_n_bits(const int64_t n)
{
  return ~mask_first_n_bits(BitsPerInt - n);
}

inline IntType mask_range_bits(const IndexRange range)
{
  const int64_t size = range.size();
  const int64_t start = range.start() & BitIndexMask;
  const int64_t end = start + size;
  BLI_assert(end <= BitsPerInt);
  if (end == BitsPerInt) {
    return mask_last_n_bits(size);
  }
  return ((IntType(1) << end) - 1) & ~((IntType(1) << start) - 1);
}

inline IntType mask_single_bit(const int64_t bit_index)
{
  BLI_assert(bit_index >= 0);
  BLI_assert(bit_index < BitsPerInt);
  return IntType(1) << bit_index;
}

inline IntType *int_containing_bit(IntType *data, const int64_t bit_index)
{
  return data + (bit_index >> BitToIntIndexShift);
}

inline const IntType *int_containing_bit(const IntType *data, const int64_t bit_index)
{
  return data + (bit_index >> BitToIntIndexShift);
}

/**
 * This is a read-only pointer to a specific bit. The value of the bit can be retrieved, but
 * not changed.
 */
class BitRef {
 private:
  /** Points to the integer that the bit is in. */
  const IntType *data_;
  /** All zeros except for a single one at the bit that is referenced. */
  IntType mask_;

  friend class MutableBitRef;

 public:
  BitRef() = default;

  /**
   * Reference a specific bit in an array. Note that #data does *not* have to point to the
   * exact integer the bit is in.
   */
  BitRef(const IntType *data, const int64_t bit_index)
  {
    data_ = int_containing_bit(data, bit_index);
    mask_ = mask_single_bit(bit_index & BitIndexMask);
  }

  /**
   * Return true when the bit is currently 1 and false otherwise.
   */
  bool test() const
  {
    const IntType value = *data_;
    const IntType masked_value = value & mask_;
    return masked_value != 0;
  }

  operator bool() const
  {
    return this->test();
  }
};

/**
 * Similar to #BitRef, but also allows changing the referenced bit.
 */
class MutableBitRef {
 private:
  /** Points to the integer that the bit is in. */
  IntType *data_;
  /** All zeros except for a single one at the bit that is referenced. */
  IntType mask_;

 public:
  MutableBitRef() = default;

  /**
   * Reference a specific bit in an array. Note that #data does *not* have to point to the
   * exact int the bit is in.
   */
  MutableBitRef(IntType *data, const int64_t bit_index)
  {
    data_ = int_containing_bit(data, bit_index);
    mask_ = mask_single_bit(bit_index & BitIndexMask);
  }

  /**
   * Support implicitly casting to a read-only #BitRef.
   */
  operator BitRef() const
  {
    BitRef bit_ref;
    bit_ref.data_ = data_;
    bit_ref.mask_ = mask_;
    return bit_ref;
  }

  /**
   * Return true when the bit is currently 1 and false otherwise.
   */
  bool test() const
  {
    const IntType value = *data_;
    const IntType masked_value = value & mask_;
    return masked_value != 0;
  }

  operator bool() const
  {
    return this->test();
  }

  /**
   * Change the bit to a 1.
   */
  void set()
  {
    *data_ |= mask_;
  }

  /**
   * Change the bit to a 0.
   */
  void reset()
  {
    *data_ &= ~mask_;
  }

  /**
   * Change the bit to a 1 if #value is true and 0 otherwise.
   */
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

inline std::ostream &operator<<(std::ostream &stream, const BitRef &bit)
{
  return stream << (bit ? "1" : "0");
}

inline std::ostream &operator<<(std::ostream &stream, const MutableBitRef &bit)
{
  return stream << BitRef(bit);
}

}  // namespace blender::bits

namespace blender {
using bits::BitRef;
using bits::MutableBitRef;
}  // namespace blender
