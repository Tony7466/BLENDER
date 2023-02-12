/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_utildefines.h"

namespace blender::bits {

/**
 * Using a large integer type is better because then it's easier to process many bits at once.
 */
using IntType = uint64_t;
static constexpr int64_t BitsPerInt = int64_t(sizeof(IntType) * 8);
static constexpr int64_t BitToIntIndexShift = 3 + (sizeof(IntType) >= 2) + (sizeof(IntType) >= 4) +
                                              (sizeof(IntType) >= 8);
static constexpr IntType BitIndexMask = (IntType(1) << BitToIntIndexShift) - 1;
static constexpr IntType MostSignificantBit = IntType(1) << (sizeof(IntType) * 8 - 1);

inline IntType mask_for_first_n_bits(const int64_t n)
{
  return (IntType(1) << n) - 1;
}

inline IntType mask_for_last_n_bits(const int64_t n)
{
  return ~((IntType(1) << (BitsPerInt - n)) - 1);
}

inline IntType mask_for_bit(const int64_t bit_index)
{
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
  const IntType *ptr_;
  /** All zeros except for a single one at the bit that is referenced. */
  IntType mask_;

  friend class MutableBitRef;

 public:
  BitRef() = default;

  /**
   * Reference a specific bit in an array. Note that #ptr does *not* have to point to the
   * exact integer the bit is in.
   */
  BitRef(const IntType *ptr, const int64_t bit_index)
  {
    ptr_ = int_containing_bit(ptr, bit_index);
    mask_ = mask_for_bit(bit_index & BitIndexMask);
  }

  /**
   * Return true when the bit is currently 1 and false otherwise.
   */
  bool test() const
  {
    const IntType value = *ptr_;
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
  IntType *ptr_;
  /** All zeros except for a single one at the bit that is referenced. */
  IntType mask_;

 public:
  MutableBitRef() = default;

  /**
   * Reference a specific bit in an array. Note that #ptr does *not* have to point to the
   * exact int the bit is in.
   */
  MutableBitRef(IntType *ptr, const int64_t bit_index)
  {
    ptr_ = int_containing_bit(ptr, bit_index);
    mask_ = mask_for_bit(bit_index & BitIndexMask);
  }

  /**
   * Support implicitly casting to a read-only #BitRef.
   */
  operator BitRef() const
  {
    BitRef bit_ref;
    bit_ref.ptr_ = ptr_;
    bit_ref.mask_ = mask_;
    return bit_ref;
  }

  /**
   * Return true when the bit is currently 1 and false otherwise.
   */
  bool test() const
  {
    const IntType value = *ptr_;
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
    *ptr_ |= mask_;
  }

  /**
   * Change the bit to a 0.
   */
  void reset()
  {
    *ptr_ &= ~mask_;
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

}  // namespace blender::bits

namespace blender {
using bits::BitRef;
using bits::MutableBitRef;
}  // namespace blender
