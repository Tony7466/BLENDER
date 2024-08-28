/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_bit_bool_conversion.hh"
#include "BLI_simd.hh"
#include "BLI_timeit.hh"

namespace blender::bits {

void bools_to_zeroed_bits(const Span<bool> bools, MutableBitSpan r_bits)
{
  BLI_assert(r_bits.size() >= bools.size());
  if (bools.is_empty()) {
    return;
  }

#ifndef NDEBUG
  /* Assert that all bits are zero. This is often already the case and simplifies the code below.
   * So it's good if the caller is responsible for zeroing the bits. */
  for (const int64_t i : bools.index_range()) {
    BLI_assert(!r_bits[i]);
  }
#endif

  int64_t bool_i = 0;
  const bool *bools_ = bools.data();

/* Conversion from bools to bits can be way faster with intrinsics. That's because instead of
 * processing one element at a time, we can process 16 at once. */
#if BLI_HAVE_SSE2
  /* Initialize zeros, so that we can compare against it. */
  const __m128i zero_bytes = _mm_set1_epi8(0);
  /* Iterate over chunks of booleans. */
  for (; bool_i + 16 < bools.size(); bool_i += 16) {
    /* Load 16 bools at once. */
    const __m128i group = _mm_loadu_si128(reinterpret_cast<const __m128i_u *>(bools_ + bool_i));
    /* Compare them all against zero. The result is a mask of the form [0x00, 0xff, 0xff, ...]. */
    const __m128i is_false_byte_mask = _mm_cmpeq_epi8(group, zero_bytes);
    /* Compress the byte-mask into a bit mask. This takes one bit from each byte. */
    const uint16_t is_false_mask = _mm_movemask_epi8(is_false_byte_mask);
    /* Now we have a bit mask where each bit corresponds to an input boolean. */
    const uint16_t is_true_mask = ~is_false_mask;

    const int start_bit_in_int = (r_bits.bit_range().start() + bool_i) & BitIndexMask;
    BitInt *start_bit_int = int_containing_bit(r_bits.data(), r_bits.bit_range().start() + bool_i);
    *start_bit_int |= BitInt(is_true_mask) << start_bit_in_int;

    if (start_bit_in_int > BitsPerInt - 16) {
      /* It's possible that the bits need inserted in two consecutive integers. */
      start_bit_int[1] |= BitInt(is_true_mask) >> (64 - start_bit_in_int);
    }
  }
#endif

  /* Process remaining bools. */
  for (; bool_i < bools.size(); bool_i++) {
    if (bools_[bool_i]) {
      r_bits[bool_i].set();
    }
  }
}

}  // namespace blender::bits
