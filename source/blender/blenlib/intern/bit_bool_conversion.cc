/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_bit_bool_conversion.hh"
#include "BLI_simd.hh"
#include "BLI_timeit.hh"

namespace blender::bits {

void bools_to_bits(const Span<bool> bools, MutableBitSpan r_bits)
{
  SCOPED_TIMER(__func__);
  BLI_assert(r_bits.size() >= bools.size());
  if (bools.is_empty()) {
    return;
  }

  int64_t bool_i = 0;
  const bool *bools_ = bools.data();

#if BLI_HAVE_SSE2
  const __m128i zero_bytes = _mm_set1_epi8(0);
  for (; bool_i + 16 < bools.size(); bool_i += 16) {
    const __m128i group = _mm_loadu_si128(reinterpret_cast<const __m128i_u *>(bools_ + bool_i));
    const __m128i is_false_byte_mask = _mm_cmpeq_epi8(group, zero_bytes);
    const uint16_t is_false_mask = _mm_movemask_epi8(is_false_byte_mask);
    const uint16_t is_true_mask = ~is_false_mask;
    const int start_bit_in_int = (r_bits.bit_range().start() + bool_i) & BitIndexMask;
    BitInt *start_bit_int = int_containing_bit(r_bits.data(), r_bits.bit_range().start() + bool_i);
    *start_bit_int |= BitInt(is_true_mask) << start_bit_in_int;
    if (start_bit_in_int > BitsPerInt - 16) {
      start_bit_int[1] |= BitInt(is_true_mask) >> (64 - start_bit_in_int);
    }
  }
#endif

  for (; bool_i < bools.size(); bool_i++) {
    r_bits[bool_i].set(bools_[bool_i]);
  }
}

}  // namespace blender::bits
