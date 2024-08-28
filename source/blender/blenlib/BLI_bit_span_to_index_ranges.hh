/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <numeric>

#include "BLI_bit_span.hh"
#include "BLI_index_ranges_builder.hh"
#include "BLI_math_bits.h"
#include "BLI_simd.hh"

namespace blender::bits {

template<typename IntT>
inline void bits_to_index_ranges(const BitSpan bits, IndexRangesBuilder<int16_t> &builder)
{
  if (bits.is_empty()) {
    return;
  }

  /* -1 because we also need to store the end of the last range. */
  constexpr int64_t max_index = std::numeric_limits<IntT>::max() - 1;
  UNUSED_VARS_NDEBUG(max_index);

  auto append_range = [&](const IndexRange range) {
    BLI_assert(range.last() <= max_index);
    builder.add_range(IntT(range.start()), IntT(range.one_after_last()));
  };

  auto append_index = [&](const int64_t index) {
    BLI_assert(index >= 0);
    BLI_assert(index <= max_index);
    builder.add(IntT(index));
  };

  auto process_bit_int = [&](const BitInt value,
                             const int64_t start_bit,
                             const int64_t bits_num,
                             const int64_t start) {
    const BitInt mask = mask_range_bits(start_bit, bits_num);
    const BitInt masked_value = mask & value;
    if (masked_value == 0) {
      /* Do nothing. */
      return;
    }
    if (masked_value == mask) {
      append_range(IndexRange::from_begin_size(start, bits_num));
      return;
    }
    const int64_t bit_i_to_output_offset = start - start_bit;
    const int bit_count = count_bits_uint64(masked_value);
    switch (bit_count) {
      case 1: {
        const int64_t set_bit_i = int64_t(bitscan_forward_uint64(masked_value));
        append_index(set_bit_i + bit_i_to_output_offset);
        return;
      }
      case 2: {
        const int64_t first_set_bit_i = int64_t(bitscan_forward_uint64(masked_value));
        const int64_t second_set_bit_i = BitsPerInt - 1 -
                                         int64_t(bitscan_reverse_uint64(masked_value));
        append_index(first_set_bit_i + bit_i_to_output_offset);
        append_index(second_set_bit_i + bit_i_to_output_offset);
        return;
      }
      default: {
        BitInt current_value = masked_value;
        while (current_value != 0) {
          const int64_t first_set_bit_i = int64_t(bitscan_forward_uint64(current_value));
          const BitInt find_unset_value = ~(current_value | mask_first_n_bits(first_set_bit_i) |
                                            ~mask);
          if (find_unset_value == 0) {
            const IndexRange range = IndexRange::from_begin_end(first_set_bit_i,
                                                                start_bit + bits_num);
            append_range(range.shift(bit_i_to_output_offset));
            break;
          }
          const int64_t next_unset_bit_i = int64_t(bitscan_forward_uint64(find_unset_value));
          const IndexRange range = IndexRange::from_begin_end(first_set_bit_i, next_unset_bit_i);
          append_range(range.shift(bit_i_to_output_offset));
          current_value &= ~mask_first_n_bits(next_unset_bit_i);
        }
        return;
      }
      case 63: {
        const int64_t unset_bit_i = bitscan_forward_uint64(~masked_value);
        const IndexRange before = IndexRange::from_begin_end(start_bit, unset_bit_i);
        const IndexRange after = IndexRange::from_begin_end(unset_bit_i + 1, start_bit + bits_num);
        if (!before.is_empty()) {
          append_range(before.shift(bit_i_to_output_offset));
        }
        if (!after.is_empty()) {
          append_range(after.shift(bit_i_to_output_offset));
        }
        return;
      }
    }
  };

  const BitInt *data = bits.data();
  const IndexRange bit_range = bits.bit_range();

  const AlignedIndexRanges ranges = split_index_range_by_alignment(bit_range, bits::BitsPerInt);
  if (!ranges.prefix.is_empty()) {
    const BitInt first_int = *int_containing_bit(data, bit_range.start());
    process_bit_int(
        first_int, BitInt(ranges.prefix.start()) & BitIndexMask, ranges.prefix.size(), 0);
  }
  if (!ranges.aligned.is_empty()) {
    const BitInt *start = int_containing_bit(data, ranges.aligned.start());
    const int64_t ints_to_check = ranges.aligned.size() / BitsPerInt;
    int64_t int_i = 0;
#if BLI_HAVE_SSE2
    __m128i_u all_ones = _mm_set1_epi8((char)0xFF);
    for (; int_i + 1 < ints_to_check; int_i += 2) {
      const __m128i_u group = _mm_loadu_si128(reinterpret_cast<const __m128i_u *>(start + int_i));
      const bool group_is_zero = _mm_testz_si128(group, group);
      if (group_is_zero) {
        continue;
      }
      const __m128i_u inverted_group = _mm_xor_si128(group, all_ones);
      const bool group_is_all_one = _mm_testz_si128(inverted_group, inverted_group);
      if (group_is_all_one) {
        append_range(IndexRange::from_begin_size(ranges.prefix.size() + int_i * BitsPerInt,
                                                 2 * BitsPerInt));
        continue;
      }
      for (int j = 0; j < 2; j++) {
        process_bit_int(
            start[int_i + j], 0, BitsPerInt, ranges.prefix.size() + (int_i + j) * BitsPerInt);
      }
    }
#endif
    for (; int_i < ints_to_check; int_i++) {
      process_bit_int(start[int_i], 0, BitsPerInt, ranges.prefix.size() + int_i * BitsPerInt);
    }
  }
  if (!ranges.suffix.is_empty()) {
    const BitInt last_int = *int_containing_bit(data, bit_range.last());
    process_bit_int(
        last_int, 0, ranges.suffix.size(), ranges.prefix.size() + ranges.aligned.size());
  }
}

}  // namespace blender::bits
