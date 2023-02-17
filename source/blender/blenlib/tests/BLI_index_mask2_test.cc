/* SPDX-License-Identifier: Apache-2.0 */

#include "BLI_array.hh"
#include "BLI_index_mask2.hh"
#include "BLI_strict_flags.h"
#include "BLI_timeit.hh"

#include "testing/testing.h"

namespace blender::index_mask {
void do_benchmark(const int64_t total);
}

namespace blender::index_mask::tests {

TEST(index_mask2, Test)
{
  // const int64_t total = 1e8;
  // do_benchmark(total);

  // Vector<int> data;
  // for (const int64_t i : IndexRange(100000)) {
  //   data.append(int(i));
  // }

  // for ([[maybe_unused]] const int64_t i : IndexRange(50)) {
  //   Vector<IndexRange> ranges = split_sorted_indices_by_chunk<int>(data);
  // }
  // ranges.as_span().print_as_lines("Ranges");

  // IndexMask mask(500000);
  // IndexMask sliced_mask = mask.slice(IndexRange(16000, 1000));
  // sliced_mask = sliced_mask.slice(IndexRange(100, 800));
  // sliced_mask.foreach_index_span([&](const int64_t offset, const Span<int16_t> indices) {
  //   for (const int64_t i : indices) {
  //     std::cout << (i + offset) << "\n";
  //   }
  // });
}

TEST(index_mask2, FindRangeEnd)
{
  EXPECT_EQ(unique_sorted_indices::find_size_of_next_range<int>({4}), 1);
  EXPECT_EQ(unique_sorted_indices::find_size_of_next_range<int>({4, 5, 6, 7}), 4);
  EXPECT_EQ(unique_sorted_indices::find_size_of_next_range<int>({4, 5, 6, 8, 9}), 3);
}

TEST(index_mask2, NonEmptyIsRange)
{
  EXPECT_TRUE(unique_sorted_indices::non_empty_is_range<int>({0, 1, 2}));
  EXPECT_TRUE(unique_sorted_indices::non_empty_is_range<int>({5}));
  EXPECT_TRUE(unique_sorted_indices::non_empty_is_range<int>({7, 8, 9, 10}));
  EXPECT_FALSE(unique_sorted_indices::non_empty_is_range<int>({3, 5}));
  EXPECT_FALSE(unique_sorted_indices::non_empty_is_range<int>({3, 4, 5, 6, 8, 9}));
}

TEST(index_mask2, NonEmptyAsRange)
{
  EXPECT_EQ(unique_sorted_indices::non_empty_as_range<int>({0, 1, 2}), IndexRange(0, 3));
  EXPECT_EQ(unique_sorted_indices::non_empty_as_range<int>({5}), IndexRange(5, 1));
  EXPECT_EQ(unique_sorted_indices::non_empty_as_range<int>({10, 11}), IndexRange(10, 2));
}

TEST(index_mask2, FindSizeOfNextRange)
{
  EXPECT_EQ(unique_sorted_indices::find_size_of_next_range<int>({0, 3, 4}), 1);
  EXPECT_EQ(unique_sorted_indices::find_size_of_next_range<int>({4, 5, 6, 7}), 4);
  EXPECT_EQ(unique_sorted_indices::find_size_of_next_range<int>({4}), 1);
  EXPECT_EQ(unique_sorted_indices::find_size_of_next_range<int>({5, 6, 7, 10, 11, 100}), 3);
}

TEST(index_mask2, FindStartOfNextRange)
{
  EXPECT_EQ(unique_sorted_indices::find_size_until_next_range<int>({4}, 3), 1);
  EXPECT_EQ(unique_sorted_indices::find_size_until_next_range<int>({4, 5}, 3), 2);
  EXPECT_EQ(unique_sorted_indices::find_size_until_next_range<int>({4, 5, 6}, 3), 0);
  EXPECT_EQ(unique_sorted_indices::find_size_until_next_range<int>({4, 5, 6, 7}, 3), 0);
  EXPECT_EQ(
      unique_sorted_indices::find_size_until_next_range<int>({0, 1, 3, 5, 10, 11, 12, 20}, 3), 4);
}

TEST(index_mask2, SplitToRangesAndSpans)
{
  Array<int> data = {1, 2, 3, 4, 7, 9, 10, 13, 14, 15, 20, 21, 22, 23, 24};
  Vector<unique_sorted_indices::RangeOrSpanVariant<int>> parts;
  const int64_t parts_num = unique_sorted_indices::split_to_ranges_and_spans<int>(data, 3, parts);

  EXPECT_EQ(parts_num, 4);
  EXPECT_EQ(parts.size(), 4);
  EXPECT_EQ(std::get<IndexRange>(parts[0]), IndexRange(1, 4));
  EXPECT_EQ(std::get<Span<int>>(parts[1]), Span<int>({7, 9, 10}));
  EXPECT_EQ(std::get<IndexRange>(parts[2]), IndexRange(13, 3));
  EXPECT_EQ(std::get<IndexRange>(parts[3]), IndexRange(20, 5));
}

TEST(index_mask2, SplitByChunk)
{
  Array<int> data = {5, 100, 16383, 16384, 16385, 20000, 20001, 100000, 101000};
  Vector<IndexRange> ranges = unique_sorted_indices::split_by_chunk<int>(data);
  EXPECT_EQ(ranges.size(), 3);
  EXPECT_EQ(data.as_span().slice(ranges[0]), Span<int>({5, 100, 16383}));
  EXPECT_EQ(data.as_span().slice(ranges[1]), Span<int>({16384, 16385, 20000, 20001}));
  EXPECT_EQ(data.as_span().slice(ranges[2]), Span<int>({100000, 101000}));
}

TEST(index_mask2, IndicesToMask)
{
  LinearAllocator<> allocator;
  Array<int> data = {
      5, 100, 16383, 16384, 16385, 20000, 20001, 50000, 50001, 50002, 100000, 101000};
  IndexMask mask = unique_sorted_indices::to_index_mask<int>(data, allocator);

  EXPECT_EQ(mask.first(), 5);
  EXPECT_EQ(mask.last(), 101000);
  EXPECT_EQ(mask.min_array_size(), 101001);
}

TEST(index_mask2, FromBits)
{
  LinearAllocator<> allocator;
  const uint64_t bits =
      0b0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'1111'0010'0000;
  const IndexMask mask = bits_to_index_mask(BitSpan(&bits, IndexRange(2, 40)), 100, allocator);
  Array<int> indices(5);
  unique_sorted_indices::from_index_mask<int>(mask, indices);
  EXPECT_EQ(indices[0], 103);
  EXPECT_EQ(indices[1], 106);
  EXPECT_EQ(indices[2], 107);
  EXPECT_EQ(indices[3], 108);
  EXPECT_EQ(indices[4], 109);

  uint64_t new_bits = 0;
  index_mask_to_bits(mask, 100, MutableBitSpan(&new_bits, IndexRange(5, 40)));
  EXPECT_EQ(new_bits, bits << 3);
}

TEST(index_mask2, FromSize)
{
  {
    IndexMask mask(5);
    Vector<IndexMaskSegment> segments;
    mask.foreach_segment([&](const IndexMaskSegment &segment) { segments.append(segment); });
    EXPECT_EQ(segments.size(), 1);
    EXPECT_EQ(segments[0].indices.size(), 5);
    EXPECT_EQ(mask.first(), 0);
    EXPECT_EQ(mask.last(), 4);
    EXPECT_EQ(mask.min_array_size(), 5);
  }
  {
    IndexMask mask(chunk_capacity);
    Vector<IndexMaskSegment> segments;
    mask.foreach_segment([&](const IndexMaskSegment &segment) { segments.append(segment); });
    EXPECT_EQ(segments.size(), 1);
    EXPECT_EQ(segments[0].indices.size(), chunk_capacity);
    EXPECT_EQ(mask.first(), 0);
    EXPECT_EQ(mask.last(), chunk_capacity - 1);
    EXPECT_EQ(mask.min_array_size(), chunk_capacity);
  }
}

TEST(index_mask2, DefaultConstructor)
{
  IndexMask mask;
  EXPECT_EQ(mask.size(), 0);
  EXPECT_EQ(mask.min_array_size(), 0);
}

}  // namespace blender::index_mask::tests
