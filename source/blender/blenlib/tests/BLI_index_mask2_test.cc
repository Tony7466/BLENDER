/* SPDX-License-Identifier: Apache-2.0 */

#include "BLI_array.hh"
#include "BLI_index_mask2.hh"
#include "BLI_strict_flags.h"
#include "BLI_timeit.hh"

#include "testing/testing.h"

namespace blender::index_mask::tests {

TEST(index_mask2, Test)
{

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
  unique_sorted_indices::split_to_ranges_and_spans<int>(data, 3, parts);

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
  ResourceScope scope;
  Array<int> data = {5, 100, 16383, 16384, 16385, 20000, 20001, 100000, 101000};
  IndexMask mask = unique_sorted_indices::to_index_mask<int>(data, scope);

  mask.foreach_segment([&](const int64_t mask_index_offset,
                           const int64_t index_offset,
                           const Span<int16_t> indices) {
    std::cout << mask_index_offset << ", " << index_offset << ", (";
    for (const int16_t i : indices) {
      std::cout << i << ", ";
    }
    std::cout << ")\n";
  });
}

}  // namespace blender::index_mask::tests
