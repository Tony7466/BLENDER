/* SPDX-License-Identifier: Apache-2.0 */

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

}  // namespace blender::index_mask::tests
