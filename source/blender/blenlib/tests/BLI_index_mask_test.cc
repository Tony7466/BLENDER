/* SPDX-License-Identifier: Apache-2.0 */

#include "BLI_array.hh"
#include "BLI_index_mask.hh"
#include "BLI_strict_flags.h"
#include "BLI_timeit.hh"

#include "testing/testing.h"

namespace blender::index_mask {
void do_benchmark(const int64_t total);
}

namespace blender::index_mask::tests {

TEST(index_mask, FindRangeEnd)
{
  EXPECT_EQ(unique_sorted_indices::find_size_of_next_range<int>({4}), 1);
  EXPECT_EQ(unique_sorted_indices::find_size_of_next_range<int>({4, 5, 6, 7}), 4);
  EXPECT_EQ(unique_sorted_indices::find_size_of_next_range<int>({4, 5, 6, 8, 9}), 3);
}

TEST(index_mask, NonEmptyIsRange)
{
  EXPECT_TRUE(unique_sorted_indices::non_empty_is_range<int>({0, 1, 2}));
  EXPECT_TRUE(unique_sorted_indices::non_empty_is_range<int>({5}));
  EXPECT_TRUE(unique_sorted_indices::non_empty_is_range<int>({7, 8, 9, 10}));
  EXPECT_FALSE(unique_sorted_indices::non_empty_is_range<int>({3, 5}));
  EXPECT_FALSE(unique_sorted_indices::non_empty_is_range<int>({3, 4, 5, 6, 8, 9}));
}

TEST(index_mask, NonEmptyAsRange)
{
  EXPECT_EQ(unique_sorted_indices::non_empty_as_range<int>({0, 1, 2}), IndexRange(0, 3));
  EXPECT_EQ(unique_sorted_indices::non_empty_as_range<int>({5}), IndexRange(5, 1));
  EXPECT_EQ(unique_sorted_indices::non_empty_as_range<int>({10, 11}), IndexRange(10, 2));
}

TEST(index_mask, FindSizeOfNextRange)
{
  EXPECT_EQ(unique_sorted_indices::find_size_of_next_range<int>({0, 3, 4}), 1);
  EXPECT_EQ(unique_sorted_indices::find_size_of_next_range<int>({4, 5, 6, 7}), 4);
  EXPECT_EQ(unique_sorted_indices::find_size_of_next_range<int>({4}), 1);
  EXPECT_EQ(unique_sorted_indices::find_size_of_next_range<int>({5, 6, 7, 10, 11, 100}), 3);
}

TEST(index_mask, FindStartOfNextRange)
{
  EXPECT_EQ(unique_sorted_indices::find_size_until_next_range<int>({4}, 3), 1);
  EXPECT_EQ(unique_sorted_indices::find_size_until_next_range<int>({4, 5}, 3), 2);
  EXPECT_EQ(unique_sorted_indices::find_size_until_next_range<int>({4, 5, 6}, 3), 0);
  EXPECT_EQ(unique_sorted_indices::find_size_until_next_range<int>({4, 5, 6, 7}, 3), 0);
  EXPECT_EQ(
      unique_sorted_indices::find_size_until_next_range<int>({0, 1, 3, 5, 10, 11, 12, 20}, 3), 4);
}

TEST(index_mask, SplitToRangesAndSpans)
{
  Array<int> data = {1, 2, 3, 4, 7, 9, 10, 13, 14, 15, 20, 21, 22, 23, 24};
  Vector<std::variant<IndexRange, Span<int>>> parts;
  const int64_t parts_num = unique_sorted_indices::split_to_ranges_and_spans<int>(data, 3, parts);

  EXPECT_EQ(parts_num, 4);
  EXPECT_EQ(parts.size(), 4);
  EXPECT_EQ(std::get<IndexRange>(parts[0]), IndexRange(1, 4));
  EXPECT_EQ(std::get<Span<int>>(parts[1]), Span<int>({7, 9, 10}));
  EXPECT_EQ(std::get<IndexRange>(parts[2]), IndexRange(13, 3));
  EXPECT_EQ(std::get<IndexRange>(parts[3]), IndexRange(20, 5));
}

TEST(index_mask, SplitByChunk)
{
  Array<int> data = {5, 100, 16383, 16384, 16385, 20000, 20001, 100000, 101000};
  Vector<IndexRange> ranges = unique_sorted_indices::split_by_chunk<int>(data);
  EXPECT_EQ(ranges.size(), 3);
  EXPECT_EQ(data.as_span().slice(ranges[0]), Span<int>({5, 100, 16383}));
  EXPECT_EQ(data.as_span().slice(ranges[1]), Span<int>({16384, 16385, 20000, 20001}));
  EXPECT_EQ(data.as_span().slice(ranges[2]), Span<int>({100000, 101000}));
}

TEST(index_mask, IndicesToMask)
{
  IndexMaskMemory memory;
  Array<int> data = {
      5, 100, 16383, 16384, 16385, 20000, 20001, 50000, 50001, 50002, 100000, 101000};
  IndexMask mask = IndexMask::from_indices<int>(data, memory);

  EXPECT_EQ(mask.first(), 5);
  EXPECT_EQ(mask.last(), 101000);
  EXPECT_EQ(mask.min_array_size(), 101001);
}

TEST(index_mask, FromBits)
{
  IndexMaskMemory memory;
  const uint64_t bits =
      0b0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'0000'1111'0010'0000;
  const IndexMask mask = IndexMask::from_bits(BitSpan(&bits, IndexRange(2, 40)), memory, 100);
  Array<int> indices(5);
  mask.to_indices<int>(indices);
  EXPECT_EQ(indices[0], 103);
  EXPECT_EQ(indices[1], 106);
  EXPECT_EQ(indices[2], 107);
  EXPECT_EQ(indices[3], 108);
  EXPECT_EQ(indices[4], 109);

  uint64_t new_bits = 0;
  mask.to_bits(MutableBitSpan(&new_bits, IndexRange(5, 40)), 100);
  EXPECT_EQ(new_bits, bits << 3);
}

TEST(index_mask, FromSize)
{
  {
    const IndexMask &mask(5);
    Vector<OffsetSpan<int64_t, int16_t>> segments;
    mask.foreach_span(
        [&](const OffsetSpan<int64_t, int16_t> segment) { segments.append(segment); });
    EXPECT_EQ(segments.size(), 1);
    EXPECT_EQ(segments[0].size(), 5);
    EXPECT_EQ(mask.first(), 0);
    EXPECT_EQ(mask.last(), 4);
    EXPECT_EQ(mask.min_array_size(), 5);
  }
  {
    const IndexMask &mask(chunk_capacity);
    Vector<OffsetSpan<int64_t, int16_t>> segments;
    mask.foreach_span(
        [&](const OffsetSpan<int64_t, int16_t> segment) { segments.append(segment); });
    EXPECT_EQ(segments.size(), 1);
    EXPECT_EQ(segments[0].size(), chunk_capacity);
    EXPECT_EQ(mask.first(), 0);
    EXPECT_EQ(mask.last(), chunk_capacity - 1);
    EXPECT_EQ(mask.min_array_size(), chunk_capacity);
  }
}

TEST(index_mask, DefaultConstructor)
{
  IndexMask mask;
  EXPECT_EQ(mask.size(), 0);
  EXPECT_EQ(mask.min_array_size(), 0);
}

TEST(index_mask, IndicesToRanges)
{
  IndexMaskMemory memory;
  const IndexMask mask = IndexMask::from_indices<int>({0, 1, 5}, memory);
  const IndexMask new_mask = grow_indices_to_ranges(
      mask, [&](const int64_t i) { return IndexRange(i * 10, 3); }, memory);
  Vector<int64_t> indices(new_mask.size());
  new_mask.to_indices<int64_t>(indices);
  EXPECT_EQ(indices.size(), 9);
  EXPECT_EQ(indices[0], 0);
  EXPECT_EQ(indices[1], 1);
  EXPECT_EQ(indices[2], 2);
  EXPECT_EQ(indices[3], 10);
  EXPECT_EQ(indices[4], 11);
  EXPECT_EQ(indices[5], 12);
  EXPECT_EQ(indices[6], 50);
  EXPECT_EQ(indices[7], 51);
  EXPECT_EQ(indices[8], 52);
}

TEST(index_mask, ForeachRange)
{
  IndexMaskMemory memory;
  const IndexMask mask = IndexMask::from_indices<int>({2, 3, 4, 10, 40, 41}, memory);
  Vector<IndexRange> ranges;
  mask.foreach_range([&](const IndexRange range) { ranges.append(range); });

  EXPECT_EQ(ranges.size(), 3);
  EXPECT_EQ(ranges[0], IndexRange(2, 3));
  EXPECT_EQ(ranges[1], IndexRange(10, 1));
  EXPECT_EQ(ranges[2], IndexRange(40, 2));
}

TEST(index_mask, Expr)
{
  IndexMaskMemory memory;

  const IndexMask &mask1(IndexRange(10, 5));
  const IndexMask &mask2(IndexRange(40, 5));
  const IndexMask &mask3 = IndexMask::from_indices<int>({12, 13, 20, 21, 22}, memory);

  const AtomicExpr expr1{mask1};
  const AtomicExpr expr2{mask2};
  const AtomicExpr expr3{mask3};
  const UnionExpr union_expr({&expr1, &expr2});
  const DifferenceExpr difference_expr(union_expr, {&expr3});

  const IndexMask result = IndexMask::from_expr(difference_expr, IndexRange(100), memory);
  std::cout << result << "\n";
}

TEST(index_mask, ToRange)
{
  IndexMaskMemory memory;
  {
    const IndexMask mask = IndexMask::from_indices<int>({4, 5, 6, 7}, memory);
    EXPECT_TRUE(mask.to_range().has_value());
    EXPECT_EQ(*mask.to_range(), IndexRange(4, 4));
  }
  {
    const IndexMask mask = IndexMask::from_indices<int>({}, memory);
    EXPECT_TRUE(mask.to_range().has_value());
    EXPECT_EQ(*mask.to_range(), IndexRange());
  }
  {
    const IndexMask mask = IndexMask::from_indices<int>({0, 1, 3, 4}, memory);
    EXPECT_FALSE(mask.to_range().has_value());
  }
  {
    const IndexRange range{16000, 40000};
    const IndexMask mask{range};
    EXPECT_TRUE(mask.to_range().has_value());
    EXPECT_EQ(*mask.to_range(), range);
  }
}

TEST(index_mask, FromRange)
{
  const auto test_range = [](const IndexRange range) {
    const IndexMask mask = range;
    EXPECT_EQ(mask.to_range(), range);
  };

  test_range({0, 0});
  test_range({0, 10});
  test_range({0, 16384});
  test_range({16320, 64});
  test_range({16384, 64});
  test_range({0, 100000});
  test_range({100000, 100000});
  test_range({688064, 64});
}

TEST(index_mask, FromPredicate)
{
  IndexMaskMemory memory;
  {
    const IndexRange range{20'000, 50'000};
    const IndexMask mask = IndexMask::from_predicate(
        IndexRange(100'000), GrainSize(1024), memory, [&](const int64_t i) {
          return range.contains(i);
        });
    EXPECT_EQ(mask.to_range(), range);
  }
  {
    const Vector<int64_t> indices = {0, 500, 20'000, 50'000};
    const IndexMask mask = IndexMask::from_predicate(
        IndexRange(100'000), GrainSize(1024), memory, [&](const int64_t i) {
          return indices.contains(i);
        });
    EXPECT_EQ(mask.size(), indices.size());
    Vector<int64_t> new_indices(mask.size());
    mask.to_indices<int64_t>(new_indices);
    EXPECT_EQ(indices, new_indices);
  }
}

}  // namespace blender::index_mask::tests
