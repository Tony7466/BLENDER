/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "BLI_array.hh"
#include "BLI_index_mask_expression.hh"
#include "BLI_rand.hh"
#include "BLI_set.hh"
#include "BLI_strict_flags.h"
#include "BLI_timeit.hh"

#include "testing/testing.h"

namespace blender::index_mask::tests {

TEST(index_mask_expression, Union)
{
  IndexMaskMemory memory;
  const IndexMask mask_a = IndexMask::from_initializers({5, IndexRange(50, 100), 100'000}, memory);
  const IndexMask mask_b = IndexMask::from_initializers({IndexRange(10, 10), 60, 200}, memory);

  ExprBuilder builder;
  const Expr &expr = builder.merge(&mask_a, &mask_b);
  const IndexMask union_mask = evaluate_expression(expr, memory);

  EXPECT_EQ(union_mask,
            IndexMask::from_initializers(
                {5, IndexRange(10, 10), IndexRange(50, 100), 200, 100'000}, memory));
}

TEST(index_mask_expression, Intersection)
{
  IndexMaskMemory memory;
  const IndexMask mask_a = IndexMask::from_initializers({5, IndexRange(50, 100), 100'000}, memory);
  const IndexMask mask_b = IndexMask::from_initializers(
      {5, 6, IndexRange(100, 100), 80000, 100'000}, memory);

  ExprBuilder builder;
  const Expr &expr = builder.intersect(&mask_a, &mask_b);
  const IndexMask intersection_mask = evaluate_expression(expr, memory);

  EXPECT_EQ(intersection_mask,
            IndexMask::from_initializers({5, IndexRange(100, 50), 100'000}, memory));
}

TEST(index_mask_expression, Difference)
{
  IndexMaskMemory memory;
  const IndexMask mask_a = IndexMask::from_initializers({5, IndexRange(50, 100), 100'000}, memory);
  const IndexMask mask_b = IndexMask::from_initializers({5, 60, IndexRange(100, 20)}, memory);

  ExprBuilder builder;
  const Expr &expr = builder.subtract(&mask_a, &mask_b);
  const IndexMask difference_mask = evaluate_expression(expr, memory);

  EXPECT_EQ(difference_mask,
            IndexMask::from_initializers(
                {IndexRange(50, 10), IndexRange(61, 39), IndexRange(120, 30), 100'000}, memory));
}

TEST(index_mask_expression, UnionLargeRanges)
{
  IndexMaskMemory memory;
  const IndexMask mask_a(IndexRange(0, 1'000'000));
  const IndexMask mask_b(IndexRange(900'000, 1'100'000));

  ExprBuilder builder;
  const Expr &expr = builder.merge(&mask_a, &mask_b);
  const IndexMask result_mask = evaluate_expression(expr, memory);

  EXPECT_EQ(result_mask, IndexMask(IndexRange(0, 2'000'000)));
}

TEST(index_mask_expression, Benchmark)
{
#ifdef NDEBUG
  const int64_t iterations = 1'000'000;
#else
  const int64_t iterations = 10;
#endif

  for ([[maybe_unused]] const int64_t _1 : IndexRange(5)) {
    const IndexMask a{IndexRange::from_begin_end(0, 3'000'000)};
    const IndexMask b{IndexRange::from_begin_end(000'000, 2'000'000)};
    ExprBuilder builder;
    const Expr &expr = builder.subtract(&a, &b);

    SCOPED_TIMER("benchmark");
    for ([[maybe_unused]] const int64_t _2 : IndexRange(iterations)) {
      IndexMaskMemory memory;
      evaluate_expression(expr, memory);
    }
  }
}

}  // namespace blender::index_mask::tests
