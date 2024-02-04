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
  const IndexMask mask_a = IndexMask::from_initializers({5, IndexRange(50, 100)}, memory);
  const IndexMask mask_b = IndexMask::from_initializers({IndexRange(10, 10), 60, 200}, memory);

  AtomicExpr a_expr{mask_a};
  AtomicExpr b_expr{mask_b};
  UnionExpr union_expr{{&a_expr, &b_expr}};
  const IndexMask union_mask = evaluate_expression(union_expr, memory);

  EXPECT_EQ(
      union_mask,
      IndexMask::from_initializers({5, IndexRange(10, 10), IndexRange(50, 100), 200}, memory));
}

}  // namespace blender::index_mask::tests
