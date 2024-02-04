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
  const IndexMask mask_a = IndexMask::from_initializers({5, IndexRange(50, 100), 100000}, memory);
  const IndexMask mask_b = IndexMask::from_initializers({IndexRange(10, 10), 60, 200}, memory);

  ExprBuilder builder;
  const Expr &expr = builder.merge(&mask_a, &mask_b);
  const IndexMask union_mask = evaluate_expression(expr, memory);

  EXPECT_EQ(union_mask,
            IndexMask::from_initializers({5, IndexRange(10, 10), IndexRange(50, 100), 200, 100000},
                                         memory));
}

}  // namespace blender::index_mask::tests
