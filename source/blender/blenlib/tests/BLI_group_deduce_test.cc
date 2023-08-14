/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "BLI_array.hh"
#include "BLI_group_deduce.hh"
#include "BLI_span.hh"

#include "testing/testing.h"

namespace blender::grouped_indices::tests {

TEST(grouped_indices, SimpleTest)
{
  const Span<int> group_indices({5, 5, 5, 3, 3, 3, 4, 4, 4, 2, 2, 2, 1, 1, 1, 0, 0, 0});
  const Span<int> groupped_indices({15, 16, 17, 12, 13, 14, 9, 10, 11, 3, 4, 5, 6, 7, 8, 0, 1, 2});
  const Span<int> groupped_offset({0, 3, 6, 9, 12, 15, 18});

  const auto test = [&](const bool fragmented, const char *test_name) {
    Array<int> offset(7, 0);
    Array<int> indices(18);
    from_indices(group_indices, fragmented, offset, indices);

    EXPECT_TRUE(offset.as_span() == groupped_offset) << test_name;
    for (const int i : offset.index_range()) {
      EXPECT_EQ(offset[i], groupped_offset[i]) << test_name;
    }

    EXPECT_TRUE(indices.as_span() == groupped_indices);
    for (const int i : indices.index_range()) {
      EXPECT_EQ(indices[i], groupped_indices[i]) << test_name;
    }
  };

  test(false, "fragmented == false");
  test(true, "fragmented == true");
}

}  // namespace blender::grouped_indices::tests
