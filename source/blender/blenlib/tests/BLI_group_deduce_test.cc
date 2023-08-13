/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "BLI_array.hh"
#include "BLI_group_deduce.hh"

#include "testing/testing.h"

namespace blender::grouped_indices::tests {

TEST(grouped_indices, DefaultTest)
{
  static const Array<int> group_indices{5, 5, 5, 3, 3, 3, 4, 4, 4, 2, 2, 2, 1, 1, 1, 0, 0, 0};

  static const Array<int> groupped_indices{
      15, 16, 17, 12, 13, 14, 9, 10, 11, 3, 4, 5, 6, 7, 8, 0, 1, 2};
  static const Array<int> groupped_offsets{0, 3, 6, 9, 12, 15, 18};

  {
    Array<int> offset(7, 0);
    Array<int> indices(18);
    from_indices(group_indices, true, offset, indices);

    EXPECT_TRUE(offset.as_span() == groupped_offsets.as_span());
    for (const int i : offset.index_range()) {
      EXPECT_EQ(offset[i], groupped_offsets[i]);
    }

    EXPECT_TRUE(indices.as_span() == groupped_indices.as_span());
    for (const int i : indices.index_range()) {
      EXPECT_EQ(indices[i], groupped_indices[i]);
    }
  }

  {
    Array<int> offset(7, 0);
    Array<int> indices(18);
    from_indices(group_indices, false, offset, indices);

    EXPECT_TRUE(offset.as_span() == groupped_offsets.as_span());
    for (const int i : offset.index_range()) {
      EXPECT_EQ(offset[i], groupped_offsets[i]);
    }

    EXPECT_TRUE(indices.as_span() == groupped_indices.as_span());
    for (const int i : indices.index_range()) {
      EXPECT_EQ(indices[i], groupped_indices[i]);
    }
  }
}

}  // namespace blender::grouped_indices::tests
