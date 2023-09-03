/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "BLI_array.hh"
#include "BLI_group_deduce.hh"
#include "BLI_noise.hh"
#include "BLI_span.hh"
#include "BLI_vector_set.hh"

#include "testing/testing.h"

namespace blender::grouped_indices::tests {

TEST(grouped_indices, EmptyTest)
{
  const Span<int> group_indices({});
  const Span<int> groupped_indices({});
  const Span<int> groupped_offset({});

  Array<int> offset;
  Array<int> indices;
  from_indices(group_indices, offset, indices, false);
  from_indices(group_indices, offset, indices, true);
}

TEST(grouped_indices, SimpleTest)
{
  const Span<int> group_indices({5, 5, 5, 3, 3, 3, 4, 4, 4, 2, 2, 2, 1, 1, 1, 0, 0, 0});
  const Span<int> groupped_indices({15, 16, 17, 12, 13, 14, 9, 10, 11, 3, 4, 5, 6, 7, 8, 0, 1, 2});
  const Span<int> groupped_offset({0, 3, 6, 9, 12, 15, 18});

  const auto test = [&](const bool fragmented, const char *test_name) {
    Array<int> offset(7, 0);
    Array<int> indices(18);
    from_indices(group_indices, offset, indices, fragmented);

    EXPECT_TRUE(offset.as_span() == groupped_offset) << test_name;
    for (const int i : offset.index_range()) {
      EXPECT_EQ(offset[i], groupped_offset[i]) << test_name;
    }

    EXPECT_TRUE(indices.as_span() == groupped_indices) << test_name;
    for (const int i : indices.index_range()) {
      EXPECT_EQ(indices[i], groupped_indices[i]) << test_name;
    }
  };

  test(false, "fragmented == false");
  test(true, "fragmented == true");
}

TEST(grouped_indices, LargeTest)
{
  const auto test =
      [](const int size, const int max_value, const bool fragmented, const int hash) {
        std::stringstream test_name_str;
        test_name_str << "size: " << size << ", max_value: " << max_value << std::boolalpha
                      << ", fragmented: " << fragmented;
        const std::string test_name = test_name_str.str();

        Array<int> group_indices(size);
        for (const int index : group_indices.index_range()) {
          const float factor = noise::hash_to_float(index, hash);
          group_indices[index] = max_value * factor;
        }

        const int total_groups = identifiers_to_indices(group_indices);

        Array<int> offset(total_groups + 1, 0);
        Array<int> indices(group_indices.size());

        const GroupedSpan<int> groups = from_indices(group_indices, offset, indices, false);

        Array<bool> group_is_filled(total_groups, false);
        Array<int> group_indices_to_fill(group_indices.size(), -1);
        for (const int group_index : groups.index_range()) {
          if (groups[group_index].is_empty()) {
            continue;
          }
          const int group_number = group_indices[groups[group_index].first()];
          EXPECT_FALSE(group_is_filled[group_number]) << test_name;
          group_is_filled[group_number] = true;

          for (const int index : groups[group_index]) {
            EXPECT_EQ(group_indices_to_fill[index], -1) << test_name;
            group_indices_to_fill[index] = group_number;
          }
        }

        for (const int i : group_indices_to_fill.index_range()) {
          EXPECT_EQ(group_indices_to_fill[i], group_indices[i]) << test_name;
        }
      };

  test(100000, 100, true, 0);
  test(10000, 100, true, 1);
  test(1000, 100, true, 2);
  test(100, 100, true, 3);

  test(100000, 100, false, 0);
  test(10000, 100, false, 1);
  test(1000, 100, false, 2);
  test(100, 100, false, 3);
}

}  // namespace blender::grouped_indices::tests
