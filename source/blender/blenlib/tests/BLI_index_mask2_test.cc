/* SPDX-License-Identifier: Apache-2.0 */

#include "BLI_index_mask2.hh"
#include "BLI_strict_flags.h"
#include "BLI_timeit.hh"

#include "testing/testing.h"

namespace blender::index_mask::tests {

TEST(index_mask2, Test)
{
  Vector<int> data;
  for (const int64_t i : IndexRange(100000)) {
    data.append(int(i));
  }

  for ([[maybe_unused]] const int64_t i : IndexRange(50)) {
    Vector<IndexRange> ranges = split_sorted_indices_by_chunk<int>(data);
  }
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

}  // namespace blender::index_mask::tests
