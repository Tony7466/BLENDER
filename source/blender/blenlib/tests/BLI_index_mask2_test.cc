/* SPDX-License-Identifier: Apache-2.0 */

#include "BLI_index_mask2.hh"
#include "BLI_strict_flags.h"
#include "BLI_timeit.hh"

#include "testing/testing.h"

namespace blender::index_mask::tests {

TEST(index_mask2, Test)
{
  IndexMask mask(500000);
  IndexMask sliced_mask = mask.slice(IndexRange(16000, 1000));
  sliced_mask = sliced_mask.slice(IndexRange(100, 800));
  sliced_mask.foreach_index_span([&](const int64_t offset, const Span<int16_t> indices) {
    for (const int64_t i : indices) {
      std::cout << (i + offset) << "\n";
    }
  });
}

}  // namespace blender::index_mask::tests
