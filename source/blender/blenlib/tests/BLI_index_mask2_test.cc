/* SPDX-License-Identifier: Apache-2.0 */

#include "BLI_index_mask2.hh"
#include "BLI_strict_flags.h"
#include "BLI_timeit.hh"

#include "testing/testing.h"

namespace blender::tests {

IndexMask2 mask;

TEST(index_mask2, Test)
{
  for ([[maybe_unused]] const int64_t i : IndexRange(5)) {
    SCOPED_TIMER("test");
    IndexMaskForRange mask_for_range(IndexRange(100'000'000));
    mask = mask_for_range;
  }
  // for (const int64_t i : mask) {
  //   std::cout << i << "\n";
  // }
}

}  // namespace blender::tests
