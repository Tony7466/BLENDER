/* SPDX-License-Identifier: Apache-2.0 */

#include "BLI_index_mask2.hh"
#include "BLI_strict_flags.h"

#include "testing/testing.h"

namespace blender::tests {

TEST(index_mask2, Test)
{
  IndexMaskForRange mask_for_range(IndexRange((1 << 16) - 10, 30));
  IndexMask2 mask = mask_for_range;
  for (const int64_t i : mask) {
    std::cout << i << "\n";
  }
}

}  // namespace blender::tests
