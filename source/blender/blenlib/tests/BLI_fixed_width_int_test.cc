/* SPDX-License-Identifier: Apache-2.0 */

#include "BLI_fixed_width_int.hh"
#include "BLI_rand.hh"

#include "testing/testing.h"

namespace blender::fixed_width_int::tests {

TEST(fixed_width_int, Fuzzy)
{
  RandomNumberGenerator rng;
  for ([[maybe_unused]] const int i : IndexRange(10000)) {
    {
      const uint64_t a = rng.get_uint64();
      const uint64_t b = rng.get_uint64();
      EXPECT_EQ(a + b, uint64_t(UInt64_8(a) + UInt64_8(b)));
      EXPECT_EQ(a * b, uint64_t(UInt64_8(a) * UInt64_8(b)));
      EXPECT_EQ(a - b, uint64_t(UInt64_8(a) - UInt64_8(b)));
      EXPECT_EQ(a < b, UInt64_8(a) < UInt64_8(b));
      EXPECT_EQ(a > b, UInt64_8(a) > UInt64_8(b));
      EXPECT_EQ(a <= b, UInt64_8(a) <= UInt64_8(b));
      EXPECT_EQ(a >= b, UInt64_8(a) >= UInt64_8(b));
      EXPECT_EQ(a == b, UInt64_8(a) == UInt64_8(b));
      EXPECT_EQ(a != b, UInt64_8(a) != UInt64_8(b));
    }
    {
      const int64_t a = int64_t(rng.get_uint64()) * (rng.get_float() < 0.5f ? -1 : 1);
      const int64_t b = int64_t(rng.get_uint64()) * (rng.get_float() < 0.5f ? -1 : 1);
      EXPECT_EQ(a + b, int64_t(Int64_8(a) + Int64_8(b)));
      EXPECT_EQ(a * b, int64_t(Int64_8(a) * Int64_8(b)));
      EXPECT_EQ(a - b, int64_t(Int64_8(a) - Int64_8(b)));
      EXPECT_EQ(a < b, Int64_8(a) < Int64_8(b));
      EXPECT_EQ(a > b, Int64_8(a) > Int64_8(b));
      EXPECT_EQ(a <= b, Int64_8(a) <= Int64_8(b));
      EXPECT_EQ(a >= b, Int64_8(a) >= Int64_8(b));
      EXPECT_EQ(a == b, Int64_8(a) == Int64_8(b));
      EXPECT_EQ(a != b, Int64_8(a) != Int64_8(b));
    }
  }
}

}  // namespace blender::fixed_width_int::tests
