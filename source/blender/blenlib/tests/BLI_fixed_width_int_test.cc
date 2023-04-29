/* SPDX-License-Identifier: Apache-2.0 */

#include "BLI_fixed_width_int.hh"
#include "BLI_rand.hh"
#include "BLI_timeit.hh"
#include "BLI_vector.hh"

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

template<typename T, size_t S>
void init_random(RandomNumberGenerator &rng, std::array<T, S> &values)
{
  for (int i = 0; i < S; i++) {
    values[i] = rng.get_uint64();
  }
}

TEST(fixed_width_int, Performance)
{
  const int amount = 1'000'000;

  using IntT = UInt256_32;
  Vector<IntT> a(amount);
  Vector<IntT> b(amount);
  Vector<IntT> c(amount);

  RandomNumberGenerator rng;
  for (int i = 0; i < amount; i++) {
    init_random(rng, a[i].v);
    init_random(rng, b[i].v);
    c[i] = IntT(0);
  }

  for ([[maybe_unused]] const int iter : IndexRange(10)) {
    {
      SCOPED_TIMER("add");
      for (const int i : a.index_range()) {
        c[i] = a[i] + b[i];
      }
    }
    {
      SCOPED_TIMER("sub");
      for (const int i : a.index_range()) {
        c[i] = a[i] - b[i];
      }
    }
    {
      SCOPED_TIMER("mul");
      for (const int i : a.index_range()) {
        c[i] = a[i] * b[i];
      }
    }
  }
}

}  // namespace blender::fixed_width_int::tests
