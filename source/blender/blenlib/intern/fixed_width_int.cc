/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_fixed_width_int.hh"
#include "BLI_rand.hh"
#include "BLI_timeit.hh"
#include "BLI_vector.hh"

namespace blender::fixed_width_int {

template<typename T, size_t S>
static void init_random(RandomNumberGenerator &rng, std::array<T, S> &values)
{
  for (int i = 0; i < S; i++) {
    values[i] = rng.get_uint64();
  }
}

void test_performance()
{
  const int amount = 1'000'000;

  using IntT = UInt256_64;
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

}  // namespace blender::fixed_width_int
