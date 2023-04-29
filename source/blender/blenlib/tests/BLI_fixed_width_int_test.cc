/* SPDX-License-Identifier: Apache-2.0 */

#include "BLI_fixed_width_int.hh"
#include "BLI_rand.hh"

#include "testing/testing.h"

namespace blender::fixed_width_int::tests {

TEST(fixed_width_int, Test)
{
  // UInt128_8 a{4806932020};
  // UInt128_8 b{65373};
  // UInt128_8 c = a - b;

  // std::cout << a << " - " << b << " = " << c << "\n";

  // Int128 a{-5};
  // Int128 b{"-5"};
  // Int128 c = a * b;
  // a.print();
  // b.print();
  // c.print();
  // UInt256 a{"100000000000"};
  // UInt256 b{"20000"};
  // UInt256 c = a * b;
  // a.print();
  // b.print();
  // c.print();

  // const MyUint16 a(280);
  // const MyUint16 b(100);

  // const MyUint16 c = a * b;
  // std::cout << a << " * " << b << " = " << c << "\n";
}

TEST(fixed_width_int, Fuzzy)
{
  RandomNumberGenerator rng;
  for ([[maybe_unused]] const int i : IndexRange(100000)) {
    const uint64_t a = rng.get_uint64();
    const uint64_t b = rng.get_uint64();
    EXPECT_EQ(a + b, uint64_t(UInt128_8(a) + UInt128_8(b)));
    EXPECT_EQ(a * b, uint64_t(UInt128_8(a) * UInt128_8(b)));
    EXPECT_EQ(a - b, uint64_t(UInt128_8(a) - UInt128_8(b)));
    EXPECT_EQ(a < b, UInt128_8(a) < UInt128_8(b));
    EXPECT_EQ(a > b, UInt128_8(a) > UInt128_8(b));
    EXPECT_EQ(a <= b, UInt128_8(a) <= UInt128_8(b));
    EXPECT_EQ(a >= b, UInt128_8(a) >= UInt128_8(b));
    EXPECT_EQ(a == b, UInt128_8(a) == UInt128_8(b));
    EXPECT_EQ(a != b, UInt128_8(a) != UInt128_8(b));
  }
}

}  // namespace blender::fixed_width_int::tests
