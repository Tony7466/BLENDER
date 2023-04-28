/* SPDX-License-Identifier: Apache-2.0 */

#include "BLI_fixed_width_int.hh"

#include "testing/testing.h"

namespace blender::fixed_width_int::tests {

TEST(fixed_width_int, Test)
{
  UInt128 a{"1000000000000000000000"};
  UInt128 b{"5000"};
  UInt128 c = a + b;
  a.print();
  b.print();
  c.print();

  // const MyUint16 a(280);
  // const MyUint16 b(100);

  // const MyUint16 c = a * b;
  // std::cout << a << " * " << b << " = " << c << "\n";
}

}  // namespace blender::fixed_width_int::tests
