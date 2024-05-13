/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "testing/testing.h"

#include "BLI_math_quaternion.hh"
#include "BLI_math_quaternion_types.hh"

namespace blender::tests {

static void quaternion_to_axis_angle_to_quaternion_test(const math::Quaternion input)
{
  const math::Quaternion result = math::to_quaternion(math::to_axis_angle(input));
  EXPECT_V3_NEAR(input.imaginary_part(), result.imaginary_part(), 1e-6);
  EXPECT_NEAR(input.w, result.w, 1e-6);
}

TEST(math_quaternion, conversion)
{
  quaternion_to_axis_angle_to_quaternion_test({});
}

}  // namespace blender::tests
