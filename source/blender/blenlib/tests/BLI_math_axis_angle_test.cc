/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "testing/testing.h"

#include "BLI_math_axis_angle_types.hh"
#include "BLI_math_quaternion.hh"
#include "BLI_math_quaternion_types.hh"

namespace blender::tests {

TEST(math_axis_angle, from_quaternion)
{
  {
    const math::AxisAngle axis_angle({0.0f, 1.0f, 0.0f}, math::AngleRadian(0.0f));
    const math::Quaternion quaternion(1.0f, {0.0f, 0.0f, 0.0f});
    const math::AxisAngle from_quaternion = math::to_axis_angle(quaternion);
    EXPECT_V3_NEAR(axis_angle.axis(), from_quaternion.axis(), 1e-6);
    EXPECT_NEAR(axis_angle.angle().radian(), from_quaternion.angle().radian(), 1e-6);
  }
  {
    const math::AxisAngle axis_angle({0.0f, -1.0f, 0.0f}, math::AngleRadian(0));
    const math::Quaternion quaternion(-1.0f, {0.0f, 0.0f, 0.0f});
    const math::AxisAngle from_quaternion = math::to_axis_angle(quaternion);
    EXPECT_V3_NEAR(axis_angle.axis(), from_quaternion.axis(), 1e-6);
    EXPECT_NEAR(axis_angle.angle().radian(), from_quaternion.angle().radian(), 1e-6);
  }
}

}  // namespace blender::tests
