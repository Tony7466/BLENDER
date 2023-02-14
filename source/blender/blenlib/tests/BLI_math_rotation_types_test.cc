/* SPDX-License-Identifier: Apache-2.0 */

#include "testing/testing.h"

#include "BLI_math_rotation.h"
#include "BLI_math_rotation.hh"
#include "BLI_math_rotation_types.hh"

namespace blender::tests {

using namespace blender::math;

TEST(math_rotation_types, QuaternionDefaultConstructor)
{
  Quaternion q{};
  EXPECT_EQ(q.w, 0.0f);
  EXPECT_EQ(q.x, 0.0f);
  EXPECT_EQ(q.y, 0.0f);
  EXPECT_EQ(q.z, 0.0f);
}

TEST(math_rotation_types, QuaternionStaticConstructor)
{
  Quaternion q = Quaternion::identity();
  EXPECT_EQ(q.w, 1.0f);
  EXPECT_EQ(q.x, 0.0f);
  EXPECT_EQ(q.y, 0.0f);
  EXPECT_EQ(q.z, 0.0f);
}

TEST(math_rotation_types, QuaternionVectorConstructor)
{
  Quaternion q{1.0f, 2.0f, 3.0f, 4.0f};
  EXPECT_EQ(q.w, 1.0f);
  EXPECT_EQ(q.x, 2.0f);
  EXPECT_EQ(q.y, 3.0f);
  EXPECT_EQ(q.z, 4.0f);
}

TEST(math_rotation_types, TypeConversion)
{
  /* All the same rotation. */
  Quaternion quaternion(0.927091f, 0.211322f, -0.124857f, 0.283295f);
  EulerXYZ euler_xyz(deg_to_rad(20.0559), deg_to_rad(-20.5632f), deg_to_rad(30.3091f));
  AxisAngle axis_angle({0.563771, -0.333098, 0.755783}, deg_to_rad(44.0284f));

  EXPECT_V4_NEAR(float4(Quaternion(euler_xyz)), float4(quaternion), 1e-4);
  EXPECT_V3_NEAR(AxisAngle(euler_xyz).axis(), axis_angle.axis(), 1e-4);
  EXPECT_NEAR(AxisAngle(euler_xyz).angle(), axis_angle.angle(), 1e-4);

  EXPECT_V3_NEAR(float3(EulerXYZ(quaternion)), float3(euler_xyz), 1e-4);
  EXPECT_V3_NEAR(AxisAngle(quaternion).axis(), axis_angle.axis(), 1e-4);
  EXPECT_NEAR(AxisAngle(quaternion).angle(), axis_angle.angle(), 1e-4);

  EXPECT_V3_NEAR(float3(EulerXYZ(axis_angle)), float3(euler_xyz), 1e-4);
  EXPECT_V4_NEAR(float4(Quaternion(axis_angle)), float4(quaternion), 1e-4);
}

}  // namespace blender::tests
