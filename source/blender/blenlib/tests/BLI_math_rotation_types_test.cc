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

TEST(math_rotation_types, QuaternionProduct)
{
  Quaternion q1{1.0f, 2.0f, 3.0f, 4.0f};
  Quaternion q2{3.0f, 4.0f, 5.0f, 6.0f};
  Quaternion result = q1 * q2;
  EXPECT_EQ(result.w, -44.0f);
  EXPECT_EQ(result.x, 8.0f);
  EXPECT_EQ(result.y, 18.0f);
  EXPECT_EQ(result.z, 16.0f);

  Quaternion result2 = q1 * 4.0f;
  EXPECT_EQ(result2.w, 4.0f);
  EXPECT_EQ(result2.x, 8.0f);
  EXPECT_EQ(result2.y, 12.0f);
  EXPECT_EQ(result2.z, 16.0f);
}

TEST(math_rotation_types, QuaternionUnaryMinus)
{
  Quaternion q{1.0f, 2.0f, 3.0f, 4.0f};
  Quaternion result = -q;
  EXPECT_EQ(result.w, -1.0f);
  EXPECT_EQ(result.x, -2.0f);
  EXPECT_EQ(result.y, -3.0f);
  EXPECT_EQ(result.z, -4.0f);
}

TEST(math_rotation_types, QuaternionExpmap)
{
  Quaternion q(0.927091f, 0.211322f, -0.124857f, 0.283295f);
  float3 expmap = normalize(q).expmap();
  EXPECT_V3_NEAR(expmap, float3(0.433225f, -0.255966f, 0.580774f), 1e-4f);
  EXPECT_V4_NEAR(float4(Quaternion::expmap(expmap)), float4(q), 1e-4f);
}

TEST(math_rotation_types, QuaternionTwistSwing)
{
  Quaternion q(0.927091f, 0.211322f, -0.124857f, 0.283295f);
  EXPECT_NEAR(float(q.twist_angle(eAxis::X)), 0.448224, 1e-4);
  EXPECT_NEAR(float(q.twist_angle(eAxis::Y)), -0.267741, 1e-4);
  EXPECT_NEAR(float(q.twist_angle(eAxis::Z)), 0.593126, 1e-4);
  EXPECT_V4_NEAR(float4(q.twist(eAxis::X)), float4(0.950871, 0, -0.184694, 0.248462), 1e-4);
  EXPECT_V4_NEAR(float4(q.twist(eAxis::Y)), float4(0.935461, 0.17162, 0, 0.308966), 1e-4);
  EXPECT_V4_NEAR(float4(q.twist(eAxis::Z)), float4(0.969409, 0.238585, -0.0576509, 0), 1e-4);
  EXPECT_V4_NEAR(float4(q.swing(eAxis::X)), float4(0.974992, 0.222241, 0, 0), 1e-4);
  EXPECT_V4_NEAR(float4(q.swing(eAxis::Y)), float4(0.991053, 0, -0.133471, 0), 1e-4);
  EXPECT_V4_NEAR(float4(q.swing(eAxis::Z)), float4(0.956347, 0, 0, 0.292235), 1e-4);
}

TEST(math_rotation_types, AngleMethods)
{
  EXPECT_NEAR(float(AngleRadian(M_PI * 0.5).wrapped()), M_PI * 0.5, 1e-4f);
  EXPECT_NEAR(float(AngleRadian(M_PI * 2.0).wrapped()), 0.0, 1e-4f);
  EXPECT_NEAR(float(AngleRadian(M_PI * 2.5).wrapped()), M_PI * 0.5, 1e-4f);
  EXPECT_NEAR(float(AngleRadian(M_PI * 1.5).wrapped()), M_PI * -0.5, 1e-4f);
  EXPECT_NEAR(float(AngleRadian(M_PI * 0.5).wrapped_around(-M_PI)), -M_PI * 1.5, 1e-4f);
  EXPECT_NEAR(float(AngleRadian(M_PI * 1.0).wrapped_around(M_PI * 0.5)), M_PI, 1e-4f);
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
