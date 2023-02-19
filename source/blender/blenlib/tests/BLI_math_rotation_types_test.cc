/* SPDX-License-Identifier: Apache-2.0 */

#include "testing/testing.h"

#include "DNA_action_types.h"

#include "BLI_math_rotation.h"
#include "BLI_math_rotation.hh"
#include "BLI_math_rotation_types.hh"

namespace blender::tests {

using namespace blender::math;

TEST(math_rotation_types, AxisSignedCross)
{
  EXPECT_EQ(
      basis_vector<float>(AxisConversion::cross(eAxisSigned::X_POS, eAxisSigned::Y_POS)),
      cross(basis_vector<float>(eAxisSigned::X_POS), basis_vector<float>(eAxisSigned::Y_POS)));
  EXPECT_EQ(
      basis_vector<float>(AxisConversion::cross(eAxisSigned::X_POS, eAxisSigned::Z_POS)),
      cross(basis_vector<float>(eAxisSigned::X_POS), basis_vector<float>(eAxisSigned::Z_POS)));
  EXPECT_EQ(
      basis_vector<float>(AxisConversion::cross(eAxisSigned::X_POS, eAxisSigned::Y_NEG)),
      cross(basis_vector<float>(eAxisSigned::X_POS), basis_vector<float>(eAxisSigned::Y_NEG)));
  EXPECT_EQ(
      basis_vector<float>(AxisConversion::cross(eAxisSigned::X_POS, eAxisSigned::Z_NEG)),
      cross(basis_vector<float>(eAxisSigned::X_POS), basis_vector<float>(eAxisSigned::Z_NEG)));
  EXPECT_EQ(
      basis_vector<float>(AxisConversion::cross(eAxisSigned::Y_POS, eAxisSigned::X_POS)),
      cross(basis_vector<float>(eAxisSigned::Y_POS), basis_vector<float>(eAxisSigned::X_POS)));
  EXPECT_EQ(
      basis_vector<float>(AxisConversion::cross(eAxisSigned::Y_POS, eAxisSigned::Z_POS)),
      cross(basis_vector<float>(eAxisSigned::Y_POS), basis_vector<float>(eAxisSigned::Z_POS)));
  EXPECT_EQ(
      basis_vector<float>(AxisConversion::cross(eAxisSigned::Y_POS, eAxisSigned::X_NEG)),
      cross(basis_vector<float>(eAxisSigned::Y_POS), basis_vector<float>(eAxisSigned::X_NEG)));
  EXPECT_EQ(
      basis_vector<float>(AxisConversion::cross(eAxisSigned::Y_POS, eAxisSigned::Z_NEG)),
      cross(basis_vector<float>(eAxisSigned::Y_POS), basis_vector<float>(eAxisSigned::Z_NEG)));
  EXPECT_EQ(
      basis_vector<float>(AxisConversion::cross(eAxisSigned::Z_POS, eAxisSigned::X_POS)),
      cross(basis_vector<float>(eAxisSigned::Z_POS), basis_vector<float>(eAxisSigned::X_POS)));
  EXPECT_EQ(
      basis_vector<float>(AxisConversion::cross(eAxisSigned::Z_POS, eAxisSigned::Y_POS)),
      cross(basis_vector<float>(eAxisSigned::Z_POS), basis_vector<float>(eAxisSigned::Y_POS)));
  EXPECT_EQ(
      basis_vector<float>(AxisConversion::cross(eAxisSigned::Z_POS, eAxisSigned::X_NEG)),
      cross(basis_vector<float>(eAxisSigned::Z_POS), basis_vector<float>(eAxisSigned::X_NEG)));
  EXPECT_EQ(
      basis_vector<float>(AxisConversion::cross(eAxisSigned::Z_POS, eAxisSigned::Y_NEG)),
      cross(basis_vector<float>(eAxisSigned::Z_POS), basis_vector<float>(eAxisSigned::Y_NEG)));
  EXPECT_EQ(
      basis_vector<float>(AxisConversion::cross(eAxisSigned::X_NEG, eAxisSigned::Y_POS)),
      cross(basis_vector<float>(eAxisSigned::X_NEG), basis_vector<float>(eAxisSigned::Y_POS)));
  EXPECT_EQ(
      basis_vector<float>(AxisConversion::cross(eAxisSigned::X_NEG, eAxisSigned::Z_POS)),
      cross(basis_vector<float>(eAxisSigned::X_NEG), basis_vector<float>(eAxisSigned::Z_POS)));
  EXPECT_EQ(
      basis_vector<float>(AxisConversion::cross(eAxisSigned::X_NEG, eAxisSigned::Y_NEG)),
      cross(basis_vector<float>(eAxisSigned::X_NEG), basis_vector<float>(eAxisSigned::Y_NEG)));
  EXPECT_EQ(
      basis_vector<float>(AxisConversion::cross(eAxisSigned::X_NEG, eAxisSigned::Z_NEG)),
      cross(basis_vector<float>(eAxisSigned::X_NEG), basis_vector<float>(eAxisSigned::Z_NEG)));
  EXPECT_EQ(
      basis_vector<float>(AxisConversion::cross(eAxisSigned::Y_NEG, eAxisSigned::X_POS)),
      cross(basis_vector<float>(eAxisSigned::Y_NEG), basis_vector<float>(eAxisSigned::X_POS)));
  EXPECT_EQ(
      basis_vector<float>(AxisConversion::cross(eAxisSigned::Y_NEG, eAxisSigned::Z_POS)),
      cross(basis_vector<float>(eAxisSigned::Y_NEG), basis_vector<float>(eAxisSigned::Z_POS)));
  EXPECT_EQ(
      basis_vector<float>(AxisConversion::cross(eAxisSigned::Y_NEG, eAxisSigned::X_NEG)),
      cross(basis_vector<float>(eAxisSigned::Y_NEG), basis_vector<float>(eAxisSigned::X_NEG)));
  EXPECT_EQ(
      basis_vector<float>(AxisConversion::cross(eAxisSigned::Y_NEG, eAxisSigned::Z_NEG)),
      cross(basis_vector<float>(eAxisSigned::Y_NEG), basis_vector<float>(eAxisSigned::Z_NEG)));
  EXPECT_EQ(
      basis_vector<float>(AxisConversion::cross(eAxisSigned::Z_NEG, eAxisSigned::X_POS)),
      cross(basis_vector<float>(eAxisSigned::Z_NEG), basis_vector<float>(eAxisSigned::X_POS)));
  EXPECT_EQ(
      basis_vector<float>(AxisConversion::cross(eAxisSigned::Z_NEG, eAxisSigned::Y_POS)),
      cross(basis_vector<float>(eAxisSigned::Z_NEG), basis_vector<float>(eAxisSigned::Y_POS)));
  EXPECT_EQ(
      basis_vector<float>(AxisConversion::cross(eAxisSigned::Z_NEG, eAxisSigned::X_NEG)),
      cross(basis_vector<float>(eAxisSigned::Z_NEG), basis_vector<float>(eAxisSigned::X_NEG)));
  EXPECT_EQ(
      basis_vector<float>(AxisConversion::cross(eAxisSigned::Z_NEG, eAxisSigned::Y_NEG)),
      cross(basis_vector<float>(eAxisSigned::Z_NEG), basis_vector<float>(eAxisSigned::Y_NEG)));
}

TEST(math_rotation_types, Euler3Order)
{
  /* Asserts those match.
   * Do not do it in the header to avoid including the DNA header everywhere.
   */
  BLI_STATIC_ASSERT(
      static_cast<int>(eEulerOrder::XYZ) == static_cast<int>(eRotationModes::ROT_MODE_XYZ), "");
  BLI_STATIC_ASSERT(
      static_cast<int>(eEulerOrder::XZY) == static_cast<int>(eRotationModes::ROT_MODE_XZY), "");
  BLI_STATIC_ASSERT(
      static_cast<int>(eEulerOrder::YXZ) == static_cast<int>(eRotationModes::ROT_MODE_YXZ), "");
  BLI_STATIC_ASSERT(
      static_cast<int>(eEulerOrder::YZX) == static_cast<int>(eRotationModes::ROT_MODE_YZX), "");
  BLI_STATIC_ASSERT(
      static_cast<int>(eEulerOrder::ZXY) == static_cast<int>(eRotationModes::ROT_MODE_ZXY), "");
  BLI_STATIC_ASSERT(
      static_cast<int>(eEulerOrder::ZYX) == static_cast<int>(eRotationModes::ROT_MODE_ZYX), "");

  EXPECT_EQ(float3(Euler3({0, 1, 2}, eEulerOrder::XYZ)), float3(0, 1, 2));
  EXPECT_EQ(float3(Euler3({0, 1, 2}, eEulerOrder::XZY)), float3(0, 1, 2));
  EXPECT_EQ(float3(Euler3({0, 1, 2}, eEulerOrder::YXZ)), float3(0, 1, 2));
  EXPECT_EQ(float3(Euler3({0, 1, 2}, eEulerOrder::YZX)), float3(0, 1, 2));
  EXPECT_EQ(float3(Euler3({0, 1, 2}, eEulerOrder::ZXY)), float3(0, 1, 2));
  EXPECT_EQ(float3(Euler3({0, 1, 2}, eEulerOrder::ZYX)), float3(0, 1, 2));

  EXPECT_EQ(float3(EulerXYZ(Euler3({0, 1, 2}, eEulerOrder::XYZ))), float3(0, 1, 2));
  EXPECT_EQ(float3(EulerXYZ(Euler3({0, 1, 2}, eEulerOrder::XZY))), float3(0, 2, 1));
  EXPECT_EQ(float3(EulerXYZ(Euler3({0, 1, 2}, eEulerOrder::YXZ))), float3(1, 0, 2));
  EXPECT_EQ(float3(EulerXYZ(Euler3({0, 1, 2}, eEulerOrder::YZX))), float3(1, 2, 0));
  EXPECT_EQ(float3(EulerXYZ(Euler3({0, 1, 2}, eEulerOrder::ZXY))), float3(2, 0, 1));
  EXPECT_EQ(float3(EulerXYZ(Euler3({0, 1, 2}, eEulerOrder::ZYX))), float3(2, 1, 0));
}

TEST(math_rotation_types, DualQuaternionUniformScaleConstructor)
{
  DualQuaternion q = {Quaternion::identity(), Quaternion::zero()};
  EXPECT_EQ(q.quat, Quaternion::identity());
  EXPECT_EQ(q.trans, Quaternion::zero());
  EXPECT_EQ(q.scale_weight, 0.0f);
  EXPECT_EQ(q.quat_weight, 1.0f);
}

TEST(math_rotation_types, DualQuaternionNonUniformScaleConstructor)
{
  DualQuaternion q = {Quaternion::identity(), Quaternion::zero(), float4x4::identity()};
  EXPECT_EQ(q.quat, Quaternion::identity());
  EXPECT_EQ(q.trans, Quaternion::zero());
  EXPECT_EQ(q.scale, float4x4::identity());
  EXPECT_EQ(q.scale_weight, 1.0f);
  EXPECT_EQ(q.quat_weight, 1.0f);
}

TEST(math_rotation_types, DualQuaternionOperators)
{
  DualQuaternion sum = DualQuaternion(Quaternion(0, 0, 1, 0), Quaternion(0, 1, 0, 1)) * 2.0f;

  EXPECT_EQ(sum.quat, Quaternion(0, 0, 2, 0));
  EXPECT_EQ(sum.trans, Quaternion(0, 2, 0, 2));
  EXPECT_EQ(sum.scale_weight, 0.0f);
  EXPECT_EQ(sum.quat_weight, 2.0f);

  sum += DualQuaternion(Quaternion(1, 0, 0, 0), Quaternion(1, 1, 1, 1), float4x4::identity()) *
         4.0f;

  EXPECT_EQ(sum.quat, Quaternion(4, 0, 2, 0));
  EXPECT_EQ(sum.trans, Quaternion(4, 6, 4, 6));
  EXPECT_EQ(sum.scale, float4x4::identity() * 4.0f);
  EXPECT_EQ(sum.scale_weight, 4.0f);
  EXPECT_EQ(sum.quat_weight, 6.0f);

  sum += 3.0f *
         DualQuaternion(Quaternion(1, 0, 0, 0), Quaternion(1, 0, 0, 0), float4x4::identity());

  EXPECT_EQ(sum.quat, Quaternion(7, 0, 2, 0));
  EXPECT_EQ(sum.trans, Quaternion(7, 6, 4, 6));
  EXPECT_EQ(sum.scale, float4x4::identity() * 7.0f);
  EXPECT_EQ(sum.scale_weight, 7.0f);
  EXPECT_EQ(sum.quat_weight, 9.0f);
}

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

  EXPECT_V4_NEAR(float4(q.twist(eAxis::X)), float4(0.974992, 0.222241, 0, 0), 1e-4);
  EXPECT_V4_NEAR(float4(q.twist(eAxis::Y)), float4(0.991053, 0, -0.133471, 0), 1e-4);
  EXPECT_V4_NEAR(float4(q.twist(eAxis::Z)), float4(0.956347, 0, 0, 0.292235), 1e-4);
  EXPECT_V4_NEAR(float4(q.swing(eAxis::X)), float4(0.950871, 0, -0.184694, 0.248462), 1e-4);
  EXPECT_V4_NEAR(float4(q.swing(eAxis::Y)), float4(0.935461, 0.17162, 0, 0.308966), 1e-4);
  EXPECT_V4_NEAR(float4(q.swing(eAxis::Z)), float4(0.969409, 0.238585, -0.0576509, 0), 1e-4);
  EXPECT_V4_NEAR(float4(q.swing(eAxis::Z) * q.twist(eAxis::Z)), float4(q), 1e-4);
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
  EulerXYZ euler_xyz(AngleRadian::from_degree(20.0559),
                     AngleRadian::from_degree(-20.5632f),
                     AngleRadian::from_degree(30.3091f));
  AxisAngle axis_angle({0.563771, -0.333098, 0.755783}, AngleRadian::from_degree(44.0284f));

  EXPECT_V4_NEAR(float4(Quaternion(euler_xyz)), float4(quaternion), 1e-4);
  EXPECT_V3_NEAR(AxisAngle(euler_xyz).axis(), axis_angle.axis(), 1e-4);
  EXPECT_NEAR(float(AxisAngle(euler_xyz).angle()), float(axis_angle.angle()), 1e-4);

  EXPECT_V3_NEAR(float3(EulerXYZ(quaternion)), float3(euler_xyz), 1e-4);
  EXPECT_V3_NEAR(AxisAngle(quaternion).axis(), axis_angle.axis(), 1e-4);
  EXPECT_NEAR(float(AxisAngle(quaternion).angle()), float(axis_angle.angle()), 1e-4);

  EXPECT_V3_NEAR(float3(EulerXYZ(axis_angle)), float3(euler_xyz), 1e-4);
  EXPECT_V4_NEAR(float4(Quaternion(axis_angle)), float4(quaternion), 1e-4);
}

TEST(math_rotation_types, Euler3Conversion)
{
  /* All the same rotation. */
  float3 ijk{0.350041, -0.358896, 0.528994};
  Euler3 euler3_xyz(ijk, eEulerOrder::XYZ);
  Euler3 euler3_xzy(ijk, eEulerOrder::XZY);
  Euler3 euler3_yxz(ijk, eEulerOrder::YXZ);
  Euler3 euler3_yzx(ijk, eEulerOrder::YZX);
  Euler3 euler3_zxy(ijk, eEulerOrder::ZXY);
  Euler3 euler3_zyx(ijk, eEulerOrder::ZYX);

  Quaternion quat_xyz(0.927091f, 0.211322f, -0.124857f, 0.283295f);
  Quaternion quat_xzy(0.943341f, 0.119427f, -0.124857f, 0.283295f);
  Quaternion quat_yxz(0.943341f, 0.211322f, -0.124857f, 0.223297f);
  Quaternion quat_yzx(0.927091f, 0.211322f, -0.214438f, 0.223297f);
  Quaternion quat_zxy(0.927091f, 0.119427f, -0.214438f, 0.283295f);
  Quaternion quat_zyx(0.943341f, 0.119427f, -0.214438f, 0.223297f);

  float3x3 mat_xyz = transpose(float3x3{{0.80831, -0.57805, -0.111775},
                                        {0.47251, 0.750174, -0.462572},
                                        {0.35124, 0.321087, 0.879508}});
  float3x3 mat_xzy = transpose(float3x3{{0.80831, -0.56431, -0.167899},
                                        {0.504665, 0.810963, -0.296063},
                                        {0.303231, 0.154577, 0.940296}});
  float3x3 mat_yxz = transpose(float3x3{{0.869098, -0.474061, -0.14119},
                                        {0.368521, 0.810963, -0.454458},
                                        {0.329941, 0.342937, 0.879508}});
  float3x3 mat_yzx = transpose(float3x3{{0.80831, -0.504665, -0.303231},
                                        {0.323403, 0.810963, -0.487596},
                                        {0.491982, 0.296063, 0.818719}});
  float3x3 mat_zxy = transpose(float3x3{{0.747521, -0.576499, -0.329941},
                                        {0.474061, 0.810963, -0.342937},
                                        {0.465272, 0.0999405, 0.879508}});
  float3x3 mat_zyx = transpose(float3x3{{0.80831, -0.47251, -0.35124},
                                        {0.370072, 0.871751, -0.321087},
                                        {0.457911, 0.129553, 0.879508}});

  EXPECT_V4_NEAR(float4(Quaternion(euler3_xyz)), float4(quat_xyz), 1e-4);
  EXPECT_V4_NEAR(float4(Quaternion(euler3_xzy)), float4(quat_xzy), 1e-4);
  EXPECT_V4_NEAR(float4(Quaternion(euler3_yxz)), float4(quat_yxz), 1e-4);
  EXPECT_V4_NEAR(float4(Quaternion(euler3_yzx)), float4(quat_yzx), 1e-4);
  EXPECT_V4_NEAR(float4(Quaternion(euler3_zxy)), float4(quat_zxy), 1e-4);
  EXPECT_V4_NEAR(float4(Quaternion(euler3_zyx)), float4(quat_zyx), 1e-4);

  EXPECT_V3_NEAR(Euler3(quat_xyz, eEulerOrder::XYZ).ijk(), ijk, 1e-4);
  EXPECT_V3_NEAR(Euler3(quat_xzy, eEulerOrder::XZY).ijk(), ijk, 1e-4);
  EXPECT_V3_NEAR(Euler3(quat_yxz, eEulerOrder::YXZ).ijk(), ijk, 1e-4);
  EXPECT_V3_NEAR(Euler3(quat_yzx, eEulerOrder::YZX).ijk(), ijk, 1e-4);
  EXPECT_V3_NEAR(Euler3(quat_zxy, eEulerOrder::ZXY).ijk(), ijk, 1e-4);
  EXPECT_V3_NEAR(Euler3(quat_zyx, eEulerOrder::ZYX).ijk(), ijk, 1e-4);

  EXPECT_M3_NEAR(from_rotation<float3x3>(euler3_xyz), mat_xyz, 1e-4);
  EXPECT_M3_NEAR(from_rotation<float3x3>(euler3_xzy), mat_xzy, 1e-4);
  EXPECT_M3_NEAR(from_rotation<float3x3>(euler3_yxz), mat_yxz, 1e-4);
  EXPECT_M3_NEAR(from_rotation<float3x3>(euler3_yzx), mat_yzx, 1e-4);
  EXPECT_M3_NEAR(from_rotation<float3x3>(euler3_zxy), mat_zxy, 1e-4);
  EXPECT_M3_NEAR(from_rotation<float3x3>(euler3_zyx), mat_zyx, 1e-4);

  EXPECT_V3_NEAR(to_euler(mat_xyz, eEulerOrder::XYZ).ijk(), ijk, 1e-4);
  EXPECT_V3_NEAR(to_euler(mat_xzy, eEulerOrder::XZY).ijk(), ijk, 1e-4);
  EXPECT_V3_NEAR(to_euler(mat_yxz, eEulerOrder::YXZ).ijk(), ijk, 1e-4);
  EXPECT_V3_NEAR(to_euler(mat_yzx, eEulerOrder::YZX).ijk(), ijk, 1e-4);
  EXPECT_V3_NEAR(to_euler(mat_zxy, eEulerOrder::ZXY).ijk(), ijk, 1e-4);
  EXPECT_V3_NEAR(to_euler(mat_zyx, eEulerOrder::ZYX).ijk(), ijk, 1e-4);

  AxisAngle axis_angle_xyz = AxisAngle({0.563771, -0.333098, 0.755783}, 0.76844f);
  AxisAngle axis_angle_xzy = AxisAngle({0.359907, -0.376274, 0.853747}, 0.676476f);
  AxisAngle axis_angle_yxz = AxisAngle({0.636846, -0.376274, 0.672937}, 0.676476f);
  AxisAngle axis_angle_yzx = AxisAngle({0.563771, -0.572084, 0.59572}, 0.76844f);
  AxisAngle axis_angle_zxy = AxisAngle({0.318609, -0.572084, 0.755783}, 0.76844f);
  AxisAngle axis_angle_zyx = AxisAngle({0.359907, -0.646237, 0.672937}, 0.676476f);

  EXPECT_V3_NEAR(AxisAngle(euler3_xyz).axis(), axis_angle_xyz.axis(), 1e-4);
  EXPECT_V3_NEAR(AxisAngle(euler3_xzy).axis(), axis_angle_xzy.axis(), 1e-4);
  EXPECT_V3_NEAR(AxisAngle(euler3_yxz).axis(), axis_angle_yxz.axis(), 1e-4);
  EXPECT_V3_NEAR(AxisAngle(euler3_yzx).axis(), axis_angle_yzx.axis(), 1e-4);
  EXPECT_V3_NEAR(AxisAngle(euler3_zxy).axis(), axis_angle_zxy.axis(), 1e-4);
  EXPECT_V3_NEAR(AxisAngle(euler3_zyx).axis(), axis_angle_zyx.axis(), 1e-4);

  EXPECT_NEAR(float(AxisAngle(euler3_xyz).angle()), float(axis_angle_xyz.angle()), 1e-4);
  EXPECT_NEAR(float(AxisAngle(euler3_xzy).angle()), float(axis_angle_xzy.angle()), 1e-4);
  EXPECT_NEAR(float(AxisAngle(euler3_yxz).angle()), float(axis_angle_yxz.angle()), 1e-4);
  EXPECT_NEAR(float(AxisAngle(euler3_yzx).angle()), float(axis_angle_yzx.angle()), 1e-4);
  EXPECT_NEAR(float(AxisAngle(euler3_zxy).angle()), float(axis_angle_zxy.angle()), 1e-4);
  EXPECT_NEAR(float(AxisAngle(euler3_zyx).angle()), float(axis_angle_zyx.angle()), 1e-4);

  EXPECT_V3_NEAR(Euler3(axis_angle_xyz, eEulerOrder::XYZ).ijk(), ijk, 1e-4);
  EXPECT_V3_NEAR(Euler3(axis_angle_xzy, eEulerOrder::XZY).ijk(), ijk, 1e-4);
  EXPECT_V3_NEAR(Euler3(axis_angle_yxz, eEulerOrder::YXZ).ijk(), ijk, 1e-4);
  EXPECT_V3_NEAR(Euler3(axis_angle_yzx, eEulerOrder::YZX).ijk(), ijk, 1e-4);
  EXPECT_V3_NEAR(Euler3(axis_angle_zxy, eEulerOrder::ZXY).ijk(), ijk, 1e-4);
  EXPECT_V3_NEAR(Euler3(axis_angle_zyx, eEulerOrder::ZYX).ijk(), ijk, 1e-4);
}

TEST(math_rotation_types, AngleSinCosOperators)
{
  AngleSinCos(M_PI_2);
  EXPECT_NEAR((AngleSinCos(M_PI_2) + AngleSinCos(M_PI)).radian(),
              AngleRadian(M_PI_2 + M_PI).wrapped().radian(),
              1e-4);
  EXPECT_NEAR((AngleSinCos(M_PI_2) - AngleSinCos(M_PI)).radian(),
              AngleRadian(M_PI_2 - M_PI).wrapped().radian(),
              1e-4);
  EXPECT_NEAR((-AngleSinCos(M_PI_2)).radian(), AngleRadian(-M_PI_2).radian(), 1e-4);
  EXPECT_NEAR((AngleSinCos(M_PI_4) * 2).radian(), AngleRadian(M_PI_4 * 2).radian(), 1e-4);
  EXPECT_NEAR((AngleSinCos(M_PI_4) * 3).radian(), AngleRadian(M_PI_4 * 3).radian(), 1e-4);
  EXPECT_NEAR((AngleSinCos(-M_PI_4) * 2).radian(), AngleRadian(-M_PI_4 * 2).radian(), 1e-4);
  EXPECT_NEAR((AngleSinCos(-M_PI_4) * 3).radian(), AngleRadian(-M_PI_4 * 3).radian(), 1e-4);
  EXPECT_NEAR((AngleSinCos(M_PI_4) / 2).radian(), AngleRadian(M_PI_4 / 2).radian(), 1e-4);
  EXPECT_NEAR((AngleSinCos(-M_PI_4) / 2).radian(), AngleRadian(-M_PI_4 / 2).radian(), 1e-4);
}

}  // namespace blender::tests
