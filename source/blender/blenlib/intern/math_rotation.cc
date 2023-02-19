/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bli
 */

#include "BLI_math_base.h"
#include "BLI_math_matrix.hh"
#include "BLI_math_rotation.hh"
#include "BLI_math_rotation_legacy.hh"
#include "BLI_math_vector.h"
#include "BLI_math_vector.hh"

namespace blender::math::detail {

template AxisAngle<float, AngleRadian<float>>::operator EulerXYZ<float>() const;
template AxisAngle<float, AngleSinCos<float>>::operator EulerXYZ<float>() const;
template AxisAngle<float, AngleRadian<float>>::operator Quaternion<float>() const;
template AxisAngle<float, AngleSinCos<float>>::operator Quaternion<float>() const;
template EulerXYZ<float>::operator Quaternion<float>() const;
template Euler3<float>::operator Quaternion<float>() const;
template Quaternion<float>::operator EulerXYZ<float>() const;

#if 0 /* Only for reference. */
void generate_axes_to_quaternion_switch_cases()
{
  std::cout << "default: *this = identity(); break;" << std::endl;
  /* Go through all 32 cases. Only 23 valid and 1 is identity. */
  for (int i : IndexRange(6)) {
    for (int j : IndexRange(6)) {
      const eAxisSigned forward = eAxisSigned(i);
      const eAxisSigned up = eAxisSigned(j);
      /* Filter the 12 invalid cases. Fall inside the default case. */
      if (axis_unsigned(forward) == axis_unsigned(up)) {
        continue;
      }
      /* Filter the identity case. Fall inside the default case. */
      if (forward == eAxisSigned::Y_POS && up == eAxisSigned::Z_POS) {
        continue;
      }

      VecBase<eAxisSigned, 3> axes{cross(forward, up), forward, up};

      float3x3 mat;
      mat.x_axis() = basis_vector<float>(axes.x);
      mat.y_axis() = basis_vector<float>(axes.y);
      mat.z_axis() = basis_vector<float>(axes.z);

      math::Quaternion q = to_quaternion(mat);
      /* Create a integer value out of the 4 possible component values (+sign). */
      int4 p = int4(round(sign(float4(q)) * min(pow(float4(q), 2.0f), float4(0.75)) * 4.0));

      auto format_component = [](int value) {
        switch (abs(value)) {
          default:
          case 0:
            return "T(0)";
          case 1:
            return (value > 0) ? "T(0.5)" : "T(-0.5)";
          case 2:
            return (value > 0) ? "T(M_SQRT1_2)" : "T(-M_SQRT1_2)";
          case 3:
            return (value > 0) ? "T(1)" : "T(-1)";
        }
      };
      auto format_axis = [](eAxisSigned axis) {
        switch (axis) {
          default:
          case eAxisSigned::X_POS:
            return "eAxisSigned::X_POS";
          case eAxisSigned::Y_POS:
            return "eAxisSigned::Y_POS";
          case eAxisSigned::Z_POS:
            return "eAxisSigned::Z_POS";
          case eAxisSigned::X_NEG:
            return "eAxisSigned::X_NEG";
          case eAxisSigned::Y_NEG:
            return "eAxisSigned::Y_NEG";
          case eAxisSigned::Z_NEG:
            return "eAxisSigned::Z_NEG";
        }
      };
      /* Use same code function as in the switch case. */
      std::cout << "case ";
      std::cout << format_axis(axes.x) << " << 16 | ";
      std::cout << format_axis(axes.y) << " << 8 | ";
      std::cout << format_axis(axes.z);
      std::cout << ": *this = {";
      std::cout << format_component(p.x) << ", ";
      std::cout << format_component(p.y) << ", ";
      std::cout << format_component(p.z) << ", ";
      std::cout << format_component(p.w) << "}; break;";
      std::cout << std::endl;
    }
  }
}
#endif

}  // namespace blender::math::detail

namespace blender::math {

float3 rotate_direction_around_axis(const float3 &direction, const float3 &axis, const float angle)
{
  BLI_ASSERT_UNIT_V3(direction);
  BLI_ASSERT_UNIT_V3(axis);

  const float3 axis_scaled = axis * math::dot(direction, axis);
  const float3 diff = direction - axis_scaled;
  const float3 cross = math::cross(axis, diff);

  return axis_scaled + diff * std::cos(angle) + cross * std::sin(angle);
}

float3 rotate_around_axis(const float3 &vector,
                          const float3 &center,
                          const float3 &axis,
                          const float angle)

{
  float3 result = vector - center;
  float mat[3][3];
  axis_angle_normalized_to_mat3(mat, axis, angle);
  mul_m3_v3(mat, result);
  return result + center;
}

}  // namespace blender::math
