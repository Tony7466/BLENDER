/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 */

#include "BLI_math_euler_types.hh"
#include "BLI_math_quaternion_types.hh"

#include "BLI_math_base.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_quaternion.hh"

namespace blender::math::detail {

/* -------------------------------------------------------------------- */
/** \name EulerXYZ
 * \{ */

template<typename T> EulerXYZ<T>::operator Quaternion<T>() const
{
  const EulerXYZ<T> &eul = *this;
  const T ti = eul.x() * T(0.5);
  const T tj = eul.y() * T(0.5);
  const T th = eul.z() * T(0.5);
  const T ci = math::cos(ti);
  const T cj = math::cos(tj);
  const T ch = math::cos(th);
  const T si = math::sin(ti);
  const T sj = math::sin(tj);
  const T sh = math::sin(th);
  const T cc = ci * ch;
  const T cs = ci * sh;
  const T sc = si * ch;
  const T ss = si * sh;

  Quaternion<T> quat;
  quat.w = cj * cc + sj * ss;
  quat.x = cj * sc - sj * cs;
  quat.y = cj * ss + sj * cc;
  quat.z = cj * cs - sj * sc;
  return quat;
}

template<typename T> EulerXYZ<T> EulerXYZ<T>::wrapped_around(const EulerXYZ &reference) const
{
  using Vec3T = VecBase<T, 3>;

  constexpr T m_2pi = T(2 * M_PI);
  Vec3T result(*this);
  Vec3T delta = result - Vec3T(reference);
  unroll<3>([&](auto i) {
    /* NOTE(campbell) We could use M_PI as pi_threshold: which is correct but 5.1 gives better
     * results. Checked with baking actions to fcurves. */
    constexpr T pi_threshold = T(5.1);
    if (abs(delta[i]) > pi_threshold) {
      /* Correct differences of about 360 degrees first. */
      result[i] += sign(-delta[i]) * floor((abs(delta[i]) / m_2pi) + T(0.5)) * m_2pi;
    }
  });
  delta = result - Vec3T(reference);

  /* Is 1 of the axis rotations larger than 180 degrees and the other small? NO ELSE IF!! */
  if (abs(delta.x) > T(3.2) && abs(delta.y) < T(1.6) && abs(delta.z) < T(1.6)) {
    result.x -= sign(delta.x) * m_2pi;
  }
  if (abs(delta.y) > T(3.2) && abs(delta.z) < T(1.6) && abs(delta.x) < T(1.6)) {
    result.y -= sign(delta.y) * m_2pi;
  }
  if (abs(delta.z) > T(3.2) && abs(delta.x) < T(1.6) && abs(delta.y) < T(1.6)) {
    result.z -= sign(delta.z) * m_2pi;
  }
  return result;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Euler3
 * \{ */

template<typename T>
Euler3<T>::Euler3(const Quaternion<T> &quat, eEulerOrder order) : order_(order)
{
  using Mat3T = MatBase<T, 3, 3>;
  BLI_assert(is_unit_scale(quat));
  Mat3T unit_mat = math::from_rotation<Mat3T>(quat);
  *this = math::to_euler<T, true>(unit_mat, this->order_);
}

template<typename T>
Euler3<T>::Euler3(const AxisAngle<T, AngleRadian<T>> &axis_angle, eEulerOrder order)
{
  /* Use quaternions as intermediate representation for now... */
  *this = Euler3(Quaternion<T>(axis_angle), order);
}

template<typename T> Euler3<T>::operator Quaternion<T>() const
{
  const Euler3<T> &eulO = *this;
  /* Swizzle to XYZ. */
  EulerXYZ<T> eul_xyz{eulO.x(), eulO.parity() ? -eulO.y() : eulO.y(), eulO.z()};
  /* Quaternion conversion. */
  Quaternion<T> quat{eul_xyz};
  /* Swizzle back from XYZ. */
  VecBase<T, 3> quat_xyz;
  quat_xyz[eulO.x_index()] = quat.x;
  quat_xyz[eulO.y_index()] = eulO.parity() ? -quat.y : quat.y;
  quat_xyz[eulO.z_index()] = quat.z;

  return {quat.w, UNPACK3(quat_xyz)};
}

/** \} */

}  // namespace blender::math::detail

/** \} */
