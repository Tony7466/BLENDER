/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 */

#include "BLI_math_axis_angle_types.hh"
#include "BLI_math_euler_types.hh"

#include "BLI_math_matrix.hh"
#include "BLI_math_quaternion.hh"

namespace blender::math::detail {

/* -------------------------------------------------------------------- */
/** \name Constructors
 * \{ */

template<typename T, typename AngleT>
AxisAngle<T, AngleT>::AxisAngle(const VecBase<T, 3> &axis, const AngleT &angle)
{
  /* TODO: After merge to limit side effects. */
  // BLI_assert(is_unit_scale(axis));
  // axis_ = axis;
  T length;
  this->axis_ = math::normalize_and_get_length(axis, length);
  this->angle_ = angle;
}

template<typename T, typename AngleT>
AxisAngle<T, AngleT>::AxisAngle(const eAxisSigned axis, const AngleT &angle)
{
  this->axis_ = basis_vector<T>(axis);
  this->angle_ = angle;
}

template<typename T, typename AngleT>
AxisAngle<T, AngleT>::AxisAngle(const VecBase<T, 3> &from, const VecBase<T, 3> &to)
{
  BLI_assert(is_unit_scale(from));
  BLI_assert(is_unit_scale(to));

  T sin;
  T cos = dot(from, to);
  this->axis_ = normalize_and_get_length(cross(from, to), sin);

  if (sin <= FLT_EPSILON) {
    if (cos > T(0)) {
      /* Same vectors, zero rotation... */
      *this = identity();
      return;
    }
    else {
      /* Colinear but opposed vectors, 180 rotation... */
      axis_ = normalize(orthogonal(from));
      sin = T(0);
      cos = T(-1);
    }
  }
  /* Avoid calculating the angle if possible. */
  this->angle_ = AngleT(cos, sin);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Conversion
 * \{ */

/* -------------- Quaternion -------------- */

template<typename T, typename AngleT> AxisAngle<T, AngleT>::AxisAngle(const Quaternion<T> &quat)
{
  BLI_assert(is_unit_scale(quat));

  /* Calculate angle/2, and sin(angle/2). */
  T ha = math::acos(quat.w);
  T si = math::sin(ha);

  /* From half-angle to angle. */
  T angle = ha * 2;
  /* Prevent division by zero for axis conversion. */
  if (math::abs(si) < 0.0005) {
    si = 1.0f;
  }

  VecBase<T, 3> axis = VecBase<T, 3>(quat.x, quat.y, quat.z) / si;
  if (math::is_zero(axis)) {
    axis[1] = 1.0f;
  }
  *this = AxisAngle<T, AngleT>(axis, angle);
}

template<typename T, typename AngleT> AxisAngle<T, AngleT>::operator Quaternion<T>() const
{
  BLI_assert(math::is_unit_scale(axis_));

  AngleT half_angle = angle() / T(2);
  T hs = half_angle.sin();
  T hc = half_angle.cos();

  Quaternion<T> quat;
  quat.w = hc;
  quat.x = axis_.x * hs;
  quat.y = axis_.y * hs;
  quat.z = axis_.z * hs;
  return quat;
}

/* -------------- EulerXYZ -------------- */

template<typename T, typename AngleT> AxisAngle<T, AngleT>::AxisAngle(const EulerXYZ<T> &euler)
{
  /* Use quaternions as intermediate representation for now... */
  *this = AxisAngle<T, AngleT>(Quaternion<T>(euler));
}

template<typename T, typename AngleT> AxisAngle<T, AngleT>::operator EulerXYZ<T>() const
{
  /* Check easy and exact conversions first. */
  if (axis_.x == T(1)) {
    return EulerXYZ<T>(T(angle()), T(0), T(0));
  }
  else if (axis_.y == T(1)) {
    return EulerXYZ<T>(T(0), T(angle()), T(0));
  }
  else if (axis_.z == T(1)) {
    return EulerXYZ<T>(T(0), T(0), T(angle()));
  }
  /* Use quaternions as intermediate representation for now... */
  return EulerXYZ<T>(Quaternion<T>(*this));
}

/* -------------- Euler3 -------------- */

template<typename T, typename AngleT> AxisAngle<T, AngleT>::AxisAngle(const Euler3<T> &euler)
{
  /* Use quaternions as intermediate representation for now... */
  *this = AxisAngle<T, AngleT>(Quaternion<T>(euler));
}

/** \} */

}  // namespace blender::math::detail

/** \} */
