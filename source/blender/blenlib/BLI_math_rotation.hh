/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 */

#include "BLI_math_rotation_types.hh"

#include "BLI_math_axis_angle.hh"
#include "BLI_math_euler.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_quaternion.hh"
#include "BLI_math_vector.hh"

namespace blender::math {

/* -------------------------------------------------------------------- */
/** \name Rotation helpers
 * \{ */

/**
 * Rotate \a a by \a b. In other word, insert the \a b rotation before \a a.
 *
 * \note Since \a a is a #Quaternion it will cast \a b to a #Quaternion.
 * This might introduce some precision loss and have performance implication.
 */
template<typename T, typename RotT>
[[nodiscard]] detail::Quaternion<T> rotate(const detail::Quaternion<T> &a, const RotT &b);

/**
 * Rotate \a a by \a b. In other word, insert the \a b rotation before \a a.
 *
 * \note Since \a a is an #AxisAngle it will cast both \a a and \a b to #Quaternion.
 * This might introduce some precision loss and have performance implication.
 */
template<typename T, typename RotT, typename AngleT>
[[nodiscard]] detail::AxisAngle<T, AngleT> rotate(const detail::AxisAngle<T, AngleT> &a,
                                                  const RotT &b);

/**
 * Rotate \a a by \a b. In other word, insert the \a b rotation before \a a.
 *
 * \note Since \a a is an #EulerXYZ it will cast both \a a and \a b to #MatBase<T, 3, 3>.
 * This might introduce some precision loss and have performance implication.
 */
template<typename T, typename RotT>
[[nodiscard]] detail::EulerXYZ<T> rotate(const detail::EulerXYZ<T> &a, const RotT &b);

/**
 * Rotate \a a by \a b. In other word, insert the \a b rotation before \a a.
 *
 * \note Since \a a is an #Euler3 it will cast both \a a and \a b to #MatBase<T, 3, 3>.
 * This might introduce some precision loss and have performance implication.
 */
template<typename T, typename RotT>
[[nodiscard]] detail::Euler3<T> rotate(const detail::Euler3<T> &a, const RotT &b);

/**
 * Return rotation from orientation \a a  to orientation \a b into another quaternion.
 */
template<typename T>
[[nodiscard]] detail::Quaternion<T> rotation_between(const detail::Quaternion<T> &a,
                                                     const detail::Quaternion<T> &b);

/**
 * Create a orientation from a triangle plane and the axis formed by the segment(v1, v2).
 * Takes pre-computed \a normal from the triangle.
 * Used for Ngons when their normal is known.
 */
template<typename T>
[[nodiscard]] detail::Quaternion<T> from_triangle(const VecBase<T, 3> &v1,
                                                  const VecBase<T, 3> &v2,
                                                  const VecBase<T, 3> &v3,
                                                  const VecBase<T, 3> &normal);

/**
 * Create a orientation from a triangle plane and the axis formed by the segment(v1, v2).
 */
template<typename T>
[[nodiscard]] detail::Quaternion<T> from_triangle(const VecBase<T, 3> &v1,
                                                  const VecBase<T, 3> &v2,
                                                  const VecBase<T, 3> &v3);

/**
 * Create a rotation from a vector and a basis rotation.
 * Used for tracking.
 * \a track_flag is supposed to be #Object.trackflag
 * \a up_flag is supposed to be #Object.upflag
 */
template<typename T>
[[nodiscard]] detail::Quaternion<T> from_vector(const VecBase<T, 3> &vector,
                                                const eAxisSigned track_flag,
                                                const eAxis up_flag);

/**
 * Returns a quaternion for converting local space to tracking space.
 * This is slightly different than from_axis_conversion for legacy reasons.
 */
template<typename T>
[[nodiscard]] detail::Quaternion<T> from_tracking(eAxisSigned forward_axis, eAxis up_axis);

/**
 * Convert euler rotation to gimbal rotation matrix.
 */
template<typename T>
[[nodiscard]] MatBase<T, 3, 3> to_gimbal_axis(const detail::Euler3<T> &rotation);

/** \} */

/* -------------------------------------------------------------------- */
/** \name Angles
 * \{ */

/**
 * Extract rotation angle from a unit quaternion.
 * Returned angle is in [0..2pi] range.
 *
 * Unlike the angle between vectors, this does *NOT* return the shortest angle.
 * See `angle_of_signed` below for this.
 */
template<typename T> [[nodiscard]] detail::AngleRadian<T> angle_of(const detail::Quaternion<T> &q)
{
  BLI_assert(is_unit_scale(q));
  return T(2) * math::safe_acos(q.w);
}

/**
 * Extract rotation angle from a unit quaternion. Always return the shortest angle.
 * Returned angle is in [-pi..pi] range.
 *
 * `angle_of` with quaternion can exceed PI radians. Having signed versions of these functions
 * allows to use 'abs(angle_of_signed(...))' to get the shortest angle between quaternions with
 * higher precision than subtracting 2pi afterwards.
 */
template<typename T>
[[nodiscard]] detail::AngleRadian<T> angle_of_signed(const detail::Quaternion<T> &q)
{
  BLI_assert(is_unit_scale(q));
  return T(2) * ((q.w >= 0.0f) ? math::safe_acos(q.w) : -math::safe_acos(-q.w));
}

/**
 * Extract angle between 2 orientations.
 * For #Quaternion, the returned angle is in [0..2pi] range.
 * For other types, the returned angle is in [0..pi] range.
 * See `angle_of` for more detail.
 */
template<typename T>
[[nodiscard]] detail::AngleRadian<T> angle_between(const detail::Quaternion<T> &a,
                                                   const detail::Quaternion<T> &b)
{
  return angle_of(rotation_between(a, b));
}
template<typename T>
[[nodiscard]] detail::AngleRadian<T> angle_between(const VecBase<T, 3> &a, const VecBase<T, 3> &b)
{
  BLI_assert(is_unit_scale(a));
  BLI_assert(is_unit_scale(b));
  return math::safe_acos(dot(a, b));
}

/**
 * Extract angle between 2 orientations.
 * Returned angle is in [-pi..pi] range.
 * See `angle_of_signed` for more detail.
 */
template<typename T>
[[nodiscard]] detail::AngleRadian<T> angle_between_signed(const detail::Quaternion<T> &a,
                                                          const detail::Quaternion<T> &b)
{
  return angle_of_signed(rotation_between(a, b));
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Template implementations
 * \{ */

template<typename T, typename RotT>
[[nodiscard]] detail::Quaternion<T> rotate(const detail::Quaternion<T> &a, const RotT &b)
{
  return a * detail::Quaternion<T>(b);
}

template<typename T, typename RotT, typename AngleT>
[[nodiscard]] detail::AxisAngle<T, AngleT> rotate(const detail::AxisAngle<T, AngleT> &a,
                                                  const RotT &b)
{
  return detail::AxisAngle<T, AngleT>(detail::Quaternion<T>(a) * detail::Quaternion<T>(b));
}

template<typename T, typename RotT>
[[nodiscard]] detail::EulerXYZ<T> rotate(const detail::EulerXYZ<T> &a, const RotT &b)
{
  MatBase<T, 3, 3> tmp = from_rotation<MatBase<T, 3, 3>>(a) * from_rotation<MatBase<T, 3, 3>>(b);
  return to_euler(tmp);
}

template<typename T, typename RotT>
[[nodiscard]] detail::Euler3<T> rotate(const detail::Euler3<T> &a, const RotT &b)
{
  MatBase<T, 3, 3> tmp = from_rotation<MatBase<T, 3, 3>>(a) * from_rotation<MatBase<T, 3, 3>>(b);
  return to_euler(tmp, a.order());
}

template<typename T>
[[nodiscard]] detail::Quaternion<T> rotation_between(const detail::Quaternion<T> &a,
                                                     const detail::Quaternion<T> &b)
{
  return invert(a) * b;
}

template<typename T>
[[nodiscard]] detail::Quaternion<T> from_triangle(const VecBase<T, 3> &v1,
                                                  const VecBase<T, 3> &v2,
                                                  const VecBase<T, 3> &v3,
                                                  const VecBase<T, 3> &normal)
{
  /* Force to used an unused var to avoid the same function signature as the version without
   * `normal` argument. */
  UNUSED_VARS(v3);

  using Vec3T = VecBase<T, 3>;

  /* Move z-axis to face-normal. */
  Vec3T z_axis = normal;
  Vec3T n = normalize(Vec3T(z_axis.y, -z_axis.x, T(0)));
  if (is_zero(n.xy())) {
    n.x = T(1);
  }

  T angle = T(-0.5) * math::safe_acos(z_axis.z);
  T si = math::sin(angle);
  detail::Quaternion<T> q1(math::cos(angle), n.x * si, n.y * si, T(0));

  /* Rotate back line v1-v2. */
  Vec3T line = transform_point(conjugate(q1), (v2 - v1));
  /* What angle has this line with x-axis? */
  line = normalize(Vec3T(line.x, line.y, T(0)));

  angle = T(0.5) * math::atan2(line.y, line.x);
  detail::Quaternion<T> q2(math::cos(angle), 0.0, 0.0, math::sin(angle));

  return q1 * q2;
}

template<typename T>
[[nodiscard]] detail::Quaternion<T> from_triangle(const VecBase<T, 3> &v1,
                                                  const VecBase<T, 3> &v2,
                                                  const VecBase<T, 3> &v3)
{
  return from_triangle(v1, v2, v3, normal_tri(v1, v2, v3));
}

template<typename T>
[[nodiscard]] detail::Quaternion<T> from_vector(const VecBase<T, 3> &vector,
                                                const eAxisSigned track_flag,
                                                const eAxis up_flag)
{
  using Vec2T = VecBase<T, 2>;
  using Vec3T = VecBase<T, 3>;
  using Vec4T = VecBase<T, 4>;

  BLI_assert(track_flag >= 0 && track_flag <= 5);
  BLI_assert(up_flag >= 0 && up_flag <= 2);

  T vec_len = length(vector);

  if (UNLIKELY(vec_len == 0.0f)) {
    return detail::Quaternion<T>::identity();
  }

  eAxis axis = axis_unsigned(track_flag);
  const Vec3T vec = is_negative(track_flag) ? vector : -vector;

  Vec3T rotation_axis;
  constexpr T eps = T(1e-4);
  T axis_len;
  switch (axis) {
    case eAxis::X:
      rotation_axis = normalize_and_get_length(Vec3T(T(0), -vec.z, vec.y), axis_len);
      if (axis_len < eps) {
        rotation_axis = Vec3T(0, 1, 0);
      }
      break;
    case eAxis::Y:
      rotation_axis = normalize_and_get_length(Vec3T(vec.z, T(0), -vec.x), axis_len);
      if (axis_len < eps) {
        rotation_axis = Vec3T(0, 0, 1);
      }
      break;
    default:
    case eAxis::Z:
      rotation_axis = normalize_and_get_length(Vec3T(-vec.y, vec.x, T(0)), axis_len);
      if (axis_len < eps) {
        rotation_axis = Vec3T(1, 0, 0);
      }
      break;
  }
  /* TODO(fclem): Can optimize here by initializing AxisAngle using the cos an sin directly.
   * Avoiding the need for safe_acos and deriving sin from cos. */
  T rotation_angle = math::safe_acos(vec[axis] / vec_len);

  detail::Quaternion<T> q1(
      detail::AxisAngle<T, detail::AngleRadian<T>>(rotation_axis, rotation_angle));

  if (axis == up_flag) {
    /* Nothing else to do. */
    return q1;
  }

  /* Extract rotation between the up axis of the rotated space and the up axis. */
  /* There might be an easier way to get this angle directly from the quaternion representation. */
  Vec3T rotated_up = transform_point(q1, Vec3T(0, 0, 1));

  /* Project using axes index instead of arithmetic. It's much faster and more precise. */
  eAxisSigned y_axis_signed = math::cross(eAxisSigned(axis), eAxisSigned(up_flag));
  eAxis x_axis = up_flag;
  eAxis y_axis = axis_unsigned(y_axis_signed);

  Vec2T projected = normalize(Vec2T(rotated_up[x_axis], rotated_up[y_axis]));
  /* Flip sign for flipped axis. */
  if (is_negative(y_axis_signed)) {
    projected.y = -projected.y;
  }
  /* Not sure if this was a bug or not in the previous implementation.
   * Carry over this weird behavior to avoid regressions. */
  if (axis == eAxis::Z) {
    projected = -projected;
  }

  detail::AngleCartesian<T> angle(projected.x, projected.y);
  detail::AngleCartesian<T> half_angle = angle / T(2);

  detail::Quaternion<T> q2(Vec4T(half_angle.cos(), vec * (half_angle.sin() / vec_len)));

  return q2 * q1;
}

template<typename T>
[[nodiscard]] detail::Quaternion<T> from_tracking(eAxisSigned forward_axis, eAxis up_axis)
{
  BLI_assert(forward_axis >= eAxisSigned::X_POS && forward_axis <= eAxisSigned::Z_NEG);
  BLI_assert(up_axis >= eAxis::X && up_axis <= eAxis::Z);
  BLI_assert(axis_unsigned(forward_axis) != up_axis);

  /* Curve have Z forward, Y up, X left. */
  return detail::Quaternion<T>(
      rotation_between(from_orthonormal_axes(eAxisSigned::Z_POS, eAxisSigned::Y_POS),
                       from_orthonormal_axes(forward_axis, eAxisSigned(up_axis))));
}

template<typename T>
[[nodiscard]] MatBase<T, 3, 3> to_gimbal_axis(const detail::Euler3<T> &rotation)
{
  using Mat3T = MatBase<T, 3, 3>;
  using Vec3T = VecBase<T, 3>;
  int i = rotation.x_index();
  int j = rotation.y_index();
  int k = rotation.z_index();

  Mat3T result;
  /* First axis is local. */
  result[i] = from_rotation<Mat3T>(rotation)[i];
  /* Second axis is local minus first rotation. */
  detail::Euler3<T> tmp_rot = rotation;
  tmp_rot.x() = T(0);
  result[j] = from_rotation<Mat3T>(tmp_rot)[j];
  /* Last axis is global. */
  result[k] = Vec3T(0);
  result[k][k] = 1;

  return result;
}

/** \} */

}  // namespace blender::math

namespace blender::math::detail {

/* Using explicit template instantiations in order to reduce compilation time. */
extern template AxisAngle<float, AngleRadian<float>>::operator EulerXYZ<float>() const;
extern template AxisAngle<float, AngleCartesian<float>>::operator EulerXYZ<float>() const;
extern template AxisAngle<float, AngleRadian<float>>::operator Quaternion<float>() const;
extern template AxisAngle<float, AngleCartesian<float>>::operator Quaternion<float>() const;
#if 0 /* TODO: Make it compile. */
extern template AxisAngle<float, AngleRadian<float>>::AxisAngle(const EulerXYZ<float> &) const;
extern template AxisAngle<float, AngleCartesian<float>>::AxisAngle(const EulerXYZ<float> &) const;
extern template AxisAngle<float, AngleRadian<float>>::AxisAngle(const Euler3<float> &) const;
extern template AxisAngle<float, AngleCartesian<float>>::AxisAngle(const Euler3<float> &) const;
extern template AxisAngle<float, AngleRadian<float>>::AxisAngle(const Quaternion<float> &) const;
extern template AxisAngle<float, AngleCartesian<float>>::AxisAngle(const Quaternion<float> &) const;
#endif
extern template EulerXYZ<float>::operator Quaternion<float>() const;
extern template Euler3<float>::operator Quaternion<float>() const;
extern template Quaternion<float>::operator EulerXYZ<float>() const;

}  // namespace blender::math::detail

/** \} */
