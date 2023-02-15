/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 */

#include "BLI_math_matrix.hh"
#include "BLI_math_rotation_types.hh"
#include "BLI_math_vector.hh"

namespace blender::math {

#ifdef DEBUG
#  define BLI_ASSERT_UNIT_QUATERNION(_q) \
    { \
      auto rot_vec = static_cast<VecBase<T, 4>>(_q); \
      T quat_length = math::length_squared(rot_vec); \
      if (!(quat_length == 0 || (math::abs(quat_length - 1) < 0.0001))) { \
        std::cout << "Warning! " << __func__ << " called with non-normalized quaternion: size " \
                  << quat_length << " *** report a bug ***\n"; \
      } \
    }
#else
#  define BLI_ASSERT_UNIT_QUATERNION(_q)
#endif

/**
 * Generic function for implementing slerp
 * (quaternions and spherical vector coords).
 *
 * \param t: factor in [0..1]
 * \param cosom: dot product from normalized vectors/quats.
 * \param r_w: calculated weights.
 */
template<typename T>
[[nodiscard]] inline VecBase<T, 2> interpolate_dot_slerp(const T t, const T cosom)
{
  const T eps = T(1e-4);

  BLI_assert(IN_RANGE_INCL(cosom, T(-1.0001), T(1.0001)));

  VecBase<T, 2> w;
  /* Within [-1..1] range, avoid aligned axis. */
  if (LIKELY(math::abs(cosom) < (T(1) - eps))) {
    const T omega = math::acos(cosom);
    const T sinom = math::sin(omega);

    w[0] = math::sin((T(1) - t) * omega) / sinom;
    w[1] = math::sin(t * omega) / sinom;
  }
  else {
    /* Fallback to lerp */
    w[0] = T(1) - t;
    w[1] = t;
  }
  return w;
}

template<typename T>
[[nodiscard]] inline detail::Quaternion<T> interpolate(const detail::Quaternion<T> &a,
                                                       const detail::Quaternion<T> &b,
                                                       T t)
{
  using Vec4T = VecBase<T, 4>;
  BLI_assert(is_unit_scale(Vec4T(a)));
  BLI_assert(is_unit_scale(Vec4T(b)));

  Vec4T quat = Vec4T(a);
  T cosom = dot(Vec4T(a), Vec4T(b));
  /* Rotate around shortest angle. */
  if (cosom < T(0)) {
    cosom = -cosom;
    quat = -quat;
  }

  VecBase<T, 2> w = interpolate_dot_slerp(t, cosom);

  return detail::Quaternion<T>(w[0] * quat + w[1] * Vec4T(b));
}

/**
 * Dot product between two quaternions.
 * Equivalent to vector dot product.
 * Equivalent to component wise multiplication followed by summation of the result.
 */
template<typename T>
[[nodiscard]] inline T dot(const detail::Quaternion<T> &a, const detail::Quaternion<T> &b)
{
  return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

/**
 * Return the conjugate of the given quaternion.
 * If the quaternion \a q represent the rotation from A to B,
 * then the conjugate of \a q represents the rotation from B to A.
 */
template<typename T>
[[nodiscard]] inline detail::Quaternion<T> conjugate(const detail::Quaternion<T> &a)
{
  return {a.w, -a.x, -a.y, -a.z};
}

/**
 * Return invert of \a q or identity if \a q is ill-formed.
 * The invert allows quaternion division.
 * \note The inverse of \a q isn't the opposite rotation. This would be the conjugate.
 */
template<typename T>
[[nodiscard]] inline detail::Quaternion<T> invert(const detail::Quaternion<T> &q)
{
  const T length_squared = dot(q, q);
  if (length_squared == 0.0f) {
    return detail::Quaternion<T>::identity();
  }
  return conjugate(q) * (1.0f / length_squared);
}
/**
 * Return invert of \a q assuming it is a unit quaternion.
 * In this case, the inverse is just the conjugate. `conjugate(q)` could be use directly,
 * but this function shows the intent better, and asserts if \a q ever becomes non-unit-length.
 */
template<typename T>
[[nodiscard]] inline detail::Quaternion<T> invert_normalized(const detail::Quaternion<T> &q)
{
  BLI_ASSERT_UNIT_QUATERNION(q);
  return conjugate(q);
}

/**
 * Return a unit quaternion representing the same rotation as \a q or
 * the identity quaternion if \a q is ill-formed.
 */
template<typename T>
[[nodiscard]] inline detail::Quaternion<T> normalize_and_get_length(const detail::Quaternion<T> &q,
                                                                    T &out_length)
{
  out_length = math::sqrt(dot(q, q));
  return (out_length != T(0)) ? (q * (T(1) / out_length)) : detail::Quaternion<T>::identity();
}

template<typename T>
[[nodiscard]] inline detail::Quaternion<T> normalize(const detail::Quaternion<T> &q)
{
  T len;
  return normalize_and_get_length(q, len);
}

/**
 * Returns true if all components are exactly equal to 0.
 */
template<typename T> [[nodiscard]] inline bool is_zero(const detail::Quaternion<T> &q)
{
  return q.w == T(0) && q.x == T(0) && q.y == T(0) && q.z == T(0);
}

/**
 * Transform \a p by rotation using the quaternion \a q .
 */
template<typename T>
[[nodiscard]] inline VecBase<T, 3> rotate(const detail::Quaternion<T> &q, const VecBase<T, 3> &v)
{
#if 0 /* Reference. */
  detail::Quaternion<T> V(T(0), UNPACK3(v));
  detail::Quaternion<T> R = q * V * conjugate(q);
  return {R.x, R.y, R.z};
#else
  /* `S = q * V` */
  detail::Quaternion<T> S;
  S.w = /* q.w * 0.0  */ -q.x * v.x - q.y * v.y - q.z * v.z;
  S.x = q.w * v.x /* + q.x * 0.0 */ + q.y * v.z - q.z * v.y;
  S.y = q.w * v.y /* + q.y * 0.0 */ + q.z * v.x - q.x * v.z;
  S.z = q.w * v.z /* + q.z * 0.0 */ + q.x * v.y - q.y * v.x;
  /* `R = S * conjugate(q)` */
  VecBase<T, 3> R;
  /* R.w = S.w * q.w + S.x * q.x + S.y * q.y + S.z * q.z = 0.0; */
  R.x = S.w * -q.x + S.x * q.w - S.y * q.z + S.z * q.y;
  R.y = S.w * -q.y + S.y * q.w - S.z * q.x + S.x * q.z;
  R.z = S.w * -q.z + S.z * q.w - S.x * q.y + S.y * q.x;
  return R;
#endif
}

}  // namespace blender::math

/* -------------------------------------------------------------------- */
/** \name Template implementations
 * \{ */

namespace blender::math::detail {

template<typename T> AxisAngle<T>::AxisAngle(const VecBase<T, 3> &axis, T angle)
{
  T length;
  axis_ = math::normalize_and_get_length(axis, length);
  if (length > 0.0f) {
    angle_cos_ = math::cos(angle);
    angle_sin_ = math::sin(angle);
    angle_ = angle;
  }
  else {
    *this = identity();
  }
}

template<typename T> AxisAngle<T>::AxisAngle(const VecBase<T, 3> &from, const VecBase<T, 3> &to)
{
  BLI_assert(is_unit_scale(from));
  BLI_assert(is_unit_scale(to));

  /* Avoid calculating the angle. */
  angle_cos_ = dot(from, to);
  axis_ = normalize_and_get_length(cross(from, to), angle_sin_);

  if (angle_sin_ <= FLT_EPSILON) {
    if (angle_cos_ > T(0)) {
      /* Same vectors, zero rotation... */
      *this = identity();
    }
    else {
      /* Colinear but opposed vectors, 180 rotation... */
      axis_ = normalize(orthogonal(from));
      angle_sin_ = T(0);
      angle_cos_ = T(-1);
    }
  }
}
template<typename T>
AxisAngleNormalized<T>::AxisAngleNormalized(const VecBase<T, 3> &axis, T angle) : AxisAngle<T>()
{
  BLI_assert(is_unit_scale(axis));
  this->axis_ = axis;
  this->angle_ = angle;
  this->angle_cos_ = math::cos(angle);
  this->angle_sin_ = math::sin(angle);
}

template<typename T> Quaternion<T>::operator EulerXYZ<T>() const
{
  using Mat3T = MatBase<T, 3, 3>;
  const Quaternion<T> &quat = *this;
  BLI_ASSERT_UNIT_QUATERNION(quat)
  Mat3T unit_mat = math::from_rotation<Mat3T>(quat);
  return math::to_euler<T, true>(unit_mat);
}

template<typename T> EulerXYZ<T>::operator Quaternion<T>() const
{
  const EulerXYZ<T> &eul = *this;
  const T ti = eul.x * T(0.5);
  const T tj = eul.y * T(0.5);
  const T th = eul.z * T(0.5);
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

template<typename T> Quaternion<T>::operator AxisAngle<T>() const
{
  const Quaternion<T> &quat = *this;
  BLI_ASSERT_UNIT_QUATERNION(quat)

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
  return AxisAngleNormalized<T>(axis, angle);
}

template<typename T> AxisAngle<T>::operator Quaternion<T>() const
{
  BLI_assert(math::is_unit_scale(axis_));

  /** Using half angle identities: sin(angle / 2) = sqrt((1 - angle_cos) / 2) */
  T sine = math::sqrt(T(0.5) - angle_cos_ * T(0.5));
  const T cosine = math::sqrt(T(0.5) + angle_cos_ * T(0.5));

  if (angle_sin_ < 0.0) {
    sine = -sine;
  }

  Quaternion<T> quat;
  quat.w = cosine;
  quat.x = axis_.x * sine;
  quat.y = axis_.y * sine;
  quat.z = axis_.z * sine;
  return quat;
}

template<typename T> EulerXYZ<T>::operator AxisAngle<T>() const
{
  /* Use quaternions as intermediate representation for now... */
  return AxisAngle<T>(Quaternion<T>(*this));
}

template<typename T> AxisAngle<T>::operator EulerXYZ<T>() const
{
  /* Use quaternions as intermediate representation for now... */
  return EulerXYZ<T>(Quaternion<T>(*this));
}

/* Using explicit template instantiations in order to reduce compilation time. */
extern template AxisAngle<float>::operator EulerXYZ<float>() const;
extern template AxisAngle<float>::operator Quaternion<float>() const;
extern template EulerXYZ<float>::operator AxisAngle<float>() const;
extern template EulerXYZ<float>::operator Quaternion<float>() const;
extern template Quaternion<float>::operator AxisAngle<float>() const;
extern template Quaternion<float>::operator EulerXYZ<float>() const;

extern template AxisAngle<double>::operator EulerXYZ<double>() const;
extern template AxisAngle<double>::operator Quaternion<double>() const;
extern template EulerXYZ<double>::operator AxisAngle<double>() const;
extern template EulerXYZ<double>::operator Quaternion<double>() const;
extern template Quaternion<double>::operator AxisAngle<double>() const;
extern template Quaternion<double>::operator EulerXYZ<double>() const;

}  // namespace blender::math::detail

/** \} */
