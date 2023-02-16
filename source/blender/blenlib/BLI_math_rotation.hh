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

template<typename T>
[[nodiscard]] inline detail::Quaternion<T> canonicalize(const detail::Quaternion<T> &q)
{
  return (q.w < 0.0) ? -q : q;
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

/**
 * Return rotation from orientation \a a  to orientation \a b into another quaternion.
 */
template<typename T>
[[nodiscard]] detail::Quaternion<T> rotation_between(const detail::Quaternion<T> &a,
                                                     const detail::Quaternion<T> &b)
{
  return invert(a) * b;
}

/**
 * Extract rotation angle from a unit quaternion.
 * Returned angle is in [0..2pi] range.
 *
 * Unlike the angle between vectors, this does *NOT* return the shortest angle.
 * See `angle_of_signed` below for this.
 */
template<typename T> [[nodiscard]] detail::AngleRadian<T> angle_of(const detail::Quaternion<T> &q)
{
  BLI_ASSERT_UNIT_QUATERNION(q);
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
  BLI_ASSERT_UNIT_QUATERNION(q);
  return T(2) * ((q.w >= 0.0f) ? math::safe_acos(q.w) : -math::safe_acos(-q.w));
}

/**
 * Extract angle between 2 orientations.
 * Returned angle is in [0..2pi] range.
 * See `angle_of` for more detail.
 */
template<typename T>
[[nodiscard]] detail::AngleRadian<T> angle_between(const detail::Quaternion<T> &a,
                                                   const detail::Quaternion<T> &b)
{
  return angle_of(rotation_between(a, b));
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

/**
 * Create a rotation from a triangle geometry.
 * Takes pre-computed \a normal from the triangle.
 * Used for Ngons when we know their normal.
 */
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
  Vec3T line = rotate(conjugate(q1), (v2 - v1));
  /* What angle has this line with x-axis? */
  line = normalize(Vec3T(line.x, line.y, T(0)));

  angle = T(0.5) * math::atan2(line.y, line.x);
  detail::Quaternion<T> q2(math::cos(angle), 0.0, 0.0, math::sin(angle));

  return q1 * q2;
}

/**
 * Create a rotation from a triangle geometry.
 */
template<typename T>
[[nodiscard]] detail::Quaternion<T> from_triangle(const VecBase<T, 3> &v1,
                                                  const VecBase<T, 3> &v2,
                                                  const VecBase<T, 3> &v3)
{
  return from_triangle(v1, v2, v3, normal_tri(v1, v2, v3));
}

template<typename T>
[[nodiscard]] detail::Quaternion<T> from_vector(const VecBase<T, 3> &vector,
                                                /* #Object.trackflag / #Object.upflag (short) */
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

  detail::Quaternion<T> q1(detail::AxisAngle<T>(rotation_axis, rotation_angle));

  if (axis == up_flag) {
    /* Nothing else to do. */
    return q1;
  }

  /* Extract rotation between the up axis of the rotated space and the up axis. */
  /* There might be an easier way to get this angle directly from the quaternion representation. */
  Vec3T rotated_up = rotate(q1, Vec3T(0, 0, 1));

  /* Project using axes index instead of arithmetic. It's much faster and more precise. */
  eAxisSigned y_axis_signed = cross(eAxisSigned(axis), eAxisSigned(up_flag));
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
  /* Some trigonometry identity to dodge `atan`.
   * https://en.wikipedia.org/wiki/List_of_trigonometric_identities#Half-angle_formulae */
  T cos_angle = projected.x;
  T cos_half_angle = math::sqrt((T(1) + cos_angle) / T(2));
  T sin_half_angle = math::sqrt((T(1) - cos_angle) / T(2));
  /* Recover sign for sine. Cosine of half angle is given to be positive or 0. */
  /* TODO(fclem): Could use copysign here. */
  if (projected.y < T(0)) {
    sin_half_angle = -sin_half_angle;
  }

  detail::Quaternion<T> q2(Vec4T(cos_half_angle, vec * (sin_half_angle / vec_len)));

  return q2 * q1;
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

template<typename MatT>
[[nodiscard]] MatT from_axis_conversion(const eAxisSigned src_forward,
                                        const eAxisSigned src_up,
                                        const eAxisSigned dst_forward,
                                        const eAxisSigned dst_up)
{
  return MatT(from_axis_conversion<float3x3>(src_forward, src_up, dst_forward, dst_up));
}

template<>
float3x3 from_axis_conversion(const eAxisSigned src_forward,
                              const eAxisSigned src_up,
                              const eAxisSigned dst_forward,
                              const eAxisSigned dst_up);

template<typename MatT>
[[nodiscard]] MatT from_axis_conversion(const eAxisSigned src_axis, const eAxisSigned dst_axis)
{
  /* Pick predictable next axis. */
  eAxisSigned src_axis_next = eAxisSigned((src_axis + 1) % 3);
  eAxisSigned dst_axis_next = eAxisSigned((dst_axis + 1) % 3);

  if (is_negative(src_axis) != is_negative(dst_axis)) {
    /* Flip both axis so matrix sign remains positive. */
    dst_axis_next = eAxisSigned(dst_axis_next + 3);
  }

  return from_axis_conversion<MatT>(src_axis, src_axis_next, dst_axis, dst_axis_next);
}

}  // namespace blender::math

/* -------------------------------------------------------------------- */
/** \name Template implementations
 * \{ */

namespace blender::math::detail {

template<typename T> Quaternion<T> Quaternion<T>::expmap(const VecBase<T, 3> &expmap)
{
  /* Obtain axis/angle representation. */
  T angle;
  VecBase<T, 3> axis = normalize_and_get_length(expmap, angle);
  if (LIKELY(angle != 0.0f)) {
    return Quaternion<T>(AxisAngle(axis, angle_wrap_rad(angle)));
  }
  return Quaternion<T>::identity();
}

template<typename T> VecBase<T, 3> Quaternion<T>::expmap() const
{
  BLI_ASSERT_UNIT_QUATERNION(*this);
  AxisAngle<T> axis_angle(*this);
  return axis_angle.axis() * axis_angle.angle();
}

template<typename T> Quaternion<T> Quaternion<T>::wrapped_around(const Quaternion &reference) const
{
  const Quaternion<T> &input = *this;
  BLI_ASSERT_UNIT_QUATERNION(input);
  T len;
  Quaternion<T> reference_normalized = normalize_and_get_length(reference, len);
  /* Skips cases case too. */
  if (len < 1e-4f) {
    return input;
  }
  Quaternion<T> result = reference * rotation_between(reference_normalized, input);
  return (distance_squared(VecBase<T, 4>(-result), VecBase<T, 4>(reference)) <
          distance_squared(VecBase<T, 4>(result), VecBase<T, 4>(reference))) ?
             -result :
             result;
}

template<typename T> AngleRadian<T> Quaternion<T>::twist_angle(const eAxis axis) const
{
  BLI_assert(axis >= 0 && axis <= 2);
  /* The calculation requires a canonical quaternion. */
  const VecBase<T, 4> input_vec(canonicalize(*this));
  return AngleRadian<T>(T(2) * math::atan2(input_vec[axis + 1], input_vec[0]));
}

template<typename T>
Quaternion<T> Quaternion<T>::twist(const eAxis axis, AngleRadian<T> &r_twist_angle) const
{
  BLI_assert(axis >= 0 && axis <= 2);
  /* The calculation requires a canonical quaternion. */
  const Quaternion<T> input = canonicalize(*this);
  const VecBase<T, 4> input_vec(input);

  T half_twist_angle = math::atan2(input_vec[axis + 1], input_vec[0]);
  r_twist_angle = T(2) * half_twist_angle;

  /* Compute swing by multiplying the original quaternion by inverted twist. */
  VecBase<T, 4> twist_inv(math::cos(half_twist_angle), T(0), T(0), T(0));
  twist_inv[axis + 1] = -math::sin(half_twist_angle);
  Quaternion<T> result = input * Quaternion<T>(twist_inv);

  BLI_assert(math::abs(VecBase<T, 4>(result)[axis + 1]) < BLI_ASSERT_UNIT_EPSILON);
  return result;
}

template<typename T> Quaternion<T> Quaternion<T>::twist(const eAxis axis) const
{
  AngleRadian<T> twist_angle;
  return twist(axis, twist_angle);
}

template<typename T>
Quaternion<T> Quaternion<T>::swing(const eAxis axis, AngleRadian<T> &r_twist_angle) const
{
  BLI_assert(axis >= 0 && axis <= 2);
  /* The calculation requires a canonical quaternion. */
  const VecBase<T, 4> input_vec(canonicalize(*this));

  T half_twist_angle = math::atan2(input_vec[axis + 1], input_vec[0]);
  r_twist_angle = T(2) * half_twist_angle;

  VecBase<T, 4> twist(math::cos(half_twist_angle), T(0), T(0), T(0));
  twist[axis + 1] = math::sin(half_twist_angle);
  return Quaternion<T>(twist);
}

template<typename T> Quaternion<T> Quaternion<T>::swing(const eAxis axis) const
{
  AngleRadian<T> twist_angle;
  return swing(axis, twist_angle);
}

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

template<typename T> Euler3<T> &Euler3<T>::operator=(const Quaternion<T> &quat)
{
  using Mat3T = MatBase<T, 3, 3>;
  BLI_ASSERT_UNIT_QUATERNION(quat)
  Mat3T unit_mat = math::from_rotation<Mat3T>(quat);
  *this = math::to_euler<T, true>(unit_mat, this->order_);
  return *this;
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

template<typename T> Euler3<T>::operator AxisAngle<T>() const
{
  /* Use quaternions as intermediate representation for now... */
  return AxisAngle<T>(Quaternion<T>(*this));
}

template<typename T> Euler3<T> &Euler3<T>::operator=(const AxisAngle<T> &axis_angle)
{
  /* Use quaternions as intermediate representation for now... */
  return (*this = Quaternion<T>(axis_angle));
}

/* Using explicit template instantiations in order to reduce compilation time. */
extern template AxisAngle<float>::operator EulerXYZ<float>() const;
extern template AxisAngle<float>::operator Quaternion<float>() const;
extern template EulerXYZ<float>::operator AxisAngle<float>() const;
extern template EulerXYZ<float>::operator Quaternion<float>() const;
extern template Euler3<float>::operator AxisAngle<float>() const;
extern template Euler3<float>::operator Quaternion<float>() const;
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
