/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 */

#include "BLI_math_angle_types.hh"
#include "BLI_math_axis_convert_types.hh"
#include "BLI_math_base.hh"
#include "BLI_math_vector_types.hh"

namespace blender::math {

namespace detail {

/* Forward declaration for casting operators. */
template<typename T> struct Euler3;
template<typename T> struct EulerXYZ;
template<typename T> struct Quaternion;

template<typename T, typename AngleT> struct AxisAngle {
  using vec3_type = VecBase<T, 3>;

 protected:
  VecBase<T, 3> axis_ = {0, 1, 0};
  AngleT angle_ = AngleT::identity();

  /**
   * A defaulted constructor would cause zero initialization instead of default initialization,
   * and not call the default member initializers.
   */
  explicit AxisAngle(){};

 public:
  /**
   * Create a rotation from a basis axis and an angle.
   */
  AxisAngle(const eAxisSigned axis, AngleT angle);

  /**
   * Create a rotation from an axis and an angle.
   * \note `axis` have to be normalized.
   */
  AxisAngle(const vec3_type &axis, AngleT angle);

  /**
   * Create a rotation from 2 normalized vectors.
   * \note `from` and `to` must be normalized.
   * \note Consider using `AxisSinCos` for faster conversion to other rotation.
   */
  AxisAngle(const vec3_type &from, const vec3_type &to);

  /** Static functions. */

  static AxisAngle identity()
  {
    return AxisAngle();
  }

  /** Methods. */

  const VecBase<T, 3> &axis() const
  {
    return axis_;
  }

  const AngleT &angle() const
  {
    return angle_;
  }

  /** Conversions. */

  explicit operator Quaternion<T>() const;
  explicit operator EulerXYZ<T>() const;

  explicit AxisAngle(const Quaternion<T> &quat);
  explicit AxisAngle(const EulerXYZ<T> &euler);
  explicit AxisAngle(const Euler3<T> &euler);

  /** Operators. */

  friend bool operator==(const AxisAngle &a, const AxisAngle &b)
  {
    return (a.axis() == b.axis()) && (a.angle() == b.angle());
  }

  friend bool operator!=(const AxisAngle &a, const AxisAngle &b)
  {
    return (a != b);
  }

  friend std::ostream &operator<<(std::ostream &stream, const AxisAngle &rot)
  {
    return stream << "AxisAngle(axis=" << rot.axis() << ", angle=" << rot.angle() << ")";
  }
};

};  // namespace detail

using AxisAngle = math::detail::AxisAngle<float, detail::AngleRadian<float>>;
using AxisSinCos = math::detail::AxisAngle<float, detail::AngleSinCos<float>>;

}  // namespace blender::math

/** \} */
