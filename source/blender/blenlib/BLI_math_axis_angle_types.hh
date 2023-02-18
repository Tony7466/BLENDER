/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 */

#include "BLI_math_axis_convert_types.hh"
#include "BLI_math_base.hh"
#include "BLI_math_vector_types.hh"

namespace blender::math {

namespace detail {

/* Forward declaration for casting operators. */
template<typename T> struct EulerXYZ;
template<typename T> struct Quaternion;

template<typename T = float> struct AxisAngle {
  using vec3_type = VecBase<T, 3>;

 protected:
  vec3_type axis_ = {0, 1, 0};
  /** Store cosine and sine so rotation is cheaper and doesn't require atan2. */
  T angle_cos_ = 1;
  T angle_sin_ = 0;
  /**
   * Source angle for interpolation.
   * It might not be computed on creation, so the getter ensures it is updated.
   */
  T angle_ = 0;

  /**
   * A defaulted constructor would cause zero initialization instead of default initialization,
   * and not call the default member initializers.
   */
  explicit AxisAngle(){};

 public:
  /**
   * Create a rotation from an basis axis and an angle.
   */
  AxisAngle(const eAxisSigned axis, T angle);

  /**
   * Create a rotation from an axis and an angle.
   * \note `axis` does not have to be normalized.
   * Use `AxisAngleNormalized` instead to skip normalization cost.
   */
  AxisAngle(const vec3_type &axis, T angle);

  /**
   * Create a rotation from 2 normalized vectors.
   * \note `from` and `to` must be normalized.
   */
  AxisAngle(const vec3_type &from, const vec3_type &to);

  /** Static functions. */

  static AxisAngle<T> identity()
  {
    return AxisAngle<T>();
  }

  /** Getters. */

  const vec3_type &axis() const
  {
    return axis_;
  }

  const T &angle() const
  {
    if (UNLIKELY(angle_ == T(0) && angle_cos_ != T(1))) {
      /* Angle wasn't computed by constructor. */
      const_cast<AxisAngle *>(this)->angle_ = math::atan2(angle_sin_, angle_cos_);
    }
    return angle_;
  }

  const T &angle_cos() const
  {
    return angle_cos_;
  }

  const T &angle_sin() const
  {
    return angle_sin_;
  }

  /** Conversions. */

  explicit operator Quaternion<T>() const;

  explicit operator EulerXYZ<T>() const;

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

/**
 * A version of AxisAngle that expects axis to be already normalized.
 * Implicitly cast back to AxisAngle.
 */
template<typename T = float> struct AxisAngleNormalized : public AxisAngle<T> {
  AxisAngleNormalized(const VecBase<T, 3> &axis, T angle);

  operator AxisAngle<T>() const
  {
    return *this;
  }
};
};  // namespace detail

using AxisAngle = math::detail::AxisAngle<float>;
using AxisAngleNormalized = math::detail::AxisAngleNormalized<float>;

}  // namespace blender::math

/** \} */
