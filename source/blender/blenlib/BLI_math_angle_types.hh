/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 *
 * Classes to represent rotation angles. They can be used as 2D rotation or as building blocks for
 * other rotation types.
 *
 * Each `blender::math::Angle***<T>` implement the same interface and can be swapped easily.
 * However, they differ in operations efficiency, storage size and the range or group of angle
 * that can be stored.
 *
 * This design allows some function overload to be more efficient with certain types.
 *
 * A `blender::math::AngleRadian<T>` is your typical radian angle.
 * - Storage : `1 * sizeof(T)`
 * - Range : [-inf..inf]
 * - Fast : Everything not slow.
 * - Slow : `cos()`, `sin()`, `tan()`, `AngleRadian(cos, sin)`
 *
 *
 * A `blender::math::AngleSinCos<T>` stores the angle as cosine + sine tuple.
 * - Storage : `2 * sizeof(T)`
 * - Range : [-pi..pi]
 * - Fast : `cos()`, `sin()`, `tan()`, `AngleSinCos(cos, sin)`
 * - Slow : Everything not fast.
 * It is only useful for intermediate representation when converting to other rotation types (eg:
 * AxisAngle > Quaternion) and for creating rotation from 2d points. In general it is offers an
 * advantage when needing trigonometric values of an angle and not directly the angle itself.
 * It is also a nice shortcut for using the trigonometric identities.
 *
 */

#include "BLI_math_base.hh"

namespace blender::math {

namespace detail {

template<typename T> struct AngleRadian {
 private:
  T value_;

 public:
  AngleRadian() = default;

  AngleRadian(const T &radian) : value_(radian){};
  explicit AngleRadian(const T &cos, const T &sin) : value_(math::atan2(sin, cos)){};

  /** Static functions. */

  static AngleRadian identity()
  {
    return 0;
  }

  static AngleRadian from_degree(const T &degrees)
  {
    return degrees * T(M_PI / 180.0);
  }

  /** Conversions. */

  /* Return angle value in radian. */
  explicit operator T() const
  {
    return value_;
  }

  /* Return angle value in degree. */
  T degree() const
  {
    return value_ * T(180.0 / M_PI);
  }

  /** Methods. */

  /**
   * Return the angle wrapped inside [-pi..pi] range.
   */
  AngleRadian wrapped() const
  {
    return math::mod(value_ + T(M_PI), T(2 * M_PI)) - T(M_PI);
  }

  /**
   * Return the angle wrapped inside [-pi..pi] range around a \a reference.
   * This mean the interpolation between the returned value and \a reference will always take the
   * shortest path.
   */
  AngleRadian wrapped_around(const AngleRadian &reference) const
  {
    return reference + (*this - reference).wrapped();
  }

  T cos() const
  {
    return math::cos(value_);
  }

  T sin() const
  {
    return math::sin(value_);
  }

  T tan() const
  {
    return math::tan(value_);
  }

  /** Operators. */

  friend AngleRadian operator+(const AngleRadian &a, const AngleRadian &b)
  {
    return a.value_ + b.value_;
  }

  friend AngleRadian operator-(const AngleRadian &a, const AngleRadian &b)
  {
    return a.value_ - b.value_;
  }

  friend AngleRadian operator*(const AngleRadian &a, const AngleRadian &b)
  {
    return a.value_ * b.value_;
  }

  friend AngleRadian operator/(const AngleRadian &a, const AngleRadian &b)
  {
    return a.value_ / b.value_;
  }

  friend AngleRadian operator-(const AngleRadian &a)
  {
    return -a.value_;
  }

  AngleRadian &operator+=(const AngleRadian &b)
  {
    value_ += b.value_;
    return *this;
  }

  AngleRadian &operator-=(const AngleRadian &b)
  {
    value_ -= b.value_;
    return *this;
  }

  AngleRadian &operator*=(const AngleRadian &b)
  {
    value_ *= b.value_;
    return *this;
  }

  AngleRadian &operator/=(const AngleRadian &b)
  {
    value_ /= b.value_;
    return *this;
  }

  friend std::ostream &operator<<(std::ostream &stream, const AngleRadian &rot)
  {
    return stream << "AngleRadian(" << rot.value_ << ")";
  }
};

template<typename T> struct AngleSinCos {
 private:
  T cos_;
  T sin_;

 public:
  AngleSinCos() = default;

  AngleSinCos(const T &cos, const T &sin) : cos_(cos), sin_(sin){};
  explicit AngleSinCos(const T &radian) : cos_(math::cos(radian)), sin_(math::sin(radian)){};
  explicit AngleSinCos(const AngleRadian<T> &angle) : cos_(angle.cos()), sin_(angle.sin()){};

  /** Static functions. */

  static AngleSinCos identity()
  {
    return {1, 0};
  }

  static AngleSinCos from_degree(const T &degrees)
  {
    return AngleSinCos(degrees * T(M_PI / 180.0));
  }

  /** Conversions. */

  /* Return angle value in radian. */
  explicit operator T() const
  {
    return math::atan2(sin_, cos_);
  }

  /** Methods. */

  T cos() const
  {
    return cos_;
  }

  T sin() const
  {
    return sin_;
  }

  T tan() const
  {
    return sin_ / cos_;
  }

  friend std::ostream &operator<<(std::ostream &stream, const AngleSinCos &rot)
  {
    return stream << "AngleSinCos(x=" << rot.cos_ << ", y=" << rot.sin_ << ")";
  }
};

}  // namespace detail

using AngleRadian = math::detail::AngleRadian<float>;
using AngleSinCos = math::detail::AngleSinCos<float>;

}  // namespace blender::math

/** \} */
