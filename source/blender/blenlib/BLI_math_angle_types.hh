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
 * A `blender::math::AngleCartesian<T>` stores the angle as cosine + sine tuple.
 * - Storage : `2 * sizeof(T)`
 * - Range : [-pi..pi]
 * - Fast : `cos()`, `sin()`, `tan()`, `AngleCartesian(cos, sin)`
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

  /* Return angle value in radian. */
  T radian() const
  {
    return value_;
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

template<typename T> struct AngleCartesian {
 private:
  T cos_;
  T sin_;

 public:
  AngleCartesian() = default;

  /**
   * Create an angle from a (x, y) position on the unit circle.
   */
  AngleCartesian(const T &x, const T &y) : cos_(x), sin_(y)
  {
    BLI_assert(math::abs(x * x + y * y - T(1)) < T(1e-4));
  }

  explicit AngleCartesian(const T &radian)
      : AngleCartesian(math::cos(radian), math::sin(radian)){};
  explicit AngleCartesian(const AngleRadian<T> &angle)
      : AngleCartesian(angle.cos(), angle.sin()){};

  /** Static functions. */

  static AngleCartesian identity()
  {
    return {1, 0};
  }

  static AngleCartesian from_degree(const T &degrees)
  {
    return AngleCartesian(degrees * T(M_PI / 180.0));
  }

  /**
   * Create an angle from a (x, y) position on the 2D plane.
   * Fallback to identity if (x, y) is origin (0, 0).
   */
  static AngleCartesian from_point(const T &x, const T &y)
  {
    T norm = math::sqrt(x * x + y * y);
    return AngleCartesian(x / norm, y / norm);
  }

  /** Conversions. */

  /* Return angle value in radian. */
  explicit operator T() const
  {
    return math::atan2(sin_, cos_);
  }

  /* Return angle value in degree. */
  T degree() const
  {
    return T(*this) * T(180.0 / M_PI);
  }

  /* Return angle value in radian. */
  T radian() const
  {
    return T(*this);
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

  /** Operators. */

  /**
   * NOTE: These use the trigonometric identities:
   * https://en.wikipedia.org/wiki/List_of_trigonometric_identities
   * (see Angle_sum_and_difference_identities, Multiple-angle_formulae and Half-angle_formulae)
   *
   * There is no identities for (arbitrary) product or quotient of angles.
   * Better leave these unimplemented to avoid accidentally using `atan` everywhere (which is the
   * purpose of this class).
   */

  friend AngleCartesian operator+(const AngleCartesian &a, const AngleCartesian &b)
  {
    return {a.cos_ * b.cos_ - a.sin_ * b.sin_, a.sin_ * b.cos_ + a.cos_ * b.sin_};
  }

  friend AngleCartesian operator-(const AngleCartesian &a, const AngleCartesian &b)
  {
    return {a.cos_ * b.cos_ + a.sin_ * b.sin_, a.sin_ * b.cos_ - a.cos_ * b.sin_};
  }

  friend AngleCartesian operator*(const AngleCartesian &a, const T &b)
  {
    if (b == T(2)) {
      return {a.cos_ * a.cos_ - a.sin_ * a.sin_, T(2) * a.sin_ * a.cos_};
    }
    if (b == T(3)) {
      return {T(4) * (a.cos_ * a.cos_ * a.cos_) - T(3) * a.cos_,
              T(3) * a.sin_ - T(4) * (a.sin_ * a.sin_ * a.sin_)};
    }
    BLI_assert_msg(0,
                   "Arbitrary angle product isn't supported with AngleCartesian<T> for "
                   "performance reason. Use AngleRadian<T> instead.");
    return identity();
  }

  friend AngleCartesian operator*(const T &b, const AngleCartesian &a)
  {
    return a * b;
  }

  friend AngleCartesian operator/(const AngleCartesian &a, const T &b)
  {
    if (b == T(2)) {
      /* Still costly but less than using `atan()`. */
      AngleCartesian result = {math::sqrt((T(1) + a.cos_) / T(2)),
                               math::sqrt((T(1) - a.cos_) / T(2))};
      /* Recover sign only for sine. Cosine of half angle is given to be positive or 0 since the
       * angle stored in #AngleCartesian is range of [-pi..pi]. */
      /* TODO(fclem): Could use copysign here. */
      if (a.sin_ < T(0)) {
        result.sin_ = -result.sin_;
      }
      return result;
    }
    BLI_assert_msg(0,
                   "Arbitrary angle quotient isn't supported with AngleCartesian<T> for "
                   "performance reason. Use AngleRadian<T> instead.");
    return identity();
  }

  friend AngleCartesian operator-(const AngleCartesian &a)
  {
    return {a.cos_, -a.sin_};
  }

  AngleCartesian &operator+=(const AngleCartesian &b)
  {
    *this = *this + b;
    return *this;
  }

  AngleCartesian &operator*=(const T &b)
  {
    *this = *this * b;
    return *this;
  }

  AngleCartesian &operator-=(const AngleCartesian &b)
  {
    *this = *this - b;
    return *this;
  }

  AngleCartesian &operator/=(const T &b)
  {
    *this = *this / b;
    return *this;
  }

  friend std::ostream &operator<<(std::ostream &stream, const AngleCartesian &rot)
  {
    return stream << "AngleCartesian(x=" << rot.cos_ << ", y=" << rot.sin_ << ")";
  }
};

}  // namespace detail

using AngleRadian = math::detail::AngleRadian<float>;
using AngleCartesian = math::detail::AngleCartesian<float>;

}  // namespace blender::math

/** \} */
