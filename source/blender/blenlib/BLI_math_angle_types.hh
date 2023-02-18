/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 */

#include "BLI_math_base.hh"

namespace blender::math {

template<typename T> static inline T deg_to_rad(T degrees)
{
  return degrees * T(M_PI / 180.0);
}

template<typename T> static inline T rad_to_deg(T radians)
{
  return radians * T(180.0 / M_PI);
}

namespace detail {

template<typename T> struct AngleRadian {
  T value;

  AngleRadian() = default;

  AngleRadian(const T &radian) : value(radian){};

  /** Static functions. */

  static AngleRadian identity()
  {
    return 0;
  }

  /** Conversions. */

  explicit operator T() const
  {
    return value;
  }

  /** Methods. */

  /**
   * Return the angle wrapped inside [-pi..pi] range.
   */
  AngleRadian wrapped() const
  {
    return math::mod(value + T(M_PI), T(2 * M_PI)) - T(M_PI);
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

  /** Operators. */

  friend AngleRadian operator+(const AngleRadian &a, const AngleRadian &b)
  {
    return a.value + b.value;
  }

  friend AngleRadian operator-(const AngleRadian &a, const AngleRadian &b)
  {
    return a.value - b.value;
  }

  friend AngleRadian operator*(const AngleRadian &a, const AngleRadian &b)
  {
    return a.value * b.value;
  }

  friend AngleRadian operator/(const AngleRadian &a, const AngleRadian &b)
  {
    return a.value / b.value;
  }

  friend AngleRadian operator-(const AngleRadian &a)
  {
    return -a.value;
  }

  AngleRadian &operator+=(const AngleRadian &b)
  {
    value += b.value;
    return *this;
  }

  AngleRadian &operator-=(const AngleRadian &b)
  {
    value -= b.value;
    return *this;
  }

  AngleRadian &operator*=(const AngleRadian &b)
  {
    value *= b.value;
    return *this;
  }

  AngleRadian &operator/=(const AngleRadian &b)
  {
    value /= b.value;
    return *this;
  }

  friend std::ostream &operator<<(std::ostream &stream, const AngleRadian &rot)
  {
    return stream << "AngleRadian(" << rot.value << ")";
  }
};

}  // namespace detail

using AngleRadian = math::detail::AngleRadian<float>;

}  // namespace blender::math

/** \} */
