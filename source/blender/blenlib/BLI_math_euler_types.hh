/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 */

#include "BLI_math_angle_types.hh"
#include "BLI_math_axis_convert_types.hh"
#include "BLI_math_base.hh"

namespace blender::math {

namespace detail {

/* Forward declaration for casting operators. */
template<typename T, typename AngleT> struct AxisAngle;
template<typename T> struct Quaternion;

template<typename T> struct EulerXYZ {
  T x, y, z;

  EulerXYZ() = default;

  EulerXYZ(const AngleRadian<T> &x, const AngleRadian<T> &y, const AngleRadian<T> &z)
  {
    this->x = T(x);
    this->y = T(y);
    this->z = T(z);
  }

  EulerXYZ(const VecBase<T, 3> &vec) : EulerXYZ(UNPACK3(vec)){};

  /**
   * Create a rotation from an basis axis and an angle.
   */
  EulerXYZ(const eAxis axis, T angle)
  {
    BLI_assert(axis >= 0 && axis <= 2);
    (&x)[axis] = angle;
  }

  /** Static functions. */

  static EulerXYZ identity()
  {
    return {0, 0, 0};
  }

  /** Conversions. */

  explicit operator VecBase<T, 3>() const
  {
    return {this->x, this->y, this->z};
  }

  explicit operator Quaternion<T>() const;

  /** Operators. */

  EulerXYZ wrapped_around(const EulerXYZ &reference) const
  {
    using Vec3T = VecBase<T, 3>;

    constexpr float m_2pi = 2 * M_PI;
    Vec3T result(*this);
    Vec3T delta = result - Vec3T(reference);
    unroll<3>([&](auto i) {
      /* NOTE(campbell) We could use M_PI as pi_threshold: which is correct but 5.1 gives better
       * results. Checked with baking actions to fcurves. */
      constexpr float pi_threshold = 5.1f;
      if (abs(delta[i]) > pi_threshold) {
        /* Correct differences of about 360 degrees first. */
        result[i] += sign(-delta[i]) * floor((abs(delta[i]) / m_2pi) + 0.5f) * m_2pi;
      }
    });
    delta = result - Vec3T(reference);

    /* Is 1 of the axis rotations larger than 180 degrees and the other small? NO ELSE IF!! */
    if (abs(delta.x) > 3.2f && abs(delta.y) < 1.6f && abs(delta.z) < 1.6f) {
      result.x -= sign(delta.x) * m_2pi;
    }
    if (abs(delta.y) > 3.2f && abs(delta.z) < 1.6f && abs(delta.x) < 1.6f) {
      result.y -= sign(delta.y) * m_2pi;
    }
    if (abs(delta.z) > 3.2f && abs(delta.x) < 1.6f && abs(delta.y) < 1.6f) {
      result.z -= sign(delta.z) * m_2pi;
    }
    return result;
  }

  friend std::ostream &operator<<(std::ostream &stream, const EulerXYZ &rot)
  {
    return stream << "EulerXYZ" << static_cast<VecBase<T, 3>>(rot);
  }
};

template<typename T> struct Euler3 {
 public:
  /* WARNING: must match the #eRotationModes in `DNA_action_types.h`
   * order matters - types are saved to file. */
  enum eOrder {
    XYZ = 1,
    XZY,
    YXZ,
    YZX,
    ZXY,
    ZYX,
  };

 private:
  /** Raw rotation values (in radian) as passed by the constructor. */
  VecBase<T, 3> ijk_;
  /** Axes order inside `ijk_`. Immutable. */
  eOrder order_;

 public:
  Euler3() = delete;

  Euler3(const VecBase<T, 3> &angles, eOrder order) : ijk_(angles), order_(order){};

  /**
   * Defines rotation order but not the rotation values.
   * Used for conversion from other rotation types.
   */
  Euler3(eOrder order) : order_(order){};

  /**
   * Create a rotation from an basis axis and an angle.
   */
  Euler3(const eAxis axis, T angle, eOrder order) : ijk_(0), order_(order)
  {
    ijk_[axis] = angle;
  }

  /** Conversions. */

  explicit operator VecBase<T, 3>() const
  {
    return ijk_;
  }

  /**
   * This isn't a conversion, but a reinterpretation shuffling the axes around.
   * Doesn't take into account the parity.
   */
  explicit operator EulerXYZ<T>() const
  {
    return {x(), y(), z()};
  }

  explicit operator Quaternion<T>() const;

  /* Conversion to Euler3 needs to be from assignment in order to choose the order type. */
  Euler3 &operator=(const AxisAngle<T, AngleRadian<T>> &axis_angle);
  Euler3 &operator=(const Quaternion<T> &quat);

  /** Methods. */

  const eOrder &order() const
  {
    return order_;
  }

  const VecBase<T, 3> &ijk() const
  {
    return ijk_;
  }

  const T &x() const
  {
    return ijk_[x_index()];
  }

  const T &y() const
  {
    return ijk_[y_index()];
  }

  const T &z() const
  {
    return ijk_[z_index()];
  }

  VecBase<T, 3> &ijk()
  {
    return ijk_;
  }

  T &x()
  {
    return ijk_[x_index()];
  }

  T &y()
  {
    return ijk_[y_index()];
  }

  T &z()
  {
    return ijk_[z_index()];
  }

  Euler3 wrapped_around(const Euler3 &reference) const
  {
    return {VecBase<T, 3>(EulerXYZ(ijk_).wrapped_around(reference.ijk_)), order_};
  }

  /** Operators. */

  friend Euler3 operator-(const Euler3 &a)
  {
    return {-a.ijk_, a.order_};
  }

  friend std::ostream &operator<<(std::ostream &stream, const Euler3 &rot)
  {
    const char *order_str;
    switch (rot.order_) {
      default:
      case XYZ:
        order_str = "XYZ";
        break;
      case XZY:
        order_str = "XZY";
        break;
      case YXZ:
        order_str = "YXZ";
        break;
      case YZX:
        order_str = "YZX";
        break;
      case ZXY:
        order_str = "ZXY";
        break;
      case ZYX:
        order_str = "ZYX";
        break;
    }
    return stream << "Euler3_" << order_str << rot.ijk_;
  }

  /* Utilities for conversions and functions operating on Euler3.
   * This should be private in theory. */

  /**
   * Parity of axis permutation (even=0, odd=1) - 'n' in original code.
   */
  bool parity() const
  {
    switch (order_) {
      default:
      case XYZ:
      case YZX:
      case ZXY:
        return false;
      case XZY:
      case YXZ:
      case ZYX:
        return true;
    }
  }

  /**
   * Source index of the 1st axis rotation.
   */
  int x_index() const
  {
    switch (order_) {
      default:
      case XYZ:
      case XZY:
        return 0;
      case YXZ:
      case YZX:
        return 1;
      case ZXY:
      case ZYX:
        return 2;
    }
  }

  /**
   * Source index of the 2nd axis rotation.
   */
  int y_index() const
  {
    switch (order_) {
      default:
      case YXZ:
      case ZXY:
        return 0;
      case XYZ:
      case ZYX:
        return 1;
      case XZY:
      case YZX:
        return 2;
    }
  }

  /**
   * Source index of the 3rd axis rotation.
   */
  int z_index() const
  {
    switch (order_) {
      default:
      case XYZ:
      case YXZ:
        return 2;
      case XZY:
      case ZXY:
        return 1;
      case YZX:
      case ZYX:
        return 0;
    }
  }
};

}  // namespace detail

using EulerXYZ = math::detail::EulerXYZ<float>;
using Euler3 = math::detail::Euler3<float>;

}  // namespace blender::math

/** \} */
