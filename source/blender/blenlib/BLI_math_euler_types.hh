/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 *
 * A `blender::math::EulerXYZ` represent a typical (X, Y, Z) euler 3D rotation.
 *
 * It is prone to gimbal lock and is not suited for many application. However it is much intuitive
 * than other rotation types. Its main use it for converting user facing rotation values to other
 * rotation types. The angle values are stored as radian and expected as such by the constructors.
 *
 *
 * A `blender::math::Euler3` represent a euler 3D rotation with runtime axis order.
 *
 * It shares the same limitations as `EulerXYZ`.
 * The rotation order is set at creation and is immutable. This avoids accidentally changing the
 * meaning of what the variable holds.
 *
 * The rotation values can still be reinterpreted like this:
 * `Euler3(float3(my_euler3_zyx_rot), Euler3::eEulerOrder::XYZ)`
 * This will swap the X and Z rotation order and will likely not produce the same rotation matrix.
 *
 * If the goal is to convert (keep the same orientation) to `Euler3` then you have to do an
 * asignment.
 */

#include "BLI_math_angle_types.hh"
#include "BLI_math_axis_convert_types.hh"
#include "BLI_math_base.hh"

namespace blender::math {

namespace detail {

/* -------------------------------------------------------------------- */
/** \name EulerXYZ
 * \{ */

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

  /**
   * Create an euler x,y,z rotation from a triple of radian angle.
   */
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

  /**
   * Return this euler orientation but wrapped around \a reference.
   *
   * This mean the interpolation between the returned value and \a reference will always take the
   * shortest path. The angle between them will not be more than pi.
   */
  EulerXYZ wrapped_around(const EulerXYZ &reference) const;

  friend std::ostream &operator<<(std::ostream &stream, const EulerXYZ &rot)
  {
    return stream << "EulerXYZ" << static_cast<VecBase<T, 3>>(rot);
  }
};

}  // namespace detail

/** \} */

/* -------------------------------------------------------------------- */
/** \name Euler3
 * \{ */

/* WARNING: must match the #eRotationModes in `DNA_action_types.h`
 * order matters - types are saved to file. */
enum eEulerOrder {
  XYZ = 1,
  XZY,
  YXZ,
  YZX,
  ZXY,
  ZYX,
};

inline std::ostream &operator<<(std::ostream &stream, eEulerOrder order)
{
  switch (order) {
    default:
    case XYZ:
      return stream << "XYZ";
    case XZY:
      return stream << "XZY";
    case YXZ:
      return stream << "YXZ";
    case YZX:
      return stream << "YZX";
    case ZXY:
      return stream << "ZXY";
    case ZYX:
      return stream << "ZYX";
  }
}

namespace detail {

template<typename T> struct Euler3 {
 public:
 private:
  /** Raw rotation values (in radian) as passed by the constructor. */
  VecBase<T, 3> ijk_;
  /** Axes order inside `ijk_`. Immutable. */
  eEulerOrder order_;

 public:
  Euler3() = delete;

  Euler3(const VecBase<T, 3> &angles, eEulerOrder order) : ijk_(angles), order_(order){};

  /**
   * Create a rotation from an basis axis and an angle.
   */
  Euler3(const eAxis axis, T angle, eEulerOrder order) : ijk_(0), order_(order)
  {
    ijk_[axis] = angle;
  }

  /**
   * Defines rotation order but not the rotation values.
   * Used for conversion from other rotation types.
   */
  Euler3(eEulerOrder order) : order_(order){};

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

  /**
   * Conversion to Euler3 needs to be constructors because of the additional order.
   */
  explicit Euler3(const AxisAngle<T, AngleRadian<T>> &axis_angle, eEulerOrder order);
  explicit Euler3(const Quaternion<T> &quat, eEulerOrder order);

  /** Methods. */

  const eEulerOrder &order() const
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

  /**
   * Return this euler orientation but wrapped around \a reference.
   *
   * This mean the interpolation between the returned value and \a reference will always take the
   * shortest path. The angle between them will not be more than pi.
   */
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
    return stream << "Euler3_" << rot.order_ << rot.ijk_;
  }

  /* Utilities for conversions and functions operating on Euler3.
   * This should be private in theory. */

  /**
   * Parity of axis permutation.
   * It is considered even if axes are not shuffled (X followed by Y which in turn followed by Z).
   * Return `true` if odd (shuffled) and `false` if even (non-shuffled).
   */
  bool parity() const
  {
    switch (order_) {
      default:
      case XYZ:
      case ZXY:
      case YZX:
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

/** \} */

}  // namespace detail

using EulerXYZ = math::detail::EulerXYZ<float>;
using Euler3 = math::detail::Euler3<float>;

}  // namespace blender::math

/** \} */
