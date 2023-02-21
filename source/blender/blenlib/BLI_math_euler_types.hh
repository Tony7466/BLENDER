/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 *
 * Euler rotations are represented as a triple of angle representing a rotation around each basis
 * vector. The order in which the three rotations are applied changes the resulting orientation.
 *
 * A `blender::math::EulerXYZ` represent an euler triple with fixed axis order (XYZ).
 * A `blender::math::Euler3` represent a euler triple with arbitrary axis order.
 *
 * They are prone to gimbal lock and is not suited for many application. However they are more
 * intuitive than other rotation types. Their main use is for converting user facing rotation
 * values to other rotation types. The angle values are stored as radian and expected as such by
 * the constructors.
 *
 * `blender::math::Euler3` shares the same limitations as `blender::math::EulerXYZ`.
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
#include "BLI_math_base.hh"
#include "BLI_math_basis_types.hh"

namespace blender::math {

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

/* -------------------------------------------------------------------- */
/** \name EulerBase
 * \{ */

template<typename T> struct EulerBase {
 protected:
  /**
   * Container for the rotation values. They are always stored as XYZ
   * Rotation values are stored without parity flipping.
   */
  VecBase<T, 3> xyz_;

  EulerBase() = default;

  EulerBase(const AngleRadian<T> &x, const AngleRadian<T> &y, const AngleRadian<T> &z)
      : xyz_(T(x), T(y), T(z)){};

  EulerBase(const VecBase<T, 3> &vec) : xyz_(vec){};

 public:
  /** Static functions. */

  /** Conversions. */

  explicit operator VecBase<T, 3>() const
  {
    return this->xyz_;
  }

  /** Methods. */

  VecBase<T, 3> &xyz()
  {
    return this->xyz_;
  }
  const VecBase<T, 3> &xyz() const
  {
    return this->xyz_;
  }

  const T &x() const
  {
    return this->xyz_.x;
  }

  const T &y() const
  {
    return this->xyz_.y;
  }

  const T &z() const
  {
    return this->xyz_.z;
  }

  T &x()
  {
    return this->xyz_.x;
  }

  T &y()
  {
    return this->xyz_.y;
  }

  T &z()
  {
    return this->xyz_.z;
  }
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name EulerXYZ
 * \{ */

/* Forward declaration for casting operators. */
template<typename T, typename AngleT> struct AxisAngle;
template<typename T> struct Quaternion;

template<typename T> struct EulerXYZ : public EulerBase<T> {
 public:
  EulerXYZ() = default;

  EulerXYZ(const AngleRadian<T> &x, const AngleRadian<T> &y, const AngleRadian<T> &z)
      : EulerBase<T>(x, y, z){};

  /**
   * Create an euler x,y,z rotation from a triple of radian angle.
   */
  EulerXYZ(const VecBase<T, 3> &vec) : EulerBase<T>(vec){};

  /**
   * Create a rotation from an basis axis and an angle.
   * This sets a single component of the euler triple, the others are left to 0.
   */
  EulerXYZ(const eAxis axis, T angle)
  {
    BLI_assert(axis >= 0 && axis <= 2);
    *static_cast<EulerBase<T> *>(this) = identity();
    this->xyz_[axis] = angle;
  }

  /** Static functions. */

  static EulerXYZ identity()
  {
    return {AngleRadian<T>::identity(), AngleRadian<T>::identity(), AngleRadian<T>::identity()};
  }

  /** Methods. */

  /**
   * Return this euler orientation but with angles wrapped inside [-pi..pi] range.
   */
  EulerXYZ wrapped() const;

  /**
   * Return this euler orientation but wrapped around \a reference.
   *
   * This mean the interpolation between the returned value and \a reference will always take the
   * shortest path. The angle between them will not be more than pi.
   */
  EulerXYZ wrapped_around(const EulerXYZ &reference) const;

  /** Conversions. */

  explicit operator Quaternion<T>() const;

  /** Operators. */

  friend EulerXYZ operator-(const EulerXYZ &a)
  {
    return {-a.xyz_};
  }

  friend bool operator==(const EulerXYZ &a, const EulerXYZ &b)
  {
    return a.xyz_ == b.xyz_;
  }

  friend std::ostream &operator<<(std::ostream &stream, const EulerXYZ &rot)
  {
    return stream << "EulerXYZ" << static_cast<VecBase<T, 3>>(rot);
  }
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name Euler3
 * \{ */

template<typename T> struct Euler3 : public EulerBase<T> {
 private:
  /** Axes order from applying the rotation. */
  eEulerOrder order_;

  /**
   * Swizzle structure allowing to shuffled assignement.
   */
  class Swizzle {
   private:
    Euler3 &eul_;

   public:
    Swizzle(Euler3 &eul) : eul_(eul){};

    Euler3 &operator=(const VecBase<T, 3> &angles)
    {
      eul_.xyz_.x = angles[eul_.i_index()];
      eul_.xyz_.y = angles[eul_.j_index()];
      eul_.xyz_.z = angles[eul_.k_index()];
      return eul_;
    }

    operator VecBase<T, 3>() const
    {
      return {eul_.i(), eul_.j(), eul_.k()};
    }
  };

 public:
  Euler3() = delete;

  /**
   * Create an euler rotation with \a order rotation ordering
   * from a triple of radian angles in XYZ order.
   * eg: If \a order is `eEulerOrder::ZXY` then `angles.z` will be the angle of the first rotation.
   */
  Euler3(const VecBase<T, 3> &angles_xyz, eEulerOrder order)
      : EulerBase<T>(angles_xyz), order_(order){};

  /**
   * Create a rotation from an basis axis and an angle.
   */
  Euler3(const eAxis axis, T angle, eEulerOrder order) : EulerBase<T>(), order_(order)
  {
    this->xyz_[axis] = angle;
  }

  /**
   * Defines rotation order but not the rotation values.
   * Used for conversion from other rotation types.
   */
  Euler3(eEulerOrder order) : order_(order){};

  /** Conversions. */

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

  /**
   * Returns the rotations angle in rotation order.
   * eg: if rotation `order` is `YZX` then `i` is the `Y` rotation.
   */
  Swizzle ijk()
  {
    return {*this};
  }
  const VecBase<T, 3> ijk() const
  {
    return {i(), j(), k()};
  }

  /**
   * Returns the rotations angle in rotation order.
   * eg: if rotation `order` is `YZX` then `i` is the `Y` rotation.
   */
  const T &i() const
  {
    return this->xyz_[i_index()];
  }
  const T &j() const
  {
    return this->xyz_[j_index()];
  }
  const T &k() const
  {
    return this->xyz_[k_index()];
  }
  T &i()
  {
    return this->xyz_[i_index()];
  }
  T &j()
  {
    return this->xyz_[j_index()];
  }
  T &k()
  {
    return this->xyz_[k_index()];
  }

  /**
   * Return this euler orientation but wrapped around \a reference.
   *
   * This mean the interpolation between the returned value and \a reference will always take the
   * shortest path. The angle between them will not be more than pi.
   */
  Euler3 wrapped_around(const Euler3 &reference) const
  {
    return {VecBase<T, 3>(EulerXYZ(this->xyz_).wrapped_around(reference.xyz_)), order_};
  }

  /** Operators. */

  friend Euler3 operator-(const Euler3 &a)
  {
    return {-a.xyz_, a.order_};
  }

  friend bool operator==(const Euler3 &a, const Euler3 &b)
  {
    return a.xyz_ == b.xyz_ && a.order_ == b.order_;
  }

  friend std::ostream &operator<<(std::ostream &stream, const Euler3 &rot)
  {
    return stream << "Euler3_" << rot.order_ << rot.xyz_;
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
   * Source Axis of the 1st axis rotation.
   */
  int i_index() const
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
   * Source Axis of the 2nd axis rotation.
   */
  int j_index() const
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
   * Source Axis of the 3rd axis rotation.
   */
  int k_index() const
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
