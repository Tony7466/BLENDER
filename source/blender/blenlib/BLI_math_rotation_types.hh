/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 */

#include "BLI_math_base.hh"
#include "BLI_math_vector_types.hh"

namespace blender::math {

template<typename T> static inline T deg_to_rad(T degrees)
{
  return degrees * T(M_PI / 180.0);
}

template<typename T> static inline T rad_to_deg(T radians)
{
  return radians * T(180.0 / M_PI);
}

enum eAxis {
  X = 0,
  Y,
  Z,
};

enum eAxisSigned {
  X_POS = 0,
  Y_POS = 1,
  Z_POS = 2,
  X_NEG = 3,
  Y_NEG = 4,
  Z_NEG = 5,
};

/**
 * Axes utilities.
 */

template<typename T> [[nodiscard]] VecBase<T, 3> basis_vector(const eAxis axis)
{
  BLI_assert(axis >= eAxis::X && axis <= eAxis::Z);
  VecBase<T, 3> vec{};
  vec[axis] = T(1);
  return vec;
}

template<typename T> [[nodiscard]] VecBase<T, 3> basis_vector(const eAxisSigned axis)
{
  BLI_assert(axis >= eAxisSigned::X_POS && axis <= eAxisSigned::Z_NEG);
  VecBase<T, 3> vec{};
  vec[axis % 3] = (axis > 2) ? T(-1) : T(1);
  return vec;
}

static inline eAxis axis_unsigned(const eAxisSigned axis)
{
  return eAxis(axis - ((axis <= 2) ? 0 : 3));
}

static inline bool is_negative(const eAxisSigned axis)
{
  return axis > Z_POS;
}

/**
 * Returns the cross direction from two basis direction.
 * Way faster than true cross product if you vectors are basis vectors.
 * Any ill-formed case will return positive X.
 */
static inline eAxisSigned cross(const eAxisSigned a, const eAxisSigned b)
{
  switch (a) {
    case eAxisSigned::X_POS:
      switch (b) {
        case eAxisSigned::X_POS:
          break; /* Ill-defined. */
        case eAxisSigned::Y_POS:
          return eAxisSigned::Z_POS;
        case eAxisSigned::Z_POS:
          return eAxisSigned::Y_NEG;
        case eAxisSigned::X_NEG:
          break; /* Ill-defined. */
        case eAxisSigned::Y_NEG:
          return eAxisSigned::Z_NEG;
        case eAxisSigned::Z_NEG:
          return eAxisSigned::Y_POS;
      }
      break;
    case eAxisSigned::Y_POS:
      switch (b) {
        case eAxisSigned::X_POS:
          return eAxisSigned::Z_NEG;
        case eAxisSigned::Y_POS:
          break; /* Ill-defined. */
        case eAxisSigned::Z_POS:
          return eAxisSigned::X_POS;
        case eAxisSigned::X_NEG:
          return eAxisSigned::Z_POS;
        case eAxisSigned::Y_NEG:
          break; /* Ill-defined. */
        case eAxisSigned::Z_NEG:
          return eAxisSigned::X_NEG;
      }
      break;
    case eAxisSigned::Z_POS:
      switch (b) {
        case eAxisSigned::X_POS:
          return eAxisSigned::Y_POS;
        case eAxisSigned::Y_POS:
          return eAxisSigned::X_NEG;
        case eAxisSigned::Z_POS:
          break; /* Ill-defined. */
        case eAxisSigned::X_NEG:
          return eAxisSigned::Y_NEG;
        case eAxisSigned::Y_NEG:
          return eAxisSigned::X_POS;
        case eAxisSigned::Z_NEG:
          break; /* Ill-defined. */
      }
      break;
    case eAxisSigned::X_NEG:
      switch (b) {
        case eAxisSigned::X_POS:
          break; /* Ill-defined. */
        case eAxisSigned::Y_POS:
          return eAxisSigned::Z_NEG;
        case eAxisSigned::Z_POS:
          return eAxisSigned::Y_POS;
        case eAxisSigned::X_NEG:
          break; /* Ill-defined. */
        case eAxisSigned::Y_NEG:
          return eAxisSigned::Z_POS;
        case eAxisSigned::Z_NEG:
          return eAxisSigned::Y_NEG;
      }
      break;
    case eAxisSigned::Y_NEG:
      switch (b) {
        case eAxisSigned::X_POS:
          return eAxisSigned::Z_POS;
        case eAxisSigned::Y_POS:
          break; /* Ill-defined. */
        case eAxisSigned::Z_POS:
          return eAxisSigned::X_NEG;
        case eAxisSigned::X_NEG:
          return eAxisSigned::Z_NEG;
        case eAxisSigned::Y_NEG:
          break; /* Ill-defined. */
        case eAxisSigned::Z_NEG:
          return eAxisSigned::X_POS;
      }
      break;
    case eAxisSigned::Z_NEG:
      switch (b) {
        case eAxisSigned::X_POS:
          return eAxisSigned::Y_NEG;
        case eAxisSigned::Y_POS:
          return eAxisSigned::X_POS;
        case eAxisSigned::Z_POS:
          break; /* Ill-defined. */
        case eAxisSigned::X_NEG:
          return eAxisSigned::Y_POS;
        case eAxisSigned::Y_NEG:
          return eAxisSigned::X_NEG;
        case eAxisSigned::Z_NEG:
          break; /* Ill-defined. */
      }
      break;
  }
  return eAxisSigned::X_POS;
}

namespace detail {

/**
 * Rotation Types
 *
 * It gives more semantic information allowing overloaded functions based on the rotation type.
 * It also prevent implicit cast from rotation to vector types.
 */

/* Forward declaration. */
template<typename T> struct AxisAngle;
template<typename T> struct Quaternion;

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

template<typename T> struct EulerXYZ {
  T x, y, z;

  EulerXYZ() = default;

  EulerXYZ(const T &x, const T &y, const T &z)
  {
    this->x = x;
    this->y = y;
    this->z = z;
  }

  EulerXYZ(const VecBase<T, 3> &vec) : EulerXYZ(UNPACK3(vec)){};

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

  explicit operator AxisAngle<T>() const;

  explicit operator Quaternion<T>() const;

  /** Operators. */

  friend std::ostream &operator<<(std::ostream &stream, const EulerXYZ &rot)
  {
    return stream << "EulerXYZ" << static_cast<VecBase<T, 3>>(rot);
  }
};

template<typename T = float> struct Quaternion {
  T w, x, y, z;

  Quaternion() = default;

  Quaternion(const T &w, const T &x, const T &y, const T &z)
  {
    this->w = w;
    this->x = x;
    this->y = y;
    this->z = z;
  }

  /** \note: W component is supposed to be first. */
  Quaternion(const VecBase<T, 4> &vec) : Quaternion(UNPACK4(vec)){};

  /** Static functions. */

  static Quaternion identity()
  {
    return {1, 0, 0, 0};
  }

  /**
   * Create a quaternion from an exponential map representation.
   * An exponential map is basically the rotation axis multiplied by the rotation angle.
   */
  static Quaternion expmap(const VecBase<T, 3> &expmap);

  /** Conversions. */

  explicit operator VecBase<T, 4>() const
  {
    return {this->w, this->x, this->y, this->z};
  }

  explicit operator EulerXYZ<T>() const;

  explicit operator AxisAngle<T>() const;

  /**
   * Create an exponential map representation of this quaternion.
   * An exponential map is basically the rotation axis multiplied by the rotation angle.
   */
  VecBase<T, 3> expmap() const;

  /**
   * Returns the full twist angle for a given basis \a axis .
   */
  AngleRadian<T> twist_angle(const eAxis axis) const;

  /**
   * Returns the twist part of this quaternion for the basis \a axis .
   */
  Quaternion twist(const eAxis axis, AngleRadian<T> &r_twist_angle) const;
  Quaternion twist(const eAxis axis) const;

  /**
   * Returns the swing part of this quaternion for the basis \a axis .
   */
  Quaternion swing(const eAxis axis, AngleRadian<T> &r_twist_angle) const;
  Quaternion swing(const eAxis axis) const;

  /** Methods. */

  /**
   * Return this quaternions orientation but wrapped around \a reference.
   *
   * This mean the interpolation between the returned value and \a reference will always take the
   * shortest path. The angle between them will not be more than pi.
   *
   * \note This quaternion is expected to be a unit quaternion.
   * \note Works even if \a reference is *not* a unit quaternion.
   */
  Quaternion wrapped_around(const Quaternion &reference) const;

  /** Operators. */

  friend Quaternion operator*(const Quaternion &a, const Quaternion &b)
  {
    return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y + a.y * b.w + a.z * b.x - a.x * b.z,
            a.w * b.z + a.z * b.w + a.x * b.y - a.y * b.x};
  }

  /* Scalar product. */
  friend Quaternion operator*(const Quaternion &a, const T &b)
  {
    return {a.w * b, a.x * b, a.y * b, a.z * b};
  }

  /* Negate the quaternion. */
  friend Quaternion operator-(const Quaternion &a)
  {
    return {-a.w, -a.x, -a.y, -a.z};
  }

  friend std::ostream &operator<<(std::ostream &stream, const Quaternion &rot)
  {
    return stream << "Quaternion" << static_cast<VecBase<T, 4>>(rot);
  }
};

template<typename T> struct AxisAngle {
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
template<typename T> struct AxisAngleNormalized : public AxisAngle<T> {
  AxisAngleNormalized(const VecBase<T, 3> &axis, T angle);

  operator AxisAngle<T>() const
  {
    return *this;
  }
};

/**
 * Intermediate Types.
 *
 * Some functions need to have higher precision than standard floats for some operations.
 */
template<typename T> struct TypeTraits {
  using DoublePrecision = T;
};
template<> struct TypeTraits<float> {
  using DoublePrecision = double;
};

};  // namespace detail

template<typename U> struct AssertUnitEpsilon<detail::Quaternion<U>> {
  static constexpr U value = AssertUnitEpsilon<U>::value * 10;
};

/* Most common used types. */
using AngleRadian = math::detail::AngleRadian<float>;
using EulerXYZ = math::detail::EulerXYZ<float>;
using Quaternion = math::detail::Quaternion<float>;
using AxisAngle = math::detail::AxisAngle<float>;
using AxisAngleNormalized = math::detail::AxisAngleNormalized<float>;

}  // namespace blender::math

/** \} */
