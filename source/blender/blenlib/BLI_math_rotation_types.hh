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
  /* Must start at 0. Used as indices in tables and vectors. */
  X = 0,
  Y,
  Z,
};

enum eAxisSigned {
  /* Match #eTrackToAxis_Modes */
  /* Must start at 0. Used as indices in tables and vectors. */
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

static inline eAxis axis_from_char(const char axis)
{
  BLI_assert(axis >= 'X' && axis <= 'Z');
  return eAxis(axis - 'X');
}

static inline bool is_negative(const eAxisSigned axis)
{
  return axis > Z_POS;
}

static inline eAxisSigned negate(const eAxisSigned axis)
{
  return eAxisSigned((axis + 3) % 6);
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

/**
 * Returns an axis triple for converting from \a src orientation to \a dst orientation.
 * This axis triple can then be converted to a rotation.
 * The third axis is chosen by right hand rule to follow blender coordinate system.
 * Returns identity if rotation is ill-defined (eg. same forward and up).
 * \a forward is Y axis in blender coordinate system.
 * \a up is Z axis in blender coordinate system.
 */
static inline VecBase<eAxisSigned, 3> axis_conversion(const eAxisSigned src_forward,
                                                      const eAxisSigned src_up,
                                                      const eAxisSigned dst_forward,
                                                      const eAxisSigned dst_up)
{
  /* Start with identity for failure cases. */
  VecBase<eAxisSigned, 3> axes(eAxisSigned::X_POS, eAxisSigned::Y_POS, eAxisSigned::Z_POS);

  if (src_forward == dst_forward && src_up == dst_up) {
    return axes;
  }

  if ((axis_unsigned(src_forward) == axis_unsigned(src_up)) ||
      (axis_unsigned(dst_forward) == axis_unsigned(dst_up))) {
    /* We could assert here! */
    return axes;
  }
  /* Reminder that blender uses right hand rule. */
  const eAxisSigned src_right = cross(src_forward, src_up);
  const eAxisSigned dst_right = cross(dst_forward, dst_up);

  eAxisSigned src_x = eAxisSigned(axis_unsigned(src_right));
  eAxisSigned src_y = eAxisSigned(axis_unsigned(src_forward));
  eAxisSigned src_z = eAxisSigned(axis_unsigned(src_up));

  /* Correct axis sign. */
  src_x = (is_negative(dst_right) != is_negative(src_right)) ? negate(src_x) : src_x;
  src_y = (is_negative(dst_forward) != is_negative(src_forward)) ? negate(src_y) : src_y;
  src_z = (is_negative(dst_up) != is_negative(src_up)) ? negate(src_z) : src_z;

  eAxis dst_x = axis_unsigned(dst_right);
  eAxis dst_y = axis_unsigned(dst_forward);
  eAxis dst_z = axis_unsigned(dst_up);

  /* Shuffle axes. */
  axes[dst_x] = src_x;
  axes[dst_y] = src_y;
  axes[dst_z] = src_z;
  return axes;
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

  explicit operator AxisAngle<T>() const;

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

  explicit operator AxisAngle<T>() const;
  explicit operator Quaternion<T>() const;

  /* Conversion to Euler3 needs to be from assignment in order to choose the order type. */
  Euler3 &operator=(const AxisAngle<T> &axis_angle);
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
  explicit Quaternion(const VecBase<T, 4> &vec) : Quaternion(UNPACK4(vec)){};

  /**
   * Creates a quaternion from real (w) and imaginary parts (x, y, z).
   */
  Quaternion(const T &real, const VecBase<T, 3> &imaginary)
      : Quaternion(real, UNPACK3(imaginary)){};

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

  /**
   * Returns the imaginary part of this quaternion (x, y, z).
   */
  const VecBase<T, 3> &imaginary_part() const
  {
    return *reinterpret_cast<const VecBase<T, 3> *>(&x);
  }
  VecBase<T, 3> &imaginary_part()
  {
    return *reinterpret_cast<VecBase<T, 3> *>(&x);
  }

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

  Quaternion &operator*=(const Quaternion &b)
  {
    *this = *this * b;
    return *this;
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
using Euler3 = math::detail::Euler3<float>;
using Quaternion = math::detail::Quaternion<float>;
using AxisAngle = math::detail::AxisAngle<float>;
using AxisAngleNormalized = math::detail::AxisAngleNormalized<float>;

}  // namespace blender::math

/** \} */
