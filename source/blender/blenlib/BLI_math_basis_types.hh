/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 *
 * An `blender::math::CartesianBasis` represents an orientation that is aligned with the basis
 * axes. This type of rotation is fast, precise and adds more meaning to the code that uses it.
 *
 * A practical reminder:
 * - Forward is typically the positive Y direction in Blender.
 * - Up is typically the positive Z direction in Blender.
 * - Right is typically the positive X direction in Blender.
 * - Blender uses right handedness.
 * - For cross product, forward = thumb, up = index, right = middle finger.
 *
 * The basis changes for each space:
 * - Object: X-right, Y-forward, Z-up
 * - World: X-right, Y-forward, Z-up
 * - Curve Tangent-Space: X-left, Y-up, Z-forward
 * - Armature Bone: (todo)
 */

#include "BLI_math_base.hh"
#include "BLI_math_vector_types.hh"

namespace blender::math {

/* -------------------------------------------------------------------- */
/** \name Axes
 * \{ */

class Axis {
 public:
  enum Value : int {
    /* Must start at 0. Used as indices in tables and vectors. */
    X = 0,
    Y,
    Z,
  };

 private:
  Value axis_;

 public:
  Axis() = default;

  constexpr Axis(Value axis) : axis_(axis){};

  /** Convert an uppercase axis character 'X', 'Y' or 'Z' to an enum value. */
  constexpr explicit Axis(char axis_char) : axis_(static_cast<Value>(axis_char - 'X'))
  {
    BLI_assert(axis_ >= Value::X && axis_ <= Value::Z);
  }

  /** Allow casting from DNA enums stored as short / int. */
  constexpr static Axis from_dna(int axis)
  {
    BLI_assert(axis >= Axis::X && axis <= Axis::Z);
    return Axis(static_cast<Value>(axis));
  }

  /** Create basis vector. */
  template<typename T> explicit operator VecBase<T, 3>() const
  {
    VecBase<T, 3> vec{};
    vec[axis_] = T(1);
    return vec;
  }

  /* Allow usage in `switch()` statements and comparisons. */
  constexpr operator Value() const
  {
    return axis_;
  }

  /** Avoid hell. */
  explicit operator bool() const = delete;

  friend std::ostream &operator<<(std::ostream &stream, Axis axis)
  {
    switch (axis.axis_) {
      default:
        BLI_assert_unreachable();
        return stream << "Invalid Axis";
      case Value::X:
        return stream << 'X';
      case Value::Y:
        return stream << 'Y';
      case Value::Z:
        return stream << 'Z';
    }
  }
};

class AxisSigned {
 public:
  enum Value : int {
    /* Match #eTrackToAxis_Modes */
    /* Must start at 0. Used as indices in tables and vectors. */
    X_POS = 0,
    Y_POS = 1,
    Z_POS = 2,
    X_NEG = 3,
    Y_NEG = 4,
    Z_NEG = 5,
  };

 private:
  enum Value axis_;

 public:
  AxisSigned() = default;

  constexpr AxisSigned(Value axis) : axis_(axis){};
  constexpr AxisSigned(Axis axis) : axis_(from_dna(axis)){};

  /** Allow casting from DNA enums stored as short / int. */
  constexpr static AxisSigned from_dna(int axis)
  {
    BLI_assert(axis >= AxisSigned::X_POS && axis <= AxisSigned::Z_NEG);
    return AxisSigned(static_cast<Value>(axis));
  }

  /** Convert / extract the axis. */
  constexpr explicit operator Axis() const
  {
    return Axis::from_dna(static_cast<int>(axis_ - ((axis_ <= 2) ? 0 : 3)));
  }

  /** Create signed 2D basis vector. */
  template<typename T> explicit operator VecBase<T, 2>() const
  {
    BLI_assert(Axis(*this) != Axis::Z);
    VecBase<T, 2> vec{};
    vec[axis_] = is_negative() ? T(-1) : T(1);
    return vec;
  }

  /** Create signed 3D basis vector. */
  template<typename T> explicit operator VecBase<T, 3>() const
  {
    VecBase<T, 3> vec{};
    vec[axis_] = is_negative() ? T(-1) : T(1);
    return vec;
  }

  /** Return the opposing axis. */
  AxisSigned operator-() const
  {
    return AxisSigned::from_dna((int(axis_) + 3) % 6);
  }

  /** Return next enum value. */
  AxisSigned next_after() const
  {
    return AxisSigned::from_dna((int(axis_) + 1) % 6);
  }

  /** Allow usage in `switch()` statements and comparisons. */
  constexpr operator Value() const
  {
    return axis_;
  }

  /** Returns -1 if axis is negative, 1 otherwise. */
  constexpr int sign() const
  {
    return is_negative() ? -1 : 1;
  }

  /** Returns true if axis is negative, false otherwise. */
  constexpr bool is_negative() const
  {
    return axis_ > Z_POS;
  }

  /** Avoid hell. */
  explicit operator bool() const = delete;

  friend std::ostream &operator<<(std::ostream &stream, AxisSigned axis)
  {
    switch (axis.axis_) {
      default:
        BLI_assert_unreachable();
        return stream << "Invalid AxisSigned";
      case Value::X_POS:
      case Value::Y_POS:
      case Value::Z_POS:
      case Value::X_NEG:
      case Value::Y_NEG:
      case Value::Z_NEG:
        return stream << Axis(axis) << (axis.sign() == -1 ? '-' : '+');
    }
  }
};
/** \} */

/* -------------------------------------------------------------------- */
/** \name Axes utilities.
 * \{ */

template<> inline AxisSigned abs(const AxisSigned &axis)
{
  return Axis(axis);
}

[[nodiscard]] inline int sign(const AxisSigned &axis)
{
  return axis.sign();
}

/**
 * Returns the cross direction from two basis direction using the right hand rule.
 * Way faster than true cross product if the vectors are basis vectors.
 * Any ill-formed case will return positive X.
 */
[[nodiscard]] inline AxisSigned cross(const AxisSigned a, const AxisSigned b)
{
  switch (a) {
    case AxisSigned::X_POS:
      switch (b) {
        case AxisSigned::X_POS:
          break; /* Ill-defined. */
        case AxisSigned::Y_POS:
          return AxisSigned::Z_POS;
        case AxisSigned::Z_POS:
          return AxisSigned::Y_NEG;
        case AxisSigned::X_NEG:
          break; /* Ill-defined. */
        case AxisSigned::Y_NEG:
          return AxisSigned::Z_NEG;
        case AxisSigned::Z_NEG:
          return AxisSigned::Y_POS;
      }
      break;
    case AxisSigned::Y_POS:
      switch (b) {
        case AxisSigned::X_POS:
          return AxisSigned::Z_NEG;
        case AxisSigned::Y_POS:
          break; /* Ill-defined. */
        case AxisSigned::Z_POS:
          return AxisSigned::X_POS;
        case AxisSigned::X_NEG:
          return AxisSigned::Z_POS;
        case AxisSigned::Y_NEG:
          break; /* Ill-defined. */
        case AxisSigned::Z_NEG:
          return AxisSigned::X_NEG;
      }
      break;
    case AxisSigned::Z_POS:
      switch (b) {
        case AxisSigned::X_POS:
          return AxisSigned::Y_POS;
        case AxisSigned::Y_POS:
          return AxisSigned::X_NEG;
        case AxisSigned::Z_POS:
          break; /* Ill-defined. */
        case AxisSigned::X_NEG:
          return AxisSigned::Y_NEG;
        case AxisSigned::Y_NEG:
          return AxisSigned::X_POS;
        case AxisSigned::Z_NEG:
          break; /* Ill-defined. */
      }
      break;
    case AxisSigned::X_NEG:
      switch (b) {
        case AxisSigned::X_POS:
          break; /* Ill-defined. */
        case AxisSigned::Y_POS:
          return AxisSigned::Z_NEG;
        case AxisSigned::Z_POS:
          return AxisSigned::Y_POS;
        case AxisSigned::X_NEG:
          break; /* Ill-defined. */
        case AxisSigned::Y_NEG:
          return AxisSigned::Z_POS;
        case AxisSigned::Z_NEG:
          return AxisSigned::Y_NEG;
      }
      break;
    case AxisSigned::Y_NEG:
      switch (b) {
        case AxisSigned::X_POS:
          return AxisSigned::Z_POS;
        case AxisSigned::Y_POS:
          break; /* Ill-defined. */
        case AxisSigned::Z_POS:
          return AxisSigned::X_NEG;
        case AxisSigned::X_NEG:
          return AxisSigned::Z_NEG;
        case AxisSigned::Y_NEG:
          break; /* Ill-defined. */
        case AxisSigned::Z_NEG:
          return AxisSigned::X_POS;
      }
      break;
    case AxisSigned::Z_NEG:
      switch (b) {
        case AxisSigned::X_POS:
          return AxisSigned::Y_NEG;
        case AxisSigned::Y_POS:
          return AxisSigned::X_POS;
        case AxisSigned::Z_POS:
          break; /* Ill-defined. */
        case AxisSigned::X_NEG:
          return AxisSigned::Y_POS;
        case AxisSigned::Y_NEG:
          return AxisSigned::X_NEG;
        case AxisSigned::Z_NEG:
          break; /* Ill-defined. */
      }
      break;
  }
  return AxisSigned::X_POS;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name CartesianBasis
 * \{ */

struct CartesianBasis {
  VecBase<AxisSigned, 3> axes = {AxisSigned::X_POS, AxisSigned::Y_POS, AxisSigned::Z_POS};

  CartesianBasis() = default;

  /**
   * Create an arbitrary basis orientation.
   * Handedness can be flipped but an axis cannot be present twice.
   */
  CartesianBasis(const AxisSigned x, const AxisSigned y, const AxisSigned z) : axes(x, y, z)
  {
    BLI_assert(abs(x) != abs(y));
    BLI_assert(abs(y) != abs(z));
    BLI_assert(abs(z) != abs(x));
  }

  const AxisSigned &x() const
  {
    return axes.x;
  }

  const AxisSigned &y() const
  {
    return axes.y;
  }

  const AxisSigned &z() const
  {
    return axes.z;
  }

  AxisSigned &x()
  {
    return axes.x;
  }

  AxisSigned &y()
  {
    return axes.y;
  }

  AxisSigned &z()
  {
    return axes.z;
  }

  friend std::ostream &operator<<(std::ostream &stream, const CartesianBasis &rot)
  {
    return stream << "CartesianBasis" << rot.axes;
  }
};

/**
 * Create an CartesianBasis for converting from \a a orientation to \a b orientation.
 * The third axis is chosen by right hand rule to follow blender coordinate system.
 * \a forward is Y axis in blender coordinate system.
 * \a up is Z axis in blender coordinate system.
 * \note \a forward and \a up must be different axes.
 */
[[nodiscard]] inline CartesianBasis from_orthonormal_axes(const AxisSigned forward,
                                                          const AxisSigned up)
{
  BLI_assert(math::abs(forward) != math::abs(up));
  return {cross(forward, up), forward, up};
}

/**
 * Create an CartesianBasis for converting from \a a orientation to \a b orientation.
 */
[[nodiscard]] inline CartesianBasis rotation_between(const CartesianBasis &a,
                                                     const CartesianBasis &b)
{

  CartesianBasis basis;
  basis.axes[abs(b.x())] = (sign(b.x()) != sign(a.x())) ? -abs(a.x()) : abs(a.x());
  basis.axes[abs(b.y())] = (sign(b.y()) != sign(a.y())) ? -abs(a.y()) : abs(a.y());
  basis.axes[abs(b.z())] = (sign(b.z()) != sign(a.z())) ? -abs(a.z()) : abs(a.z());
  return basis;
}

/**
 * Create an CartesianBasis for converting from an \a a orientation defined only by its forward
 * vector to a \a b orientation defined only by its forward vector.
 * Rotation is given to be non flipped and deterministic.
 */
[[nodiscard]] inline CartesianBasis rotation_between(const AxisSigned a_forward,
                                                     const AxisSigned b_forward)
{
  /* Pick predictable next axis. */
  AxisSigned a_up = AxisSigned(abs(a_forward.next_after()));
  AxisSigned b_up = AxisSigned(abs(b_forward.next_after()));

  if (sign(a_forward) != sign(b_forward)) {
    /* Flip both axis (up and right) so resulting rotation matrix sign remains positive. */
    b_up = -b_up;
  }
  return rotation_between(from_orthonormal_axes(a_forward, a_up),
                          from_orthonormal_axes(b_forward, b_up));
}

/** \} */

}  // namespace blender::math

/** \} */
