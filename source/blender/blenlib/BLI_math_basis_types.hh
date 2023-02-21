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
/** \name Conversion
 * \{ */

enum Axis {
  /* Must start at 0. Used as indices in tables and vectors. */
  X = 0,
  Y,
  Z,
};

enum AxisSigned {
  /* Match #eTrackToAxis_Modes */
  /* Must start at 0. Used as indices in tables and vectors. */
  X_POS = 0,
  Y_POS = 1,
  Z_POS = 2,
  X_NEG = 3,
  Y_NEG = 4,
  Z_NEG = 5,
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name Axes utilities.
 * \{ */

[[nodiscard]] inline Axis axis_unsigned(const AxisSigned axis)
{
  return Axis(axis - ((axis <= 2) ? 0 : 3));
}

template<typename T> [[nodiscard]] inline VecBase<T, 3> basis_vector(const AxisSigned axis)
{
  BLI_assert(axis >= AxisSigned::X_POS && axis <= AxisSigned::Z_NEG);
  VecBase<T, 3> vec{};
  vec[axis % 3] = (axis > 2) ? T(-1) : T(1);
  return vec;
}

template<typename T> [[nodiscard]] inline VecBase<T, 3> basis_vector(const Axis axis)
{
  BLI_assert(axis >= Axis::X && axis <= Axis::Z);
  return basis_vector<T>(AxisSigned(axis));
}

[[nodiscard]] inline Axis axis_from_char(const char axis)
{
  BLI_assert(axis >= 'X' && axis <= 'Z');
  return Axis(axis - 'X');
}

[[nodiscard]] inline bool is_negative(const AxisSigned axis)
{
  return axis > Z_POS;
}

[[nodiscard]] inline AxisSigned negate(const AxisSigned axis)
{
  return AxisSigned((axis + 3) % 6);
}

template<> inline AxisSigned abs(const AxisSigned &axis)
{
  return AxisSigned(axis_unsigned(axis));
}

[[nodiscard]] inline int sign(const AxisSigned &axis)
{
  return is_negative(axis) ? -1 : 1;
}

inline std::ostream &operator<<(std::ostream &stream, AxisSigned axis)
{
  switch (axis) {
    default:
      BLI_assert_unreachable();
      return stream << "Invalid Axis";
    case X_POS:
      return stream << "X_POS";
    case Y_POS:
      return stream << "Y_POS";
    case Z_POS:
      return stream << "Z_POS";
    case X_NEG:
      return stream << "X_NEG";
    case Y_NEG:
      return stream << "Y_NEG";
    case Z_NEG:
      return stream << "Z_NEG";
  }
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
  basis.axes[abs(b.x())] = (sign(b.x()) != sign(a.x())) ? negate(abs(a.x())) : abs(a.x());
  basis.axes[abs(b.y())] = (sign(b.y()) != sign(a.y())) ? negate(abs(a.y())) : abs(a.y());
  basis.axes[abs(b.z())] = (sign(b.z()) != sign(a.z())) ? negate(abs(a.z())) : abs(a.z());
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
  AxisSigned a_up = AxisSigned((a_forward + 1) % 3);
  AxisSigned b_up = AxisSigned((b_forward + 1) % 3);

  if (is_negative(a_forward) != is_negative(b_forward)) {
    /* Flip both axis (up and right) so resulting rotation matrix sign remains positive. */
    b_up = negate(b_up);
  }
  return rotation_between(from_orthonormal_axes(a_forward, a_up),
                          from_orthonormal_axes(b_forward, b_up));
}

/** \} */

}  // namespace blender::math

/** \} */
