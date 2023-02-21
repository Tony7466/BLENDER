/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 *
 * An `blender::math::CartesianBasis` represents an orientation that is aligned with the basis
 * axes. This type of rotation is fast, precise and adds more meaning to the code that uses it.
 *
 * A practical reminder:
 * - Forward is typically the positive Y direction in blender.
 * - Up is typically the positive Z direction in blender.
 * - Right is typically the positive X direction in blender.
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

/** \} */

/* -------------------------------------------------------------------- */
/** \name Axes utilities.
 * \{ */

[[nodiscard]] inline eAxis axis_unsigned(const eAxisSigned axis)
{
  return eAxis(axis - ((axis <= 2) ? 0 : 3));
}

template<typename T> [[nodiscard]] inline VecBase<T, 3> basis_vector(const eAxisSigned axis)
{
  BLI_assert(axis >= eAxisSigned::X_POS && axis <= eAxisSigned::Z_NEG);
  VecBase<T, 3> vec{};
  vec[axis % 3] = (axis > 2) ? T(-1) : T(1);
  return vec;
}

template<typename T> [[nodiscard]] inline VecBase<T, 3> basis_vector(const eAxis axis)
{
  BLI_assert(axis >= eAxis::X && axis <= eAxis::Z);
  return basis_vector<T>(eAxisSigned(axis));
}

[[nodiscard]] inline eAxis axis_from_char(const char axis)
{
  BLI_assert(axis >= 'X' && axis <= 'Z');
  return eAxis(axis - 'X');
}

[[nodiscard]] inline bool is_negative(const eAxisSigned axis)
{
  return axis > Z_POS;
}

[[nodiscard]] inline eAxisSigned negate(const eAxisSigned axis)
{
  return eAxisSigned((axis + 3) % 6);
}

template<> inline eAxisSigned abs(const eAxisSigned &axis)
{
  return eAxisSigned(axis_unsigned(axis));
}

[[nodiscard]] inline int sign(const eAxisSigned &axis)
{
  return is_negative(axis) ? -1 : 1;
}

inline std::ostream &operator<<(std::ostream &stream, eAxisSigned axis)
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
[[nodiscard]] inline eAxisSigned cross(const eAxisSigned a, const eAxisSigned b)
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

/** \} */

/* -------------------------------------------------------------------- */
/** \name CartesianBasis
 * \{ */

struct CartesianBasis {
  VecBase<eAxisSigned, 3> axes = {eAxisSigned::X_POS, eAxisSigned::Y_POS, eAxisSigned::Z_POS};

  CartesianBasis() = default;

  /**
   * Create an arbitrary basis orientation.
   * Handedness can be flipped but an axis cannot be present twice.
   */
  CartesianBasis(const eAxisSigned x, const eAxisSigned y, const eAxisSigned z) : axes(x, y, z)
  {
    BLI_assert(abs(x) != abs(y));
    BLI_assert(abs(y) != abs(z));
    BLI_assert(abs(z) != abs(x));
  }

  const eAxisSigned &x() const
  {
    return axes.x;
  }

  const eAxisSigned &y() const
  {
    return axes.y;
  }

  const eAxisSigned &z() const
  {
    return axes.z;
  }

  eAxisSigned &x()
  {
    return axes.x;
  }

  eAxisSigned &y()
  {
    return axes.y;
  }

  eAxisSigned &z()
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
[[nodiscard]] inline CartesianBasis from_orthonormal_axes(const eAxisSigned forward,
                                                          const eAxisSigned up)
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
[[nodiscard]] inline CartesianBasis rotation_between(const eAxisSigned a_forward,
                                                     const eAxisSigned b_forward)
{
  /* Pick predictable next axis. */
  eAxisSigned a_up = eAxisSigned((a_forward + 1) % 3);
  eAxisSigned b_up = eAxisSigned((b_forward + 1) % 3);

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
