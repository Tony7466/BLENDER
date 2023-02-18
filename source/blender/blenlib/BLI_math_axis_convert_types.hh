/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 */

#include "BLI_math_base.hh"
#include "BLI_math_vector_types.hh"

namespace blender::math {

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
 * This axis triple can then be converted to a rotation.
 */
struct AxisConversion {
  VecBase<eAxisSigned, 3> axes;

  AxisConversion() = delete;

  /**
   * Create an AxisConversion for converting from \a src orientation to \a dst orientation.
   * The third axis is chosen by right hand rule to follow blender coordinate system.
   * Returns identity if rotation is ill-defined (eg. same forward and up).
   * \a forward is Y axis in blender coordinate system.
   * \a up is Z axis in blender coordinate system.
   */
  AxisConversion(const eAxisSigned src_forward,
                 const eAxisSigned src_up,
                 const eAxisSigned dst_forward,
                 const eAxisSigned dst_up)
      : axes(axis_conversion(src_forward, src_up, dst_forward, dst_up))
  {
  }

  AxisConversion(const eAxisSigned src_forward, const eAxisSigned dst_forward)
  {
    /* Pick predictable next axis. */
    eAxisSigned src_up = eAxisSigned((src_forward + 1) % 3);
    eAxisSigned dst_up = eAxisSigned((dst_forward + 1) % 3);

    if (is_negative(src_forward) != is_negative(dst_forward)) {
      /* Flip both axis (up and right) so resulting rotation matrix sign remains positive. */
      dst_up = negate(dst_up);
    }
    axes = axis_conversion(src_forward, src_up, dst_forward, dst_up);
  }

  /**
   * Returns the cross direction from two basis direction.
   * Way faster than true cross product if the vectors are basis vectors.
   * Any ill-formed case will return positive X.
   */
  static eAxisSigned cross(const eAxisSigned a, const eAxisSigned b)
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

 private:
  static VecBase<eAxisSigned, 3> axis_conversion(const eAxisSigned src_forward,
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
};

}  // namespace blender::math

/** \} */
