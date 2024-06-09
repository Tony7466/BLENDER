/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <variant>

#include "DNA_node_types.h"

namespace blender::nodes::inverse_eval {

struct BoolElem {
  bool affected = false;

  operator bool() const
  {
    return this->affected;
  }

  static BoolElem all()
  {
    return {true};
  }
};

struct FloatElem {
  bool affected = false;

  operator bool() const
  {
    return this->affected;
  }

  static FloatElem all()
  {
    return {true};
  }
};

struct IntElem {
  bool affected = false;

  operator bool() const
  {
    return this->affected;
  }

  static IntElem all()
  {
    return {true};
  }
};

struct VectorElem {
  FloatElem x;
  FloatElem y;
  FloatElem z;

  operator bool() const
  {
    return this->x || this->y || this->z;
  }

  static VectorElem all()
  {
    return {true, true, true};
  }
};

struct RotationElem {
  VectorElem euler;
  VectorElem axis;
  FloatElem angle;

  operator bool() const
  {
    return this->euler || this->axis || this->angle;
  }

  bool only_euler_angles() const
  {
    return !(this->axis || this->angle);
  }

  bool only_axis_angle() const
  {
    return !this->euler;
  }

  static RotationElem all()
  {
    return {VectorElem::all(), VectorElem::all(), true};
  }
};

struct TransformElem {
  VectorElem translation;
  RotationElem rotation;
  VectorElem scale;

  operator bool() const
  {
    return this->translation || this->rotation || this->scale;
  }

  static TransformElem all()
  {
    return {VectorElem::all(), RotationElem::all(), VectorElem::all()};
  }
};

using ElemVariant =
    std::variant<BoolElem, FloatElem, IntElem, VectorElem, RotationElem, TransformElem>;

class InverseElemEvalParams {
 public:
  const bNode &node;

  template<typename T> T get_output_elem(StringRef identifier) const;
  template<typename T> void set_input_elem(StringRef identifier, T elem);
};

class InverseEvalParams {
 public:
  const bNode &node;

  template<typename T> T get_output(StringRef identifier) const;
  template<typename T> T get_input(StringRef identifier) const;
  template<typename T> void set_input(StringRef identifier, T value);
};

}  // namespace blender::nodes::inverse_eval
