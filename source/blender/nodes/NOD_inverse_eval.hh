/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <variant>

#include "BLI_map.hh"
#include "BLI_struct_equality_utils.hh"

#include "DNA_node_types.h"

namespace blender::nodes::inverse_eval {

struct BoolElem {
  bool affected = false;

  operator bool() const
  {
    return this->affected;
  }

  BLI_STRUCT_EQUALITY_OPERATORS_1(BoolElem, affected)

  void merge(const BoolElem &other)
  {
    this->affected |= other.affected;
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

  BLI_STRUCT_EQUALITY_OPERATORS_1(FloatElem, affected)

  void merge(const FloatElem &other)
  {
    this->affected |= other.affected;
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

  BLI_STRUCT_EQUALITY_OPERATORS_1(IntElem, affected)

  void merge(const IntElem &other)
  {
    this->affected |= other.affected;
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

  BLI_STRUCT_EQUALITY_OPERATORS_3(VectorElem, x, y, z)

  void merge(const VectorElem &other)
  {
    this->x.merge(other.x);
    this->y.merge(other.y);
    this->z.merge(other.z);
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

  BLI_STRUCT_EQUALITY_OPERATORS_3(RotationElem, euler, axis, angle)

  void merge(const RotationElem &other)
  {
    this->euler.merge(other.euler);
    this->axis.merge(other.axis);
    this->angle.merge(other.angle);
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

  BLI_STRUCT_EQUALITY_OPERATORS_3(TransformElem, translation, rotation, scale)

  void merge(const TransformElem &other)
  {
    this->translation.merge(other.translation);
    this->rotation.merge(other.rotation);
    this->scale.merge(other.scale);
  }

  static TransformElem all()
  {
    return {VectorElem::all(), RotationElem::all(), VectorElem::all()};
  }
};

struct ElemVariant {
  std::variant<BoolElem, FloatElem, IntElem, VectorElem, RotationElem, TransformElem> elem;

  operator bool() const
  {
    return std::visit([](const auto &value) { return bool(value); }, this->elem);
  }

  void merge(const ElemVariant &other)
  {
    std::visit(
        [&](auto &value) {
          using T = std::decay_t<decltype(value)>;
          value.merge(std::get<T>(other.elem));
        },
        this->elem);
  }

  BLI_STRUCT_EQUALITY_OPERATORS_1(ElemVariant, elem)
};

struct SocketElem {
  const bNodeSocket *socket = nullptr;
  ElemVariant elem;
};

class InverseElemEvalParams {
 private:
  const Map<const bNodeSocket *, ElemVariant> &elem_by_socket_;
  Vector<SocketElem> &input_elems_;

 public:
  const bNode &node;

  InverseElemEvalParams(const bNode &node,
                        const Map<const bNodeSocket *, ElemVariant> &elem_by_socket,
                        Vector<SocketElem> &input_elems);

  template<typename T> T get_output_elem(const StringRef identifier) const
  {
    const bNodeSocket &socket = node.output_by_identifier(identifier);
    if (const ElemVariant *elem = this->elem_by_socket_.lookup_ptr(&socket)) {
      return std::get<T>(elem->elem);
    }
    return T();
  }

  template<typename T> void set_input_elem(const StringRef identifier, T elem)
  {
    const bNodeSocket &socket = node.input_by_identifier(identifier);
    input_elems_.append({&socket, ElemVariant{elem}});
  }
};

class InverseEvalParams {
 public:
  const bNode &node;

  template<typename T> T get_output(StringRef identifier) const
  {
    /* TODO */
    UNUSED_VARS(identifier);
    return T();
  }

  template<typename T> T get_input(StringRef identifier) const
  {
    /* TODO */
    UNUSED_VARS(identifier);
    return T();
  }

  template<typename T> void set_input(StringRef identifier, T value)
  {
    /* TODO */
    UNUSED_VARS(identifier, value);
  }
};

}  // namespace blender::nodes::inverse_eval
