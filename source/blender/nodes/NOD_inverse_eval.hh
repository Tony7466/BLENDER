/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <optional>
#include <variant>

#include "BLI_map.hh"
#include "BLI_struct_equality_utils.hh"

#include "DNA_node_types.h"

#include "BKE_node_runtime.hh"
#include "BKE_node_socket_value.hh"

namespace blender::nodes::inverse_eval {

struct PrimitiveValueElem {
  bool affected = false;

  operator bool() const
  {
    return this->affected;
  }

  BLI_STRUCT_EQUALITY_OPERATORS_1(PrimitiveValueElem, affected)

  void merge(const PrimitiveValueElem &other)
  {
    this->affected |= other.affected;
  }

  void intersect(const PrimitiveValueElem &other)
  {
    this->affected &= other.affected;
  }

  uint64_t hash() const
  {
    return get_default_hash(this->affected);
  }
};

struct BoolElem : public PrimitiveValueElem {
  static BoolElem all()
  {
    return {true};
  }
};

struct FloatElem : public PrimitiveValueElem {
  static FloatElem all()
  {
    return {true};
  }
};

struct IntElem : public PrimitiveValueElem {
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

  uint64_t hash() const
  {
    return get_default_hash(this->x, this->y, this->z);
  }

  void merge(const VectorElem &other)
  {
    this->x.merge(other.x);
    this->y.merge(other.y);
    this->z.merge(other.z);
  }

  void intersect(const VectorElem &other)
  {
    this->x.intersect(other.x);
    this->y.intersect(other.y);
    this->z.intersect(other.z);
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

  uint64_t hash() const
  {
    return get_default_hash(this->euler, this->axis, this->angle);
  }

  void merge(const RotationElem &other)
  {
    this->euler.merge(other.euler);
    this->axis.merge(other.axis);
    this->angle.merge(other.angle);
  }

  void intersect(const RotationElem &other)
  {
    this->euler.intersect(other.euler);
    this->axis.intersect(other.axis);
    this->angle.intersect(other.angle);
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

struct MatrixElem {
  VectorElem translation;
  RotationElem rotation;
  VectorElem scale;
  FloatElem any_non_transform;

  operator bool() const
  {
    return this->translation || this->rotation || this->scale || this->any_non_transform;
  }

  BLI_STRUCT_EQUALITY_OPERATORS_4(MatrixElem, translation, rotation, scale, any_non_transform)

  uint64_t hash() const
  {
    return get_default_hash(
        this->translation, this->rotation, this->scale, this->any_non_transform);
  }

  void merge(const MatrixElem &other)
  {
    this->translation.merge(other.translation);
    this->rotation.merge(other.rotation);
    this->scale.merge(other.scale);
    this->any_non_transform.merge(other.any_non_transform);
  }

  void intersect(const MatrixElem &other)
  {
    this->translation.intersect(other.translation);
    this->rotation.intersect(other.rotation);
    this->scale.intersect(other.scale);
    this->any_non_transform.intersect(other.any_non_transform);
  }

  static MatrixElem all()
  {
    return {VectorElem::all(), RotationElem::all(), VectorElem::all(), FloatElem::all()};
  }
};

struct ElemVariant {
  std::variant<BoolElem, FloatElem, IntElem, VectorElem, RotationElem, MatrixElem> elem;

  operator bool() const
  {
    return std::visit([](const auto &value) { return bool(value); }, this->elem);
  }

  uint64_t hash() const
  {
    return std::visit([](auto &value) { return value.hash(); }, this->elem);
  }

  void merge(const ElemVariant &other)
  {
    BLI_assert(this->elem.index() == other.elem.index());
    std::visit(
        [&](auto &value) {
          using T = std::decay_t<decltype(value)>;
          value.merge(std::get<T>(other.elem));
        },
        this->elem);
  }

  void intersect(const ElemVariant &other)
  {
    BLI_assert(this->elem.index() == other.elem.index());
    std::visit(
        [&](auto &value) {
          using T = std::decay_t<decltype(value)>;
          value.intersect(std::get<T>(other.elem));
        },
        this->elem);
  }

  void set_all()
  {
    std::visit(
        [](auto &value) {
          using T = std::decay_t<decltype(value)>;
          value = T::all();
        },
        this->elem);
  }

  void clear_all()
  {
    std::visit(
        [](auto &value) {
          using T = std::decay_t<decltype(value)>;
          value = T();
        },
        this->elem);
  }

  BLI_STRUCT_EQUALITY_OPERATORS_1(ElemVariant, elem)
};

struct SocketElem {
  const bNodeSocket *socket = nullptr;
  ElemVariant elem;

  uint64_t hash() const
  {
    return get_default_hash(this->socket, this->elem);
  }

  BLI_STRUCT_EQUALITY_OPERATORS_2(SocketElem, socket, elem)
};

struct GroupInputElem {
  int group_input_index = 0;
  ElemVariant elem;

  uint64_t hash() const
  {
    return get_default_hash(this->group_input_index, this->elem);
  }

  BLI_STRUCT_EQUALITY_OPERATORS_2(GroupInputElem, group_input_index, elem)
};

struct ValueNodeElem {
  const bNode *node = nullptr;
  ElemVariant elem;

  uint64_t hash() const
  {
    return get_default_hash(this->node, this->elem);
  }

  BLI_STRUCT_EQUALITY_OPERATORS_2(ValueNodeElem, node, elem)
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

class ElemEvalParams {
 private:
  const Map<const bNodeSocket *, ElemVariant> &elem_by_socket_;
  Vector<SocketElem> &output_elems_;

 public:
  const bNode &node;

  ElemEvalParams(const bNode &node,
                 const Map<const bNodeSocket *, ElemVariant> &elem_by_socket,
                 Vector<SocketElem> &output_elems);

  template<typename T> T get_input_elem(const StringRef identifier) const
  {
    const bNodeSocket &socket = node.input_by_identifier(identifier);
    if (const ElemVariant *elem = this->elem_by_socket_.lookup_ptr(&socket)) {
      return std::get<T>(elem->elem);
    }
    return T();
  }

  template<typename T> void set_output_elem(const StringRef identifier, T elem)
  {
    const bNodeSocket &socket = node.output_by_identifier(identifier);
    output_elems_.append({&socket, ElemVariant{elem}});
  }
};

class InverseEvalParams {
 private:
  const Map<const bNodeSocket *, bke::SocketValueVariant> &socket_values_;
  Map<const bNodeSocket *, bke::SocketValueVariant> &updated_socket_values_;

 public:
  const bNode &node;

  InverseEvalParams(const bNode &node,
                    const Map<const bNodeSocket *, bke::SocketValueVariant> &socket_values,
                    Map<const bNodeSocket *, bke::SocketValueVariant> &updated_socket_values);

  template<typename T> T get_output(const StringRef identifier) const
  {
    const bNodeSocket &socket = node.output_by_identifier(identifier);
    if (const bke::SocketValueVariant *value = socket_values_.lookup_ptr(&socket)) {
      return value->get<T>();
    }
    return T();
  }

  template<typename T> T get_input(const StringRef identifier) const
  {
    const bNodeSocket &socket = node.input_by_identifier(identifier);
    if (const bke::SocketValueVariant *value = socket_values_.lookup_ptr(&socket)) {
      return value->get<T>();
    }
    return T();
  }

  template<typename T> void set_input(const StringRef identifier, T value)
  {
    const bNodeSocket &socket = node.input_by_identifier(identifier);
    updated_socket_values_.add(&socket, bke::SocketValueVariant(value));
  }
};

std::optional<ElemVariant> get_elem_variant_for_socket_type(eNodeSocketDatatype type);

}  // namespace blender::nodes::inverse_eval
