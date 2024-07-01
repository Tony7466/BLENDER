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

#include "NOD_value_elem.hh"

namespace blender::nodes::inverse_eval {

using namespace value_elem;

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

}  // namespace blender::nodes::inverse_eval
