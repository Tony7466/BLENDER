/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#pragma once

#include "DNA_node_types.h"

#include "BLI_any.hh"
#include "BLI_cpp_type.hh"
#include "BLI_generic_pointer.hh"

namespace blender::fn {
class GField;
}

namespace blender::bke {

/**
 * Storage for the socket value float simple types like floats, integers, strings, etc.
 * In geometry nodes, those can be either a single value or field.
 */
struct SocketValueVariant {
  enum class Kind {
    None,
    Single,
    Field,
  };

  Kind kind = Kind::None;
  eNodeSocketDatatype socket_type;
  /**
   * Either empty or contains one of the following:
   * - Simple type: `int`, `float`, `float3`, `std::string`, ...
   * - #GField
   */
  Any<void, 16> value;

  GPointer get_single() const;
  GMutablePointer get_single();

  void store_single(eNodeSocketDatatype socket_type, const void *value);

  template<typename T> T extract_as();
  template<typename T> T get_as() const;
  template<typename T> void store_as(T &&value)
  {
    this->store_as_impl<std::decay_t<T>>(std::decay_t<T>(std::forward<T>(value)));
  }

  bool is_context_dependent_field() const;

  void convert_to_single();
  void *ensure_uninitialized_single(eNodeSocketDatatype socket_type);
  void *ensure_uninitialized_single(const CPPType &cpp_type);

 private:
  template<typename T> void store_as_impl(T value);
};

}  // namespace blender::bke
