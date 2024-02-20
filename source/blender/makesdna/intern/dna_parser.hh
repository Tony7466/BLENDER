/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

#define DEBUG_PRINT_DNA_PARSER true

#include "BLI_vector.hh"

#include <optional>
#include <string_view>
#include <variant>

namespace blender::dna::parser {

namespace ast {
/** Convenient wrapper for unused definitions, as forward declarations or commented structs. */
struct Omitted {
  bool operator==(const Omitted &) const
  {
    return true;
  }
};

/* Constant int defined value. */
struct ConstInt {
  std::string name;
  int32_t value;
  bool operator==(const ConstInt &other) const
  {
    return name == other.name && value == other.value;
  }
};

/**
 * Variable declaration, for convenience defined as type group since multiple variables can be
 * defined in a single line, accepts followings declarations:
 * `int value;`
 * `int value[256][DEFINE_VALUE];`
 * `float value1,value2[256][256];`
 */
struct Variable {

  struct Item {
    /** Pointer specification is owned by each item. */
    std::optional<std::string> ptr;
    std::string name;
    using Size = std::variant<int32_t, std::string>;
    /** Item array size definition, empty for not arrays items. */
    Vector<Size> size;
    bool operator==(const Item &other) const
    {
      return ptr == other.ptr && name == other.name && size == other.size;
    }
  };
  /** Shared const tag for variables declaration. */
  bool const_tag;
  /** Shared type for variables declaration. */
  std::string type;
  Vector<Item> items;
  bool operator==(const Variable &other) const
  {
    return type == other.type && items == other.items;
  }
};

/**
 * Function pointer declaration.
 */
struct FunctionPtr {
  /** Function return type. */
  std::string type;
  std::string name;
  Vector<Variable> params;
  bool operator==(const FunctionPtr &other) const
  {
    return type == other.type && name == other.name && params == other.params;
  }
};
/**
 * Struct declaration.
 */
struct Struct {
  std::string name;
  Vector<std::variant<Variable, FunctionPtr>> items;
  bool operator==(const Struct &other) const
  {
    return name == other.name && items == other.items;
  }
};
/**
 * Enum declaration.
 */
struct Enum {
  struct Item {
    std::string name;
    using Value = std::variant<int32_t, std::string>;
    /* Int value or string expression. */
    std::optional<Value> value;
    bool operator==(const Item &other) const
    {
      return name == other.name && value == other.value;
    }
  };
  /* Enum name, unset for unnamed enums. */
  std::optional<std::string> name;
  /** Fixed type specification. */
  std::optional<std::string> type;
  Vector<Item> items;
  bool operator==(const Enum &other) const
  {
    return name == other.name && type == other.type && items == other.items;
  }
};

using CppType = std::variant<Omitted, ConstInt, Enum, Struct, FunctionPtr>;
}  // namespace ast

bool parse_include(std::string_view text, std::string_view file, Vector<ast::CppType> &c);
}  // namespace blender::dna::parser
