/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

#define DEBUG_PRINT_DNA_PARSER true

#include "BLI_vector.hh"
#include "dna_lexer.hh"

#include <optional>
#include <string_view>
#include <variant>

namespace blender::dna::parser {

namespace ast {
using namespace lex;

/* Constant int defined value. */
struct ConstInt {
  std::string_view name;
  int32_t value{0};

  static std::optional<ConstInt> parse(TokenIterator &cont);

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
    std::string_view name;
    using Size = std::variant<std::string_view, int32_t>;
    /** Item array size definition, empty for not arrays items. */
    Vector<Size> size;
    bool operator==(const Item &other) const
    {
      return ptr == other.ptr && name == other.name && size == other.size;
    }
  };
  bool const_tag{false};
  /** Shared type for variables declaration. */
  std::string_view type;
  Vector<Item> items;
  bool operator==(const Variable &other) const
  {
    return type == other.type && items == other.items;
  }
  static std::optional<Variable> parse(TokenIterator &cont);
};

/* Function pointer declaration. */
struct FunctionPtr {

  bool const_tag{false};
  std::string_view type;
  std::string_view name;
  Vector<Variable> params;
  bool operator==(const FunctionPtr &other) const
  {
    return type == other.type && name == other.name && params == other.params;
  }
  static std::optional<FunctionPtr> parse(TokenIterator &cont);
};

/* Struct declaration.*/
struct Struct {
  std::string_view name;
  Vector<std::variant<Variable, FunctionPtr>> items;
  bool operator==(const Struct &other) const
  {
    return name == other.name && items == other.items;
  }
  static std::optional<Struct> parse(TokenIterator &cont);
};

/* Enum declaration. */
struct Enum {
  /* Enum name, unset for unnamed enums. */
  std::optional<std::string_view> name;
  /** Fixed type specification. */
  std::optional<std::string_view> type;
  bool operator==(const Enum &other) const
  {
    return name == other.name && type == other.type;
  }
  static std::optional<Enum> parse(TokenIterator &cont);
};

using CppType = std::variant<ConstInt, Enum, Struct, FunctionPtr, Variable>;

}  // namespace ast

bool parse_include(std::string_view filepath,
                   std::string_view text,
                   lex::TokenIterator &cont,
                   Vector<ast::CppType> &c);
}  // namespace blender::dna::parser
