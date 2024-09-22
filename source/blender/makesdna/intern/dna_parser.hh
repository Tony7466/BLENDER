/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "BLI_vector.hh"
#include "dna_lexer.hh"

#include <optional>
#include <string>
#include <string_view>
#include <variant>

namespace blender::dna::parser {

namespace ast {

using namespace lex;

/** Constant int defined value. */
struct DefineInt {
  std::string_view name;
  int32_t value{0};

  bool operator==(const DefineInt &other) const;
};

/**
 * Variable declaration, can hold multiple inline declarations, like:
 * `float *value1,value2[256][256];`
 */
struct Variable {
  struct Item {
    std::optional<std::string> ptr;
    std::string_view name;
    /** Item array size definition, empty for not arrays items. */
    Vector<std::variant<std::string_view, int32_t>> array_size;

    bool operator==(const Item &other) const;
  };
  bool const_tag{false};
  std::string_view type;
  Vector<Item> items;

  bool operator==(const Variable &other) const;
};

/** Function pointer declaration. */
struct FunctionPtr {
  bool const_tag{false};
  std::string_view type;
  std::string_view name;

  bool operator==(const FunctionPtr &other) const;
};

/** Pointer to array declaration. */
struct PointerToArray {
  std::string_view type;
  std::string_view name;
  int32_t size;

  bool operator==(const PointerToArray &other) const;
};

/** Struct declaration.*/
struct Struct {
  std::string_view name;
  /** Recursive struct keep inline buffer capacity to `0`. */
  Vector<std::variant<Variable, FunctionPtr, PointerToArray, Struct>, 0> items;
  /** Name set if struct is declared as member variable. */
  std::string_view member_name;

  bool operator==(const Struct &other) const;
};

/** Enum declaration. */
struct Enum {
  /** Enum name, unset for unnamed enums. */
  std::optional<std::string_view> name;
  /** Fixed type specification. */
  std::optional<std::string_view> type;

  bool operator==(const Enum &other) const;
};

using CppType = std::variant<DefineInt, Enum, Struct, FunctionPtr, Variable>;

}  // namespace ast

std::string read_file(std::string_view filepath);

struct CppFile {
  std::string filepath;
  std::string text;
  Vector<ast::CppType> cpp_defs;
};

std::string to_string(const CppFile &cpp_file);

std::optional<CppFile> parse_file(std::string_view filepath);
}  // namespace blender::dna::parser
