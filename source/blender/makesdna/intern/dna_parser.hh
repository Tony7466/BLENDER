/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "BLI_string_ref.hh"
#include "BLI_vector.hh"

#include <optional>
#include <string>
#include <variant>

namespace blender::dna::parser {

namespace ast {

/** Constant int defined value. */
struct DefineInt {
  StringRef name;
  int32_t value = 0;
};

/**
 * Variable declaration, can hold multiple inline declarations, like:
 * `float *value1,value2[256][256];`
 */
struct Variable {
  struct Item {
    std::optional<std::string> ptr;
    StringRef name;
    /** Item array size definition, empty for not arrays items. */
    Vector<std::variant<StringRef, int32_t>> array_size;
  };
  bool const_tag = false;
  StringRef type;
  Vector<Item> items;
};

/** Function pointer declaration. */
struct FunctionPtr {
  bool const_tag = false;
  StringRef type;
  StringRef name;
};

/** Pointer to array declaration. */
struct PointerToArray {
  StringRef type;
  StringRef name;
  int32_t size;
};

/** Struct declaration. */
struct Struct {
  StringRef name;
  /** Recursive struct keep inline buffer capacity to `0`. */
  Vector<std::variant<Variable, FunctionPtr, PointerToArray, Struct>, 0> items;
  /** Name set if struct is declared as member variable. */
  StringRef member_name;
};

/** Enum declaration. */
struct Enum {
  /** Enum name, unset for unnamed enums. */
  std::optional<StringRef> name;
  /** Fixed type specification. */
  std::optional<StringRef> type;
};

using CppType = std::variant<DefineInt, Enum, Struct, FunctionPtr, Variable>;

}  // namespace ast

std::string read_file(StringRef filepath);

struct CppFile {
  std::string filepath;
  std::string text;
  Vector<ast::CppType> cpp_defs;
};

std::string to_string(const CppFile &cpp_file);

std::optional<CppFile> parse_file(StringRef filepath);
}  // namespace blender::dna::parser
