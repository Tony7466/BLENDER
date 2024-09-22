/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "dna_parser.hh"
#include <fmt/format.h>
#include <fstream>
#include <iostream>
#include <sstream>

namespace blender::dna::parser {

void to_string(std::stringstream &ss, const ast::Variable &var, size_t /*padding*/)
{
  ss << fmt::format("{}{} ", var.const_tag ? "const " : "", var.type);

  bool first = true;
  for (auto &item : var.items) {
    if (!first) {
      ss << ',';
    }
    first = false;
    ss << fmt::format("{}{}", item.ptr.value_or(""), item.name);
    for (auto &size : item.array_size) {
      std::visit([&ss](auto &&val) { ss << fmt::format("[{}]", val); }, size);
    }
  }
}

void to_string(std::stringstream &ss, const ast::FunctionPtr &fn, size_t /*padding*/)
{
  const std::string const_tag = fn.const_tag ? "const " : "";
  ss << fmt::format("{}{} (*{})(...)", const_tag, fn.type, fn.name);
}

void to_string(std::stringstream &ss, const ast::PointerToArray &ptr, size_t /*padding*/)
{
  ss << fmt::format("{} (*{})[{}]", ptr.type, ptr.name, ptr.size);
}

void to_string(std::stringstream &ss, const ast::DefineInt &val, size_t /*padding*/)
{
  ss << fmt::format("#define {} {}", val.name, val.value);
}

void to_string(std::stringstream &ss, const ast::Enum &val, size_t /*padding*/)
{
  ss << fmt::format("enum {}", val.name.value_or("unnamed"));
  if (val.type) {
    ss << fmt::format(": {}", val.type.value());
  }
  ss << " {...}";
}

void to_string(std::stringstream &ss, const ast::Struct &val, size_t padding)
{
  const auto add_padding = [&ss](size_t padding) {
    ss << fmt::format("{: >{}}", "", padding * 4);
  };

  ss << fmt::format("struct {} {{\n", val.name);
  for (auto &item : val.items) {
    add_padding(padding + 1);
    std::visit([&ss, padding](auto &&val) { to_string(ss, val, padding + 1); }, item);
    ss << ";\n";
  }
  add_padding(padding);
  ss << '}';
}

std::string to_string(const CppFile &cpp_file)
{
  std::stringstream ss;
  for (auto &cpp_def : cpp_file.cpp_defs) {
    std::visit([&ss](auto &&cpp_def) { to_string(ss, cpp_def, 0); }, cpp_def);
    ss << ";\n";
  }
  return ss.str();
}

}  // namespace blender::dna::parser

namespace blender::dna::parser::ast {

struct ParseFailedResult {};

static constexpr ParseFailedResult parse_failed{};

template<typename T> class ParseResult {
 public:
  using value_type = T;

 private:
  std::optional<T> data_;

 public:
  ParseResult() = delete;
  ParseResult(ParseFailedResult) : data_{std::nullopt} {};
  ParseResult(T &&value) : data_{std::move(value)} {};

  bool success() const
  {
    return data_.has_value();
  }

  bool fail() const
  {
    return data_.has_value();
  }

  T &value() &
  {
    BLI_assert(success());
    return data_.value();
  }

  const T &value() const &
  {
    BLI_assert(success());
    return data_.value();
  }

  T &&value() &&
  {
    BLI_assert(success());
    return std::move(data_.value());
  }
};

template<typename T> struct Parser {
  static ParseResult<T> parse()
  {
    BLI_assert_unreachable();
    return parse_failed;
  }
};

template<typename T> ParseResult<T> parse_t(TokenIterator &token_iterator)
{
  return Parser<T>::parse(token_iterator);
};

template<typename T> bool parse_t_success(TokenIterator &token_iterator)
{
  return Parser<T>::parse(token_iterator).success();
};

template<typename T> bool parse_t_fail(TokenIterator &token_iterator)
{
  return Parser<T>::parse(token_iterator).success();
};

/**
 * Parser that matches a sequence of elements to parse, fails if any `Args` in `Args...` fails to
 * parse.
 * The sequence: `Sequence<HashSymbol, PragmaKeyword, OnceKeyword>` parses when the
 * text contains `#pragma once`.
 */
template<class... Args> using Sequence = std::tuple<Args...>;
template<class... Args> struct Parser<Sequence<Args...>> {

 private:
  template<std::size_t I>
  static inline bool parse_idx(Sequence<Args...> &sequence, TokenIterator &token_iterator)
  {
    using T = std::tuple_element_t<I, std::tuple<Args...>>;
    ParseResult<T> val = parse_t<T>(token_iterator);
    if (val.success()) {
      std::get<I>(sequence) = std::move(val.value());
    }
    return val.success();
  };

  template<std::size_t... I>
  static inline bool parse_impl(Sequence<Args...> &sequence,
                                std::index_sequence<I...> /*indices*/,
                                TokenIterator &token_iterator)
  {
    return (parse_idx<I>(sequence, token_iterator) && ...);
  };

 public:
  static ParseResult<Sequence<Args...>> parse(TokenIterator &token_iterator)
  {
    token_iterator.push_waypoint();
    Sequence<Args...> sequence{};
    const bool success = parse_impl(sequence, std::index_sequence_for<Args...>{}, token_iterator);
    token_iterator.end_waypoint(success);
    if (success) {
      return sequence;
    }
    return parse_failed;
  }
};

/**
 * Parser that don't fails if `Type` can't be parsed.
 * The sequence `Sequence<Optional<ConstKeyword>, IntKeyword, Identifier, SemicolonSymbol>` success
 * either if text is `const int num;` or `int num;`
 */
template<typename T> using Optional = std::optional<T>;
template<typename T> struct Parser<Optional<T>> {
 public:
  static ParseResult<Optional<T>> parse(TokenIterator &token_iterator)
  {
    token_iterator.push_waypoint();
    ParseResult<T> result = parse_t<T>(token_iterator);
    token_iterator.end_waypoint(result.success());
    if (result.success()) {
      return Optional<T>{std::move(result.value())};
    }
    return Optional<T>{std::nullopt};
  }
};

/**
 * Parser that tries to match any `Arg` in `Args...`
 * The sequence `Sequence<Variant<IntKeyword, FloatKeyword>, Identifier, SemicolonSymbol>`
 * success either if text is `int num;` or `float num;`
 */
template<class... Args> using Variant = std::variant<Args...>;
template<class... Args> struct Parser<Variant<Args...>> {

 private:
  template<typename T>
  static bool parse_variant(Variant<Args...> &variant, TokenIterator &token_iterator)
  {
    token_iterator.push_waypoint();
    ParseResult<T> val = parse_t<T>(token_iterator);
    if (val.success()) {
      variant.template emplace<T>(std::move(val.value()));
    }
    token_iterator.end_waypoint(val.success());
    return val.success();
  };

 public:
  static ParseResult<Variant<Args...>> parse(TokenIterator &token_iterator)
  {
    Variant<Args...> variant{};
    if ((parse_variant<Args>(variant, token_iterator) || ...)) {
      return variant;
    }
    return parse_failed;
  }
};

/** Keyword parser. */
template<KeywordType Type> struct Keyword {};
template<KeywordType Type> struct Parser<Keyword<Type>> {
  static ParseResult<Keyword<Type>> parse(TokenIterator &token_iterator)
  {
    if (token_iterator.next_keyword(Type)) {
      return Keyword<Type>{};
    }
    return parse_failed;
  }
};

using ConstKeyword = Keyword<KeywordType::CONST>;
using IncludeKeyword = Keyword<KeywordType::INCLUDE>;
using StructKeyword = Keyword<KeywordType::STRUCT>;
using DefineKeyword = Keyword<KeywordType::DEFINE>;
using UnsignedKeyword = Keyword<KeywordType::UNSIGNED>;
using IntKeyword = Keyword<KeywordType::INT>;
using Int8Keyword = Keyword<KeywordType::INT8_T>;
using Int16Keyword = Keyword<KeywordType::INT16_T>;
using Int32Keyword = Keyword<KeywordType::INT32_T>;
using Int64Keyword = Keyword<KeywordType::INT64_T>;
using UInt8Keyword = Keyword<KeywordType::UINT8_T>;
using UInt16Keyword = Keyword<KeywordType::UINT16_T>;
using UInt32Keyword = Keyword<KeywordType::UINT32_T>;
using UInt64Keyword = Keyword<KeywordType::UINT64_T>;
using FloatKeyword = Keyword<KeywordType::FLOAT>;
using DoubleKeyword = Keyword<KeywordType::DOUBLE>;
using ShortKeyword = Keyword<KeywordType::SHORT>;
using CharKeyword = Keyword<KeywordType::CHAR>;
using VoidKeyword = Keyword<KeywordType::VOID>;
using IfKeyword = Keyword<KeywordType::IF>;
using IfDefKeyword = Keyword<KeywordType::IFDEF>;
using IfnDefKeyword = Keyword<KeywordType::IFNDEF>;
using EndIfKeyword = Keyword<KeywordType::ENDIF>;
using ExternKeyword = Keyword<KeywordType::EXTERN>;
using TypedefKeyword = Keyword<KeywordType::TYPEDEF>;
using PragmaKeyword = Keyword<KeywordType::PRAGMA>;
using OnceKeyword = Keyword<KeywordType::ONCE>;
using EnumKeyword = Keyword<KeywordType::ENUM>;
using ClassKeyword = Keyword<KeywordType::CLASS>;
using DNADeprecatedKeyword = Keyword<KeywordType::DNA_DEPRECATED>;
using DNADeprecatedAllowKeyword = Keyword<KeywordType::DNA_DEPRECATED_ALLOW>;

/** Symbol parser. */
template<SymbolType type> struct Symbol {};
template<SymbolType Type> struct Parser<Symbol<Type>> {
  static ParseResult<Symbol<Type>> parse(TokenIterator &token_iterator)
  {
    if (token_iterator.next_symbol(Type)) {
      return Symbol<Type>{};
    }
    return parse_failed;
  }
};

using LBracketSymbol = Symbol<SymbolType::LBRACKET>;
using RBracketSymbol = Symbol<SymbolType::RBRACKET>;
using LBraceSymbol = Symbol<SymbolType::LBRACE>;
using RBraceSymbol = Symbol<SymbolType::RBRACE>;
using LParenSymbol = Symbol<SymbolType::LPAREN>;
using RParenSymbol = Symbol<SymbolType::RPAREN>;
using StarSymbol = Symbol<SymbolType::STAR>;
using SemicolonSymbol = Symbol<SymbolType::SEMICOLON>;
using ColonSymbol = Symbol<SymbolType::COLON>;
using CommaSymbol = Symbol<SymbolType::COMMA>;
using HashSymbol = Symbol<SymbolType::HASH>;
using LessSymbol = Symbol<SymbolType::LESS>;
using GreaterSymbol = Symbol<SymbolType::GREATER>;
using AssignSymbol = Symbol<SymbolType::ASSIGN>;
using MinusSymbol = Symbol<SymbolType::MINUS>;

static void skip_until_match_paired_symbols(SymbolType left,
                                            SymbolType right,
                                            TokenIterator &token_iterator);

/**
 * Parses a macro call, `MacroCall<KeywordType::DNA_DEFINE_CXX_METHODS>` parses
 * `DNA_DEFINE_CXX_METHODS(...)`.
 */
template<KeywordType Type> struct MacroCall {};
template<KeywordType Type> struct Parser<MacroCall<Type>> {
  static ParseResult<MacroCall<Type>> parse(TokenIterator &token_iterator)
  {
    if (parse_t<Sequence<Keyword<Type>, LParenSymbol>>(token_iterator).success()) {
      skip_until_match_paired_symbols(SymbolType::LPAREN, SymbolType::RPAREN, token_iterator);
      parse_t<SemicolonSymbol>(token_iterator);
      return MacroCall<Type>{};
    }
    return parse_failed;
  }
};

/** Parses a string literal. */
struct StringLiteral {
  std::string_view value;
};
template<> struct Parser<StringLiteral> {
  static ParseResult<StringLiteral> parse(TokenIterator &token_iterator)
  {
    if (StringLiteralToken *literal = token_iterator.next<StringLiteralToken>(); literal) {
      return StringLiteral{literal->where};
    }
    return parse_failed;
  }
};

/** Parses a int literal. */
struct IntLiteral {
  int value;
};
template<> struct Parser<IntLiteral> {
  static ParseResult<IntLiteral> parse(TokenIterator &token_iterator)
  {
    if (IntLiteralToken *value = token_iterator.next<IntLiteralToken>(); value) {
      return IntLiteral{value->val};
    }
    return parse_failed;
  }
};

/** Parses a identifier. */
struct Identifier {
  std::string_view str;
};
template<> struct Parser<Identifier> {
  static ParseResult<Identifier> parse(TokenIterator &token_iterator)
  {
    if (IdentifierToken *identifier = token_iterator.next<IdentifierToken>(); identifier) {
      return Identifier{identifier->where};
    }
    return parse_failed;
  }
};

/** Parses a include, either `#include "include_name.hh"` or `#include <path/to/include.hh>`. */
struct Include {};
template<> struct Parser<Include> {
  static ParseResult<Include> parse(TokenIterator &token_iterator)
  {
    if (parse_t<Sequence<HashSymbol, IncludeKeyword>>(token_iterator).success()) {
      TokenVariant *token = token_iterator.next_variant();
      while (token && !std::holds_alternative<BreakLineToken>(*token)) {
        token = token_iterator.next_variant();
      }
      return Include{};
    }
    return parse_failed;
  }
};

/** Check if a token is a symbol and has a type. */
static bool inline is_symbol_type(const TokenVariant &token, const SymbolType type)
{
  return std::holds_alternative<SymbolToken>(token) && std::get<SymbolToken>(token).type == type;
}

/** Parses `#define` directives except to const int defines. */
struct Define {};
template<> struct Parser<Define> {
  static ParseResult<Define> parse(TokenIterator &token_iterator)
  {
    if (!parse_t<Sequence<HashSymbol, DefineKeyword>>(token_iterator).success()) {
      return parse_failed;
    }
    bool scape_bl = false;
    for (TokenVariant *token = token_iterator.next_variant(); token;
         token = token_iterator.next_variant())
    {
      if (std::holds_alternative<BreakLineToken>(*token) && !scape_bl) {
        break;
      }
      scape_bl = is_symbol_type(*token, SymbolType::BACKSLASH);
    }
    return Define{};
  }
};

/** Parses const int defines, like `#define FILE_MAX 1024`. */
template<> struct Parser<DefineInt> {
  static ParseResult<DefineInt> parse(TokenIterator &token_iterator)
  {
    using DefineConstIntSeq = Sequence<HashSymbol, DefineKeyword, Identifier, IntLiteral>;
    ParseResult<DefineConstIntSeq> val = parse_t<DefineConstIntSeq>(token_iterator);
    if (!val.success() || !token_iterator.next<BreakLineToken>()) {
      return parse_failed;
    }
    return DefineInt{std::move(std::get<2>(val.value()).str), std::get<3>(val.value()).value};
  }
};

bool DefineInt::operator==(const DefineInt &other) const
{
  return name == other.name && value == other.value;
}

/** Parses most c++ primitive types. */
struct PrimitiveType {
  std::string_view str;
};
template<> struct Parser<PrimitiveType> {
  static ParseResult<PrimitiveType> parse(TokenIterator &token_iterator)
  {
    /* TODO: Add all primitive types. */
    using PrimitiveTypeVariants = Variant<IntKeyword,
                                          CharKeyword,
                                          ShortKeyword,
                                          FloatKeyword,
                                          DoubleKeyword,
                                          VoidKeyword,
                                          Sequence<UnsignedKeyword, IntKeyword>,
                                          Sequence<UnsignedKeyword, ShortKeyword>,
                                          Sequence<UnsignedKeyword, CharKeyword>,
                                          Int8Keyword,
                                          Int16Keyword,
                                          Int32Keyword,
                                          Int64Keyword,
                                          UInt8Keyword,
                                          UInt16Keyword,
                                          UInt32Keyword,
                                          UInt64Keyword,
                                          Keyword<KeywordType::LONG>,
                                          Keyword<KeywordType::ULONG>>;
    ParseResult<PrimitiveTypeVariants> type = parse_t<PrimitiveTypeVariants>(token_iterator);
    if (!type.success()) {
      return parse_failed;
    }

    /* Use `unsigned int` as uint32?.... */
    /* Note: makesdna ignores `unsigned` keyword. */
    static constexpr const char *primitive_types[]{
        "int",      "char",     "short",    "float",   "double",  "void",    "int",
        "short",    "char",     "int8_t",   "int16_t", "int32_t", "int64_t", "uint8_t",
        "uint16_t", "uint32_t", "uint64_t", "long",    "ulong",
    };
    return PrimitiveType{primitive_types[type.value().index()]};
  }
};

/**
 * Parses the type in variable declarations or function return value, either a primitive type or
 * custom type.
 */
struct Type {
  bool const_tag{false};
  std::string_view str;
};
template<> struct Parser<Type> {
  static ParseResult<Type> parse(TokenIterator &token_iterator)
  {
    using TypeVariant = Variant<PrimitiveType, Sequence<Optional<StructKeyword>, Identifier>>;
    using TypeSequence = Sequence<Optional<ConstKeyword>, TypeVariant>;

    ParseResult<TypeSequence> type_seq = parse_t<TypeSequence>(token_iterator);
    if (!type_seq.success()) {
      return parse_failed;
    }
    const bool const_tag = std::get<0>(type_seq.value()).has_value();
    TypeVariant &type_variant = std::get<1>(type_seq.value());
    if (std::holds_alternative<PrimitiveType>(type_variant)) {
      return Type{const_tag, std::get<0>(type_variant).str};
    }
    return Type{const_tag, std::move(std::get<1>(std::get<1>(type_variant)).str)};
  }
};

/**
 * Parses variable array size declarations: in `int num[3][4][FILE_MAX];`
 * parses `[3][4][FILE_MAX]`.
 * */
static Vector<std::variant<std::string_view, int32_t>> variable_array_size_parse(
    TokenIterator &token_iterator)
{
  Vector<std::variant<std::string_view, int32_t>> result;
  /* Dynamic array. */
  if (parse_t<Sequence<LBracketSymbol, RBracketSymbol>>(token_iterator).success()) {
    result.append("");
  }
  while (true) {
    using ArraySize = Sequence<LBracketSymbol, Variant<IntLiteral, Identifier>, RBracketSymbol>;
    ParseResult<ArraySize> size_seq = parse_t<ArraySize>(token_iterator);
    if (!size_seq.success()) {
      break;
    }
    auto &item_size = std::get<1>(size_seq.value());
    if (std::holds_alternative<IntLiteral>(item_size)) {
      result.append(std::get<IntLiteral>(item_size).value);
    }
    else {
      result.append(std::move(std::get<Identifier>(item_size).str));
    }
  }
  return result;
}

/**
 * Variable parser, parses multiple inline declarations, like:
 * `int value;`
 * `const int value[256][DEFINE_VALUE];`
 * `float *value1,value2[256][256];`
 */
template<> struct Parser<Variable> {
  static ParseResult<Variable> parse(TokenIterator &token_iterator)
  {
    ParseResult<Type> type = parse_t<Type>(token_iterator);
    if (!type.success()) {
      return parse_failed;
    }
    Variable variable{};
    variable.const_tag = type.value().const_tag;
    variable.type = type.value().str;

    while (true) {
      std::string start;
      for (; parse_t<StarSymbol>(token_iterator).success();) {
        start += '*';
      }
      ParseResult<Identifier> name = parse_t<Identifier>(token_iterator);
      if (!name.success()) {
        return parse_failed;
      }
      Variable::Item item{};
      item.ptr = !start.empty() ? std::optional{start} : std::nullopt;
      item.name = name.value().str;
      item.array_size = variable_array_size_parse(token_iterator);
      variable.items.append(std::move(item));
      parse_t<DNADeprecatedKeyword>(token_iterator);
      if (parse_t<SemicolonSymbol>(token_iterator).success()) {
        break;
      }
      if (!parse_t<CommaSymbol>(token_iterator).success()) {
        return parse_failed;
      }
    }
    return variable;
  }
};

bool Variable::Item::operator==(const Variable::Item &other) const
{
  return ptr == other.ptr && name == other.name && array_size == other.array_size;
}

bool Variable::operator==(const Variable &other) const
{
  return type == other.type && items == other.items;
}

/* Skips tokens until match the closing right symbol, like function body braces `{...}`. */
static void skip_until_match_paired_symbols(SymbolType left,
                                            SymbolType right,
                                            TokenIterator &token_iterator)
{
  int left_count = 1;
  for (TokenVariant *token = token_iterator.next_variant(); token;
       token = token_iterator.next_variant())
  {
    if (is_symbol_type(*token, right)) {
      left_count--;
      if (left_count == 0) {
        break;
      }
    }
    else if (is_symbol_type(*token, left)) {
      left_count++;
    }
  }
};

/**
 * Parses function pointer variables, like `bool (*poll)(struct bContext *);`
 */
template<> struct Parser<FunctionPtr> {
  static ParseResult<FunctionPtr> parse(TokenIterator &token_iterator)
  {
    using FunctionPtrBegin = Sequence<Type,
                                      Optional<StarSymbol>,
                                      LParenSymbol,
                                      StarSymbol,
                                      Identifier,
                                      RParenSymbol,
                                      LParenSymbol>;
    const ParseResult<FunctionPtrBegin> fn = parse_t<FunctionPtrBegin>(token_iterator);
    if (!fn.success()) {
      return parse_failed;
    }
    FunctionPtr fn_ptr{};
    fn_ptr.const_tag = std::get<0>(fn.value()).const_tag;
    fn_ptr.type = std::get<0>(fn.value()).str;
    fn_ptr.name = std::get<4>(fn.value()).str;
    /* Skip Function params. */
    skip_until_match_paired_symbols(SymbolType::LPAREN, SymbolType::RPAREN, token_iterator);

    /* Closing sequence. */
    if (!parse_t<SemicolonSymbol>(token_iterator).success()) {
      return parse_failed;
    }
    return fn_ptr;
  }
};

bool FunctionPtr::operator==(const FunctionPtr &other) const
{
  return type == other.type && name == other.name;
}

bool PointerToArray::operator==(const PointerToArray &other) const
{
  return type == other.type && name == other.name && size == other.size;
}

/**
 * Parses array pointer variables, like `float (*vert_coords_prev)[3];`
 */
template<> struct Parser<PointerToArray> {
  static ParseResult<PointerToArray> parse(TokenIterator &token_iterator)
  {
    using PointerToArraySequence = Sequence<Type,
                                            LParenSymbol,
                                            StarSymbol,
                                            Identifier,
                                            RParenSymbol,
                                            LBracketSymbol,
                                            IntLiteral,
                                            RBracketSymbol,
                                            SemicolonSymbol>;
    ParseResult<PointerToArraySequence> val = parse_t<PointerToArraySequence>(token_iterator);
    if (!val.success()) {
      return parse_failed;
    }
    PointerToArray ptr{};
    ptr.type = std::get<0>(val.value()).str;
    ptr.name = std::get<3>(val.value()).str;
    ptr.size = std::get<6>(val.value()).value;
    return ptr;
  }
};

template<KeywordType... type> static bool is_keyword_type(TokenVariant token)
{
  return std::holds_alternative<KeywordToken>(token) &&
         ((std::get<KeywordToken>(token).type == type) || ...);
}

/**
 * Parses `#if....#endif` code blocks.
 */
struct IfDef {};
template<> struct Parser<IfDef> {
  static ParseResult<IfDef> parse(TokenIterator &token_iterator)
  {
    using IfDefBeginSequence =
        Sequence<HashSymbol, Variant<IfDefKeyword, IfKeyword, IfnDefKeyword>>;
    const ParseResult<IfDefBeginSequence> val = parse_t<IfDefBeginSequence>(token_iterator);
    if (!val.success()) {
      return parse_failed;
    };
    int ifdef_deep = 1;
    bool hash_carried = false;
    for (TokenVariant *token = token_iterator.next_variant(); token;
         token = token_iterator.next_variant())
    {
      if (hash_carried &&
          is_keyword_type<KeywordType::IF, KeywordType::IFDEF, KeywordType::IFNDEF>(*token))
      {
        ifdef_deep++;
      }
      if (hash_carried && is_keyword_type<KeywordType::ENDIF>(*token)) {
        ifdef_deep--;
      }
      if (ifdef_deep == 0) {
        break;
      }
      hash_carried = is_symbol_type(*token, SymbolType::HASH);
    }
    /* Not matching #endif. */
    if (ifdef_deep != 0) {
      return parse_failed;
    }
    return IfDef{};
  }
};

/**
 * Parses struct declarations.
 */
template<> struct Parser<Struct> {
  static ParseResult<Struct> parse(TokenIterator &token_iterator)
  {
    using StructBeginSequence =
        Sequence<Optional<TypedefKeyword>, StructKeyword, Optional<Identifier>, LBraceSymbol>;
    ParseResult<StructBeginSequence> struct_seq = parse_t<StructBeginSequence>(token_iterator);
    if (!struct_seq.success()) {
      return parse_failed;
    }
    Struct result{};
    if (std::get<2>(struct_seq.value()).has_value()) {
      result.name = std::get<2>(struct_seq.value()).value().str;
    }
    while (true) {
      using DNA_DEF_CCX_Macro = MacroCall<lex::KeywordType::DNA_DEFINE_CXX_METHODS>;

      if (auto member = parse_t<Variant<Variable, FunctionPtr, PointerToArray, Struct>>(
              token_iterator);
          member.success())
      {
        result.items.append(std::move(member.value()));
      }
      else if (parse_t<Variant<DNA_DEF_CCX_Macro, IfDef>>(token_iterator).success()) {
      }
      else {
        break;
      }
    }
    using StructEndSequence = Sequence<RBraceSymbol, Optional<Identifier>, SemicolonSymbol>;
    ParseResult<StructEndSequence> struct_end = parse_t<StructEndSequence>(token_iterator);
    if (!struct_end.success()) {
      return parse_failed;
    }
    if (std::get<1>(struct_end.value()).has_value()) {
      result.member_name = std::get<1>(struct_end.value()).value().str;
    }
    if (result.member_name == result.name && result.name.empty()) {
      return parse_failed;
    }
    return result;
  }
};

bool Struct::operator==(const Struct &other) const
{
  return name == other.name && items == other.items;
}

/** Parses skipped declarations by makesdna. */
struct Skip {};
template<> struct Parser<Skip> {
  static ParseResult<Skip> parse(TokenIterator &token_iterator)
  {
    using UnusedDeclarations =
        Variant<Define,
                Include,
                IfDef,
                Sequence<HashSymbol, PragmaKeyword, OnceKeyword>,
                Sequence<HashSymbol, HashSymbol, Struct>,
                Sequence<ExternKeyword, Variable>,
                MacroCall<lex::KeywordType::BLI_STATIC_ASSERT_ALIGN>,
                MacroCall<lex::KeywordType::ENUM_OPERATORS>,
                Sequence<TypedefKeyword, StructKeyword, Identifier, Identifier, SemicolonSymbol>>;
    if (parse_t<UnusedDeclarations>(token_iterator).success()) {
      return Skip{};
    }
    /* Forward declarations. */
    if (parse_t<Sequence<StructKeyword, Identifier>>(token_iterator).success()) {
      for (; parse_t<Sequence<CommaSymbol, Identifier>>(token_iterator).success();) {
      }
      if (parse_t<SemicolonSymbol>(token_iterator).success()) {
        return Skip{};
      }
    }
    else if (token_iterator.next<BreakLineToken>()) {
      return Skip{};
    }
    return parse_failed;
  }
};

/** Parse enums, with a name or not and with a fixed type or not. */
template<> struct Parser<Enum> {
  static ParseResult<Enum> parse(TokenIterator &token_iterator)
  {
    using EnumBeginSequence = Sequence<Optional<TypedefKeyword>,
                                       EnumKeyword,
                                       Optional<ClassKeyword>,
                                       Optional<Identifier>,
                                       Optional<Sequence<ColonSymbol, PrimitiveType>>,
                                       LBraceSymbol>;
    ParseResult<EnumBeginSequence> enum_begin = parse_t<EnumBeginSequence>(token_iterator);
    if (!enum_begin.success()) {
      return parse_failed;
    }
    Enum enum_def;
    if (std::get<3>(enum_begin.value()).has_value()) {
      enum_def.name = std::get<3>(enum_begin.value()).value().str;
    }
    if (std::get<4>(enum_begin.value()).has_value()) {
      enum_def.type = std::get<1>(std::get<4>(enum_begin.value()).value()).str;
    }
    /* Skip enum body. */
    skip_until_match_paired_symbols(SymbolType::LBRACE, SymbolType::RBRACE, token_iterator);

    /* Enum end sequence. */
    if (!parse_t<Sequence<Optional<Identifier>, Optional<DNADeprecatedKeyword>, SemicolonSymbol>>(
             token_iterator)
             .success())
    {
      return parse_failed;
    }
    return enum_def;
  }
};
bool Enum::operator==(const Enum &other) const
{
  return name == other.name && type == other.type;
}

}  // namespace blender::dna::parser::ast

namespace blender::dna::parser {

static void print_unhandled_token_error(std::string_view filepath,
                                        std::string_view text,
                                        lex::TokenVariant *what)
{
  const auto visit_fn = [text, filepath](auto &&token) {
    std::string_view::iterator itr = text.begin();
    size_t line = 1;
    while (itr < token.where.begin()) {
      if (itr[0] == '\n') {
        line++;
      }
      itr++;
    }
    fmt::print("{}{} Unhandled token: \"{}\"\n", filepath, line, token.where);
  };
  std::visit(visit_fn, *what);
}

std::string read_file(std::string_view filepath)
{
  std::ifstream file(filepath.data());
  if (!file.is_open()) {
    fprintf(stderr, "Can't read file %s\n", filepath.data());
    return "";
  }
  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

std::optional<CppFile> parse_file(std::string_view filepath)
{
  using namespace ast;
  CppFile cpp_file;
  cpp_file.text = read_file(filepath);
  /* Generate tokens. */
  lex::TokenIterator token_iterator;
  token_iterator.process_text(filepath, cpp_file.text);

  int dna_deprecated_allow_count = 0;
  using DNADeprecatedAllowSeq = Sequence<HashSymbol, IfDefKeyword, DNADeprecatedAllowKeyword>;
  using EndIfSeq = Sequence<HashSymbol, EndIfKeyword>;

  while (!token_iterator.has_finish()) {
    using CPPTypeVariant = Variant<Struct,
                                   Enum,
                                   Sequence<Optional<TypedefKeyword>, FunctionPtr>,
                                   Variable,
                                   DefineInt,
                                   DNADeprecatedAllowSeq,
                                   EndIfSeq,
                                   Skip>;
    ParseResult<CPPTypeVariant> val = parse_t<CPPTypeVariant>(token_iterator);
    if (!val.success()) {
      print_unhandled_token_error(filepath, cpp_file.text, token_iterator.last_unmatched);
      return std::nullopt;
    }
    if (std::holds_alternative<Struct>(val.value())) {
      cpp_file.cpp_defs.append(std::move(std::get<Struct>(val.value())));
    }
    else if (std::holds_alternative<DefineInt>(val.value())) {
      cpp_file.cpp_defs.append(std::move(std::get<DefineInt>(val.value())));
    }
    else if (std::holds_alternative<Variable>(val.value())) {
      continue;
      cpp_file.cpp_defs.append(std::move(std::get<Variable>(val.value())));
    }
    else if (std::holds_alternative<Enum>(val.value())) {
      Enum &enum_def = std::get<Enum>(val.value());
      /* Keep only named enums with fixed type. */
      if (!enum_def.name.has_value() || !enum_def.type.has_value()) {
        continue;
      }
      cpp_file.cpp_defs.append(std::move(std::get<Enum>(val.value())));
    }
    else if (std::holds_alternative<DNADeprecatedAllowSeq>(val.value())) {
      dna_deprecated_allow_count++;
    }
    else if (std::holds_alternative<EndIfSeq>(val.value())) {
      dna_deprecated_allow_count--;
      BLI_assert(dna_deprecated_allow_count >= 0);
    }
  }
  constexpr std::string_view debug_file = "DNA_action_types.h";
  if (!debug_file.empty() && filepath.find(debug_file) != filepath.npos) {
    printf("%s", to_string(cpp_file).c_str());
  }
  return cpp_file;
}

}  // namespace blender::dna::parser
