/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "dna_parser.hh"
#include <fmt/format.h>

// #define DEBUG_PRINT_DNA_PARSER

#ifdef DEBUG_PRINT_DNA_PARSER
namespace blender::dna::parser {

void printf_struct(const ast::Struct &val, size_t padding);

struct StructMemberPrinter {
  size_t padding;

  void operator()(const ast::Variable &var) const
  {
    const std::string const_tag = var.const_tag ? "const " : "";
    fmt::print("{}{} ", const_tag, var.type);

    bool first = true;
    for (auto &variable_item : var.items) {
      if (!first) {
        printf(",");
      }
      first = false;
      fmt::print("{}{}", variable_item.ptr.value_or(""), variable_item.name);
      for (auto &size : variable_item.size) {
        if (std::holds_alternative<std::string_view>(size)) {
          fmt::print("[{}]", std::get<std::string_view>(size));
        }
        else {
          fmt::print("[{}]", std::get<int32_t>(size));
        }
      }
    }
  }

  void operator()(const ast::FunctionPtr &fn) const
  {
    const std::string const_tag = fn.const_tag ? "const " : "";
    fmt::print("{}{} (*{})(...)", const_tag, fn.type, fn.name);
  }

  void operator()(const ast::PointerToArray &ptr) const
  {
    fmt::print("{} (*{})[{}]", ptr.type, ptr.name, ptr.size);
  }

  void operator()(const ast::Struct &val) const
  {
    printf_struct(val, padding);
  }
};

struct ParserDebugPrinter {
  size_t padding;

  void operator()(const ast::DefineInt &val) const
  {
    fmt::print("#define {} {}\n", val.name, val.value);
  }

  void operator()(const ast::Enum &val) const
  {
    fmt::print("enum {}", val.name.value_or("unnamed"));
    if (val.type) {
      fmt::print(": {}", val.type.value());
    }
    printf(" {...};\n");
  }

  void operator()(const ast::Struct &val) const
  {
    printf_struct(val, padding);
    printf(";\n");
  }

  void operator()(const ast::FunctionPtr &fn) const
  {
    StructMemberPrinter{padding + 1}.operator()(fn);
    printf("\n");
  }

  void operator()(const ast::Variable &var) const
  {
    StructMemberPrinter{padding + 1}.operator()(var);
    printf("\n");
  }
};

void printf_struct(const ast::Struct &val, size_t padding)
{
  const auto print_padding = [](size_t padding) { fmt::print("{: >{}}", "", padding * 4); };

  fmt::print("struct {} {{\n", val.name);
  for (auto &item : val.items) {
    print_padding(padding + 1);
    std::visit(StructMemberPrinter{padding + 1}, item);
    printf(";\n");
  }
  print_padding(padding);
  printf("}");
};

}  // namespace blender::dna::parser
#endif

namespace blender::dna::parser::ast {

/**
 * Parser that matches a sequence of elements to parse, fails if any `Args` in `Args...` fails to
 * parse.
 * The sequence: `Sequence<HashSymbol, PragmaKeyword, OnceKeyword>` parses when the
 * text contains `#pragma once`.
 */
template<class... Args> struct Sequence : public std::tuple<Args...> {

 private:
  template<std::size_t I> inline bool parse_type(TokenIterator &token_iterator)
  {
    using Type = std::tuple_element_t<I, std::tuple<Args...>>;
    std::optional<Type> val = Type::parse(token_iterator);
    if (val.has_value()) {
      std::get<I>(*this) = std::move(val.value());
    }
    return val.has_value();
  };

  template<std::size_t... I>
  inline bool parse_impl(std::index_sequence<I...> /*indices*/, TokenIterator &token_iterator)
  {
    return (parse_type<I>(token_iterator) && ...);
  };

 public:
  static std::optional<Sequence> parse(TokenIterator &token_iterator)
  {
    token_iterator.push_waypoint();
    Sequence sequence;
    const bool success = sequence.parse_impl(std::make_index_sequence<sizeof...(Args)>{},
                                             token_iterator);
    token_iterator.end_waypoint(success);
    if (success) {
      return sequence;
    }
    return std::nullopt;
  }
};

/**
 * Parser that don't fails if `Type` can't be parsed.
 * The sequence `Sequence<Optional<ConstKeyword>, IntKeyword, Identifier, SemicolonSymbol>` success
 * either if text is `const int num;` or `int num;`
 */
template<typename Type> struct Optional : public std::optional<Type> {
  static std::optional<Optional> parse(TokenIterator &token_iterator)
  {
    token_iterator.push_waypoint();
    std::optional<Type> result = Type::parse(token_iterator);
    token_iterator.end_waypoint(result.has_value());
    return Optional{std::move(result)};
  }
};

/**
 * Parser that tries to match any `Arg` in `Args...`
 * The sequence `Sequence<Variant<IntKeyword, FloatKeyword>, Identifier, SemicolonSymbol>`
 * success either if text is `int num;` or `float num;`
 */
template<class... Args> struct Variant : public std::variant<Args...> {
 private:
  template<typename Type> inline bool parse_type(TokenIterator &token_iterator)
  {
    token_iterator.push_waypoint();
    std::optional<Type> val = Type::parse(token_iterator);
    if (val.has_value()) {
      (*this).template emplace<Type>(std::move(val.value()));
    }
    token_iterator.end_waypoint(val.has_value());
    return val.has_value();
  };

 public:
  static std::optional<Variant> parse(TokenIterator &token_iterator)
  {
    Variant variant;
    if ((variant.parse_type<Args>(token_iterator) || ...)) {
      return variant;
    }
    return std::nullopt;
  }
};

/** Keyword parser. */
template<KeywordType Type> struct Keyword {
  static std::optional<Keyword> parse(TokenIterator &token_iterator)
  {
    if (token_iterator.next_keyword(Type)) {
      return Keyword{};
    }
    return std::nullopt;
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
template<SymbolType type> struct Symbol {
  static std::optional<Symbol> parse(TokenIterator &token_iterator)
  {
    if (token_iterator.next_symbol(type)) {
      return Symbol{};
    }
    return std::nullopt;
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
template<lex::KeywordType Type> struct MacroCall {
  static std::optional<MacroCall> parse(TokenIterator &token_iterator)
  {
    if (Sequence<Keyword<Type>, LParenSymbol>::parse(token_iterator).has_value()) {
      skip_until_match_paired_symbols(SymbolType::LPAREN, SymbolType::RPAREN, token_iterator);
      SemicolonSymbol::parse(token_iterator);
      return MacroCall{};
    }
    return std::nullopt;
  }
};

/** Parses a string literal. */
struct StringLiteral {
  std::string_view value;
  static std::optional<StringLiteral> parse(TokenIterator &token_iterator)
  {
    if (StringLiteralToken *literal = token_iterator.next<StringLiteralToken>(); literal) {
      return StringLiteral{literal->where};
    }
    return std::nullopt;
  }
};

/** Parses a int literal. */
struct IntLiteral {
  int value;
  static std::optional<IntLiteral> parse(TokenIterator &token_iterator)
  {
    if (IntLiteralToken *value = token_iterator.next<IntLiteralToken>(); value) {
      return IntLiteral{value->val};
    }
    return std::nullopt;
  }
};

/** Parses a identifier. */
struct Identifier {
  std::string_view str;
  static std::optional<Identifier> parse(TokenIterator &token_iterator)
  {
    if (IdentifierToken *identifier = token_iterator.next<IdentifierToken>(); identifier) {
      return Identifier{identifier->where};
    }
    return std::nullopt;
  }
};

/** Parses a include, either `#include "include_name.hh"` or `#include <path/to/include.hh>`. */
struct Include {
  static std::optional<Include> parse(TokenIterator &token_iterator)
  {
    if (Sequence<HashSymbol, IncludeKeyword>::parse(token_iterator).has_value()) {
      TokenVariant *token = token_iterator.next_variant();
      while (token && !std::holds_alternative<BreakLineToken>(*token)) {
        token = token_iterator.next_variant();
      }
      return Include{};
    }
    return std::nullopt;
  }
};

/** Check if a token is a symbol and has a type. */
static bool inline is_symbol_type(const TokenVariant &token, const SymbolType type)
{
  return std::holds_alternative<SymbolToken>(token) && std::get<SymbolToken>(token).type == type;
}

/** Parses `#define` directives except to const int defines. */
struct Define {
  static std::optional<Define> parse(TokenIterator &token_iterator)
  {
    if (!Sequence<HashSymbol, DefineKeyword>::parse(token_iterator)) {
      return std::nullopt;
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
std::optional<DefineInt> DefineInt::parse(TokenIterator &token_iterator)
{
  using DefineConstIntSeq = Sequence<HashSymbol, DefineKeyword, Identifier, IntLiteral>;
  std::optional<DefineConstIntSeq> val = DefineConstIntSeq::parse(token_iterator);
  if (!val.has_value() || !token_iterator.next<BreakLineToken>()) {
    return std::nullopt;
  }
  return DefineInt{std::get<2>(val.value()).str, std::get<3>(val.value()).value};
}

bool DefineInt::operator==(const DefineInt &other) const
{
  return name == other.name && value == other.value;
}

/** Parses most c++ primitive types. */
struct PrimitiveType {
  std::string_view str;
  static std::optional<PrimitiveType> parse(TokenIterator &token_iterator)
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
    std::optional<PrimitiveTypeVariants> type = PrimitiveTypeVariants::parse(token_iterator);
    if (!type.has_value()) {
      return std::nullopt;
    }

    /* Use `unsigned int` as uint32?.... */
    using namespace std::string_view_literals;
    /* Note: makesdna ignores `unsigned` keyword. */
    static constexpr std::string_view primitive_types[]{
        "int"sv,      "char"sv,     "short"sv,   "float"sv,   "double"sv,
        "void"sv,     "int"sv,      "short"sv,   "char"sv,    "int8_t"sv,
        "int16_t"sv,  "int32_t"sv,  "int64_t"sv, "uint8_t"sv, "uint16_t"sv,
        "uint32_t"sv, "uint64_t"sv, "long"sv,    "ulong"sv,
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
  static std::optional<Type> parse(TokenIterator &token_iterator)
  {
    using TypeVariant = Variant<PrimitiveType, Sequence<Optional<StructKeyword>, Identifier>>;
    using TypeSequence = Sequence<Optional<ConstKeyword>, TypeVariant>;

    const std::optional<TypeSequence> type_seq = TypeSequence::parse(token_iterator);
    if (!type_seq) {
      return std::nullopt;
    }
    const bool const_tag = std::get<0>(type_seq.value()).has_value();
    const TypeVariant &type_variant = std::get<1>(type_seq.value());
    if (std::holds_alternative<PrimitiveType>(type_variant)) {
      return Type{const_tag, std::get<0>(type_variant).str};
    }
    return Type{const_tag, std::get<1>(std::get<1>(type_variant)).str};
  }
};

/** Parses the array part of variable declarations: with `int num[3][4];` parses `[3][4]`. */
static Vector<std::variant<std::string_view, int32_t>> variable_size_array_part(
    TokenIterator &token_iterator)
{
  Vector<std::variant<std::string_view, int32_t>> result;
  /* Dynamic array. */
  if (Sequence<LBracketSymbol, RBracketSymbol>::parse(token_iterator).has_value()) {
    result.append(std::string_view{""});
  }
  while (true) {
    using ArraySize = Sequence<LBracketSymbol, Variant<IntLiteral, Identifier>, RBracketSymbol>;
    const std::optional<ArraySize> size_seq = ArraySize::parse(token_iterator);
    if (!size_seq.has_value()) {
      break;
    }
    const auto &item_size = std::get<1>(size_seq.value());
    if (std::holds_alternative<IntLiteral>(item_size)) {
      result.append(std::get<IntLiteral>(item_size).value);
    }
    else {
      result.append(std::get<Identifier>(item_size).str);
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
std::optional<Variable> Variable::parse(TokenIterator &token_iterator)
{
  const std::optional<Type> type{Type::parse(token_iterator)};
  if (!type) {
    return std::nullopt;
  }
  Variable variable;
  variable.const_tag = type.value().const_tag;
  variable.type = type.value().str;

  while (true) {
    std::string start;
    for (; StarSymbol::parse(token_iterator);) {
      start += '*';
    }
    std::optional<Identifier> name{Identifier::parse(token_iterator)};
    if (!name.has_value()) {
      return std::nullopt;
    }
    Variable::Item item{};
    item.ptr = !start.empty() ? std::optional{start} : std::nullopt;
    item.name = name.value().str;
    item.size = variable_size_array_part(token_iterator);
    variable.items.append(std::move(item));
    DNADeprecatedKeyword::parse(token_iterator);
    if (SemicolonSymbol::parse(token_iterator).has_value()) {
      break;
    }
    if (!CommaSymbol::parse(token_iterator).has_value()) {
      return std::nullopt;
    }
  }
  return variable;
}

bool Variable::Item::operator==(const Variable::Item &other) const
{
  return ptr == other.ptr && name == other.name && size == other.size;
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
std::optional<FunctionPtr> FunctionPtr::parse(TokenIterator &token_iterator)
{
  using FunctionPtrBegin = Sequence<Type,
                                    Optional<StarSymbol>,
                                    LParenSymbol,
                                    StarSymbol,
                                    Identifier,
                                    RParenSymbol,
                                    LParenSymbol>;
  const std::optional<FunctionPtrBegin> fn = FunctionPtrBegin::parse(token_iterator);
  if (!fn.has_value()) {
    return std::nullopt;
  }
  FunctionPtr fn_ptr{};
  fn_ptr.const_tag = std::get<0>(fn.value()).const_tag;
  fn_ptr.type = std::get<0>(fn.value()).str;
  fn_ptr.name = std::get<4>(fn.value()).str;
  /* Skip Function params. */
  skip_until_match_paired_symbols(SymbolType::LPAREN, SymbolType::RPAREN, token_iterator);

  /* Closing sequence. */
  if (!SemicolonSymbol::parse(token_iterator).has_value()) {
    return std::nullopt;
  }
  return fn_ptr;
}

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
std::optional<PointerToArray> PointerToArray::parse(TokenIterator &token_iterator)
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
  std::optional<PointerToArraySequence> val = PointerToArraySequence::parse(token_iterator);
  if (!val.has_value()) {
    return std::nullopt;
  }
  PointerToArray ptr{};
  ptr.type = std::get<0>(val.value()).str;
  ptr.name = std::get<3>(val.value()).str;
  ptr.size = std::get<6>(val.value()).value;
  return ptr;
}

template<KeywordType... type> static bool is_keyword_type(TokenVariant token)
{
  return std::holds_alternative<KeywordToken>(token) &&
         ((std::get<KeywordToken>(token).type == type) || ...);
}

/**
 * Parses `#if....#endif` code blocks.
 */
struct IfDef {
  static std::optional<IfDef> parse(TokenIterator &token_iterator)
  {
    using IfDefBeginSequence =
        Sequence<HashSymbol, Variant<IfDefKeyword, IfKeyword, IfnDefKeyword>>;
    const std::optional<IfDefBeginSequence> val = IfDefBeginSequence::parse(token_iterator);
    if (!val.has_value()) {
      return std::nullopt;
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
      return std::nullopt;
    }
    return IfDef{};
  }
};

/**
 * Parses struct declarations.
 */
std::optional<Struct> Struct::parse(TokenIterator &token_iterator)
{
  using StructBeginSequence =
      Sequence<Optional<TypedefKeyword>, StructKeyword, Optional<Identifier>, LBraceSymbol>;
  std::optional<StructBeginSequence> struct_seq = StructBeginSequence::parse(token_iterator);
  if (!struct_seq.has_value()) {
    return std::nullopt;
  }
  Struct result{};
  if (std::get<2>(struct_seq.value()).has_value()) {
    result.name = std::get<2>(struct_seq.value()).value().str;
  }
  while (true) {
    using DNA_DEF_CCX_Macro = MacroCall<lex::KeywordType::DNA_DEFINE_CXX_METHODS>;

    if (auto member = Variant<Variable, FunctionPtr, PointerToArray, Struct>::parse(
            token_iterator);
        member.has_value())
    {
      result.items.append(std::move(member.value()));
    }
    else if (DNA_DEF_CCX_Macro::parse(token_iterator).has_value() ||
             IfDef::parse(token_iterator).has_value())
    {
    }
    else {
      break;
    }
  }
  using StructEndSequence = Sequence<RBraceSymbol, Optional<Identifier>, SemicolonSymbol>;
  std::optional<StructEndSequence> struct_end = StructEndSequence ::parse(token_iterator);
  if (!struct_end.has_value()) {
    return std::nullopt;
  }
  if (std::get<1>(struct_end.value()).has_value()) {
    result.member_name = std::get<1>(struct_end.value()).value().str;
  }
  if (result.member_name == result.name && result.name.empty()) {
    return std::nullopt;
  }
  return result;
}

bool Struct::operator==(const Struct &other) const
{
  return name == other.name && items == other.items;
}

/** Parses skipped declarations by makesdna. */
struct Skip {
  static std::optional<Skip> parse(TokenIterator &token_iterator)
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
    if (UnusedDeclarations::parse(token_iterator).has_value()) {
      return Skip{};
    }
    /* Forward declarations. */
    if (Sequence<StructKeyword, Identifier>::parse(token_iterator).has_value()) {
      for (; Sequence<CommaSymbol, Identifier>::parse(token_iterator).has_value();) {
      }
      if (SemicolonSymbol::parse(token_iterator).has_value()) {
        return Skip{};
      }
    }
    else if (token_iterator.next<BreakLineToken>()) {
      return Skip{};
    }
    return std::nullopt;
  }
};

/** Parse enums, with a name or not and with a fixed type or not. */
std::optional<Enum> ast::Enum::parse(TokenIterator &token_iterator)
{
  using EnumBeginSequence = Sequence<Optional<TypedefKeyword>,
                                     EnumKeyword,
                                     Optional<ClassKeyword>,
                                     Optional<Identifier>,
                                     Optional<Sequence<ColonSymbol, PrimitiveType>>,
                                     LBraceSymbol>;
  std::optional<EnumBeginSequence> enum_begin = EnumBeginSequence::parse(token_iterator);
  if (!enum_begin.has_value()) {
    return std::nullopt;
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
  if (!Sequence<Optional<Identifier>, Optional<DNADeprecatedKeyword>, SemicolonSymbol>::parse(
           token_iterator)
           .has_value())
  {
    return std::nullopt;
  }
  return enum_def;
}
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

bool parse_include(std::string_view filepath,
                   std::string_view text,
                   lex::TokenIterator &token_iterator,
                   Vector<ast::CppType> &dest)
{
  using namespace ast;
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
    std::optional<CPPTypeVariant> val = CPPTypeVariant::parse(token_iterator);
    if (!val.has_value()) {
      print_unhandled_token_error(filepath, text, token_iterator.last_unmatched);
      return false;
    }
    if (std::holds_alternative<Struct>(val.value())) {
      dest.append(std::move(std::get<Struct>(val.value())));
    }
    else if (std::holds_alternative<DefineInt>(val.value())) {
      dest.append(std::move(std::get<DefineInt>(val.value())));
    }
    else if (std::holds_alternative<Variable>(val.value())) {
      continue;
      dest.append(std::move(std::get<Variable>(val.value())));
    }
    else if (std::holds_alternative<Enum>(val.value())) {
      Enum &enum_def = std::get<Enum>(val.value());
      /* Keep only named enums with fixed type. */
      if (!enum_def.name.has_value() || !enum_def.type.has_value()) {
        continue;
      }
      dest.append(enum_def);
    }
    else if (std::holds_alternative<DNADeprecatedAllowSeq>(val.value())) {
      dna_deprecated_allow_count++;
    }
    else if (std::holds_alternative<EndIfSeq>(val.value())) {
      dna_deprecated_allow_count--;
      BLI_assert(dna_deprecated_allow_count >= 0);
    }
  }
#ifdef DEBUG_PRINT_DNA_PARSER
  constexpr std::string_view debug_file{"DNA_action_types.h"};
  if (!debug_file.empty() && filepath.find(debug_file) != filepath.npos) {
    for (auto &val : dest) {
      std::visit(ParserDebugPrinter{}, val);
    }
  }
#endif
  return true;
}

}  // namespace blender::dna::parser
