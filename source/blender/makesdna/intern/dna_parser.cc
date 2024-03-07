/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "dna_parser.hh"
#include <fmt/format.h>

#ifdef DEBUG_PRINT_DNA_PARSER
namespace blender::dna::parser {

void printf_struct(ast::Struct &val, size_t padding);

struct StructMemberPrinter {
  size_t padding;
  void operator()(ast::Variable &var) const
  {
    if (var.const_tag) {
      printf("const ");
    }
    printf("%s", fmt::format("{} ", var.type).c_str());

    bool first = true;
    for (auto &variable_item : var.items) {
      if (!first) {
        printf(",");
      }
      first = false;
      printf("%s",
             fmt::format("{}{}", variable_item.ptr.value_or(""), variable_item.name).c_str());
      for (auto &size : variable_item.size) {
        if (std::holds_alternative<std::string_view>(size)) {
          printf("%s", fmt::format("[{}]", std::get<std::string_view>(size)).c_str());
        }
        else {
          printf("%s", fmt::format("[{}]", std::get<int32_t>(size)).c_str());
        }
      }
    }
  }
  void operator()(ast::FunctionPtr &fn) const
  {
    if (fn.const_tag) {
      printf("%s", "const ");
    }
    printf("%s", fmt::format("{} (*{})(...)", fn.type, fn.name).c_str());
  }
  void operator()(ast::PointerToArray &ptr) const
  {
    printf("%s", fmt::format("{} (*{})[{}]", ptr.type, ptr.name, ptr.size).c_str());
  }
  void operator()(ast::Struct &val) const
  {
    printf_struct(val, padding);
  }
};

struct ParserDebugPrinter {
  size_t padding;
  void operator()(ast::DefineInt &val) const
  {
    printf("%s\n", fmt::format("#define {} {}", val.name, val.value).c_str());
  }
  void operator()(ast::Enum &val) const
  {
    printf("%s", fmt::format("enum {}", val.name.value_or("unnamed")).c_str());
    if (val.type) {
      printf("%s", fmt::format(": {}", val.type.value()).c_str());
    }
    printf(" {...};\n");
  }
  void operator()(ast::Struct &val) const
  {
    printf_struct(val, padding);
    printf(";\n");
  }
  void operator()(ast::FunctionPtr &fn) const
  {
    StructMemberPrinter{padding + 1}.operator()(fn);
    printf("\n");
  }
  void operator()(ast::Variable &var) const
  {
    StructMemberPrinter{padding + 1}.operator()(var);
    printf("\n");
  }
};

void printf_struct(ast::Struct &val, size_t padding)
{
  printf("%s\n", fmt::format("struct {} {{", val.name).c_str());
  for (auto &item : val.items) {
    for (size_t x = 0; x < padding + 1; x++) {
      printf("    ");
    }
    std::visit(StructMemberPrinter{padding + 1}, item);
    printf(";\n");
  }
  for (size_t x = 0; x < padding; x++) {
    printf("    ");
  }
  printf("}");
};
#endif

}  // namespace blender::dna::parser

namespace blender::dna::parser::ast {

/**
 * Parser that matches a sequence of elements to parse, fails if any `Args` in `Args...` fails to
 * parse.
 * Given the following example:`Sequence<HashSymbol,PragmaKeyword,OnceKeyword>` parses when the
 * text contains `#pragma once`.
 */
template<class... Args> struct Sequence : public std::tuple<Args...> {

 private:
  template<std::size_t I, typename Type>
  static inline bool parse_type(TokenIterator &cont, Sequence &sequence)
  {
    std::optional<Type> val = Type::parse(cont);
    if (val.has_value()) {
      std::get<I>(sequence) = std::move(val.value());
    }
    return val.has_value();
  };

  template<std::size_t... I>
  static inline bool parse_impl(std::index_sequence<I...> /*indices*/,
                                TokenIterator &cont,
                                Sequence &sequence)
  {
    return (parse_type<I, Args>(cont, sequence) && ...);
  };

 public:
  static std::optional<Sequence> parse(TokenIterator &cont)
  {
    cont.push_waypoint();
    Sequence sequence;
    const bool success = parse_impl(std::make_index_sequence<sizeof...(Args)>{}, cont, sequence);
    cont.end_waypoint(success);
    if (success) {
      return sequence;
    }
    return std::nullopt;
  }
};

/**
 * Parser that don't fails if `Type` can't be parsed.
 * Parsing the sequence `Sequence<Optional<ConstKeyword>,IntKeyword, Identifier,SemicolonSymbol>`
 * success either if text is `const int num;` or `int num;`
 */
template<typename Type> struct Optional : public std::optional<Type> {
  static std::optional<Optional> parse(TokenIterator &cont)
  {
    cont.push_waypoint();
    std::optional<Type> result = Type::parse(cont);
    cont.end_waypoint(result.has_value());
    return Optional{std::move(result)};
  }
};

/**
 * Parser that tries to match any `Arg` in `Args...`
 * Parsing the sequence `Sequence<Variant<IntKeyword,FloatKeyword>,Identifier,SemicolonSymbol>`
 * success either if text is `int num;` or `float num;`
 */
template<class... Args> struct Variant : public std::variant<Args...> {
 private:
  template<typename Type> static inline bool parse_type(TokenIterator &cont, Variant &variant)
  {
    cont.push_waypoint();
    std::optional<Type> val = Type::parse(cont);
    if (val.has_value()) {
      variant.template emplace<Type>(std::move(val.value()));
    }
    cont.end_waypoint(val.has_value());
    return val.has_value();
  };

 public:
  static std::optional<Variant> parse(TokenIterator &cont)
  {
    Variant tmp;
    if ((parse_type<Args>(cont, tmp) || ...)) {
      return tmp;
    }
    return std::nullopt;
  }
};

/** Keyword parser. */
template<KeywordType Type> struct Keyword {
  static std::optional<Keyword> parse(TokenIterator &cont)
  {
    if (cont.next_keyword(Type)) {
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
  static std::optional<Symbol> parse(TokenIterator &cont)
  {
    if (cont.next_symbol(type)) {
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
                                            TokenIterator &cont);

/**
 * Parses a macro call, `MacroCall<KeywordType::DNA_DEFINE_CXX_METHODS>` parses
 * `DNA_DEFINE_CXX_METHODS(...)`.
 */
template<lex::KeywordType Type> struct MacroCall {
  static std::optional<MacroCall> parse(TokenIterator &cont)
  {
    if (Sequence<Keyword<Type>, LParenSymbol>::parse(cont).has_value()) {
      skip_until_match_paired_symbols(SymbolType::LPAREN, SymbolType::RPAREN, cont);
      SemicolonSymbol::parse(cont);
      return MacroCall{};
    }
    return std::nullopt;
  }
};

/** Parses a string literal. */
struct StringLiteral {
  std::string_view value;
  static std::optional<StringLiteral> parse(TokenIterator &cont)
  {
    if (StringLiteralToken *literal = cont.next<StringLiteralToken>(); literal) {
      return StringLiteral{literal->where};
    }
    return std::nullopt;
  }
};

/** Parses a int literal. */
struct IntLiteral {
  int value;
  static std::optional<IntLiteral> parse(TokenIterator &cont)
  {
    if (IntLiteralToken *value = cont.next<IntLiteralToken>(); value) {
      return IntLiteral{value->val};
    }
    return std::nullopt;
  }
};

/** Parses a identifier. */
struct Identifier {
  std::string_view str;
  static std::optional<Identifier> parse(TokenIterator &cont)
  {
    if (IdentifierToken *identifier = cont.next<IdentifierToken>(); identifier) {
      return Identifier{identifier->where};
    }
    return std::nullopt;
  }
};

/** Parses a include, either `#include "include_name.hh"` or `#include <path/to/include.hh>`. */
struct Include {
  static std::optional<Include> parse(TokenIterator &cont)
  {
    if (Sequence<HashSymbol, IncludeKeyword>::parse(cont).has_value()) {
      TokenVariant *token = cont.next_variant();
      while (token && !std::holds_alternative<BreakLineToken>(*token)) {
        token = cont.next_variant();
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
  static std::optional<Define> parse(TokenIterator &cont)
  {
    if (!Sequence<HashSymbol, DefineKeyword>::parse(cont)) {
      return std::nullopt;
    }
    bool scape_bl = false;
    for (TokenVariant *token = cont.next_variant(); token; token = cont.next_variant()) {
      if (std::holds_alternative<BreakLineToken>(*token) && !scape_bl) {
        break;
      }
      scape_bl = is_symbol_type(*token, SymbolType::BACKSLASH);
    }
    return Define{};
  }
};

/** Parses const int defines, like `#define FILE_MAX 1024`. */
std::optional<DefineInt> DefineInt::parse(TokenIterator &cont)
{
  using DefineConstIntSeq = Sequence<HashSymbol, DefineKeyword, Identifier, IntLiteral>;
  std::optional<DefineConstIntSeq> val = DefineConstIntSeq::parse(cont);
  if (!val.has_value() || !cont.next<BreakLineToken>()) {
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
  static std::optional<PrimitiveType> parse(TokenIterator &cont)
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
    std::optional<PrimitiveTypeVariants> type = PrimitiveTypeVariants::parse(cont);
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
  static std::optional<Type> parse(TokenIterator &cont)
  {
    using TypeVariant = Variant<PrimitiveType, Sequence<Optional<StructKeyword>, Identifier>>;
    using TypeSequence = Sequence<Optional<ConstKeyword>, TypeVariant>;

    const std::optional<TypeSequence> type_seq = TypeSequence::parse(cont);
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
    TokenIterator &cont)
{
  Vector<std::variant<std::string_view, int32_t>> result;
  /* Dynamic array. */
  if (Sequence<LBracketSymbol, RBracketSymbol>::parse(cont).has_value()) {
    result.append(std::string_view{""});
  }
  while (true) {
    using ArraySize = Sequence<LBracketSymbol, Variant<IntLiteral, Identifier>, RBracketSymbol>;
    const std::optional<ArraySize> size_seq = ArraySize::parse(cont);
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
std::optional<Variable> Variable::parse(TokenIterator &cont)
{
  const std::optional<Type> type{Type::parse(cont)};
  if (!type) {
    return std::nullopt;
  }
  Variable variable;
  variable.const_tag = type.value().const_tag;
  variable.type = type.value().str;

  while (true) {
    std::string start;
    for (; StarSymbol::parse(cont);) {
      start += '*';
    }
    std::optional<Identifier> name{Identifier::parse(cont)};
    if (!name.has_value()) {
      return std::nullopt;
    }
    Variable::Item item{};
    item.ptr = !start.empty() ? std::optional{start} : std::nullopt;
    item.name = name.value().str;
    item.size = variable_size_array_part(cont);
    variable.items.append(std::move(item));
    DNADeprecatedKeyword::parse(cont);
    if (SemicolonSymbol::parse(cont).has_value()) {
      break;
    }
    if (!CommaSymbol::parse(cont).has_value()) {
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
static void skip_until_match_paired_symbols(SymbolType left, SymbolType right, TokenIterator &cont)
{
  int left_count = 1;
  for (TokenVariant *token = cont.next_variant(); token; token = cont.next_variant()) {
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
std::optional<FunctionPtr> FunctionPtr::parse(TokenIterator &cont)
{
  using FunctionPtrBegin = Sequence<Type,
                                    Optional<StarSymbol>,
                                    LParenSymbol,
                                    StarSymbol,
                                    Identifier,
                                    RParenSymbol,
                                    LParenSymbol>;
  const std::optional<FunctionPtrBegin> fn = FunctionPtrBegin::parse(cont);
  if (!fn.has_value()) {
    return std::nullopt;
  }
  FunctionPtr fn_ptr{};
  fn_ptr.const_tag = std::get<0>(fn.value()).const_tag;
  fn_ptr.type = std::get<0>(fn.value()).str;
  fn_ptr.name = std::get<4>(fn.value()).str;
  /* Skip Function params. */
  skip_until_match_paired_symbols(SymbolType::LPAREN, SymbolType::RPAREN, cont);

  /* Closing sequence. */
  if (!SemicolonSymbol::parse(cont).has_value()) {
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
std::optional<PointerToArray> PointerToArray::parse(TokenIterator &cont)
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
  std::optional<PointerToArraySequence> val = PointerToArraySequence::parse(cont);
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
  static std::optional<IfDef> parse(TokenIterator &cont)
  {
    using IfDefBeginSequence =
        Sequence<HashSymbol, Variant<IfDefKeyword, IfKeyword, IfnDefKeyword>>;
    const std::optional<IfDefBeginSequence> val = IfDefBeginSequence::parse(cont);
    if (!val.has_value()) {
      return std::nullopt;
    };
    int ifdef_deep = 1;
    bool hash_carried = false;
    for (TokenVariant *token = cont.next_variant(); token; token = cont.next_variant()) {
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
std::optional<Struct> Struct::parse(TokenIterator &cont)
{
  using StructBeginSequence =
      Sequence<Optional<TypedefKeyword>, StructKeyword, Optional<Identifier>, LBraceSymbol>;
  std::optional<StructBeginSequence> struct_seq = StructBeginSequence::parse(cont);
  if (!struct_seq.has_value()) {
    return std::nullopt;
  }
  Struct result{};
  if (std::get<2>(struct_seq.value()).has_value()) {
    result.name = std::get<2>(struct_seq.value()).value().str;
  }
  while (true) {
    using DNA_DEF_CCX_Macro = MacroCall<lex::KeywordType::DNA_DEFINE_CXX_METHODS>;

    if (auto member = Variant<Variable, FunctionPtr, PointerToArray, Struct>::parse(cont);
        member.has_value())
    {
      result.items.append(std::move(member.value()));
    }
    else if (DNA_DEF_CCX_Macro::parse(cont).has_value() || IfDef::parse(cont).has_value()) {
    }
    else {
      break;
    }
  }
  using StructEndSequence = Sequence<RBraceSymbol, Optional<Identifier>, SemicolonSymbol>;
  std::optional<StructEndSequence> struct_end = StructEndSequence ::parse(cont);
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

/** Parses non used definitions that DNA. */
struct Skip {
  static std::optional<Skip> parse(TokenIterator &cont)
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
    if (UnusedDeclarations::parse(cont).has_value()) {
      return Skip{};
    }
    /* Forward declare. */
    if (Sequence<StructKeyword, Identifier>::parse(cont).has_value()) {
      for (; Sequence<CommaSymbol, Identifier>::parse(cont).has_value();) {
      }
      if (SemicolonSymbol::parse(cont).has_value()) {
        return Skip{};
      }
    }
    else if (cont.next<BreakLineToken>()) {
      return Skip{};
    }
    return std::nullopt;
  }
};

/** Parse enums, with a name or not and with a fixed type or not. */
std::optional<Enum> ast::Enum::parse(TokenIterator &cont)
{
  using EnumBeginSequence = Sequence<Optional<TypedefKeyword>,
                                     EnumKeyword,
                                     Optional<ClassKeyword>,
                                     Optional<Identifier>,
                                     Optional<Sequence<ColonSymbol, PrimitiveType>>,
                                     LBraceSymbol>;
  std::optional<EnumBeginSequence> enum_begin = EnumBeginSequence::parse(cont);
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
  skip_until_match_paired_symbols(SymbolType::LBRACE, SymbolType::RBRACE, cont);

  /* Enum end sequence. */
  if (!Sequence<Optional<Identifier>, Optional<DNADeprecatedKeyword>, SemicolonSymbol>::parse(cont)
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
  auto visit_fn = [text, filepath](auto &&token) {
    std::string_view::iterator itr = text.begin();
    size_t line = 1;
    while (itr < token.where.begin()) {
      if (itr[0] == '\n') {
        line++;
      }
      itr++;
    }
    printf("%s\n",
           fmt::format("{}{} Unhandled token: \"{}\"", filepath, line, token.where).c_str());
  };
  std::visit(visit_fn, *what);
}
bool parse_include(std::string_view filepath,
                   std::string_view text,
                   lex::TokenIterator &cont,
                   Vector<ast::CppType> &c)
{
  using namespace ast;
  int dna_deprecated_allow_count = 0;
  using DNADeprecatedAllowSeq = Sequence<HashSymbol, IfDefKeyword, DNADeprecatedAllowKeyword>;
  using EndIfSeq = Sequence<HashSymbol, EndIfKeyword>;

  while (!cont.has_finish()) {
    using CPPTypeVariant = Variant<Struct,
                                   Enum,
                                   Sequence<Optional<TypedefKeyword>, FunctionPtr>,
                                   Variable,
                                   DefineInt,
                                   DNADeprecatedAllowSeq,
                                   EndIfSeq,
                                   Skip>;
    std::optional<CPPTypeVariant> val = CPPTypeVariant::parse(cont);
    if (!val.has_value()) {
      print_unhandled_token_error(filepath, text, cont.last_unmatched);
      return false;
    }
    if (std::holds_alternative<Struct>(val.value())) {
      c.append(std::move(std::get<Struct>(val.value())));
    }
    // else if (std::holds_alternative<Sequence<Optional<TypedefKeyword>,
    // FunctionPtr>>(val.value()))
    //{
    //   c.append(std::move(std::get<1>(
    //       std::get<Sequence<Optional<TypedefKeyword>, FunctionPtr>>(val.value()))));
    // }
    else if (std::holds_alternative<DefineInt>(val.value())) {
      c.append(std::move(std::get<DefineInt>(val.value())));
    }
    else if (std::holds_alternative<Variable>(val.value())) {
      continue;
      c.append(std::move(std::get<Variable>(val.value())));
    }
    else if (std::holds_alternative<Enum>(val.value())) {
      Enum &enum_def = std::get<Enum>(val.value());
      /** Keep only named enums with fixed type. */
      if (!enum_def.name.has_value() || !enum_def.type.has_value()) {
        continue;
      }
      c.append(enum_def);
    }
    else if (std::holds_alternative<DNADeprecatedAllowSeq>(val.value())) {
      dna_deprecated_allow_count++;
    }
    else if (std::holds_alternative<EndIfSeq>(val.value())) {
      dna_deprecated_allow_count++;
      if (dna_deprecated_allow_count < 0) {
        return false;
      }
    }
  }
  //   for (auto &val : c) {
  //     std::visit(ParserDebugPrinter{}, val);
  //   }

  return true;
}

}  // namespace blender::dna::parser
