/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include <iostream>

#include "dna_parser.hh"

#ifdef DEBUG_PRINT_DNA_PARSER

namespace blender::dna::parser {
struct StructMemberPrinter {

  void operator()(ast::Variable &var) const
  {
    if (var.const_tag) {
      std::cout << "const " << var.type << " ";
    }
    else {
      std::cout << var.type << " ";
    }
    bool first = true;
    for (auto &variable_item : var.items) {
      if (!first) {
        std::cout << ",";
      }
      first = false;
      std::cout << variable_item.ptr.value_or("") << variable_item.name;
      for (auto &size : variable_item.size) {
        if (std::holds_alternative<std::string_view>(size)) {
          std::cout << "[" << std::get<std::string_view>(size) << "]";
        }
        else {
          std::cout << "[" << std::get<int32_t>(size) << "]";
        }
      }
    }
  }
  void operator()(ast::FunctionPtr &fn) const
  {
    if (fn.const_tag) {
      std::cout << "const ";
    }
    std::cout << fn.type << " *(*" << fn.name << ")(...)";
  }
  void operator()(ast::PointerToArray &ptr) const
  {
    std::cout << ptr.type << " (*" << ptr.name << ")[" << ptr.size << "]";
  }
};

struct ParserDebugPrinter {
  void operator()(ast::DefineInt &val) const
  {
    std::cout << "#define " << val.name << " " << val.value << "\n";
  }
  void operator()(ast::Enum &val) const
  {
    std::cout << "enum " << val.name.value_or("unnamed");
    if (val.type) {
      std::cout << ": " << val.type.value();
    }
    std::cout << " {...};\n";
  }
  void operator()(ast::Struct &val) const
  {
    std::cout << "struct " << val.name << " {\n";
    for (auto &item : val.items) {
      std::cout << "    ";
      std::visit(StructMemberPrinter{}, item);
      std::cout << ";\n";
    }
    std::cout << "};\n";
  }
  void operator()(ast::FunctionPtr &fn) const
  {
    StructMemberPrinter().operator()(fn);
    std::cout << "\n";
  }
  void operator()(ast::Variable &var) const
  {
    StructMemberPrinter().operator()(var);
    std::cout << "\n";
  }
};
#endif

}  // namespace blender::dna::parser

namespace blender::dna::parser::ast {

/* Sequence of elements, fails if any `Args` in `Args...` fails to parse. */
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
  static inline bool parse_impl(std::index_sequence<I...>, TokenIterator &cont, Sequence &sequence)
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

/* Optional element that don't fails if `Type` can't be parsed. */
template<typename Type> struct Optional : public std::optional<Type> {
  static std::optional<Optional> parse(TokenIterator &cont)
  {
    cont.push_waypoint();
    std::optional<Type> result = Type::parse(cont);
    cont.end_waypoint(result.has_value());
    return Optional{std::move(result)};
  }
};

/* A variant element. */
template<class... Args> struct Variant : public std::variant<Args...> {
 private:
  template<typename Type> static inline bool parse_type(TokenIterator &cont, Variant &where)
  {
    cont.push_waypoint();
    std::optional<Type> val = Type::parse(cont);
    if (val.has_value()) {
      where.emplace<Type>(std::move(val.value()));
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
using ExternKeyword = Keyword<KeywordType::EXTERN>;
using TypedefKeyword = Keyword<KeywordType::TYPEDEF>;
using PragmaKeyword = Keyword<KeywordType::PRAGMA>;
using OnceKeyword = Keyword<KeywordType::ONCE>;
using EnumKeyword = Keyword<KeywordType::ENUM>;
using ClassKeyword = Keyword<KeywordType::CLASS>;
using DNA_DEF_CCXKeyword = Keyword<KeywordType::DNA_DEFINE_CXX_METHODS>;
using DNADepecratedKeyword = Keyword<KeywordType::DNA_DEPRECATED>;
using EnumOperatorsKeyword = Keyword<KeywordType::ENUM_OPERATORS>;

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
using GreatherSymbol = Symbol<SymbolType::GREATHER>;
using AssignSymbol = Symbol<SymbolType::ASSIGN>;
using MinusSymbol = Symbol<SymbolType::MINUS>;

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
      scape_bl = std::holds_alternative<SymbolToken>(*token) &&
                 std::get<SymbolToken>(*token).type == SymbolType::BACKSLASH;
    }
    return Define{};
  }
};

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
                                          UInt64Keyword>;
    std::optional<PrimitiveTypeVariants> type = PrimitiveTypeVariants::parse(cont);
    if (!type.has_value()) {
      return std::nullopt;
    }
    /* Use `unsigned int` as uint32?.... */
    static constexpr std::string_view primitive_types[]{
        "int",
        "char",
        "short",
        "float",
        "double",
        "void",
        "unsigned int",
        "unsigned short",
        "unsigned char",
        "int8_t",
        "int16_t",
        "int32_t",
        "int64_t",
        "uint8_t",
        "uint16_t",
        "uint32_t",
        "uint64_t",
    };
    return PrimitiveType{primitive_types[type.value().index()]};
  }
};

struct TypeOrStruct {
  bool const_tag{false};
  std::string_view str;
  static std::optional<TypeOrStruct> parse(TokenIterator &cont)
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
      return TypeOrStruct{const_tag, std::get<0>(type_variant).str};
    }
    else {
      return TypeOrStruct{const_tag, std::get<1>(std::get<1>(type_variant)).str};
    }
  }
};

static Vector<std::variant<std::string_view, int32_t>> variable_size_array_part(
    TokenIterator &cont)
{
  Vector<std::variant<std::string_view, int32_t>> result;
  /* Dynamic array. */
  if (Sequence<LBracketSymbol, RBracketSymbol>::parse(cont).has_value()) {
    result.append(std::string_view{""});
  }
  while (true) {
    using ArraySyze = Sequence<LBracketSymbol, Variant<IntLiteral, Identifier>, RBracketSymbol>;
    const std::optional<ArraySyze> size_seq = ArraySyze::parse(cont);
    if (!size_seq.has_value()) {
      break;
    }
    auto &item_size = std::get<1>(size_seq.value());
    if (std::holds_alternative<IntLiteral>(item_size)) {
      result.append(std::get<IntLiteral>(item_size).value);
    }
    else {
      result.append(std::get<Identifier>(item_size).str);
    }
  }
  return result;
}

std::optional<Variable> Variable::parse(TokenIterator &cont)
{
  const std::optional<TypeOrStruct> type{TypeOrStruct::parse(cont)};
  if (!type) {
    return std::nullopt;
  }
  Variable variable;
  variable.const_tag = type.value().const_tag;
  variable.type = type.value().str;

  while (true) {
    std::string start = "";
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
    DNADepecratedKeyword::parse(cont);
    if (SemicolonSymbol::parse(cont).has_value()) {
      break;
    }
    else if (!CommaSymbol::parse(cont).has_value()) {
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

/* Skips tokens until match the closing right symbol, like function body braces `{}`. */
static void skip_until_match_symbols(SymbolType left, SymbolType right, TokenIterator &cont)
{
  int left_count = 1;
  for (TokenVariant *token = cont.next_variant(); token; token = cont.next_variant()) {
    if (!std::holds_alternative<SymbolToken>(*token)) {
      continue;
    }
    const SymbolType type = std::get<SymbolToken>(*token).type;
    if (type == right) {
      left_count--;
    }
    else if (type == left) {
      left_count++;
    }
    if (left_count == 0) {
      break;
    }
  }
};

std::optional<FunctionPtr> FunctionPtr::parse(TokenIterator &cont)
{
  using FunctionPtrBegin = Sequence<TypeOrStruct,
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
  skip_until_match_symbols(SymbolType::LPAREN, SymbolType::RPAREN, cont);

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
std::optional<PointerToArray> PointerToArray::parse(TokenIterator &cont)
{
  using PointerToArraySequence = Sequence<TypeOrStruct,
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

struct IfDefSection {
  static std::optional<IfDefSection> parse(TokenIterator &cont)
  {
    using IfDefBeginSequence =
        Sequence<HashSymbol, Variant<IfDefKeyword, IfKeyword, IfnDefKeyword>>;
    const std::optional<IfDefBeginSequence> val = IfDefBeginSequence::parse(cont);
    if (!val.has_value()) {
      return std::nullopt;
    };
    auto match_closing = [](TokenVariant *token) {
      return std::holds_alternative<KeywordToken>(*token) &&
             std::get<KeywordToken>(*token).type == KeywordType::ENDIF;
    };
    auto match_opening = [](TokenVariant *token) {
      return (std::holds_alternative<KeywordToken>(*token) &&
              (std::get<KeywordToken>(*token).type == KeywordType::IF ||
               std::get<KeywordToken>(*token).type == KeywordType::IFDEF ||
               std::get<KeywordToken>(*token).type == KeywordType::IFNDEF));
    };
    int ifdef_deep = 1;
    bool hash_carried = false;
    for (TokenVariant *token = cont.next_variant(); token; token = cont.next_variant()) {
      if (hash_carried && match_opening(token)) {
        ifdef_deep++;
      }
      if (hash_carried && match_closing(token)) {
        ifdef_deep--;
      }
      if (ifdef_deep == 0) {
        break;
      }
      hash_carried = std::holds_alternative<SymbolToken>(*token) &&
                     std::get<SymbolToken>(*token).type == SymbolType::HASH;
    }
    /* Not matching #endif. */
    if (ifdef_deep != 0) {
      return std::nullopt;
    }
    return IfDefSection{};
  }
};

std::optional<Struct> Struct::parse(TokenIterator &cont)
{
  using StructBeginSequence =
      Sequence<Optional<TypedefKeyword>, StructKeyword, Identifier, LBraceSymbol>;
  std::optional<StructBeginSequence> struct_seq = StructBeginSequence::parse(cont);
  if (!struct_seq.has_value()) {
    return std::nullopt;
  }
  Struct result{};
  result.name = std::get<2>(struct_seq.value()).str;
  while (true) {
    using DNA_DEF_CCX_SEQ = Sequence<DNA_DEF_CCXKeyword,
                                     LParenSymbol,
                                     Identifier,
                                     RParenSymbol,
                                     Optional<SemicolonSymbol>>;
    if (auto member = Variant<Variable, FunctionPtr, PointerToArray>::parse(cont);
        member.has_value())
    {
      result.items.append(std::move(member.value()));
    }
    else if (DNA_DEF_CCX_SEQ::parse(cont).has_value()) {
    }
    else if (IfDefSection::parse(cont).has_value()) {
    }
    else {
      break;
    }
  }
  if (!Sequence<RBraceSymbol, Optional<Identifier>, SemicolonSymbol>::parse(cont).has_value()) {
    return std::nullopt;
  }
  return result;
}

bool Struct::operator==(const Struct &other) const
{
  return name == other.name && items == other.items;
}

struct Skip {
  static std::optional<Skip> parse(TokenIterator &cont)
  {
    using UnusedDeclarations = Variant<Define,
                                       Include,
                                       IfDefSection,
                                       Sequence<HashSymbol, PragmaKeyword, OnceKeyword>,
                                       Sequence<HashSymbol, HashSymbol, Struct>,
                                       Sequence<ExternKeyword, Variable>>;
    if (UnusedDeclarations::parse(cont).has_value()) {
      return Skip{};
    }
    /* Forward declare. */
    else if (Sequence<StructKeyword, Identifier>::parse(cont).has_value()) {
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

std::optional<Enum> ast::Enum::parse(TokenIterator &cont)
{
  /* Enum begin sequence. */
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
  skip_until_match_symbols(SymbolType::LBRACE, SymbolType::RBRACE, cont);

  /* Enum end sequence. */
  if (!Sequence<Optional<Identifier>, Optional<DNADepecratedKeyword>, SemicolonSymbol>::parse(cont)
           .has_value())
  {
    return std::nullopt;
  }
  /* Optional enum operator defines. */
  if (Sequence<EnumOperatorsKeyword, LParenSymbol>::parse(cont).has_value()) {
    skip_until_match_symbols(SymbolType::LPAREN, SymbolType::RPAREN, cont);
    SemicolonSymbol::parse(cont);
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

  std::visit(
      [text, filepath](auto &&token) {
        std::string_view::iterator itr = text.begin();
        size_t line = 1;
        while (itr < token.where.begin()) {
          if (itr[0] == '\n') {
            line++;
          }
          itr++;
        }
        std::cout << filepath << "(" << line << ") Unhandled token: \"" << token.where << "\"\n";
      },
      *what);
}
bool parse_include(std::string_view filepath,
                   std::string_view text,
                   lex::TokenIterator &cont,
                   Vector<ast::CppType> &c)
{
  using namespace ast;
  while (!cont.has_finish()) {
    using CPPTypeVariant = Variant<Struct,
                                   Enum,
                                   Sequence<Optional<TypedefKeyword>, FunctionPtr>,
                                   Variable,
                                   DefineInt,
                                   Skip>;
    std::optional<CPPTypeVariant> val = CPPTypeVariant::parse(cont);
    if (!val.has_value()) {
      print_unhandled_token_error(filepath, text, cont.last);
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
      c.append(std::move(std::get<Variable>(val.value())));
    }
    else if (std::holds_alternative<Enum>(val.value())) {
      c.append(std::move(std::get<Enum>(val.value())));
    }
  }
  for (auto &val : c) {
    //std::visit(ParserDebugPrinter{}, val);
  }

  return true;
}

}  // namespace blender::dna::parser
