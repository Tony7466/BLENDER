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
    std::cout << fn.type << " *(*" << fn.name << ")(";
    for (ast::Variable &var : fn.params) {
      if (var.const_tag) {
        std::cout << "const " << var.type << " ";
      }
      else {
        std::cout << var.type << " ";
      }
      const bool is_last = &fn.params.last() != &var;

      std::cout << var.items[0].ptr.value_or("") << var.items[0].name;
      for (auto &size : var.items[0].size) {
        if (std::holds_alternative<std::string_view>(size)) {
          std::cout << "[" << std::get<std::string_view>(size) << "]";
        }
        else {
          std::cout << "[" << std::get<int32_t>(size) << "]";
        }
      }
      if (is_last) {
        std::cout << ",";
      }
    }
    std::cout << ")";
  }
};

struct ParserDebugPrinter {
  void operator()(ast::ConstInt &val) const
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
    if ((parse_type<I, Args>(cont, sequence) && ...)) {
      return true;
    };
    return false;
  };

 public:
  static std::optional<Sequence> parse(TokenIterator &cont)
  {
    cont.push_waypoint();
    Sequence sequence;
    if (parse_impl(std::make_index_sequence<sizeof...(Args)>{}, cont, sequence)) {
      cont.end_waypoint(true);
      return sequence;
    }
    cont.end_waypoint(false);
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

static bool parse_keyword(TokenIterator &cont, KeywordType keyword_type)
{
  KeywordToken *keyword = cont.next<KeywordToken>();
  if (keyword && keyword->type == keyword_type) {
    return true;
  }
  else if (keyword) {
    cont.step_back();
  }
  return false;
}

template<KeywordType keyword> struct Keyword {
  static std::optional<Keyword> parse(TokenIterator &cont)
  {
    if (parse_keyword(cont, keyword)) {
      return Keyword{};
    }
    return std::nullopt;
  }
};

using ConstKeyword = Keyword<KeywordType::CONST>;
using IncludeKeyword = Keyword<KeywordType::INCLUDE>;
using UnsignedKeyword = Keyword<KeywordType::UNSIGNED>;
using StructKeyword = Keyword<KeywordType::STRUCT>;
using DefineKeyword = Keyword<KeywordType::DEFINE>;
using IntKeyword = Keyword<KeywordType::INT>;
using FloatKeyword = Keyword<KeywordType::FLOAT>;
using VoidKeyword = Keyword<KeywordType::VOID>;
using ShortKeyword = Keyword<KeywordType::SHORT>;
using CharKeyword = Keyword<KeywordType::CHAR>;
using IfKeyword = Keyword<KeywordType::IF>;
using IfDefKeyword = Keyword<KeywordType::IFDEF>;
using TypedefKeyword = Keyword<KeywordType::TYPEDEF>;
using PragmaKeyword = Keyword<KeywordType::PRAGMA>;
using OnceKeyword = Keyword<KeywordType::ONCE>;
using EnumKeyword = Keyword<KeywordType::ENUM>;
using ClassKeyword = Keyword<KeywordType::CLASS>;
using DNA_DEF_CCXKeyword = Keyword<KeywordType::DNA_DEFINE_CXX_METHODS>;
using DNADepecratedKeyword = Keyword<KeywordType::DNA_DEPRECATED>;
using EnumOperatorsKeyword = Keyword<KeywordType::ENUM_OPERATORS>;

static bool parse_symbol(TokenIterator &cont, SymbolType symbol_type)
{
  SymbolToken *symbol = cont.next<SymbolToken>();
  if (symbol && symbol->type == symbol_type) {
    return true;
  }
  else if (symbol) {
    cont.step_back();
  }
  return false;
}

template<SymbolType symbol> struct Symbol {
  static std::optional<Symbol> parse(TokenIterator &cont)
  {
    if (parse_symbol(cont, symbol)) {
      return Symbol{};
    }
    return {};
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
    TokenVariant *carried = cont.next_variant();
    for (TokenVariant *current = carried; current; current = cont.next_variant()) {
      if (std::holds_alternative<BreakLineToken>(*current) &&
          !(std::holds_alternative<SymbolToken>(*carried) &&
            std::get<SymbolToken>(*carried).type == SymbolType::BACKSLASH))
      {
        break;
      }
      carried = current;
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

std::optional<ConstInt> ConstInt::parse(TokenIterator &cont)
{
  using DefineConstIntSeq = Sequence<HashSymbol, DefineKeyword, Identifier, IntLiteral>;
  std::optional<DefineConstIntSeq> val = DefineConstIntSeq::parse(cont);
  if (!val.has_value() || !cont.next<BreakLineToken>()) {
    return std::nullopt;
  }
  return ConstInt{std::get<2>(val.value()).str, std::get<3>(val.value()).value};
}

struct PrimitiveType {
  std::string_view str;
  static std::optional<PrimitiveType> parse(TokenIterator &cont)
  {
    /* TODO: Add all primitive types. */
    using PrimitiveTypeVariants = Variant<IntKeyword,
                                          CharKeyword,
                                          Sequence<UnsignedKeyword, IntKeyword>,
                                          FloatKeyword,
                                          VoidKeyword,
                                          ShortKeyword>;
    std::optional<PrimitiveTypeVariants> type = PrimitiveTypeVariants::parse(cont);
    if (!type.has_value()) {
      return std::nullopt;
    }
    switch (type.value().index()) {
      case 0:
        return PrimitiveType{std::string_view{"int"}};
      case 1:
        return PrimitiveType{std::string_view{"char"}};
      case 2:
        return PrimitiveType{std::string_view{"unsigned int"}};
      case 3:
        return PrimitiveType{std::string_view{"float"}};
      case 4:
        return PrimitiveType{std::string_view{"void"}};
      case 5:
        return PrimitiveType{std::string_view{"short"}};
      case 6:
        return PrimitiveType{std::string_view{"char"}};
      default:
        break;
    }
    return std::nullopt;
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

template<bool stop_at_first = false>
std::optional<Variable> parse_variable_impl(TokenIterator &cont)
{
  const std::optional<TypeOrStruct> type{TypeOrStruct::parse(cont)};
  if (!type) {
    return std::nullopt;
  }
  Variable var;
  var.const_tag = type.value().const_tag;
  var.type = type.value().str;

  while (true) {
    std::string start = "";
    for (; StarSymbol::parse(cont);) {
      start += '*';
    }
    std::optional<Identifier> name{Identifier::parse(cont)};
    if (!name.has_value()) {
      return std::nullopt;
    }
    var.items.append({});
    Variable::Item &item = var.items.last();
    item.ptr = !start.empty() ? std::optional{start} : std::nullopt;
    item.name = name.value().str;
    item.size = variable_size_array_part(cont);
    DNADepecratedKeyword::parse(cont);
    if constexpr (stop_at_first) {
      break;
    }
    if (SemicolonSymbol::parse(cont).has_value()) {
      break;
    }
    else if (!CommaSymbol::parse(cont).has_value()) {
      return std::nullopt;
    }
  }
  return var;
}

std::optional<Variable> Variable::parse(TokenIterator &cont)
{
  return parse_variable_impl(cont);
}

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
    return {};
  }
  FunctionPtr fn_ptr{};
  fn_ptr.const_tag = std::get<0>(fn.value()).const_tag;
  fn_ptr.type = std::get<0>(fn.value()).str;
  fn_ptr.name = std::get<4>(fn.value()).str;
  /* Function params. */
  while (true) {
    std::optional<Variable> var = parse_variable_impl<true>(cont);
    if (!var.has_value()) {
      break;
    }
    fn_ptr.params.append(std::move(var.value()));
    if (!CommaSymbol::parse(cont).has_value()) {
      break;
    }
  }
  /* Closing sequence. */
  if (!Sequence<RParenSymbol, SemicolonSymbol>::parse(cont).has_value()) {
    return {};
  }
  return fn_ptr;
}

struct IfDefSection {
  static std::optional<IfDefSection> parse(TokenIterator &cont)
  {
    using IfDefBeginSequence = Sequence<HashSymbol, Variant<IfDefKeyword, IfKeyword>>;
    const std::optional<IfDefBeginSequence> val = IfDefBeginSequence::parse(cont);
    if (!val.has_value()) {
      return std::nullopt;
    };
    auto match_closing = [](TokenVariant *carried, TokenVariant *current) {
      return (std::holds_alternative<SymbolToken>(*carried) &&
              std::get<SymbolToken>(*carried).type == SymbolType::HASH &&
              std::holds_alternative<KeywordToken>(*current) &&
              std::get<KeywordToken>(*current).type == KeywordType::ENDIF);
    };
    auto match_opening = [](TokenVariant *carried, TokenVariant *current) {
      return (std::holds_alternative<SymbolToken>(*carried) &&
              std::get<SymbolToken>(*carried).type == SymbolType::HASH &&
              std::holds_alternative<KeywordToken>(*current) &&
              (std::get<KeywordToken>(*current).type == KeywordType::IF ||
               std::get<KeywordToken>(*current).type == KeywordType::IFDEF));
    };
    int ifdef_deep = 1;
    TokenVariant *carried = cont.next_variant();
    for (TokenVariant *current = carried; current && ifdef_deep != 0;
         current = cont.next_variant())
    {
      if (match_opening(carried, current)) {
        ifdef_deep++;
      }
      if (match_closing(carried, current)) {
        ifdef_deep--;
      }
      carried = current;
    }
    /* Not matching #endif. */
    if (ifdef_deep != 0) {
      return std::nullopt;
    }
    return IfDefSection{};
  }
};

std::optional<Struct> ast::Struct::parse(TokenIterator &cont)
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
    if (auto member = Variant<Variable, FunctionPtr>::parse(cont); member.has_value()) {
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

struct Skippers {
  static std::optional<Skippers> parse(TokenIterator &cont)
  {
    /* #pragma one. */
    if (Sequence<HashSymbol, PragmaKeyword, OnceKeyword>::parse(cont).has_value()) {
      return Skippers{};
    }
    /* Forward declare. */
    if (Sequence<StructKeyword, Identifier>::parse(cont).has_value()) {
      for (; Sequence<CommaSymbol, Identifier>::parse(cont).has_value();) {
      }
      if (SemicolonSymbol::parse(cont).has_value()) {
        return Skippers{};
      }
    }
    /* Break lines. */
    if (cont.next<BreakLineToken>()) {
      return Skippers{};
    }
    return std::nullopt;
  }
};

std::optional<Enum> ast::Enum::parse(TokenIterator &cont)
{
  /* Opening enum define sequence. */
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

  /* Skip tokens until match closing `}`. */
  int brace_deep = 1;
  for (TokenVariant *token = cont.next_variant(); token && brace_deep != 0;
       token = cont.next_variant())
  {
    if (!std::holds_alternative<SymbolToken>(*token)) {
      continue;
    }
    if (std::get<SymbolToken>(*token).type == SymbolType::RBRACE) {
      brace_deep--;
    }
    else if (std::get<SymbolToken>(*token).type == SymbolType::LBRACE) {
      brace_deep++;
    }
  }
  /* Optional enum operator defines. */
  using EnumOperatorsSeq = Sequence<EnumOperatorsKeyword,
                                    LParenSymbol,
                                    Identifier,
                                    CommaSymbol,
                                    Identifier,
                                    RParenSymbol,
                                    Optional<SemicolonSymbol>>;
  /* Closing enum defines. */
  if (!Sequence<Optional<Identifier>,
                Optional<DNADepecratedKeyword>,
                SemicolonSymbol,
                Optional<EnumOperatorsSeq>>::parse(cont)
           .has_value())
  {
    return std::nullopt;
  }
  return enum_def;
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
                                   FunctionPtr,
                                   Variable,
                                   ConstInt,
                                   Define,
                                   Include,
                                   IfDefSection,
                                   Skippers,
                                   Sequence<HashSymbol, HashSymbol, Struct>>;
    std::optional<CPPTypeVariant> val = CPPTypeVariant::parse(cont);
    if (!val.has_value()) {
     // print_unhandled_token_error(filepath, text, cont.last);
      return false;
    }
    if (std::holds_alternative<Struct>(val.value())) {
      c.append(std::move(std::get<Struct>(val.value())));
    }
    else if (std::holds_alternative<FunctionPtr>(val.value())) {
      c.append(std::move(std::get<FunctionPtr>(val.value())));
    }
    else if (std::holds_alternative<ConstInt>(val.value())) {
      c.append(std::move(std::get<ConstInt>(val.value())));
    }
    else if (std::holds_alternative<Variable>(val.value())) {
      c.append(std::move(std::get<Variable>(val.value())));
    }
    else if (std::holds_alternative<Enum>(val.value())) {
      c.append(std::move(std::get<Enum>(val.value())));
    }
  }
  for (auto &val : c) {
    // std::visit(ParserDebugPrinter{}, val);
  }

  return true;
}

}  // namespace blender::dna::parser
