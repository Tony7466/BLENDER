#include "dna_lexer.hh"
#include <cctype>
#include <charconv>
#include <fmt/format.h>

namespace blender::dna::lex {

static std::string_view string_view_from_range(const std::string_view::iterator first,
                                               const std::string_view::iterator last)
{
  return std::string_view{&*first, size_t(last - first)};
}

void TokenIterator::eval_space(std::string_view::iterator &itr, std::string_view::iterator last)
{
  while (itr < last && itr[0] != '\n' && std::isspace(itr[0])) {
    itr++;
  }
}

void TokenIterator::eval_break_line(std::string_view::iterator &itr,
                                    std::string_view::iterator /*last*/)
{
  if (itr[0] != '\n') {
    return;
  }
  this->append(BreakLineToken{string_view_from_range(itr, itr + 1)});
  itr++;
}

void TokenIterator::eval_identifier(std::string_view::iterator &itr,
                                    std::string_view::iterator last)
{
  if (!(std::isalpha(itr[0]) || itr[0] == '_')) {
    return;
  }
  std::string_view::iterator start{itr++};
  while (itr < last && (std::isalnum(itr[0]) || itr[0] == '_')) {
    itr++;
  }
  struct KeywordItem {
    std::string_view word;
    KeywordType type;
  };
  using namespace std::string_view_literals;
  static constexpr KeywordItem keywords[]{
      {"BLI_STATIC_ASSERT_ALIGN"sv, KeywordType::BLI_STATIC_ASSERT_ALIGN},
      {"DNA_DEFINE_CXX_METHODS"sv, KeywordType::DNA_DEFINE_CXX_METHODS},
      {"DNA_DEPRECATED"sv, KeywordType::DNA_DEPRECATED},
      {"DNA_DEPRECATED_ALLOW"sv, KeywordType::DNA_DEPRECATED_ALLOW},
      {"ENUM_OPERATORS"sv, KeywordType::ENUM_OPERATORS},
      {"extern"sv, KeywordType::EXTERN},
      {"char"sv, KeywordType::CHAR},
      {"char16_t"sv, KeywordType::CHAR16_T},
      {"char32_t"sv, KeywordType::CHAR32_T},
      {"class"sv, KeywordType::CLASS},
      {"const"sv, KeywordType::CONST},
      {"define"sv, KeywordType::DEFINE},
      {"double"sv, KeywordType::DOUBLE},
      {"endif"sv, KeywordType::ENDIF},
      {"enum"sv, KeywordType::ENUM},
      {"float"sv, KeywordType::FLOAT},
      {"if"sv, KeywordType::IF},
      {"ifdef"sv, KeywordType::IFDEF},
      {"ifndef"sv, KeywordType::IFNDEF},
      {"include"sv, KeywordType::INCLUDE},
      {"int"sv, KeywordType::INT},
      {"int16_t"sv, KeywordType::INT16_T},
      {"int32_t"sv, KeywordType::INT32_T},
      {"int64_t"sv, KeywordType::INT64_T},
      {"int8_t"sv, KeywordType::INT8_T},
      {"long"sv, KeywordType::LONG},
      {"ulong"sv, KeywordType::ULONG},
      {"once"sv, KeywordType::ONCE},
      {"pragma"sv, KeywordType::PRAGMA},
      {"private"sv, KeywordType::PRIVATE},
      {"public"sv, KeywordType::PUBLIC},
      {"short"sv, KeywordType::SHORT},
      {"signed"sv, KeywordType::SIGNED},
      {"struct"sv, KeywordType::STRUCT},
      {"typedef"sv, KeywordType::TYPEDEF},
      {"uint16_t"sv, KeywordType::UINT16_T},
      {"uint32_t"sv, KeywordType::UINT32_T},
      {"uint64_t"sv, KeywordType::UINT64_T},
      {"uint8_t"sv, KeywordType::UINT8_T},
      {"unsigned"sv, KeywordType::UNSIGNED},
      {"void"sv, KeywordType::VOID},
  };

  std::string_view str = string_view_from_range(start, itr);
  auto test_keyword_fn = [str](const KeywordItem &val) -> bool { return val.word == str; };
  const KeywordItem *keyword_itr = std::find_if(
      std::begin(keywords), std::end(keywords), test_keyword_fn);
  if (keyword_itr != std::end(keywords)) {
    this->append(KeywordToken{str, keyword_itr->type});
    return;
  }
  this->append(IdentifierToken{str});
}

void TokenIterator::eval_line_comment(std::string_view::iterator &itr,
                                      std::string_view::iterator last)
{
  if (last - itr < 2) {
    return;
  }
  if (!(itr[0] == '/' && itr[1] == '/')) {
    return;
  }
  while (itr != last && itr[0] != '\n') {
    itr++;
  }
}

void TokenIterator::eval_int_literal(std::string_view::iterator &itr,
                                     std::string_view::iterator last)
{
  const std::string_view::iterator start{itr};
  while (itr < last && std::isdigit(itr[0])) {
    itr++;
  }
  if (itr == start) {
    return;
  }
  int val{};
  std::from_chars(&*start, &*itr, val);
  this->append(IntLiteralToken{string_view_from_range(start, itr), val});
}

void TokenIterator::eval_multiline_comment(std::string_view::iterator &itr,
                                           std::string_view::iterator last)
{
  if (last - itr < +2) {
    return;
  }
  if (!(itr[0] == '/' && itr[1] == '*')) {
    return;
  }
  char carry = itr[0];
  itr += 2;
  while (itr < last && !(carry == '*' && itr[0] == '/')) {
    carry = itr[0];
    itr++;
  }
  if (itr < last) {
    itr++;
  }
}

void TokenIterator::eval_symbol(std::string_view::iterator &itr,
                                std::string_view::iterator /*last*/)
{
  static constexpr SymbolType symbols[] = {
      SymbolType::Colon,     SymbolType::Semicolon, SymbolType::LParen,   SymbolType::RParen,
      SymbolType::LBracket,  SymbolType::RBracket,  SymbolType::LBrace,   SymbolType::RBrace,
      SymbolType::Assign,    SymbolType::Hash,      SymbolType::Dot,      SymbolType::Comma,
      SymbolType::Star,      SymbolType::Less,      SymbolType::Greater,  SymbolType::BitOr,
      SymbolType::BitAnd,    SymbolType::Plus,      SymbolType::Minus,    SymbolType::Exclamation,
      SymbolType::Percent,   SymbolType::Caret,     SymbolType::Question, SymbolType::Tilde,
      SymbolType::Backslash, SymbolType::Slash,
  };
  if (std::find(std::begin(symbols), std::end(symbols), SymbolType(itr[0])) != std::end(symbols)) {
    this->append(SymbolToken{string_view_from_range(itr, itr + 1), SymbolType(itr[0])});
    itr++;
  }
}

void TokenIterator::eval_string_literal(std::string_view::iterator &itr,
                                        std::string_view::iterator last)
{
  const char opening = itr[0];
  if (!(opening == '"' || opening == '\'')) {
    return;
  }
  const std::string_view::iterator start{itr++};
  bool scape{false};
  while (itr < last && !(!scape && itr[0] == opening)) {
    scape = itr[0] == '\\' && !scape;
    itr++;
  }
  if (!(itr < last)) {
    itr = start;
    return;
  }
  itr++;
  this->append(StringLiteralToken{string_view_from_range(start, itr)});
}

void TokenIterator::print_unkown_token(std::string_view filepath,
                                       std::string_view::iterator start,
                                       std::string_view::iterator where)
{
  size_t line = 1;
  while (start < where) {
    if (start[0] == '\n') {
      line++;
    }
    start++;
  }
  fmt::print("{}({}) Unknown token: ({})\n", filepath, line, where[0]);
}

void TokenIterator::skip_break_lines()
{
  while (next_ < token_stream_.end() && std::holds_alternative<BreakLineToken>(*next_)) {
    next_++;
  }
}

void TokenIterator::process_text(std::string_view filepath, std::string_view text)
{
  std::string_view::iterator itr = text.begin();
  const std::string_view::iterator end = text.end();
  const auto eval_token_type =
      [this, &itr, end](void (TokenIterator::*fn)(std::string_view::iterator &,
                                                  std::string_view::iterator)) -> bool {
    std::string_view::iterator current = itr;
    (this->*fn)(itr, end);
    return current != itr;
  };

  while (itr != text.end()) {
    const std::string_view::iterator current = itr;

    if (eval_token_type(&TokenIterator::eval_space) ||
        eval_token_type(&TokenIterator::eval_line_comment) ||
        eval_token_type(&TokenIterator::eval_multiline_comment) ||
        eval_token_type(&TokenIterator::eval_identifier) ||
        eval_token_type(&TokenIterator::eval_int_literal) ||
        eval_token_type(&TokenIterator::eval_string_literal) ||
        eval_token_type(&TokenIterator::eval_symbol) ||
        eval_token_type(&TokenIterator::eval_break_line))
    {
      continue;
    }
    /* Unknown token found. */
    if (current == itr) {
      print_unkown_token(filepath, text.begin(), itr);
      token_stream_.clear();
      break;
    }
  }
  next_ = token_stream_.begin();
};

TokenVariant *TokenIterator::next_variant()
{
  if (next_ < token_stream_.end()) {
    return next_++;
  }
  return nullptr;
}

bool TokenIterator::has_finish()
{
  return !(next_ < token_stream_.end());
}

void TokenIterator::push_waypoint()
{
  waypoints_.append(next_);
}

void TokenIterator::end_waypoint(bool success)
{
  if (!success) {
    if (last_unmatched < next_) {
      last_unmatched = next_;
    }
    next_ = waypoints_.last();
  }
  waypoints_.remove_last();
}

KeywordToken *TokenIterator::next_keyword(KeywordType type)
{
  TokenVariant *tmp = next_;
  if (KeywordToken *keyword = next<KeywordToken>(); keyword && keyword->type == type) {
    return keyword;
  }
  next_ = tmp;
  return nullptr;
}

SymbolToken *TokenIterator::next_symbol(SymbolType type)
{
  TokenVariant *tmp = next_;
  if (SymbolToken *symbol = next<SymbolToken>(); symbol && symbol->type == type) {
    return symbol;
  }
  next_ = tmp;
  return nullptr;
}

}  // namespace blender::dna::lex
