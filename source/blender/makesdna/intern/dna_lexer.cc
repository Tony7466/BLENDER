#include "dna_lexer.hh"
#include <charconv>
#include <iostream>

#include <algorithm>
namespace blender::dna::lex {

/* Match any withe space except break lines. */
static void eval_space(std::string_view::iterator &itr,
                       std::string_view::iterator last,
                       TokenIterator & /*cont*/)
{
  while (itr < last && itr[0] != '\n' && std::isspace(itr[0])) {
    itr++;
  }
}

/* Match break lines, added as tokens since are needed for `#define` blocks. */
static void eval_break_line(std::string_view::iterator &itr,
                            std::string_view::iterator /*last*/,
                            TokenIterator &cont)
{
  if (itr[0] != '\n') {
    return;
  }
  cont.append(BreakLineToken{});
  itr++;
}

/* Match any identifier substring, also matches c++ keywords. */
static void eval_identifier(std::string_view::iterator &itr,
                            std::string_view::iterator last,
                            TokenIterator &cont)
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
  constexpr KeywordItem keywords[]{
      {"DNA_DEFINE_CXX_METHODS", KeywordType::DNA_DEFINE_CXX_METHODS},
      {"DNA_DEPRECATED", KeywordType::DNA_DEPRECATED},
      {"ENUM_OPERATORS", KeywordType::ENUM_OPERATORS},
      {"char", KeywordType::CHAR},
      {"char16_t", KeywordType::CHAR16_T},
      {"char32_t", KeywordType::CHAR32_T},
      {"class", KeywordType::CLASS},
      {"const", KeywordType::CONST},
      {"define", KeywordType::DEFINE},
      {"double", KeywordType::DOUBLE},
      {"endif", KeywordType::ENDIF},
      {"enum", KeywordType::ENUM},
      {"float", KeywordType::FLOAT},
      {"if", KeywordType::IF},
      {"ifdef", KeywordType::IFDEF},
      {"include", KeywordType::INCLUDE},
      {"int", KeywordType::INT},
      {"int16_t", KeywordType::INT16_T},
      {"int32_t", KeywordType::INT32_T},
      {"int64_t", KeywordType::INT64_T},
      {"int8_t", KeywordType::INT8_T},
      {"long", KeywordType::LONG},
      {"once", KeywordType::ONCE},
      {"pragma", KeywordType::PRAGMA},
      {"private", KeywordType::PRIVATE},
      {"public", KeywordType::PUBLIC},
      {"short", KeywordType::SHORT},
      {"signed", KeywordType::SIGNED},
      {"struct", KeywordType::STRUCT},
      {"typedef", KeywordType::TYPEDEF},
      {"uint16_t", KeywordType::UINT16_T},
      {"uint32_t", KeywordType::UINT32_T},
      {"uint64_t", KeywordType::UINT64_T},
      {"uint8_t", KeywordType::UINT8_T},
      {"unsigned", KeywordType::UNSIGNED},
      {"void", KeywordType::VOID},
  };

  std::string_view str{start._Unwrapped(), size_t(itr - start)};
  auto test_keyword_fn = [str](const KeywordItem &val) -> bool { return val.word == str; };
  auto keyword_itr = std::find_if(std::begin(keywords), std::end(keywords), test_keyword_fn);
  if (keyword_itr != std::end(keywords)) {
    cont.append(KeywordToken{str, keyword_itr->type});
    return;
  }
  cont.append(IdentifierToken{str});
}

/* Match a line comment until break line. */
static void eval_line_comment(std::string_view::iterator &itr,
                              std::string_view::iterator last,
                              TokenIterator & /*cont*/)
{
  if (std::distance(itr, last) < 2) {
    return;
  }
  if (!(itr[0] == '/' && itr[1] == '/')) {
    return;
  }
  std::string_view::iterator start{itr++};
  while (itr != last && itr[0] != '\n') {
    itr++;
  }
}

/* Match a int literal. */
static void eval_int_literal(std::string_view::iterator &itr,
                             std::string_view::iterator last,
                             TokenIterator &cont)
{
  std::string_view::iterator start{itr};
  while (itr < last && std::isdigit(itr[0])) {
    itr++;
  }
  if (itr == start) {
    return;
  }
  int val{};
  const std::string_view str{start._Unwrapped(), size_t(itr - start)};
  auto transform_Result = std::from_chars(start._Unwrapped(), itr._Unwrapped(), val);
  cont.append(IntLiteralToken{str, val});
}

/* Match a c-style comment. */
static void eval_multiline_comment(std::string_view::iterator &itr,
                                   std::string_view::iterator last,
                                   TokenIterator & /*cont*/)
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

/* Match a symbol. */
static void eval_symbol(std::string_view::iterator &itr,
                        std::string_view::iterator /*last*/,
                        TokenIterator &cont)
{
  struct SymbolItem {
    char value;
    SymbolType type;
  };
  constexpr SymbolItem symbols[] = {
      {'!', SymbolType::EXCLAMATION}, {'#', SymbolType::HASH},       {'%', SymbolType::PERCENT},
      {'&', SymbolType::BIT_AND},     {'(', SymbolType::LPAREN},     {')', SymbolType::RPAREN},
      {'*', SymbolType::STAR},        {'+', SymbolType::PLUS},       {',', SymbolType::COMMA},
      {'-', SymbolType::MINUS},       {'.', SymbolType::DOT},        {'/', SymbolType::SLASH},
      {':', SymbolType::COLON},       {';', SymbolType::SEMICOLON},  {'<', SymbolType::LESS},
      {'=', SymbolType::ASSIGN},      {'>', SymbolType::GREATHER},   {'?', SymbolType::QUESTION},
      {'[', SymbolType::LBRACKET},    {'\\', SymbolType::BACKSLASH}, {']', SymbolType::RBRACKET},
      {'^', SymbolType::CARET},       {'{', SymbolType::LBRACE},     {'|', SymbolType::BIT_OR},
      {'}', SymbolType::RBRACE},      {'~', SymbolType::TILDE},
  };
  auto test_symbol = [itr](const SymbolItem &item) -> bool { return item.value == itr[0]; };
  auto symbol_itr = std::find_if(std::begin(symbols), std::end(symbols), test_symbol);
  if (symbol_itr != std::end(symbols)) {
    const std::string_view str{itr._Unwrapped(), 1};
    cont.append(SymbolToken{str, symbol_itr->type});
    itr++;
  }
}

/* Match a string literal. */
static void eval_string_literal(std::string_view::iterator &itr,
                                std::string_view::iterator last,
                                TokenIterator &cont)
{
  if (!(itr[0] == '"')) {
    return;
  }
  std::string_view::iterator start{itr};
  char carry = '"';
  itr++;
  while (itr < last && !(carry != '\\' && itr[0] == '"')) {
    carry = itr[0];
    itr++;
  }
  if (!(itr < last)) {
    itr = start;
    return;
  }
  itr++;
  cont.append(StringLiteralToken{std::string_view{start._Unwrapped(), size_t(itr - start)}});
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
  std::cout << filepath << "(" << line << ") Unknown token: (" << (where[0]) << ")\n";
}

void TokenIterator::skip_break_lines()
{
  if (!next_) {
    return;
  }
  while (next_ < token_stream_.end() && (std::holds_alternative<BreakLineToken>(*next_))) {
    next_++;
  }
}

void TokenIterator::process_text(std::string_view filepath, std::string_view text)
{
  std::string_view::iterator itr = text.begin();
  const std::string_view::iterator end = text.end();

  auto eval_token =
      [this](std::string_view::iterator &itr, std::string_view::iterator end, auto &&fn) -> bool {
    std::string_view::iterator current = itr;
    fn(itr, end, *this);
    return current != itr;
  };

  while (itr != text.end()) {
    const std::string_view::iterator current = itr;

    if (eval_token(itr, end, eval_space) || eval_token(itr, end, eval_line_comment) ||
        eval_token(itr, end, eval_multiline_comment) || eval_token(itr, end, eval_identifier) ||
        eval_token(itr, end, eval_int_literal) || eval_token(itr, end, eval_string_literal) ||
        eval_token(itr, end, eval_symbol) || eval_token(itr, end, eval_break_line))
    {
      continue;
    }
    /* A token could not be parsed. */
    if (current == itr) {
      print_unkown_token(filepath, text.begin(), itr);
      token_stream_.clear();
      break;
    }
  }
  if (!token_stream_.is_empty()) {
    next_ = token_stream_.begin();
  }
  else {
    next_ = nullptr;
  }
};

TokenVariant *TokenIterator::next_variant()
{
  if (next_ && next_ < token_stream_.end()) {
    return next_++;
  }
  return nullptr;
}

void TokenIterator::step_back()
{
  if (last < next_) {
    last = next_;
  }
  next_--;
}

bool TokenIterator::has_finish()
{
  return next_ == nullptr || !(next_ < token_stream_.end());
}

void TokenIterator::push_waypoint()
{
  waypoints_.append(next_);
}

void TokenIterator::end_waypoint(bool success)
{
  if (!success) {
    if (last < next_) {
      last = next_;
    }
    next_ = waypoints_.last();
  }
  waypoints_.remove_last();
}
}  // namespace blender::dna::lex
