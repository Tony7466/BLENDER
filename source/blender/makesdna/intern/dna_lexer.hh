#pragma once
#include "BLI_vector.hh"
#include <string_view>
#include <variant>

namespace blender::dna::lex {

enum class SymbolType : int8_t {
  COLON = 0,
  SEMICOLON,
  LPAREN,
  RPAREN,
  LBRACKET,
  RBRACKET,
  LBRACE,
  RBRACE,
  ASSIGN,
  HASH,
  DOT,
  COMMA,
  STAR,
  LESS,
  GREATHER,
  BIT_OR,
  BIT_AND,
  PLUS,
  MINUS,
  EXCLAMATION,
  PERCENT,
  CARET,
  QUESTION,
  TILDE,
  BACKSLASH,
  SLASH,
};
enum class KeywordType : int8_t {
  INCLUDE = 0,
  STRUCT,
  TYPEDEF,
  CLASS,
  ENUM,
  DEFINE,
  PUBLIC,
  PRIVATE,
  CONST,
  VOID,
  CHAR,
  CHAR16_T,
  CHAR32_T,
  UNSIGNED,
  SIGNED,
  SHORT,
  LONG,
  INT,
  INT8_T,
  INT16_T,
  INT32_T,
  INT64_T,
  UINT8_T,
  UINT16_T,
  UINT32_T,
  UINT64_T,
  FLOAT,
  DOUBLE,
  IF,
  IFDEF,
  IFNDEF,
  ENDIF,
  PRAGMA,
  ONCE,
  DNA_DEFINE_CXX_METHODS,
  DNA_DEPRECATED,
  ENUM_OPERATORS,
};
struct Token {
  std::string_view where;
};
struct BreakLineToken : public Token {};
struct IdentifierToken : public Token {};
struct StringLiteralToken : public Token {};
struct IntLiteralToken : public Token {
  int32_t val{0};
};
struct SymbolToken : public Token {
  SymbolType type;
};
struct KeywordToken : public Token {
  KeywordType type;
};

using TokenVariant = std::variant<BreakLineToken,
                                  IdentifierToken,
                                  IntLiteralToken,
                                  SymbolToken,
                                  KeywordToken,
                                  StringLiteralToken>;
struct TokenIterator {
  TokenVariant *last{nullptr};

 private:
  Vector<TokenVariant> token_stream_;
  Vector<TokenVariant *> waypoints_;
  TokenVariant *next_{nullptr};

 public:
  /* Iterates over the input text looking for tokens. */
  void process_text(std::string_view filepath, std::string_view text);

  /* Add a return point in case the token parser fails create an item. */
  void push_waypoint();

  /* Removes the las return point, if `success==false` the iterator steps back to the return point.
   */
  void end_waypoint(bool success);

  /* Return the pointer to the next token, and advances the iterator. */
  TokenVariant *next_variant();

  /* Checks if token stream dont still contains tokens. */
  bool has_finish();

  /* Appends a token. */
  template<class Type> void append(Type &&val)
  {
    token_stream_.append(val);
  }

 private:
  /* Print line where a unkown token was found. */
  void print_unkown_token(std::string_view filepath,
                          std::string_view::iterator start,
                          std::string_view::iterator where);

  void skip_break_lines();

 public:
  /** Return the next token if it type matches to `Type`.
   * Break lines are skipped for not break lines requested tokens.
   */
  template<class Type> Type *next()
  {
    TokenVariant *current_next = next_;
    if constexpr (!std::is_same_v<Type, BreakLineToken>) {
      skip_break_lines();
    }
    if (next_ < token_stream_.end() && std::holds_alternative<Type>(*next_)) {
      return &std::get<Type>(*next_++);
    }
    if (last < next_) {
      last = next_;
    }
    next_ = current_next;
    return nullptr;
  }
  KeywordToken *next_keyword(KeywordType type);
  SymbolToken *next_symbol(SymbolType type);
};

void print_line_error(std::string_view::iterator start, std::string_view::iterator where);

}  // namespace blender::dna::lex
