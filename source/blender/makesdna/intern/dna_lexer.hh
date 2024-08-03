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
  GREATER,
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
  ULONG,
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
  EXTERN,
  PRAGMA,
  ONCE,
  DNA_DEFINE_CXX_METHODS,
  DNA_DEPRECATED,
  ENUM_OPERATORS,
  BLI_STATIC_ASSERT_ALIGN,
  DNA_DEPRECATED_ALLOW
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
  /** Last token that fails to match a token request. */
  TokenVariant *last_unmatched{nullptr};

 private:
  /** Token stream. */
  Vector<TokenVariant> token_stream_;
  /** Return points to use for roll back when parser fails to parse tokens. */
  Vector<TokenVariant *> waypoints_;
  /** Pointer to next token to iterate. */
  TokenVariant *next_{nullptr};

  /** Print the line where an unkown token was found. */
  void print_unkown_token(std::string_view filepath,
                          std::string_view::iterator start,
                          std::string_view::iterator where);

  void skip_break_lines();

 public:
  /** Iterates over the input text looking for tokens. */
  void process_text(std::string_view filepath, std::string_view text);

  /**
   * Add the current token as waypoint, in case the token parser needs the iterator to roll
   * back.
   */
  void push_waypoint();

  /**
   * Removes the last waypoint, if `success==false` the iterator rolls back to this last
   * waypoint.
   */
  void end_waypoint(bool success);

  /** Return the pointer to the next token, and advances the iterator. */
  TokenVariant *next_variant();

  /** Checks if the token iterator has reach the last token. */
  bool has_finish();

  /** Appends a token. */
  template<class TokenType> void append(TokenType &&token)
  {
    token_stream_.append(std::move(token));
  }

  /**
   * Return the next token if it type matches to `Type`.
   * Break lines are skipped for non break line requested tokens.
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
    if (last_unmatched < next_) {
      last_unmatched = next_;
    }
    next_ = current_next;
    return nullptr;
  }

  /** Return the next token if it matches to the requested keyword type. */
  KeywordToken *next_keyword(KeywordType type);

  /** Return the next token if it matches to the requested symbol type. */
  SymbolToken *next_symbol(SymbolType type);
};

}  // namespace blender::dna::lex
