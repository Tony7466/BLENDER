#pragma once
#include "BLI_vector.hh"
#include <string_view>
#include <variant>

namespace blender::dna::lex {

enum class SymbolType : char {
  Colon = ':',
  Semicolon = ';',
  LParen = '(',
  RParen = ')',
  LBracket = '[',
  RBracket = ']',
  LBrace = '{',
  RBrace = '}',
  Assign = '=',
  Hash = '#',
  Dot = '.',
  Comma = ',',
  Star = '*',
  Less = '<',
  Greater = '>',
  BitOr = '|',
  BitAnd = '&',
  Plus = '+',
  Minus = '-',
  Exclamation = '!',
  Percent = '%',
  Caret = '^',
  Question = '?',
  Tilde = '~',
  Backslash = '\\',
  Slash = '/',
};

enum class KeywordType : int8_t {
  Include = 0, /* include */
  Struct,      /* struct */
  Typedef,     /* typedef */
  Class,       /* class */
  Enum,        /* enum */
  Define,      /* define */
  Public,      /* public */
  Private,     /* private */
  Const,       /* const */
  Void,        /* void */
  Char,        /* char */
  Char16_t,    /* char16_t */
  Char32_t,    /* char32_t */
  Unsigned,    /* unsigned */
  Signed,      /* signed */
  Short,       /* short */
  Long,        /* long */
  Ulong,       /* ulong */
  Int,         /* int */
  Int8_t,      /* int8_t */
  Int16_t,     /* int16_t */
  Int32_t,     /* int32_t */
  Int64_t,     /* int64_t */
  Uint8_t,     /* uint8_t */
  Uint16_t,    /* uint16_t */
  Uint32_t,    /* uint32_t */
  Uint64_t,    /* uint64_t */
  Float,       /* float */
  Double,      /* double */
  If,          /* if */
  Ifdef,       /* ifdef */
  Ifndef,      /* ifndef */
  Endif,       /* endif */
  Extern,      /* extern */
  Pragma,      /* pragma */
  Once,        /* once */
  /* Common Blender macros in DNA. */
  BLIStaticAssertAlign, /* BLI_STATIC_ASSERT_ALIGN */
  DNADefineCxxMethods,  /* DNA_DEFINE_CXX_METHODS */
  DNADeprecated,        /* DNA_DEPRECATED */
  DNADeprecatedAllow,   /* DNA_DEPRECATED_ALLOW */
  EnumOperators,        /* ENUM_OPERATORS */
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

 private:
  /** Match any whitespace except break lines. */
  void eval_space(std::string_view::iterator &itr, std::string_view::iterator last);
  /** Match break lines. */
  void eval_break_line(std::string_view::iterator &itr, std::string_view::iterator last);
  /** Match identifiers and `C++` keywords. */
  void eval_identifier(std::string_view::iterator &itr, std::string_view::iterator last);
  /** Match single-line comment. */
  void eval_line_comment(std::string_view::iterator &itr, std::string_view::iterator last);
  /** Match a int literal. */
  void eval_int_literal(std::string_view::iterator &itr, std::string_view::iterator last);
  /** Match a multi-line comment. */
  void eval_multiline_comment(std::string_view::iterator &itr, std::string_view::iterator last);
  /** Match a symbol. */
  void eval_symbol(std::string_view::iterator &itr, std::string_view::iterator last);
  /** Match a string or char literal. */
  void eval_string_literal(std::string_view::iterator &itr, std::string_view::iterator last);

  /** Appends a token. */
  template<class TokenType> void append(TokenType &&token)
  {
    token_stream_.append(std::move(token));
  }
};

}  // namespace blender::dna::lex
