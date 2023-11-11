#pragma once

#include <string>
#include <cctype>
#include <cstdint>

using size_t = std::size_t;

enum class TokenKind { IDENT, NUMBER, LPAREN, RPAREN, COMMA, PLUS, MINUS, MUL, DIV, EXP, END };

struct Token {
  TokenKind kind;
  size_t index;
  std::string_view value;

  const char *kind_str() {
    const char *names[] = { "IDENT", "NUMBER", "LPAREN", "RPAREN", "COMMA", "PLUS", "MINUS", "MUL", "DIV", "EXP", "END" };
    return names[static_cast<size_t>(kind)];
  }
};

struct LexerError {
  size_t index;
  const char *message;
};

class Lexer {
  const char *text;
  size_t index;
  size_t start;

public:
  void parse(const char* text) {
      this->text = text;
      index = 0;
  }

  bool next_token(Token &r_token, LexerError &r_error)
  {
    if (end()) {
      r_token = make_token(TokenKind::END);
      return true;
    }

    start = index;
    r_error = make_error("unexpected character");
    char c = next();
    bool handled = true;

    if (isspace(c)) {
      return next_token(r_token, r_error);
    }

    switch (c) {
    case '(':
      r_token = make_token(TokenKind::LPAREN);
      break;
    case ')':
      r_token = make_token(TokenKind::RPAREN);
      break;
    case ',':
      r_token = make_token(TokenKind::COMMA);
      break;
    case '+':
      r_token = make_token(TokenKind::PLUS);
      break;
    case '-':
      r_token = make_token(TokenKind::MINUS);
      break;
    case '*':
      r_token = make_token(TokenKind::MUL);
      break;
    case '/':
      r_token = make_token(TokenKind::DIV);
      break;
    case '^':
      r_token = make_token(TokenKind::EXP);
      break;
    default:
      handled = false;
      break;
    }

    if (handled) {
      return true;
    }

    if (c == '_' || isalpha(c)) {
      return parse_identifier(r_token, r_error);
    }

    if (c == '.' || isdigit(c)) {
      return parse_number(r_token, r_error, c);
    }

    return false;
  }

private:
  bool parse_number(Token &r_token, LexerError &r_error, char c)
  {
    if (c == '.') {
      if (end() || !isdigit(peek())) {
          r_error = make_error("expected digit");
          return false;
      }

      consume_number();
    }
    else {
      consume_number();

      if (match('.')) {
          consume_number();
      }
    }

    r_token = make_token(TokenKind::NUMBER);
    return true;
  }

  void consume_number()
  {
    while (!end()) {
      char c = peek();

      if (isdigit(c)) {
        next();
      } else {
        break;
      }
    }
  }

  bool parse_identifier(Token &r_token, LexerError r_error)
  {
    while (!end()) {
      char c = peek();

      if (c == '_' || isalnum(c)) {
        next();
      } else {
        break;
      }
    }

    r_token = make_token(TokenKind::IDENT);
    return true;
  }

  LexerError make_error(const char* message) {
    return LexerError{index, message};
  }

  Token make_token(TokenKind kind) {
    return Token{ kind, start, std::string_view(text + start, index - start) };
  }

  bool match(char c) {
    if (!end() && peek() == c) {
      next();
      return true;
    }

    return false;
  }

  bool end()
  {
    return text[index] == '\0';
  }

  char next()
  {
    return text[index++];
  }

  char peek()
  {
    return text[index];
  }
};
