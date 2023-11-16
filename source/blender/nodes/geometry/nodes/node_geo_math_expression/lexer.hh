#pragma once

#include <string>
#include <cctype>
#include <cstdint>

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
  void init(const char* text) {
      this->text = text;
      index = 0;
  }

  Token next_token()
  {
    if (end()) {
      return make_token(TokenKind::END);
    }

    start = index;
    char c = next();

    if (isspace(c)) {
      return next_token();
    }

    switch (c) {
    case '(':
      return make_token(TokenKind::LPAREN);
      break;
    case ')':
      return make_token(TokenKind::RPAREN);
      break;
    case ',':
      return make_token(TokenKind::COMMA);
      break;
    case '+':
      return make_token(TokenKind::PLUS);
      break;
    case '-':
      return make_token(TokenKind::MINUS);
      break;
    case '*':
      return make_token(TokenKind::MUL);
      break;
    case '/':
      return make_token(TokenKind::DIV);
      break;
    case '^':
      return make_token(TokenKind::EXP);
      break;
    }

    if (c == '_' || isalpha(c)) {
      return parse_identifier();
    }

    if (c == '.' || isdigit(c)) {
      return parse_number(c);
    }

    throw LexerError { start, "unexpected character" };
  }

private:
  Token parse_number(char c)
  {
    if (c == '.') {
      if (end() || !isdigit(peek())) {
          throw make_error("expected digit");
      }

      consume_number();
    }
    else {
      consume_number();

      if (match('.')) {
          consume_number();
      }
    }

    return make_token(TokenKind::NUMBER);
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

  Token parse_identifier()
  {
    while (!end()) {
      char c = peek();

      if (c == '_' || isalnum(c)) {
        next();
      } else {
        break;
      }
    }

    return make_token(TokenKind::IDENT);
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
