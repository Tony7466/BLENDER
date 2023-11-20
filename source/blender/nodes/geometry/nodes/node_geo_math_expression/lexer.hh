#pragma once

#include <cctype>
#include <cstdint>
#include <string_view>

namespace blender::nodes::node_geo_math_expression_cc {

enum class TokenKind { IDENT, NUMBER, LPAREN, RPAREN, DOT, COMMA, PLUS, MINUS, MUL, DIV, END };

struct Token {
  TokenKind kind;
  size_t index;
  std::string_view value;

  const char *kind_str()
  {
    const char *names[] = {"IDENT",
                           "NUMBER",
                           "LPAREN",
                           "RPAREN",
                           "DOT",
                           "COMMA",
                           "PLUS",
                           "MINUS",
                           "MUL",
                           "DIV",
                           "END"};
    return names[static_cast<size_t>(kind)];
  }
};

struct LexerError {
  size_t index;
  char c;
  const char *message;

  LexerError(size_t index, char c, const char *message) : index(index), c(c), message(message) {}
};

class Lexer {
  std::string_view text;
  size_t index;
  size_t start;

 public:
  void init(std::string_view text)
  {
    this->text = text;
    index = 0;
    start = 0;
  }

  Token next_token()
  {
    if (end()) {
      start = index;
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
      case '.':
        if (!std::isdigit(peek())) {
          return make_token(TokenKind::DOT);
        }

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
    }

    if (c == '_' || isalpha(c)) {
      return parse_identifier();
    }

    if (c == '.' || isdigit(c)) {
      return parse_number(c);
    }

    throw make_error("unexpected character");
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
      }
      else {
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
      }
      else {
        break;
      }
    }

    return make_token(TokenKind::IDENT);
  }

  LexerError make_error(const char *message)
  {
    return LexerError(index - 1, text[index - 1], message);
  }

  Token make_token(TokenKind kind)
  {
    return Token{kind, start, text.substr(start, index - start)};
  }

  bool match(char c)
  {
    if (!end() && peek() == c) {
      next();
      return true;
    }

    return false;
  }

  bool end()
  {
    return index >= text.size();
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

}  // namespace blender::nodes::node_geo_math_expression_cc