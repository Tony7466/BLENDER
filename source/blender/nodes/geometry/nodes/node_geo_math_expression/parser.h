#pragma once

#include <iostream>
#include <memory>
#include <cstdio>
#include <string>
#include <vector>

#include "lexer.h"
#include "expression.h"

struct ParserError {
  Token token;
  const char *message;
};

class Parser {;
  const char *text;
  Lexer lexer;
  Token peeked_token;
  bool peeked;
  std::vector<std::string_view> *variables;

public:
  std::unique_ptr<Expression> parse(const char *text, std::vector<std::string_view> &r_variables) {
    variables = &r_variables;
    variables->clear();
    peeked = false;
    this->text = text;
    lexer.parse(text);
    auto expr = parse_expression();

    expect(TokenKind::END, "expected end of input");

    return expr;
  }

  std::unique_ptr<Expression> parse_expression() {
    return parse_addition();
  }

  std::unique_ptr<Expression> parse_addition() {
    auto expr = parse_multiplication();

    Token token;
    while(match({TokenKind::PLUS, TokenKind::MINUS}, token)) {
      expr = std::make_unique<BinaryExpression>(std::move(expr), parse_multiplication(), token);
    }

    return expr;
  }

  std::unique_ptr<Expression> parse_multiplication() {
    auto expr = parse_unary();
    Token token;

    while(match({TokenKind::MUL, TokenKind::DIV}, token)) {
      expr = std::make_unique<BinaryExpression>(std::move(expr), parse_unary(), token);
    }

    return expr;
  }

  std::unique_ptr<Expression> parse_unary() {
    Token token;

    if(match({TokenKind::MINUS}, token)) {
      return std::make_unique<UnaryExpression>(parse_unary(), token);
    }

    return parse_primary();
  }

  std::unique_ptr<Expression> parse_primary() {
    Token token = next();

    if(token.kind == TokenKind::IDENT) {
      if(match(TokenKind::LPAREN)) {
        return parse_call(token);
      }

      variables->emplace_back(token.value);
      return std::make_unique<VariableExpression>(token);
    }

    if(token.kind == TokenKind::NUMBER) {
      return std::make_unique<NumberExpression>(token);
    }

    if(token.kind == TokenKind::LPAREN) {
      auto expr = std::make_unique<GroupExpression>(parse_expression(), token);
      expect(TokenKind::RPAREN, "expected closing paren");
      return expr;
    }

    throw ParserError { token, "unexpected token" };
  }

  std::unique_ptr<Expression> parse_call(Token token) {
    std::vector<std::unique_ptr<Expression>> args;

    do {
      args.emplace_back(parse_expression());
    } while(match(TokenKind::COMMA));

    expect(TokenKind::RPAREN, "expected closing paren");

    return std::make_unique<CallExpression>(std::move(args), token);
  }

  void expect(TokenKind kind, const char *message) {
    if(peek().kind != kind) {
      throw ParserError{ peek(), message };
    }

    next();
  }

  template<size_t N>
  bool match(const TokenKind (&kind)[N], Token &r_token) {
    for(size_t i = 0; i < N; i++) {
      if(peek().kind == kind[i]) {
        r_token = next();
        return true;
      }
    }

    return false;
  }

  bool match(const TokenKind kind) {
    if(peek().kind == kind) {
      next();
      return true;
    }

    return false;
  }

  Token next() {
    Token token = peek();
    peeked = false;
    return token;
  }

  Token peek() {
    if(peeked) {
      return peeked_token;
    }

    LexerError error;

    if (!lexer.next_token(peeked_token, error)) {
      throw error;
    }

    peeked = true;
    return peeked_token;
  }
};