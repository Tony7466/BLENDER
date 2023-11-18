#pragma once

#include <iostream>
#include <memory>
#include <cstdio>
#include <string>
#include <vector>
#include <set>

#include "lexer.hh"
#include "expression.hh"

namespace blender::nodes::node_geo_math_expression_cc {

struct ParserError {
  Token token;
  const char *message;
};

class Parser {;
  Lexer lexer;
  Token peeked_token;
  bool peeked;

public:
  std::unique_ptr<Expression> parse(std::string_view text)
  {
    peeked = false;
    lexer.init(text);
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

    if(token.kind == TokenKind::END) {
      throw ParserError { token, "unexpected end of expression" };
    } else {
      throw ParserError { token, "unexpected token" };
    }
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

    peeked_token = lexer.next_token();
    peeked = true;
    return peeked_token;
  }
};

inline void parse_var_names(std::string_view vars, std::function<void(std::string_view)> cb) {
  size_t start = vars.size();

  for(size_t i = 0; i < vars.size(); i++) {
    char c = vars[i];

    if(start == vars.size() && (c == '_' || isalpha(c))) {
      start = i;
    } else if(start != vars.size() && (c != '_' && !isalnum(c))) {
      cb(vars.substr(start, i - start));
      start = vars.size();
    }
    
    if(start != vars.size() && i == vars.size() - 1) {
      cb(vars.substr(start, i + 1 - start));
    }
  }
}

}