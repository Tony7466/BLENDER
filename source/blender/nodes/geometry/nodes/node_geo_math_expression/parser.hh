#pragma once

#include <memory>
#include <string_view>

#include "BLI_vector.hh"

#include "functions.hh"
#include "lexer.hh"

namespace blender::nodes::node_geo_math_expression_cc {

class Expression;

struct ParserError {
  Token token;
  const char *message;

  ParserError(Token token, const char *message) : token(token), message(message) {}
};

class Parser {
  Lexer lexer;
  Token peeked_token;
  bool peeked;

 public:
  std::unique_ptr<Expression> parse(std::string_view text);
  std::unique_ptr<Expression> parse_expression();
  std::unique_ptr<Expression> parse_addition();
  std::unique_ptr<Expression> parse_multiplication();
  std::unique_ptr<Expression> parse_unary();
  std::unique_ptr<Expression> parse_ufcs();
  std::unique_ptr<Expression> parse_primary();

  std::unique_ptr<Expression> parse_call(
      Token token,
      Vector<std::unique_ptr<Expression>> args = Vector<std::unique_ptr<Expression>>());

  std::unique_ptr<Expression> make_call_expression(Vector<std::unique_ptr<Expression>> args,
                                                   Token token);

  [[maybe_unused]] Token expect(TokenKind kind, const char *message);
  template<size_t N> bool match(const TokenKind (&kind)[N], Token &r_token);
  bool match(const TokenKind kind);
  bool check(const TokenKind kind);
  Token next();
  Token peek();
};

void parse_var_names(std::string_view vars, std::function<void(std::string_view)> cb);

}  // namespace blender::nodes::node_geo_math_expression_cc