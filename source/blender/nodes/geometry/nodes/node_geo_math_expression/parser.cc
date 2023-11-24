#include <cmath>

#include "expression.hh"
#include "parser.hh"

namespace blender::nodes::node_geo_math_expression_cc {

std::unique_ptr<Expression> Parser::parse(std::string_view text)
{
  peeked = false;
  lexer.init(text);
  auto expr = parse_expression();

  expect(TokenKind::END, "expected end of input");

  return expr;
}

std::unique_ptr<Expression> Parser::parse_expression()
{
  return parse_addition();
}

std::unique_ptr<Expression> Parser::parse_addition()
{
  auto expr = parse_multiplication();

  Token token;
  while (match({TokenKind::PLUS, TokenKind::MINUS}, token)) {
    expr = std::make_unique<BinaryExpression>(std::move(expr), parse_multiplication(), token);
  }

  return expr;
}

std::unique_ptr<Expression> Parser::parse_multiplication()
{
  auto expr = parse_unary();
  Token token;

  while (match({TokenKind::MUL, TokenKind::DIV}, token)) {
    expr = std::make_unique<BinaryExpression>(std::move(expr), parse_unary(), token);
  }

  return expr;
}

std::unique_ptr<Expression> Parser::parse_unary()
{
  Token token;

  if (match({TokenKind::MINUS}, token)) {
    return std::make_unique<UnaryExpression>(parse_unary(), token);
  }

  return parse_ufcs();
}

std::unique_ptr<Expression> Parser::parse_ufcs()
{
  auto expr = parse_primary();
  Token token;

  while (match({TokenKind::DOT}, token)) {
    Token ident = expect(TokenKind::IDENT, "expected identifier");
    Vector<std::unique_ptr<Expression>> args;
    args.append(std::move(expr));

    if (match(TokenKind::LPAREN)) {
      expr = parse_call(ident, std::move(args));
    }
    else {
      expr = make_call_expression(std::move(args), ident);
    }
  }

  return expr;
}

std::unique_ptr<Expression> Parser::parse_primary()
{
  Token token = next();

  if (token.kind == TokenKind::IDENT) {
    if (match(TokenKind::LPAREN)) {
      return parse_call(token);
    }

    return parse_identifier(token);
  }

  if (token.kind == TokenKind::NUMBER) {
    return std::make_unique<NumberExpression>(parse_number(token.value), token);
  }

  if (token.kind == TokenKind::LPAREN) {
    auto expr = std::make_unique<GroupExpression>(parse_expression(), token);
    expect(TokenKind::RPAREN, "expected closing paren");
    return expr;
  }

  if (token.kind == TokenKind::END) {
    throw ParserError(token, "unexpected end of expression");
  }
  else {
    throw ParserError(token, "unexpected token");
  }
}

std::unique_ptr<Expression> Parser::parse_call(Token token,
                                               Vector<std::unique_ptr<Expression>> args)
{
  if (!check(TokenKind::RPAREN)) {
    do {
      args.append(parse_expression());
    } while (match(TokenKind::COMMA));
  }

  expect(TokenKind::RPAREN, "expected closing paren");
  return make_call_expression(std::move(args), token);
}

std::unique_ptr<Expression> Parser::parse_identifier(Token token)
{
  if (token.value == "pi") {
    return std::make_unique<NumberExpression>(M_PI, token);
  }
  else if (token.value == "tau") {
    return std::make_unique<NumberExpression>(M_PI * 2, token);
  }
  else if (token.value == "e") {
    return std::make_unique<NumberExpression>(M_E, token);
  }

  return std::make_unique<VariableExpression>(token);
}

std::unique_ptr<Expression> Parser::make_call_expression(Vector<std::unique_ptr<Expression>> args,
                                                         Token token)
{
  auto def = lookup_function(token.value);

  if (!def.has_value()) {
    throw ParserError(token, "invalid function");
  }

  if (args.size() != def->arg_count) {
    throw ParserError(token, "incorrect number of arguments");
  }

  return std::make_unique<CallExpression>(def->name, std::move(args), token);
}

float Parser::parse_number(std::string_view text)
{
  double d;
  auto result = std::from_chars(text.data(), text.data() + text.size(), d);

  if (result.ec != std::errc()) {
    // The number has already been parsed and is valid.
    BLI_assert_unreachable();
  }

  return d;
}

[[maybe_unused]] Token Parser::expect(TokenKind kind, const char *message)
{
  if (peek().kind != kind) {
    throw ParserError(peek(), message);
  }

  return next();
}

template<size_t N> bool Parser::match(const TokenKind (&kind)[N], Token &r_token)
{
  for (size_t i = 0; i < N; i++) {
    if (peek().kind == kind[i]) {
      r_token = next();
      return true;
    }
  }

  return false;
}

bool Parser::match(const TokenKind kind)
{
  if (peek().kind == kind) {
    next();
    return true;
  }

  return false;
}

bool Parser::check(const TokenKind kind)
{
  if (peek().kind == kind) {
    return true;
  }

  return false;
}

Token Parser::next()
{
  Token token = peek();
  peeked = false;
  return token;
}

Token Parser::peek()
{
  if (peeked) {
    return peeked_token;
  }

  peeked_token = lexer.next_token();
  peeked = true;
  return peeked_token;
}

void parse_var_names(std::string_view vars, std::function<void(std::string_view)> cb)
{
  size_t start = vars.size();

  for (size_t i = 0; i < vars.size(); i++) {
    char c = vars[i];

    if (start == vars.size() && (c == '_' || isalpha(c))) {
      start = i;
    }
    else if (start != vars.size() && (c != '_' && !isalnum(c))) {
      cb(vars.substr(start, i - start));
      start = vars.size();
    }

    if (start != vars.size() && i == vars.size() - 1) {
      cb(vars.substr(start, i + 1 - start));
    }
  }
}

};  // namespace blender::nodes::node_geo_math_expression_cc