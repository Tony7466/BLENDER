#pragma once

#include <charconv>
#include <memory>
#include <string>
#include <cstdio>

#include "lexer.hh"
#include "operation.hh"
#include "evaluation_context.hh"

struct EvaluationError {
  class Expression *expression;
  const char *message;
};

class Expression {
protected:
  Token token;

public:
  Expression(Token token) : token(token) {}

  const Token &get_token() const {
    return token;
  }

  virtual ValueKind evaluate(EvaluationContext &ctx) = 0;

  static ValueKind pow(EvaluationContext &ctx, Expression *x, Expression *y);
  static ValueKind lerp(EvaluationContext &ctx, Expression *a, Expression *b, Expression *t);
  static ValueKind negate(EvaluationContext &ctx, Expression *x);
  static ValueKind add(EvaluationContext &ctx, Expression *left, Expression *right);
  static ValueKind sub(EvaluationContext &ctx, Expression *left, Expression *right);
  static ValueKind mul(EvaluationContext &ctx, Expression *left, Expression *right);
  static ValueKind div(EvaluationContext &ctx, Expression *left, Expression *right);
};

class NumberExpression : public Expression {
public:
  NumberExpression(Token token) : Expression(token) {}

  ValueKind evaluate(EvaluationContext &ctx) override {
    double d;
    auto result = std::from_chars(token.value.data(), token.value.data() + token.value.size(), d);

    if(result.ec != std::errc()) {
      throw EvaluationError { this, "failed to parse number literal" };
    }

    ctx.push_op(Operation::float_op(d));

    return ValueKind::FLOAT;
  }
};

class GroupExpression : public Expression {
  std::unique_ptr<Expression> expr;
public:
  GroupExpression(std::unique_ptr<Expression> expr, Token token) : Expression(token), expr(std::move(expr)) {}

  ValueKind evaluate(EvaluationContext &ctx) override {
    return expr->evaluate(ctx);
  }
};

class VariableExpression : public Expression {
public:
  VariableExpression(Token token) : Expression(token) {}

  ValueKind evaluate(EvaluationContext &ctx) override {
    ValueKind value;
    
    try {
      value = ctx.get_variable(token.value);
    } catch(const char *err) {
      throw EvaluationError { this, err };
    }

    ctx.push_op(Operation::variable_op(token.value));

    return value;
  }
};

class CallExpression : public Expression {
public:
  enum class FunctionName {
    POW,
    LERP,
    VEC,
    X, Y, Z,
    LEN
  };

  struct FunctionDef {
    FunctionName name;
    size_t args_size;
  };

private:
  const std::vector<std::unique_ptr<Expression>> args;

public:
  CallExpression(std::vector<std::unique_ptr<Expression>> args, Token token) : Expression(token), args(std::move(args)) {}

  ValueKind evaluate(EvaluationContext &ctx) override {
    try {
      if(token.value == "pow") {
        if(args.size() != 2) { throw "incorrect number of arguments"; }
        return pow(ctx, args[0].get(), args[1].get());
      }

      if(token.value == "lerp") {
        if(args.size() != 3) { throw "incorrect number of arguments"; }
        return lerp(ctx, args[0].get(), args[1].get(), args[2].get());
      }


      /*case FunctionName::LERP:
        return Value::lerp(evaluated_args[0].get(), evaluated_args[1].get(), evaluated_args[2].get());
      case FunctionName::VEC:
        return Value::vec(evaluated_args[0].get(), evaluated_args[1].get(), evaluated_args[2].get());
      case FunctionName::X:
        return Value::x(evaluated_args[0].get());
      case FunctionName::Y:
        return Value::y(evaluated_args[0].get());
      case FunctionName::Z:
        return Value::z(evaluated_args[0].get());
      case FunctionName::LEN:
        return Value::len(evaluated_args[0].get());*/

      throw "invalid function";
    } catch (const char *err) {
      throw EvaluationError{ this, err };
    }
  }
};

class UnaryExpression : public Expression {
  std::unique_ptr<Expression> expr;

public:
  UnaryExpression(std::unique_ptr<Expression> expr, Token token) : Expression(token), expr(std::move(expr)) {}

  ValueKind evaluate(EvaluationContext &ctx) override {
    try {
      switch (token.kind) {
        case TokenKind::MINUS:
          return negate(ctx, expr.get());
        default:
          throw EvaluationError { this, "invalid unary operator" };
      }
    } catch (const char *err) {
      throw EvaluationError{ this, err };
    }
  }
};

class BinaryExpression : public Expression {
  std::unique_ptr<Expression> left;
  std::unique_ptr<Expression> right;
public:
  BinaryExpression(std::unique_ptr<Expression> left, std::unique_ptr<Expression> right, Token token) : Expression(token), left(std::move(left)), right(std::move(right)) {}

  ValueKind evaluate(EvaluationContext &ctx) override {
    try {
      switch (token.kind) {
        case TokenKind::PLUS:
          return add(ctx, left.get(), right.get());
        case TokenKind::MINUS:
          return sub(ctx, left.get(), right.get());
        case TokenKind::MUL:
          return mul(ctx, left.get(), right.get());
        case TokenKind::DIV:
          return div(ctx, left.get(), right.get());
        default:
          throw EvaluationError { this, "invalid binary operator" };
      }
    } catch (const char *err) {
      throw EvaluationError{ this, err };
    }
  }
};
