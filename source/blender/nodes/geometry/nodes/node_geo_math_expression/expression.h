#pragma once

#include <memory>
#include <string>
#include <cstdio>

#include "lexer.h"
#include "value.h"
#include "evaluation_context.h"

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

  virtual std::unique_ptr<Expression> clone() const = 0;
  virtual std::unique_ptr<Value> evaluate(EvaluationContext &ctx) = 0;
};

class NumberExpression : public Expression {
public:
  NumberExpression(Token token) : Expression(token) {}

  std::unique_ptr<Expression> clone() const {
    return std::make_unique<NumberExpression>(*this);
  }

  std::unique_ptr<Value> evaluate(EvaluationContext &/*ctx*/) override {
    double d;
    auto result = std::from_chars(token.value.data(), token.value.data() + token.value.size(), d);
    return std::make_unique<ScalarValue>(d);
  }
};

class GroupExpression : public Expression {
  std::unique_ptr<Expression> expr;
public:
  GroupExpression(std::unique_ptr<Expression> expr, Token token) : Expression(token), expr(std::move(expr)) {}

  std::unique_ptr<Expression> clone() const {
    return std::make_unique<GroupExpression>(expr->clone(), token);
  }

  std::unique_ptr<Value> evaluate(EvaluationContext &ctx) override {
    return expr->evaluate(ctx);
  }
};

class VariableExpression : public Expression {
public:
  VariableExpression(Token token) : Expression(token) {}

  std::unique_ptr<Expression> clone() const {
    return std::make_unique<VariableExpression>(*this);
  }

  std::unique_ptr<Value> evaluate(EvaluationContext &ctx) override {
    return ctx.get_variable(token);
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
  struct FunctionDef def;

public:
  CallExpression(std::vector<std::unique_ptr<Expression>> args, FunctionDef def, Token token) : Expression(token), args(std::move(args)), def(def) {}

  std::unique_ptr<Expression> clone() const {
    std::vector<std::unique_ptr<Expression>> args_copy;

    for(auto &arg : args) {
      args_copy.emplace_back(arg->clone());
    }

    return std::make_unique<CallExpression>(std::move(args_copy), def, token);
  }

  std::unique_ptr<Value> evaluate(EvaluationContext &ctx) override {
    std::vector<std::unique_ptr<Value>> evaluated_args;

    for(auto &arg : args) {
      evaluated_args.emplace_back(arg->evaluate(ctx));
    }

    try {
      switch (def.name) {
        case FunctionName::POW:
          return evaluated_args[0]->pow(evaluated_args[1].get());
        case FunctionName::LERP:
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
          return Value::len(evaluated_args[0].get());
        default:
          throw EvaluationError { this, "invalid function" };
      }
    } catch (const char *err) {
      throw EvaluationError{ this, err };
    }
  }
};

class UnaryExpression : public Expression {
  std::unique_ptr<Expression> expr;

public:
  UnaryExpression(std::unique_ptr<Expression> expr, Token token) : Expression(token), expr(std::move(expr)) {}

  std::unique_ptr<Expression> clone() const {
    return std::make_unique<UnaryExpression>(expr->clone(), token);
  }

  std::unique_ptr<Value> evaluate(EvaluationContext &ctx) override {
    auto value = expr->evaluate(ctx);

    try {
      switch (token.kind) {
        case TokenKind::MINUS:
          return value->neg();
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

  std::unique_ptr<Expression> clone() const {
    return std::make_unique<BinaryExpression>(left->clone(), right->clone(), token);
  }

  std::unique_ptr<Value> evaluate(EvaluationContext &ctx) override {
    auto vleft = left->evaluate(ctx);
    auto vright = right->evaluate(ctx);

    try {
      switch (token.kind) {
        case TokenKind::PLUS:
          return vleft->add(vright.get());
        case TokenKind::MINUS:
          return vleft->sub(vright.get());
        case TokenKind::MUL:
          return vleft->mul(vright.get());
        case TokenKind::DIV:
          return vleft->div(vright.get());
        default:
          throw EvaluationError { this, "invalid binary operator" };
      }
    } catch (const char *err) {
      throw EvaluationError{ this, err };
    }
  }
};
