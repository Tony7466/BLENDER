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

  virtual std::unique_ptr<Value> evaluate(EvaluationContext &ctx) = 0;
};

class NumberExpression : public Expression {
public:
  NumberExpression(Token token) : Expression(token) {}

  std::unique_ptr<Value> evaluate(EvaluationContext &ctx) override {
    return ctx.get_number(token);
  }
};

class GroupExpression : public Expression {
  std::unique_ptr<Expression> expr;
public:
  GroupExpression(std::unique_ptr<Expression> expr, Token token) : expr(std::move(expr)), Expression(token) {}

  std::unique_ptr<Value> evaluate(EvaluationContext &ctx) override {
    return expr->evaluate(ctx);
  }
};

class VariableExpression : public Expression {
public:
  VariableExpression(Token token) : Expression(token) {}

  std::unique_ptr<Value> evaluate(EvaluationContext &ctx) override {
    return ctx.get_variable(token);
  }
};

class CallExpression : public Expression {
public:
  enum class FunctionName {
    POW,
    LERP
  };

  struct FunctionDef {
    FunctionName name;
    size_t args_size;
  };

private:
  const std::vector<std::unique_ptr<Expression>> args;
  struct FunctionDef def;

public:
  CallExpression(std::vector<std::unique_ptr<Expression>> args, FunctionDef def, Token token) : args(std::move(args)), def(def), Expression(token) {}

  std::unique_ptr<Value> evaluate(EvaluationContext &ctx) override {
    std::vector<std::unique_ptr<Value>> evaluated_args;

    for(auto &arg : args) {
      evaluated_args.emplace_back(arg->evaluate(ctx));
    }

    switch(def.name) {
      case FunctionName::POW:
        return evaluated_args[0]->pow(evaluated_args[1].get());
      case FunctionName::LERP:
        return Value::lerp(evaluated_args[0].get(), evaluated_args[1].get(), evaluated_args[2].get());
    }

    throw EvaluationError { this, "invalid function" };
  }
};

class UnaryExpression : public Expression {
  std::unique_ptr<Expression> expr;

public:
  UnaryExpression(std::unique_ptr<Expression> expr, Token token) : expr(std::move(expr)), Expression(token) {}

  std::unique_ptr<Value> evaluate(EvaluationContext &ctx) override {
    auto value = expr->evaluate(ctx);

    switch(token.kind) {
      case TokenKind::MINUS:
        return value->neg();
    }

    throw EvaluationError { this, "invalid unary operator" };
  }
};

class BinaryExpression : public Expression {
  std::unique_ptr<Expression> left;
  std::unique_ptr<Expression> right;
public:
  BinaryExpression(std::unique_ptr<Expression> left, std::unique_ptr<Expression> right, Token token) : left(std::move(left)), right(std::move(right)), Expression(token) {}

  std::unique_ptr<Value> evaluate(EvaluationContext &ctx) override {
    auto vleft = left->evaluate(ctx);
    auto vright = right->evaluate(ctx);

    switch(token.kind) {
      case TokenKind::PLUS:
        return vleft->add(vright.get());
      case TokenKind::MINUS:
        return vleft->sub(vright.get());
      case TokenKind::MUL:
        return vleft->mul(vright.get());
      case TokenKind::DIV:
        return vleft->div(vright.get());
    }

    throw EvaluationError { this, "invalid binary operator" };
  }
};
