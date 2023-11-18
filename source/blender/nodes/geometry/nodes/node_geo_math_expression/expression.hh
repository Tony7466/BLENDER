#pragma once

#include <charconv>
#include <memory>
#include <string>
#include <cstdio>

#include "FN_field.hh"
#include "NOD_math_functions.hh"

#include "lexer.hh"
#include "evaluation_context.hh"

namespace blender::nodes::node_geo_math_expression_cc {

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

  virtual fn::GField compile(EvaluationContext &ctx) = 0;

  static fn::GField constant(float f) {
    auto c = std::make_shared<mf::CustomMF_Constant<float>>(f);
    return fn::Field<float>(fn::FieldOperation::Create(std::move(c)));
  }

  static fn::GField constant(float3 f3) {
    auto c = std::make_shared<mf::CustomMF_Constant<float3>>(f3);
    return fn::Field<float3>(fn::FieldOperation::Create(std::move(c)));
  }

  static fn::GField fl_fl_to_fl(fn::GField a, fn::GField b, NodeMathOperation op) {
    const mf::MultiFunction *fn = nullptr;

    try_dispatch_float_math_fl_fl_to_fl(op, [&fn](auto devi_fn, auto function, const blender::nodes::FloatMathOperationInfo &info) {
      static auto _fn = mf::build::SI2_SO<float, float, float>(info.title_case_name.c_str(), function, devi_fn);
      fn = &_fn;
    });

    BLI_assert(fn != nullptr);

    return fn::Field<float>(fn::FieldOperation::Create(*fn, { std::move(a), std::move(b) }));
  }

  static fn::GField fl3_fl3_to_fl3(fn::GField a, fn::GField b, NodeVectorMathOperation op) {
    const mf::MultiFunction *fn = nullptr;

    try_dispatch_float_math_fl3_fl3_to_fl3(op, [&fn](auto devi_fn, auto function, const blender::nodes::FloatMathOperationInfo &info) {
      static auto _fn = mf::build::SI2_SO<float3, float3, float3>(info.title_case_name.c_str(), function, devi_fn);
      fn = &_fn;
    });

    BLI_assert(fn != nullptr);

    return fn::Field<float3>(fn::FieldOperation::Create(*fn, { std::move(a), std::move(b) }));
  }

  static fn::GField fl3_fl_to_fl3(fn::GField a, fn::GField b, NodeVectorMathOperation op) {
    const mf::MultiFunction *fn = nullptr;

    try_dispatch_float_math_fl3_fl_to_fl3(op, [&fn](auto devi_fn, auto function, const blender::nodes::FloatMathOperationInfo &info) {
      static auto _fn = mf::build::SI2_SO<float3, float, float3>(info.title_case_name.c_str(), function, devi_fn);
      fn = &_fn;
    });

    BLI_assert(fn != nullptr);

    return fn::Field<float3>(fn::FieldOperation::Create(*fn, { std::move(a), std::move(b) }));
  }

  static fn::GField fl3_to_fl(fn::GField a, NodeVectorMathOperation op) {
    const mf::MultiFunction *fn = nullptr;

    try_dispatch_float_math_fl3_to_fl(op, [&fn](auto devi_fn, auto function, const blender::nodes::FloatMathOperationInfo &info) {
      static auto _fn = mf::build::SI1_SO<float3, float>(info.title_case_name.c_str(), function, devi_fn);
      fn = &_fn;
    });

    BLI_assert(fn != nullptr);

    return fn::Field<float3>(fn::FieldOperation::Create(*fn, { std::move(a) }));
  }

  static fn::GField pow(EvaluationContext &ctx, Expression *x, Expression *y);
  static fn::GField lerp(EvaluationContext &ctx, Expression *a, Expression *b, Expression *t);
  static fn::GField negate(EvaluationContext &ctx, Expression *x);
  static fn::GField add(EvaluationContext &ctx, Expression *left, Expression *right);
  static fn::GField sub(EvaluationContext &ctx, Expression *left, Expression *right);
  static fn::GField mul(EvaluationContext &ctx, Expression *left, Expression *right);
  static fn::GField div(EvaluationContext &ctx, Expression *left, Expression *right);
  static fn::GField vec(EvaluationContext &ctx, Expression *x, Expression *y, Expression *z);
  static fn::GField len(EvaluationContext &ctx, Expression *v);
  static fn::GField x(EvaluationContext &ctx, Expression *v);
  static fn::GField y(EvaluationContext &ctx, Expression *v);
  static fn::GField z(EvaluationContext &ctx, Expression *v);
};

class NumberExpression : public Expression {
public:
  NumberExpression(Token token) : Expression(token) {}

  fn::GField compile(EvaluationContext &/*ctx*/) override {
    double d;
    auto result = std::from_chars(token.value.data(), token.value.data() + token.value.size(), d);

    if(result.ec != std::errc()) {
      throw EvaluationError { this, "failed to parse number literal" };
    }

    return constant(d);
  }
};

class GroupExpression : public Expression {
  std::unique_ptr<Expression> expr;
public:
  GroupExpression(std::unique_ptr<Expression> expr, Token token) : Expression(token), expr(std::move(expr)) {}

  fn::GField compile(EvaluationContext &ctx) override {
    return expr->compile(ctx);
  }
};

class VariableExpression : public Expression {
public:
  VariableExpression(Token token) : Expression(token) {}

  fn::GField compile(EvaluationContext &ctx) override {
    fn::GField value;
    
    try {
      value = ctx.get_variable(token.value);
    } catch(const char *err) {
      throw EvaluationError { this, err };
    }

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

  fn::GField compile(EvaluationContext &ctx) override {
    try {
      if(token.value == "pow") {
        if(args.size() != 2) { throw "incorrect number of arguments"; }
        return pow(ctx, args[0].get(), args[1].get());
      }

      if(token.value == "lerp") {
        if(args.size() != 3) { throw "incorrect number of arguments"; }
        return lerp(ctx, args[0].get(), args[1].get(), args[2].get());
      }

      if(token.value == "vec") {
        if(args.size() != 3) { throw "incorrect number of arguments"; }
        return vec(ctx, args[0].get(), args[1].get(), args[2].get());
      }

      if(token.value == "len") {
        if(args.size() != 1) { throw "incorrect number of arguments"; }
        return len(ctx, args[0].get());
      }

      if(token.value == "x") {
        if(args.size() != 1) { throw "incorrect number of arguments"; }
        return x(ctx, args[0].get());
      }

      if(token.value == "y") {
        if(args.size() != 1) { throw "incorrect number of arguments"; }
        return y(ctx, args[0].get());
      }

      if(token.value == "z") {
        if(args.size() != 1) { throw "incorrect number of arguments"; }
        return z(ctx, args[0].get());
      }

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

  fn::GField compile(EvaluationContext &ctx) override {
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

  fn::GField compile(EvaluationContext &ctx) override {
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

}