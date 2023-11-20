#pragma once

#include <charconv>
#include <memory>
#include <string>

#include "BLI_vector.hh"
#include "FN_field.hh"
#include "NOD_math_functions.hh"

#include "evaluation_context.hh"
#include "lexer.hh"

namespace blender::nodes::node_geo_math_expression_cc {

struct EvaluationError {
  class Expression *expression;
  const char *message;
};

class Expression {
 protected:
  Token token;

  static int _type_id()
  {
    static int type_id = 0;
    return type_id++;
  }

  template<typename T> static int _type_id()
  {
    static int type_id = _type_id();
    return type_id;
  }

  virtual int type_id() const = 0;

 public:
  Expression(Token token) : token(token) {}
  virtual ~Expression() = default;

  const Token &get_token() const
  {
    return token;
  }

  virtual fn::GField compile(EvaluationContext &ctx) = 0;

  template<typename T> T *as()
  {
    return type_id() == _type_id<T>() ? static_cast<T *>(this) : nullptr;
  }

 protected:
  static fn::GField constant(float f)
  {
    auto c = std::make_shared<mf::CustomMF_Constant<float>>(f);
    return fn::Field<float>(fn::FieldOperation::Create(std::move(c)));
  }

  static fn::GField constant(float3 f3)
  {
    auto c = std::make_shared<mf::CustomMF_Constant<float3>>(f3);
    return fn::Field<float3>(fn::FieldOperation::Create(std::move(c)));
  }

  static fn::GField fl_fl_to_fl(fn::GField a, fn::GField b, NodeMathOperation op)
  {
    const mf::MultiFunction *fn = nullptr;

    try_dispatch_float_math_fl_fl_to_fl(
        op,
        [&fn](auto devi_fn, auto function, const blender::nodes::FloatMathOperationInfo &info) {
          static auto _fn = mf::build::SI2_SO<float, float, float>(
              info.title_case_name.c_str(), function, devi_fn);
          fn = &_fn;
        });

    BLI_assert(fn != nullptr);

    return fn::Field<float>(fn::FieldOperation::Create(*fn, {std::move(a), std::move(b)}));
  }

  static fn::GField fl3_fl3_to_fl3(fn::GField a, fn::GField b, NodeVectorMathOperation op)
  {
    const mf::MultiFunction *fn = nullptr;

    try_dispatch_float_math_fl3_fl3_to_fl3(
        op,
        [&fn](auto devi_fn, auto function, const blender::nodes::FloatMathOperationInfo &info) {
          static auto _fn = mf::build::SI2_SO<float3, float3, float3>(
              info.title_case_name.c_str(), function, devi_fn);
          fn = &_fn;
        });

    BLI_assert(fn != nullptr);

    return fn::Field<float3>(fn::FieldOperation::Create(*fn, {std::move(a), std::move(b)}));
  }

  static fn::GField fl3_fl_to_fl3(fn::GField a, fn::GField b, NodeVectorMathOperation op)
  {
    const mf::MultiFunction *fn = nullptr;

    try_dispatch_float_math_fl3_fl_to_fl3(
        op,
        [&fn](auto devi_fn, auto function, const blender::nodes::FloatMathOperationInfo &info) {
          static auto _fn = mf::build::SI2_SO<float3, float, float3>(
              info.title_case_name.c_str(), function, devi_fn);
          fn = &_fn;
        });

    BLI_assert(fn != nullptr);

    return fn::Field<float3>(fn::FieldOperation::Create(*fn, {std::move(a), std::move(b)}));
  }

  static fn::GField fl3_to_fl(fn::GField a, NodeVectorMathOperation op)
  {
    const mf::MultiFunction *fn = nullptr;

    try_dispatch_float_math_fl3_to_fl(
        op,
        [&fn](auto devi_fn, auto function, const blender::nodes::FloatMathOperationInfo &info) {
          static auto _fn = mf::build::SI1_SO<float3, float>(
              info.title_case_name.c_str(), function, devi_fn);
          fn = &_fn;
        });

    BLI_assert(fn != nullptr);

    return fn::Field<float>(fn::FieldOperation::Create(*fn, {std::move(a)}));
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
 protected:
  int type_id() const override
  {
    return _type_id<NumberExpression>();
  }

 public:
  NumberExpression(Token token) : Expression(token) {}

  float value()
  {
    double d;
    auto result = std::from_chars(token.value.data(), token.value.data() + token.value.size(), d);

    if (result.ec != std::errc()) {
      throw EvaluationError{this, "failed to parse number literal"};
    }

    return d;
  }

  fn::GField compile(EvaluationContext & /*ctx*/) override
  {
    return constant(value());
  }
};

class GroupExpression : public Expression {
  std::unique_ptr<Expression> expr;

 protected:
  int type_id() const override
  {
    return _type_id<GroupExpression>();
  }

 public:
  GroupExpression(std::unique_ptr<Expression> expr, Token token)
      : Expression(token), expr(std::move(expr))
  {
  }

  fn::GField compile(EvaluationContext &ctx) override
  {
    return expr->compile(ctx);
  }
};

class VariableExpression : public Expression {
 protected:
  int type_id() const override
  {
    return _type_id<VariableExpression>();
  }

 public:
  VariableExpression(Token token) : Expression(token) {}

  fn::GField compile(EvaluationContext &ctx) override
  {
    fn::GField value;

    try {
      value = ctx.get_variable(token.value);
    }
    catch (const char *err) {
      throw EvaluationError{this, err};
    }

    return value;
  }
};

class CallExpression : public Expression {
  const Vector<std::unique_ptr<Expression>> args;

 protected:
  int type_id() const override
  {
    return _type_id<CallExpression>();
  }

 public:
  CallExpression(Vector<std::unique_ptr<Expression>> args, Token token)
      : Expression(token), args(std::move(args))
  {
  }

  fn::GField compile(EvaluationContext &ctx) override
  {
    try {
      if (token.value == "pow") {
        if (args.size() != 2) {
          throw "incorrect number of arguments";
        }
        return pow(ctx, args[0].get(), args[1].get());
      }

      if (token.value == "lerp") {
        if (args.size() != 3) {
          throw "incorrect number of arguments";
        }
        return lerp(ctx, args[0].get(), args[1].get(), args[2].get());
      }

      if (token.value == "vec") {
        if (args.size() != 3) {
          throw "incorrect number of arguments";
        }

        auto x = args[0]->as<NumberExpression>();
        auto y = args[1]->as<NumberExpression>();
        auto z = args[2]->as<NumberExpression>();

        if (x && y && z) {
          // optimize to constant
          return constant({x->value(), y->value(), z->value()});
        }

        return vec(ctx, args[0].get(), args[1].get(), args[2].get());
      }

      if (token.value == "len") {
        if (args.size() != 1) {
          throw "incorrect number of arguments";
        }
        return len(ctx, args[0].get());
      }

      if (token.value == "x") {
        if (args.size() != 1) {
          throw "incorrect number of arguments";
        }
        return x(ctx, args[0].get());
      }

      if (token.value == "y") {
        if (args.size() != 1) {
          throw "incorrect number of arguments";
        }
        return y(ctx, args[0].get());
      }

      if (token.value == "z") {
        if (args.size() != 1) {
          throw "incorrect number of arguments";
        }
        return z(ctx, args[0].get());
      }

      throw "invalid function";
    }
    catch (const char *err) {
      throw EvaluationError{this, err};
    }
  }
};

class UnaryExpression : public Expression {
  std::unique_ptr<Expression> expr;

 protected:
  int type_id() const override
  {
    return _type_id<UnaryExpression>();
  }

 public:
  UnaryExpression(std::unique_ptr<Expression> expr, Token token)
      : Expression(token), expr(std::move(expr))
  {
  }

  fn::GField compile(EvaluationContext &ctx) override
  {
    if (token.kind != TokenKind::MINUS) {
      throw EvaluationError{this, "invalid unary operator"};
    }

    try {
      auto number_expr = expr->as<NumberExpression>();

      if (number_expr) {
        // optimize to constant
        return constant(-number_expr->value());
      }

      return negate(ctx, expr.get());
    }
    catch (const char *err) {
      throw EvaluationError{this, err};
    }
  }
};

class BinaryExpression : public Expression {
  std::unique_ptr<Expression> left;
  std::unique_ptr<Expression> right;

 protected:
  int type_id() const override
  {
    return _type_id<BinaryExpression>();
  }

 public:
  BinaryExpression(std::unique_ptr<Expression> left,
                   std::unique_ptr<Expression> right,
                   Token token)
      : Expression(token), left(std::move(left)), right(std::move(right))
  {
  }

  fn::GField compile(EvaluationContext &ctx) override
  {
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
          throw EvaluationError{this, "invalid binary operator"};
      }
    }
    catch (const char *err) {
      throw EvaluationError{this, err};
    }
  }
};

}  // namespace blender::nodes::node_geo_math_expression_cc