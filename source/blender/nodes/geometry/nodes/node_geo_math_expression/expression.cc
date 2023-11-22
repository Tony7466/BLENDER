#include "expression.hh"

namespace blender::nodes::node_geo_math_expression_cc {

template<typename... T, typename... Args> bool args_are(Args... args)
{
  return (args.cpp_type().template is<T>() && ...);
}

static fn::GField fl_to_fl(fn::GField a, NodeMathOperation op)
{
  const mf::MultiFunction *fn = nullptr;

  try_dispatch_float_math_fl_to_fl(
      op, [&fn](auto devi_fn, auto function, const blender::nodes::FloatMathOperationInfo &info) {
        static auto _fn = mf::build::SI1_SO<float, float>(
            info.title_case_name.c_str(), function, devi_fn);
        fn = &_fn;
      });

  BLI_assert(fn != nullptr);

  return fn::Field<float>(fn::FieldOperation::Create(*fn, {std::move(a)}));
}

static fn::GField fl_fl_to_fl(fn::GField a, fn::GField b, NodeMathOperation op)
{
  const mf::MultiFunction *fn = nullptr;

  try_dispatch_float_math_fl_fl_to_fl(
      op, [&fn](auto devi_fn, auto function, const blender::nodes::FloatMathOperationInfo &info) {
        static auto _fn = mf::build::SI2_SO<float, float, float>(
            info.title_case_name.c_str(), function, devi_fn);
        fn = &_fn;
      });

  BLI_assert(fn != nullptr);

  return fn::Field<float>(fn::FieldOperation::Create(*fn, {std::move(a), std::move(b)}));
}

static fn::GField fl_fl_fl_to_fl(fn::GField a, fn::GField b, fn::GField c, NodeMathOperation op)
{
  const mf::MultiFunction *fn = nullptr;

  try_dispatch_float_math_fl_fl_fl_to_fl(
      op, [&fn](auto devi_fn, auto function, const blender::nodes::FloatMathOperationInfo &info) {
        static auto _fn = mf::build::SI3_SO<float, float, float, float>(
            info.title_case_name.c_str(), function, devi_fn);
        fn = &_fn;
      });

  BLI_assert(fn != nullptr);

  return fn::Field<float>(
      fn::FieldOperation::Create(*fn, {std::move(a), std::move(b), std::move(c)}));
}

static fn::GField fl3_to_fl3(fn::GField a, NodeVectorMathOperation op)
{
  const mf::MultiFunction *fn = nullptr;

  try_dispatch_float_math_fl3_to_fl3(
      op, [&fn](auto devi_fn, auto function, const blender::nodes::FloatMathOperationInfo &info) {
        static auto _fn = mf::build::SI1_SO<float3, float3>(
            info.title_case_name.c_str(), function, devi_fn);
        fn = &_fn;
      });

  BLI_assert(fn != nullptr);

  return fn::Field<float3>(fn::FieldOperation::Create(*fn, {std::move(a)}));
}

static fn::GField fl3_fl3_to_fl3(fn::GField a, fn::GField b, NodeVectorMathOperation op)
{
  const mf::MultiFunction *fn = nullptr;

  try_dispatch_float_math_fl3_fl3_to_fl3(
      op, [&fn](auto devi_fn, auto function, const blender::nodes::FloatMathOperationInfo &info) {
        static auto _fn = mf::build::SI2_SO<float3, float3, float3>(
            info.title_case_name.c_str(), function, devi_fn);
        fn = &_fn;
      });

  BLI_assert(fn != nullptr);

  return fn::Field<float3>(fn::FieldOperation::Create(*fn, {std::move(a), std::move(b)}));
}

static fn::GField fl3_fl3_fl3_to_fl3(fn::GField a,
                                     fn::GField b,
                                     fn::GField c,
                                     NodeVectorMathOperation op)
{
  const mf::MultiFunction *fn = nullptr;

  try_dispatch_float_math_fl3_fl3_fl3_to_fl3(
      op, [&fn](auto devi_fn, auto function, const blender::nodes::FloatMathOperationInfo &info) {
        static auto _fn = mf::build::SI3_SO<float3, float3, float3, float3>(
            info.title_case_name.c_str(), function, devi_fn);
        fn = &_fn;
      });

  BLI_assert(fn != nullptr);

  return fn::Field<float3>(
      fn::FieldOperation::Create(*fn, {std::move(a), std::move(b), std::move(c)}));
}

static fn::GField fl3_fl3_fl_to_fl3(fn::GField a,
                                    fn::GField b,
                                    fn::GField c,
                                    NodeVectorMathOperation op)
{
  const mf::MultiFunction *fn = nullptr;

  try_dispatch_float_math_fl3_fl3_fl_to_fl3(
      op, [&fn](auto devi_fn, auto function, const blender::nodes::FloatMathOperationInfo &info) {
        static auto _fn = mf::build::SI3_SO<float3, float3, float, float3>(
            info.title_case_name.c_str(), function, devi_fn);
        fn = &_fn;
      });

  BLI_assert(fn != nullptr);

  return fn::Field<float3>(
      fn::FieldOperation::Create(*fn, {std::move(a), std::move(b), std::move(c)}));
}

static fn::GField fl3_fl3_to_fl(fn::GField a, fn::GField b, NodeVectorMathOperation op)
{
  const mf::MultiFunction *fn = nullptr;

  try_dispatch_float_math_fl3_fl3_to_fl(
      op, [&fn](auto devi_fn, auto function, const blender::nodes::FloatMathOperationInfo &info) {
        static auto _fn = mf::build::SI2_SO<float3, float3, float>(
            info.title_case_name.c_str(), function, devi_fn);
        fn = &_fn;
      });

  BLI_assert(fn != nullptr);

  return fn::Field<float>(fn::FieldOperation::Create(*fn, {std::move(a), std::move(b)}));
}

static fn::GField fl3_fl_to_fl3(fn::GField a, fn::GField b, NodeVectorMathOperation op)
{
  const mf::MultiFunction *fn = nullptr;

  try_dispatch_float_math_fl3_fl_to_fl3(
      op, [&fn](auto devi_fn, auto function, const blender::nodes::FloatMathOperationInfo &info) {
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
      op, [&fn](auto devi_fn, auto function, const blender::nodes::FloatMathOperationInfo &info) {
        static auto _fn = mf::build::SI1_SO<float3, float>(
            info.title_case_name.c_str(), function, devi_fn);
        fn = &_fn;
      });

  BLI_assert(fn != nullptr);

  return fn::Field<float>(fn::FieldOperation::Create(*fn, {std::move(a)}));
}

fn::GField CallExpression::compile(EvaluationContext &ctx)
{
  // At this point the number of arguments has already been validated by the parser.

  // Custom functions.
  switch (name) {
    case FunctionName::LERP:
      return lerp(ctx, args[0].get(), args[1].get(), args[2].get());
    case FunctionName::VEC:
      return vec(ctx, args[0].get(), args[1].get(), args[2].get());
    case FunctionName::X:
      return x(ctx, args[0].get());
    case FunctionName::Y:
      return y(ctx, args[0].get());
    case FunctionName::Z:
      return z(ctx, args[0].get());
    default:
      break;
  }

  // The rest is for the built-in math functions.
  Vector<fn::GField> _args;

  for (auto &arg : args) {
    _args.append(arg->compile(ctx));
  }

  auto error = EvaluationError{this, "invalid arguments"};

  // This code was generated with a JSON description of the existing functions.
  if (_args.size() == 1) {
    if (args_are<float>(_args[0])) {
      switch (name) {
        case FunctionName::EXPONENT:
          return fl_to_fl(_args[0], NODE_MATH_EXPONENT);
        case FunctionName::SQRT:
          return fl_to_fl(_args[0], NODE_MATH_SQRT);
        case FunctionName::INV_SQRT:
          return fl_to_fl(_args[0], NODE_MATH_INV_SQRT);
        case FunctionName::ABSOLUTE:
          return fl_to_fl(_args[0], NODE_MATH_ABSOLUTE);
        case FunctionName::RADIANS:
          return fl_to_fl(_args[0], NODE_MATH_RADIANS);
        case FunctionName::DEGREES:
          return fl_to_fl(_args[0], NODE_MATH_DEGREES);
        case FunctionName::SIGN:
          return fl_to_fl(_args[0], NODE_MATH_SIGN);
        case FunctionName::ROUND:
          return fl_to_fl(_args[0], NODE_MATH_ROUND);
        case FunctionName::FLOOR:
          return fl_to_fl(_args[0], NODE_MATH_FLOOR);
        case FunctionName::CEIL:
          return fl_to_fl(_args[0], NODE_MATH_CEIL);
        case FunctionName::FRACTION:
          return fl_to_fl(_args[0], NODE_MATH_FRACTION);
        case FunctionName::TRUNC:
          return fl_to_fl(_args[0], NODE_MATH_TRUNC);
        case FunctionName::SINE:
          return fl_to_fl(_args[0], NODE_MATH_SINE);
        case FunctionName::COSINE:
          return fl_to_fl(_args[0], NODE_MATH_COSINE);
        case FunctionName::TANGENT:
          return fl_to_fl(_args[0], NODE_MATH_TANGENT);
        case FunctionName::SINH:
          return fl_to_fl(_args[0], NODE_MATH_SINH);
        case FunctionName::COSH:
          return fl_to_fl(_args[0], NODE_MATH_COSH);
        case FunctionName::TANH:
          return fl_to_fl(_args[0], NODE_MATH_TANH);
        case FunctionName::ARCSINE:
          return fl_to_fl(_args[0], NODE_MATH_ARCSINE);
        case FunctionName::ARCCOSINE:
          return fl_to_fl(_args[0], NODE_MATH_ARCCOSINE);
        case FunctionName::ARCTANGENT:
          return fl_to_fl(_args[0], NODE_MATH_ARCTANGENT);
        default:
          throw error;
      }
    }
    if (args_are<float3>(_args[0])) {
      switch (name) {
        case FunctionName::ABSOLUTE:
          return fl3_to_fl3(_args[0], NODE_VECTOR_MATH_ABSOLUTE);
        case FunctionName::FLOOR:
          return fl3_to_fl3(_args[0], NODE_VECTOR_MATH_FLOOR);
        case FunctionName::CEIL:
          return fl3_to_fl3(_args[0], NODE_VECTOR_MATH_CEIL);
        case FunctionName::FRACTION:
          return fl3_to_fl3(_args[0], NODE_VECTOR_MATH_FRACTION);
        case FunctionName::SINE:
          return fl3_to_fl3(_args[0], NODE_VECTOR_MATH_SINE);
        case FunctionName::COSINE:
          return fl3_to_fl3(_args[0], NODE_VECTOR_MATH_COSINE);
        case FunctionName::TANGENT:
          return fl3_to_fl3(_args[0], NODE_VECTOR_MATH_TANGENT);
        case FunctionName::LENGTH:
          return fl3_to_fl(_args[0], NODE_VECTOR_MATH_LENGTH);
        case FunctionName::NORMALIZE:
          return fl3_to_fl3(_args[0], NODE_VECTOR_MATH_NORMALIZE);
        default:
          throw error;
      }
    }
  }
  if (_args.size() == 2) {
    if (args_are<float, float>(_args[0], _args[1])) {
      switch (name) {
        case FunctionName::ADD:
          return fl_fl_to_fl(_args[0], _args[1], NODE_MATH_ADD);
        case FunctionName::SUBTRACT:
          return fl_fl_to_fl(_args[0], _args[1], NODE_MATH_SUBTRACT);
        case FunctionName::MULTIPLY:
          return fl_fl_to_fl(_args[0], _args[1], NODE_MATH_MULTIPLY);
        case FunctionName::DIVIDE:
          return fl_fl_to_fl(_args[0], _args[1], NODE_MATH_DIVIDE);
        case FunctionName::POWER:
          return fl_fl_to_fl(_args[0], _args[1], NODE_MATH_POWER);
        case FunctionName::LOGARITHM:
          return fl_fl_to_fl(_args[0], _args[1], NODE_MATH_LOGARITHM);
        case FunctionName::MINIMUM:
          return fl_fl_to_fl(_args[0], _args[1], NODE_MATH_MINIMUM);
        case FunctionName::MAXIMUM:
          return fl_fl_to_fl(_args[0], _args[1], NODE_MATH_MAXIMUM);
        case FunctionName::LESS_THAN:
          return fl_fl_to_fl(_args[0], _args[1], NODE_MATH_LESS_THAN);
        case FunctionName::GREATER_THAN:
          return fl_fl_to_fl(_args[0], _args[1], NODE_MATH_GREATER_THAN);
        case FunctionName::MODULO:
          return fl_fl_to_fl(_args[0], _args[1], NODE_MATH_MODULO);
        case FunctionName::FLOORED_MODULO:
          return fl_fl_to_fl(_args[0], _args[1], NODE_MATH_FLOORED_MODULO);
        case FunctionName::SNAP:
          return fl_fl_to_fl(_args[0], _args[1], NODE_MATH_SNAP);
        case FunctionName::ARCTAN2:
          return fl_fl_to_fl(_args[0], _args[1], NODE_MATH_ARCTAN2);
        case FunctionName::PINGPONG:
          return fl_fl_to_fl(_args[0], _args[1], NODE_MATH_PINGPONG);
        default:
          throw error;
      }
    }
    if (args_are<float3, float3>(_args[0], _args[1])) {
      switch (name) {
        case FunctionName::ADD:
          return fl3_fl3_to_fl3(_args[0], _args[1], NODE_VECTOR_MATH_ADD);
        case FunctionName::SUBTRACT:
          return fl3_fl3_to_fl3(_args[0], _args[1], NODE_VECTOR_MATH_SUBTRACT);
        case FunctionName::MULTIPLY:
          return fl3_fl3_to_fl3(_args[0], _args[1], NODE_VECTOR_MATH_MULTIPLY);
        case FunctionName::DIVIDE:
          return fl3_fl3_to_fl3(_args[0], _args[1], NODE_VECTOR_MATH_DIVIDE);
        case FunctionName::MINIMUM:
          return fl3_fl3_to_fl3(_args[0], _args[1], NODE_VECTOR_MATH_MINIMUM);
        case FunctionName::MAXIMUM:
          return fl3_fl3_to_fl3(_args[0], _args[1], NODE_VECTOR_MATH_MAXIMUM);
        case FunctionName::MODULO:
          return fl3_fl3_to_fl3(_args[0], _args[1], NODE_VECTOR_MATH_MODULO);
        case FunctionName::SNAP:
          return fl3_fl3_to_fl3(_args[0], _args[1], NODE_VECTOR_MATH_SNAP);
        case FunctionName::CROSS_PRODUCT:
          return fl3_fl3_to_fl3(_args[0], _args[1], NODE_VECTOR_MATH_CROSS_PRODUCT);
        case FunctionName::PROJECT:
          return fl3_fl3_to_fl3(_args[0], _args[1], NODE_VECTOR_MATH_PROJECT);
        case FunctionName::REFLECT:
          return fl3_fl3_to_fl3(_args[0], _args[1], NODE_VECTOR_MATH_REFLECT);
        case FunctionName::DOT_PRODUCT:
          return fl3_fl3_to_fl(_args[0], _args[1], NODE_VECTOR_MATH_DOT_PRODUCT);
        case FunctionName::DISTANCE:
          return fl3_fl3_to_fl(_args[0], _args[1], NODE_VECTOR_MATH_DISTANCE);
        default:
          throw error;
      }
    }
    if (args_are<float3, float>(_args[0], _args[1])) {
      switch (name) {
        case FunctionName::SCALE:
          return fl3_fl_to_fl3(_args[0], _args[1], NODE_VECTOR_MATH_SCALE);
        default:
          throw error;
      }
    }
  }
  if (_args.size() == 3) {
    if (args_are<float, float, float>(_args[0], _args[1], _args[2])) {
      switch (name) {
        case FunctionName::MULTIPLY_ADD:
          return fl_fl_fl_to_fl(_args[0], _args[1], _args[2], NODE_MATH_MULTIPLY_ADD);
        case FunctionName::COMPARE:
          return fl_fl_fl_to_fl(_args[0], _args[1], _args[2], NODE_MATH_COMPARE);
        case FunctionName::SMOOTH_MIN:
          return fl_fl_fl_to_fl(_args[0], _args[1], _args[2], NODE_MATH_SMOOTH_MIN);
        case FunctionName::SMOOTH_MAX:
          return fl_fl_fl_to_fl(_args[0], _args[1], _args[2], NODE_MATH_SMOOTH_MAX);
        case FunctionName::WRAP:
          return fl_fl_fl_to_fl(_args[0], _args[1], _args[2], NODE_MATH_WRAP);
        default:
          throw error;
      }
    }
    if (args_are<float3, float3, float3>(_args[0], _args[1], _args[2])) {
      switch (name) {
        case FunctionName::MULTIPLY_ADD:
          return fl3_fl3_fl3_to_fl3(_args[0], _args[1], _args[2], NODE_VECTOR_MATH_MULTIPLY_ADD);
        case FunctionName::WRAP:
          return fl3_fl3_fl3_to_fl3(_args[0], _args[1], _args[2], NODE_VECTOR_MATH_WRAP);
        case FunctionName::FACEFORWARD:
          return fl3_fl3_fl3_to_fl3(_args[0], _args[1], _args[2], NODE_VECTOR_MATH_FACEFORWARD);
        default:
          throw error;
      }
    }
    if (args_are<float3, float3, float>(_args[0], _args[1], _args[2])) {
      switch (name) {
        case FunctionName::REFRACT:
          return fl3_fl3_fl_to_fl3(_args[0], _args[1], _args[2], NODE_VECTOR_MATH_REFRACT);
        default:
          throw error;
      }
    }
  }

  throw error;
}

fn::GField Expression::add(EvaluationContext &ctx, Expression *left, Expression *right)
{
  fn::GField _left = left->compile(ctx);
  fn::GField _right = right->compile(ctx);

  if (args_are<float, float>(_left, _right)) {
    return fl_fl_to_fl(_left, _right, NODE_MATH_ADD);
  }

  if (args_are<float3, float3>(_left, _right)) {
    return fl3_fl3_to_fl3(_left, _right, NODE_VECTOR_MATH_ADD);
  }

  throw "invalid operands";
}

fn::GField Expression::sub(EvaluationContext &ctx, Expression *left, Expression *right)
{
  fn::GField _left = left->compile(ctx);
  fn::GField _right = right->compile(ctx);

  if (args_are<float, float>(_left, _right)) {
    return fl_fl_to_fl(_left, _right, NODE_MATH_SUBTRACT);
  }

  if (args_are<float3, float3>(_left, _right)) {
    return fl3_fl3_to_fl3(_left, _right, NODE_VECTOR_MATH_SUBTRACT);
  }

  throw "invalid operands";
}

fn::GField Expression::mul(EvaluationContext &ctx, Expression *left, Expression *right)
{
  fn::GField _left = left->compile(ctx);
  fn::GField _right = right->compile(ctx);

  if (args_are<float, float>(_left, _right)) {
    return fl_fl_to_fl(_left, _right, NODE_MATH_MULTIPLY);
  }

  if (args_are<float3, float>(_left, _right)) {
    return fl3_fl_to_fl3(_left, _right, NODE_VECTOR_MATH_SCALE);
  }

  if (args_are<float, float3>(_left, _right)) {
    return fl3_fl_to_fl3(_right, _left, NODE_VECTOR_MATH_SCALE);
  }

  throw "invalid operands";
}

fn::GField Expression::div(EvaluationContext &ctx, Expression *left, Expression *right)
{
  fn::GField _left = left->compile(ctx);
  fn::GField _right = right->compile(ctx);

  if (args_are<float, float>(_left, _right)) {
    return fl_fl_to_fl(_left, _right, NODE_MATH_DIVIDE);
  }

  if (args_are<float3, float>(_left, _right)) {
    auto one_div_right = fl_fl_to_fl(constant(1.0f), _right, NODE_MATH_DIVIDE);
    return fl3_fl_to_fl3(_left, one_div_right, NODE_VECTOR_MATH_SCALE);
  }

  throw "invalid operands";
}

fn::GField Expression::negate(EvaluationContext &ctx, Expression *x)
{
  fn::GField _x = x->compile(ctx);

  if (args_are<float>(_x)) {
    return fl_fl_to_fl(_x, constant(-1.0f), NODE_MATH_MULTIPLY);
  }

  if (args_are<float3>(_x)) {
    return fl3_fl_to_fl3(_x, constant(-1.0f), NODE_VECTOR_MATH_SCALE);
  }

  throw "invalid operand";
}

fn::GField Expression::lerp(EvaluationContext &ctx, Expression *a, Expression *b, Expression *t)
{
  // (b - a) * t + a
  fn::GField _a = a->compile(ctx);
  fn::GField _b = b->compile(ctx);
  fn::GField _t = t->compile(ctx);

  if (args_are<float, float, float>(_a, _b, _t)) {
    auto b_sub_a = fl_fl_to_fl(_b, _a, NODE_MATH_SUBTRACT);
    auto mul_t = fl_fl_to_fl(b_sub_a, _t, NODE_MATH_MULTIPLY);
    return fl_fl_to_fl(mul_t, _a, NODE_MATH_ADD);
  }

  if (args_are<float3, float3, float>(_a, _b, _t)) {
    auto b_sub_a = fl3_fl3_to_fl3(_b, _a, NODE_VECTOR_MATH_SUBTRACT);
    auto mul_t = fl3_fl_to_fl3(b_sub_a, _t, NODE_VECTOR_MATH_SCALE);
    return fl3_fl3_to_fl3(mul_t, _a, NODE_VECTOR_MATH_ADD);
  }

  throw "invalid operands";
}

fn::GField Expression::vec(EvaluationContext &ctx, Expression *x, Expression *y, Expression *z)
{
  fn::GField _x = x->compile(ctx);
  fn::GField _y = y->compile(ctx);
  fn::GField _z = z->compile(ctx);

  if (args_are<float, float, float>(_x, _y, _z)) {
    static auto fn = mf::build::SI3_SO<float, float, float, float3>("vec",
                                                                    [](float x, float y, float z) {
                                                                      return float3{x, y, z};
                                                                    });

    return fn::Field<float3>(
        fn::FieldOperation::Create(fn, {std::move(_x), std::move(_y), std::move(_z)}));
  }

  throw "invalid operands";
}

fn::GField Expression::x(EvaluationContext &ctx, Expression *v)
{
  auto _v = v->compile(ctx);

  if (args_are<float3>(_v)) {
    static auto fn = mf::build::SI1_SO<float3, float>("x", [](float3 v) { return v.x; });

    return fn::Field<float>(fn::FieldOperation::Create(fn, {std::move(_v)}));
  }

  throw "invalid operands";
}

fn::GField Expression::y(EvaluationContext &ctx, Expression *v)
{
  auto _v = v->compile(ctx);

  if (args_are<float3>(_v)) {
    static auto fn = mf::build::SI1_SO<float3, float>("y", [](float3 v) { return v.y; });

    return fn::Field<float>(fn::FieldOperation::Create(fn, {std::move(_v)}));
  }

  throw "invalid operands";
}

fn::GField Expression::z(EvaluationContext &ctx, Expression *v)
{
  auto _v = v->compile(ctx);

  if (args_are<float3>(_v)) {
    static auto fn = mf::build::SI1_SO<float3, float>("z", [](float3 v) { return v.z; });

    return fn::Field<float>(fn::FieldOperation::Create(fn, {std::move(_v)}));
  }

  throw "invalid operands";
}

}  // namespace blender::nodes::node_geo_math_expression_cc