#include "expression.hh"

namespace blender::nodes::node_geo_math_expression_cc {

// TODO: variadic template?
template<typename T>
inline static bool args_are(fn::GField a) {
  return a.cpp_type().is<T>();
}

template<typename T, typename U>
inline static bool args_are(fn::GField a, fn::GField b) {
  return a.cpp_type().is<T>() && b.cpp_type().is<U>();
}

template<typename T, typename U, typename V>
inline static bool args_are(fn::GField a, fn::GField b, fn::GField c) {
  return a.cpp_type().is<T>() && b.cpp_type().is<U>() && c.cpp_type().is<V>();
}

fn::GField Expression::add(EvaluationContext &ctx, Expression *left, Expression *right) {
  fn::GField _left = left->compile(ctx);
  fn::GField _right = right->compile(ctx);

  if(args_are<float, float>(_left, _right)) {
    return fl_fl_to_fl(_left, _right, NODE_MATH_ADD);
  }

  if(args_are<float3, float3>(_left, _right)) {
    return fl3_fl3_to_fl3(_left, _right, NODE_VECTOR_MATH_ADD);
  }

  throw "invalid operands";
}

fn::GField Expression::sub(EvaluationContext &ctx, Expression *left, Expression *right) {
  fn::GField _left = left->compile(ctx);
  fn::GField _right = right->compile(ctx);

  if(args_are<float, float>(_left, _right)) {
    return fl_fl_to_fl(_left, _right, NODE_MATH_SUBTRACT);
  }

  if(args_are<float3, float3>(_left, _right)) {
    return fl3_fl3_to_fl3(_left, _right, NODE_VECTOR_MATH_SUBTRACT);
  }

  throw "invalid operands";
}

fn::GField Expression::mul(EvaluationContext &ctx, Expression *left, Expression *right) {
  fn::GField _left = left->compile(ctx);
  fn::GField _right = right->compile(ctx);

  if(args_are<float, float>(_left, _right)) {
    return fl_fl_to_fl(_left, _right, NODE_MATH_MULTIPLY);
  }

  if(args_are<float3, float>(_left, _right)) {
    return fl3_fl_to_fl3(_left, _right, NODE_VECTOR_MATH_SCALE);
  }

  if(args_are<float, float3>(_left, _right)) {
    return fl3_fl_to_fl3(_right, _left, NODE_VECTOR_MATH_SCALE);
  }

  throw "invalid operands";
}

fn::GField Expression::div(EvaluationContext &ctx, Expression *left, Expression *right) {
  fn::GField _left = left->compile(ctx);
  fn::GField _right = right->compile(ctx);

  if(args_are<float, float>(_left, _right)) {
    return fl_fl_to_fl(_left, _right, NODE_MATH_DIVIDE);
  }

  if(args_are<float3, float>(_left, _right)) {
    auto one_div_right = fl_fl_to_fl(constant(1.0f), _right, NODE_MATH_DIVIDE);
    return fl3_fl_to_fl3(_left, one_div_right, NODE_VECTOR_MATH_SCALE);
  }

  throw "invalid operands";
}

fn::GField Expression::negate(EvaluationContext &ctx, Expression *x) {
  fn::GField _x = x->compile(ctx);
  
  if(args_are<float>(_x)) {
    return fl_fl_to_fl(_x, constant(-1.0f), NODE_MATH_MULTIPLY);
  }

  if(args_are<float3>(_x)) {
    return fl3_fl_to_fl3(_x, constant(-1.0f), NODE_VECTOR_MATH_SCALE);
  }

  throw "invalid operand";
}

fn::GField Expression::pow(EvaluationContext &ctx, Expression *x, Expression *y) {
  fn::GField _x = x->compile(ctx);
  fn::GField _y = y->compile(ctx);

  if(args_are<float, float>(_x, _y)) {
    return fl_fl_to_fl(_x, _y, NODE_MATH_EXPONENT);
  }

  throw "invalid operands";
}

fn::GField Expression::lerp(EvaluationContext &ctx, Expression *a, Expression *b, Expression *t) {
  // (b - a) * t + a
  fn::GField _a = a->compile(ctx);
  fn::GField _b = b->compile(ctx);
  fn::GField _t = t->compile(ctx);

  if(args_are<float, float, float>(_a, _b, _t)) {
    auto b_sub_a = fl_fl_to_fl(_b, _a, NODE_MATH_SUBTRACT);
    auto mul_t = fl_fl_to_fl(b_sub_a, _t, NODE_MATH_MULTIPLY);
    return fl_fl_to_fl(mul_t, _a, NODE_MATH_ADD);
  }

  if(args_are<float3, float3, float>(_a, _b, _t)) {
    auto b_sub_a = fl3_fl3_to_fl3(_b, _a, NODE_VECTOR_MATH_SUBTRACT);
    auto mul_t = fl3_fl_to_fl3(b_sub_a, _t, NODE_VECTOR_MATH_SCALE);
    return fl3_fl3_to_fl3(mul_t, _a, NODE_VECTOR_MATH_ADD);
  }

  throw "invalid operands";
}

fn::GField Expression::vec(EvaluationContext &ctx, Expression *x, Expression *y, Expression *z) {
  fn::GField _x = x->compile(ctx);
  fn::GField _y = y->compile(ctx);
  fn::GField _z = z->compile(ctx);

  if(args_are<float, float, float>(_x, _y, _z)) {
    static auto fn = mf::build::SI3_SO<float, float, float, float3>("vec", [](float x, float y, float z) {
      return float3 {x, y, z};
    });

    return fn::Field<float3>(fn::FieldOperation::Create(fn, { std::move(_x), std::move(_y), std::move(_z) }));
  }

  throw "invalid operands";
}

fn::GField Expression::len(EvaluationContext &ctx, Expression *v) {
  auto _v = v->compile(ctx);

  if(args_are<float3>(_v)) {
    return fl3_to_fl(_v, NODE_VECTOR_MATH_LENGTH);
  }

  throw "invalid operands";
}

fn::GField Expression::x(EvaluationContext &ctx, Expression *v) {
  auto _v = v->compile(ctx);

  if(args_are<float3>(_v)) {
    static auto fn = mf::build::SI1_SO<float3, float>("x", [](float3 v) {
      return v.x;
    });

    return fn::Field<float>(fn::FieldOperation::Create(fn, { std::move(_v) }));
  }

  throw "invalid operands";
}

fn::GField Expression::y(EvaluationContext &ctx, Expression *v) {
  auto _v = v->compile(ctx);

  if(args_are<float3>(_v)) {
    static auto fn = mf::build::SI1_SO<float3, float>("y", [](float3 v) {
      return v.y;
    });

    return fn::Field<float>(fn::FieldOperation::Create(fn, { std::move(_v) }));
  }

  throw "invalid operands";
}

fn::GField Expression::z(EvaluationContext &ctx, Expression *v) {
  auto _v = v->compile(ctx);

  if(args_are<float3>(_v)) {
    static auto fn = mf::build::SI1_SO<float3, float>("z", [](float3 v) {
      return v.z;
    });

    return fn::Field<float>(fn::FieldOperation::Create(fn, { std::move(_v) }));
  }

  throw "invalid operands";
}

}