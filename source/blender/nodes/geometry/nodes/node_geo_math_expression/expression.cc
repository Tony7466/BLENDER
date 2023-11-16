#include "expression.hh"

inline bool ops_are(ValueKind left, ValueKind right, ValueKind a) {
  return left == a && right == a;
}

inline bool ops_are(ValueKind left, ValueKind right, ValueKind a, ValueKind b) {
  return (left == a && right == b) || (left == b && right == a);
}

ValueKind Expression::add(EvaluationContext &ctx, Expression *left, Expression *right) {
  ValueKind _left = left->evaluate(ctx);
  ValueKind _right = right->evaluate(ctx);

  if(_left != _right) {
    throw "invalid operands";
  }

  switch(_left) {
      case ValueKind::FLOAT:
          ctx.push_op(Operation::math_op(Operation::OpKind::MATH_FL_FL_TO_FL, NODE_MATH_ADD));
          break;
      case ValueKind::VECTOR:
          ctx.push_op(Operation::vector_math_op(Operation::OpKind::VECTOR_MATH_FL3_FL3_TO_FL3, NODE_VECTOR_MATH_ADD));
          break;
      default:
          BLI_assert_unreachable();
  }

  return _left;
}

ValueKind Expression::sub(EvaluationContext &ctx, Expression *left, Expression *right) {
  ValueKind _left = left->evaluate(ctx);
  ValueKind _right = right->evaluate(ctx);

  if(_left != _right) {
    throw "invalid operands";
  }

  switch(_left) {
      case ValueKind::FLOAT:
          ctx.push_op(Operation::math_op(Operation::OpKind::MATH_FL_FL_TO_FL, NODE_MATH_SUBTRACT));
          break;
      case ValueKind::VECTOR:
          ctx.push_op(Operation::vector_math_op(Operation::OpKind::VECTOR_MATH_FL3_FL3_TO_FL3, NODE_VECTOR_MATH_SUBTRACT));
          break;
      default:
          BLI_assert_unreachable();
  }

    return _left;
}

ValueKind Expression::mul(EvaluationContext &ctx, Expression *left, Expression *right) {
  auto savepoint = ctx.savepoint();
  
  ValueKind _left = left->evaluate(ctx);
  ValueKind _right = right->evaluate(ctx);
  ValueKind result;

  if(ops_are(_left, _right, ValueKind::FLOAT)) {
    result = ValueKind::FLOAT;
  } else if(ops_are(_left, _right, ValueKind::FLOAT, ValueKind::VECTOR)) {
    result = ValueKind::VECTOR;
  } else {
    throw "invalid operands";
  }

  switch(result) {
      case ValueKind::FLOAT:
          ctx.push_op(Operation::math_op(Operation::OpKind::MATH_FL_FL_TO_FL, NODE_MATH_MULTIPLY));
          break;
      case ValueKind::VECTOR:
          ctx.rollback(savepoint);

          if(_left == ValueKind::VECTOR) {
            left->evaluate(ctx);
            right->evaluate(ctx);
          } else {
            right->evaluate(ctx);
            left->evaluate(ctx);
          }

          ctx.push_op(Operation::vector_math_op(Operation::OpKind::VECTOR_MATH_FL3_FL_TO_FL3, NODE_VECTOR_MATH_SCALE));
          break;
      default:
          BLI_assert_unreachable();
  }

  return result;
}

ValueKind Expression::div(EvaluationContext &ctx, Expression *left, Expression *right) {
  ValueKind _left = left->evaluate(ctx);

  switch(_left) {
      case ValueKind::FLOAT: {
          ValueKind _right = right->evaluate(ctx);

          if(_right != ValueKind::FLOAT) {
              throw "divisor must be a float";
          }

          ctx.push_op(Operation::math_op(Operation::OpKind::MATH_FL_FL_TO_FL, NODE_MATH_DIVIDE));
          break;
      }
      case ValueKind::VECTOR: {
          // v / f needs to be turned into v scaled by 1 / f
          ctx.push_op(Operation::float_op(1.0f));

          ValueKind _right = right->evaluate(ctx);

          if(_right != ValueKind::FLOAT) {
              throw "divisor must be a float";
          }

          ctx.push_op(Operation::math_op(Operation::OpKind::MATH_FL_FL_TO_FL, NODE_MATH_DIVIDE));
          ctx.push_op(Operation::vector_math_op(Operation::OpKind::VECTOR_MATH_FL3_FL_TO_FL3, NODE_VECTOR_MATH_SCALE));
          break;
      }
      default:
          BLI_assert_unreachable();
  }

  return _left;
}

ValueKind Expression::negate(EvaluationContext &ctx, Expression *x) {
  // there's no blender negate math op, so turn it into a multiply with -1
  ValueKind _x = x->evaluate(ctx);
  
  switch(_x) {
      case ValueKind::FLOAT:
        ctx.push_op(Operation::float_op(-1.0));
        ctx.push_op(Operation::math_op(Operation::OpKind::MATH_FL_FL_TO_FL, NODE_MATH_MULTIPLY));
        break;
      case ValueKind::VECTOR:
        ctx.push_op(Operation::float_op(-1.0));
        ctx.push_op(Operation::vector_math_op(Operation::OpKind::VECTOR_MATH_FL3_FL_TO_FL3, NODE_VECTOR_MATH_SCALE));
        break;
      default:
        BLI_assert_unreachable();
  }

  return _x;
}

ValueKind Expression::pow(EvaluationContext &ctx, Expression *x, Expression *y) {
  ValueKind _x = x->evaluate(ctx);
  ValueKind _y = y->evaluate(ctx);

  if(!ops_are(_x, _y, ValueKind::FLOAT)) {
    throw "invalid arguments";
  }
  
  ctx.push_op(Operation::math_op(Operation::OpKind::MATH_FL_FL_TO_FL, NODE_MATH_EXPONENT));
  return _x;
}

ValueKind Expression::lerp(EvaluationContext &ctx, Expression *a, Expression *b, Expression *t) {
  // (b - a) * t + a
  ValueKind b_sub_a = sub(ctx, b, a);
  t->evaluate(ctx);

  switch(b_sub_a) {
      case ValueKind::FLOAT:
          ctx.push_op(Operation::math_op(Operation::OpKind::MATH_FL_FL_TO_FL, NODE_MATH_MULTIPLY));
          a->evaluate(ctx);
          ctx.push_op(Operation::math_op(Operation::OpKind::MATH_FL_FL_TO_FL, NODE_MATH_ADD));
          break;
      case ValueKind::VECTOR:
          ctx.push_op(Operation::vector_math_op(Operation::OpKind::VECTOR_MATH_FL3_FL_TO_FL3, NODE_VECTOR_MATH_SCALE));
          a->evaluate(ctx);
          ctx.push_op(Operation::vector_math_op(Operation::OpKind::VECTOR_MATH_FL3_FL3_TO_FL3, NODE_VECTOR_MATH_ADD));
          break;
      default:
          BLI_assert_unreachable();
  }

  return b_sub_a;
}

ValueKind Expression::vec(EvaluationContext &ctx, Expression *x, Expression *y, Expression *z) {
  ValueKind _x = x->evaluate(ctx);
  ValueKind _y = y->evaluate(ctx);
  ValueKind _z = z->evaluate(ctx);

  if(_x != ValueKind::FLOAT || _y != ValueKind::FLOAT || _z != ValueKind::FLOAT) {
    throw "invalid arguments";
  }

  ctx.push_op(Operation::make_vector_op());

  return ValueKind::VECTOR;
}

ValueKind Expression::len(EvaluationContext &ctx, Expression *v) {
  ValueKind _v = v->evaluate(ctx);

  if(_v != ValueKind::VECTOR) {
    throw "invalid arguments";
  }

  ctx.push_op(Operation::vector_math_op(Operation::OpKind::VECTOR_MATH_FL3_TO_FL, NODE_VECTOR_MATH_LENGTH));

  return ValueKind::FLOAT;
}