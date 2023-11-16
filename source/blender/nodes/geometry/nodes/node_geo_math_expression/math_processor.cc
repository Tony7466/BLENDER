#include "NOD_math_functions.hh"

#include "math_processor.hh"

namespace blender::nodes::node_geo_math_expression_cc {

Constant MathProcessor::execute() {
  stack.clear();

  for(size_t i = 0; i < ops.size(); i++) {
      Operation op = ops[i];
      Constant c;

      switch(op.kind) {
          case Operation::OpKind::CONSTANT:
              c = op.op.constant;
              break;
          case Operation::OpKind::VARIABLE:
              c = variable_cb(op.op.variable);
              break;
          case Operation::OpKind::MATH_FL_FL_TO_FL:
              blender::nodes::try_dispatch_float_math_fl_fl_to_fl(op.op.math, [this, &c](auto, auto function, const blender::nodes::FloatMathOperationInfo &) {
                  c.kind = ValueKind::FLOAT;
                  c.value.f = function(this->pop().value.f, this->pop().value.f);
              });
              break;
          case Operation::OpKind::VECTOR_MATH_FL3_FL3_TO_FL3:
              blender::nodes::try_dispatch_float_math_fl3_fl3_to_fl3(op.op.vector_math, [this, &c](auto, auto function, const blender::nodes::FloatMathOperationInfo &) {
                  c.kind = ValueKind::VECTOR;
                  c.value.f3 = function(this->pop().value.f3, this->pop().value.f3);
              });
              break;
          case Operation::OpKind::VECTOR_MATH_FL3_FL_TO_FL3:
              blender::nodes::try_dispatch_float_math_fl3_fl_to_fl3(op.op.vector_math, [this, &c](auto, auto function, const blender::nodes::FloatMathOperationInfo &) {
                  c.kind = ValueKind::VECTOR;
                  c.value.f3 = function(this->pop().value.f3, this->pop().value.f);
              });
              break;
          default:
              BLI_assert_unreachable();
      }

      push(c);
  }

  return pop();
}

}