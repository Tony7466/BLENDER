#include "NOD_math_functions.hh"

#include "math_processor.hh"

Constant MathProcessor::execute() {
  stack.clear();

  for(size_t i = 0; i < ops.size(); i++) {
      Operation op = ops[i];

      switch(op.kind) {
          case Operation::OpKind::CONSTANT:
            push(op.op.constant);
            break;
          case Operation::OpKind::VARIABLE:
            push(variable_cb(op.op.variable));
            break;
          case Operation::OpKind::MAKE_VECTOR: {
            float z = this->pop_float();
            float y = this->pop_float();
            float x = this->pop_float();
            push_vector(blender::float3(x, y, z));
            break;
          }
          case Operation::OpKind::MATH_FL_FL_TO_FL:
            blender::nodes::try_dispatch_float_math_fl_fl_to_fl(op.op.math, [this](auto, auto function, const blender::nodes::FloatMathOperationInfo &) {
                float b = this->pop_float();
                float a = this->pop_float();
                this->push_float(function(a, b));
            });
            break;
          case Operation::OpKind::VECTOR_MATH_FL3_FL3_TO_FL3:
            blender::nodes::try_dispatch_float_math_fl3_fl3_to_fl3(op.op.vector_math, [this](auto, auto function, const blender::nodes::FloatMathOperationInfo &) {
                blender::float3 b = this->pop_vector();
                blender::float3 a = this->pop_vector();
                this->push_vector(function(a, b));
            });
            break;
          case Operation::OpKind::VECTOR_MATH_FL3_FL_TO_FL3:
            blender::nodes::try_dispatch_float_math_fl3_fl_to_fl3(op.op.vector_math, [this](auto, auto function, const blender::nodes::FloatMathOperationInfo &) {
                float b = this->pop_float();
                blender::float3 a = this->pop_vector();
                this->push_vector(function(a, b));
            });
            break;
          case Operation::OpKind::VECTOR_MATH_FL3_TO_FL:
            blender::nodes::try_dispatch_float_math_fl3_to_fl(op.op.vector_math, [this](auto, auto function, const blender::nodes::FloatMathOperationInfo &) {
                blender::float3 a = this->pop_vector();
                this->push_float(function(a));
            });
            break;
          default:
            BLI_assert_unreachable();
      }
  }

  return pop();
}
