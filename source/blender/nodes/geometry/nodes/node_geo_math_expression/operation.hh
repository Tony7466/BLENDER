#pragma once

#include <string_view>

#include <fmt/format.h>
#include "DNA_node_types.h"
#include "BLI_math_vector_types.hh"

enum class ValueKind {
  FLOAT,
  VECTOR
};

struct Constant {
  ValueKind kind;

  union {
    float f;
    blender::float3 f3;
  } value;

  std::string to_string() const {
    switch(kind) {
      case ValueKind::FLOAT:
        return fmt::format("{}", value.f);
      case ValueKind::VECTOR:
        return fmt::format("({}, {}, {})", value.f3.x, value.f3.y, value.f3.z);
    }

    return "UNKNOWN_CONSTANT";
  }
};

struct Operation {
  enum class OpKind {
    CONSTANT,
    VARIABLE,
    MATH_FL_FL_TO_FL,
    VECTOR_MATH_FL3_FL3_TO_FL3,
    VECTOR_MATH_FL3_FL_TO_FL3
  };

  OpKind kind;

  union Poop {
    Constant constant;
    std::string_view variable = "";
    NodeMathOperation math;
    NodeVectorMathOperation vector_math;
  } op;

  std::string to_string() const {
    const char *kinds[] = {"CONSTANT", "VARIABLE", "MATH_FL_FL_TO_FL", "VECTOR_MATH_FL3_FL3_TO_FL3", "VECTOR_MATH_FL3_FL_TO_FL3"};
    const char *kind_str = kinds[size_t(kind)];

    switch(kind) {
      case OpKind::CONSTANT:
        return fmt::format("{} {}", kind_str, op.constant.to_string());
      case OpKind::VARIABLE:
        return fmt::format("{} {}", kind_str, op.variable);
      case OpKind::MATH_FL_FL_TO_FL:
        return fmt::format("{} {}", kind_str, (int)op.math);
      case OpKind::VECTOR_MATH_FL3_FL3_TO_FL3:
      case OpKind::VECTOR_MATH_FL3_FL_TO_FL3:
        return fmt::format("{} {}", kind_str, (int)op.vector_math);
    }

    return "UNKNOWN_OPERATION";
  }

  static Operation float_op(float f) {
    return { OpKind::CONSTANT, { .constant = { ValueKind::FLOAT, { .f = f } } } };
  }

  static Operation variable_op(std::string_view name) {
    return { OpKind::VARIABLE, { .variable = name } };
  }

  static Operation vector_op(blender::float3 f3) {
    return { OpKind::CONSTANT, { .constant = { ValueKind::VECTOR, { .f3 = f3 } } } };
  }

  static Operation math_op(OpKind kind, NodeMathOperation op) {
    return { kind, { .math = op } };
  }

  static Operation vector_math_op(OpKind kind, NodeVectorMathOperation op) {
    return { kind, { .vector_math = op } };
  }
};
