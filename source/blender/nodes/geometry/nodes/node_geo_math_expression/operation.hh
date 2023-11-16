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

  float get_float() {
    BLI_assert(kind == ValueKind::FLOAT);
    return value.f;
  }

  blender::float3 get_vector() {
    BLI_assert(kind == ValueKind::VECTOR);
    return value.f3;
  }

  static Constant make_float(float f) {
    return Constant { ValueKind::FLOAT, { .f = f } };
  }

  static Constant make_vector(float x, float y, float z) {
    return Constant { ValueKind::VECTOR, { .f3 = blender::float3(x, y, z) } };
  }

  static Constant make_vector(blender::float3 f3) {
    return Constant { ValueKind::VECTOR, { .f3 = f3 } };
  }

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
    MAKE_VECTOR,
    MATH_FL_FL_TO_FL,
    VECTOR_MATH_FL3_FL3_TO_FL3,
    VECTOR_MATH_FL3_FL_TO_FL3,
    VECTOR_MATH_FL3_TO_FL
  };

  OpKind kind;

  union Poop {
    Constant constant;
    std::string_view variable = "";
    NodeMathOperation math;
    NodeVectorMathOperation vector_math;
  } op;

  std::string to_string() const {
    const char *kinds[] = {"CONSTANT", "VARIABLE", "MAKE_VECTOR", "MATH_FL_FL_TO_FL", "VECTOR_MATH_FL3_FL3_TO_FL3", "VECTOR_MATH_FL3_FL_TO_FL3", "VECTOR_MATH_FL3_TO_FL"};
    const char *kind_str = kinds[size_t(kind)];

    switch(kind) {
      case OpKind::CONSTANT:
        return fmt::format("{} {}", kind_str, op.constant.to_string());
      case OpKind::VARIABLE:
        return fmt::format("{} {}", kind_str, op.variable);
      case OpKind::MAKE_VECTOR:
        return fmt::format("{}", kind_str);
      case OpKind::MATH_FL_FL_TO_FL:
        return fmt::format("{} {}", kind_str, (int)op.math);
      case OpKind::VECTOR_MATH_FL3_FL3_TO_FL3:
      case OpKind::VECTOR_MATH_FL3_FL_TO_FL3:
        return fmt::format("{} {}", kind_str, (int)op.vector_math);
      case OpKind::VECTOR_MATH_FL3_TO_FL:
        return fmt::format("{} {}", kind_str, (int)op.vector_math);
    }

    return "UNKNOWN_OPERATION";
  }

  static Operation float_op(float f) {
    return { OpKind::CONSTANT, { .constant = Constant::make_float(f) } };
  }

  static Operation variable_op(std::string_view name) {
    return { OpKind::VARIABLE, { .variable = name } };
  }

  static Operation make_vector_op() {
    return { OpKind::MAKE_VECTOR };
  }

  static Operation vector_op(blender::float3 f3) {
    return { OpKind::CONSTANT, { .constant = Constant::make_vector(f3) } };
  }

  static Operation math_op(OpKind kind, NodeMathOperation op) {
    return { kind, { .math = op } };
  }

  static Operation vector_math_op(OpKind kind, NodeVectorMathOperation op) {
    return { kind, { .vector_math = op } };
  }
};
