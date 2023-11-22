#pragma once

#include <optional>

#include "BLI_map.hh"

namespace blender::nodes::node_geo_math_expression_cc {

enum class FunctionName {
  EXPONENT,
  SQRT,
  INV_SQRT,
  ABSOLUTE,
  RADIANS,
  DEGREES,
  SIGN,
  ROUND,
  FLOOR,
  CEIL,
  FRACTION,
  TRUNC,
  SINE,
  COSINE,
  TANGENT,
  SINH,
  COSH,
  TANH,
  ARCSINE,
  ARCCOSINE,
  ARCTANGENT,
  ADD,
  SUBTRACT,
  MULTIPLY,
  DIVIDE,
  POWER,
  LOGARITHM,
  MINIMUM,
  MAXIMUM,
  LESS_THAN,
  GREATER_THAN,
  MODULO,
  FLOORED_MODULO,
  SNAP,
  ARCTAN2,
  PINGPONG,
  MULTIPLY_ADD,
  COMPARE,
  SMOOTH_MIN,
  SMOOTH_MAX,
  WRAP,
  CROSS_PRODUCT,
  PROJECT,
  REFLECT,
  DOT_PRODUCT,
  DISTANCE,
  FACEFORWARD,
  REFRACT,
  LENGTH,
  SCALE,
  NORMALIZE,

  // Custom functions.
  LERP,
  VEC,
  X,
  Y,
  Z
};

struct FunctionDef {
  FunctionName name;
  int arg_count;
};

class FunctionLookup {
  Map<std::string_view, FunctionDef> funcs;

 public:
  FunctionLookup()
  {
    funcs.add("exponent", {FunctionName::EXPONENT, 1});
    funcs.add("sqrt", {FunctionName::SQRT, 1});
    funcs.add("inv_sqrt", {FunctionName::INV_SQRT, 1});
    funcs.add("absolute", {FunctionName::ABSOLUTE, 1});
    funcs.add("radians", {FunctionName::RADIANS, 1});
    funcs.add("degrees", {FunctionName::DEGREES, 1});
    funcs.add("sign", {FunctionName::SIGN, 1});
    funcs.add("round", {FunctionName::ROUND, 1});
    funcs.add("floor", {FunctionName::FLOOR, 1});
    funcs.add("ceil", {FunctionName::CEIL, 1});
    funcs.add("fraction", {FunctionName::FRACTION, 1});
    funcs.add("trunc", {FunctionName::TRUNC, 1});
    funcs.add("sine", {FunctionName::SINE, 1});
    funcs.add("cosine", {FunctionName::COSINE, 1});
    funcs.add("tangent", {FunctionName::TANGENT, 1});
    funcs.add("sinh", {FunctionName::SINH, 1});
    funcs.add("cosh", {FunctionName::COSH, 1});
    funcs.add("tanh", {FunctionName::TANH, 1});
    funcs.add("arcsine", {FunctionName::ARCSINE, 1});
    funcs.add("arccosine", {FunctionName::ARCCOSINE, 1});
    funcs.add("arctangent", {FunctionName::ARCTANGENT, 1});
    funcs.add("add", {FunctionName::ADD, 2});
    funcs.add("subtract", {FunctionName::SUBTRACT, 2});
    funcs.add("multiply", {FunctionName::MULTIPLY, 2});
    funcs.add("divide", {FunctionName::DIVIDE, 2});
    funcs.add("power", {FunctionName::POWER, 2});
    funcs.add("logarithm", {FunctionName::LOGARITHM, 2});
    funcs.add("minimum", {FunctionName::MINIMUM, 2});
    funcs.add("maximum", {FunctionName::MAXIMUM, 2});
    funcs.add("less_than", {FunctionName::LESS_THAN, 2});
    funcs.add("greater_than", {FunctionName::GREATER_THAN, 2});
    funcs.add("modulo", {FunctionName::MODULO, 2});
    funcs.add("floored_modulo", {FunctionName::FLOORED_MODULO, 2});
    funcs.add("snap", {FunctionName::SNAP, 2});
    funcs.add("arctan2", {FunctionName::ARCTAN2, 2});
    funcs.add("pingpong", {FunctionName::PINGPONG, 2});
    funcs.add("multiply_add", {FunctionName::MULTIPLY_ADD, 3});
    funcs.add("compare", {FunctionName::COMPARE, 3});
    funcs.add("smooth_min", {FunctionName::SMOOTH_MIN, 3});
    funcs.add("smooth_max", {FunctionName::SMOOTH_MAX, 3});
    funcs.add("wrap", {FunctionName::WRAP, 3});
    funcs.add("cross_product", {FunctionName::CROSS_PRODUCT, 2});
    funcs.add("project", {FunctionName::PROJECT, 2});
    funcs.add("reflect", {FunctionName::REFLECT, 2});
    funcs.add("dot_product", {FunctionName::DOT_PRODUCT, 2});
    funcs.add("distance", {FunctionName::DISTANCE, 2});
    funcs.add("faceforward", {FunctionName::FACEFORWARD, 3});
    funcs.add("refract", {FunctionName::REFRACT, 3});
    funcs.add("length", {FunctionName::LENGTH, 1});
    funcs.add("scale", {FunctionName::SCALE, 2});
    funcs.add("normalize", {FunctionName::NORMALIZE, 1});

    // Custom functions.
    funcs.add("lerp", {FunctionName::LERP, 3});
    funcs.add("vec", {FunctionName::VEC, 3});
    funcs.add("x", {FunctionName::X, 1});
    funcs.add("y", {FunctionName::Y, 1});
    funcs.add("z", {FunctionName::Z, 1});
  }

  std::optional<FunctionDef> lookup(std::string_view name) const
  {
    if (funcs.contains(name)) {
      return funcs.lookup(name);
    }

    return std::nullopt;
  }
};

inline std::optional<FunctionDef> lookup_function(std::string_view name)
{
  static const FunctionLookup funcs{};
  return funcs.lookup(name);
}

}  // namespace blender::nodes::node_geo_math_expression_cc