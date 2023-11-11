#pragma once

#include <memory>
#include <map>
#include <charconv>

#include "value.h"

class EvaluationContext {
  const char *text;
  blender::nodes::GeoNodeExecParams params;

public:
  EvaluationContext(blender::nodes::GeoNodeExecParams params) : params(params) {}

  std::unique_ptr<Value> get_number(Token token) {
    double d;
    auto result = std::from_chars(token.value.data(), token.value.data() + token.value.size(), d);
    return std::make_unique<DoubleValue>(d);
  }

  std::unique_ptr<Value> get_variable(Token token) {
    return std::make_unique<DoubleValue>(params.extract_input<float>(token.value));
  }
};
