#pragma once

#include <memory>
#include <map>
#include <charconv>

#include "value.h"

class EvaluationContext {
  const std::function<std::unique_ptr<Value>(std::string_view)> variable_cb;

public:
  EvaluationContext(const std::function<std::unique_ptr<Value>(std::string_view)> variable_cb) : variable_cb(std::move(variable_cb)) {}

  std::unique_ptr<Value> get_variable(Token token) {
    return variable_cb(token.value);
  }
};
