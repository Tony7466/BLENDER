#pragma once

#include "evaluation_context.hh"

EvaluationContext::EvaluationContext(const std::function<ValueKind(std::string_view)> variable_cb) : variable_cb(std::move(variable_cb)) {

}

ValueKind EvaluationContext::get_variable(std::string_view name) {
  return variable_cb(name);
}

void EvaluationContext::push_op(Operation op) {
  printf("added op: %s\n", op.to_string().c_str());
  operations.emplace_back(op);
}

