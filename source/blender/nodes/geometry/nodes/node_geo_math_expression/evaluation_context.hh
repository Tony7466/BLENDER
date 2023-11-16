#pragma once

#include <string_view>
#include <functional>
#include <memory>

#include "operation.hh"

class EvaluationContext {
  const std::function<ValueKind(std::string_view)> variable_cb;
  std::vector<Operation> operations;

public:
  EvaluationContext(const std::function<ValueKind(std::string_view)> variable_cb);

  ValueKind get_variable(std::string_view name);
  void push_op(Operation op);
  
  const std::vector<Operation> &get_operations() const {
    return operations;
  }

  size_t savepoint() const {
    return operations.size();
  }

  void rollback(size_t savepoint) {
    operations.resize(savepoint);
  }
};
