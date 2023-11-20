#pragma once

#include <functional>
#include <memory>
#include <string_view>

#include "FN_field.hh"

namespace blender::nodes::node_geo_math_expression_cc {

class EvaluationContext {
  const std::function<fn::GField(std::string_view)> variable_cb;

 public:
  EvaluationContext(const std::function<fn::GField(std::string_view)> variable_cb);

  fn::GField get_variable(std::string_view name);
};

}  // namespace blender::nodes::node_geo_math_expression_cc