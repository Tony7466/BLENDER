#include "evaluation_context.hh"

namespace blender::nodes::node_geo_math_expression_cc {

EvaluationContext::EvaluationContext(const std::function<fn::GField(std::string_view)> variable_cb) : variable_cb(std::move(variable_cb)) {

}

fn::GField EvaluationContext::get_variable(std::string_view name) {
  return variable_cb(name);
}

}