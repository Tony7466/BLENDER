#pragma once

#include <memory>
#include <map>
#include <charconv>

//#include "BLI_math_vector_types.hh"

#include "value.h"

class EvaluationContext {
  blender::nodes::GeoNodeExecParams *params;

public:
  EvaluationContext(blender::nodes::GeoNodeExecParams *params) : params(params) {}

  std::unique_ptr<Value> get_number(Token token) {
    double d;
    auto result = std::from_chars(token.value.data(), token.value.data() + token.value.size(), d);
    return std::make_unique<ScalarValue>(d);
  }

  std::unique_ptr<Value> get_variable(Token token) {
    if(token.value[0] == 'v') {
        blender::float3 value(0, 0, 0);
        
        if(params) {
          value = params->extract_input<blender::float3>(token.value);
        }

        return std::make_unique<VectorValue>(blender::double3(value.x, value.y, value.z));
    }

    float value = 0.0;

    if(params) {
      value = params->extract_input<float>(token.value);
    }

    return std::make_unique<ScalarValue>(value);
  }
};
