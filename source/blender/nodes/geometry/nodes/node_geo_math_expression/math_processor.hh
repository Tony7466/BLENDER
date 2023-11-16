#pragma once

#include <functional>
#include <vector>

#include "operation.hh"

namespace blender::nodes::node_geo_math_expression_cc {

class MathProcessor {
    const std::vector<Operation> &ops;
    std::vector<Constant> stack;
    std::function<Constant(std::string_view)> variable_cb;

public:
    MathProcessor(const std::vector<Operation> &ops, std::function<Constant(std::string_view)> variable_cb) : ops(ops), variable_cb(variable_cb) {

    }

    Constant execute();

    void push(Constant c) {
        stack.emplace_back(c);
    }

    Constant pop() {
        Constant c = stack.back();
        stack.pop_back();
        return c;
    }
};

}