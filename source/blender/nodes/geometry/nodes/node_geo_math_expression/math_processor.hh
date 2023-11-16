#pragma once

#include <functional>
#include <vector>

#include "operation.hh"

class MathProcessor {
    const std::vector<Operation> &ops;
    std::vector<Constant> stack;
    std::function<Constant(std::string_view)> variable_cb;

    void push(Constant c) {
        stack.emplace_back(c);
    }

    Constant pop() {
        Constant c = stack.back();
        stack.pop_back();
        return c;
    }

    void push_float(float f) {
        push(Constant::make_float(f));
    }

    void push_vector(blender::float3 f3) {
        push(Constant::make_vector(f3));
    }

    float pop_float() {
        return pop().get_float();
    }

    blender::float3 pop_vector() {
        return pop().get_vector();
    }

public:
    MathProcessor(const std::vector<Operation> &ops, std::function<Constant(std::string_view)> variable_cb) : ops(ops), variable_cb(variable_cb) {

    }

    Constant execute();
};
