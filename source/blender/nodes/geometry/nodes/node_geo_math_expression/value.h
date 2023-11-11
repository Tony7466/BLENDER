#pragma once

#include <memory>
#include <cmath>

class Value {
public:
  virtual double get_double() = 0;

  virtual std::unique_ptr<Value> add(Value *right) = 0;
  virtual std::unique_ptr<Value> sub(Value *right) = 0;
  virtual std::unique_ptr<Value> mul(Value *right) = 0;
  virtual std::unique_ptr<Value> div(Value *right) = 0;
  virtual std::unique_ptr<Value> pow(Value *right) = 0;
  virtual std::unique_ptr<Value> neg() = 0;

  virtual std::unique_ptr<Value> add_impl(class DoubleValue *left) = 0;
  virtual std::unique_ptr<Value> sub_impl(class DoubleValue *left) = 0;
  virtual std::unique_ptr<Value> mul_impl(class DoubleValue *left) = 0;
  virtual std::unique_ptr<Value> div_impl(class DoubleValue *left) = 0;
  virtual std::unique_ptr<Value> pow_impl(class DoubleValue *left) = 0;

  static std::unique_ptr<Value> lerp(Value *a, Value *b, Value *t) {
    // a + (b - a) * t
    return b->sub(a)->mul(t)->add(a);
  }
};

class DoubleValue : public Value {
  double value;

public:
  DoubleValue(double value) : value(value) {}

  double get_double() override {
    return value;
  }

  std::unique_ptr<Value> add(Value *right) override {
    return right->add_impl(this);
  }

  std::unique_ptr<Value> sub(Value *right) override {
    return right->sub_impl(this);
  }

  std::unique_ptr<Value> mul(Value *right) override {
    return right->mul_impl(this);
  }

  std::unique_ptr<Value> div(Value *right) override {
    return right->div_impl(this);
  }

  std::unique_ptr<Value> pow(Value *right) override {
    return right->pow_impl(this);
  }

  std::unique_ptr<Value> add_impl(DoubleValue *left) override {
    return std::make_unique<DoubleValue>(left->get_double() + value);
  }

  std::unique_ptr<Value> sub_impl(DoubleValue *left) override {
    return std::make_unique<DoubleValue>(left->get_double() - value);
  }

  std::unique_ptr<Value> mul_impl(DoubleValue *left) override {
    return std::make_unique<DoubleValue>(left->get_double() * value);
  }

  std::unique_ptr<Value> div_impl(DoubleValue *left) override {
    return std::make_unique<DoubleValue>(left->get_double() / value);
  }

  std::unique_ptr<Value> pow_impl(DoubleValue *left) override {
    return std::make_unique<DoubleValue>(std::pow(left->get_double(), value));
  }

  std::unique_ptr<Value> neg() override {
    return std::make_unique<DoubleValue>(-value);
  }
};