#pragma once

#include <memory>
#include <cmath>

class Value {
public:
  virtual double get_double() const = 0;

  virtual std::unique_ptr<Value> add(const Value *right) const = 0;
  virtual std::unique_ptr<Value> sub(const Value *right) const = 0;
  virtual std::unique_ptr<Value> mul(const Value *right) const = 0;
  virtual std::unique_ptr<Value> div(const Value *right) const = 0;
  virtual std::unique_ptr<Value> pow(const Value *right) const = 0;
  virtual std::unique_ptr<Value> neg() const = 0;

  static std::unique_ptr<Value> lerp(const Value *a, const Value *b, const Value *t) {
    // a + (b - a) * t
    return b->sub(a)->mul(t)->add(a);
  }

  virtual std::unique_ptr<Value> add(const class DoubleValue *left) const = 0;
  virtual std::unique_ptr<Value> sub(const class DoubleValue *left) const = 0;
  virtual std::unique_ptr<Value> mul(const class DoubleValue *left) const = 0;
  virtual std::unique_ptr<Value> div(const class DoubleValue *left) const = 0;
  virtual std::unique_ptr<Value> pow(const class DoubleValue *left) const = 0;
};

class DoubleValue : public Value {
  const double value;

public:
  DoubleValue(double value) : value(value) {}

  double get_double() const override {
    return value;
  }

  std::unique_ptr<Value> add(const Value *right) const override {
    return right->add(this);
  }

  std::unique_ptr<Value> sub(const Value *right) const override {
    return right->sub(this);
  }

  std::unique_ptr<Value> mul(const Value *right) const override {
    return right->mul(this);
  }

  std::unique_ptr<Value> div(const Value *right) const override {
    return right->div(this);
  }

  std::unique_ptr<Value> pow(const Value *right) const override {
    return right->pow(this);
  }

  std::unique_ptr<Value> add(const DoubleValue *left) const override {
    return std::make_unique<DoubleValue>(left->get_double() + value);
  }

  std::unique_ptr<Value> sub(const DoubleValue *left) const override {
    return std::make_unique<DoubleValue>(left->get_double() - value);
  }

  std::unique_ptr<Value> mul(const DoubleValue *left) const override {
    return std::make_unique<DoubleValue>(left->get_double() * value);
  }

  std::unique_ptr<Value> div(const DoubleValue *left) const override {
    return std::make_unique<DoubleValue>(left->get_double() / value);
  }

  std::unique_ptr<Value> pow(const DoubleValue *left) const override {
    return std::make_unique<DoubleValue>(std::pow(left->get_double(), value));
  }

  std::unique_ptr<Value> neg() const override {
    return std::make_unique<DoubleValue>(-value);
  }
};
