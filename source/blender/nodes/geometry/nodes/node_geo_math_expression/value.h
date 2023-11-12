#pragma once

#include <memory>
#include <cmath>

class ScalarValue;
class VectorValue;

class Value {
public:
  virtual double get_scalar() const = 0;
  virtual blender::double3 get_vector() const = 0;

  virtual bool is_scalar() const
  {
    return false;
  }

  virtual bool is_vector() const
  {
    return false;
  }

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

  static std::unique_ptr<Value> vec(const Value *x, const Value *y, const Value *z);
  static std::unique_ptr<Value> x(const Value *v);
  static std::unique_ptr<Value> y(const Value *v);
  static std::unique_ptr<Value> z(const Value *v);
  static std::unique_ptr<Value> len(const Value *v);

  virtual std::unique_ptr<Value> add(const ScalarValue *left) const = 0;
  virtual std::unique_ptr<Value> sub(const ScalarValue *left) const = 0;
  virtual std::unique_ptr<Value> mul(const ScalarValue *left) const = 0;
  virtual std::unique_ptr<Value> div(const ScalarValue *left) const = 0;
  virtual std::unique_ptr<Value> pow(const ScalarValue *left) const = 0;

  virtual std::unique_ptr<Value> add(const VectorValue *left) const = 0;
  virtual std::unique_ptr<Value> sub(const VectorValue *left) const = 0;
  virtual std::unique_ptr<Value> mul(const VectorValue *left) const = 0;
  virtual std::unique_ptr<Value> div(const VectorValue *left) const = 0;
  virtual std::unique_ptr<Value> pow(const VectorValue *left) const = 0;
};

class ScalarValue : public Value {
  const double value;

public:
  ScalarValue(double value) : value(value) {}

  bool is_scalar() const override
  {
    return true;
  }

  double get_scalar() const override {
    return value;
  }

  blender::double3 get_vector() const override {
    //TODO: create error type for this class
    throw "get_vector() called for ScalarValue";
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

  std::unique_ptr<Value> neg() const override {
    return std::make_unique<ScalarValue>(-value);
  }

  // scalar op scalar
  std::unique_ptr<Value> add(const ScalarValue *left) const override {
    return std::make_unique<ScalarValue>(left->get_scalar() + value);
  }

  std::unique_ptr<Value> sub(const ScalarValue *left) const override {
    return std::make_unique<ScalarValue>(left->get_scalar() - value);
  }

  std::unique_ptr<Value> mul(const ScalarValue *left) const override {
    return std::make_unique<ScalarValue>(left->get_scalar() * value);
  }

  std::unique_ptr<Value> div(const ScalarValue *left) const override {
    return std::make_unique<ScalarValue>(left->get_scalar() / value);
  }

  std::unique_ptr<Value> pow(const ScalarValue *left) const override {
    return std::make_unique<ScalarValue>(std::pow(left->get_scalar(), value));
  }

  // vector op scalar
  std::unique_ptr<Value> add(const VectorValue *left) const override {
    throw "invalid operation: VectorValue + ScalarValue";
  }

  std::unique_ptr<Value> sub(const VectorValue *left) const override {
    throw "invalid operation: VectorValue - ScalarValue";
  }

  std::unique_ptr<Value> mul(const VectorValue *left) const override;

  std::unique_ptr<Value> div(const VectorValue *left) const override;

  std::unique_ptr<Value> pow(const VectorValue *left) const override {
    throw "invalid operation: pow(VectorValue, ScalarValue)";
  }
};

class VectorValue : public Value {
  const blender::double3 value;

public:
  VectorValue(blender::double3 value) : value(value) {}

  bool is_vector() const override
  {
    return true;
  }

  double get_scalar() const override {
    //TODO: create error type for this class
    throw "get_scalar() called for VectorValue";
  }

  blender::double3 get_vector() const override {
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

  std::unique_ptr<Value> neg() const override {
    return std::make_unique<VectorValue>(-value);
  }

  // vector op vector
  std::unique_ptr<Value> add(const VectorValue *left) const override {
    return std::make_unique<VectorValue>(left->get_vector() + value);
  }

  std::unique_ptr<Value> sub(const VectorValue *left) const override {
    return std::make_unique<VectorValue>(left->get_vector() - value);
  }

  std::unique_ptr<Value> mul(const VectorValue *left) const override {
    throw "invalid operation: VectorValue * VectorValue";
  }

  std::unique_ptr<Value> div(const VectorValue *left) const override {
    throw "invalid operation: VectorValue / VectorValue";
  }

  std::unique_ptr<Value> pow(const VectorValue *left) const override {
    throw "invalid operation: pow(VectorValue, VectorValue)";
  }

  // vector op scalar
  std::unique_ptr<Value> add(const ScalarValue *left) const override {
    throw "invalid operation: ScalarValue + VectorValue";
  }

  std::unique_ptr<Value> sub(const ScalarValue *left) const override {
    throw "invalid operation: ScalarValue - VectorValue";
  }

  std::unique_ptr<Value> mul(const ScalarValue *left) const override {
    return std::make_unique<VectorValue>(left->get_scalar() * value);
  }

  std::unique_ptr<Value> div(const ScalarValue *left) const override {
    return std::make_unique<VectorValue>(left->get_scalar() / value);
  }

  std::unique_ptr<Value> pow(const ScalarValue *left) const override {
    throw "invalid operation: pow(ScalarValue, VectorValue)";
  }
};

std::unique_ptr<Value> Value::vec(const Value *x, const Value *y, const Value *z)
{
  return std::make_unique<VectorValue>(blender::double3(x->get_scalar(), y->get_scalar(), z->get_scalar()));
}

std::unique_ptr<Value> Value::x(const Value *v)
{
  return std::make_unique<ScalarValue>(v->get_vector().x);
}

std::unique_ptr<Value> Value::y(const Value *v)
{
  return std::make_unique<ScalarValue>(v->get_vector().y);
}

std::unique_ptr<Value> Value::z(const Value *v)
{
  return std::make_unique<ScalarValue>(v->get_vector().z);
}

std::unique_ptr<Value> Value::len(const Value *v)
{
  return std::make_unique<ScalarValue>(blender::math::length(v->get_vector()));
}

std::unique_ptr<Value> ScalarValue::mul(const VectorValue *left) const
{
  return std::make_unique<VectorValue>(left->get_vector() * value);
}

std::unique_ptr<Value> ScalarValue::div(const VectorValue *left) const
{
  return std::make_unique<VectorValue>(left->get_vector() / value);
}
