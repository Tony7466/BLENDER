/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_assert.h"
#include "BLI_math_matrix_types.hh"
#include "BLI_math_vector_types.hh"

#include "GPU_shader.hh"
#include "GPU_state.hh"
#include "GPU_texture.hh"

#include "COM_context.hh"
#include "COM_domain.hh"
#include "COM_result.hh"

namespace blender::realtime_compositor {

Result::Result(Context &context, DataType type, DataPrecision precision)
    : context_(&context), type_(type), precision_(precision), texture(context, type, precision)
{
}

void Result::allocate_texture(Domain domain)
{
  /* The result is not actually needed, so allocate a dummy single value texture instead. See the
   * method description for more information. */
  if (!should_compute()) {
    allocate_single_value();
    increment_reference_count();
    return;
  }

  is_single_value_ = false;
  if (context_->use_gpu()) {
    this->texture.allocate_gpu_from_pool(domain.size);
  }
  else {
    this->texture.allocate_cpu(domain.size);
  }
  domain_ = domain;
}

void Result::allocate_single_value()
{
  is_single_value_ = true;
  /* Single values are stored in 1x1 textures as well as the single value members. */
  if (context_->use_gpu()) {
    this->texture.allocate_gpu_from_pool(int2(1));
  }
  else {
    this->texture.allocate_cpu(int2(1));
  }
  domain_ = Domain::identity();
}

void Result::allocate_invalid()
{
  allocate_single_value();
  switch (type_) {
    case DataType::Float:
      set_float_value(0.0f);
      break;
    case DataType::Vector:
      set_vector_value(float4(0.0f));
      break;
    case DataType::Color:
      set_color_value(float4(0.0f));
      break;
    default:
      /* Other types are internal and do not support single values. */
      BLI_assert_unreachable();
      break;
  }
}

void Result::pass_through(Result &target)
{
  /* Increment the reference count of the master by the original reference count of the target. */
  increment_reference_count(target.reference_count());

  /* Make the target an exact copy of this result, but keep the initial reference count, as this is
   * a property of the original result and is needed for correctly resetting the result before the
   * next evaluation. */
  const int initial_reference_count = target.initial_reference_count_;
  target = *this;
  target.initial_reference_count_ = initial_reference_count;

  target.master_ = this;
}

void Result::steal_data(Result &source)
{
  BLI_assert(type_ == source.type_);
  BLI_assert(!this->texture.is_allocated() && source.texture.is_allocated());
  BLI_assert(master_ == nullptr && source.master_ == nullptr);

  is_single_value_ = source.is_single_value_;
  this->texture = source.texture;
  context_ = source.context_;
  domain_ = source.domain_;

  switch (type_) {
    case DataType::Float:
      float_value_ = source.float_value_;
      break;
    case DataType::Vector:
      vector_value_ = source.vector_value_;
      break;
    case DataType::Color:
      color_value_ = source.color_value_;
      break;
    default:
      /* Other types are internal and do not support single values. */
      break;
  }

  source.reset();
}

void Result::wrap_external(GPUTexture *texture)
{
  BLI_assert(!master_);
  is_single_value_ = false;
  this->texture.wrap_external(texture);
  domain_ = Domain(int2(GPU_texture_width(texture), GPU_texture_height(texture)));
}

void Result::set_transformation(const float3x3 &transformation)
{
  domain_.transformation = transformation;
}

void Result::transform(const float3x3 &transformation)
{
  domain_.transform(transformation);
}

RealizationOptions &Result::get_realization_options()
{
  return domain_.realization_options;
}

float Result::get_float_value() const
{
  return float_value_;
}

float4 Result::get_vector_value() const
{
  return vector_value_;
}

float4 Result::get_color_value() const
{
  return color_value_;
}

float Result::get_float_value_default(float default_value) const
{
  if (is_single_value()) {
    return get_float_value();
  }
  return default_value;
}

float4 Result::get_vector_value_default(const float4 &default_value) const
{
  if (is_single_value()) {
    return get_vector_value();
  }
  return default_value;
}

float4 Result::get_color_value_default(const float4 &default_value) const
{
  if (is_single_value()) {
    return get_color_value();
  }
  return default_value;
}

void Result::set_float_value(float value)
{
  float_value_ = value;
  GPU_texture_update(this->texture, GPU_DATA_FLOAT, &float_value_);
}

void Result::set_vector_value(const float4 &value)
{
  vector_value_ = value;
  GPU_texture_update(this->texture, GPU_DATA_FLOAT, vector_value_);
}

void Result::set_color_value(const float4 &value)
{
  color_value_ = value;
  GPU_texture_update(this->texture, GPU_DATA_FLOAT, color_value_);
}

void Result::set_initial_reference_count(int count)
{
  initial_reference_count_ = count;
}

void Result::reset()
{
  const int initial_reference_count = initial_reference_count_;
  *this = Result(*context_, type_, precision_);
  initial_reference_count_ = initial_reference_count;
  reference_count_ = initial_reference_count;
}

void Result::increment_reference_count(int count)
{
  /* If there is a master result, increment its reference count instead. */
  if (master_) {
    master_->increment_reference_count(count);
    return;
  }

  reference_count_ += count;
}

void Result::release()
{
  /* If there is a master result, release it instead. */
  if (master_) {
    master_->release();
    return;
  }

  /* Decrement the reference count, and if it is not yet zero, don't release the texture. */
  reference_count_--;
  if (reference_count_ != 0) {
    return;
  }

  this->texture.free();
}

bool Result::should_compute()
{
  return initial_reference_count_ != 0;
}

DataType Result::type() const
{
  return type_;
}

DataPrecision Result::precision() const
{
  return precision_;
}

void Result::set_precision(DataPrecision precision)
{
  this->texture.set_precision(precision);
  precision_ = precision;
}

bool Result::is_single_value() const
{
  return is_single_value_;
}

bool Result::is_allocated() const
{
  return this->texture.is_allocated();
}

int Result::reference_count() const
{
  /* If there is a master result, return its reference count instead. */
  if (master_) {
    return master_->reference_count();
  }
  return reference_count_;
}

const Domain &Result::domain() const
{
  return domain_;
}

}  // namespace blender::realtime_compositor
