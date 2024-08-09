/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "MEM_guardedalloc.h"

#include "BLI_assert.h"
#include "BLI_math_vector_types.hh"

#include "GPU_shader.hh"
#include "GPU_state.hh"
#include "GPU_texture.hh"

#include "COM_context.hh"
#include "COM_texture.hh"

namespace blender::realtime_compositor {

Texture::Texture(Context &context, DataType type, DataPrecision precision)
    : context_(&context), type_(type), precision_(precision)
{
}

eGPUTextureFormat Texture::gpu_format(DataType type, DataPrecision precision)
{
  switch (precision) {
    case DataPrecision::Half:
      switch (type) {
        case DataType::Float:
          return GPU_R16F;
        case DataType::Vector:
        case DataType::Color:
          return GPU_RGBA16F;
        case DataType::Float2:
          return GPU_RG16F;
        case DataType::Float3:
          return GPU_RGB16F;
        case DataType::Int2:
          return GPU_RG16I;
      }
      break;
    case DataPrecision::Full:
      switch (type) {
        case DataType::Float:
          return GPU_R32F;
        case DataType::Vector:
        case DataType::Color:
          return GPU_RGBA32F;
        case DataType::Float2:
          return GPU_RG32F;
        case DataType::Float3:
          return GPU_RGB32F;
        case DataType::Int2:
          return GPU_RG32I;
      }
      break;
  }

  BLI_assert_unreachable();
  return GPU_RGBA32F;
}

eGPUTextureFormat Texture::gpu_format(eGPUTextureFormat format, DataPrecision precision)
{
  switch (precision) {
    case DataPrecision::Half:
      switch (format) {
        /* Already half precision, return the input format. */
        case GPU_R16F:
        case GPU_RG16F:
        case GPU_RGB16F:
        case GPU_RGBA16F:
        case GPU_RG16I:
          return format;

        case GPU_R32F:
          return GPU_R16F;
        case GPU_RG32F:
          return GPU_RG16F;
        case GPU_RGB32F:
          return GPU_RGB16F;
        case GPU_RGBA32F:
          return GPU_RGBA16F;
        case GPU_RG32I:
          return GPU_RG16I;
        default:
          break;
      }
      break;
    case DataPrecision::Full:
      switch (format) {
        /* Already full precision, return the input format. */
        case GPU_R32F:
        case GPU_RG32F:
        case GPU_RGB32F:
        case GPU_RGBA32F:
        case GPU_RG32I:
          return format;

        case GPU_R16F:
          return GPU_R32F;
        case GPU_RG16F:
          return GPU_RG32F;
        case GPU_RGB16F:
          return GPU_RGB32F;
        case GPU_RGBA16F:
          return GPU_RGBA32F;
        case GPU_RG16I:
          return GPU_RG32I;
        default:
          break;
      }
      break;
  }

  BLI_assert_unreachable();
  return format;
}

DataPrecision Texture::precision(eGPUTextureFormat format)
{
  switch (format) {
    case GPU_R16F:
    case GPU_RG16F:
    case GPU_RGB16F:
    case GPU_RGBA16F:
    case GPU_RG16I:
      return DataPrecision::Half;
    case GPU_R32F:
    case GPU_RG32F:
    case GPU_RGB32F:
    case GPU_RGBA32F:
    case GPU_RG32I:
      return DataPrecision::Full;
    default:
      break;
  }

  BLI_assert_unreachable();
  return DataPrecision::Full;
}

DataType Texture::type(eGPUTextureFormat format)
{
  switch (format) {
    case GPU_R16F:
    case GPU_R32F:
      return DataType::Float;
    case GPU_RG16F:
    case GPU_RG32F:
      return DataType::Float2;
    case GPU_RGB16F:
    case GPU_RGB32F:
      return DataType::Float3;
    case GPU_RGBA16F:
    case GPU_RGBA32F:
      return DataType::Color;
    case GPU_RG16I:
    case GPU_RG32I:
      return DataType::Int2;
    default:
      break;
  }

  BLI_assert_unreachable();
  return DataType::Color;
}

Texture::operator GPUTexture *() const
{
  return gpu_texture_;
}

eGPUTextureFormat Texture::get_gpu_texture_format() const
{
  return Texture::gpu_format(type_, precision_);
}

void Texture::allocate_gpu(int2 size)
{
  size_ = size;
  storage_type_ = DataStorageType::GPU;
  gpu_texture_ = GPU_texture_create_2d("Compositor Texture",
                                       size.x,
                                       size.y,
                                       1,
                                       this->get_gpu_texture_format(),
                                       GPU_TEXTURE_USAGE_GENERAL,
                                       nullptr);
}

void Texture::allocate_gpu_from_pool(int2 size)
{
  size_ = size;
  storage_type_ = DataStorageType::GPU;
  gpu_texture_ = context_->texture_pool().acquire(size, this->get_gpu_texture_format());
  is_from_pool_ = true;
}

void Texture::allocate_cpu(int2 size)
{
  size_ = size;
  switch (type_) {
    case DataType::Float:
      float_texture_ = static_cast<float *>(
          MEM_malloc_arrayN(size.x * size.y, sizeof(float), __func__));
      storage_type_ = DataStorageType::FloatCPU;
      break;

    case DataType::Vector:
    case DataType::Color:
      float_texture_ = static_cast<float *>(
          MEM_malloc_arrayN(size.x * size.y, sizeof(float4), __func__));
      storage_type_ = DataStorageType::FloatCPU;
      break;

    case DataType::Float2:
      float_texture_ = static_cast<float *>(
          MEM_malloc_arrayN(size.x * size.y, sizeof(float2), __func__));
      storage_type_ = DataStorageType::FloatCPU;
      break;

    case DataType::Float3:
      float_texture_ = static_cast<float *>(
          MEM_malloc_arrayN(size.x * size.y, sizeof(float3), __func__));
      storage_type_ = DataStorageType::FloatCPU;
      break;

    case DataType::Int2:
      integer_texture_ = static_cast<int *>(
          MEM_malloc_arrayN(size.x * size.y, sizeof(int), __func__));
      storage_type_ = DataStorageType::IntegerCPU;
      break;
  }
}

void Texture::wrap_external(GPUTexture *texture)
{
  BLI_assert(!this->is_allocated());
  BLI_assert(GPU_texture_format(texture) == this->get_gpu_texture_format());

  gpu_texture_ = texture;
  storage_type_ = DataStorageType::GPU;
  is_external_ = true;
  size_ = int2(GPU_texture_width(texture), GPU_texture_height(texture));
}

void Texture::wrap_external(float *texture, int2 size)
{
  BLI_assert(!this->is_allocated());

  float_texture_ = texture;
  storage_type_ = DataStorageType::FloatCPU;
  is_external_ = true;
  size_ = size;
}

void Texture::wrap_external(int *texture, int2 size)
{
  BLI_assert(!this->is_allocated());

  integer_texture_ = texture;
  storage_type_ = DataStorageType::IntegerCPU;
  is_external_ = true;
  size_ = size;
}

void Texture::free()
{
  if (is_external_) {
    return;
  }

  switch (storage_type_) {
    case DataStorageType::GPU:
      if (is_from_pool_) {
        context_->texture_pool().release(gpu_texture_);
      }
      else {
        GPU_texture_free(gpu_texture_);
      }
      gpu_texture_ = nullptr;
      break;
    case DataStorageType::FloatCPU:
      MEM_freeN(float_texture_);
      float_texture_ = nullptr;
      break;
    case DataStorageType::IntegerCPU:
      MEM_freeN(integer_texture_);
      integer_texture_ = nullptr;
      break;
  }
}

bool Texture::is_allocated() const
{
  switch (storage_type_) {
    case DataStorageType::GPU:
      return gpu_texture_ != nullptr;
    case DataStorageType::FloatCPU:
      return float_texture_ != nullptr;
    case DataStorageType::IntegerCPU:
      return integer_texture_ != nullptr;
  }
}

void Texture::set_precision(DataPrecision precision)
{
  /* Changing the precision can only be done if it wasn't allocated yet. */
  BLI_assert(!this->is_allocated());
  precision_ = precision;
}

void Texture::bind_as_texture(GPUShader *shader, const char *texture_name) const
{
  /* Make sure any prior writes to the texture are reflected before reading from it. */
  GPU_memory_barrier(GPU_BARRIER_TEXTURE_FETCH);

  const int texture_image_unit = GPU_shader_get_sampler_binding(shader, texture_name);
  GPU_texture_bind(gpu_texture_, texture_image_unit);
}

void Texture::bind_as_image(GPUShader *shader, const char *image_name, bool read) const
{
  /* Make sure any prior writes to the texture are reflected before reading from it. */
  if (read) {
    GPU_memory_barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS);
  }

  const int image_unit = GPU_shader_get_sampler_binding(shader, image_name);
  GPU_texture_image_bind(gpu_texture_, image_unit);
}

void Texture::unbind_as_texture() const
{
  GPU_texture_unbind(gpu_texture_);
}

void Texture::unbind_as_image() const
{
  GPU_texture_image_unbind(gpu_texture_);
}

}  // namespace blender::realtime_compositor
