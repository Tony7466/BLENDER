/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_math_vector_types.hh"

#include "GPU_shader.hh"
#include "GPU_texture.hh"

namespace blender::realtime_compositor {

class Context;

/* Make sure to update the format related static methods in the Texture class. */
enum class DataType : uint8_t {
  /* The following types are user facing and can be used as inputs and outputs of operations. The
   * color type represents an RGBA color. And the vector type represents a generic 4-component
   * vector, which can encode two 2D vectors, one 3D vector with the last component ignored, or
   * other dimensional data. */
  Float,
  Vector,
  Color,

  /* The following types are for internal use only, not user facing, and can't be used as inputs
   * and outputs of operations. It follows that they needn't be handled in implicit operations like
   * type conversion, shader, or single value reduction operations. */
  Float2,
  Float3,
  Int2,
};

/* The precision of the data. CPU data is always stored using full precision at the moment. */
enum class DataPrecision : uint8_t {
  Full,
  Half,
};

/* The type of storage used to hold the data. */
enum class DataStorageType : uint8_t {
  /* Stored as a GPUTexture on the GPU. */
  GPU,
  /* Stored as a contiguous float buffer the CPU. */
  FloatCPU,
  /* Stored as a contiguous integer buffer the CPU. */
  IntegerCPU,
};

/* ------------------------------------------------------------------------------------------------
 * Texture
 *
 * A texture stores the data of an image either on the GPU or the CPU depending on the allocation
 * method used. See the allocation methods for more information. */
class Texture {
 private:
  /* The context that the texture was created within, this should be initialized during
   * construction. */
  Context *context_ = nullptr;
  /* The data type of the texture's pixels. */
  DataType type_ = DataType::Float;
  /* The precision of the texture. CPU data is already stored in full precision. */
  DataPrecision precision_ = DataPrecision::Half;
  /* The type of the stored data. */
  DataStorageType storage_type_ = DataStorageType::GPU;
  /* The size of the texture. */
  int2 size_ = int2(1);
  /* The pixel data, stored in a GPU texture for GPU contexts or raw contagious arrays for CPU
   * contexts. */
  union {
    GPUTexture *gpu_texture_ = nullptr;
    float *float_texture_;
    int *integer_texture_;
  };
  /* If true, then the texture wraps external data that is not allocated nor managed by the
   * texture. This is set up by a call to the wrap_external method. In that case, when the
   * reference count eventually reach zero, the texture will not be freed. */
  bool is_external_ = false;
  /* If true, the GPU texture that holds the data was allocated from the texture pool of the
   * context and should be released back into the pool instead of being freed. For CPU storage,
   * this is irrelevant. */
  bool is_from_pool_ = false;

 public:
  /* Construct a texture of the given type and precision within the given context. */
  Texture(Context &context, DataType type, DataPrecision precision);

  /* Returns the appropriate texture format based on the given data type and precision. */
  static eGPUTextureFormat gpu_format(DataType type, DataPrecision precision);

  /* Returns the texture format that corresponds to the give one, but with the given precision. */
  static eGPUTextureFormat gpu_format(eGPUTextureFormat format, DataPrecision precision);

  /* Returns the precision of the given format. */
  static DataPrecision precision(eGPUTextureFormat format);

  /* Returns the type of the given format. */
  static DataType type(eGPUTextureFormat format);

  /* Implicit conversion to the internal GPU texture. */
  operator GPUTexture *() const;

  /* Returns the appropriate GPU texture format based on the texture's type and precision. */
  eGPUTextureFormat get_gpu_texture_format() const;

  /* Allocate the texture using GPU storage for the given size. This should be used as opposed to
   * allocate_gpu_from_pool for persistent data that spans more than one compositor evaluation. */
  void allocate_gpu(int2 size);

  /* Allocate the texture for the given size from the texture pool. Use the allocate_gpu method for
   * persistent data instead, see the description of that method for more information. */
  void allocate_gpu_from_pool(int2 size);

  /* Allocate the texture using CPU storage for the given size. */
  void allocate_cpu(int2 size);

  /* Set up the texture to wrap an external GPU texture that is not allocated nor managed by the
   * texture. The is_external_ member will be set to true, the size will be set to have the same
   * size as the texture, and the texture will be set to the given texture. See the is_external_
   * member for more information. The given texture should have the same format as the texture and
   * is assumed to have a lifetime that covers the evaluation of the compositor. */
  void wrap_external(GPUTexture *texture);

  /* Identical to GPU variant of wrap_external but wraps a float buffer instead. */
  void wrap_external(float *texture, int2 size);

  /* Identical to GPU variant of wrap_external but wraps an integer buffer instead. */
  void wrap_external(int *texture, int2 size);

  /* Frees the allocated data or releases it back into the pool, unless is_external_ is true, in
   * which case does nothing. */
  void free();

  /* Returns true if the texture is allocated. */
  bool is_allocated() const;

  /* Sets the precision of the texture. */
  void set_precision(DataPrecision precision);

  /* Bind the GPU texture to the texture image unit with the given name in the currently bound
   * given shader. This also inserts a memory barrier for texture fetches to ensure any prior
   * writes to the texture are reflected before reading from it. */
  void bind_as_texture(GPUShader *shader, const char *texture_name) const;

  /* Bind the GPU texture to the image unit with the given name in the currently bound given
   * shader. If read is true, a memory barrier will be inserted for image reads to ensure any prior
   * writes to the images are reflected before reading from it. */
  void bind_as_image(GPUShader *shader, const char *image_name, bool read = false) const;

  /* Unbind the texture which was previously bound using bind_as_texture. */
  void unbind_as_texture() const;

  /* Unbind the texture which was previously bound using bind_as_image. */
  void unbind_as_image() const;
};

}  // namespace blender::realtime_compositor
