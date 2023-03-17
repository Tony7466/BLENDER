/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 */

#include "vk_data_conversion.hh"

namespace blender::gpu {
static ConversionType type_of_conversion_float(eGPUTextureFormat device_format)
{
  switch (device_format) {
    case GPU_RGBA32F:
    case GPU_RG32F:
    case GPU_R32F:
    case GPU_DEPTH_COMPONENT32F:
      return ConversionType::PASS_THROUGH;

    case GPU_RGBA16F:
    case GPU_RG16F:
    case GPU_R16F:
    case GPU_RGB16F:
      return ConversionType::FLOAT_TO_HALF;

    case GPU_RGBA8:
    case GPU_RG8:
    case GPU_R8:
      return ConversionType::FLOAT_TO_UNORM8;

    case GPU_RGBA8_SNORM:
    case GPU_RGB8_SNORM:
    case GPU_RG8_SNORM:
    case GPU_R8_SNORM:
      return ConversionType::FLOAT_TO_SNORM8;

    case GPU_RGBA16:
    case GPU_RG16:
    case GPU_R16:
      return ConversionType::FLOAT_TO_UNORM16;

    case GPU_RGBA16_SNORM:
    case GPU_RGB16_SNORM:
    case GPU_RG16_SNORM:
    case GPU_R16_SNORM:
      return ConversionType::FLOAT_TO_SNORM16;

    case GPU_RGB32F: /* GPU_RGB32F Not supported by vendors. */
    case GPU_RGBA8UI:
    case GPU_RGBA8I:
    case GPU_RGBA16UI:
    case GPU_RGBA16I:
    case GPU_RGBA32UI:
    case GPU_RGBA32I:
    case GPU_RG8UI:
    case GPU_RG8I:
    case GPU_RG16UI:
    case GPU_RG16I:
    case GPU_RG32UI:
    case GPU_RG32I:
    case GPU_R8UI:
    case GPU_R8I:
    case GPU_R16UI:
    case GPU_R16I:
    case GPU_R32UI:
    case GPU_R32I:
    case GPU_RGB10_A2:
    case GPU_RGB10_A2UI:
    case GPU_R11F_G11F_B10F:
    case GPU_DEPTH32F_STENCIL8:
    case GPU_DEPTH24_STENCIL8:
    case GPU_SRGB8_A8:
    case GPU_RGB8UI:
    case GPU_RGB8I:
    case GPU_RGB8:
    case GPU_RGB16UI:
    case GPU_RGB16I:
    case GPU_RGB16:
    case GPU_RGB32UI:
    case GPU_RGB32I:
    case GPU_SRGB8_A8_DXT1:
    case GPU_SRGB8_A8_DXT3:
    case GPU_SRGB8_A8_DXT5:
    case GPU_RGBA8_DXT1:
    case GPU_RGBA8_DXT3:
    case GPU_RGBA8_DXT5:
    case GPU_SRGB8:
    case GPU_RGB9_E5:
    case GPU_DEPTH_COMPONENT24:
    case GPU_DEPTH_COMPONENT16:
      return ConversionType::UNSUPPORTED;
  }
  return ConversionType::UNSUPPORTED;
}

static ConversionType type_of_conversion_int(eGPUTextureFormat device_format)
{
  switch (device_format) {
    case GPU_RGBA32I:
    case GPU_RG32I:
    case GPU_R32I:
      return ConversionType::PASS_THROUGH;

    case GPU_RGBA16I:
    case GPU_RG16I:
    case GPU_R16I:
      return ConversionType::I32_TO_I16;

    case GPU_RGBA8I:
    case GPU_RG8I:
    case GPU_R8I:
      return ConversionType::I32_TO_I8;

    case GPU_RGBA8UI:
    case GPU_RGBA8:
    case GPU_RGBA16UI:
    case GPU_RGBA16F:
    case GPU_RGBA16:
    case GPU_RGBA32UI:
    case GPU_RGBA32F:
    case GPU_RG8UI:
    case GPU_RG8:
    case GPU_RG16UI:
    case GPU_RG16F:
    case GPU_RG32UI:
    case GPU_RG32F:
    case GPU_RG16:
    case GPU_R8UI:
    case GPU_R8:
    case GPU_R16UI:
    case GPU_R16F:
    case GPU_R16:
    case GPU_R32UI:
    case GPU_R32F:
    case GPU_RGB10_A2:
    case GPU_RGB10_A2UI:
    case GPU_R11F_G11F_B10F:
    case GPU_DEPTH32F_STENCIL8:
    case GPU_DEPTH24_STENCIL8:
    case GPU_SRGB8_A8:
    case GPU_RGBA8_SNORM:
    case GPU_RGBA16_SNORM:
    case GPU_RGB8UI:
    case GPU_RGB8I:
    case GPU_RGB8:
    case GPU_RGB8_SNORM:
    case GPU_RGB16UI:
    case GPU_RGB16I:
    case GPU_RGB16F:
    case GPU_RGB16:
    case GPU_RGB16_SNORM:
    case GPU_RGB32UI:
    case GPU_RGB32I:
    case GPU_RGB32F:
    case GPU_RG8_SNORM:
    case GPU_RG16_SNORM:
    case GPU_R8_SNORM:
    case GPU_R16_SNORM:
    case GPU_SRGB8_A8_DXT1:
    case GPU_SRGB8_A8_DXT3:
    case GPU_SRGB8_A8_DXT5:
    case GPU_RGBA8_DXT1:
    case GPU_RGBA8_DXT3:
    case GPU_RGBA8_DXT5:
    case GPU_SRGB8:
    case GPU_RGB9_E5:
    case GPU_DEPTH_COMPONENT32F:
    case GPU_DEPTH_COMPONENT24:
    case GPU_DEPTH_COMPONENT16:
      return ConversionType::UNSUPPORTED;
  }
  return ConversionType::UNSUPPORTED;
}

static ConversionType type_of_conversion_uint(eGPUTextureFormat device_format)
{
  switch (device_format) {
    case GPU_RGBA32UI:
    case GPU_RG32UI:
    case GPU_R32UI:
      return ConversionType::PASS_THROUGH;

    case GPU_RGBA16UI:
    case GPU_RG16UI:
    case GPU_R16UI:
    case GPU_RGB16UI:
      return ConversionType::UI32_TO_UI16;

    case GPU_RGBA8UI:
    case GPU_RG8UI:
    case GPU_R8UI:
      return ConversionType::UI32_TO_UI8;

    case GPU_RGBA8I:
    case GPU_RGBA8:
    case GPU_RGBA16I:
    case GPU_RGBA16F:
    case GPU_RGBA16:
    case GPU_RGBA32I:
    case GPU_RGBA32F:
    case GPU_RG8I:
    case GPU_RG8:
    case GPU_RG16I:
    case GPU_RG16F:
    case GPU_RG16:
    case GPU_RG32I:
    case GPU_RG32F:
    case GPU_R8I:
    case GPU_R8:
    case GPU_R16I:
    case GPU_R16F:
    case GPU_R16:
    case GPU_R32I:
    case GPU_R32F:
    case GPU_RGB10_A2:
    case GPU_RGB10_A2UI:
    case GPU_R11F_G11F_B10F:
    case GPU_DEPTH32F_STENCIL8:
    case GPU_DEPTH24_STENCIL8:
    case GPU_SRGB8_A8:
    case GPU_RGBA8_SNORM:
    case GPU_RGBA16_SNORM:
    case GPU_RGB8UI:
    case GPU_RGB8I:
    case GPU_RGB8:
    case GPU_RGB8_SNORM:
    case GPU_RGB16I:
    case GPU_RGB16F:
    case GPU_RGB16:
    case GPU_RGB16_SNORM:
    case GPU_RGB32UI:
    case GPU_RGB32I:
    case GPU_RGB32F:
    case GPU_RG8_SNORM:
    case GPU_RG16_SNORM:
    case GPU_R8_SNORM:
    case GPU_R16_SNORM:
    case GPU_SRGB8_A8_DXT1:
    case GPU_SRGB8_A8_DXT3:
    case GPU_SRGB8_A8_DXT5:
    case GPU_RGBA8_DXT1:
    case GPU_RGBA8_DXT3:
    case GPU_RGBA8_DXT5:
    case GPU_SRGB8:
    case GPU_RGB9_E5:
    case GPU_DEPTH_COMPONENT32F:
    case GPU_DEPTH_COMPONENT24:
    case GPU_DEPTH_COMPONENT16:
      return ConversionType::UNSUPPORTED;
  }
  return ConversionType::UNSUPPORTED;
}

static ConversionType type_of_conversion_half(eGPUTextureFormat device_format)
{
  switch (device_format) {
    case GPU_RGBA16F:
    case GPU_RG16F:
    case GPU_R16F:
      return ConversionType::PASS_THROUGH;

    case GPU_RGBA8UI:
    case GPU_RGBA8I:
    case GPU_RGBA8:
    case GPU_RGBA16UI:
    case GPU_RGBA16I:
    case GPU_RGBA16:
    case GPU_RGBA32UI:
    case GPU_RGBA32I:
    case GPU_RGBA32F:
    case GPU_RG8UI:
    case GPU_RG8I:
    case GPU_RG8:
    case GPU_RG16UI:
    case GPU_RG16I:
    case GPU_RG16:
    case GPU_RG32UI:
    case GPU_RG32I:
    case GPU_RG32F:
    case GPU_R8UI:
    case GPU_R8I:
    case GPU_R8:
    case GPU_R16UI:
    case GPU_R16I:
    case GPU_R16:
    case GPU_R32UI:
    case GPU_R32I:
    case GPU_R32F:
    case GPU_RGB10_A2:
    case GPU_RGB10_A2UI:
    case GPU_R11F_G11F_B10F:
    case GPU_DEPTH32F_STENCIL8:
    case GPU_DEPTH24_STENCIL8:
    case GPU_SRGB8_A8:
    case GPU_RGBA8_SNORM:
    case GPU_RGBA16_SNORM:
    case GPU_RGB8UI:
    case GPU_RGB8I:
    case GPU_RGB8:
    case GPU_RGB8_SNORM:
    case GPU_RGB16UI:
    case GPU_RGB16I:
    case GPU_RGB16F:
    case GPU_RGB16:
    case GPU_RGB16_SNORM:
    case GPU_RGB32UI:
    case GPU_RGB32I:
    case GPU_RGB32F:
    case GPU_RG8_SNORM:
    case GPU_RG16_SNORM:
    case GPU_R8_SNORM:
    case GPU_R16_SNORM:
    case GPU_SRGB8_A8_DXT1:
    case GPU_SRGB8_A8_DXT3:
    case GPU_SRGB8_A8_DXT5:
    case GPU_RGBA8_DXT1:
    case GPU_RGBA8_DXT3:
    case GPU_RGBA8_DXT5:
    case GPU_SRGB8:
    case GPU_RGB9_E5:
    case GPU_DEPTH_COMPONENT32F:
    case GPU_DEPTH_COMPONENT24:
    case GPU_DEPTH_COMPONENT16:
      return ConversionType::UNSUPPORTED;
  }
  return ConversionType::UNSUPPORTED;
}

static ConversionType type_of_conversion_ubyte(eGPUTextureFormat device_format)
{
  switch (device_format) {
    case GPU_RGBA8UI:
    case GPU_RG8UI:
    case GPU_R8UI:
      return ConversionType::PASS_THROUGH;

    case GPU_RGBA8I:
    case GPU_RGBA8:
    case GPU_RGBA16UI:
    case GPU_RGBA16I:
    case GPU_RGBA16F:
    case GPU_RGBA16:
    case GPU_RGBA32UI:
    case GPU_RGBA32I:
    case GPU_RGBA32F:
    case GPU_RG8I:
    case GPU_RG8:
    case GPU_RG16UI:
    case GPU_RG16I:
    case GPU_RG16F:
    case GPU_RG16:
    case GPU_RG32UI:
    case GPU_RG32I:
    case GPU_RG32F:
    case GPU_R8I:
    case GPU_R8:
    case GPU_R16UI:
    case GPU_R16I:
    case GPU_R16F:
    case GPU_R16:
    case GPU_R32UI:
    case GPU_R32I:
    case GPU_R32F:
    case GPU_RGB10_A2:
    case GPU_RGB10_A2UI:
    case GPU_R11F_G11F_B10F:
    case GPU_DEPTH32F_STENCIL8:
    case GPU_DEPTH24_STENCIL8:
    case GPU_SRGB8_A8:
    case GPU_RGBA8_SNORM:
    case GPU_RGBA16_SNORM:
    case GPU_RGB8UI:
    case GPU_RGB8I:
    case GPU_RGB8:
    case GPU_RGB8_SNORM:
    case GPU_RGB16UI:
    case GPU_RGB16I:
    case GPU_RGB16F:
    case GPU_RGB16:
    case GPU_RGB16_SNORM:
    case GPU_RGB32UI:
    case GPU_RGB32I:
    case GPU_RGB32F:
    case GPU_RG8_SNORM:
    case GPU_RG16_SNORM:
    case GPU_R8_SNORM:
    case GPU_R16_SNORM:
    case GPU_SRGB8_A8_DXT1:
    case GPU_SRGB8_A8_DXT3:
    case GPU_SRGB8_A8_DXT5:
    case GPU_RGBA8_DXT1:
    case GPU_RGBA8_DXT3:
    case GPU_RGBA8_DXT5:
    case GPU_SRGB8:
    case GPU_RGB9_E5:
    case GPU_DEPTH_COMPONENT32F:
    case GPU_DEPTH_COMPONENT24:
    case GPU_DEPTH_COMPONENT16:
      return ConversionType::UNSUPPORTED;
  }
  return ConversionType::UNSUPPORTED;
}

ConversionType conversion_type_for_update(eGPUDataFormat host_format,
                                          eGPUTextureFormat device_format)
{
  BLI_assert(validate_data_format(device_format, host_format));

  switch (host_format) {
    case GPU_DATA_FLOAT:
      return type_of_conversion_float(device_format);
    case GPU_DATA_UINT:
      return type_of_conversion_uint(device_format);
    case GPU_DATA_INT:
      return type_of_conversion_int(device_format);
    case GPU_DATA_HALF_FLOAT:
      return type_of_conversion_half(device_format);
    case GPU_DATA_UBYTE:
      return type_of_conversion_ubyte(device_format);

    case GPU_DATA_UINT_24_8:
    case GPU_DATA_10_11_11_REV:
    case GPU_DATA_2_10_10_10_REV:
      return ConversionType::UNSUPPORTED;
  }

  return ConversionType::UNSUPPORTED;
}

static ConversionType invert(ConversionType type)
{
#define CASE_SINGLE(a, b) \
  case ConversionType::a##_TO_##b: \
    return ConversionType::b##_TO_##a;

#define CASE_PAIR(a, b) \
  CASE_SINGLE(a, b) \
  CASE_SINGLE(b, a)

  switch (type) {
    case ConversionType::PASS_THROUGH:
      return ConversionType::PASS_THROUGH;

      CASE_PAIR(FLOAT, UNORM8)
      CASE_PAIR(FLOAT, SNORM8)
      CASE_PAIR(FLOAT, UNORM16)
      CASE_PAIR(FLOAT, SNORM16)
      CASE_PAIR(UI32, UI16)
      CASE_PAIR(I32, I16)
      CASE_PAIR(UI32, UI8)
      CASE_PAIR(I32, I8)
      CASE_PAIR(FLOAT, HALF)

    case ConversionType::UNSUPPORTED:
      return ConversionType::UNSUPPORTED;
  }

#undef CASE_PAIR
#undef CASE_SINGLE

  return ConversionType::UNSUPPORTED;
}

ConversionType conversion_type_for_read(eGPUDataFormat host_format,
                                        eGPUTextureFormat device_format)
{
  return invert(conversion_type_for_update(host_format, device_format));
}

/* Copy the contents of src to dst with out performing any actual conversion. */
template<typename DestinationType, typename SourceType>
void copy_unchecked(MutableSpan<DestinationType> dst, Span<SourceType> src)
{
  BLI_assert(src.size() == dst.size());
  for (int64_t index : IndexRange(src.size())) {
    dst[index] = src[index];
  }
}

template<typename DestinationType, typename SourceType>
void copy_unchecked(void *dst_memory,
                    const void *src_memory,
                    eGPUTextureFormat device_format,
                    size_t sample_len)
{
  size_t total_components = to_component_len(device_format) * sample_len;
  Span<SourceType> src = Span<SourceType>(static_cast<const SourceType *>(src_memory),
                                          total_components);
  MutableSpan<DestinationType> dst = MutableSpan<DestinationType>(
      static_cast<DestinationType *>(dst_memory), total_components);
  copy_unchecked<DestinationType, SourceType>(dst, src);
}

/* Float <=> unsigned normalized */
template<typename Type> constexpr int32_t unorm_scalar()
{
  return ((1 << (sizeof(Type) * 8)) - 1);
}
template<typename Type> constexpr int32_t snorm_scalar()
{
  return (1 << (sizeof(Type) * 8 - 1));
}
template<typename Type> constexpr int32_t snorm_max()
{
  return ((1 << (sizeof(Type) * 8)) - 1);
}
template<typename Type> constexpr int32_t snorm_delta()
{
  return (1 << (sizeof(Type) * 8 - 1)) - 1;
}

template<typename DestinationType, typename SourceType>
static DestinationType to_unorm(SourceType value)
{
  static constexpr int32_t Multiplier = unorm_scalar<DestinationType>();
  static constexpr int32_t Max = Multiplier;

  int32_t before_clamping = value * Multiplier;
  return clamp_i(before_clamping, 0, Max);
}

template<typename DestinationType, typename SourceType>
static DestinationType from_unorm(SourceType value)
{
  static constexpr int32_t Multiplier = unorm_scalar<SourceType>();
  return DestinationType(value) / Multiplier;
}

template<typename DestinationType, typename SourceType>
void float_to_unorm(MutableSpan<DestinationType> dst, Span<SourceType> src)
{
  BLI_assert(src.size() == dst.size());
  for (int64_t index : IndexRange(src.size())) {
    dst[index] = to_unorm<DestinationType, SourceType>(src[index]);
  }
}

template<typename DestinationType, typename SourceType>
void float_to_unorm(void *dst_memory,
                    const void *src_memory,
                    eGPUTextureFormat device_format,
                    size_t sample_len)
{
  size_t total_components = to_component_len(device_format) * sample_len;
  Span<SourceType> src = Span<SourceType>(static_cast<const SourceType *>(src_memory),
                                          total_components);
  MutableSpan<DestinationType> dst = MutableSpan<DestinationType>(
      static_cast<DestinationType *>(dst_memory), total_components);
  float_to_unorm<DestinationType, SourceType>(dst, src);
}

template<typename DestinationType, typename SourceType>
void unorm_to_float(MutableSpan<DestinationType> dst, Span<SourceType> src)
{
  BLI_assert(src.size() == dst.size());
  for (int64_t index : IndexRange(src.size())) {
    dst[index] = from_unorm<DestinationType, SourceType>(src[index]);
  }
}

template<typename DestinationType, typename SourceType>
void unorm_to_float(void *dst_memory,
                    const void *src_memory,
                    eGPUTextureFormat device_format,
                    size_t sample_len)
{
  size_t total_components = to_component_len(device_format) * sample_len;
  Span<SourceType> src = Span<SourceType>(static_cast<const SourceType *>(src_memory),
                                          total_components);
  MutableSpan<DestinationType> dst = MutableSpan<DestinationType>(
      static_cast<DestinationType *>(dst_memory), total_components);
  unorm_to_float<DestinationType, SourceType>(dst, src);
}

/* Float <=> signed normalized */

/* TODO: SNORM needs to be shifted...*/
template<typename DestinationType, typename SourceType>
static DestinationType to_snorm(SourceType value)
{
  static constexpr int32_t Multiplier = snorm_scalar<DestinationType>();
  static constexpr int32_t Max = snorm_max<DestinationType>();
  static constexpr int32_t Delta = snorm_delta<DestinationType>();
  return (clamp_i((value * Multiplier + Delta), 0, Max));
}

template<typename DestinationType, typename SourceType>
static DestinationType from_snorm(SourceType value)
{
  static constexpr int32_t Multiplier = snorm_scalar<SourceType>();
  static constexpr int32_t Delta = snorm_delta<SourceType>();
  return DestinationType(int32_t(value) - Delta) / Multiplier;
}

template<typename DestinationType, typename SourceType>
void float_to_snorm(MutableSpan<DestinationType> dst, Span<SourceType> src)
{
  BLI_assert(src.size() == dst.size());
  for (int64_t index : IndexRange(src.size())) {
    const SourceType src_value = src[index];
    const DestinationType dst_value = to_snorm<DestinationType, SourceType>(src_value);
    dst[index] = dst_value;
  }
}

template<typename DestinationType, typename SourceType>
void float_to_snorm(void *dst_memory,
                    const void *src_memory,
                    eGPUTextureFormat device_format,
                    size_t sample_len)
{
  size_t total_components = to_component_len(device_format) * sample_len;
  Span<SourceType> src = Span<SourceType>(static_cast<const SourceType *>(src_memory),
                                          total_components);
  MutableSpan<DestinationType> dst = MutableSpan<DestinationType>(
      static_cast<DestinationType *>(dst_memory), total_components);
  float_to_snorm<DestinationType, SourceType>(dst, src);
}

template<typename DestinationType, typename SourceType>
void snorm_to_float(MutableSpan<DestinationType> dst, Span<SourceType> src)
{
  BLI_assert(src.size() == dst.size());
  for (int64_t index : IndexRange(src.size())) {
    const SourceType src_value = src[index];
    const DestinationType dst_value = from_snorm<DestinationType, SourceType>(src_value);
    dst[index] = dst_value;
  }
}

template<typename DestinationType, typename SourceType>
void snorm_to_float(void *dst_memory,
                    const void *src_memory,
                    eGPUTextureFormat device_format,
                    size_t sample_len)
{
  size_t total_components = to_component_len(device_format) * sample_len;
  Span<SourceType> src = Span<SourceType>(static_cast<const SourceType *>(src_memory),
                                          total_components);
  MutableSpan<DestinationType> dst = MutableSpan<DestinationType>(
      static_cast<DestinationType *>(dst_memory), total_components);
  snorm_to_float<DestinationType, SourceType>(dst, src);
}

void convert(ConversionType type,
             eGPUTextureFormat device_format,
             size_t sample_len,
             void *dst_memory,
             const void *src_memory)
{
  switch (type) {
    case ConversionType::UNSUPPORTED:
      return;

    case ConversionType::PASS_THROUGH:
      memcpy(dst_memory, src_memory, sample_len * to_bytesize(device_format));
      return;

    case ConversionType::UI32_TO_UI16:
      copy_unchecked<uint16_t, uint32_t>(dst_memory, src_memory, device_format, sample_len);
      break;

    case ConversionType::UI16_TO_UI32:
      copy_unchecked<uint32_t, uint16_t>(dst_memory, src_memory, device_format, sample_len);
      break;

    case ConversionType::UI32_TO_UI8:
      copy_unchecked<uint8_t, uint32_t>(dst_memory, src_memory, device_format, sample_len);
      break;

    case ConversionType::UI8_TO_UI32:
      copy_unchecked<uint32_t, uint8_t>(dst_memory, src_memory, device_format, sample_len);
      break;

    case ConversionType::I32_TO_I16:
      copy_unchecked<int16_t, int32_t>(dst_memory, src_memory, device_format, sample_len);
      break;

    case ConversionType::I16_TO_I32:
      copy_unchecked<int32_t, int16_t>(dst_memory, src_memory, device_format, sample_len);
      break;

    case ConversionType::I32_TO_I8:
      copy_unchecked<int8_t, int32_t>(dst_memory, src_memory, device_format, sample_len);
      break;

    case ConversionType::I8_TO_I32:
      copy_unchecked<int32_t, int8_t>(dst_memory, src_memory, device_format, sample_len);
      break;

    case ConversionType::FLOAT_TO_UNORM8:
      float_to_unorm<uint8_t, float>(dst_memory, src_memory, device_format, sample_len);
      break;
    case ConversionType::UNORM8_TO_FLOAT:
      unorm_to_float<float, uint8_t>(dst_memory, src_memory, device_format, sample_len);
      break;

    case ConversionType::FLOAT_TO_SNORM8:
      float_to_snorm<uint8_t, float>(dst_memory, src_memory, device_format, sample_len);
      break;
    case ConversionType::SNORM8_TO_FLOAT:
      snorm_to_float<float, uint8_t>(dst_memory, src_memory, device_format, sample_len);
      break;

    case ConversionType::FLOAT_TO_UNORM16:
      float_to_unorm<uint16_t, float>(dst_memory, src_memory, device_format, sample_len);
      break;
    case ConversionType::UNORM16_TO_FLOAT:
      unorm_to_float<float, uint16_t>(dst_memory, src_memory, device_format, sample_len);
      break;

    case ConversionType::FLOAT_TO_SNORM16:
      float_to_snorm<uint16_t, float>(dst_memory, src_memory, device_format, sample_len);
      break;
    case ConversionType::SNORM16_TO_FLOAT:
      snorm_to_float<float, uint16_t>(dst_memory, src_memory, device_format, sample_len);
      break;

    case ConversionType::FLOAT_TO_HALF:
    case ConversionType::HALF_TO_FLOAT:
      BLI_assert_unreachable();
      return;
  }
}

}  // namespace blender::gpu
