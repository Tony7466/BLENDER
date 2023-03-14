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
      return ConversionType::UNMODIFIED;

    case GPU_RGBA16F:
    case GPU_RG16F:
    case GPU_R16F:
    case GPU_RGB16F:
      return ConversionType::FLOAT_TO_HALF;

    case GPU_RGB32F: /* GPU_RGB32F Not supported by vendors. */
    case GPU_RGBA8UI:
    case GPU_RGBA8I:
    case GPU_RGBA8:
    case GPU_RGBA16UI:
    case GPU_RGBA16I:
    case GPU_RGBA16:
    case GPU_RGBA32UI:
    case GPU_RGBA32I:
    case GPU_RG8UI:
    case GPU_RG8I:
    case GPU_RG8:
    case GPU_RG16UI:
    case GPU_RG16I:
    case GPU_RG16:
    case GPU_RG32UI:
    case GPU_RG32I:
    case GPU_R8UI:
    case GPU_R8I:
    case GPU_R8:
    case GPU_R16UI:
    case GPU_R16I:
    case GPU_R16:
    case GPU_R32UI:
    case GPU_R32I:
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
    case GPU_RGB16:
    case GPU_RGB16_SNORM:
    case GPU_RGB32UI:
    case GPU_RGB32I:
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
      return ConversionType::UNMODIFIED;

    case GPU_RGBA8UI:
    case GPU_RGBA8I:
    case GPU_RGBA8:
    case GPU_RGBA16UI:
    case GPU_RGBA16I:
    case GPU_RGBA16F:
    case GPU_RGBA16:
    case GPU_RGBA32UI:
    case GPU_RGBA32F:
    case GPU_RG8UI:
    case GPU_RG8I:
    case GPU_RG8:
    case GPU_RG16UI:
    case GPU_RG16I:
    case GPU_RG16F:
    case GPU_RG16:
    case GPU_RG32UI:
    case GPU_RG32F:
    case GPU_R8UI:
    case GPU_R8I:
    case GPU_R8:
    case GPU_R16UI:
    case GPU_R16I:
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
      return ConversionType::UNMODIFIED;

    case GPU_RGBA16UI:
    case GPU_RG16UI:
    case GPU_R16UI:
    case GPU_RGB16UI:
      return ConversionType::UI32_TO_UI16;

    case GPU_RGBA8UI:
    case GPU_RGBA8I:
    case GPU_RGBA8:
    case GPU_RGBA16I:
    case GPU_RGBA16F:
    case GPU_RGBA16:
    case GPU_RGBA32I:
    case GPU_RGBA32F:
    case GPU_RG8UI:
    case GPU_RG8I:
    case GPU_RG8:
    case GPU_RG16I:
    case GPU_RG16F:
    case GPU_RG16:
    case GPU_RG32I:
    case GPU_RG32F:
    case GPU_R8UI:
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
    case GPU_RGBA8UI:
    case GPU_RGBA8I:
    case GPU_RGBA8:
    case GPU_RGBA16UI:
    case GPU_RGBA16I:
    case GPU_RGBA16F:
    case GPU_RGBA16:
    case GPU_RGBA32UI:
    case GPU_RGBA32I:
    case GPU_RGBA32F:
    case GPU_RG8UI:
    case GPU_RG8I:
    case GPU_RG8:
    case GPU_RG16UI:
    case GPU_RG16I:
    case GPU_RG16F:
    case GPU_RG16:
    case GPU_RG32UI:
    case GPU_RG32I:
    case GPU_RG32F:
    case GPU_R8UI:
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
    case GPU_DATA_UINT_24_8:
    case GPU_DATA_10_11_11_REV:
    case GPU_DATA_2_10_10_10_REV:
      return ConversionType::UNSUPPORTED;
  }

  return ConversionType::UNSUPPORTED;
}

static ConversionType invert(ConversionType type)
{
  switch (type) {
    case ConversionType::UNMODIFIED:
      return ConversionType::UNMODIFIED;

    case ConversionType::UI16_TO_UI32:
      return ConversionType::UI32_TO_UI16;
    case ConversionType::UI32_TO_UI16:
      return ConversionType::UI16_TO_UI32;

    case ConversionType::FLOAT_TO_HALF:
      return ConversionType::HALF_TO_FLOAT;
    case ConversionType::HALF_TO_FLOAT:
      return ConversionType::FLOAT_TO_HALF;

    case ConversionType::UNSUPPORTED:
      return ConversionType::UNSUPPORTED;
  }

  return ConversionType::UNSUPPORTED;
}

ConversionType conversion_type_for_read(eGPUDataFormat host_format,
                                        eGPUTextureFormat device_format)
{
  return invert(conversion_type_for_update(host_format, device_format));
}

/* Copy the contents of src to dst with out performing any actual conversion. */
template<typename SourceType, typename DestinationType>
void copy_unchecked(MutableSpan<DestinationType> dst, Span<SourceType> src)
{
  BLI_assert(src.size() == dst.size());
  for (SourceType index : IndexRange(src.size())) {
    dst[index] = src[index];
  }
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

    case ConversionType::UNMODIFIED:
      memcpy(dst_memory, src_memory, sample_len * to_bytesize(device_format));
      return;

    case ConversionType::UI16_TO_UI32: {
      size_t component_len = to_component_len(device_format) * sample_len;
      Span<uint16_t> src = Span<uint16_t>(static_cast<const uint16_t *>(src_memory),
                                          component_len);
      MutableSpan<uint32_t> dst = MutableSpan<uint32_t>(static_cast<uint32_t *>(dst_memory),
                                                        component_len);
      copy_unchecked<uint16_t, uint32_t>(dst, src);
      break;
    }

    case ConversionType::UI32_TO_UI16: {
      size_t component_len = to_component_len(device_format) * sample_len;
      Span<uint32_t> src = Span<uint32_t>(static_cast<const uint32_t *>(src_memory),
                                          component_len);
      MutableSpan<uint16_t> dst = MutableSpan<uint16_t>(static_cast<uint16_t *>(dst_memory),
                                                        component_len);
      copy_unchecked<uint32_t, uint16_t>(dst, src);
      break;
    }

    case ConversionType::FLOAT_TO_HALF:
    case ConversionType::HALF_TO_FLOAT:
      BLI_assert_unreachable();
      return;
  }
}

}  // namespace blender::gpu
