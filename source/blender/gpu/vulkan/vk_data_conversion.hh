/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "BLI_math_vector_types.hh"

#include "gpu_texture_private.hh"

namespace blender::gpu {

/**
 * Convert host buffer to device buffer.
 *
 * \param dst_buffer: device buffer.
 * \param src_buffer: host buffer.
 * \param buffer_size: number of pixels to convert from the start of the given buffer.
 * \param host_format: format of the host buffer.
 * \param device_format: format of the device buffer.
 *
 * \note Will assert when the host_format/device_format combination isn't valid
 * (#validate_data_format) or supported. Some combinations aren't supported in Vulkan due to
 * platform incompatibility.
 */
void convert_host_to_device(void *dst_buffer,
                            const void *src_buffer,
                            size_t buffer_size,
                            eGPUDataFormat host_format,
                            eGPUTextureFormat device_format);

/**
 * Convert host buffer to device buffer with row length.
 *
 * \param dst_buffer: device buffer.
 * \param src_buffer: host buffer.
 * \param src_size: size of the host buffer.
 * \param src_row_length: Length of a single row of the buffer (in pixels).
 * \param host_format: format of the host buffer.
 * \param device_format: format of the device buffer.
 *
 * \note Will assert when the host_format/device_format combination isn't valid
 * (#validate_data_format) or supported. Some combinations aren't supported in Vulkan due to
 * platform incompatibility.
 */
void convert_host_to_device(void *dst_buffer,
                            const void *src_buffer,
                            uint2 src_size,
                            uint src_row_length,
                            eGPUDataFormat host_format,
                            eGPUTextureFormat device_format);

/**
 * Convert device buffer to host buffer.
 *
 * \param dst_buffer: host buffer
 * \param src_buffer: device buffer.
 * \param buffer_size: number of pixels to convert from the start of the given buffer.
 * \param host_format: format of the host buffer
 * \param device_format: format of the device buffer.
 *
 * \note Will assert when the host_format/device_format combination isn't valid
 * (#validate_data_format) or supported. Some combinations aren't supported in Vulkan due to
 * platform incompatibility.
 */
void convert_device_to_host(void *dst_buffer,
                            const void *src_buffer,
                            size_t buffer_size,
                            eGPUDataFormat host_format,
                            eGPUTextureFormat device_format);

/**
 * Are all attributes of the given vertex format natively supported or does conversion needs to
 * happen.
 *
 * \param vertex_format: the vertex format to check if an associated buffer requires conversion
 *                       being done on the host.
 */
bool conversion_needed(const GPUVertFormat &vertex_format);

/**
 * Convert the given `data` to contain Vulkan natively supported data formats.
 *
 * When for an vertex attribute the fetch mode is set to GPU_FETCH_INT_TO_FLOAT and the attribute
 * is an int32_t or uint32_t the conversion will be done. Attributes of 16 or 8 bits are supported
 * natively and will be done in Vulkan.
 *
 * \param data: Buffer to convert. Data will be converted in place.
 * \param vertex_format: Vertex format of the given data. Attributes that aren't supported will be
 *        converted to a supported one.
 *  \param vertex_len: Number of vertices of the given data buffer;
 *        The number of vertices to convert.
 */
void convert_in_place(void *data, const GPUVertFormat &vertex_format, const uint vertex_len);

/* -------------------------------------------------------------------- */
/** \name Floating point conversions
 * \{ */

template<bool HasSignBit, uint8_t MantissaBitLen, uint8_t ExponentBitLen>
class FloatingPointFormat {
 public:
  static constexpr bool HasSign = HasSignBit;
  static constexpr uint8_t MantissaLen = MantissaBitLen;
  static constexpr uint8_t MantissaShift = 0;
  static constexpr uint32_t MantissaMask = (1 << MantissaBitLen) - 1;
  static constexpr uint8_t ExponentShift = MantissaBitLen;
  static constexpr uint8_t ExponentLen = ExponentBitLen;
  static constexpr uint32_t ExponentMask = (1 << ExponentBitLen) - 1;
  static constexpr uint8_t SignShift = MantissaBitLen + ExponentBitLen;
  static constexpr uint32_t SignMask = HasSignBit ? 1 : 0;
  static constexpr uint32_t ExponentBias = (1 << (ExponentBitLen - 1)) - 1;

  uint32_t get_mantissa(uint32_t floating_point_number)
  {
    return (floating_point_number >> MantissaShift) & MantissaMask;
  }
  uint32_t clear_mantissa(uint32_t floating_point_number)
  {
    return floating_point_number & ~(MantissaMask << MantissaShift);
  }
  uint32_t set_mantissa(uint32_t mantissa, uint32_t floating_point_number)
  {
    uint32_t result = clear_mantissa(floating_point_number);
    result |= mantissa << MantissaShift;
    return result;
  }

  uint32_t get_exponent(uint32_t floating_point_number)
  {
    return ((floating_point_number >> ExponentShift) & ExponentMask);
  }
  uint32_t clear_exponent(uint32_t floating_point_number)
  {
    return floating_point_number & ~(ExponentMask << ExponentShift);
  }
  uint32_t set_exponent(uint32_t exponent, uint32_t floating_point_number)
  {
    uint32_t result = clear_exponent(floating_point_number);
    result |= (exponent) << ExponentShift;
    return result;
  }

  bool is_signed(uint32_t floating_point_number)
  {
    if constexpr (HasSignBit) {
      return (floating_point_number >> SignShift) & SignMask;
    }
    return false;
  }
  uint32_t clear_sign(uint32_t floating_point_number)
  {
    return floating_point_number & ~(1 << SignShift);
  }

  uint32_t set_sign(bool sign, uint32_t floating_point_number)
  {
    if constexpr (HasSignBit) {
      return floating_point_number;
    }
    uint32_t result = clear_sign(floating_point_number);
    result |= uint32_t(sign) << SignShift;
    return result;
  }
};

using Format32F = FloatingPointFormat<true, 23, 8>;
using Format16F = FloatingPointFormat<true, 10, 5>;
using Format11F = FloatingPointFormat<false, 6, 5>;
using Format10F = FloatingPointFormat<false, 5, 5>;

template<typename DestinationFormat, typename SourceFormat>
uint32_t convert_float_formats(uint32_t value)
{
  SourceFormat src_format;
  DestinationFormat dst_format;
  /*
  printf("Source MS:%d MM:%x ES:%d EM:%x EB:%x\n",
         SourceFormat::MantissaShift,
         SourceFormat::MantissaMask,
         SourceFormat::ExponentShift,
         SourceFormat::ExponentMask,
         SourceFormat::ExponentBias);
  printf("Destination MS:%d MM:%x ES:%d EM:%x EB:%x\n",
         DestinationFormat::MantissaShift,
         DestinationFormat::MantissaMask,
         DestinationFormat::ExponentShift,
         DestinationFormat::ExponentMask,
         DestinationFormat::ExponentBias);
         */

  bool is_signed = src_format.is_signed(value);
  uint32_t mantissa = src_format.get_mantissa(value);
  int32_t exponent = src_format.get_exponent(value);
  printf("src:%x S:%d, M:%x E:%x\n", value, is_signed, mantissa, exponent);
  /* Sign conversion */
  if (is_signed && !DestinationFormat::HasSign) {
    // NOTE: we clamp to zero.
    return 0;
  }

  /* Mantissa conversion */
  if constexpr (SourceFormat::MantissaLen > DestinationFormat::MantissaLen) {
    mantissa = mantissa >> (SourceFormat::MantissaLen - DestinationFormat::MantissaLen);
  }
  else if constexpr (SourceFormat::MantissaLen < DestinationFormat::MantissaLen) {
    mantissa = mantissa << (DestinationFormat::MantissaLen - SourceFormat::MantissaLen);
  }

  /* Exponent conversion */
  const bool is_denormalized = exponent == 0;
  if (!is_denormalized) {
    exponent -= SourceFormat::ExponentBias;
    /*
    if constexpr (SourceFormat::ExponentLen > DestinationFormat::ExponentLen) {
      exponent = exponent >> (SourceFormat::ExponentLen - DestinationFormat::ExponentLen);
    }
    else if constexpr (SourceFormat::ExponentLen < DestinationFormat::ExponentLen) {
      exponent = exponent << (DestinationFormat::ExponentLen - SourceFormat::ExponentLen);
    }
    */
    exponent += DestinationFormat::ExponentBias;
  }

  uint32_t result = 0;
  result = dst_format.set_sign(is_signed, result);
  result = dst_format.set_exponent(exponent, result);
  result = dst_format.set_mantissa(mantissa, result);
  printf("dst:%x S:%d, M:%x E:%x\n", result, is_signed, mantissa, exponent);
  return result;
}

/* \} */

};  // namespace blender::gpu
