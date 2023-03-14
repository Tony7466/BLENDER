
/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "gpu_texture_private.hh"

namespace blender::gpu {

enum class ConversionType {
  /** No conversion needed, result can be directly read back to host memory. */
  PASS_THROUGH,

  UI16_TO_UI32,
  UI32_TO_UI16,

  /*
  UI8_TO_UI32,
  I16_TO_I32,
  I8_TO_I32,
  UI8_TO_I32,
  UI8_TO_FLOAT,
  UI8_TO_UBYTE,
  */

  /** Convert device 16F to floats. */
  HALF_TO_FLOAT,
  FLOAT_TO_HALF,

  /**
   * The requested conversion isn't supported.
   */
  UNSUPPORTED,
};

/**
 * Determine the type of conversion that is needed to read back data from GPU device to host
 * memory.
 */
ConversionType conversion_type_for_read(eGPUDataFormat host_format,
                                        eGPUTextureFormat device_format);
ConversionType conversion_type_for_update(eGPUDataFormat host_format,
                                          eGPUTextureFormat device_format);

void convert(ConversionType type,
             eGPUTextureFormat device_format,
             size_t sample_len,
             void *dst_memory,
             const void *src_memory);

};  // namespace blender::gpu
