/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "device/device.h"

#include "integrator/pass_accessor_cpu.h"

#include "session/buffers.h"

#include "util/log.h"
#include "util/tbb.h"

// clang-format off
#include "kernel/device/cpu/compat.h"
#include "kernel/device/cpu/globals.h"
#include "kernel/types.h"
#include "kernel/film/read.h"
// clang-format on

CCL_NAMESPACE_BEGIN

/* --------------------------------------------------------------------
 * Kernel processing.
 */

inline void PassAccessorCPU::run_get_pass_kernel_processor_float(
    const KernelFilmConvert *kfilm_convert,
    const RenderBuffers *render_buffers,
    const BufferParams &buffer_params,
    const Destination &destination,
    const CPUKernels::FilmConvertFunction func) const
{
  /* NOTE: No overlays are applied since they are not used for final renders.
   * Can be supported via some sort of specialization to avoid code duplication. */

  DCHECK_EQ(destination.stride, 0) << "Custom stride for float destination is not implemented.";

  const int64_t pass_stride = buffer_params.pass_stride;
  const int64_t buffer_row_stride = buffer_params.stride * buffer_params.pass_stride;

  const float *window_data = render_buffers->buffer.data() + buffer_params.window_x * pass_stride +
                             buffer_params.window_y * buffer_row_stride;

  const int pixel_stride = destination.pixel_stride ? destination.pixel_stride :
                                                      destination.num_components;

  /* Calculate how many full plus partial slices there are */
  int slice_height;
  int slice_stride;
  int slices;
  if(buffer_params.slice_height > 0) {
    /* Copy over each slice */
    slices = buffer_params.window_height/buffer_params.slice_height;
    slices += (slices*buffer_params.slice_height < buffer_params.window_height) ? 1 : 0;
    slice_height = buffer_params.slice_height;
    slice_stride = buffer_params.slice_stride;
  }
  else {
    /* Assign each row to a slice */
    slices = buffer_params.window_height;
    slice_height = 1;
    slice_stride = 1;
  }

  /* Copy over each slice to the destination */
  parallel_for(0, slices, [&](int slice) {
  //for(int slice = 0;slice < slices;++slice) {
    int y = slice*slice_height;
    const float *buffer = window_data + y * buffer_row_stride;
    int height = std::min(slice_height, buffer_params.window_height - y);
    int pixel_y = slice * slice_stride;
    float *pixels = destination.pixels +
                    (pixel_y * buffer_params.width + destination.offset) * pixel_stride;
    for (int row = 0; row < height;
         row++, buffer += buffer_row_stride, pixels += buffer_params.width * pixel_stride)
    {
      func(kfilm_convert, buffer, pixels, buffer_params.window_width, pass_stride, pixel_stride);
    }
  });
}

inline void PassAccessorCPU::run_get_pass_kernel_processor_half_rgba(
    const KernelFilmConvert *kfilm_convert,
    const RenderBuffers *render_buffers,
    const BufferParams &buffer_params,
    const Destination &destination,
    const CPUKernels::FilmConvertHalfRGBAFunction func) const
{
  const int64_t pass_stride = buffer_params.pass_stride;
  const int64_t buffer_row_stride = buffer_params.stride * buffer_params.pass_stride;

  const float *window_data = render_buffers->buffer.data() + buffer_params.window_x * pass_stride +
                             buffer_params.window_y * buffer_row_stride;

  half4 *dst_start = destination.pixels_half_rgba + destination.offset;
  const int destination_stride = destination.stride != 0 ? destination.stride :
                                                           buffer_params.width;

  /* Calculate how many full plus partial slices there are */
  int slice_height;
  int slice_stride;
  int slices;
  if(buffer_params.slice_height > 0) {
    /* Copy over each slice */
    slices = buffer_params.window_height/buffer_params.slice_height;
    slices += (slices*buffer_params.slice_height < buffer_params.window_height) ? 1 : 0;
    slice_height = buffer_params.slice_height;
    slice_stride = buffer_params.slice_stride;
  }
  else {
    /* Assign each row to a slice */
    slices = buffer_params.window_height;
    slice_height = 1;
    slice_stride = 1;
  }
  
  parallel_for(0, slices, [&](int slice) {
  //for (int slice = 0; slice < slices; ++slice) {
    int y = slice*slice_height;
    const float *buffer = window_data + y * buffer_row_stride;
    int height = std::min(slice_height, buffer_params.window_height - y);
    int pixel_y = slice * buffer_params.slice_stride;
    half4 *pixels = dst_start + pixel_y * destination_stride;
    for (int row = 0; row < height;
         row++, buffer += buffer_row_stride, pixels += destination_stride) {
      func(kfilm_convert, buffer, pixels, buffer_params.window_width, pass_stride);
    }
  });
}

/* --------------------------------------------------------------------
 * Pass accessors.
 */

#define DEFINE_PASS_ACCESSOR(pass) \
  void PassAccessorCPU::get_pass_##pass(const RenderBuffers *render_buffers, \
                                        const BufferParams &buffer_params, \
                                        const Destination &destination) const \
  { \
    const CPUKernels &kernels = Device::get_cpu_kernels(); \
    KernelFilmConvert kfilm_convert; \
    init_kernel_film_convert(&kfilm_convert, buffer_params, destination); \
\
    if (destination.pixels) { \
      run_get_pass_kernel_processor_float(&kfilm_convert, \
                                          render_buffers, \
                                          buffer_params, \
                                          destination, \
                                          kernels.film_convert_##pass); \
    } \
\
    if (destination.pixels_half_rgba) { \
      run_get_pass_kernel_processor_half_rgba(&kfilm_convert, \
                                              render_buffers, \
                                              buffer_params, \
                                              destination, \
                                              kernels.film_convert_half_rgba_##pass); \
    } \
  }

/* Float (scalar) passes. */
DEFINE_PASS_ACCESSOR(depth)
DEFINE_PASS_ACCESSOR(mist)
DEFINE_PASS_ACCESSOR(sample_count)
DEFINE_PASS_ACCESSOR(float)

/* Float3 passes. */
DEFINE_PASS_ACCESSOR(light_path)
DEFINE_PASS_ACCESSOR(shadow_catcher)
DEFINE_PASS_ACCESSOR(float3)

/* Float4 passes. */
DEFINE_PASS_ACCESSOR(motion)
DEFINE_PASS_ACCESSOR(cryptomatte)
DEFINE_PASS_ACCESSOR(shadow_catcher_matte_with_shadow)
DEFINE_PASS_ACCESSOR(combined)
DEFINE_PASS_ACCESSOR(float4)

#undef DEFINE_PASS_ACCESSOR

CCL_NAMESPACE_END
