/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

ccl_device_inline void kernel_gpu_film_convert_half_write(ccl_global uchar4 *rgba,
                                                          const int rgba_offset,
                                                          const int rgba_stride,
                                                          const int x,
                                                          const int y,
                                                          const half4 half_pixel)
{
  /* Work around HIP issue with half float display, see #92972. */
#ifdef __KERNEL_HIP__
  ccl_global half *out = ((ccl_global half *)rgba) + (rgba_offset + y * rgba_stride + x) * 4;
  out[0] = half_pixel.x;
  out[1] = half_pixel.y;
  out[2] = half_pixel.z;
  out[3] = half_pixel.w;
#else
  ccl_global half4 *out = ((ccl_global half4 *)rgba) + rgba_offset + y * rgba_stride + x;
  *out = half_pixel;
#endif
}

#ifdef __KERNEL_METAL__

/* Fetch into a local variable on Metal - there is minimal overhead. Templating the
 * film_get_pass_pixel_... functions works on MSL, but not on other compilers. */
#  define FILM_GET_PASS_PIXEL_F32(variant, input_channel_count) \
    float local_pixel[4]; \
    film_get_pass_pixel_##variant(&kfilm_convert, buffer, local_pixel); \
    if (input_channel_count >= 1) { \
      pixel[0] = local_pixel[0]; \
    } \
    if (input_channel_count >= 2) { \
      pixel[1] = local_pixel[1]; \
    } \
    if (input_channel_count >= 3) { \
      pixel[2] = local_pixel[2]; \
    } \
    if (input_channel_count >= 4) { \
      pixel[3] = local_pixel[3]; \
    }

#else

#  define FILM_GET_PASS_PIXEL_F32(variant, input_channel_count) \
    film_get_pass_pixel_##variant(&kfilm_convert, buffer, pixel);

#endif

#define KERNEL_FILM_CONVERT_VARIANT(variant, input_channel_count) \
  ccl_gpu_kernel(GPU_KERNEL_BLOCK_NUM_THREADS, GPU_KERNEL_MAX_REGISTERS) \
      ccl_gpu_kernel_signature(film_convert_##variant, \
                               const KernelFilmConvert kfilm_convert, \
                               ccl_global float *pixels, \
                               ccl_global float *render_buffer, \
                               int num_pixels, \
                               int width, \
                               int offset, \
                               int stride, \
                               int channel_offset, \
                               int rgba_offset, \
                               int rgba_stride) \
  { \
    const int render_pixel_index = ccl_gpu_global_id_x(); \
    if (render_pixel_index >= num_pixels) { \
      return; \
    } \
\
    const int x = render_pixel_index % width; \
    const int y = render_pixel_index / width; \
\
    const uint64_t buffer_pixel_index = x + y * stride; \
    ccl_global const float *buffer = render_buffer + offset + \
                                     buffer_pixel_index * kfilm_convert.pass_stride; \
\
    ccl_global float *pixel = pixels + channel_offset + \
                              (render_pixel_index + rgba_offset) * kfilm_convert.pixel_stride; \
\
    FILM_GET_PASS_PIXEL_F32(variant, input_channel_count); \
  } \
  ccl_gpu_kernel_postfix \
\
  ccl_gpu_kernel(GPU_KERNEL_BLOCK_NUM_THREADS, GPU_KERNEL_MAX_REGISTERS) \
      ccl_gpu_kernel_signature(film_convert_##variant##_half_rgba, \
                               const KernelFilmConvert kfilm_convert, \
                               ccl_global uchar4 *rgba, \
                               ccl_global float *render_buffer, \
                               int num_pixels, \
                               int width, \
                               int offset, \
                               int stride, \
                               int rgba_offset, \
                               int rgba_stride) \
  { \
    const int render_pixel_index = ccl_gpu_global_id_x(); \
    if (render_pixel_index >= num_pixels) { \
      return; \
    } \
\
    const int x = render_pixel_index % width; \
    const int y = render_pixel_index / width; \
\
    const uint64_t buffer_pixel_index = x + y * stride; \
    ccl_global const float *buffer = render_buffer + offset + \
                                     buffer_pixel_index * kfilm_convert.pass_stride; \
\
    float pixel[4]; \
    film_get_pass_pixel_##variant(&kfilm_convert, buffer, pixel); \
\
    if (input_channel_count == 1) { \
      pixel[1] = pixel[2] = pixel[0]; \
    } \
    if (input_channel_count <= 3) { \
      pixel[3] = 1.0f; \
    } \
\
    film_apply_pass_pixel_overlays_rgba(&kfilm_convert, buffer, pixel); \
\
    const half4 half_pixel = float4_to_half4_display( \
        make_float4(pixel[0], pixel[1], pixel[2], pixel[3])); \
    kernel_gpu_film_convert_half_write(rgba, rgba_offset, rgba_stride, x, y, half_pixel); \
  } \
  ccl_gpu_kernel_postfix

/* 1 channel inputs */
KERNEL_FILM_CONVERT_VARIANT(depth, 1)
KERNEL_FILM_CONVERT_VARIANT(mist, 1)
KERNEL_FILM_CONVERT_VARIANT(sample_count, 1)
KERNEL_FILM_CONVERT_VARIANT(float, 1)

/* 3 channel inputs */
KERNEL_FILM_CONVERT_VARIANT(light_path, 3)
KERNEL_FILM_CONVERT_VARIANT(float3, 3)

/* 4 channel inputs */
KERNEL_FILM_CONVERT_VARIANT(motion, 4)
KERNEL_FILM_CONVERT_VARIANT(cryptomatte, 4)
KERNEL_FILM_CONVERT_VARIANT(shadow_catcher, 4)
KERNEL_FILM_CONVERT_VARIANT(shadow_catcher_matte_with_shadow, 4)
KERNEL_FILM_CONVERT_VARIANT(combined, 4)
KERNEL_FILM_CONVERT_VARIANT(float4, 4)

#undef KERNEL_FILM_CONVERT_VARIANT
