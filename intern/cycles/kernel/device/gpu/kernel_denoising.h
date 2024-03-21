/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

ccl_gpu_kernel(GPU_KERNEL_BLOCK_NUM_THREADS, GPU_KERNEL_MAX_REGISTERS)
    ccl_gpu_kernel_signature(filter_color_preprocess,
                             ccl_global float *render_buffer,
                             int full_x,
                             int full_y,
                             int width,
                             int height,
                             int offset,
                             int stride,
                             int pass_stride,
                             int pass_denoised)
{
  const int work_index = ccl_gpu_global_id_x();
  const int y = work_index / width;
  const int x = work_index - y * width;

  if (x >= width || y >= height) {
    return;
  }

  const uint64_t render_pixel_index = offset + (x + full_x) + (y + full_y) * stride;
  ccl_global float *buffer = render_buffer + render_pixel_index * pass_stride;

  ccl_global float *color_out = buffer + pass_denoised;
  color_out[0] = clamp(color_out[0], 0.0f, 10000.0f);
  color_out[1] = clamp(color_out[1], 0.0f, 10000.0f);
  color_out[2] = clamp(color_out[2], 0.0f, 10000.0f);
}
ccl_gpu_kernel_postfix

ccl_gpu_kernel(GPU_KERNEL_BLOCK_NUM_THREADS, GPU_KERNEL_MAX_REGISTERS)
    ccl_gpu_kernel_signature(filter_guiding_preprocess,
                             ccl_global float *guiding_buffer,
                             int guiding_pass_stride,
                             int guiding_pass_albedo,
                             int guiding_pass_normal,
                             int guiding_pass_flow,
                             ccl_global const float *render_buffer,
                             int render_offset,
                             int render_stride,
                             int render_pass_stride,
                             int render_pass_sample_count,
                             int render_pass_denoising_albedo,
                             int render_pass_denoising_normal,
                             int render_pass_motion,
                             int full_x,
                             int full_y,
                             int width,
                             int height,
                             int num_samples)
{
  const int work_index = ccl_gpu_global_id_x();
  const int y = work_index / width;
  const int x = work_index - y * width;

  if (x >= width || y >= height) {
    return;
  }

  const uint64_t guiding_pixel_index = x + y * width;
  ccl_global float *guiding_pixel = guiding_buffer + guiding_pixel_index * guiding_pass_stride;

  const uint64_t render_pixel_index = render_offset + (x + full_x) + (y + full_y) * render_stride;
  ccl_global const float *buffer = render_buffer + render_pixel_index * render_pass_stride;

  float pixel_scale;
  if (render_pass_sample_count == PASS_UNUSED) {
    pixel_scale = 1.0f / num_samples;
  }
  else {
    pixel_scale = 1.0f / __float_as_uint(buffer[render_pass_sample_count]);
  }

  /* Albedo pass. */
  if (guiding_pass_albedo != PASS_UNUSED) {
    kernel_assert(render_pass_denoising_albedo != PASS_UNUSED);

    ccl_global const float *aledo_in = buffer + render_pass_denoising_albedo;
    ccl_global float *albedo_out = guiding_pixel + guiding_pass_albedo;

    albedo_out[0] = aledo_in[0] * pixel_scale;
    albedo_out[1] = aledo_in[1] * pixel_scale;
    albedo_out[2] = aledo_in[2] * pixel_scale;
  }

  /* Normal pass. */
  if (guiding_pass_normal != PASS_UNUSED) {
    kernel_assert(render_pass_denoising_normal != PASS_UNUSED);

    ccl_global const float *normal_in = buffer + render_pass_denoising_normal;
    ccl_global float *normal_out = guiding_pixel + guiding_pass_normal;

    normal_out[0] = normal_in[0] * pixel_scale;
    normal_out[1] = normal_in[1] * pixel_scale;
    normal_out[2] = normal_in[2] * pixel_scale;
  }

  /* Flow pass. */
  if (guiding_pass_flow != PASS_UNUSED) {
    kernel_assert(render_pass_motion != PASS_UNUSED);

    ccl_global const float *motion_in = buffer + render_pass_motion;
    ccl_global float *flow_out = guiding_pixel + guiding_pass_flow;

    flow_out[0] = -motion_in[0] * pixel_scale;
    flow_out[1] = -motion_in[1] * pixel_scale;
  }
}
ccl_gpu_kernel_postfix

ccl_gpu_kernel(GPU_KERNEL_BLOCK_NUM_THREADS, GPU_KERNEL_MAX_REGISTERS)
    ccl_gpu_kernel_signature(filter_guiding_set_fake_albedo,
                             ccl_global float *guiding_buffer,
                             int guiding_pass_stride,
                             int guiding_pass_albedo,
                             int width,
                             int height)
{
  kernel_assert(guiding_pass_albedo != PASS_UNUSED);

  const int work_index = ccl_gpu_global_id_x();
  const int y = work_index / width;
  const int x = work_index - y * width;

  if (x >= width || y >= height) {
    return;
  }

  const uint64_t guiding_pixel_index = x + y * width;
  ccl_global float *guiding_pixel = guiding_buffer + guiding_pixel_index * guiding_pass_stride;

  ccl_global float *albedo_out = guiding_pixel + guiding_pass_albedo;

  albedo_out[0] = 0.5f;
  albedo_out[1] = 0.5f;
  albedo_out[2] = 0.5f;
}
ccl_gpu_kernel_postfix

ccl_gpu_kernel(GPU_KERNEL_BLOCK_NUM_THREADS, GPU_KERNEL_MAX_REGISTERS)
    ccl_gpu_kernel_signature(filter_color_postprocess,
                             ccl_global float *render_buffer,
                             int full_x,
                             int full_y,
                             int width,
                             int height,
                             int offset,
                             int stride,
                             int pass_stride,
                             int num_samples,
                             int pass_noisy,
                             int pass_denoised,
                             int pass_sample_count,
                             int num_components,
                             int use_compositing)
{
  const int work_index = ccl_gpu_global_id_x();
  const int y = work_index / width;
  const int x = work_index - y * width;

  if (x >= width || y >= height) {
    return;
  }

  const uint64_t render_pixel_index = offset + (x + full_x) + (y + full_y) * stride;
  ccl_global float *buffer = render_buffer + render_pixel_index * pass_stride;

  float pixel_scale;
  if (pass_sample_count == PASS_UNUSED) {
    pixel_scale = num_samples;
  }
  else {
    pixel_scale = __float_as_uint(buffer[pass_sample_count]);
  }

  ccl_global float *denoised_pixel = buffer + pass_denoised;

  denoised_pixel[0] *= pixel_scale;
  denoised_pixel[1] *= pixel_scale;
  denoised_pixel[2] *= pixel_scale;

  if (num_components == 3) {
    /* Pass without alpha channel. */
  }
  else if (!use_compositing) {
    /* Currently compositing passes are either 3-component (derived by dividing light passes)
     * or do not have transparency (shadow catcher). Implicitly rely on this logic, as it
     * simplifies logic and avoids extra memory allocation. */
    ccl_global const float *noisy_pixel = buffer + pass_noisy;
    denoised_pixel[3] = noisy_pixel[3];
  }
  else {
    /* Assigning to zero since this is a default alpha value for 3-component passes, and it
     * is an opaque pixel for 4 component passes. */
    denoised_pixel[3] = 0;
  }
}
ccl_gpu_kernel_postfix
