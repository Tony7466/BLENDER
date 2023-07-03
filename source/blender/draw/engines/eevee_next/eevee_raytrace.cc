/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation.
 */

/** \file
 * \ingroup eevee
 *
 * The raytracing module class handles ray generation, scheduling, tracing and denoising.
 */

#include <fstream>
#include <iostream>

#include "BKE_global.h"

#include "eevee_instance.hh"

#include "eevee_raytrace.hh"

namespace blender::eevee {

/* -------------------------------------------------------------------- */
/** \name Raytracing
 *
 * \{ */

void RaytracingModule::init()
{
  enabled_ = true;

  const int flag = inst_.scene->eevee.flag;

  use_spatial_denoise_ = (flag & SCE_EEVEE_RAYTRACE_DENOISE);
  use_temporal_denoise_ = use_spatial_denoise_ && (flag & SCE_EEVEE_RAYTRACE_DENOISE_TEMPORAL);
  use_bilateral_denoise_ = use_temporal_denoise_ && (flag & SCE_EEVEE_RAYTRACE_DENOISE_BILATERAL);

  if (!U.experimental.use_eevee_debug) {
    /* These cannot be turned off separately when not in debug mode. */
    use_temporal_denoise_ = use_bilateral_denoise_ = use_spatial_denoise_;
  }

  data_.valid_history_reflection = 1;
  data_.resolution_scale = 1;
  data_.resolution_bias = int2(0);
  data_.thickness = 1.0f;
  data_.quality = 1.0f;
  data_.brightness_clamp = 1.0f;
  data_.max_roughness = 1.0f;
  data_.pool_offset = 0;
}

void RaytracingModule::sync()
{
  /* Setup. */
  {
    PassSimple &pass = tile_classify_ps_;
    pass.init();
    pass.shader_set(inst_.shaders.static_shader_get(RAY_TILE_CLASSIFY));
    pass.bind_texture("gbuffer_closure_tx", &inst_.gbuffer.closure_tx);
    pass.bind_texture("stencil_tx", &renderbuf_stencil_view_);
    pass.bind_image("tile_mask_img", &tile_mask_tx_);
    pass.bind_ssbo("ray_dispatch_buf", &ray_dispatch_buf_);
    pass.bind_ssbo("ray_tiles_buf", &ray_tiles_buf_);
    pass.push_constant("closure_active", &closure_active_);
    pass.dispatch(&tile_dispatch_size_);
    pass.barrier(GPU_BARRIER_TEXTURE_FETCH);
  }
  for (auto type : IndexRange(2)) {
    PassSimple &pass = (type == 0) ? generate_reflect_ps_ : generate_refract_ps_;
    pass.init();
    pass.shader_set(inst_.shaders.static_shader_get((type == 0) ? RAY_GENERATE_REFLECT :
                                                                  RAY_GENERATE_REFRACT));
    pass.bind_texture(RBUFS_UTILITY_TEX_SLOT, inst_.pipelines.utility_tx);
    pass.bind_texture("stencil_tx", &renderbuf_stencil_view_);
    pass.bind_texture("gbuffer_closure_tx", &inst_.gbuffer.closure_tx);
    pass.bind_texture("gbuffer_color_tx", &inst_.gbuffer.color_tx);
    pass.bind_image("out_ray_data_img", &ray_data_tx_);
    pass.bind_ssbo("tiles_coord_buf", &ray_tiles_buf_);
    inst_.sampling.bind_resources(&pass);
    pass.dispatch(ray_dispatch_buf_);
    pass.barrier(GPU_BARRIER_SHADER_STORAGE | GPU_BARRIER_TEXTURE_FETCH |
                 GPU_BARRIER_SHADER_IMAGE_ACCESS);
  }
  /* Tracing. */
  for (auto type : IndexRange(2)) {
    PassSimple &pass = (type == 0) ? trace_reflect_ps_ : trace_refract_ps_;
    pass.init();
    pass.shader_set(inst_.shaders.static_shader_get((type == 0) ? RAY_TRACE_SCREEN_REFLECT :
                                                                  RAY_TRACE_SCREEN_REFRACT));
    pass.bind_ssbo("tiles_coord_buf", &ray_tiles_buf_);
    pass.bind_image("ray_data_img", &ray_data_tx_);
    pass.bind_image("ray_time_img", &ray_time_tx_);
    pass.bind_image("ray_radiance_img", &ray_radiance_tx_);
    pass.bind_texture("depth_tx", &renderbuf_depth_view_);
    pass.bind_ubo("raytrace_buf", &data_);
    inst_.hiz_buffer.bind_resources(&pass);
    inst_.sampling.bind_resources(&pass);
    inst_.reflection_probes.bind_resources(&pass);
    pass.dispatch(ray_dispatch_buf_);
    pass.barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS);
  }
  /* Denoise. */
  for (auto type : IndexRange(2)) {
    PassSimple &pass = (type == 0) ? denoise_spatial_reflect_ps_ : denoise_spatial_refract_ps_;
    pass.init();
    pass.shader_set(inst_.shaders.static_shader_get((type == 0) ? RAY_DENOISE_SPATIAL_REFLECT :
                                                                  RAY_DENOISE_SPATIAL_REFRACT));
    pass.bind_ssbo("tiles_coord_buf", &ray_tiles_buf_);
    pass.bind_texture("depth_tx", &renderbuf_depth_view_);
    pass.bind_texture("gbuffer_closure_tx", &inst_.gbuffer.closure_tx);
    pass.bind_image("ray_data_img", &ray_data_tx_);
    pass.bind_image("ray_time_img", &ray_time_tx_);
    pass.bind_image("ray_radiance_img", &ray_radiance_tx_);
    pass.bind_image("out_radiance_img", &out_radiance_tx_);
    pass.bind_ubo("raytrace_buf", &data_);
    inst_.sampling.bind_resources(&pass);
    pass.dispatch(ray_dispatch_buf_);
    pass.barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS);
  }
#if 0
  {
    PassSimple &pass = denoise_temporal_ps_;
    pass.init();
    pass.shader_set(inst_.shaders.static_shader_get(RAY_DENOISE_TEMPORAL));
    pass.bind_ssbo("tiles_coord_buf", &ray_tiles_buf_);
    pass.bind_texture("depth_tx", &renderbuf_depth_view_);
    pass.bind_texture("gbuffer_closure_tx", &inst_.gbuffer.closure_tx);
    pass.bind_image("in_ray_radiance_img", &denoised_spatial_tx_);
    pass.bind_image("out_ray_radiance_img", &out_radiance_tx_);
    inst_.sampling.bind_resources(&pass);
    pass.dispatch(ray_dispatch_buf_);
    pass.barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS);
  }
  {
    PassSimple &pass = denoise_bilateral_ps_;
    pass.init();
    pass.shader_set(inst_.shaders.static_shader_get(RAY_DENOISE_BILATERAL));
    pass.bind_ssbo("tiles_coord_buf", &ray_tiles_buf_);
    pass.bind_texture("depth_tx", &renderbuf_depth_view_);
    pass.bind_texture("gbuffer_closure_tx", &inst_.gbuffer.closure_tx);
    pass.bind_image("in_ray_radiance_img", &denoised_temporal_tx_);
    pass.bind_image("out_ray_radiance_img", &out_radiance_tx_);
    inst_.sampling.bind_resources(&pass);
    pass.dispatch(ray_dispatch_buf_);
    pass.barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS);
  }
#endif
}

void RaytracingModule::debug_pass_sync() {}

void RaytracingModule::debug_draw(View & /* view */, GPUFrameBuffer * /* view_fb */) {}

void RaytracingModule::trace(eClosureBits closure_bit, GPUTexture *out_radiance_tx, View &view)
{
  if (closure_bit == 0) {
    return;
  }
  BLI_assert_msg(count_bits_i(closure_bit) == 1,
                 "Only one closure type can be raytraced at a time.");
  BLI_assert_msg(closure_bit == (closure_bit & (CLOSURE_REFLECTION | CLOSURE_REFRACTION)),
                 "Only reflection and refraction are implemented.");

  int2 extent(GPU_texture_width(out_radiance_tx), GPU_texture_height(out_radiance_tx));

  DRW_stats_group_start("Raytracing");

  closure_active_ = closure_bit;

  tile_dispatch_size_ = int3(math::divide_ceil(extent, int2(RAYTRACE_GROUP_SIZE)), 1);
  const int tile_count = tile_dispatch_size_.x * tile_dispatch_size_.y;

  renderbuf_stencil_view_ = inst_.render_buffers.depth_tx.stencil_view();
  renderbuf_depth_view_ = inst_.render_buffers.depth_tx;

  /* TODO(fclem): Half-Res tracing. */
  int2 tracing_res = extent;

  data_.full_resolution = extent;
  data_.full_resolution_inv = 1.0f / float2(extent);
  data_.skip_denoise = !use_spatial_denoise_;
  data_.push_update();

  tile_mask_tx_.acquire(tile_dispatch_size_.xy(), GPU_R32UI);
  ray_tiles_buf_.resize(ceil_to_multiple_u(tile_count, 512));

  /* Ray setup. */
  GPU_storagebuf_clear_to_zero(ray_dispatch_buf_);
  inst_.manager->submit(tile_classify_ps_, view);

  /* Tracing rays. */
  ray_data_tx_.acquire(tracing_res, GPU_RGBA16F);
  ray_time_tx_.acquire(tracing_res, GPU_R32F);
  ray_radiance_tx_.acquire(tracing_res, GPU_RGBA16F);

  if (closure_bit == CLOSURE_REFLECTION) {
    inst_.manager->submit(generate_reflect_ps_, view);
    inst_.manager->submit(trace_reflect_ps_, view);
  }
  else if (closure_bit == CLOSURE_REFRACTION) {
    inst_.manager->submit(generate_refract_ps_, view);
    inst_.manager->submit(trace_refract_ps_, view);
  }

  {
    /* Denoise spatial. */
    denoised_spatial_tx_.acquire(extent, GPU_RGBA16F);
    out_radiance_tx_ = !use_temporal_denoise_ ? out_radiance_tx : denoised_spatial_tx_;

    if (closure_bit == CLOSURE_REFLECTION) {
      inst_.manager->submit(denoise_spatial_reflect_ps_, view);
    }
    else if (closure_bit == CLOSURE_REFRACTION) {
      inst_.manager->submit(denoise_spatial_refract_ps_, view);
    }

    ray_data_tx_.release();
    ray_time_tx_.release();
    ray_radiance_tx_.release();
    tile_mask_tx_.release();
  }

  if (!use_temporal_denoise_) {
    denoised_spatial_tx_.release();
    DRW_stats_group_end();
    return;
  }

  {
    /* Denoise temporal. */
    denoised_temporal_tx_.acquire(extent, GPU_RGBA16F);
    out_radiance_tx_ = !use_bilateral_denoise_ ? out_radiance_tx : denoised_spatial_tx_;

    inst_.manager->submit(denoise_temporal_ps_, view);

    denoised_spatial_tx_.release();
  }

  if (!use_bilateral_denoise_) {
    denoised_temporal_tx_.release();
    DRW_stats_group_end();
    return;
  }

  {
    /* Denoise bilateral. */
    out_radiance_tx_ = out_radiance_tx;

    inst_.manager->submit(denoise_bilateral_ps_, view);

    denoised_temporal_tx_.release();
  }

  DRW_stats_group_end();
}

/** \} */

}  // namespace blender::eevee
