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
}

void RaytracingModule::sync()
{
  raytrace_ps_.init();

  /* TODO(fclem): This is an optimization to have but not necessary for initial release. */
#define TILE_COMPACTION 0
  {
    auto &sub = raytrace_ps_.sub("TileClassify");
    sub.shader_set(inst_.shaders.static_shader_get(RAY_TILE_CLASSIFY));
    sub.bind_texture("gbuffer_closure_tx", &inst_.gbuffer.closure_tx);
    sub.bind_texture("stencil_tx", &renderbuf_stencil_view_);
    sub.bind_image("tile_mask_img", &tile_mask_tx_);
#if TILE_COMPACTION == 0
    sub.bind_ssbo("dispatch_reflect_buf", &dispatch_reflect_buf_);
    sub.bind_ssbo("dispatch_refract_buf", &dispatch_refract_buf_);
    sub.bind_ssbo("tiles_reflect_buf", &tiles_reflect_buf_);
    sub.bind_ssbo("tiles_refract_buf", &tiles_refract_buf_);
#endif
    sub.dispatch(&tile_dispatch_size_);
    sub.barrier(GPU_BARRIER_TEXTURE_FETCH);
  }
#if TILE_COMPACTION == 1
  {
    auto &sub = raytrace_ps_.sub("TileCompaction");
    sub.shader_set(inst_.shaders.static_shader_get(RAY_TILE_COMPACTION));
    sub.bind_texture("tile_mask_img", &tile_mask_tx_);
    sub.dispatch(&tile_dispatch_size_);
    sub.barrier(GPU_BARRIER_SHADER_STORAGE);
  }
#endif
  {
    auto &sub = raytrace_ps_.sub("RayGenerate");
    sub.shader_set(inst_.shaders.static_shader_get(RAY_GENERATE));
    sub.bind_ssbo("tiles_coord_buf", &tiles_reflect_buf_);
    sub.bind_texture(RBUFS_UTILITY_TEX_SLOT, inst_.pipelines.utility_tx);
    sub.bind_texture("stencil_tx", &renderbuf_stencil_view_);
    sub.bind_texture("depth_tx", &renderbuf_depth_view_);
    sub.bind_texture("gbuffer_closure_tx", &inst_.gbuffer.closure_tx);
    sub.bind_texture("gbuffer_color_tx", &inst_.gbuffer.color_tx);
    sub.bind_image("out_ray_data_img", &ray_data_tx_);
    sub.push_constant("active_closure_type", int(CLOSURE_REFLECTION));
    sub.dispatch(dispatch_reflect_buf_);
    sub.barrier(GPU_BARRIER_SHADER_STORAGE | GPU_BARRIER_TEXTURE_FETCH);
  }
#if 0 /* TODO */
  {
    auto &sub = raytrace_ps_.sub("RayScreenTrace");
  }
#endif
}

void RaytracingModule::debug_pass_sync() {}

void RaytracingModule::debug_draw(View &view, GPUFrameBuffer *view_fb) {}

void RaytracingModule::trace(int2 extent, eClosureBits closure_bits, View &view)
{
  tile_dispatch_size_ = int3(math::divide_ceil(extent, int2(RAYTRACE_GROUP_SIZE)), 1);
  const int tile_count = tile_dispatch_size_.x * tile_dispatch_size_.y;

  renderbuf_stencil_view_ = inst_.render_buffers.depth_tx.stencil_view();
  renderbuf_depth_view_ = inst_.render_buffers.depth_tx;

  tile_mask_tx_.acquire(tile_dispatch_size_.xy(), GPU_R32UI);
  tiles_reflect_buf_.resize(ceil_to_multiple_u(tile_count, 512));

  int2 tracing_res = extent;
  ray_data_tx_.ensure_2d(GPU_RGBA16F, tracing_res);
  // ray_data_tx_.acquire(tracing_res, GPU_RGBA16F);

  GPU_storagebuf_clear_to_zero(dispatch_reflect_buf_);

  inst_.manager->submit(raytrace_ps_, view);

  // ray_data_tx_.release();
  tile_mask_tx_.release();
}

/** \} */

}  // namespace blender::eevee
