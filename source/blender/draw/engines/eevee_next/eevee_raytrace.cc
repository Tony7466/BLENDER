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

  {
    auto sub = raytrace_ps_.sub("TileClassify");
    sub.shader_set(inst_.shaders.static_shader_get(RAY_TILE_CLASSIFY));
    sub.bind_texture("gbuffer_closure_tx", &inst_.gbuffer.closure_tx);
    sub.bind_texture("stencil_tx", inst_.render_buffers.depth_tx.stencil_view());
    sub.bind_image("tile_mask_img", &tile_mask_tx_);
    sub.dispatch(&tile_dispatch_size_);
    sub.barrier(GPU_BARRIER_TEXTURE_FETCH);
  }
}

void RaytracingModule::trace(int2 extent, eClosureBits closure_bits, View &view)
{
  tile_dispatch_size_ = int3(math::divide_ceil(extent, int2(RAYTRACE_GROUP_SIZE)), 1);

  tile_mask_tx_.acquire(tile_dispatch_size_.xy(), GPU_R32UI);

  inst_.manager->submit(raytrace_ps_, view);

  tile_mask_tx_.release();
}

/** \} */

}  // namespace blender::eevee
