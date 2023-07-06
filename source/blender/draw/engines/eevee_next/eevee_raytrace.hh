/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation.
 */

/** \file
 * \ingroup eevee
 *
 * The raytracing module class handles ray generation, scheduling, tracing and denoising.
 */

#pragma once

#include "DRW_render.h"

#include "eevee_shader_shared.hh"

namespace blender::eevee {

class Instance;

/* -------------------------------------------------------------------- */
/** \name Raytracing
 * \{ */

class RaytracingModule {
 private:
  Instance &inst_;

  draw::PassSimple tile_classify_ps_ = {"TileClassify"};
  draw::PassSimple generate_reflect_ps_ = {"RayGenerate.Reflection"};
  draw::PassSimple generate_refract_ps_ = {"RayGenerate.Refraction"};
  draw::PassSimple trace_reflect_ps_ = {"Trace.Reflection"};
  draw::PassSimple trace_refract_ps_ = {"Trace.Refraction"};
  draw::PassSimple denoise_spatial_reflect_ps_ = {"DenoiseSpatial.Reflection"};
  draw::PassSimple denoise_spatial_refract_ps_ = {"DenoiseSpatial.Refraction"};
  draw::PassSimple denoise_temporal_ps_ = {"DenoiseTemporal"};
  draw::PassSimple denoise_bilateral_ps_ = {"DenoiseBilateral"};

  /** Dispatch with enough tiles for the whole screen. */
  int3 tile_dispatch_size_ = int3(1);
  /** 2D tile mask to check which unused adjacent tile we need to clear. */
  TextureFromPool tile_mask_tx_ = {"tile_mask_tx"};
  /** Indirect dispatch rays. Avoid dispatching workgroups that ultimately won't do any tracing. */
  DispatchIndirectBuf ray_dispatch_buf_ = {"ray_dispatch_buf_"};
  /** Tile buffer that contains tile coordinates. */
  RaytraceTileBuf ray_tiles_buf_ = {"ray_tiles_buf_"};
  /** Texture containing the ray direction and pdf. */
  TextureFromPool ray_data_tx_ = {"ray_data_tx"};
  /** Texture containing the ray hit time. */
  TextureFromPool ray_time_tx_ = {"ray_data_tx"};
  /** Texture containing the ray hit radiance (tracing-res). */
  TextureFromPool ray_radiance_tx_ = {"ray_radiance_tx"};
  /** Textures containing the ray hit radiance denoised (full-res). */
  TextureFromPool denoised_spatial_tx_ = {"denoised_spatial_tx_"};
  TextureFromPool denoised_temporal_tx_ = {"denoised_temporal_tx_"};
  /** Ray hit variance and hit depth for temporal denoising. Output of spatial denoise. */
  TextureFromPool hit_variance_tx_ = {"hit_variance_tx_"};
  TextureFromPool hit_depth_tx_ = {"hit_depth_tx_"};
  /** Output of the denoise passes. */
  GPUTexture *out_radiance_tx_ = nullptr;

  /** Pointer to inst_.render_buffers.depth_tx.stencil_view() updated before submission. */
  GPUTexture *renderbuf_stencil_view_ = nullptr;
  /** Pointer to inst_.render_buffers.depth_tx updated before submission. */
  GPUTexture *renderbuf_depth_view_ = nullptr;
  /** Closure being ray-traced. (Is #eClosureBits but is being used as push_constant). */
  int closure_active_;

  bool enabled_ = false;
  bool use_spatial_denoise_ = true;
  bool use_temporal_denoise_ = true;
  bool use_bilateral_denoise_ = true;

  RaytraceDataBuf data_;

 public:
  RaytracingModule(Instance &inst) : inst_(inst){};

  void init();

  void sync();

  /**
   * Raytrace the scene and resolve a radiance buffer for the corresponding `closure_bit` into the
   * given `out_radiance_tx`.
   */
  void trace(eClosureBits closure_bit, GPUTexture *out_radiance_tx, View &view);

  TextureFromPool &release_results(int2 extent, eClosureBits closure_bit, View &view);

  void debug_pass_sync();
  void debug_draw(View &view, GPUFrameBuffer *view_fb);
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name Raytracing Buffers
 *
 * Contain persistent data used for temporal denoising. Similar to \class GBuffer but only contains
 * persistent data.
 * \{ */

struct RaytraceBuffer {};

/** \} */

}  // namespace blender::eevee
