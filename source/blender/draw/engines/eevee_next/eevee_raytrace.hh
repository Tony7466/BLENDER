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

  draw::PassSimple raytrace_ps_ = {"Raytrace"};

  TextureFromPool tile_mask_tx_ = {"tile_mask_tx"};

  /**
   * Each ray type has a indirect dispatch and a tile buffer that contains references to tiles.
   * This avoid dispatching workgroups that ultimately won't do any tracing.
   */

  /** Resources for reflection rays. */
  DispatchIndirectBuf dispatch_reflect_buf_ = {"dispatch_reflect_buf"};
  RaytraceTileBuf tiles_reflect_buf_ = {"tiles_reflect_buf"};

  /** Trace results. Results are in scheduled tile order (tile order inside RaytraceTileBuf). */
  StorageArrayBuffer<float4, 512, true> trace_result_buf_ = {"trace_result_buf_"};

  int3 tile_dispatch_size_ = int3(1);

  bool enabled_ = false;

 public:
  RaytracingModule(Instance &inst) : inst_(inst){};

  void init();

  void sync();

  void trace(int2 extent, eClosureBits closure_bits, View &view);

  void debug_pass_sync();
  void debug_draw(View &view, GPUFrameBuffer *view_fb);

  bool enabled() const
  {
    return enabled_;
  }
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
