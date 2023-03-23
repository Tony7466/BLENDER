/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup eevee
 */

#pragma once

#include "DNA_lightprobe_types.h"

#include "eevee_lightprobe.hh"
#include "eevee_shader_shared.hh"

namespace blender::eevee {

class Instance;
class CapturePipeline;

/**
 * Baking related pass and data. Not used at runtime.
 */
class IrradianceBake {
  friend CapturePipeline;

 private:
  Instance &inst_;

  /** Light cache being baked. */
  LightCache *light_cache_ = nullptr;
  /** Surface elements that represent the scene. */
  SurfelBuf surfels_buf_;
  /** Capture state. */
  CaptureInfoBuf capture_info_buf_;
  /** Framebuffer. */
  Framebuffer empty_raster_fb_ = {"empty_raster_fb_"};
  View view_ = {"ortho_raster_view"};

 public:
  IrradianceBake(Instance &inst) : inst_(inst){};

  void sync();

  /** Create a surfel representation of the scene from the \a grid using the capture pipeline. */
  void surfels_create(const IrradianceGrid &grid);
  /** Evaluate direct lighting (and also clear the surfels radiance). */
  void surfels_lights_eval();
  /** Propagate light from surfel to surfel in a random direction over the sphere. */
  void propagate_light_sample();

  /** Read grid final irradiance back to CPU into \a light_cache_grid . */
  void read_result(LightCacheIrradianceGrid &light_cache_grid);
};

/**
 * Runtime container of diffuse indirect lighting.
 * Also have debug and baking components.
 */
class IrradianceCache {
 public:
  IrradianceBake bake;

 private:
  Instance &inst_;

  PassSimple debug_surfels_ps_ = {"IrradianceCache.Debug"};
  /** Debug surfel elements copied from the light cache. */
  draw::StorageVectorBuffer<Surfel> surfels_buf_;

 public:
  IrradianceCache(Instance &inst) : bake(inst), inst_(inst){};
  ~IrradianceCache(){};

  void init();
  void sync();

  void debug_pass_sync();
  void debug_draw(View &view, GPUFrameBuffer *view_fb);
};

}  // namespace blender::eevee
