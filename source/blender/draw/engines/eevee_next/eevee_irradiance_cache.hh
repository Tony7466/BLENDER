/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup eevee
 */

#pragma once

#include "DNA_lightprobe_types.h"

#include "BLI_math_quaternion_types.hh"

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
  /** Evaluate light object contribution and store result to surfel. */
  PassSimple surfel_light_eval_ps_ = {"LightEval"};
  /** Propagate light from surfel to surfel. */
  PassSimple surfel_light_propagate_ps_ = {"LightPropagate"};
  /** Capture surfel lighting to irradiance samples. */
  PassSimple irradiance_capture_ps_ = {"IrradianceCapture"};

  /** Basis orientation for each baking projection. */
  math::CartesianBasis basis_x_ = math::from_orthonormal_axes(math::AxisSigned::Y_POS,
                                                              math::AxisSigned::X_POS);
  math::CartesianBasis basis_y_ = math::from_orthonormal_axes(math::AxisSigned::Z_POS,
                                                              math::AxisSigned::Y_POS);
  math::CartesianBasis basis_z_ = math::from_orthonormal_axes(math::AxisSigned::X_POS,
                                                              math::AxisSigned::Z_POS);
  /** Views for each baking projection. */
  View view_x_ = {"BakingViewX"};
  View view_y_ = {"BakingViewY"};
  View view_z_ = {"BakingViewZ"};
  /** Pixel resolution in each of the projection axes. Match the target surfel density. */
  int3 grid_pixel_extent_ = int3(0);
  /** Information for surfel list building. */
  SurfelListInfoBuf list_info_buf_ = {"list_info_buf_"};
  /** List array containing list start surfel index. Cleared to -1. */
  StorageArrayBuffer<int, 16, true> list_start_buf_ = {"list_start_buf_"};

  /* Dispatch size for per surfel workload. */
  int3 dispatch_per_surfel_ = int3(1);
  /* Dispatch size for per surfel list workload. */
  int3 dispatch_per_list_ = int3(1);

  /* Surfel per unit distance. */
  float surfel_density_ = 2.0f;
  /* Orientation of the irradiance grid being baked. */
  math::Quaternion grid_orientation_;
  /* Object center of the irradiance grid being baked. */
  float3 grid_location_;
  /* Bounding box vertices of the irradiance grid being baked. In world space. */
  Vector<float3> grid_bbox_vertices;

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
