/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup eevee
 */

#pragma once

#include "eevee_shader_shared.hh"

namespace blender::eevee {

class Instance;
class CapturePipeline;

class IrradianceCache {
  friend CapturePipeline;

 private:
  Instance &inst_;

  /** Surface elements that represent the scene. */
  SurfelBuf surfels_buf_;
  /** Capture state. */
  CaptureInfoBuf capture_info_buf_;

  PassSimple debug_surfels_ps_ = {"IrradianceCache.Debug"};

  Framebuffer empty_raster_fb_ = {"empty_raster_fb_"};
  View view_ = {"ortho_raster_view"};

  /* TODO: Remove this. */
  void generate_random_surfels();

 public:
  IrradianceCache(Instance &inst) : inst_(inst){};
  ~IrradianceCache(){};

  void init();
  void sync();

  void create_surfels();
  void propagate_light();

  void debug_pass_sync();
  void debug_draw(View &view, GPUFrameBuffer *view_fb);
};

}  // namespace blender::eevee
