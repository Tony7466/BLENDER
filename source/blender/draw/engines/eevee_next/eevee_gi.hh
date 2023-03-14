/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "eevee_shader_shared.hh"

namespace blender::eevee {

class Instance;

class GI {
 private:
  Instance &inst_;

  SurfelBuf surfels;
  PassSimple debug_surfels_ps_ = {"GI.Debug"};
  GPUShader *debug_surfels_sh_ = nullptr;

  /* TODO: Remove this. */
  void generate_random_surfels();

 public:
  GI(Instance &inst) : inst_(inst){};
  ~GI(){};

  void init();
  void sync();

  void debug_pass_sync();
  void debug_draw(View &view, GPUFrameBuffer *view_fb);
};

}  // namespace blender::eevee
