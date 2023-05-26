/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup eevee
 *
 * Ground Truth Ambient Occlusion
 *
 */

#pragma once

#include "eevee_shader_shared.hh"

namespace blender::eevee {

class Instance;

/* -------------------------------------------------------------------- */
/** \name AmbientOcclusion
 * \{ */

class AmbientOcclusion {
 private:
  class Instance &inst_;

  bool debug_;

  /* TODO: Move somewhere else. */
  UniformBuffer<RayTracingData> rt_data_;

  UniformBuffer<AOData> data_;

  Texture dummy_horizons_tx_;
  Texture horizons_tx_;
  Texture horizons_debug_tx_;

  Framebuffer fb_ = {"GTAO"};

  PassSimple horizons_search_ps_ = {"GTAO Horizons Search"};

 public:
  AmbientOcclusion(Instance &inst) : inst_(inst){};
  ~AmbientOcclusion(){};

  void init();

  void sync();

  void render(View &view);
};

/** \} */

}  // namespace blender::eevee
