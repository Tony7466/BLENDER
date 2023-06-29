/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. */

/** \file
 * \ingroup eevee
 */

#pragma once

#include "eevee_shader_shared.hh"

#include "BKE_cryptomatte.hh"

extern "C" {
struct Material;
}

namespace blender::eevee {

class Instance;
class WorldPipeline;
class CaptureView;

/* -------------------------------------------------------------------- */
/** \name Reflection Probes
 * \{ */

class ReflectionProbeModule {
 private:
  /** The max number of probes to track. */
  static constexpr int max_probes_ = 1;

  /**
   * The maximum resolution of a cubemap side.
   *
   * Must be a power of two; intension to be used as a cubemap atlas.
   */
  static constexpr int max_resolution_ = 2048;
  static constexpr int max_mipmap_levels_ = log(max_resolution_) + 1;

  Instance &instance_;

  Texture cubemaps_tx_ = {"Probes"};

 public:
  ReflectionProbeModule(Instance &instance) : instance_(instance) {}

  void init();

  template<typename T> void bind_resources(draw::detail::PassBase<T> *pass)
  {
    pass->bind_texture(REFLECTION_PROBE_TEX_SLOT, cubemaps_tx_);
  }

 private:
  /* Capture View requires access to the cubemaps texture for framebuffer configuration. */
  friend class CaptureView;
};

}  // namespace blender::eevee
