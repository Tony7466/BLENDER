/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. */

/** \file
 * \ingroup eevee
 *
 * Cubemaps
 *
 * Cubemaps record light from different locations in the scene. These cubemaps are used to add
 * environment and indirect lighting from light probes.
 *
 * - Although we have Global illumination light probes can still be used for situations that
 *   GI doesn't work. For example semi-transparent surfaces.
 * - The first cubemap is always reserved for the world shading.
 *
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
class ReflectionProbe {
 public:
  enum Type { UNUSED, WORLD };

  Type type;
  bool is_dirty = false;

  bool needs_update() const;
};

class ReflectionProbeModule {
 private:
  /** The max number of probes to track. */
  static constexpr int max_probes = 1;

  /**
   * The maximum resolution of a cubemap side.
   *
   * Must be a power of two; intension to be used as a cubemap atlas.
   */
  static constexpr int max_resolution = 2048;
  static constexpr int max_mipmap_levels = log(max_resolution) + 1;

  /**
   * Index of the probe that is used for world background.
   *
   * NOTE: First probe always contains the world probe.
   */
  static constexpr int world_slot = 0;

  Instance &instance_;

  Vector<ReflectionProbe> cubemaps_;
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
