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

/* -------------------------------------------------------------------- */
/** \name Reflection Probes
 * \{ */
class ReflectionProbe {
 public:
  enum Type { Unused, World };

  Type type;
  bool is_dirty = false;

  bool needs_update() const;
};

class ReflectionProbeModule {
 private:
  /** The max number of probes to track. */
  static constexpr int MAX_PROBES = 1;

  /**
   * The maximum resolution of a cubemap side.
   *
   * Must be a power of two; intension to be used as a cubemap atlas.
   */
  static constexpr int MAX_RESOLUTION = 2048;
  static constexpr int MIPMAP_LEVELS = 12;

  /**
   * Index of the probe that is used for world background.
   *
   * NOTE: First probe always contains the world probe.
   */
  static constexpr int WORLD_SLOT = 0;

  Instance &instance_;

  Vector<ReflectionProbe> cubemaps_;
  Texture cubemaps_tx_ = {"Probes"};

 public:
  ReflectionProbeModule(Instance &instance) : instance_(instance) {}

  void init();
  void set_world_dirty();

  void sync();

  template<typename T> void bind_resources(draw::detail::PassBase<T> *pass)
  {
    pass->bind_texture(REFLECTION_PROBE_TEX_SLOT, cubemaps_tx_);
  }

 private:
  void sync(const ReflectionProbe &cubemap);

  friend class WorldPipeline;
};

}  // namespace blender::eevee
