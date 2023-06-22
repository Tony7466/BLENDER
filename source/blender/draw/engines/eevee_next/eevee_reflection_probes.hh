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

#include "draw_manager.hh"

#include "eevee_shader_shared.hh"

#include "BKE_cryptomatte.hh"

extern "C" {
struct Material;
}

namespace blender::eevee {

class Instance;
struct ObjectHandle;
class WorldProbePipeline;

/* -------------------------------------------------------------------- */
/** \name Reflection Probes
 * \{ */

class ReflectionProbe {
 public:
  enum Type { UNUSED, WORLD, PROBE };

  Type type = Type::UNUSED;
  bool is_dirty = false;
  /* When reflection probe is a probe its ObjectKey.hash_value is copied here to keep track between
   * draws.*/
  uint64_t object_hash_value = 0;

  /**
   * Probes that aren't used during a draw can be cleared.
   */
  bool is_used = false;

  /**
   * Index into ReflectionProbeDataBuf.
   * -1 = not added yet.
   */
  int index = -1;

  bool needs_update() const;
};

class ReflectionProbeModule {
 private:
  /** The max number of probes to track. */
  static constexpr int INITIAL_PROBES = 1;

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
  ReflectionProbeDataBuf data_buf_;
  Vector<ReflectionProbe> cubemaps_;
  Texture cubemaps_tx_ = {"Probes"};

 public:
  ReflectionProbeModule(Instance &instance) : instance_(instance) {}

  void init();
  void set_world_dirty();

  void sync_object(Object *ob, ObjectHandle &ob_handle, ResourceHandle res_handle, bool is_dirty);
  void end_sync();

  template<typename T> void bind_resources(draw::detail::PassBase<T> *pass)
  {
    pass->bind_texture(REFLECTION_PROBE_TEX_SLOT, cubemaps_tx_);
    pass->bind_ssbo(REFLECTION_PROBE_BUF_SLOT, data_buf_);
  }

  void debug_print() const;
  void validate() const;

 private:
  void sync(const ReflectionProbe &cubemap);
  ReflectionProbe &find_or_insert(ObjectHandle &ob_handle);

  /** Get the number of layers that is needed to store probes. */
  int needed_layers_get() const;

  void remove_unused_probes();

  /* TODO: also add _len() which is a max + 1. */
  /* Get the number of reflection probe data elements. */
  int reflection_probe_data_index_max() const;

  /**
   * Remove reflection probe data from the module.
   * Ensures that data_buf is sequential and cubemaps are relinked to its corresponding data.
   */
  void remove_reflection_probe_data(int reflection_probe_data_index);

  /**
   * Create a reflection probe data element that points to an empty spot in the cubemap that can
   * hold a texture with the given subdivision_level.
   */
  ReflectionProbeData find_empty_reflection_probe_data(int subdivision_level) const;

  void upload_dummy_cubemap(const ReflectionProbe &probe);

  friend class WorldProbePipeline;
};

}  // namespace blender::eevee
