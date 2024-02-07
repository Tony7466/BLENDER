/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup eevee
 */

#pragma once

#include "eevee_lightprobe.hh"
#include "eevee_shader_shared.hh"

#include "BKE_cryptomatte.hh"

extern "C" {
struct Material;
}

namespace blender::eevee {

class Instance;
struct ObjectHandle;
struct WorldHandle;
class CaptureView;
struct ReflectionProbeUpdateInfo;

/* -------------------------------------------------------------------- */
/** \name Reflection Probe Module
 * \{ */

class ReflectionProbeModule {
  friend LightProbeModule;
  /* Capture View requires access to the cube-maps texture for frame-buffer configuration. */
  friend class CaptureView;
  /* Instance requires access to #update_probes_this_sample_ */
  friend class Instance;

 private:
  /**
   * The maximum resolution of a cube-map side.
   *
   * Must be a power of two; intention to be used as a cube-map atlas.
   */
  static constexpr int max_resolution_ = 2048;

  Instance &instance_;
  ReflectionProbeDataBuf data_buf_;

  /* World probe is stored and managed by the reflection probe module. */
  ReflectionCube world_probe;

  /** Probes texture stored in octahedral mapping. */
  Texture probes_tx_ = {"Probes"};

  PassSimple remap_ps_ = {"Probe.CubemapToOctahedral"};
  PassSimple update_irradiance_ps_ = {"Probe.UpdateIrradiance"};
  PassSimple select_ps_ = {"Probe.Select"};

  int3 dispatch_probe_pack_ = int3(1);
  int3 dispatch_probe_select_ = int3(1);

  /**
   * Texture containing a cube-map where the probe should be rendering to.
   *
   * NOTE: TextureFromPool doesn't support cube-maps.
   */
  Texture cubemap_tx_ = {"Probe.Cubemap"};
  /** Index of the probe being updated. */
  int probe_index_ = 0;
  /** Mip level being sampled for remapping. */
  int probe_mip_level_ = 0;
  /** Updated Probe coordinates in the atlas. */
  ReflectionProbeCoordinate probe_sampling_coord_;
  ReflectionProbeWriteCoordinate probe_write_coord_;
  /** World coordinates in the atlas. */
  ReflectionProbeCoordinate world_sampling_coord_;
  /** Number of the probe to process in the select phase. */
  int reflection_probe_count_ = 0;

  /**
   * True if the next redraw will trigger a light-probe sphere update.
   * As syncing the draw passes for rendering has a significant overhead,
   * we only trigger this sync path if we detect updates. But we only know
   * this after `end_sync` which is too late to sync objects for light-probe
   * rendering. So we tag the next redraw (or sample) to do the sync.
   */
  bool update_probes_next_sample_ = false;
  /** True if the this redraw will trigger a light-probe sphere update. */
  bool update_probes_this_sample_ = false;
  /** Compute world irradiance coefficient and store them into the volume probe atlas. */
  bool do_world_irradiance_update = true;

  /** Viewport data display drawing. */
  bool do_display_draw_ = false;
  ReflectionProbeDisplayDataBuf display_data_buf_;
  PassSimple viewport_display_ps_ = {"ReflectionProbeModule.Viewport Display"};

 public:
  ReflectionProbeModule(Instance &instance);

  void init();
  void begin_sync();
  void sync_world(::World *world);
  void end_sync();

  void viewport_draw(View &view, GPUFrameBuffer *view_fb);

  template<typename PassType> void bind_resources(PassType &pass)
  {
    pass.bind_texture(REFLECTION_PROBE_TEX_SLOT, &probes_tx_);
    pass.bind_ubo(REFLECTION_PROBE_BUF_SLOT, &data_buf_);
  }

  void set_view(View &view);

  void debug_print() const;

  int atlas_extent() const
  {
    return max_resolution_;
  }

  /**
   * Get the resolution of a single cube-map side when rendering probes.
   *
   * The cube-maps are rendered half size of the size of the octahedral texture.
   */
  int probe_render_extent() const;

  ReflectionProbeAtlasCoordinate world_atlas_coord_get() const;

  void tag_world_for_update()
  {
    world_probe.do_render = true;
    do_world_irradiance_update = true;
  }

  void tag_world_irradiance_for_update()
  {
    do_world_irradiance_update = true;
  }

 private:
  /** Get the number of layers that is needed to store probes. */
  int needed_layers_get() const;

  void ensure_cubemap_render_target(int resolution);

  /**
   * Create a reflection probe data element that points to an empty spot in the cubemap that
   * can hold a texture with the given subdivision_level.
   */
  ReflectionProbeAtlasCoordinate find_empty_atlas_region(int subdivision_level) const;

  /**
   * Pop the next reflection probe that requires to be updated.
   */
  std::optional<ReflectionProbeUpdateInfo> world_update_info_pop();
  std::optional<ReflectionProbeUpdateInfo> probe_update_info_pop();

  void remap_to_octahedral_projection(const ReflectionProbeAtlasCoordinate &atlas_coord);
  void update_probes_texture_mipmaps();
  void update_world_irradiance();

  bool has_only_world_probe() const;

  /**
   * Ensure atlas texture is the right size.
   * Returns true if the texture has been cleared and all probes needs to be rendered again.
   */
  bool ensure_atlas();

  ReflectionProbeUpdateInfo update_info_from_probe(const ReflectionCube &probe);

  eLightProbeResolution reflection_probe_resolution() const;
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name Reflection Probe Update Info
 * \{ */

struct ReflectionProbeUpdateInfo {
  float3 probe_pos;
  /**
   * Resolution of the cubemap to be rendered.
   */
  int resolution;

  float2 clipping_distances;

  ReflectionProbeAtlasCoordinate atlas_coord;

  bool do_render;
  bool do_world_irradiance_update;
};

/** \} */

}  // namespace blender::eevee
