/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "eevee_shader_shared.hh"

namespace blender::eevee {

class Instance;

class Volumes {
 private:
  Instance &inst_;

  bool enabled_;

  /* eevee_volumes.c e_data */
  Texture depth_src_tx_;
  Texture dummy_zero_tx_;
  Texture dummy_one_tx_;
  Texture dummy_flame_tx_;
  Texture dummy_scatter_tx_;
  Texture dummy_transmit_tx_;
  /* EEVEE_CommonUniformBuffer */
  VolumesDataBuf data_;
  /* EEVEE_TextureList */
  Texture prop_scattering_tx_;
  Texture prop_extinction_tx_;
  Texture prop_emission_tx_;
  Texture prop_phase_tx_;
  SwapChain<Texture, 2> scatter_tx_;
  SwapChain<Texture, 2> transmit_tx_;
  /* EEVEE_FramebufferList */
  Framebuffer volumetric_fb_;
  Framebuffer scatter_fb_;
  Framebuffer integration_fb_;
  Framebuffer resolve_fb_;
  /* EEVEE_EffectsInfo */
  int current_sample_;
  float light_clamp_;
#if 0
  /* TODO (Miguel Pozo): These ones were used for transparency */
  struct GPUTexture *volume_scatter;
  struct GPUTexture *volume_transmit;
#endif
  /* EEVEE_PassList */
  PassSimple world_ps_ = {"Volumes.World"};
  PassMain objects_ps_ = {"Volumes.Objects"};
  PassSimple scatter_ps_ = {"Volumes.Scatter"};
  PassSimple integration_ps_ = {"Volumes.Integration"};
  PassSimple resolve_ps_ = {"Volumes.Resolve"};
  PassSimple accum_ps_ = {"Volumes.Accum"};

  float4x4 prev_view_projection_matrix;

  template<typename PassType> void bind_common_resources(PassType &ps)
  {
    ps.bind_ubo("volumes_buf", data_);
    inst_.lights.bind_resources(&ps);
    inst_.shadows.bind_resources(&ps);
  }

 public:
  Volumes(Instance &inst) : inst_(inst){};

  ~Volumes(){};

  void set_jitter(uint current_sample);

  void init();

  void begin_sync();

  void sync_object(Object *ob, ObjectHandle &ob_handle, ResourceHandle res_handle);

  void end_sync();

  void draw_compute(View &view);

  void draw_resolve(View &view);
};
}  // namespace blender::eevee

/** \} */
