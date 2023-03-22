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
  Texture scatter_tx_;
  Texture transmit_tx_;
  Texture scatter_history_tx_;
  Texture transmit_history_tx_;
  /* EEVEE_FramebufferList */
  Framebuffer volumetric_fb_;
  Framebuffer scatter_fb_;
  /* EEVEE_EffectsInfo */
  int current_sample_;
  float light_clamp_;
#if 0
  /* TODO (Miguel Pozo): Are these the same ones from EEVEE_TextureList ? */
  struct GPUTexture *volume_scatter;
  struct GPUTexture *volume_transmit;
#endif
  /* EEVEE_PassList */
  PassMain world_ps_ = {"Volumes.World"};
  PassMain objects_ps_ = {"Volumes.Objects"};
  PassMain scatter_ps_ = {"Volumes.Scatter"};
  PassMain integration_ps_ = {"Volumes.Integration"};
  PassMain resolve_ps_ = {"Volumes.Resolve"};
  PassMain accum_ps_ = {"Volumes.Accum"};

  void bind_common_buffers(PassMain &ps);

 public:
  Volumes(Instance &inst) : inst_(inst){};

  ~Volumes(){};

  void set_jitter(uint current_sample);

  void init();

  void begin_sync();

  void sync_object(Object *ob, ObjectHandle &ob_handle, ResourceHandle res_handle);

  void end_sync();

  void draw_compute(View &view);

  void draw_resolve(View &view, Framebuffer &fb);
};
}  // namespace blender::eevee

/** \} */
