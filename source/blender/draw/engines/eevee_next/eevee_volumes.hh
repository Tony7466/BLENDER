/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "eevee_shader_shared.hh"

namespace blender::eevee {

class Instance;

class Volumes {
 private:
  Instance &inst_;

  bool enabled_;

  VolumesDataBuf data_;

  Texture prop_scattering_tx_;
  Texture prop_extinction_tx_;
  Texture prop_emission_tx_;
  Texture prop_phase_tx_;
  Texture scatter_tx_;
  Texture extinction_tx_;
  Texture integrated_scatter_tx_;
  Texture integrated_transmit_tx_;
  Texture dummy_scatter_tx_;
  Texture dummy_transmit_tx_;

  GPUTexture *transparent_pass_scatter_tx_;
  GPUTexture *transparent_pass_transmit_tx_;

  Framebuffer volumetric_fb_;
  Framebuffer scatter_fb_;
  Framebuffer integration_fb_;
  Framebuffer resolve_fb_;

  PassSimple world_ps_ = {"Volumes.World"};
  PassSimple scatter_ps_ = {"Volumes.Scatter"};
  PassSimple integration_ps_ = {"Volumes.Integration"};
  PassSimple resolve_ps_ = {"Volumes.Resolve"};
  PassSimple accum_ps_ = {"Volumes.Accum"};

 public:
  Volumes(Instance &inst) : inst_(inst){};

  ~Volumes(){};

  template<typename PassType> void bind_resources(PassType &ps)
  {
    ps.bind_ubo(VOLUMES_BUF_SLOT, data_);
    ps.bind_texture(VOLUME_SCATTERING_TEX_SLOT, &transparent_pass_scatter_tx_);
    ps.bind_texture(VOLUME_TRANSMITTANCE_TEX_SLOT, &transparent_pass_transmit_tx_);
  }

  template<typename PassType>
  void bind_volume_pass_resources(PassType &ps, bool is_material_pass = false)
  {
    ps.bind_ubo(VOLUMES_BUF_SLOT, data_);
    if (is_material_pass) {
      ps.bind_image(0, &prop_scattering_tx_);
      ps.bind_image(1, &prop_extinction_tx_);
      ps.bind_image(2, &prop_emission_tx_);
      ps.bind_image(3, &prop_phase_tx_);
    }
    inst_.lights.bind_resources(&ps);
    inst_.shadows.bind_resources(&ps);
  }

  void set_jitter(uint current_sample);

  void init();

  void begin_sync();

  void sync_object(Object *ob, ObjectHandle &ob_handle, ResourceHandle res_handle);

  void end_sync();

  void draw_compute(View &view);

  void draw_resolve(View &view);
};
}  // namespace blender::eevee
