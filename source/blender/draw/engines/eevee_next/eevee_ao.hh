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
  bool render_pass_enabled_;

  AODataBuf data_;

  Texture dummy_horizons_tx_;
  Texture horizons_tx_;

  Framebuffer fb_ = {"GTAO"};

  PassSimple horizons_search_ps_ = {"GTAO Horizons Search"};
  PassSimple render_pass_ps_ = {"GTAO Render Pass"};

  /* Used as pointers for texture views in the AO Render Pass. */
  GPUTexture *rp_normal_tx_ = nullptr;
  GPUTexture *rp_ao_tx_ = nullptr;

 public:
  AmbientOcclusion(Instance &inst) : inst_(inst)
  {
    dummy_horizons_tx_.ensure_2d(GPU_RGBA8,
                                 int2(1),
                                 GPU_TEXTURE_USAGE_ATTACHMENT | GPU_TEXTURE_USAGE_SHADER_READ,
                                 float4(0));
  };
  ~AmbientOcclusion(){};

  void init();

  void sync();

  void render(View &view);
  void render_pass(View &view);

  template<typename T>
  void bind_resources(draw::detail::PassBase<T> *pass, bool bind_horizons = true)
  {
    inst_.sampling.bind_resources(pass);
    inst_.hiz_buffer.bind_resources(pass);
    inst_.raytracing.bind_resources(pass);
    pass->bind_texture(RBUFS_UTILITY_TEX_SLOT, &inst_.pipelines.utility_tx);
    pass->bind_ubo(AO_BUF_SLOT, &data_);
    if (bind_horizons) {
      pass->bind_texture(AO_HORIZONS_TEX_SLOT,
                         data_.enabled ? &horizons_tx_ : &dummy_horizons_tx_);
    }
  }
};

/** \} */

}  // namespace blender::eevee
