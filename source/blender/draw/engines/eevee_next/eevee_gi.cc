/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_rand.hh"
#include "eevee_instance.hh"

#include "eevee_gi.hh"

namespace blender::eevee {

void GI::generate_random_surfels()
{
  const int surfels_len = 256;
  surfels.resize(surfels_len);

  RandomNumberGenerator rng;
  rng.seed(0);

  for (Surfel &surfel : surfels) {
    float3 random = rng.get_unit_float3();
    surfel.position = random * 3.0f;
    surfel.normal = random;
    surfel.color = float4(rng.get_float(), rng.get_float(), rng.get_float(), 1.0f);
  }

  surfels.push_update();
}

void GI::init()
{
  if (debug_surfels_sh_ == nullptr) {
    debug_surfels_sh_ = GPU_shader_create_from_info_name("eevee_gi_debug_surfels");
  }

  /* TODO: Remove this. */
  generate_random_surfels();
}

void GI::sync()
{
  debug_pass_sync();
}

void GI::debug_pass_sync()
{
  if (inst_.debug_mode == eDebugMode::DEBUG_GI_SURFELS) {
    debug_surfels_ps_.init();
    debug_surfels_ps_.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH |
                                DRW_STATE_DEPTH_LESS_EQUAL);
    debug_surfels_ps_.shader_set(debug_surfels_sh_);
    debug_surfels_ps_.bind_ssbo(GI_SURFEL_BUF_SLOT, surfels);
    debug_surfels_ps_.push_constant("surfel_radius", 0.25f);
    debug_surfels_ps_.draw_procedural(GPU_PRIM_TRI_STRIP, surfels.size(), 4);
  }
}

void GI::debug_draw(View &view, GPUFrameBuffer *view_fb)
{
  if (inst_.debug_mode == eDebugMode::DEBUG_GI_SURFELS) {
    inst_.info = "Debug Mode: GI Surfels";
    GPU_framebuffer_bind(view_fb);
    inst_.manager->submit(debug_surfels_ps_, view);
  }
}

}  // namespace blender::eevee
