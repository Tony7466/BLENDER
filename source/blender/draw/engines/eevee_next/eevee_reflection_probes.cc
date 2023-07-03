/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. */

#include "eevee_reflection_probes.hh"
#include "eevee_instance.hh"

namespace blender::eevee {

void ReflectionProbeModule::init()
{
  if (!initialized_) {
    const int max_mipmap_levels = log(max_resolution_) + 1;
    cubemap_tx_.ensure_cube(GPU_RGBA16F,
                            max_resolution_,
                            GPU_TEXTURE_USAGE_SHADER_READ | GPU_TEXTURE_USAGE_ATTACHMENT,
                            nullptr,
                            max_mipmap_levels);
    GPU_texture_mipmap_mode(cubemap_tx_, true, true);
    probes_tx_.ensure_2d_array(GPU_RGBA16F,
                               int2(max_resolution_),
                               max_probes_,
                               GPU_TEXTURE_USAGE_SHADER_READ | GPU_TEXTURE_USAGE_ATTACHMENT,
                               nullptr,
                               max_mipmap_levels);
    GPU_texture_mipmap_mode(probes_tx_, true, true);
    initialized_ = true;
  }

  {
    PassSimple &pass = remap_ps_;
    pass.init();
    pass.shader_set(instance_.shaders.static_shader_get(REFLECTION_PROBE_REMAP));
    pass.bind_texture(REFLECTION_PROBE_TEX_SLOT, cubemap_tx_);
    pass.bind_image(REFLECTION_PROBE_OCTAHEDRAL_SLOT, probes_tx_);
    pass.dispatch(int2(ceil_division(max_resolution_, REFLECTION_PROBE_GROUP_SIZE)));
  }
}

}  // namespace blender::eevee
