/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup eevee
 *
 * Ground Truth Ambient Occlusion
 *
 */

/**
 * Occlusion Algorithm Overview:
 *
 * We separate the computation into 2 steps.
 *
 * - First we scan the neighborhood pixels to find the maximum horizon angle.
 *   We save this angle in a RG8 array texture.
 *
 * - Then we use this angle to compute occlusion with the shading normal at
 *   the shading stage. This let us do correct shadowing for each diffuse / specular
 *   lobe present in the shader using the correct normal.
 */

#pragma once

#include "eevee_ao.hh"
#include "eevee_instance.hh"

#include "GPU_capabilities.h"

namespace blender::eevee {

/* -------------------------------------------------------------------- */
/** \name AmbientOcclusion
 * \{ */

void AmbientOcclusion::init()
{
  debug_ = G.debug_value == 6;

  const Scene *scene = inst_.scene;
  data_.enabled = (scene->eevee.flag & SCE_EEVEE_GTAO_ENABLED) ||
                  (inst_.film.enabled_passes_get() & EEVEE_RENDER_PASS_AO);

  if (!data_.enabled) {
    /* Early return. */
    data_.push_update();
    horizons_tx_.free();
    return;
  }

  data_.distance = scene->eevee.gtao_distance;
  data_.factor = std::max(1e-4f, scene->eevee.gtao_factor);
  data_.quality = scene->eevee.gtao_quality;
  data_.use_bent_normals = scene->eevee.flag & SCE_EEVEE_GTAO_BENT_NORMALS;
  data_.bounce_factor = (scene->eevee.flag & SCE_EEVEE_GTAO_BOUNCE) ? 1.0f : 0.0f;

  data_.push_update();

  eGPUTextureUsage usage = GPU_TEXTURE_USAGE_ATTACHMENT | GPU_TEXTURE_USAGE_SHADER_READ;
  horizons_tx_.ensure_2d(GPU_RGBA8, inst_.film.render_extent_get(), usage);

  /* Compute pixel size. Size is multiplied by 2 because it is applied in NDC [-1..1] range. */
  rt_data_.pixel_size = float2(2.0f) / float2(inst_.film.render_extent_get());
  rt_data_.push_update();
}

void AmbientOcclusion::sync()
{
  if (!data_.enabled) {
    return;
  }

  horizons_search_ps_.init();
  horizons_search_ps_.state_set(DRW_STATE_WRITE_COLOR);
  horizons_search_ps_.shader_set(inst_.shaders.static_shader_get(AO));
  horizons_search_ps_.bind_ubo(RAYTRACE_BUF_SLOT, &rt_data_);
  horizons_search_ps_.bind_ubo(AO_BUF_SLOT, &data_);
  horizons_search_ps_.bind_texture("utility_tx", &inst_.pipelines.utility_tx);
  inst_.hiz_buffer.bind_resources(&horizons_search_ps_);

  horizons_search_ps_.draw_procedural(GPU_PRIM_TRIS, 1, 3);
}

void AmbientOcclusion::render(View &view)
{
  if (!data_.enabled) {
    return;
  }

  inst_.hiz_buffer.update();

  fb_.ensure(GPU_ATTACHMENT_NONE, GPU_ATTACHMENT_TEXTURE(horizons_tx_));
  inst_.manager->submit(horizons_search_ps_, view);

  if (GPU_mip_render_workaround() ||
      GPU_type_matches_ex(GPU_DEVICE_INTEL_UHD, GPU_OS_WIN, GPU_DRIVER_ANY, GPU_BACKEND_OPENGL))
  {
    /* Fix dot corruption on intel HD5XX/HD6XX series. */
    GPU_flush();
  }
}

}  // namespace blender::eevee
