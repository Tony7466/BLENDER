/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup eevee
 *
 * Ground Truth Ambient Occlusion
 *
 * Based on Practical Realtime Strategies for Accurate Indirect Occlusion
 * http://blog.selfshadow.com/publications/s2016-shading-course/activision/s2016_pbs_activision_occlusion.pdf
 * http://blog.selfshadow.com/publications/s2016-shading-course/activision/s2016_pbs_activision_occlusion.pptx
 *
 * Algorithm Overview:
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
  const Scene *scene = inst_.scene;
  data_.enabled = (scene->eevee.flag & SCE_EEVEE_GTAO_ENABLED) ||
                  (inst_.film.enabled_passes_get() & EEVEE_RENDER_PASS_AO);
  render_pass_enabled_ = data_.enabled && inst_.film.enabled_passes_get() & EEVEE_RENDER_PASS_AO;

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
}

void AmbientOcclusion::sync()
{
  if (!data_.enabled) {
    return;
  }

  horizons_search_ps_.init();
  horizons_search_ps_.state_set(DRW_STATE_WRITE_COLOR);
  horizons_search_ps_.shader_set(inst_.shaders.static_shader_get(AO_HORIZONS));
  bind_resources(&horizons_search_ps_, false);

  horizons_search_ps_.draw_procedural(GPU_PRIM_TRIS, 1, 3);

  if (render_pass_enabled_) {
    render_pass_ps_.init();
    render_pass_ps_.shader_set(inst_.shaders.static_shader_get(AO_PASS));
    bind_resources(&render_pass_ps_);

    render_pass_ps_.bind_image("in_normal_img", &rp_normal_tx_);
    render_pass_ps_.bind_image("out_ao_img", &rp_ao_tx_);

    render_pass_ps_.barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS & GPU_BARRIER_TEXTURE_FETCH);
    render_pass_ps_.dispatch(
        math::divide_ceil(inst_.film.render_extent_get(), int2(AO_PASS_TILE_SIZE)));
  }
}

void AmbientOcclusion::render(View &view)
{
  if (!data_.enabled) {
    return;
  }

  inst_.hiz_buffer.update();

  fb_.ensure(GPU_ATTACHMENT_NONE, GPU_ATTACHMENT_TEXTURE(horizons_tx_));
  fb_.bind();
  inst_.manager->submit(horizons_search_ps_, view);

  if (GPU_mip_render_workaround() ||
      GPU_type_matches_ex(GPU_DEVICE_INTEL_UHD, GPU_OS_WIN, GPU_DRIVER_ANY, GPU_BACKEND_OPENGL))
  {
    /* Fix dot corruption on intel HD5XX/HD6XX series. */
    GPU_flush();
  }
}

void AmbientOcclusion::render_pass(View &view)
{
  if (!render_pass_enabled_) {
    return;
  }

  inst_.hiz_buffer.update();

  RenderBuffers &rb = inst_.render_buffers;

  rb.rp_color_tx.ensure_layer_views();
  rp_normal_tx_ = rb.rp_color_tx.layer_view(rb.data.normal_id);
  rb.rp_value_tx.ensure_layer_views();
  rp_ao_tx_ = rb.rp_value_tx.layer_view(rb.data.ambient_occlusion_id);

  inst_.manager->submit(render_pass_ps_, view);
}

}  // namespace blender::eevee
