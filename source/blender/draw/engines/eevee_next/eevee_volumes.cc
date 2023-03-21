/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw_engine
 *
 * Volumetric effects rendering using frostbite approach.
 */

#pragma once

#include "DRW_render.h"

#include "BLI_listbase.h"
#include "BLI_rand.h"
#include "BLI_string_utils.h"

#include "DNA_fluid_types.h"
#include "DNA_object_force_types.h"
#include "DNA_volume_types.h"
#include "DNA_world_types.h"

#include "BKE_fluid.h"
#include "BKE_global.h"
#include "BKE_mesh.h"
#include "BKE_modifier.h"
#include "BKE_volume.h"
#include "BKE_volume_render.h"

#include "ED_screen.h"

#include "DEG_depsgraph_query.h"

#include "GPU_capabilities.h"
#include "GPU_context.h"
#include "GPU_material.h"
#include "GPU_texture.h"

#include "draw_common.hh"

#include "eevee_instance.hh"
#include "eevee_pipeline.hh"
#include "eevee_shader.hh"

#include "eevee_volumes.hh"

#define LOOK_DEV_STUDIO_LIGHT_ENABLED(v3d) \
  ((v3d) && (((v3d->shading.type == OB_MATERIAL) && \
              ((v3d->shading.flag & V3D_SHADING_SCENE_WORLD) == 0)) || \
             ((v3d->shading.type == OB_RENDER) && \
              ((v3d->shading.flag & V3D_SHADING_SCENE_WORLD_RENDER) == 0))))

namespace blender::eevee {

void Volumes::bind_common_buffers(PassMain &ps)
{
  ps.bind_ubo(VOLUMES_BUF_SLOT, data_);
#if 0
  /* TODO (Miguel Pozo) */
  DRW_shgroup_uniform_block(grp, "common_block", sldata->common_ubo);
  /* TODO(fclem): remove those (need to clean the GLSL files). */
  DRW_shgroup_uniform_block(grp, "grid_block", sldata->grid_ubo);
  DRW_shgroup_uniform_block(grp, "probe_block", sldata->probe_ubo);
  DRW_shgroup_uniform_block(grp, "planar_block", sldata->planar_ubo);
  DRW_shgroup_uniform_block(grp, "light_block", sldata->light_ubo);
  DRW_shgroup_uniform_block(grp, "shadow_block", sldata->shadow_ubo);
  DRW_shgroup_uniform_block(grp, "renderpass_block", sldata->renderpass_ubo.combined);

  /* Should this go here? */
  ps.bind_texture("irradianceGrid", &lcache->grid_tx.tex);
  ps.bind_texture("shadowCubeTexture", &sldata->shadow_cube_pool);
  ps.bind_texture("shadowCascadeTexture", &sldata->shadow_cascade_pool);
#endif
}

void Volumes::set_jitter(uint current_sample)
{
  double ht_point[3];
  double ht_offset[3] = {0.0, 0.0};
  const uint3 ht_primes = {3, 7, 2};

  BLI_halton_3d(ht_primes, ht_offset, current_sample, ht_point);

  data_.jitter = float3(ht_point);
}

void Volumes::init()
{
  enabled_ = false;

  const DRWContextState *draw_ctx = DRW_context_state_get();
  const Scene *scene_eval = DEG_get_evaluated_scene(draw_ctx->depsgraph);

  const float3 viewport_size = DRW_viewport_size_get();
  const int tile_size = scene_eval->eevee.volumetric_tile_size;

  /* Find Froxel Texture resolution. */
  int3 tex_size = int3(math::ceil(math::max(float3(1.0f), viewport_size / float(tile_size))));
  tex_size.z = std::max(1, scene_eval->eevee.volumetric_samples);

  /* Clamp 3D texture size based on device maximum. */
  int3 max_size = int3(GPU_max_texture_3d_size());
  BLI_assert(tex_size == math::min(tex_size, max_size));
  tex_size = math::min(tex_size, max_size);

  data_.coord_scale_a = float2(viewport_size) / float2(tile_size * tex_size);
  data_.coord_scale_b = 1.0f / float2(viewport_size);

  /* TODO: compute snap to maxZBuffer for clustered rendering. */
  if (data_.tex_size != tex_size) {
    data_.tex_size = tex_size;
    data_.inv_tex_size = 1.0f / float3(tex_size);
    data_.history_alpha = 0.0f;
  }
  else {
    /* Like frostbite's paper, 5% blend of the new frame. */
    data_.history_alpha = 0.95f;
  }

  bool do_taa = inst_.sampling.sample_count() > 1;
  /* TODO (Miguel Pozo): Confirm this is equivalent to:
   * ((effects->enabled_effects & EFFECT_TAA) != 0) */

  if (draw_ctx->evil_C != nullptr) {
    struct wmWindowManager *wm = CTX_wm_manager(draw_ctx->evil_C);
    do_taa = do_taa && (ED_screen_animation_no_scrub(wm) == nullptr);
  }

  /* Temporal Super sampling jitter */
  /* TODO (Miguel Pozo): Double check current_sample and current_sample_ logic. */
  uint current_sample = 0;

  if (do_taa) {
    /* If TAA is in use do not use the history buffer. */
    data_.history_alpha = 0.0f;
    current_sample = inst_.sampling.current_sample() - 1;
    current_sample_ = -1;
  }
  else if (DRW_state_is_image_render()) {
    uint3 ht_primes = {3, 7, 2};
    const uint max_sample = ht_primes[0] * ht_primes[1] * ht_primes[2];
    current_sample = current_sample_ = (current_sample_ + 1) % max_sample;
    if (current_sample != max_sample - 1) {
      /* TODO (Miguel Pozo): This shouldn't be done here ? */
      DRW_viewport_request_redraw();
    }
  }

  set_jitter(current_sample);

  /* TODO (Miguel Pozo): Check eevee_lights use of effects->volume_light_clamp. */
  light_clamp_ = scene_eval->eevee.volumetric_light_clamp;
  /* Disable clamp if equal to 0. */
  if (light_clamp_ == 0.0) {
    light_clamp_ = FLT_MAX;
  }

  if ((scene_eval->eevee.flag & SCE_EEVEE_VOLUMETRIC_SHADOWS) == 0) {
    data_.shadow_steps = 0;
  }
  else {
    data_.shadow_steps = float(scene_eval->eevee.volumetric_shadow_samples);
  }

  data_.use_lights = (scene_eval->eevee.flag & SCE_EEVEE_VOLUMETRIC_LIGHTS) != 0;
  data_.use_soft_shadows = (scene_eval->eevee.flag & SCE_EEVEE_SHADOW_SOFT) != 0;
}

void Volumes::begin_sync()
{
  const DRWContextState *draw_ctx = DRW_context_state_get();
  Scene *scene = draw_ctx->scene;
  const Scene *scene_eval = DEG_get_evaluated_scene(draw_ctx->depsgraph);

  /* TODO (Miguel Pozo): Done here since it needs to run after camera sync.
   * Is this the way to go? */
  float integration_start = scene_eval->eevee.volumetric_start;
  float integration_end = scene_eval->eevee.volumetric_end;
  if (inst_.camera.is_perspective()) {
    float sample_distribution = scene_eval->eevee.volumetric_sample_distribution;
    sample_distribution = 4.0f * std::max(1.0f - sample_distribution, 1e-2f);

    const float clip_start = inst_.camera.data_get().clip_near;
    /* Negate */
    float near = integration_start = std::min(-integration_start, clip_start - 1e-4f);
    float far = integration_end = std::min(-integration_end, near - 1e-4f);

    data_.depth_param[0] = (far - near * exp2(1.0f / sample_distribution)) / (far - near);
    data_.depth_param[1] = (1.0f - data_.depth_param[0]) / near;
    data_.depth_param[2] = sample_distribution;
  }
  else {
    const float clip_start = inst_.camera.data_get().clip_near;
    const float clip_end = inst_.camera.data_get().clip_far;
    integration_start = std::min(integration_end, clip_start);
    integration_end = std::max(-integration_end, clip_end);

    data_.depth_param[0] = integration_start;
    data_.depth_param[1] = integration_end;
    data_.depth_param[2] = 1.0f / (integration_end - integration_start);
  }

  data_.push_update();

  /* Quick breakdown of the Volumetric rendering:
   *
   * The rendering is separated in 4 stages:
   *
   * - Material Parameters : we collect volume properties of
   *   all participating media in the scene and store them in
   *   a 3D texture aligned with the 3D frustum.
   *   This is done in 2 passes, one that clear the texture
   *   and/or evaluate the world volumes, and the 2nd one that
   *   additively render object volumes.
   *
   * - Light Scattering : the volume properties then are sampled
   *   and light scattering is evaluated for each cell of the
   *   volume texture. Temporal super-sampling (if enabled) occurs here.
   *
   * - Volume Integration : the scattered light and extinction is
   *   integrated (accumulated) along the view-rays. The result is stored
   *   for every cell in another texture.
   *
   * - Full-screen Resolve : From the previous stage, we get two
   *   3D textures that contains integrated scattered light and extinction
   *   for "every" positions in the frustum. We only need to sample
   *   them and blend the scene color with those factors. This also
   *   work for alpha blended materials.
   */

  /* World pass is not additive as it also clear the buffer. */
  world_ps_.init();
  world_ps_.state_set(DRW_STATE_WRITE_COLOR);
  bind_common_buffers(world_ps_);

  objects_ps_.init();
  objects_ps_.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_BLEND_ADD);
  bind_common_buffers(objects_ps_);

  GPUMaterial *mat = nullptr;

  /* World Volumetric */
  ::World *world = scene->world;
  if (world && world->use_nodes && world->nodetree &&
      !LOOK_DEV_STUDIO_LIGHT_ENABLED(draw_ctx->v3d)) {

    mat = inst_.shaders.world_shader_get(world, world->nodetree, MAT_PIPE_VOLUME);

    if (!GPU_material_has_volume_output(mat)) {
      /* TODO (Miguel Pozo): This should never happen ? */
      mat = nullptr;
    }
  }

  PassMain::Sub &ps = world_ps_.sub("World Volume");
  if (mat) {
    ps.shader_set(GPU_material_get_shader(mat));
    if (volume_sub_pass(ps, nullptr, nullptr, mat)) {
      /* TODO (Miguel Pozo):
       * effects->enabled_effects |= (EFFECT_VOLUMETRIC | EFFECT_POST_BUFFER);
       */
      enabled_ = true;
    }
  }
  else {
    /* If no world or volume material is present just clear the buffer with this drawcall */
    ps.shader_set(inst_.shaders.static_shader_get(VOLUME_CLEAR));
  }
  ps.draw_procedural(GPU_PRIM_TRIS, 1, data_.tex_size.z);
}

void Volumes::sync_object(Object *ob, ObjectHandle &ob_handle, ResourceHandle /*res_handle*/)
{
  int mat_nr = VOLUME_MATERIAL_NR;
  bool has_motion = inst_.velocity.step_object_sync(ob, ob_handle.object_key, ob_handle.recalc);
  Material &material = inst_.materials.material_get(ob, has_motion, mat_nr - 1, MAT_GEOM_VOLUME);

  /* TODO (Miguel Pozo): Equivalent ? Default volume is already returned from material_get.
      if (ma == nullptr) {
        if (ob->type == OB_VOLUME) {
          ma = BKE_material_default_volume();
        }
        else {
          return;
        }
      }
  */

  float3x3 world_matrix = float3x3(float4x4(ob->object_to_world));
  float3 size = math::to_scale(world_matrix);
  /* Check if any of the axes have 0 length. (see #69070) */
  const float epsilon = 1e-8f;
  if (size.x < epsilon || size.y < epsilon || size.z < epsilon) {
    return;
  }

  /* If shader failed to compile or is currently compiling. */
  if (material.shading.gpumat == nullptr) {
    return;
  }

  GPUShader *shader = GPU_material_get_shader(material.shading.gpumat);
  if (shader == nullptr) {
    return;
  }

  PassMain::Sub &ps = material.shading.sub_pass->sub(ob->id.name);
  if (volume_sub_pass(ps, inst_.scene, ob, material.shading.gpumat)) {
    /* TODO (Miguel Pozo): Is any equivalent required here?
     * DRW_shgroup_add_material_resources(grp, mat); */

    /* TODO (Miguel Pozo):
     * effects->enabled_effects |= (EFFECT_VOLUMETRIC | EFFECT_POST_BUFFER);
     */
    enabled_ = true;
    ps.draw_procedural(GPU_PRIM_TRIS, 1, data_.tex_size.z);
  }
}

void Volumes::end_sync()
{
  if (!enabled_) {
    prop_scattering_tx_.free();
    prop_extinction_tx_.free();
    prop_emission_tx_.free();
    prop_phase_tx_.free();
    scatter_tx_.free();
    transmit_tx_.free();
    scatter_history_tx_.free();
    transmit_history_tx_.free();
    /* volumetric_fb_ */;
    /* scatter_fb_ */;
    return;
  }

  eGPUTextureUsage usage = GPU_TEXTURE_USAGE_SHADER_READ | GPU_TEXTURE_USAGE_ATTACHMENT;
  /* Volume properties: We evaluate all volumetric objects
   * and store their final properties into each froxel */
  prop_scattering_tx_.ensure_3d(GPU_R11F_G11F_B10F, data_.tex_size, usage);
  prop_extinction_tx_.ensure_3d(GPU_R11F_G11F_B10F, data_.tex_size, usage);
  prop_emission_tx_.ensure_3d(GPU_R11F_G11F_B10F, data_.tex_size, usage);
  prop_phase_tx_.ensure_3d(GPU_RG16F, data_.tex_size, usage);

  /* Volume scattering: We compute for each froxel the
   * Scattered light towards the view. We also resolve temporal
   * super sampling during this stage. */
  scatter_tx_.ensure_3d(GPU_R11F_G11F_B10F, data_.tex_size, usage);
  transmit_tx_.ensure_3d(GPU_R11F_G11F_B10F, data_.tex_size, usage);

  /* Final integration: We compute for each froxel the
   * amount of scattered light and extinction coef at this
   * given depth. We use these textures as double buffer
   * for the volumetric history. */
  scatter_history_tx_.ensure_3d(GPU_R11F_G11F_B10F, data_.tex_size, usage);
  transmit_history_tx_.ensure_3d(GPU_R11F_G11F_B10F, data_.tex_size, usage);

  float4 scatter = float4(0.0f);
  float4 transmit = float4(1.0f);
  /* TODO (Miguel Pozo): DRW_TEX_WRAP ? */
  dummy_scatter_tx_.ensure_3d(GPU_RGBA8, int3(1), GPU_TEXTURE_USAGE_SHADER_READ, scatter);
  dummy_transmit_tx_.ensure_3d(GPU_RGBA8, int3(1), GPU_TEXTURE_USAGE_SHADER_READ, transmit);

#if 0
    /* TODO */
    scatter_tx_ = dummy_scatter_tx_;
    transmit_tx_ = dummy_transmit_tx_;
#endif

  scatter_ps_.init();
  scatter_ps_.state_set(DRW_STATE_WRITE_COLOR);
  scatter_ps_.shader_set(inst_.shaders.static_shader_get(
      data_.use_lights ? VOLUME_SCATTER_WITH_LIGHTS : VOLUME_SCATTER));
  bind_common_buffers(scatter_ps_);

  scatter_ps_.bind_texture("volumeScattering", &prop_scattering_tx_);
  scatter_ps_.bind_texture("volumeExtinction", &prop_extinction_tx_);
  scatter_ps_.bind_texture("volumeEmission", &prop_emission_tx_);
  scatter_ps_.bind_texture("volumePhase", &prop_phase_tx_);
  scatter_ps_.bind_texture("historyScattering", &scatter_history_tx_);
  scatter_ps_.bind_texture("historyTransmittance", &transmit_history_tx_);
#if 0
    /* TODO */
    scatter_ps_.bind_texture("irradianceGrid", &lcache->grid_tx.tex);
    scatter_ps_.bind_texture("shadowCubeTexture", &sldata->shadow_cube_pool);
    scatter_ps_.bind_texture("shadowCascadeTexture", &sldata->shadow_cascade_pool);
#endif
  scatter_ps_.draw_procedural(GPU_PRIM_TRIS, 1, data_.tex_size.z);

  integration_ps_.init();
  integration_ps_.state_set(DRW_STATE_WRITE_COLOR);
  integration_ps_.shader_set(inst_.shaders.static_shader_get(VOLUME_INTEGRATION));
  bind_common_buffers(integration_ps_);
  integration_ps_.bind_texture("volumeScattering", &scatter_tx_);
  integration_ps_.bind_texture("volumeExtinction", &transmit_tx_);
  integration_ps_.bind_image("finalScattering_img", &scatter_history_tx_);
  integration_ps_.bind_image("finalTransmittance_img", &transmit_history_tx_);
  integration_ps_.draw_procedural(GPU_PRIM_TRIS, 1, 1);

  resolve_ps_.init();
  resolve_ps_.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_BLEND_CUSTOM);
  resolve_ps_.shader_set(inst_.shaders.static_shader_get(VOLUME_RESOLVE));
  bind_common_buffers(integration_ps_);
  resolve_ps_.bind_texture("inScattering", &scatter_tx_);
  resolve_ps_.bind_texture("inTransmittance", &transmit_tx_);
#if 0
    /* TODO */
    resolve_ps_.bind_texture("inSceneDepth", &e_data.depth_src);
#endif
  resolve_ps_.draw_procedural(GPU_PRIM_TRIS, 1, data_.tex_size.z);
}

void Volumes::draw_compute(View &view)
{
  if (!enabled_) {
    return;
  }

  DRW_stats_group_start("Volumes");

  /* We sample the shadow-maps using shadow sampler. We need to enable Comparison mode.
   * TODO(fclem): avoid this by using sampler objects. */
#if 0
    /* TODO */
    GPU_texture_compare_mode(sldata->shadow_cube_pool, true);
    GPU_texture_compare_mode(sldata->shadow_cascade_pool, true);
#endif

  volumetric_fb_.ensure(GPU_ATTACHMENT_NONE,
                        GPU_ATTACHMENT_TEXTURE(prop_scattering_tx_),
                        GPU_ATTACHMENT_TEXTURE(prop_extinction_tx_),
                        GPU_ATTACHMENT_TEXTURE(prop_emission_tx_),
                        GPU_ATTACHMENT_TEXTURE(prop_phase_tx_));

  scatter_fb_.ensure(GPU_ATTACHMENT_NONE,
                     GPU_ATTACHMENT_TEXTURE(scatter_tx_),
                     GPU_ATTACHMENT_TEXTURE(transmit_tx_));

  volumetric_fb_.bind();
  inst_.manager->submit(world_ps_, view);
  inst_.manager->submit(objects_ps_, view);

  scatter_fb_.bind();
  inst_.manager->submit(scatter_ps_, view);

  volumetric_fb_.bind();
  inst_.manager->submit(integration_ps_, view);

#if 0
    SWAP(struct GPUFrameBuffer *, fbl->scatter_fb_, fbl->integration_fb_);
    SWAP(GPUTexture *, scatter_tx_, scatter_history_tx_);
    SWAP(GPUTexture *, transmit_tx_, transmit_history_tx_);

    effects->volume_scatter = scatter_tx_;
    effects->volume_transmit = transmit_tx_;
#endif

  DRW_stats_group_end();
}

void Volumes::draw_resolve(View &view, Framebuffer &fb)
{
  if (!enabled_) {
    return;
  }

  GPU_memory_barrier(GPU_BARRIER_TEXTURE_FETCH);

  fb.bind();
  inst_.manager->submit(resolve_ps_, view);
}

/* -------------------------------------------------------------------- */
/** \name Render Passes
 * \{ */

#if 0
  void EEVEE_volumes_output_init(EEVEE_ViewLayerData *sldata, EEVEE_Data *vedata, uint tot_samples)
  {
    EEVEE_FramebufferList *fbl = vedata->fbl;
    EEVEE_TextureList *txl = vedata->txl;
    EEVEE_StorageList *stl = vedata->stl;
    EEVEE_PassList *psl = vedata->psl;
    EEVEE_EffectsInfo *effects = stl->effects;

    /* Create FrameBuffer. */

    /* Should be enough precision for many samples. */
    const eGPUTextureFormat texture_format_accum = (tot_samples > 128) ? GPU_RGBA32F : GPU_RGBA16F;
    DRW_texture_ensure_fullscreen_2d(&scatter_accum_tx_, texture_format_accum, 0);
    DRW_texture_ensure_fullscreen_2d(&transmittance_accum_tx_, texture_format_accum, 0);

    GPU_framebuffer_ensure_config(&fbl->volumetric_accum_fb,
                                  {GPU_ATTACHMENT_NONE,
                                   GPU_ATTACHMENT_TEXTURE(scatter_accum_tx_),
                                   GPU_ATTACHMENT_TEXTURE(transmittance_accum_tx_)});

    /* Create Pass and shgroup. */
    DRW_PASS_CREATE(psl->volumetric_accum_ps, DRW_STATE_WRITE_COLOR | DRW_STATE_BLEND_ADD_FULL);
    DRWShadingGroup *grp = nullptr;
    if ((effects->enabled_effects & EFFECT_VOLUMETRIC) != 0) {
      grp = DRW_shgroup_create(EEVEE_shaders_volumes_resolve_sh_get(true),
                               psl->volumetric_accum_ps);
      DRW_shgroup_uniform_texture_ref(grp, "inScattering", &scatter_tx_);
      DRW_shgroup_uniform_texture_ref(grp, "inTransmittance", &transmit_tx_);
      DRW_shgroup_uniform_texture_ref(grp, "inSceneDepth", &e_data.depth_src);
      DRW_shgroup_uniform_block(grp, "common_block", sldata->common_ubo);
      DRW_shgroup_uniform_block(grp, "renderpass_block", sldata->renderpass_ubo.combined);
    }
    else {
      /* There is no volumetrics in the scene. Use a shader to fill the accum textures with a
       * default value. */
      grp = DRW_shgroup_create(EEVEE_shaders_volumes_accum_sh_get(), psl->volumetric_accum_ps);
    }
    DRW_shgroup_call(grp, DRW_cache_fullscreen_quad_get(), nullptr);
  }

  void EEVEE_volumes_output_accumulate(EEVEE_ViewLayerData *UNUSED(sldata), EEVEE_Data *vedata)
  {
    EEVEE_FramebufferList *fbl = vedata->fbl;
    EEVEE_PassList *psl = vedata->psl;
    EEVEE_EffectsInfo *effects = vedata->stl->effects;

    if (fbl->volumetric_accum_fb != nullptr) {
      /* Accum pass */
      GPU_framebuffer_bind(fbl->volumetric_accum_fb);

      /* Clear texture. */
      if (effects->taa_current_sample == 1) {
        const float clear[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        GPU_framebuffer_clear_color(fbl->volumetric_accum_fb, clear);
      }

      DRW_draw_pass(psl->volumetric_accum_ps);

      /* Restore */
      GPU_framebuffer_bind(fbl->main_fb);
    }
  }
#endif
}  // namespace blender::eevee

/** \} */
