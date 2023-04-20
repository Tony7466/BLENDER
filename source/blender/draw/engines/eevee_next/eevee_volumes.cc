/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw_engine
 *
 * Volumetric effects rendering using frostbite approach.
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

#pragma once

#include "DRW_render.h"

#include "BLI_listbase.h"
#include "BLI_rand.h"
#include "BLI_string_utils.h"

#include "DNA_fluid_types.h"
#include "DNA_object_force_types.h"
#include "DNA_volume_types.h"
#include "DNA_world_types.h"

#include "BKE_attribute_math.hh"
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

  data_.coord_scale = float2(viewport_size) / float2(tile_size * tex_size);
  data_.viewport_size_inv = 1.0f / float2(viewport_size);

  /* TODO: compute snap to maxZBuffer for clustered rendering. */
  if (data_.tex_size != tex_size) {
    data_.tex_size = tex_size;
    data_.inv_tex_size = 1.0f / float3(tex_size);
  }

  set_jitter(inst_.sampling.current_sample() - 1);

  if ((scene_eval->eevee.flag & SCE_EEVEE_VOLUMETRIC_SHADOWS) == 0) {
    data_.shadow_steps = 0;
  }
  else {
    data_.shadow_steps = float(scene_eval->eevee.volumetric_shadow_samples);
  }

  data_.use_lights = (scene_eval->eevee.flag & SCE_EEVEE_VOLUMETRIC_LIGHTS) != 0;
  data_.use_soft_shadows = (scene_eval->eevee.flag & SCE_EEVEE_SHADOW_SOFT) != 0;

  data_.light_clamp = scene_eval->eevee.volumetric_light_clamp;
}

void Volumes::begin_sync()
{
  const DRWContextState *draw_ctx = DRW_context_state_get();
  Scene *scene = draw_ctx->scene;
  const Scene *scene_eval = DEG_get_evaluated_scene(draw_ctx->depsgraph);

  /* TODO (Miguel Pozo): Done here since it needs to run after camera sync.
   * Is this the way to go? */

  /* Negate clip values (View matrix forward vector is -Z). */
  const float clip_start = -inst_.camera.data_get().clip_near;
  const float clip_end = -inst_.camera.data_get().clip_far;
  float integration_start = scene_eval->eevee.volumetric_start;
  float integration_end = scene_eval->eevee.volumetric_end;

  if (inst_.camera.is_perspective()) {
    float sample_distribution = scene_eval->eevee.volumetric_sample_distribution;
    sample_distribution = 4.0f * std::max(1.0f - sample_distribution, 1e-2f);

    float near = integration_start = std::min(-integration_start, clip_start - 1e-4f);
    float far = integration_end = std::min(-integration_end, near - 1e-4f);

    data_.depth_param[0] = (far - near * exp2(1.0f / sample_distribution)) / (far - near);
    data_.depth_param[1] = (1.0f - data_.depth_param[0]) / near;
    data_.depth_param[2] = sample_distribution;
  }
  else {
    integration_start = std::min(integration_end, clip_start);
    integration_end = std::max(-integration_end, clip_end);

    data_.depth_param[0] = integration_start;
    data_.depth_param[1] = integration_end;
    data_.depth_param[2] = 1.0f / (integration_end - integration_start);
  }

  data_.push_update();

  /* World Volume Pass */

  world_ps_.init();
  world_ps_.state_set(DRW_STATE_WRITE_COLOR);
  bind_volume_pass_resources(world_ps_);

  GPUMaterial *material = nullptr;

  ::World *world = scene->world;
  if (world && world->use_nodes && world->nodetree &&
      !LOOK_DEV_STUDIO_LIGHT_ENABLED(draw_ctx->v3d)) {

    material = inst_.shaders.world_shader_get(world, world->nodetree, MAT_PIPE_VOLUME);

    if (!GPU_material_has_volume_output(material)) {
      material = nullptr;
    }
  }

  PassSimple::Sub &ps = world_ps_.sub("World Volume");
  if (material) {
    ps.material_set(*inst_.manager, material);
    if (volume_sub_pass(ps, nullptr, nullptr, material)) {
      enabled_ = true;
    }
  }
  else {
    /* If no world or volume material is present just clear the buffer. */
    ps.shader_set(inst_.shaders.static_shader_get(VOLUME_CLEAR));
  }
  ps.dispatch(math::divide_ceil(data_.tex_size, int3(VOLUME_GROUP_SIZE)));
  ps.barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS);
}

void Volumes::sync_object(Object *ob, ObjectHandle & /*ob_handle*/, ResourceHandle res_handle)
{
  ::Material *material = BKE_object_material_get(ob, VOLUME_MATERIAL_NR);
  if (material == nullptr) {
    if (ob->type == OB_VOLUME) {
      material = BKE_material_default_volume();
    }
    else {
      return;
    }
  }

  float3x3 world_matrix = float3x3(float4x4(ob->object_to_world));
  float3 size = math::to_scale(world_matrix);
  /* Check if any of the axes have 0 length. (see #69070) */
  const float epsilon = 1e-8f;
  if (size.x < epsilon || size.y < epsilon || size.z < epsilon) {
    return;
  }

  MaterialPass material_pass = inst_.materials.material_pass_get(
      ob, material, MAT_PIPE_VOLUME, MAT_GEOM_VOLUME_OBJECT);

  /* If shader failed to compile or is currently compiling. */
  if (material_pass.gpumat == nullptr) {
    return;
  }

  GPUShader *shader = GPU_material_get_shader(material_pass.gpumat);
  if (shader == nullptr) {
    return;
  }

  auto to_grid_coords = [&](float3 wP) -> int3 {
    const float4x4 &view_matrix = inst_.camera.data_get().viewmat;
    const float4x4 &perspective_matrix = inst_.camera.data_get().winmat * view_matrix;

    float view_z = (view_matrix * float4(wP, 1.0)).z;

    float volume_z;
    float3 depth_params = data_.depth_param;
    if (inst_.camera.is_orthographic()) {
      volume_z = (view_z - depth_params.x) * depth_params.z;
    }
    else {
      volume_z = depth_params.z * log2(view_z * depth_params.y + depth_params.x);
    }

    float4 grid_coords = perspective_matrix * float4(wP, 1.0);
    grid_coords /= grid_coords.w;
    grid_coords = (grid_coords * 0.5f) + float4(0.5f);

    grid_coords.x *= data_.coord_scale.x;
    grid_coords.y *= data_.coord_scale.y;
    grid_coords.z = volume_z;

    return int3(grid_coords.xyz() * float3(data_.tex_size));
  };

  const BoundBox &bbox = *BKE_object_boundbox_get(ob);
  int3 grid_coords_min = int3(INT32_MAX);
  int3 grid_coords_max = int3(INT32_MIN);

  for (float3 corner : bbox.vec) {
    corner = (float4x4(ob->object_to_world) * float4(corner, 1.0)).xyz();
    int3 grid_coord = to_grid_coords(corner);
    grid_coords_min = math::min(grid_coords_min, grid_coord);
    grid_coords_max = math::max(grid_coords_max, grid_coord);
  }

  bool is_visible = false;
  for (int i : IndexRange(3)) {
    is_visible = is_visible || (grid_coords_min[i] >= 0 && grid_coords_min[i] < data_.tex_size[i]);
    is_visible = is_visible || (grid_coords_max[i] >= 0 && grid_coords_max[i] < data_.tex_size[i]);
  }
  if (!is_visible) {
    return;
  }

  grid_coords_min = math::clamp(grid_coords_min, int3(0), data_.tex_size);
  grid_coords_max = math::clamp(grid_coords_max, int3(0), data_.tex_size);
  int3 grid_size = grid_coords_max - grid_coords_min + int3(1);

  PassMain::Sub &ps = material_pass.sub_pass->sub(ob->id.name);
  if (volume_sub_pass(ps, inst_.scene, ob, material_pass.gpumat)) {
    enabled_ = true;
    ps.push_constant("drw_ResourceID", int(res_handle.resource_index()));
    ps.push_constant("grid_coords_min", grid_coords_min);
    ps.dispatch(math::divide_ceil(grid_size, int3(VOLUME_GROUP_SIZE)));
    ps.barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS);
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
    extinction_tx_.free();
    integrated_scatter_tx_.free();
    integrated_transmit_tx_.free();

    float4 scatter = float4(0.0f);
    float4 transmit = float4(1.0f);
    dummy_scatter_tx_.ensure_3d(GPU_RGBA8, int3(1), GPU_TEXTURE_USAGE_SHADER_READ, scatter);
    dummy_transmit_tx_.ensure_3d(GPU_RGBA8, int3(1), GPU_TEXTURE_USAGE_SHADER_READ, transmit);
    transparent_pass_scatter_tx_ = dummy_scatter_tx_;
    transparent_pass_transmit_tx_ = dummy_transmit_tx_;

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
  extinction_tx_.ensure_3d(GPU_R11F_G11F_B10F, data_.tex_size, usage);

  /* Final integration: We compute for each froxel the
   * amount of scattered light and extinction coef at this
   * given depth. We use these textures as double buffer
   * for the volumetric history. */
  integrated_scatter_tx_.ensure_3d(GPU_R11F_G11F_B10F, data_.tex_size, usage);
  integrated_transmit_tx_.ensure_3d(GPU_R11F_G11F_B10F, data_.tex_size, usage);

  transparent_pass_scatter_tx_ = integrated_scatter_tx_;
  transparent_pass_transmit_tx_ = integrated_transmit_tx_;

  scatter_ps_.init();
  scatter_ps_.shader_set(inst_.shaders.static_shader_get(
      data_.use_lights ? VOLUME_SCATTER_WITH_LIGHTS : VOLUME_SCATTER));
  bind_volume_pass_resources(scatter_ps_);
  scatter_ps_.bind_texture("scattering_tx", &prop_scattering_tx_);
  scatter_ps_.bind_texture("extinction_tx", &prop_extinction_tx_);
  scatter_ps_.bind_texture("emission_tx", &prop_emission_tx_);
  scatter_ps_.bind_texture("phase_tx", &prop_phase_tx_);
  scatter_ps_.bind_image("out_scattering", &scatter_tx_);
  scatter_ps_.bind_image("out_extinction", &extinction_tx_);

  scatter_ps_.barrier(GPU_BARRIER_TEXTURE_FETCH);
  scatter_ps_.dispatch(math::divide_ceil(data_.tex_size, int3(VOLUME_GROUP_SIZE)));
  scatter_ps_.barrier(GPU_BARRIER_TEXTURE_FETCH);

  integration_ps_.init();
  integration_ps_.shader_set(inst_.shaders.static_shader_get(VOLUME_INTEGRATION));
  bind_volume_pass_resources(integration_ps_);
  integration_ps_.bind_texture("scattering_tx", &scatter_tx_);
  integration_ps_.bind_texture("extinction_tx", &extinction_tx_);
  integration_ps_.bind_image("out_scattering", &integrated_scatter_tx_);
  integration_ps_.bind_image("out_transmittance", &integrated_transmit_tx_);

  integration_ps_.dispatch(math::divide_ceil(int2(data_.tex_size), int2(VOLUME_2D_GROUP_SIZE)));
  integration_ps_.barrier(GPU_BARRIER_TEXTURE_FETCH);

  resolve_ps_.init();
  resolve_ps_.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_BLEND_CUSTOM);
  resolve_ps_.shader_set(inst_.shaders.static_shader_get(VOLUME_RESOLVE));
  bind_volume_pass_resources(resolve_ps_);
  resolve_ps_.bind_texture("inScattering", &integrated_scatter_tx_);
  resolve_ps_.bind_texture("inTransmittance", &integrated_transmit_tx_);
  resolve_ps_.bind_texture("inSceneDepth", &inst_.render_buffers.depth_tx);

  resolve_ps_.draw_procedural(GPU_PRIM_TRIS, 1, 3);
}

void Volumes::draw_compute(View &view)
{
  if (!enabled_) {
    return;
  }

  DRW_stats_group_start("Volumes");

  inst_.manager->submit(world_ps_, view);
  inst_.pipelines.volume.render(view);

  inst_.manager->submit(scatter_ps_, view);

  inst_.manager->submit(integration_ps_, view);

  DRW_stats_group_end();
}

void Volumes::draw_resolve(View &view)
{
  if (!enabled_) {
    return;
  }

  resolve_fb_.ensure(GPU_ATTACHMENT_NONE,
                     GPU_ATTACHMENT_TEXTURE(inst_.render_buffers.combined_tx));
  resolve_fb_.bind();
  inst_.manager->submit(resolve_ps_, view);
}

}  // namespace blender::eevee

/** \} */
