#ifndef RESTIR_H_
#define RESTIR_H_

#include "kernel/integrator/reservoir.h"
#include "kernel/integrator/state.h"
#include "kernel/util/profiling.h"

CCL_NAMESPACE_BEGIN

struct SpatialResampling {
  ccl_global const KernelWorkTile *tile;
  int radius;

  SpatialResampling(ccl_global const KernelWorkTile *ccl_restrict tile, const int radius)
  {
    this->tile = tile;
    this->radius = radius;
  }

  uint get_render_pixel_index(const int x, const int y)
  {
    return (uint)tile->offset + x + y * tile->stride;
  }

  /* Get neighbors in a square. */
  uint get_next_neighbor(const float2 rand)
  {
    const float2 offset = floor(rand * radius + 0.5f);

    const int x = tile->x + (int)offset.x;
    const int y = tile->y + (int)offset.y;

    if (x < tile->min_x || x > tile->max_x || y < tile->min_y || y > tile->max_y) {
      /* TODO(weizhen): this is wasting samples, but can we do better? For example overscan? */
      /* Out of bound. */
      return -1;
    }

    return get_render_pixel_index(x, y);
  }

  static bool is_valid_neighbor(const ccl_private ShaderData *sd,
                                const ccl_private ShaderData *neighbor_sd)
  {
    /* TODO(weizhen): find a good criterion, for example the distance to the normal plane of sd. */
    return (sd->object == neighbor_sd->object) && (sd->type == neighbor_sd->type);
  }
};

ccl_device_inline void integrator_restir_unpack_reservoir(KernelGlobals kg,
                                                          ccl_private Reservoir *reservoir,
                                                          const ccl_global float *buffer)
{
  PROFILING_INIT(kg, PROFILING_RESTIR_RESERVOIR_PASSES);
  int i = 0;
  /* TODO(weizhen): this works for diffuse surfaces. For specular, probably `sd->wi` is needed
   * instead. */
  reservoir->ls.object = (int)buffer[i++];
  if (reservoir->ls.object == OBJECT_NONE) {
    /* Analytic light. */
    reservoir->ls.lamp = (int)buffer[i++];
  }
  else {
    /* Mesh light. */
    reservoir->ls.prim = (int)buffer[i++];
  }

  reservoir->ls.u = buffer[i++];
  reservoir->ls.v = buffer[i++];

  reservoir->total_weight = buffer[i++];
}

/* TODO(weizhen): better to pack in another pass. */
ccl_device_inline void integrator_restir_unpack_shader(ccl_private ShaderData *sd,
                                                       ccl_private uint32_t *path_flag,
                                                       const ccl_global float *buffer)
{
  int i = 0;

  *path_flag = (uint32_t)buffer[i++];

  /* TODO(weizhen): `ShaderData` requires quite some storage for now, but we can use
   `ray_length`, `lcg_state`, `time`, and `wi` from state. We can also compress the data, and add
   a separate pass for it. */
  sd->u = buffer[i++];
  sd->v = buffer[i++];
  sd->ray_length = buffer[i++];
  sd->type = (uint)buffer[i++];
  sd->object = (int)buffer[i++];
  sd->prim = (int)buffer[i++];
  /* TODO(weizhen): only Huang Hair BSDF needs LCG now. Can we find a way to remove the usage? */
  sd->lcg_state = (uint)buffer[i++];
  sd->time = buffer[i++];
  sd->dP = buffer[i++];
  sd->dI = buffer[i++];
  sd->wi = make_float3(buffer[i], buffer[i + 1], buffer[i + 2]);

  /* (TODO): do I need to write to this? */
  // sd->closure_transparent_extinction;
}

ccl_device_inline void integrator_restir_unpack_shader(KernelGlobals kg,
                                                       ccl_private ShaderData *sd,
                                                       ccl_private uint32_t *path_flag,
                                                       const uint32_t render_pixel_index,
                                                       const ccl_global float *ccl_restrict
                                                           render_buffer)
{
  PROFILING_INIT(kg, PROFILING_RESTIR_RESERVOIR_PASSES);
  if (kernel_data.film.pass_surface_data != PASS_UNUSED) {
    const uint64_t render_buffer_offset = render_pixel_index * kernel_data.film.pass_stride;
    ccl_global const float *buffer = render_buffer + render_buffer_offset +
                                     kernel_data.film.pass_surface_data;
    integrator_restir_unpack_shader(sd, path_flag, buffer);
  }
}

ccl_device_inline void integrator_restir_unpack_shader(KernelGlobals kg,
                                                       IntegratorState state,
                                                       ccl_private ShaderData *sd,
                                                       ccl_private uint32_t *path_flag,
                                                       ccl_global float *ccl_restrict
                                                           render_buffer)
{
  const uint32_t render_pixel_index = INTEGRATOR_STATE(state, path, render_pixel_index);
  integrator_restir_unpack_shader(kg, sd, path_flag, render_pixel_index, render_buffer);
}

ccl_device_inline void integrator_restir_unpack_reservoir(KernelGlobals kg,
                                                          ccl_private Reservoir *reservoir,
                                                          ccl_private ShaderData *sd,
                                                          ccl_private uint32_t *path_flag,
                                                          const uint32_t render_pixel_index,
                                                          const ccl_global float *ccl_restrict
                                                              render_buffer)
{
  PROFILING_INIT(kg, PROFILING_RESTIR_RESERVOIR_PASSES);
  if (kernel_data.film.pass_restir_reservoir != PASS_UNUSED) {
    const uint64_t render_buffer_offset = render_pixel_index * kernel_data.film.pass_stride;
    ccl_global const float *buffer = render_buffer + render_buffer_offset +
                                     kernel_data.film.pass_restir_reservoir;
    integrator_restir_unpack_reservoir(kg, reservoir, buffer);
  }
  integrator_restir_unpack_shader(kg, sd, path_flag, render_pixel_index, render_buffer);
}

/* TODO(weizhen): move these radiance function to somewhere else. */
ccl_device_forceinline void shader_data_setup_from_restir(KernelGlobals kg,
                                                          ConstIntegratorState state,
                                                          ccl_private ShaderData *sd,
                                                          const uint32_t path_flag,
                                                          ccl_global float *ccl_restrict
                                                              render_buffer)
{
  PROFILING_INIT(kg, PROFILING_RESTIR_SURFACE_DATA_SETUP);
  shader_setup_from_restir(kg, sd);

  PROFILING_EVENT(PROFILING_RESTIR_SHADER_SETUP);
  /* TODO(weizhen): what features are needed here? Is this the right state? */
  surface_shader_eval<KERNEL_FEATURE_NODE_MASK_SURFACE_SHADOW>(
      kg, state, sd, render_buffer, path_flag);
  /* TODO(weizhen): do we call `surface_shader_prepare_closures()` here? */
}

/* Evaluate BSDF * L * cos_NO, accounting for visibility. */
ccl_device_forceinline void radiance_eval(KernelGlobals kg,
                                          IntegratorState state,
                                          ccl_private ShaderData *sd,
                                          ccl_private LightSample *ls,
                                          ccl_private BsdfEval *radiance)
{
  ShaderDataCausticsStorage emission_sd_storage;
  ccl_private ShaderData *emission_sd = AS_SHADER_DATA(&emission_sd_storage);

  /* TODO(weizhen): where should we check visibility? */
  const bool check_visibility = kernel_data.integrator.restir_spatial_visibility;
  PROFILING_INIT(kg, PROFILING_RESTIR_LIGHT_EVAL);
  const Spectrum light_eval = light_sample_shader_eval(
      kg, state, emission_sd, ls, sd->time, sd, check_visibility);
  PROFILING_EVENT(PROFILING_RESTIR_BSDF_EVAL);
  surface_shader_bsdf_eval(kg, state, sd, ls->D, radiance, ls->shader);

  bsdf_eval_mul(radiance, light_eval);
}

ccl_device bool integrator_restir(KernelGlobals kg,
                                  IntegratorState state,
                                  ccl_global const KernelWorkTile *ccl_restrict tile,
                                  ccl_global float *ccl_restrict render_buffer,
                                  const int x,
                                  const int y,
                                  const int scheduled_sample)
{
  PROFILING_INIT(kg, PROFILING_RESTIR_SPATIAL_RESAMPLING);
  uint32_t path_flag;
  ShaderData sd;
  integrator_restir_unpack_shader(kg, state, &sd, &path_flag, render_buffer);

  if (!(sd.type)) {
    /* No interesction at the current shading point. */
    /* TODO(weizhen): revisit this condition when we support background and distant lights. */
    return false;
  }

  shader_data_setup_from_restir(kg, state, &sd, path_flag, render_buffer);

  /* Load random number state. */
  RNGState rng_state;
  path_state_rng_load(state, &rng_state);

  /* Plus one to account for the current pixel. */
  const int samples = kernel_data.integrator.restir_spatial_samples + 1;

  const int radius = kernel_data.integrator.restir_spatial_radius;
  SpatialResampling spatial_resampling(tile, radius);
  Reservoir reservoir;

  /* TODO(weizhen): add options for pairwiseMIS and biasedMIS. The current MIS weight is not good
   * for point light with soft falloff and area light with small spread. */
  /* Uniformly sample neighboring reservoirs within a radius. There is probability to pick the same
   * reservoir twice, but the chance should be low if the radius is big enough and low descrepancy
   * samples are used. */
  for (int i = 0; i < samples; i++) {
    PROFILING_EVENT(PROFILING_RESTIR_SPATIAL_RESAMPLING);
    const float3 rand = path_branched_rng_3D(kg, &rng_state, i, samples, PRNG_SPATIAL_RESAMPLING);
    const float2 rand_neighbor = float3_to_float2(rand) * 2.0f - 1.0f;
    const float rand_pick = rand.z;

    /* Always put the current sample in the reservoir. */
    const uint neighbor_pixel_index = (i == 0) ?
                                          INTEGRATOR_STATE(state, path, render_pixel_index) :
                                          spatial_resampling.get_next_neighbor(rand_neighbor);

    if (neighbor_pixel_index == -1) {
      continue;
    }

    Reservoir neighbor_reservoir;
    uint32_t neighbor_path_flag;
    ShaderData neighbor_sd;
    integrator_restir_unpack_reservoir(kg,
                                       &neighbor_reservoir,
                                       &neighbor_sd,
                                       &neighbor_path_flag,
                                       neighbor_pixel_index,
                                       render_buffer);

    if (!spatial_resampling.is_valid_neighbor(&sd, &neighbor_sd) || neighbor_reservoir.is_empty())
    {
      continue;
    }

    /* Evaluate neighbor sample from the current shading point. */
    if (!light_sample_from_uv(kg, &sd, path_flag, &neighbor_reservoir.ls)) {
      continue;
    }
    radiance_eval(kg, state, &sd, &neighbor_reservoir.ls, &neighbor_reservoir.radiance);
    PROFILING_EVENT(PROFILING_RESTIR_RESERVOIR);
    if (sample_copy_direction(kg, neighbor_reservoir)) {
      /* Jacobian for non-identity shift. */
      neighbor_reservoir.total_weight *= neighbor_reservoir.ls.jacobian_area_to_solid_angle();
    }
    reservoir.add_reservoir(neighbor_reservoir, rand_pick);
  }

  PROFILING_EVENT(PROFILING_RESTIR_SPATIAL_RESAMPLING);
  if (reservoir.is_empty()) {
    return false;
  }

  /* Loop over neighborhood again to determine valid samples. Skip the current pixel, because if
   * the reservoir is not empty it must be valid. */
  int valid_neighbors = 1;
  for (int i = 1; i < samples; i++) {
    const float2 rand_neighbor =
        path_branched_rng_2D(kg, &rng_state, i, samples, PRNG_SPATIAL_RESAMPLING) * 2.0f - 1.0f;

    const uint neighbor_pixel_index = spatial_resampling.get_next_neighbor(rand_neighbor);

    if (neighbor_pixel_index == -1) {
      continue;
    }

    Reservoir neighbor_reservoir;
    uint32_t neighbor_path_flag;
    ShaderData neighbor_sd;
    integrator_restir_unpack_reservoir(kg,
                                       &neighbor_reservoir,
                                       &neighbor_sd,
                                       &neighbor_path_flag,
                                       neighbor_pixel_index,
                                       render_buffer);

    if (!SpatialResampling::is_valid_neighbor(&sd, &neighbor_sd)) {
      continue;
    }

    shader_data_setup_from_restir(kg, state, &neighbor_sd, neighbor_path_flag, render_buffer);

    /* Evaluate picked sample from neighboring shading points. */
    LightSample picked_ls = reservoir.ls;
    if (!light_sample_from_uv(kg, &neighbor_sd, neighbor_path_flag, &picked_ls)) {
      continue;
    }
    radiance_eval(kg, state, &neighbor_sd, &picked_ls, &neighbor_reservoir.radiance);

    if (!bsdf_eval_is_zero(&neighbor_reservoir.radiance)) {
      valid_neighbors++;
    }
  }

  BsdfEval radiance = reservoir.radiance;
  const float unbiased_contribution_weight = reservoir.total_weight *
                                             reservoir.ls.jacobian_solid_angle_to_area() /
                                             reduce_add(fabs(radiance.sum)) / valid_neighbors;
  bsdf_eval_mul(&radiance, unbiased_contribution_weight);

  integrate_direct_light_create_shadow_path<true>(
      kg, state, &rng_state, &sd, &reservoir.ls, &radiance, 0);

  return true;
}

CCL_NAMESPACE_END

#endif  // RESTIR_H_
