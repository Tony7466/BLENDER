#ifndef RESTIR_H_
#define RESTIR_H_

#include "kernel/integrator/reservoir.h"
#include "kernel/integrator/state.h"

CCL_NAMESPACE_BEGIN

struct SpatialResampling {
  static const int num_neighbors = 8;

  static uint get_render_pixel_index(ccl_global const KernelWorkTile *ccl_restrict tile,
                                     const int x,
                                     const int y)
  {
    return (uint)tile->offset + x + y * tile->stride;
  }

  static uint get_next_neighbor(ccl_global const KernelWorkTile *ccl_restrict tile, const int id)
  {
    /* TODO(weizhen): adjust the numbers based on `num_neighbors`. */
    const int dx = id % 3 - 1;
    const int dy = id / 3 - 1;
    const int x = tile->x + dx;
    const int y = tile->y + dy;

    if (x < tile->min_x || x > tile->max_x || y < tile->min_y || y > tile->max_y) {
      /* Out of bound. */
      return -1;
    }

    return get_render_pixel_index(tile, x, y);
  }

  static uint is_valid_neighbor(const ccl_private ShaderData *sd,
                                const ccl_private ShaderData *neighbor_sd)
  {
    /* TODO(weizhen): find a good criterion. */
    return (sd->object == neighbor_sd->object) && (sd->type == neighbor_sd->type);
  }
};

ccl_device_inline void integrator_restir_unpack_reservoir(KernelGlobals kg,
                                                          ccl_private Reservoir *reservoir,
                                                          const ccl_global float *buffer)
{
  int i = 0;
  /* TODO(weizhen): this works for diffuse surfaces. For specular, probably `sd->wi` is needed
   * instead. */

#ifdef __LIGHT_TREE__
  if (kernel_data.integrator.use_light_tree) {
    reservoir->ls.emitter_id = (int)buffer[i++];
  }
  else
#endif
  {
    reservoir->ls.lamp = (int)buffer[i++];
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
  int i = 4;

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
  sd->wi = make_float3(buffer[i], buffer[i + 1], buffer[i + 2]);

  /* (TODO): do I need to write to this? */
  // sd->closure_transparent_extinction;
}

ccl_device_inline void integrator_restir_unpack_shader(KernelGlobals kg,
                                                       IntegratorState state,
                                                       ccl_private ShaderData *sd,
                                                       ccl_private uint32_t *path_flag,
                                                       ccl_global float *ccl_restrict
                                                           render_buffer)
{
  if (kernel_data.film.pass_restir_reservoir != PASS_UNUSED) {
    ccl_global const float *buffer = film_pass_pixel_render_buffer(kg, state, render_buffer) +
                                     kernel_data.film.pass_restir_reservoir;
    integrator_restir_unpack_shader(sd, path_flag, buffer);
  }
}

ccl_device_inline void integrator_restir_unpack_shader(KernelGlobals kg,
                                                       ccl_private ShaderData *sd,
                                                       ccl_private uint32_t *path_flag,
                                                       const uint64_t render_pixel_index,
                                                       const ccl_global float *ccl_restrict
                                                           render_buffer)
{
  if (kernel_data.film.pass_restir_reservoir != PASS_UNUSED) {
    const uint64_t render_buffer_offset = render_pixel_index * kernel_data.film.pass_stride;
    ccl_global const float *buffer = render_buffer + render_buffer_offset +
                                     kernel_data.film.pass_restir_reservoir;
    integrator_restir_unpack_shader(sd, path_flag, buffer);
  }
}

ccl_device_inline void integrator_restir_unpack_reservoir(KernelGlobals kg,
                                                          ccl_private Reservoir *reservoir,
                                                          ccl_private ShaderData *sd,
                                                          ccl_private uint32_t *path_flag,
                                                          const uint64_t render_pixel_index,
                                                          const ccl_global float *ccl_restrict
                                                              render_buffer)
{
  if (kernel_data.film.pass_restir_reservoir != PASS_UNUSED) {
    const uint64_t render_buffer_offset = render_pixel_index * kernel_data.film.pass_stride;
    ccl_global const float *buffer = render_buffer + render_buffer_offset +
                                     kernel_data.film.pass_restir_reservoir;

    integrator_restir_unpack_reservoir(kg, reservoir, buffer);
    integrator_restir_unpack_shader(sd, path_flag, buffer);
  }
}

/* TODO(weizhen): move these radiance function to somewhere else. */
ccl_device_forceinline void shader_data_setup_from_restir(KernelGlobals kg,
                                                          ConstIntegratorState state,
                                                          ccl_private ShaderData *sd,
                                                          const uint32_t path_flag,
                                                          ccl_global float *ccl_restrict
                                                              render_buffer)
{
  shader_setup_from_restir(kg, sd);

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
  const Spectrum light_eval = light_sample_shader_eval(
      kg, state, emission_sd, ls, sd->time, sd, check_visibility);

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
  PROFILING_INIT(kg, PROFILING_SHADE_RESTIR);

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

  const float rand = path_state_rng_1D(kg, &rng_state, PRNG_SPATIAL_RESAMPLING);

  Reservoir reservoir(rand);

  /* TODO(weizhen): add options for pairwiseMIS and biasedMIS. The current MIS weight is not good
   * for point light with soft falloff and area light with small spread. */
  /* Fetch neighboring reservoirs. */
  for (int i = 0; i < SpatialResampling::num_neighbors + 1; i++) {
    const uint neighbor_pixel_index = SpatialResampling::get_next_neighbor(tile, i);

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

    if (!SpatialResampling::is_valid_neighbor(&sd, &neighbor_sd) || neighbor_reservoir.is_empty())
    {
      continue;
    }

    /* Evaluate neighbor sample from the current shading point. */
    if (!light_sample_from_uv(kg, &sd, path_flag, &neighbor_reservoir.ls)) {
      continue;
    }
    radiance_eval(kg, state, &sd, &neighbor_reservoir.ls, &neighbor_reservoir.radiance);

    reservoir.add_sample(
        neighbor_reservoir.ls, neighbor_reservoir.radiance, neighbor_reservoir.total_weight);
  }

  if (reservoir.is_empty()) {
    return false;
  }

  /* Loop over neighborhood again to determine valid samples. */
  int valid_neighbors = 0;
  for (int i = 0; i < SpatialResampling::num_neighbors + 1; i++) {
    const uint neighbor_pixel_index = SpatialResampling::get_next_neighbor(tile, i);

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

  if (valid_neighbors == 0) {
    return false;
  }

  BsdfEval radiance = reservoir.radiance;
  const float unbiased_contribution_weight = reservoir.total_weight /
                                             reduce_add(fabs(radiance.sum)) / valid_neighbors;
  bsdf_eval_mul(&radiance, unbiased_contribution_weight);

  integrate_direct_light_create_shadow_path<true>(
      kg, state, &rng_state, &sd, &reservoir.ls, &radiance, 0);

  return true;
}

CCL_NAMESPACE_END

#endif  // RESTIR_H_
