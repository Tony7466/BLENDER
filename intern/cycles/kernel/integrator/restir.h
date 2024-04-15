#ifndef RESTIR_H_
#define RESTIR_H_

#include "kernel/integrator/reservoir.h"
#include "kernel/integrator/state.h"

CCL_NAMESPACE_BEGIN

ccl_device_inline void integrator_restir_unpack_reservoir(ccl_private Reservoir *reservoir,
                                                          const ccl_global float *buffer)
{
  int i = 0;
  /* TODO(weizhen): this works for diffuse surfaces. For specular, probably `sd->wi` is needed
   * instead. */
  reservoir->ls.emitter_id = (int)buffer[i++];

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

    integrator_restir_unpack_reservoir(reservoir, buffer);
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

/* Evaluate BSDF * L * cos_NO. */
ccl_device_forceinline void radiance_eval(KernelGlobals kg,
                                          IntegratorState state,
                                          ccl_private ShaderData *sd,
                                          ccl_private LightSample *ls,
                                          ccl_private BsdfEval *radiance)
{
  ShaderDataCausticsStorage emission_sd_storage;
  ccl_private ShaderData *emission_sd = AS_SHADER_DATA(&emission_sd_storage);

  const Spectrum light_eval = light_sample_shader_eval(kg, state, emission_sd, ls, sd->time);

  /* TODO(weizhen): path guiding needs state. */
  surface_shader_bsdf_eval(kg, state, sd, ls->D, radiance, ls->shader);

  bsdf_eval_mul(radiance, light_eval);
}

ccl_device void integrator_restir(KernelGlobals kg,
                                  IntegratorState state,
                                  ccl_global float *ccl_restrict render_buffer)
{
  PROFILING_INIT(kg, PROFILING_SHADE_RESTIR);

  /* TODO(weizhen): read neighbor weights, pick a neighbor, then retrieve sd. */
  Reservoir reservoir;
  uint32_t path_flag;
  ShaderData sd;
  const uint64_t render_pixel_index = INTEGRATOR_STATE(state, path, render_pixel_index);
  integrator_restir_unpack_reservoir(
      kg, &reservoir, &sd, &path_flag, render_pixel_index, render_buffer);
  if (reservoir.is_empty()) {
    return;
  }

  shader_data_setup_from_restir(kg, state, &sd, path_flag, render_buffer);

  /* Evaluate sample from the current shading point. */
  light_sample_from_uv(kg, &sd, path_flag, &reservoir.ls);
  radiance_eval(kg, state, &sd, &reservoir.ls, &reservoir.radiance);

  BsdfEval radiance = reservoir.radiance;
  bsdf_eval_mul(&radiance, reservoir.total_weight);

  /* Load random number state. */
  RNGState rng_state;
  path_state_rng_load(state, &rng_state);

  integrate_direct_light_create_shadow_path<true>(
      kg, state, &rng_state, &sd, &reservoir.ls, &radiance, 0);
}

CCL_NAMESPACE_END

#endif  // RESTIR_H_
