#ifndef RESTIR_H_
#define RESTIR_H_

#include "kernel/integrator/reservoir.h"
#include "kernel/integrator/state.h"

CCL_NAMESPACE_BEGIN

ccl_device_inline void integrator_restir_unpack_reservoir(KernelGlobals kg,
                                                          IntegratorState state,
                                                          ccl_private Reservoir *reservoir,
                                                          ccl_private uint32_t *path_flag,
                                                          ccl_private ShaderData *sd,
                                                          ccl_global float *ccl_restrict
                                                              render_buffer)
{
  if (kernel_data.film.pass_restir_reservoir != PASS_UNUSED) {
    ccl_global const float *buffer = film_pass_pixel_render_buffer(kg, state, render_buffer) +
                                     kernel_data.film.pass_restir_reservoir;

    int i = 0;
    /* TODO(weizhen): this works for diffuse surfaces. For specular, probably `sd->wi` is needed
     * instead. */
    reservoir->ls.emitter_id = (int)buffer[i++];

    reservoir->ls.u = buffer[i++];
    reservoir->ls.v = buffer[i++];

    reservoir->total_weight = buffer[i++];

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
}

/* TODO(weizhen): move these radiance function to somewhere else. */
ccl_device_forceinline void radiance_eval_setup_from_reservoir(KernelGlobals kg,
                                                               ConstIntegratorState state,
                                                               ccl_private ShaderData *sd,
                                                               ccl_private LightSample *ls,
                                                               const uint32_t path_flag,
                                                               ccl_global float *ccl_restrict
                                                                   render_buffer)
{
  shader_setup_from_reservoir(kg, sd);

  /* TODO(weizhen): what features are needed here? Is this the right state? */
  surface_shader_eval<KERNEL_FEATURE_NODE_MASK_SURFACE_SHADOW>(
      kg, state, sd, render_buffer, path_flag);
  /* TODO(weizhen): do we call `surface_shader_prepare_closures()` here? */

  light_sample_from_uv(kg, sd->time, sd->P, sd->N, sd->flag, path_flag, ls);
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

/* TODO(weizhen): state or shadow_state? */
ccl_device void integrator_restir(KernelGlobals kg,
                                  IntegratorState state,
                                  ccl_global float *ccl_restrict render_buffer)
{
  PROFILING_INIT(kg, PROFILING_SHADE_RESTIR);

  /* TODO(weizhen): read neighbor weights, pick a neighbor, then retrieve sd. */
  Reservoir reservoir;
  uint32_t path_flag;
  ShaderData sd;
  integrator_restir_unpack_reservoir(kg, state, &reservoir, &path_flag, &sd, render_buffer);

  if (!reservoir.is_empty()) {
    radiance_eval_setup_from_reservoir(kg, state, &sd, &reservoir.ls, path_flag, render_buffer);
    radiance_eval(kg, state, &sd, &reservoir.ls, &reservoir.radiance);

    bsdf_eval_mul(&reservoir.radiance, reservoir.total_weight);

    /* Load random number state. */
    RNGState rng_state;
    path_state_rng_load(state, &rng_state);

    integrate_direct_light_create_shadow_path<true>(
        kg, state, &rng_state, &sd, &reservoir.ls, &reservoir.radiance, 0);
  }
}

CCL_NAMESPACE_END

#endif  // RESTIR_H_
