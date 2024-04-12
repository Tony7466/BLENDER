#ifndef RESTIR_H_
#define RESTIR_H_

#include "kernel/integrator/state.h"
#include "kernel/light/sample.h"

CCL_NAMESPACE_BEGIN

typedef struct Reservoir {
  LightSample ls;
  BsdfEval radiance;

  float total_weight;
  float rand;
  int num_light_samples;
  int num_bsdf_samples;

  Reservoir() {}

  Reservoir(const bool use_ris, const float rand)
  {
    total_weight = 0.0f;
    this->rand = rand;
    /* TODO(weizhen): find optimal values automatically, or set manually when debugging. */
    this->num_light_samples = use_ris ? 8 : 1;
    this->num_bsdf_samples = use_ris ? 3 : 1;
  }

  bool is_empty()
  {
    return total_weight == 0.0f;
  }

 private:
  void add_sample(const ccl_private LightSample &ls,
                  const ccl_private BsdfEval &radiance,
                  const float mis_weight)
  {
    const float weight = reduce_add(fabs(radiance.sum)) * mis_weight;
    if (!(weight > 0.0f)) {
      return;
    }

    total_weight += weight;
    const float thresh = weight / total_weight;

    if (rand < thresh || weight == total_weight) {
      this->ls = ls;
      this->radiance = radiance;
      rand = rand / thresh;
    }
    else {
      rand = (rand - thresh) / (1.0f - thresh);
    }

    /* Ensure the `rand` is always within 0..1 range, which could be violated above when
     * `-ffast-math` is used. */
    rand = saturatef(rand);
  }

 public:
  float power_heuristic(int num_a, float pdf_a, int num_b, float pdf_b)
  {
    return (pdf_a * pdf_a) / (pdf_a * pdf_a * (float)num_a + pdf_b * pdf_b * (float)num_b);
  }

  void add_light_sample(const ccl_private LightSample &ls,
                        const ccl_private BsdfEval &radiance,
                        const float bsdf_pdf)
  {
    const float mis_weight = power_heuristic(
        num_light_samples, ls.pdf, num_bsdf_samples, bsdf_pdf);
    add_sample(ls, radiance, mis_weight);
  }

  void add_bsdf_sample(const ccl_private LightSample &ls,
                       const ccl_private BsdfEval &radiance,
                       const float bsdf_pdf)
  {
    const float mis_weight = power_heuristic(
        num_bsdf_samples, bsdf_pdf, num_light_samples, ls.pdf);
    add_sample(ls, radiance, mis_weight);
  }
} Reservoir;

ccl_device_inline void integrator_restir_unpack_reservoir(KernelGlobals kg,
                                                          IntegratorShadowState state,
                                                          ccl_private Reservoir *reservoir,
                                                          ccl_private uint32_t *path_flag,
                                                          ccl_private ShaderData *sd,
                                                          ccl_global float *ccl_restrict
                                                              render_buffer)
{

  if (kernel_data.film.pass_restir_reservoir != PASS_UNUSED) {
    ccl_global const float *buffer = film_pass_pixel_render_buffer_shadow(
                                         kg, state, render_buffer) +
                                     kernel_data.film.pass_restir_reservoir;

    int i = 0;
    /* TODO(weizhen): this works for diffuse surfaces. For specular, probably `sd->wi` is needed
     * instead. */
    reservoir->ls.emitter_id = (int)buffer[i++];

    reservoir->ls.u = buffer[i++];
    reservoir->ls.v = buffer[i++];

    reservoir->total_weight = buffer[i++];

    *path_flag = (uint32_t)buffer[i++];

    /* TODO(weizhen): `ShaderData` requires quite some storage for now, but probably everything can
     * be computed from camera and primary ray, if we retrieve the random number. We can also add a
     * separate pass for it. */
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

/* TODO(weizhen): state or shadow_state? */
ccl_device void integrator_restir(KernelGlobals kg,
                                  IntegratorShadowState state,
                                  ccl_global float *ccl_restrict render_buffer)
{
  PROFILING_INIT(kg, PROFILING_SHADE_RESTIR);

  /* TODO(weizhen): read neighbor weights, pick a neighbor, then retrieve sd. */

  /* write everything in render buffer, setup shader, eval shader, etc, following
   * integrate_surface. */
  Reservoir reservoir;
  uint32_t path_flag;
  ShaderData sd;
  integrator_restir_unpack_reservoir(kg, state, &reservoir, &path_flag, &sd, render_buffer);
  shader_setup_from_reservoir(kg, &sd);

  /* TODO(weizhen): what features are needed here? Is this the right state? */
  surface_shader_eval<KERNEL_FEATURE_NODE_MASK_SURFACE_SHADOW>(
      kg, state, &sd, render_buffer, path_flag);
  /* TODO(weizhen): do we call `surface_shader_prepare_closures()` here? */

  light_sample_from_uv(kg, sd.time, sd.P, sd.N, sd.flag, path_flag, &reservoir.ls);

  ShaderDataCausticsStorage emission_sd_storage;
  ccl_private ShaderData *emission_sd = AS_SHADER_DATA(&emission_sd_storage);

  const Spectrum light_eval = light_sample_shader_eval(
      kg, state, emission_sd, &reservoir.ls, sd.time);
  /* TODO(weizhen): path guiding needs state. */
  surface_shader_bsdf_eval(
      kg, INTEGRATOR_STATE_NULL, &sd, reservoir.ls.D, &reservoir.radiance, reservoir.ls.shader);

  bsdf_eval_mul(&reservoir.radiance, light_eval);
  bsdf_eval_mul(&reservoir.radiance, reservoir.total_weight / reduce_add(reservoir.radiance.sum));

  /* TODO(weizhen): move shadow ray tracing after this. */
  if (INTEGRATOR_STATE(state, shadow_path, bounce) == 0) {
    INTEGRATOR_STATE_WRITE(state, shadow_path, throughput) = reservoir.radiance.sum;
  }

  film_write_direct_light(kg, state, render_buffer);
  integrator_shadow_path_terminate(kg, state, DEVICE_KERNEL_INTEGRATOR_RESTIR);
  return;
}

CCL_NAMESPACE_END

#endif  // RESTIR_H_
