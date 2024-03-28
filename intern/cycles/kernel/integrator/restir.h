#ifndef RESTIR_H_
#define RESTIR_H_

CCL_NAMESPACE_BEGIN

#include "kernel/light/common.h"
#include "kernel/types.h"

typedef struct Reservoir {
  LightSample ls;
  BsdfEval radiance;

  float total_weight;
  float rand;
  int num_light_samples;
  int num_bsdf_samples;

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

ccl_device void integrator_restir(KernelGlobals kg,
                                  IntegratorShadowState state,
                                  ccl_global float *ccl_restrict render_buffer)
{
  PROFILING_INIT(kg, PROFILING_SHADE_RESTIR);
  film_write_direct_light(kg, state, render_buffer);
  integrator_shadow_path_terminate(kg, state, DEVICE_KERNEL_INTEGRATOR_RESTIR);
  return;
}

CCL_NAMESPACE_END

#endif  // RESTIR_H_
