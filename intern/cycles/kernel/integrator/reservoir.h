#ifndef RESERVOIR_H_
#define RESERVOIR_H_

#include "kernel/light/sample.h"

CCL_NAMESPACE_BEGIN

struct Reservoir {
  LightSample ls;
  BsdfEval radiance;
  float total_weight;

  int num_light_samples;
  int num_bsdf_samples;
  int max_samples;

  Reservoir()
  {
    total_weight = 0.0f;
  }

  Reservoir(KernelGlobals kg, const bool use_ris)
  {
    total_weight = 0.0f;
    /* TODO(weizhen): maybe find optimal values automatically. */
    num_light_samples = use_ris ? kernel_data.integrator.restir_light_samples : 1;
    num_bsdf_samples = use_ris ? kernel_data.integrator.restir_bsdf_samples : 1;
    max_samples = num_light_samples + num_bsdf_samples;
  }

  bool is_empty() const
  {
    return total_weight == 0.0f;
  }

  void add_sample(const ccl_private LightSample &ls,
                  const ccl_private BsdfEval &radiance,
                  const float mis_weight,
                  const float rand)
  {
    const float weight = reduce_add(fabs(radiance.sum)) * mis_weight;
    if (!(weight > 0.0f)) {
      return;
    }

    total_weight += weight;
    const float thresh = weight / total_weight;

    if (rand <= thresh) {
      this->ls = ls;
      this->radiance = radiance;
    }
  }

  float power_heuristic(int num_a, float pdf_a, int num_b, float pdf_b)
  {
    return (pdf_a * pdf_a) / (pdf_a * pdf_a * (float)num_a + pdf_b * pdf_b * (float)num_b);
  }

  void add_light_sample(const ccl_private LightSample &ls,
                        const ccl_private BsdfEval &radiance,
                        const float bsdf_pdf,
                        const float rand)
  {
    const float mis_weight = power_heuristic(
        num_light_samples, ls.pdf, num_bsdf_samples, bsdf_pdf);
    add_sample(ls, radiance, mis_weight / ls.pdf, rand);
  }

  void add_bsdf_sample(const ccl_private LightSample &ls,
                       const ccl_private BsdfEval &radiance,
                       const float bsdf_pdf,
                       const float rand)
  {
    const float mis_weight = power_heuristic(
        num_bsdf_samples, bsdf_pdf, num_light_samples, ls.pdf);
    add_sample(ls, radiance, mis_weight / bsdf_pdf, rand);
  }
};

CCL_NAMESPACE_END

#endif  // RESERVOIR_H_
