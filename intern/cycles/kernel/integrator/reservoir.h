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
    /* TODO(weizhen): replace `reduce_add()` with luminance. */
    const float weight = reduce_add(fabs(radiance.sum)) * mis_weight;

    if (!(weight > 0.0f)) {
      /* Should be theoretically captured by the following condition, but we can not trust floating
       * point precision. */
      return;
    }

    total_weight += weight;

    if (rand * total_weight <= weight) {
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

    /* TODO(weizhen): Convert pdf to area measure when returning the pdf instead of here. */
    const float jacobian = ls.jacobian_solid_angle_to_area();
    float ls_pdf_in_area = ls.pdf;
    if (jacobian > 0.0f) {
      ls_pdf_in_area *= jacobian;
    }

    add_sample(ls, radiance, mis_weight / ls_pdf_in_area, rand);
  }

  void add_bsdf_sample(const ccl_private LightSample &ls,
                       const ccl_private BsdfEval &radiance,
                       const float bsdf_pdf,
                       const float rand)
  {
    const float mis_weight = power_heuristic(
        num_bsdf_samples, bsdf_pdf, num_light_samples, ls.pdf);

    /* TODO(weizhen): Convert pdf to area measure when returning the pdf instead of here. */
    const float jacobian = ls.jacobian_solid_angle_to_area();
    float bsdf_pdf_in_area = bsdf_pdf;
    if (jacobian > 0.0f) {
      bsdf_pdf_in_area *= jacobian;
    }

    add_sample(ls, radiance, mis_weight / bsdf_pdf_in_area, rand);
  }

  void add_reservoir(const Reservoir &other, const float rand);
};

void Reservoir::add_reservoir(const Reservoir &other, const float rand)
{
  add_sample(other.ls, other.radiance, other.total_weight, rand);
}

/* TODO(weizhen): maybe it should be determined when the sample is drawn, not afterwards. */
ccl_device_inline bool sample_copy_direction(KernelGlobals kg,
                                             ccl_private const Reservoir &reservoir)
{
  kernel_assert(!reservoir.is_empty());

  if (reservoir.ls.type == LIGHT_AREA) {
    const ccl_global KernelLight *klight = &kernel_data_fetch(lights, reservoir.ls.lamp);
    if (klight->area.tan_half_spread == 0.0f) {
      return true;
    }
  }

  if (reservoir.ls.type == LIGHT_POINT) {
    const ccl_global KernelLight *klight = &kernel_data_fetch(lights, reservoir.ls.lamp);
    if (!klight->spot.is_sphere) {
      return true;
    }
  }

  return false;
}

CCL_NAMESPACE_END

#endif  // RESERVOIR_H_
