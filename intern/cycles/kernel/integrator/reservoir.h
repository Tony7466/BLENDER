#ifndef RESERVOIR_H_
#define RESERVOIR_H_

#include "kernel/light/common.h"

CCL_NAMESPACE_BEGIN

struct Reservoir {
  LightSample ls;
  BsdfEval radiance;
  float luminance;
  float total_weight = 0.0f;

  int num_light_samples;
  int num_bsdf_samples = 1;
  int max_samples;
  float3 rgb_to_y;

  int direct_sampling_method;

  Reservoir() = default;

  Reservoir(Reservoir &other)
  {
    ls = other.ls;
    radiance = other.radiance;
    total_weight = other.total_weight;
  }

  Reservoir(KernelGlobals kg)
  {
    rgb_to_y = float4_to_float3(kernel_data.film.rgb_to_y);
  }

  Reservoir(KernelGlobals kg, const bool use_ris)
  {
    /* TODO(weizhen): maybe find optimal values automatically. */
    num_light_samples = use_ris ? kernel_data.integrator.restir_light_samples : 1;
    max_samples = num_light_samples + num_bsdf_samples;
    rgb_to_y = float4_to_float3(kernel_data.film.rgb_to_y);
    direct_sampling_method = kernel_data.integrator.direct_light_sampling_type;
  }

  bool is_empty() const
  {
    return total_weight == 0.0f;
  }

  float compute_luminance(const ccl_private BsdfEval &radiance) const
  {
    return dot(spectrum_to_rgb(radiance.sum), rgb_to_y);
  }

  bool finalize()
  {
    if (is_empty()) {
      return false;
    }
    /* Apply unbiased contribution weight. */
    total_weight /= luminance;
    return true;
  }

  void add_sample(const ccl_private LightSample &ls,
                  const ccl_private BsdfEval &radiance,
                  const float mis_weight,
                  const float rand)
  {
    const float luminance = compute_luminance(radiance);
    const float weight = luminance * mis_weight;

    if (!(weight > 0.0f)) {
      /* Should be theoretically captured by the following condition, but we can not trust floating
       * point precision. */
      return;
    }

    total_weight += weight;

    if (rand * total_weight <= weight) {
      this->ls = ls;
      this->radiance = radiance;
      this->luminance = luminance;
    }
  }

  float power_heuristic(int num_a, float pdf_a, int num_b, float pdf_b)
  {
    return (pdf_a * pdf_a) / (pdf_a * pdf_a * (float)num_a + pdf_b * pdf_b * (float)num_b);
  }

  float mis_weight_nee(float ls_pdf, float bsdf_pdf)
  {
#ifdef WITH_CYCLES_DEBUG
    if (direct_sampling_method == DIRECT_LIGHT_SAMPLING_FORWARD) {
      ls_pdf *= (bsdf_pdf == 0.0f);
    }
    else if (direct_sampling_method == DIRECT_LIGHT_SAMPLING_NEE) {
      bsdf_pdf = 0.0f;
    }
#endif
    return power_heuristic(num_light_samples, ls_pdf, num_bsdf_samples, bsdf_pdf);
  }

  float mis_weight_forward(float bsdf_pdf, float ls_pdf)
  {
#ifdef WITH_CYCLES_DEBUG
    if (direct_sampling_method == DIRECT_LIGHT_SAMPLING_FORWARD) {
      ls_pdf = 0.0f;
    }
    else if (direct_sampling_method == DIRECT_LIGHT_SAMPLING_NEE) {
      bsdf_pdf *= (ls_pdf == 0.0f);
    }
#endif
    return power_heuristic(num_bsdf_samples, bsdf_pdf, num_light_samples, ls_pdf);
  }

  void add_light_sample(const ccl_private LightSample &ls,
                        const ccl_private BsdfEval &radiance,
                        const float bsdf_pdf,
                        const float rand)
  {
    const float mis_weight = mis_weight_nee(ls.pdf, bsdf_pdf);

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
    const float mis_weight = mis_weight_forward(bsdf_pdf, ls.pdf);

    /* TODO(weizhen): Convert pdf to area measure when returning the pdf instead of here. */
    const float jacobian = ls.jacobian_solid_angle_to_area();
    float bsdf_pdf_in_area = bsdf_pdf;
    if (jacobian > 0.0f) {
      bsdf_pdf_in_area *= jacobian;
    }

    add_sample(ls, radiance, mis_weight / bsdf_pdf_in_area, rand);
  }

  void add_reservoir(KernelGlobals kg, Reservoir &other, const float rand);
};

/* TODO(weizhen): maybe it should be determined when the sample is drawn, not afterwards. */
ccl_device_inline bool sample_copy_direction(KernelGlobals kg,
                                             ccl_private const Reservoir &reservoir)
{
  if (reservoir.is_empty()) {
    return false;
  }

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

void Reservoir::add_reservoir(KernelGlobals kg, Reservoir &other, const float rand)
{
  if (other.is_empty()) {
    return;
  }

  add_sample(other.ls, other.radiance, other.total_weight, rand);
}

struct SpatialReservoir {
  Reservoir reservoir;
  /* TODO(weizhen): maybe the neighbors should share the same flag. */
  uint32_t path_flag;
  ShaderData sd;
  uint3 pixel_index;

  SpatialReservoir() = default;

  SpatialReservoir(KernelGlobals kg) : reservoir(kg)
  {
    pixel_index = make_uint3(-1, -1, -1);
  }

  SpatialReservoir(SpatialReservoir &other)
  {
    reservoir = other.reservoir;
    path_flag = other.path_flag;
    sd = other.sd;
    pixel_index = other.pixel_index;
  }

  bool is_empty() const
  {
    return reservoir.is_empty();
  }

  void add_reservoir(KernelGlobals kg, SpatialReservoir &other, const float rand)
  {
    reservoir.add_reservoir(kg, other.reservoir, rand);
  }
};

ccl_device_inline bool restir_unpack_reservoir(KernelGlobals kg,
                                               ccl_private Reservoir *reservoir,
                                               const ccl_global float *buffer)
{
  PROFILING_INIT(kg, PROFILING_RESTIR_RESERVOIR_PASSES);
  int i = 0;
  /* TODO(weizhen): this works for diffuse surfaces. For specular, probably `sd->wi` is needed
   * instead. */
  /* TODO(weizhen): compress the data. */
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

  reservoir->luminance = buffer[i];

  return !reservoir->is_empty();
}

struct GlobalReservoir {
  Spectrum radiance;
  float total_weight = 0.0f;
  uint32_t path_flag;

  float3 rgb_to_y;

  GlobalReservoir() = default;

  GlobalReservoir(KernelGlobals kg)
  {
    rgb_to_y = float4_to_float3(kernel_data.film.rgb_to_y);
  }

  float compute_luminance(const ccl_private Spectrum &radiance) const
  {
    return dot(spectrum_to_rgb(radiance), rgb_to_y);
  }

  bool add_sample(const ccl_private Spectrum &radiance, const float rand)
  {
    const float weight = compute_luminance(radiance);

    if (!(weight > 0.0f)) {
      /* Should be theoretically captured by the following condition, but we can not trust floating
       * point precision. */
      return false;
    }

    total_weight += weight;

    if (rand * total_weight > weight) {
      return false;
    }

    this->radiance = radiance;

    return true;
  }

  bool is_empty() const
  {
    return total_weight == 0.0f;
  }

  bool finalize()
  {
    if (is_empty()) {
      return false;
    }
    /* Apply unbiased contribution weight. */
    total_weight /= compute_luminance(radiance);
    radiance *= total_weight;
    return true;
  }
};

ccl_device_inline bool restir_unpack_reservoir_pt(KernelGlobals kg,
                                                  ccl_private GlobalReservoir *reservoir,
                                                  ccl_global float *buffer)
{
  PROFILING_INIT(kg, PROFILING_RESTIR_RESERVOIR_PASSES);

  reservoir->total_weight = kernel_read_pass_float(buffer);
  reservoir->path_flag = (uint32_t)kernel_read_pass_float(buffer + 1);
  reservoir->radiance = kernel_read_pass_float3(buffer + 2);

  return !reservoir->is_empty();
}

CCL_NAMESPACE_END

#endif  // RESERVOIR_H_
