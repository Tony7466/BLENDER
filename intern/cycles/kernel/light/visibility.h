/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2023 Blender Foundation */

#pragma once

CCL_NAMESPACE_BEGIN

// float EGB to RGBE conversion as described by Greg Ward
// http://www.graphics.cornell.edu/online/formats/rgbe/

ccl_device_inline SpectrumRGBE SpectrumToSpectrumRGBE(const Spectrum &spec)
{
  SpectrumRGBE specRGBE;

  float v;
  int e;
  v = max(spec.x, max(spec.y, spec.z));
  if (v < 1e-32f) {
    specRGBE.r = specRGBE.g = specRGBE.b = specRGBE.e = 0;
  }
  else {
    v = frexp(v, &e) * 256.0f / v;
    specRGBE.r = (uint8_t)(spec.x * v);
    specRGBE.g = (uint8_t)(spec.y * v);
    specRGBE.b = (uint8_t)(spec.z * v);
    specRGBE.e = (uint8_t)(e + 128);
  }
  return specRGBE;
}

ccl_device_inline Spectrum SpectrumRGBEToSpectrum(const SpectrumRGBE &specRGBE)
{
  Spectrum spec;

  if (specRGBE.e) {
    const float f = ldexp(1.0f, specRGBE.e - (int)(128 + 8));
    spec.x = specRGBE.r * f;
    spec.y = specRGBE.g * f;
    spec.z = specRGBE.b * f;
  }
  else {
    spec.x = spec.y = spec.z = 0.0f;
  }
  return spec;
}

ccl_device_inline BsdfEvalRGBE BsdfEvalToBsdfEvalRGBE(const BsdfEval &bsdfEval)
{
  BsdfEvalRGBE bsdfEvalRGBE;
  bsdfEvalRGBE.diffuse = SpectrumToSpectrumRGBE(bsdfEval.diffuse);
  bsdfEvalRGBE.glossy = SpectrumToSpectrumRGBE(bsdfEval.glossy);
  bsdfEvalRGBE.sum = SpectrumToSpectrumRGBE(bsdfEval.sum);
  return bsdfEvalRGBE;
}

ccl_device_inline BsdfEval BsdfEvalRGBEToBsdfEval(const BsdfEvalRGBE &bsdfEvalRGBE)
{
  BsdfEval bsdfEval;
  bsdfEval.diffuse = SpectrumRGBEToSpectrum(bsdfEvalRGBE.diffuse);
  bsdfEval.glossy = SpectrumRGBEToSpectrum(bsdfEvalRGBE.glossy);
  bsdfEval.sum = SpectrumRGBEToSpectrum(bsdfEvalRGBE.sum);
  return bsdfEval;
}

ccl_device_inline void light_visibility_correction(KernelGlobals kg,
                                                   IntegratorState state,
                                                   const int shader,
                                                   Spectrum *light_visibility)
{
  Spectrum visible_components = zero_spectrum();
  const BsdfEval scatter_eval = BsdfEvalRGBEToBsdfEval(
      INTEGRATOR_STATE(state, path, scatter_eval));
  Spectrum transmission = scatter_eval.sum - (scatter_eval.glossy + scatter_eval.diffuse);
  if (!(shader & SHADER_EXCLUDE_DIFFUSE)) {
    visible_components += scatter_eval.diffuse;
  }

  if (!(shader & SHADER_EXCLUDE_GLOSSY)) {
    visible_components += scatter_eval.glossy;
  }

  if (!(shader & SHADER_EXCLUDE_TRANSMIT)) {
    visible_components += transmission;
  }

  *light_visibility = safe_divide(visible_components, scatter_eval.sum);
}

CCL_NAMESPACE_END
