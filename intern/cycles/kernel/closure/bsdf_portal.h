/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

CCL_NAMESPACE_BEGIN

typedef struct PortalClosure {
  SHADER_CLOSURE_BASE;

  float3 P;
  float3 D;
} PortalClosure;

static_assert(sizeof(ShaderClosure) >= sizeof(PortalClosure), "PortalClosure is too large!");

ccl_device void bsdf_portal_setup(ccl_private ShaderData *sd,
                                  const Spectrum weight,
                                  uint32_t path_flag,
                                  float3 position,
                                  float3 direction)
{
  /* Check cutoff weight. */
  float sample_weight = fabsf(average(weight));
  if (!(sample_weight >= CLOSURE_WEIGHT_CUTOFF)) {
    return;
  }

  sd->closure_transparent_extinction += weight;
  sd->flag |= SD_BSDF | SD_PORTAL;

  ccl_private PortalClosure *pc = (ccl_private PortalClosure *)closure_alloc(
      sd, sizeof(PortalClosure), CLOSURE_BSDF_PORTAL_ID, weight);

  if (pc) {
    pc->sample_weight = sample_weight;
    pc->N = sd->N;
    pc->P = position;
    pc->D = direction;
  }
}

ccl_device Spectrum bsdf_portal_eval(ccl_private const ShaderClosure *sc,
                                     const float3 wi,
                                     const float3 wo,
                                     ccl_private float *pdf)
{
  *pdf = 0.0f;
  return zero_spectrum();
}

CCL_NAMESPACE_END
