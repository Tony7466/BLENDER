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

  if (sd->flag & SD_TRANSPARENT || sd->flag & SD_PORTAL) {
    sd->closure_transparent_extinction += weight;
  }
  else {
    sd->closure_transparent_extinction = weight;
  }
  sd->flag |= SD_BSDF | SD_PORTAL;

  if (path_flag & PATH_RAY_TERMINATE) {
    /* In this case the number of closures is set to zero to disable
     * all others, but we still want to get transparency so increase
     * the number just for this. */
    sd->num_closure_left = 1;
  }

  ccl_private PortalClosure *pc = (ccl_private PortalClosure *)closure_alloc(
      sd, sizeof(PortalClosure), CLOSURE_BSDF_PORTAL_ID, weight);

  if (pc) {
    pc->sample_weight = sample_weight;
    pc->N = sd->N;
    pc->P = position;
    pc->D = direction;
  }
  else if (path_flag & PATH_RAY_TERMINATE) {
    sd->num_closure_left = 0;
  }
}

CCL_NAMESPACE_END
