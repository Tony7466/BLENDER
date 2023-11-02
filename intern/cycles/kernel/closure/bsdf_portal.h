/* SPDX-FileCopyrightText: 2009-2010 Sony Pictures Imageworks Inc., et al. All Rights Reserved.
 * SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Adapted code from Open Shading Language. */

#pragma once

CCL_NAMESPACE_BEGIN

ccl_device void bsdf_portal_setup(ccl_private ShaderData *sd,
                                  const Spectrum weight,
                                  uint32_t path_flag)
{
  /* Check cutoff weight. */
  float sample_weight = fabsf(average(weight));
  if (!(sample_weight >= CLOSURE_WEIGHT_CUTOFF)) {
    return;
  }

  sd->flag |= SD_BSDF | SD_PORTAL;
  if (sd->flag & SD_TRANSPARENT) {
    sd->closure_transparent_extinction += weight;
  }
  else {
    sd->closure_transparent_extinction = weight;
  }
  // sd->closure_transparent_extinction = one_spectrum();
  // sd->flag |= SD_TRANSPARENT;

  // /* This is kept from the transparent closure, not sure if we want the same for portals. */
  // if (path_flag & PATH_RAY_TERMINATE) {
  //   /* In this case the number of closures is set to zero to disable
  //    * all others, but we still want to get transparency so increase
  //    * the number just for this. */
  //   sd->num_closure_left = 1;
  // }

  /* Create new portal BSDF. */
  ccl_private ShaderClosure *bsdf = closure_alloc(
      sd, sizeof(ShaderClosure), CLOSURE_BSDF_PORTAL_ID, weight);

  if (bsdf) {
    bsdf->sample_weight = sample_weight;
    bsdf->N = sd->N;
  }
  else if (path_flag & PATH_RAY_TERMINATE) {
    sd->num_closure_left = 0;
  }
}

ccl_device int bsdf_portal_sample(ccl_private const ShaderClosure *sc,
                                  float3 Ng,
                                  float3 wi,
                                  ccl_private Spectrum *eval,
                                  ccl_private float3 *wo,
                                  ccl_private float *pdf)
{
  // only one direction is possible
  *wo = -wi;
  *pdf = 1;
  *eval = one_spectrum();
  return LABEL_TRANSMIT | LABEL_PORTAL;
}

CCL_NAMESPACE_END
