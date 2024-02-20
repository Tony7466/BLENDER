/* SPDX-FileCopyrightText: 2009-2010 Sony Pictures Imageworks Inc., et al. All Rights Reserved.
 * SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Adapted code from Open Shading Language. */

#pragma once

CCL_NAMESPACE_BEGIN

typedef struct TransparentBsdf {
  SHADER_CLOSURE_BASE;

  /* For nested dielectrics, we need the ability to skip a surface while still recording
   * its IOR so it can be added to the volume stack. */
  float ior;
} TransparentBsdf;

static_assert(sizeof(ShaderClosure) >= sizeof(TransparentBsdf), "TransparentBsdf is too large!");

ccl_device void bsdf_transparent_setup(ccl_private ShaderData *sd,
                                       const Spectrum weight,
                                       uint32_t path_flag,
                                       float ior)
{
  /* Check cutoff weight. */
  float sample_weight = fabsf(average(weight));
  if (!(sample_weight >= CLOSURE_WEIGHT_CUTOFF)) {
    return;
  }

  if (sd->flag & SD_TRANSPARENT) {
    sd->closure_transparent_extinction += weight;

    /* Add weight to existing transparent BSDF. */
    for (int i = 0; i < sd->num_closure; i++) {
      ccl_private ShaderClosure *sc = &sd->closure[i];

      if (sc->type == CLOSURE_BSDF_TRANSPARENT_ID) {
        ccl_private TransparentBsdf *bsdf = (ccl_private TransparentBsdf *)sc;
        /* If there are multiple skipped refractive closures, average their IORs. */
        bsdf->ior = (bsdf->ior * bsdf->sample_weight + ior * sample_weight) / (bsdf->sample_weight + sample_weight);
        bsdf->weight += weight;
        bsdf->sample_weight += sample_weight;
        break;
      }
    }
  }
  else {
    sd->flag |= SD_BSDF | SD_TRANSPARENT;
    sd->closure_transparent_extinction = weight;

    if (path_flag & PATH_RAY_TERMINATE) {
      /* In this case the number of closures is set to zero to disable
       * all others, but we still want to get transparency so increase
       * the number just for this. */
      sd->num_closure_left = 1;
    }

    /* Create new transparent BSDF. */
    ccl_private TransparentBsdf *bsdf = (ccl_private TransparentBsdf *)closure_alloc(
        sd, sizeof(TransparentBsdf), CLOSURE_BSDF_TRANSPARENT_ID, weight);

    if (bsdf) {
      bsdf->sample_weight = sample_weight;
      bsdf->N = sd->N;
      bsdf->ior = ior;
    }
    else if (path_flag & PATH_RAY_TERMINATE) {
      sd->num_closure_left = 0;
    }
  }
}

ccl_device void bsdf_transparent_setup(ccl_private ShaderData *sd,
                                       const Spectrum weight,
                                       uint32_t path_flag)
{
  bsdf_transparent_setup(sd, weight, path_flag, 1.0f);
}

ccl_device Spectrum bsdf_transparent_eval(ccl_private const ShaderClosure *sc,
                                          const float3 wi,
                                          const float3 wo,
                                          ccl_private float *pdf)
{
  *pdf = 0.0f;
  return zero_spectrum();
}

ccl_device int bsdf_transparent_sample(ccl_private const ShaderClosure *sc,
                                       float3 Ng,
                                       float3 wi,
                                       ccl_private Spectrum *eval,
                                       ccl_private float3 *wo,
                                       ccl_private float *pdf,
                                       ccl_private float *ior)
{
  ccl_private const TransparentBsdf *bsdf = (ccl_private const TransparentBsdf *)sc;
  // only one direction is possible
  *wo = -wi;
  /* Some high number for MIS. */
  *pdf = 1e6f;
  *eval = one_spectrum() * 1e6f;
  *ior = bsdf->ior;
  return LABEL_TRANSMIT | LABEL_TRANSPARENT;
}

CCL_NAMESPACE_END
