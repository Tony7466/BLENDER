/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

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

  if (path_flag & PATH_RAY_TERMINATE) {
    /* In this case the number of closures is set to zero to disable
     * all others, but we still want to get transparency so increase
     * the number just for this. */
    sd->num_closure_left = 1;
  }

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

CCL_NAMESPACE_END
