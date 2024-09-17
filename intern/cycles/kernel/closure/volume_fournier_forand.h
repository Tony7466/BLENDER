/* SPDX-FileCopyrightText: 2011-2024 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "kernel/closure/volume_util.h"

CCL_NAMESPACE_BEGIN

/* FOURNIER-FORAND CLOSURE */

typedef struct FournierForandVolume {
  SHADER_CLOSURE_VOLUME_BASE;

  float B;
  float IOR;
} FournierForandVolume;
static_assert(sizeof(ShaderVolumeClosure) >= sizeof(FournierForandVolume),
              "FournierForandVolume is too large!");

ccl_device int volume_fournier_forand_setup(ccl_private FournierForandVolume *volume)
{
  volume->type = CLOSURE_VOLUME_FOURNIER_FORAND_ID;
  /* clamp backscatter fraction to avoid delta function */
  volume->B = min(fabsf(volume->B), 0.5f - 1e-3f);

  return SD_SCATTER;
}

ccl_device Spectrum volume_fournier_forand_eval(ccl_private const ShaderData *sd,
                                                ccl_private const ShaderVolumeClosure *svc,
                                                float3 wo,
                                                ccl_private float *pdf)
{
  ccl_private const FournierForandVolume *volume = (ccl_private const FournierForandVolume *)svc;

  /* note that wi points towards the viewer */
  float cos_theta = dot(-sd->wi, wo);
  *pdf = phase_fournier_forand(cos_theta, volume->B, volume->IOR);

  return make_spectrum(*pdf);
}

ccl_device int volume_fournier_forand_sample(ccl_private const ShaderData *sd,
                                             ccl_private const ShaderVolumeClosure *svc,
                                             float2 rand,
                                             ccl_private Spectrum *eval,
                                             ccl_private float3 *wo,
                                             ccl_private float *pdf)
{
  ccl_private const FournierForandVolume *volume = (ccl_private const FournierForandVolume *)svc;

  /* note that wi points towards the viewer and so is used negated */
  *wo = phase_fournier_forand_sample(-sd->wi, volume->B, volume->IOR, rand, pdf);
  *eval = make_spectrum(*pdf); /* perfect importance sampling */

  return LABEL_VOLUME_SCATTER;
}

CCL_NAMESPACE_END
