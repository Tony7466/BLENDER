/* SPDX-FileCopyrightText: 2011-2024 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

CCL_NAMESPACE_BEGIN

/* Given a random number, sample a direction that makes an angle of theta with direction D. */
ccl_device float3 phase_sample_direction(float3 D, float cos_theta, float rand)
{
  float sin_theta = sin_from_cos(cos_theta);
  float phi = M_2PI_F * rand;
  float3 dir = make_float3(sin_theta * cosf(phi), sin_theta * sinf(phi), cos_theta);

  float3 T, B;
  make_orthonormals(D, &T, &B);
  dir = dir.x * T + dir.y * B + dir.z * D;

  return dir;
}

/* Given cosine between rays, return probability density that a photon bounces
 * to that direction. The g parameter controls how different it is from the
 * uniform sphere. g=0 uniform diffuse-like, g=1 close to sharp single ray. */
ccl_device float phase_henyey_greenstein(float cos_theta, float g)
{
  if (fabsf(g) < 1e-3f) {
    return M_1_PI_F * 0.25f;
  }
  return ((1.0f - g * g) / safe_powf(1.0f + g * g - 2.0f * g * cos_theta, 1.5f)) *
         (M_1_PI_F * 0.25f);
}

ccl_device float3 phase_henyey_greenstein_sample(float3 D,
                                                 float g,
                                                 float2 rand,
                                                 ccl_private float *pdf)
{
  float cos_theta;

  if (fabsf(g) < 1e-3f) {
    cos_theta = (1.0f - 2.0f * rand.x);
  }
  else {
    float k = (1.0f - g * g) / (1.0f - g + 2.0f * g * rand.x);
    cos_theta = (1.0f + g * g - k * k) / (2.0f * g);
  }

  *pdf = phase_henyey_greenstein(cos_theta, g);

  return phase_sample_direction(D, cos_theta, rand.y);
}

CCL_NAMESPACE_END
