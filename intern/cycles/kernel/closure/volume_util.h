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

/* Given cosine between rays, return probability density that a photon bounces to that direction
 * according to the constant Rayleigh phase function.
 * See https://doi.org/10.1364/JOSAA.28.002436 for details. */
ccl_device float phase_rayleigh(float cos_theta)
{
  return (0.1875f * M_1_PI_F) * (1.0f + sqr(cos_theta));
}

ccl_device float3 phase_rayleigh_sample(float3 D, float2 rand, ccl_private float *pdf)
{
  float a = 2.0f * (1.0f - 2.0f * rand.x);
  float b = sqrtf(1.0f + sqr(a));
  float cos_theta = powf(a + b, 1.0f / 3.0f) + powf(a - b, 1.0f / 3.0f);
  *pdf = phase_rayleigh(cos_theta);

  return phase_sample_direction(D, cos_theta, rand.y);
}

/* Given cosine between rays, return probability density that a photon bounces to that direction
 * according to the Draine phase function. This is a generalisation of the Henyey-Greenstein
 * function which bridges the cases of HG and Rayleigh scattering. The parameter g mainly controls
 * the first moment <cos theta>, and alpha the second moment <cos2 theta> of the exact phase
 * function. alpha=0 reduces to HG function, g=0, alpha=1 reduces to Rayleigh function, alpha=1
 * reduces to Cornette-Shanks function.
 * See https://doi.org/10.1086/379118 for details. */
ccl_device float phase_draine(float cos_theta, float g, float alpha)
{
  /* Check special cases. */
  if (fabsf(g) < 1e-3f && alpha > 0.999f) {
    return phase_rayleigh(cos_theta);
  }
  else if (fabsf(alpha) < 1e-3f) {
    return phase_henyey_greenstein(cos_theta, g);
  }

  return ((1.0f - g * g) * (1.0f + alpha * cos_theta * cos_theta)) /
         (4.0f * (1.0f + (alpha * (1.0f + 2.0f * g * g)) / 3.0f) * M_PI_F *
          powf(1.0f + g * g - 2.0f * g * cos_theta, 1.5f));
}

/* Adapted from the hlsl code provided in https://research.nvidia.com/labs/rtr/approximate-mie/ */
ccl_device float phase_draine_sample_cos(float g, float alpha, float rand)
{
  const float g2 = g * g;
  const float g3 = g * g2;
  const float g4 = g2 * g2;
  const float g6 = g2 * g4;
  const float pgp1_2 = (1.0f + g2) * (1.0f + g2);
  const float T1a = -alpha + alpha * g4;
  const float T1a3 = T1a * T1a * T1a;
  const float T2 = -1296.0f * (-1.0f + g2) * (alpha - alpha * g2) * (T1a) *
                   (4.0f * g2 + alpha * pgp1_2);
  const float T3 = 3.0f * g2 * (1.0f + g * (-1.0f + 2.0f * rand)) +
                   alpha * (2.0f + g2 + g3 * (1.0f + 2.0f * g2) * (-1.0f + 2.0f * rand));
  const float T4a = 432.0f * T1a3 + T2 + 432.0f * (alpha - alpha * g2) * T3 * T3;
  const float T4b = -144.0f * alpha * g2 + 288.0f * alpha * g4 - 144.0f * alpha * g6;
  const float T4b3 = T4b * T4b * T4b;
  const float T4 = T4a + sqrtf(-4.0f * T4b3 + T4a * T4a);
  const float T4p3 = powf(T4, 1.0f / 3.0f);
  const float T6 = (2.0f * T1a +
                    (48.0f * powf(2.0f, 1.0f / 3.0f) *
                     (-(alpha * g2) + 2.0f * alpha * g4 - alpha * g6)) /
                        T4p3 +
                    T4p3 / (3.0f * powf(2.0f, 1.0f / 3.0f))) /
                   (alpha - alpha * g2);
  const float T5 = 6.0f * (1.0f + g2) + T6;
  return (1.0f + g2 -
          powf(-0.5f * sqrtf(T5) + sqrtf(6.0f * (1.0f + g2) -
                                         (8.0f * T3) / (alpha * (-1.0f + g2) * sqrtf(T5)) - T6) /
                                       2.0f,
               2.0f)) /
         (2.0f * g);
}

ccl_device float3
phase_draine_sample(float3 D, float g, float alpha, float2 rand, ccl_private float *pdf)
{
  float cos_theta;

  if (fabsf(g) < 1e-3f || fabsf(alpha) < 1e-3f) {
    cos_theta = (1.0f - 2.0f * rand.x);
    *pdf = M_1_PI_F * 0.25f;
  }
  else {
    cos_theta = phase_draine_sample_cos(g, alpha, rand.x); /* TODO: Fold in */
    *pdf = phase_draine(cos_theta, g, alpha);
  }

  return phase_sample_direction(D, cos_theta, rand.y);
}

/* Given cosine between rays, return probability density that a photon bounces to that direction
 * according to the Fournier-Forand phase function. The n parameter is the particle index of
 * refraction and controls how much of the light is refracted. B is the particle backscatter
 * fraction, B = b_b / b.
 * See https://doi.org/10.1117/12.366488 for details. */

ccl_device float phase_fournier_forand_sigma(float n, float sin_theta_2)
{
  float u = 4.0f * sqr(sin_theta_2);
  return u / (3.0f * sqr(n - 1.0f));
}

ccl_device float phase_fournier_forand_cdf(float theta, float n, float B)
{
  if (theta <= 0.0f) {
    return 0.0f;
  }
  else if (theta >= M_PI_F) {
    return 1.0f;
  }
  float s90 = phase_fournier_forand_sigma(n, sinf(M_PI_2_F / 2.0f));
  float s180 = phase_fournier_forand_sigma(n, sinf(M_PI_F / 2.0f));
  float sin_theta_2 = sinf(theta / 2.0f);
  float s_theta = phase_fournier_forand_sigma(n, sin_theta_2);
  float slope = 2.0f * (logf(2.0f * B * (s90 - 1.0f) + 1.0f) / logf(s90)) + 3.0f;
  float v = (3.0f - slope) / 2.0f;
  float pow_stheta_v = powf(s_theta, v);
  float pow_s180_v = powf(s180, v);
  float cdf = 1.0f / ((1.0f - s_theta) * pow_stheta_v);
  cdf *= ((1.0f - powf(s_theta, v + 1.0f)) - (1.0f - pow_stheta_v) * sqr(sin_theta_2));
  cdf += (1.0f / 8.0f) * ((1.0f - pow_s180_v) / ((s180 - 1.0f) * pow_s180_v)) * cosf(theta) *
         sqr(sinf(theta));
  return cdf;
}

ccl_device float interpolate_linear(float ax, float ay, float bx, float by, float x)
{
  return mix(ay, by, inverse_lerp(ax, bx, x));
}

ccl_device float phase_fournier_forand_find_angle(float rand, float B, float n)
{
  const float ANGLE_TOL = 1.0f / 180.0f;
  const float CDF_TOL = 0.01f;

  int it = 0;
  float l_low, l_up, m = 0.0f;
  float theta = 0.8726646259971648f;  // 50 degrees
  float fm = phase_fournier_forand_cdf(theta, n, B);
  float err = fm - rand;
  if (err < 0.0f) {
    l_low = theta;
    l_up = M_PI_F;
  }
  else if (err > 0.0f) {
    l_low = 0.0f;
    l_up = theta;
  }
  else {
    return theta;
  }

  while (it < 100 && (fabsf(l_low - l_up) > ANGLE_TOL || fabsf(err) > CDF_TOL)) {
    m = (l_low + l_up) / 2.0f;
    fm = phase_fournier_forand_cdf(m, n, B);
    err = fm - rand;
    it += 1;

    if (signf(phase_fournier_forand_cdf(l_low, n, B) - rand) == signf(err)) {
      l_low = m;
    }
    else if (signf(phase_fournier_forand_cdf(l_up, n, B) - rand) == signf(err)) {
      l_up = m;
    }
  }
  m = (l_low + l_up) / 2.0f;
  fm = phase_fournier_forand_cdf(m, n, B);
  err = fm - rand;
  if (err < 0.0f) {
    return mix(m, l_up, inverse_lerp(fm, phase_fournier_forand_cdf(l_up, n, B), rand));
  }
  else if (err > 0.0f) {
    return mix(l_low, m, inverse_lerp(phase_fournier_forand_cdf(l_low, n, B), fm, rand));
  }
  return m;
}

ccl_device float phase_fournier_forand(float cos_theta, float B, float IOR)
{
  cos_theta = clamp(cos_theta, -0.99999f, 0.99999f);
  float s90 = phase_fournier_forand_sigma(IOR, sinf(M_PI_2_F / 2.0f));
  float s180 = phase_fournier_forand_sigma(IOR, sinf(M_PI_F / 2.0f));
  float sin_theta_2 = sqrtf(0.5f * (1.0f - cos_theta));
  float s_theta = phase_fournier_forand_sigma(IOR, sin_theta_2);
  float slope = 2.0f * (logf(2.0f * B * (s90 - 1.0f) + 1.0f) / logf(s90)) + 3.0f;
  float v = (3.0f - slope) / 2.0f;
  float pow_stheta_v = powf(s_theta, v);
  float pow_s180_v = powf(s180, v);
  float pf = 1.0f / (4.0f * M_PI_F * sqr(1.0f - s_theta) * pow_stheta_v);
  pf *= (v * (1.0f - s_theta) - (1.0f - pow_stheta_v) +
         (s_theta * (1.0f - pow_stheta_v) - v * (1.0f - s_theta)) * (1.0f / sqr(sin_theta_2)));
  pf += ((1.0f - pow_s180_v) / (16.0f * M_PI_F * (s180 - 1.0f) * pow_s180_v)) *
        (3.0f * sqr(cos_theta) - 1.0f);
  return pf;
}

ccl_device float3
phase_fournier_forand_sample(float3 D, float B, float IOR, float2 rand, ccl_private float *pdf)
{
  float cos_theta = cosf(phase_fournier_forand_find_angle(rand.x, B, IOR));
  *pdf = phase_fournier_forand(cos_theta, IOR, B);

  return phase_sample_direction(D, cos_theta, rand.y);
}

CCL_NAMESPACE_END
