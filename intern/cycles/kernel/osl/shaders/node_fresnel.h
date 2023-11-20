/* SPDX-FileCopyrightText: 2009-2010 Sony Pictures Imageworks Inc., et al. All Rights Reserved.
 * SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Adapted code from Open Shading Language. */

float fresnel_dielectric_cos(float cosi, float eta)
{
  /* compute fresnel reflectance without explicitly computing
   * the refracted direction */
  float c = fabs(cosi);
  float g = eta * eta - 1 + c * c;
  float result;

  if (g > 0) {
    g = sqrt(g);
    float A = (g - c) / (g + c);
    float B = (c * (g + c) - 1) / (c * (g - c) + 1);
    result = 0.5 * A * A * (1 + B * B);
  }
  else {
    result = 1.0; /* TIR (no refracted component) */
  }

  return result;
}

color fresnel_conductor(float cosi, color eta, color k)
{
  color cosi2 = color(cosi * cosi);
  color one = color(1, 1, 1);
  color tmp_f = eta * eta + k * k;
  color tmp = tmp_f * cosi2;
  color Rparl2 = (tmp - (2.0 * eta * cosi) + one) / (tmp + (2.0 * eta * cosi) + one);
  color Rperp2 = (tmp_f - (2.0 * eta * cosi) + cosi2) / (tmp_f + (2.0 * eta * cosi) + cosi2);
  return (Rparl2 + Rperp2) * 0.5;
}

vector conductor_ior_from_color(color reflectivity, color edge_tint)
{
  /* "Artist Friendly Metallic Fresnel", Ole Gulbrandsen, 2014
   * https://jcgt.org/published/0003/04/03/paper.pdf */

  vector r = clamp(reflectivity, 0.0, 0.99);
  vector r_sqrt = sqrt(r);
  vector one = 1.0;

  vector n_min = (one - r) / (one + r);
  vector n_max = (one + r_sqrt) / (one - r_sqrt);

  return mix(n_max, n_min, edge_tint);
}

vector conductor_extinction_from_color(color reflectivity, vector n)
{
  /* "Artist Friendly Metallic Fresnel", Ole Gulbrandsen, 2014
   * https://jcgt.org/published/0003/04/03/paper.pdf */

  vector r = clamp(reflectivity, 0.0, 0.99);

  vector np1 = n + 1.0;
  vector nm1 = n - 1.0;
  vector k2 = ((r * np1 * np1) - (nm1 * nm1)) / (1.0 - r);
  k2 = max(k2, 0.0);

  return sqrt(k2);
}

float F0_from_ior(float eta)
{
  float f0 = (eta - 1.0) / (eta + 1.0);
  return f0 * f0;
}

float ior_from_F0(float f0)
{
  float sqrt_f0 = sqrt(clamp(f0, 0.0, 0.99));
  return (1.0 + sqrt_f0) / (1.0 - sqrt_f0);
}
