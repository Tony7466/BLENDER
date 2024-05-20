/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Sampling of Normal Distribution Function for various BxDF.
 */

#pragma BLENDER_REQUIRE(eevee_bxdf_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_bxdf_microfacet_lib.glsl)
#pragma BLENDER_REQUIRE(draw_math_geom_lib.glsl)

/* -------------------------------------------------------------------- */
/** \name Uniform Hemisphere
 * \{ */

float sample_pdf_uniform_hemisphere()
{
  return 0.5 * M_1_PI;
}

vec3 sample_uniform_hemisphere(vec3 rand)
{
  float z = rand.x;                      /* cos theta */
  float r = sqrt(max(0.0, 1.0 - z * z)); /* sin theta */
  float x = r * rand.y;
  float y = r * rand.z;
  return vec3(x, y, z);
}

vec3 sample_uniform_hemisphere(vec3 rand, vec3 N, vec3 T, vec3 B, out float pdf)
{
  vec3 tH = sample_uniform_hemisphere(rand);
  pdf = sample_pdf_uniform_hemisphere();
  return mat3(T, B, N) * tH;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Cosine Hemisphere
 * \{ */

float sample_pdf_cosine_hemisphere(float cos_theta)
{
  return cos_theta * M_1_PI;
}

vec3 sample_cosine_hemisphere(vec3 rand)
{
  float z = sqrt(max(1e-16, rand.x));    /* cos theta */
  float r = sqrt(max(0.0, 1.0 - z * z)); /* sin theta */
  float x = r * rand.y;
  float y = r * rand.z;
  return vec3(x, y, z);
}

vec3 sample_cosine_hemisphere(vec3 rand, vec3 N, vec3 T, vec3 B, out float pdf)
{
  vec3 tH = sample_cosine_hemisphere(rand);
  pdf = sample_pdf_cosine_hemisphere(tH.z);
  return mat3(T, B, N) * tH;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Cosine Hemisphere
 * \{ */

float sample_pdf_uniform_sphere()
{
  return 1.0 / (4.0 * M_PI);
}

vec3 sample_uniform_sphere(vec3 rand)
{
  float cos_theta = rand.x * 2.0 - 1.0;
  float sin_theta = safe_sqrt(1.0 - cos_theta * cos_theta);
  return vec3(sin_theta * rand.yz, cos_theta);
}

vec3 sample_uniform_sphere(vec3 rand, vec3 N, vec3 T, vec3 B, out float pdf)
{
  pdf = sample_pdf_uniform_sphere();
  return mat3(T, B, N) * sample_uniform_sphere(rand);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Uniform Cone sampling
 * \{ */

vec3 sample_uniform_cone(vec3 rand, float angle)
{
  float z = mix(cos(angle), 1.0, rand.x); /* cos theta */
  float r = sqrt(max(0.0, 1.0 - z * z));  /* sin theta */
  float x = r * rand.y;
  float y = r * rand.z;
  return vec3(x, y, z);
}

vec3 sample_uniform_cone(vec3 rand, float angle, vec3 N, vec3 T, vec3 B)
{
  vec3 tH = sample_uniform_cone(rand, angle);
  /* TODO: pdf? */
  return mat3(T, B, N) * tH;
}

/** \} */
