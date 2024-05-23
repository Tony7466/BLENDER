/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * BxDF evaluation functions.
 */

#pragma BLENDER_REQUIRE(gpu_shader_utildefines_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_base_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_fast_lib.glsl)

struct BsdfSample {
  vec3 direction;
  float pdf;
};

struct BsdfEval {
  float throughput;
  float pdf;
};

/* Represent an approximation of a bunch of rays from a BSDF. */
struct LightProbeRay {
  /* Average direction of sampled rays or its approximation.
   * Magnitude will reduce directionnality of spherical harmonic evaluation. */
  vec3 dominant_direction;
  /* Perceptual roughness in [0..1] range.
   * Modulate blur level of spherical probe and blend between sphere probe and spherical harmonic
   * evaluation at higher roughness. */
  float perceptual_roughness;
};

/* -------------------------------------------------------------------- */
/** \name Lambert
 *
 * Not really a microfacet model but fits this file.
 * \{ */

float bsdf_lambert(vec3 N, vec3 L, out float pdf)
{
  float cos_theta = saturate(dot(N, L));
  pdf = cos_theta;
  return cos_theta;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Utils
 * \{ */

/* Fresnel monochromatic, perfect mirror */
float F_eta(float eta, float cos_theta)
{
  /* Compute fresnel reflectance without explicitly computing
   * the refracted direction. */
  float c = abs(cos_theta);
  float g = eta * eta - 1.0 + c * c;
  if (g > 0.0) {
    g = sqrt(g);
    float A = (g - c) / (g + c);
    float B = (c * (g + c) - 1.0) / (c * (g - c) + 1.0);
    return 0.5 * A * A * (1.0 + B * B);
  }
  /* Total internal reflections. */
  return 1.0;
}

/* Return the equivalent reflective roughness resulting in a similar lobe. */
float refraction_roughness_remapping(float roughness, float ior)
{
  /* This is a very rough mapping used by manually curve fitting the apparent roughness
   * (blurriness) of GGX reflections and GGX refraction.
   * A better fit is desirable if it is in the same order of complexity.  */
  if (ior > 1.0) {
    return roughness * sqrt_fast(1.0 - 1.0 / ior);
  }
  else {
    return roughness * sqrt_fast(saturate(1.0 - ior)) * 0.8;
  }
}

/**
 * `roughness` is expected to be the linear (from UI) roughness from.
 */
vec3 reflection_dominant_dir(vec3 N, vec3 V, float roughness)
{
  /* From Frostbite PBR Course
   * http://www.frostbite.com/wp-content/uploads/2014/11/course_notes_moving_frostbite_to_pbr.pdf
   * Listing 22.
   * Note that the reference labels squared roughness (GGX input) as roughness. */
  float m = square(roughness);
  vec3 R = -reflect(V, N);
  float smoothness = 1.0 - m;
  float fac = smoothness * (sqrt(smoothness) + m);
  return normalize(mix(N, R, fac));
}

/**
 * `roughness` is expected to be the reflection roughness from `refraction_roughness_remapping`.
 */
vec3 refraction_dominant_dir(vec3 N, vec3 V, float ior, float roughness)
{
  /* Reusing same thing as reflection_dominant_dir for now with the roughness mapped to
   * reflection roughness. */
  float m = square(roughness);
  vec3 R = refract(-V, N, 1.0 / ior);
  float smoothness = 1.0 - m;
  float fac = smoothness * (sqrt(smoothness) + m);
  return normalize(mix(-N, R, fac));
}

/** \} */
