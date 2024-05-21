/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma BLENDER_REQUIRE(eevee_bxdf_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_vector_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_matrix_lib.glsl)

/* -------------------------------------------------------------------- */
/** \name Diffuse BSDF
 * \{ */

BsdfSample bxdf_diffuse_sample(vec3 rand)
{
  float cos_theta = safe_sqrt(rand.x);
  BsdfSample samp;
  samp.direction = vec3(rand.yz * sin_from_cos(cos_theta), cos_theta);
  samp.pdf = cos_theta * M_1_PI;
  return samp;
}

BsdfEval bxdf_diffuse_eval(vec3 N, vec3 L)
{
  BsdfEval eval;
  eval.throughput = eval.pdf = saturate(dot(N, L));
  return eval;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Translucent BSDF
 * \{ */

BsdfSample bxdf_translucent_sample(vec3 rand, float thickness)
{
  if (thickness > 0.0) {
    /* Two transmission events inside a sphere is a uniform sphere distribution. */
    float cos_theta = rand.x * 2.0 - 1.0;
    BsdfSample samp;
    samp.direction = vec3(rand.yz * sin_from_cos(cos_theta), -cos_theta);
    samp.pdf = 0.25 * M_1_PI;
    return samp;
  }

  /* Inverted cosine distribution. */
  BsdfSample samp = bxdf_diffuse_sample(rand);
  samp.direction = -samp.direction;
  return samp;
}

BsdfEval bxdf_translucent_eval(vec3 N, vec3 L, float thickness)
{
  if (thickness > 0.0) {
    /* Two transmission events inside a sphere is a uniform sphere distribution. */
    BsdfEval eval;
    eval.throughput = eval.pdf = 0.25 * M_1_PI;
    return eval;
  }

  BsdfEval eval;
  eval.throughput = eval.pdf = saturate(dot(-N, L));
  return eval;
}

/** \} */
