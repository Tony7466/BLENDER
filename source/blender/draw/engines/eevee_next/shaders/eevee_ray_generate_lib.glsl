/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Ray generation routines for each BSDF types.
 */

#pragma BLENDER_REQUIRE(gpu_shader_codegen_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_matrix_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_vector_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_utildefines_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_bxdf_sampling_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_ray_types_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_sampling_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_thickness_lib.glsl)

/* Returns view-space ray. */
BsdfSample ray_generate_direction(vec3 random_point_cos_hemisphere,
                                  ClosureUndetermined cl,
                                  vec3 V,
                                  float thickness)
{
  switch (cl.type) {
    case CLOSURE_BSDF_MICROFACET_GGX_REFRACTION_ID:
      bxdf_ggx_context_amend_transmission(cl, V, thickness);
      break;
  }

  mat3 tangent_to_world = from_up_axis(cl.N);

  BsdfSample samp;
  samp.pdf = 0.0;
  samp.direction = vec3(0.0);
  switch (cl.type) {
    case CLOSURE_BSDF_TRANSLUCENT_ID:
      samp = bxdf_translucent_sample(random_point_cos_hemisphere, thickness);
      break;
    case CLOSURE_BSSRDF_BURLEY_ID:
    case CLOSURE_BSDF_DIFFUSE_ID:
      samp = bxdf_diffuse_sample(random_point_cos_hemisphere);
      break;
    case CLOSURE_BSDF_MICROFACET_GGX_REFLECTION_ID: {
      samp = bxdf_ggx_sample_reflection(random_point_cos_hemisphere,
                                        V * tangent_to_world,
                                        square(to_closure_reflection(cl).roughness));
      break;
    }
    case CLOSURE_BSDF_MICROFACET_GGX_REFRACTION_ID: {
      samp = bxdf_ggx_sample_transmission(random_point_cos_hemisphere,
                                          V * tangent_to_world,
                                          square(to_closure_refraction(cl).roughness),
                                          to_closure_refraction(cl).ior,
                                          thickness);
      break;
    }
  }
  samp.direction = tangent_to_world * samp.direction;

  return samp;
}
